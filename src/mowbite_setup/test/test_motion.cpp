#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"
#include "mowbite_setup/program.h"

using namespace mowbite_setup;

namespace {

const char* const kList = R"({
  "catalog": 1,
  "steps": [
    {"id": "stop", "stage": "inputs", "check": "input", "params": {"role": "stop"}},
    {"id": "lift", "stage": "inputs", "check": "input", "params": {"role": "lift"}},
    {"id": "bumper", "stage": "inputs", "check": "input", "params": {"role": "bumper"}},
    {"id": "wheels", "stage": "wheels", "check": "wheels", "confirm": ["raised", "hand_at_stop"]},
    {"id": "stop_moving", "stage": "wheels", "check": "stop_moving", "confirm": ["raised", "hand_at_stop"],
     "parts": [{"id": "stop", "role": "stop"}, {"id": "bumper", "role": "bumper_left"}]},
    {"id": "no_command", "stage": "wheels", "check": "no_command", "confirm": ["raised", "hand_at_stop"]},
    {"id": "silence", "stage": "wheels", "check": "silence", "confirm": ["raised", "hand_at_stop"],
     "params": {"rounds": 2, "emergency": ["HIGH_LEVEL"]}},
    {"id": "driver_end", "stage": "wheels", "check": "driver_end", "confirm": ["raised", "hand_at_stop"],
     "params": {"emergency": ["HIGH_LEVEL"]}}
  ]})";

// The list with the step on the ground after those jacked up.
std::string GroundList() {
  json list = json::parse(kList);
  list["steps"].push_back({{"id", "ground"}, {"stage", "ground"}, {"check", "ground"}, {"confirm", {"area_clear"}}});
  return list.dump();
}

// The list with the step blade, which restarts the OS to allow the blade.
std::string BladeList() {
  json list = json::parse(kList);
  list["restart"] = "self";
  list["steps"].push_back({{"id", "blade"},
                           {"stage", "blade"},
                           {"check", "blade"},
                           {"params", {{"node", "tron_mcu"}, {"rpm", 2500}, {"emergency", {"HIGH_LEVEL"}}}},
                           {"confirm", {"blade_removed", "hand_at_stop"}}});
  return list.dump();
}

const json kBladeConfirm = json::array({"blade_removed", "hand_at_stop"});

const char* const kInputsPassed =
    R"({"schema": 1, "platform": "tron", "steps": {"stop": {"state": "ok"}, "lift": {"state": "ok"},
        "bumper": {"state": "ok"}}})";

constexpr double kWall = 1700000000.0;
constexpr double kTick = 0.02;
constexpr double kTrack = 0.4;
const json kConfirm = json::array({"raised", "hand_at_stop"});

uint32_t Bit(const char* name) {
  return EmergencyBit(name);
}

// The program with a mower around it, jacked up: the logic takes SETUP while setup/motion runs, its commands reach the
// board after a zero once an emergency was released, the board turns the wheels as asked and stops them without
// commands (watchdog) or without its host (hold_without_host after the last one), hw/silence and hw/end as a driver
// does them.
struct Mower {
  explicit Mower(bool inputs_passed = true, const char* list = kList)
      : program(*ParseSteps(json::parse(list), nullptr), "tron") {
    if (inputs_passed) {
      EXPECT_TRUE(program.Load(kInputsPassed, kWall, nullptr));
    }
    capabilities.silence_max_s = 1.0;
    capabilities.driver_end = true;
    program.OnCapabilities(capabilities);
    program.SetTrack(kTrack);
    program.SetDriverRestarts(3);
    hardware.latch_count = 1;
    hardware.telemetry_ok = true;
    World idle;
    idle.state = "IDLE";
    program.OnWorld(idle);
    Run(0.5);
    program.TakeEvents();
  }

  void Latch(uint32_t bits) {
    hardware.emergency_reason |= bits | Bit("LATCH");
    ++hardware.latch_count;
  }

  void Step() {
    now += kTick;
    const auto motion = program.MotionNow();
    if (auto ended = program.TakeMotionEnd()) motion_ends.push_back(*ended);
    const std::string state = motion && logic_takes && (world == "SETUP" || hardware.emergency_reason == 0) ? "SETUP"
                                                                                                             : "IDLE";
    if (state != world) {
      world = state;
      World next;
      next.state = world;
      program.OnWorld(next);
    }
    const bool host = now >= silent_until && now >= gone_until;
    if (hardware.emergency_reason != 0) armed = false;
    TurnBlade(motion, host);
    const auto command = program.CommandNow();
    if (command && motion) {
      EXPECT_LE(std::abs(command->linear) + std::abs(command->angular) * kTrack / 2, motion->max_linear + 1e-9);
      EXPECT_LE(std::abs(command->angular), motion->max_angular + 1e-9);
    }
    if (command && host && world == "SETUP" && hardware.emergency_reason == 0) {
      const bool zero = command->linear == 0 && command->angular == 0;
      if (armed || zero) {
        armed = true;
        setpoint = *command;
        setpoint_at = now;
      }
    }
    const double kept = host ? (watchdog ? watchdog_s : 1e9) : hold_without_host;
    double left = 0, right = 0;
    if (hardware.emergency_reason == 0 && setpoint_at >= 0 && now - setpoint_at <= kept) {
      left = setpoint.linear - setpoint.angular * kTrack / 2;
      right = setpoint.linear + setpoint.angular * kTrack / 2;
      if (swapped) std::swap(left, right);
    }
    if (silent_until >= 0 && now >= silent_until) {
      silent_until = -1;
      Latch(Bit("HIGH_LEVEL"));
    }
    if (gone_until >= 0 && now >= gone_until) {
      gone_until = -1;
      if (comes_back) {
        // A new driver: latched from its first frame, and until it hears its board an input missing as well.
        hardware.emergency_reason = Bit("HIGH_LEVEL") | Bit("LATCH") | Bit("TIMEOUT_INPUTS");
        hardware.latch_count = 1;
        hardware.telemetry_ok = false;
        hearing_at = now + 0.8;
      } else {
        gone_until = 1e9;
      }
    }
    if (hearing_at >= 0 && now >= hearing_at) {
      hearing_at = -1;
      hardware.telemetry_ok = true;
      hardware.emergency_reason &= ~Bit("TIMEOUT_INPUTS");
    }
    if (gone_until < 0) {
      program.OnHardware(hardware, now);
      program.OnWheels(left, right, now);
    }
    if (on_ground) {
      // The mower moves as its wheels turn; the IMU and the antenna (at base_link) see it.
      const double turn = (right - left) / kTrack;
      yaw += turn * kTick;
      x += (left + right) / 2 * std::cos(yaw) * kTick;
      y += (left + right) / 2 * std::sin(yaw) * kTick;
      program.OnImu({0, 0, 9.81, turn * imu_scale}, now);
      if (now - fix_at >= 0.2) {
        fix_at = now;
        program.OnFix(solution, 0.02, x * gnss_scale, y * gnss_scale, now);
      }
    }
    program.Tick(now, kWall + now);
    if (const auto release = program.TakeRelease()) {
      releases++;
      if (release->latch_count == hardware.latch_count && !hardware.stop && !hardware.bumper_left) {
        hardware.emergency_reason = 0;
      }
    }
    if (const auto seconds = program.TakeSilence()) {
      silences++;
      silence_blade = program.SilenceWithBlade();
      program.OnSilenceAnswer(silence_code.empty(), silence_code, now, kWall + now);
      if (silence_code.empty()) silent_until = now + *seconds;
    }
    if (program.TakeEnd()) {
      program.OnEndAnswer(true, "", now, kWall + now);
      gone_until = now + 3.0;
    }
    if (keep && program.running() && now - kept_at >= 0.5) {
      kept_at = now;
      json result;
      program.Keep({{"step", running_step}}, now, &result);
    }
    for (const Event& event : program.TakeEvents()) {
      json details = event.details;
      details["id"] = "x";
      details["t"] = kWall;
      details["type"] = event.type;
      const auto problems = mowbite_api::CheckEvent(event.type, mowbite_api::Json(details));
      EXPECT_TRUE(problems.empty()) << event.type << ": " << (problems.empty() ? "" : problems[0]);
      if (event.type == "SETUP_STEP") ends.push_back(event.details);
    }
  }

  void Run(double seconds) {
    for (const double end = now + seconds - 1e-9; now < end;) Step();
  }

  // Runs until the step ended, at most seconds; its SETUP_STEP.
  json RunToEnd(double seconds) {
    const size_t before = ends.size();
    for (const double end = now + seconds; now < end && ends.size() == before;) Step();
    return ends.size() > before ? ends.back() : json();
  }

  std::string Start(const std::string& id, json confirm = kConfirm) {
    running_step = id;
    json params = {{"step", id}, {"version", program.version()}};
    if (!confirm.is_null()) params["confirm"] = confirm;
    const auto refusal = program.Start(params, now, kWall + now);
    return refusal ? refusal->reason : "";
  }

  json StepState(const std::string& id) {
    const json state = program.State(now, {});
    for (const auto& step : state["steps"]) {
      if (step["id"] == id) return step;
    }
    return nullptr;
  }

  // Runs until the step asks which wheel turns, then answers.
  bool AnswerWheel(const std::string& wheel, double seconds = 25) {
    for (const double end = now + seconds; now < end;) {
      Step();
      const json step = StepState("wheels");
      for (const auto& input : step["inputs"]) {
        if (input["name"] != "wheel" || !input["ask"].get<bool>()) continue;
        Run(0.5);
        json params = {{"step", "wheels"}, {"version", program.version()}, {"values", {{"wheel", wheel}}}};
        return !program.Answer(params, now, kWall + now);
      }
    }
    return false;
  }

  // Runs until the wheels turn in this part of stop_moving (from 1), then triggers the input with its emergency and
  // lets go after hold s.
  void PressWhileTurning(int part, bool Hardware::*input, const char* bit, double hold = 0.5) {
    const auto turning = [this, part] {
      const json progress = StepState("stop_moving")["progress"];
      return progress.is_object() && progress["n"] == part && std::abs(setpoint.linear) > 0.01 &&
             hardware.emergency_reason == 0;
    };
    for (const double end = now + 10; now < end && !turning();) Step();
    Run(0.5);
    hardware.*input = true;
    Latch(Bit(bit));
    Run(hold);
    hardware.*input = false;
  }

  // The blade as the logic and the board turn it: in SETUP without emergency the logic asks the board for it, renewing
  // the lease while the step asks BLADE_ON and once on BLADE_ONCE; the board turns it up within a second and stops it
  // blade_stop s after its lease ran out, an emergency or hold_without_host s without its host.
  void TurnBlade(const std::optional<Motion>& motion, bool host) {
    const int asked = motion && world == "SETUP" && hardware.emergency_reason == 0 && host ? motion->blade : 0;
    if (asked == 1 || (asked == 2 && blade_asked != 2)) blade_until = now + motion->blade_lease_s;
    if (asked == 0 && blade_asked != 0 && host) blade_until = -1;
    blade_asked = asked;
    if (host) heard_at = now;
    const bool on = capabilities.blade_allowed && now < blade_until && hardware.emergency_reason == 0 &&
                    now - heard_at <= hold_without_host;
    if (on) {
      if (blade_since < 0) blade_since = now;
      blade_off_since = -1;
      rpm = std::min(1.0, (now - blade_since) / 0.8) * 3000;
    } else {
      blade_since = -1;
      if (blade_off_since < 0) blade_off_since = now;
      if (now - blade_off_since >= blade_stop && !blade_runs_on) rpm = 0;
    }
    if (restarts_blade && hardware.emergency_reason == 0 && blade_off_since >= 0 && now - blade_off_since > 2.5) {
      rpm = 3000;
    }
    hardware.blade_ok = true;
    hardware.blade_rpm = rpm;
  }

  // Runs until part n of blade (from 1) waits with the blade turning, then triggers the input with its emergency and
  // lets go after hold s.
  void PressWhileBlade(int part, bool Hardware::*input, uint32_t bits, double hold = 0.5) {
    const auto waits = [this, part] {
      const json step = StepState("blade");
      return step["progress"].is_object() && step["progress"]["n"] == part && !step["ends_at"].is_null();
    };
    for (const double end = now + 30; now < end && !waits();) Step();
    ASSERT_TRUE(waits()) << StepState("blade").dump();
    ASSERT_GE(rpm, 2500);
    Run(0.3);
    hardware.*input = true;
    Latch(bits);
    Run(hold);
    hardware.*input = false;
  }

  // On the ground with an RTK fix; with a date the lock is open.
  void PutOnGround(bool date = true) {
    on_ground = true;
    if (date) capabilities.safety_measurement_date = "2026-11-17";
    program.OnCapabilities(capabilities);
    Run(0.5);
  }

  // Runs until part n of ground (from 1) drives, then triggers the input with its emergency and lets go after hold s.
  void PressOnGround(int part, bool Hardware::*input, uint32_t bits, double hold = 0.5) {
    const auto driving = [this, part] {
      const json progress = StepState("ground")["progress"];
      return progress.is_object() && progress["n"] == part && std::abs(setpoint.linear) > 0.01 &&
             hardware.emergency_reason == 0;
    };
    for (const double end = now + 30; now < end && !driving();) Step();
    ASSERT_TRUE(driving()) << StepState("ground").dump();
    Run(0.5);
    hardware.*input = true;
    Latch(bits);
    Run(hold);
    hardware.*input = false;
  }

  Program program;
  Capabilities capabilities;
  Hardware hardware;
  double now = 0;
  std::string world = "IDLE";
  bool logic_takes = true;
  bool armed = true;
  bool watchdog = true;
  bool swapped = false;
  bool keep = true;
  bool comes_back = true;
  double hold_without_host = 0.2;
  double watchdog_s = 0.5;
  bool on_ground = false;
  double x = 0;
  double y = 0;
  double yaw = 0;
  int solution = 4;
  double imu_scale = 1;
  double gnss_scale = 1;
  double fix_at = -1;
  Command setpoint;
  double setpoint_at = -1;
  double silent_until = -1;
  double gone_until = -1;
  double hearing_at = -1;
  double kept_at = -1;
  std::string silence_code;
  bool silence_blade = false;
  std::string running_step;
  double rpm = 0;
  int blade_asked = 0;
  double blade_until = -1;
  double heard_at = 0;
  double blade_since = -1;
  double blade_off_since = -1;
  double blade_stop = 0.3;
  bool blade_runs_on = false;
  bool restarts_blade = false;
  int releases = 0;
  int silences = 0;
  std::vector<json> ends;
  std::vector<std::string> motion_ends;
};

json ValueOf(const json& step, const std::string& id) {
  for (const auto& value : step["values"]) {
    if (value["id"] == id) return value;
  }
  return nullptr;
}

}  // namespace

TEST(Motion, AStepThatMovesNeedsTheInputsAndTheConfirmations) {
  Mower fresh(false);
  EXPECT_EQ(fresh.StepState("wheels")["blocked"][0]["reason"], "SAFETY_INPUTS");
  EXPECT_EQ(fresh.Start("wheels"), "SAFETY_INPUTS");
  EXPECT_FALSE(fresh.program.MotionNow());

  Mower mower;
  EXPECT_EQ(mower.StepState("wheels")["blocked"], json::array());
  EXPECT_EQ(mower.Start("wheels", json::array({"raised"})), "CONFIRM_MISSING");
  mower.hardware.dock_contact = true;
  mower.Run(0.1);
  EXPECT_EQ(mower.Start("wheels"), "ON_DOCK");
  mower.hardware.dock_contact = false;
  mower.Latch(Bit("STOP"));
  mower.Run(0.1);
  EXPECT_EQ(mower.Start("wheels"), "EMERGENCY");
  mower.hardware.emergency_reason = 0;
  mower.program.SetDriverRestarts(1);
  mower.Run(0.1);
  EXPECT_EQ(mower.Start("driver_end"), "RESTARTS_LOW");
  mower.capabilities.silence_max_s = 0;
  mower.program.OnCapabilities(mower.capabilities);
  EXPECT_EQ(mower.Start("silence"), "NOT_OFFERED");
  // The motion of a step names its bounds whether it runs or not.
  const json motion = mower.StepState("wheels")["motion"];
  EXPECT_EQ(motion["raised"], true);
  EXPECT_EQ(motion["max_speed"], 0.05);
  EXPECT_EQ(motion["max_time"], mowbite_api::kSetupMoveTime);
}

TEST(Motion, TheStepListHoldsTheRulesOfMoving) {
  std::string why;
  json list = json::parse(kList);
  list["steps"][3]["confirm"] = json::array({"raised"});
  EXPECT_FALSE(ParseSteps(list, &why));
  EXPECT_NE(why.find("hand_at_stop"), std::string::npos) << why;
  list = json::parse(kList);
  list["steps"].erase(1);
  EXPECT_FALSE(ParseSteps(list, &why));
  EXPECT_NE(why.find("input of lift"), std::string::npos) << why;
  list = json::parse(kList);
  list["steps"][3]["parts"] = json::array({{{"id", "forward"}}});
  EXPECT_FALSE(ParseSteps(list, &why));
  list = json::parse(kList);
  list["steps"][3]["params"] = {{"speed", 0.2}};
  EXPECT_FALSE(ParseSteps(list, &why));
}

TEST(Motion, WheelsTurnPartByPartAndOneWheelIsAskedFor) {
  Mower mower;
  ASSERT_EQ(mower.Start("wheels"), "");
  ASSERT_TRUE(mower.program.MotionNow());
  EXPECT_EQ(mower.program.MotionNow()->max_linear, 0.05);
  EXPECT_DOUBLE_EQ(mower.program.MotionNow()->max_angular, 0.05 / (kTrack / 2));
  ASSERT_TRUE(mower.AnswerWheel("right")) << (mower.ends.empty() ? "" : mower.ends.back().dump());
  const json end = mower.RunToEnd(10);
  ASSERT_FALSE(end.is_null());
  EXPECT_EQ(end["state"], "ok") << end.dump();
  const json step = mower.StepState("wheels");
  for (const char* part : {"forward", "backward", "left", "right", "one_wheel"}) {
    const json value = ValueOf(step, part);
    EXPECT_NEAR(value["value"].get<double>(), 0.05, 0.005) << part;
    EXPECT_EQ(value["ok"], true) << part;
  }
  mower.Run(0.1);
  EXPECT_EQ(mower.motion_ends, std::vector<std::string>{"wheels"});
  EXPECT_FALSE(mower.program.MotionNow());
  EXPECT_FALSE(mower.program.CommandNow());
}

TEST(Motion, AWrongAnswerOrSwappedWheelsFail) {
  Mower answer;
  ASSERT_EQ(answer.Start("wheels"), "");
  ASSERT_TRUE(answer.AnswerWheel("left"));
  json end = answer.RunToEnd(5);
  EXPECT_EQ(end["reason"], "WRONG_WHEEL") << end.dump();
  EXPECT_EQ(end["answer"], "left");

  Mower swapped;
  swapped.swapped = true;
  ASSERT_EQ(swapped.Start("wheels"), "");
  end = swapped.RunToEnd(30);
  EXPECT_EQ(end["reason"], "WRONG_DIRECTION") << end.dump();
  EXPECT_EQ(end["part"], "left");
}

TEST(Motion, StopMovingStopsReleasesAndTurnsAgain) {
  Mower mower;
  ASSERT_EQ(mower.Start("stop_moving"), "");
  mower.PressWhileTurning(1, &Hardware::stop, "STOP");
  mower.PressWhileTurning(2, &Hardware::bumper_left, "COLLISION", 0.1);
  const json end = mower.RunToEnd(20);
  ASSERT_FALSE(end.is_null());
  EXPECT_EQ(end["state"], "ok") << end.dump();
  EXPECT_GE(mower.releases, 2);
  EXPECT_EQ(mower.hardware.emergency_reason, 0u);
  const json stand = ValueOf(mower.StepState("stop_moving"), "stop");
  EXPECT_LE(stand["value"].get<double>(), 0.1);
  EXPECT_EQ(stand["ok"], true);
}

TEST(Motion, AnotherEmergencyEndsTheStepAndIsNotReleased) {
  Mower mower;
  ASSERT_EQ(mower.Start("stop_moving"), "");
  mower.PressWhileTurning(1, &Hardware::lift, "LIFT");
  ASSERT_FALSE(mower.ends.empty());
  const json end = mower.ends.back();
  EXPECT_EQ(end["reason"], "EMERGENCY") << end.dump();
  EXPECT_EQ(mower.releases, 0);
  EXPECT_EQ(mower.motion_ends, std::vector<std::string>{"stop_moving"});
}

TEST(Motion, NoCommandStandsByTheWatchdogs) {
  Mower mower;
  ASSERT_EQ(mower.Start("no_command"), "");
  json end = mower.RunToEnd(20);
  EXPECT_EQ(end["state"], "ok") << end.dump();
  EXPECT_NEAR(ValueOf(mower.StepState("no_command"), "stand")["value"].get<double>(), 0.5, 0.06);

  Mower runs_on;
  runs_on.watchdog = false;
  ASSERT_EQ(runs_on.Start("no_command"), "");
  end = runs_on.RunToEnd(20);
  EXPECT_EQ(end["reason"], "LIMIT") << end.dump();
}

TEST(Motion, SilenceRoundsStopReleaseAndTurnAgain) {
  Mower mower;
  ASSERT_EQ(mower.Start("silence"), "");
  json end = mower.RunToEnd(40);
  EXPECT_EQ(end["state"], "ok") << end.dump();
  EXPECT_EQ(mower.silences, 2);
  const json step = mower.StepState("silence");
  EXPECT_NEAR(ValueOf(step, "round_1")["value"].get<double>(), 0.2, 0.06);
  EXPECT_NEAR(ValueOf(step, "round_2")["value"].get<double>(), 0.2, 0.06);
  EXPECT_EQ(ValueOf(step, "spread")["ok"], true);

  // A board that keeps its setpoint without the host fails the first round.
  Mower holds;
  holds.hold_without_host = 5;
  holds.capabilities.silence_max_s = 2;
  holds.program.OnCapabilities(holds.capabilities);
  ASSERT_EQ(holds.Start("silence"), "");
  end = holds.RunToEnd(20);
  EXPECT_EQ(end["reason"], "LIMIT") << end.dump();
  EXPECT_EQ(end["part"], "round_1");

  Mower refuses;
  refuses.silence_code = "CHARGING";
  ASSERT_EQ(refuses.Start("silence"), "");
  end = refuses.RunToEnd(10);
  EXPECT_EQ(end["reason"], "DRIVER_REFUSED") << end.dump();
  EXPECT_EQ(end["code"], "CHARGING");
}

TEST(Motion, DriverEndComesBackLatchedAndIsReleased) {
  Mower mower;
  ASSERT_EQ(mower.Start("driver_end"), "");
  json end = mower.RunToEnd(30);
  EXPECT_EQ(end["state"], "ok") << end.dump();
  EXPECT_NEAR(ValueOf(mower.StepState("driver_end"), "return")["value"].get<double>(), 3.8, 0.1);
  EXPECT_EQ(mower.hardware.emergency_reason, 0u);

  Mower gone;
  gone.comes_back = false;
  ASSERT_EQ(gone.Start("driver_end"), "");
  end = gone.RunToEnd(30);
  EXPECT_EQ(end["reason"], "NO_RETURN") << end.dump();
}

TEST(Motion, WithoutKeepOrLogicTheStepStops) {
  Mower lost;
  ASSERT_EQ(lost.Start("no_command"), "");
  lost.Run(1);
  lost.keep = false;
  json end = lost.RunToEnd(2);
  EXPECT_EQ(end["reason"], "RELEASED") << end.dump();
  EXPECT_FALSE(lost.program.CommandNow());

  Mower refused;
  refused.logic_takes = false;
  ASSERT_EQ(refused.Start("wheels"), "");
  end = refused.RunToEnd(5);
  EXPECT_EQ(end["reason"], "LOGIC_REFUSED") << end.dump();

  Mower aborted;
  ASSERT_EQ(aborted.Start("wheels"), "");
  aborted.Run(1);
  EXPECT_FALSE(aborted.program.Abort(json::object(), aborted.now, kWall + aborted.now));
  aborted.Run(0.1);
  EXPECT_EQ(aborted.motion_ends, std::vector<std::string>{"wheels"});
  EXPECT_FALSE(aborted.program.CommandNow());
}

TEST(Motion, AStopNotPressedInTimeAsksWithTheWheelsStanding) {
  for (const char* answer : {"no", "yes"}) {
    Mower mower;
    ASSERT_EQ(mower.Start("stop_moving"), "");
    mower.Run(12);
    ASSERT_TRUE(mower.program.running()) << answer;
    json pressed;
    const json step = mower.StepState("stop_moving");
    for (const auto& input : step["inputs"]) {
      if (input["name"] == "pressed") pressed = input;
    }
    ASSERT_EQ(pressed["ask"], true) << answer;
    // The step still runs with setup.keep, its motion stands.
    ASSERT_TRUE(mower.program.MotionNow());
    ASSERT_TRUE(mower.program.CommandNow());
    EXPECT_EQ(mower.program.CommandNow()->linear, 0.0);
    json params = {{"step", "stop_moving"}, {"version", mower.program.version()}, {"values", {{"pressed", answer}}}};
    ASSERT_FALSE(mower.program.Answer(params, mower.now, kWall + mower.now)) << answer;
    const json end = mower.RunToEnd(1);
    EXPECT_EQ(end["reason"], std::string(answer) == "yes" ? "NOT_SEEN" : "NOT_DONE") << end.dump();
    EXPECT_EQ(end["part"], "stop");
    EXPECT_EQ(mower.StepState("stop_moving")["state"], std::string(answer) == "yes" ? "failed" : "open");
    EXPECT_EQ(mower.program.TakeLock().has_value(), std::string(answer) == "yes") << answer;
    mower.Run(0.1);
    EXPECT_FALSE(mower.program.CommandNow());
  }
}

// setup.reset while the wheels turn ends the motion at once, as setup.abort does.
TEST(Motion, AResetWhileTheWheelsTurnEndsTheMotion) {
  Mower mower;
  ASSERT_EQ(mower.Start("wheels"), "");
  mower.Run(1);
  ASSERT_TRUE(mower.program.CommandNow());
  EXPECT_FALSE(mower.program.Reset({{"version", mower.program.version()}}, kWall + mower.now));
  mower.Run(0.1);
  EXPECT_EQ(mower.motion_ends, std::vector<std::string>{"wheels"});
  EXPECT_FALSE(mower.program.MotionNow());
  EXPECT_FALSE(mower.program.CommandNow());
}

// The question pressed waits for its answer without setup.keep, the wheels standing.
TEST(Motion, TheQuestionPressedWaitsWithoutKeep) {
  Mower mower;
  ASSERT_EQ(mower.Start("stop_moving"), "");
  mower.Run(12);
  ASSERT_TRUE(mower.program.running());
  mower.keep = false;
  mower.Run(5);
  ASSERT_TRUE(mower.program.running());
  ASSERT_TRUE(mower.program.CommandNow());
  EXPECT_EQ(mower.program.CommandNow()->linear, 0.0);
  EXPECT_EQ(mower.program.CommandNow()->angular, 0.0);
  json params = {{"step", "stop_moving"}, {"version", mower.program.version()}, {"values", {{"pressed", "yes"}}}};
  ASSERT_FALSE(mower.program.Answer(params, mower.now, kWall + mower.now));
  const json end = mower.RunToEnd(1);
  EXPECT_EQ(end["reason"], "NOT_SEEN") << end.dump();
  EXPECT_TRUE(mower.program.TakeLock().has_value());
}

TEST(Motion, NoAnswerWhichWheelTurnsIsNotDone) {
  Mower mower;
  ASSERT_EQ(mower.Start("wheels"), "");
  const json end = mower.RunToEnd(40);
  EXPECT_EQ(end["reason"], "NOT_DONE") << end.dump();
  EXPECT_EQ(mower.StepState("wheels")["state"], "open");
  EXPECT_FALSE(mower.program.TakeLock());
}

TEST(Motion, WheelsThatRunOnCloseTheLock) {
  Mower runs_on;
  runs_on.watchdog = false;
  ASSERT_EQ(runs_on.Start("no_command"), "");
  const json end = runs_on.RunToEnd(20);
  ASSERT_EQ(end["reason"], "LIMIT") << end.dump();
  const auto text = runs_on.program.TakeLock();
  ASSERT_TRUE(text);
  const json lock = json::parse(*text);
  EXPECT_EQ(lock["failures"], 1);
  EXPECT_EQ(lock["causes"][0]["step"], "no_command");
  EXPECT_EQ(lock["causes"][0]["reason"], "LIMIT");
  EXPECT_EQ(lock["causes"][0]["limit"], runs_on.StepState("no_command")["values"][0]["max"]);
}

TEST(Ground, DrivesItsPartsInOneRunAndPasses) {
  Mower mower(true, GroundList().c_str());
  mower.PutOnGround();
  const json motion = mower.StepState("ground")["motion"];
  EXPECT_EQ(motion["raised"], false);
  EXPECT_EQ(motion["max_speed"], mowbite_api::kSetupGroundSpeed);
  EXPECT_EQ(mower.Start("ground"), "CONFIRM_MISSING");
  ASSERT_EQ(mower.Start("ground", json::array({"area_clear"})), "");
  ASSERT_TRUE(mower.program.MotionNow());
  EXPECT_FALSE(mower.program.MotionNow()->raised);
  EXPECT_DOUBLE_EQ(mower.program.MotionNow()->max_linear, 0.3);
  EXPECT_DOUBLE_EQ(mower.program.MotionNow()->max_angular, 0.3);
  // straight and turn by themselves; stop back towards the start.
  mower.PressOnGround(3, &Hardware::stop, Bit("STOP"));
  EXPECT_LT(mower.setpoint.linear, 0.0);
  // Pressed in its middle the bumper may bring both sides at once.
  mower.PressOnGround(4, &Hardware::bumper_left, Bit("COLLISION") | Bit("COLLISION_MULTIPLE"));
  const json end = mower.RunToEnd(20);
  ASSERT_EQ(end["state"], "ok") << end.dump();
  const json step = mower.StepState("ground");
  for (const auto& value : step["values"]) EXPECT_EQ(value["ok"], true) << value.dump();
  EXPECT_NEAR(ValueOf(step, "straight_gnss")["value"].get<double>(), 1.0, 0.03);
  EXPECT_NEAR(ValueOf(step, "turn_left")["value"].get<double>(), 1.0, 0.03);
  EXPECT_NEAR(ValueOf(step, "turn_right")["value"].get<double>(), 1.0, 0.03);
  // The watchdog of 0.5 s: about 0.15 m on after the program fell silent.
  EXPECT_NEAR(ValueOf(step, "straight_stand")["value"].get<double>(), 0.5, 0.1);
  EXPECT_NEAR(ValueOf(step, "straight_way")["value"].get<double>(), 0.15, 0.04);
  EXPECT_LE(ValueOf(step, "stop_stand")["value"].get<double>(), 0.1);
  EXPECT_EQ(ValueOf(step, "turn_left")["texts"]["de"]["name"], "Drehen links: Drehung Lagesensor");
  EXPECT_EQ(ValueOf(step, "turn_right")["texts"]["en"]["name"], "Turn right: Turn of the motion sensor");
  // Both emergencies released by the program.
  EXPECT_EQ(mower.hardware.emergency_reason, 0u);
  EXPECT_EQ(mower.releases, 2);
  mower.Run(0.1);
  EXPECT_EQ(mower.motion_ends, std::vector<std::string>{"ground"});
}

TEST(Ground, NeedsAFixAndWithTheLockClosedTheStepsJackedUp) {
  Mower mower(true, GroundList().c_str());
  mower.solution = 3;
  mower.PutOnGround(false);
  json blocked = mower.StepState("ground")["blocked"];
  ASSERT_EQ(blocked.size(), 2u) << blocked.dump();
  EXPECT_EQ(blocked[0]["reason"], "NO_RTK");
  EXPECT_EQ(blocked[1]["reason"], "SAFETY_AIR");
  EXPECT_EQ(mower.Start("ground", json::array({"area_clear"})), "NO_RTK");
  mower.solution = 4;
  mower.Run(0.5);
  EXPECT_EQ(mower.Start("ground", json::array({"area_clear"})), "SAFETY_AIR");
  // Passed jacked up, also with the lock closed.
  Mower passed(false, GroundList().c_str());
  json record = json::parse(kInputsPassed);
  for (const char* step : {"wheels", "stop_moving", "no_command", "silence", "driver_end"}) {
    record["steps"][step] = {{"state", "ok"}};
  }
  ASSERT_TRUE(passed.program.Load(record.dump(), kWall, nullptr));
  passed.PutOnGround(false);
  EXPECT_EQ(passed.StepState("ground")["blocked"], json::array());
  // With the lock open the steps jacked up are not asked for.
  Mower open(true, GroundList().c_str());
  open.PutOnGround(true);
  EXPECT_EQ(open.StepState("ground")["blocked"], json::array());
}

TEST(Ground, AMowerThatDoesNotStandClosesTheLock) {
  {
    Mower runs_on(true, GroundList().c_str());
    runs_on.watchdog = false;
    runs_on.PutOnGround();
    ASSERT_EQ(runs_on.Start("ground", json::array({"area_clear"})), "");
    const json end = runs_on.RunToEnd(30);
    ASSERT_EQ(end["reason"], "NO_STANDSTILL") << end.dump();
    EXPECT_EQ(end["part"], "straight");
    const auto text = runs_on.program.TakeLock();
    ASSERT_TRUE(text);
    EXPECT_EQ(json::parse(*text)["causes"][0]["reason"], "NO_STANDSTILL");
    runs_on.Run(0.1);
    EXPECT_FALSE(runs_on.program.CommandNow());
  }
  Mower slow(true, GroundList().c_str());
  slow.watchdog_s = 1.5;
  slow.PutOnGround();
  ASSERT_EQ(slow.Start("ground", json::array({"area_clear"})), "");
  const json end = slow.RunToEnd(30);
  ASSERT_EQ(end["reason"], "LIMIT") << end.dump();
  EXPECT_EQ(end["value_id"], "straight_stand");
  EXPECT_TRUE(slow.program.TakeLock());
}

TEST(Ground, TheWayByGnssAndTheTurnOfTheImuAreChecked) {
  {
    Mower gnss(true, GroundList().c_str());
    gnss.gnss_scale = 1.5;
    gnss.PutOnGround();
    ASSERT_EQ(gnss.Start("ground", json::array({"area_clear"})), "");
    const json end = gnss.RunToEnd(30);
    ASSERT_EQ(end["reason"], "LIMIT") << end.dump();
    EXPECT_EQ(end["value_id"], "straight_gnss");
  }
  {
    Mower imu(true, GroundList().c_str());
    imu.imu_scale = -1;
    imu.PutOnGround();
    ASSERT_EQ(imu.Start("ground", json::array({"area_clear"})), "");
    const json end = imu.RunToEnd(40);
    ASSERT_EQ(end["reason"], "WRONG_DIRECTION") << end.dump();
    EXPECT_EQ(end["part"], "turn");
    EXPECT_TRUE(imu.program.TakeLock());
  }
  Mower lost(true, GroundList().c_str());
  lost.PutOnGround();
  ASSERT_EQ(lost.Start("ground", json::array({"area_clear"})), "");
  lost.Run(2);
  lost.solution = 3;
  const json end = lost.RunToEnd(30);
  ASSERT_EQ(end["reason"], "NO_RTK") << end.dump();
  EXPECT_FALSE(lost.program.TakeLock());
}

TEST(Ground, AStopNotPressedAsksOnceBackAtTheStart) {
  Mower mower(true, GroundList().c_str());
  mower.PutOnGround();
  ASSERT_EQ(mower.Start("ground", json::array({"area_clear"})), "");
  json pressed;
  for (const double end = mower.now + 60; mower.now < end && pressed.is_null();) {
    mower.Step();
    const json step = mower.StepState("ground");
    for (const auto& input : step["inputs"]) {
      if (input["name"] == "pressed" && input["ask"].get<bool>()) pressed = input;
    }
  }
  ASSERT_FALSE(pressed.is_null()) << (mower.ends.empty() ? mower.StepState("ground").dump() : mower.ends.back().dump());
  EXPECT_EQ(mower.StepState("ground")["progress"]["n"], 3);
  // It came back to about where straight began and stands there.
  EXPECT_LT(std::abs(mower.x), 0.3);
  EXPECT_EQ(mower.program.CommandNow()->linear, 0.0);
  json params = {{"step", "ground"}, {"version", mower.program.version()}, {"values", {{"pressed", "no"}}}};
  ASSERT_FALSE(mower.program.Answer(params, mower.now, kWall + mower.now));
  const json end = mower.RunToEnd(1);
  EXPECT_EQ(end["reason"], "NOT_DONE") << end.dump();
  EXPECT_EQ(end["part"], "stop");
  EXPECT_FALSE(mower.program.TakeLock());
}

namespace {

// The mower of BladeList with the blade allowed by the driver.
struct BladeMower : Mower {
  BladeMower() : Mower(true, BladeList().c_str()) {
    capabilities.blade_allowed = true;
    capabilities.blade_lease_max_s = 2.0;
    program.OnCapabilities(capabilities);
    Run(0.2);
  }
};

}  // namespace

TEST(Blade, StopsItsBladeInSixWaysAndPasses) {
  BladeMower mower;
  const json motion = mower.StepState("blade")["motion"];
  EXPECT_EQ(motion["blade"], true);
  EXPECT_EQ(motion["wheels"], false);
  EXPECT_EQ(mower.StepState("blade")["restart"], false);
  EXPECT_EQ(mower.Start("blade"), "CONFIRM_MISSING");
  ASSERT_EQ(mower.Start("blade", kBladeConfirm), "");
  ASSERT_TRUE(mower.program.MotionNow());
  EXPECT_EQ(mower.program.MotionNow()->max_linear, 0);
  EXPECT_EQ(mower.program.MotionNow()->blade_percent, 100);
  EXPECT_DOUBLE_EQ(mower.program.MotionNow()->blade_lease_s, 1.0);
  mower.PressWhileBlade(4, &Hardware::stop, Bit("STOP"));
  EXPECT_TRUE(mower.silence_blade);
  // Lifted at the front both sensors may switch.
  mower.PressWhileBlade(5, &Hardware::lift, Bit("LIFT") | Bit("LIFT_MULTIPLE"));
  mower.PressWhileBlade(6, &Hardware::bumper_left, Bit("COLLISION"));
  const json end = mower.RunToEnd(20);
  ASSERT_EQ(end["state"], "ok") << end.dump();
  const json step = mower.StepState("blade");
  for (const auto& value : step["values"]) EXPECT_EQ(value["ok"], true) << value.dump();
  // The speed when it passed rpm.
  EXPECT_GE(ValueOf(step, "rpm")["value"].get<double>(), 2500);
  EXPECT_NEAR(ValueOf(step, "on_off")["value"].get<double>(), 0.3, 0.1);
  // One lease of 1 s, then the board's 0.3 s.
  EXPECT_NEAR(ValueOf(step, "lease")["value"].get<double>(), 1.3, 0.1);
  EXPECT_NEAR(ValueOf(step, "silent")["value"].get<double>(), 0.5, 0.1);
  EXPECT_LE(ValueOf(step, "stop")["value"].get<double>(), 0.4);
  EXPECT_EQ(mower.hardware.emergency_reason, 0u);
  // silent, stop, lift, bumper: four emergencies released by the program.
  EXPECT_EQ(mower.releases, 4);
  EXPECT_EQ(mower.silences, 1);
  // No wheel got a command.
  EXPECT_LT(mower.setpoint_at, 0);
  mower.Run(0.1);
  EXPECT_EQ(mower.motion_ends, std::vector<std::string>{"blade"});
  EXPECT_FALSE(mower.program.MotionNow());
}

TEST(Blade, TheFirstStartAllowsTheBladeAndRestartsNotTheStep) {
  Mower mower(true, BladeList().c_str());
  mower.program.SetLayer(Layer(), {});
  mower.Run(0.2);
  EXPECT_EQ(mower.StepState("blade")["restart"], true);
  EXPECT_EQ(mower.StepState("blade")["blocked"], json::array());
  const int version = mower.program.version();
  ASSERT_EQ(mower.Start("blade", kBladeConfirm), "");
  // No attempt, no run: the step waits for the next start after the restart.
  EXPECT_FALSE(mower.program.running());
  EXPECT_GT(mower.program.version(), version);
  EXPECT_EQ(mower.StepState("blade")["attempt"], 0);
  EXPECT_EQ(mower.StepState("blade")["state"], "open");
  const auto text = mower.program.TakeLayer();
  ASSERT_TRUE(text);
  EXPECT_NE(text->find("allow_blade: true"), std::string::npos) << *text;
  EXPECT_FALSE(mower.program.TakeRestart(mower.now));
  mower.program.LayerWritten(true, mower.now, kWall);
  mower.program.TakeChanged();
  EXPECT_EQ(mower.Start("blade", kBladeConfirm), "BUSY");
  EXPECT_TRUE(mower.program.TakeRestart(mower.now));
  EXPECT_EQ(mower.StepState("blade")["blocked"][0]["reason"], "BUSY");

  // After the restart: the driver allows it, or the file kept by hand or the driver does not.
  Layer layer;
  ASSERT_TRUE(layer.Parse(*text, nullptr));
  Mower after(true, BladeList().c_str());
  after.program.SetLayer(layer, {});
  after.Run(0.2);
  json blocked = after.StepState("blade")["blocked"];
  ASSERT_EQ(blocked.size(), 1u) << blocked.dump();
  EXPECT_EQ(blocked[0]["reason"], "BLADE_NOT_ALLOWED");
  EXPECT_EQ(after.Start("blade", kBladeConfirm), "BLADE_NOT_ALLOWED");
  Mower hand(true, BladeList().c_str());
  hand.program.SetLayer(Layer(), {"tron_mcu.allow_blade"});
  hand.Run(0.2);
  const auto refusal = hand.program.Start(
      {{"step", "blade"}, {"version", hand.program.version()}, {"confirm", kBladeConfirm}}, hand.now, kWall);
  ASSERT_TRUE(refusal);
  EXPECT_EQ(refusal->reason, "BLADE_NOT_ALLOWED");
  EXPECT_EQ(refusal->data["key"], "tron_mcu.allow_blade");
  after.capabilities.blade_allowed = true;
  after.program.OnCapabilities(after.capabilities);
  after.Run(0.2);
  EXPECT_EQ(after.StepState("blade")["restart"], false);
  EXPECT_EQ(after.Start("blade", kBladeConfirm), "");
  EXPECT_TRUE(after.program.running());
}

// allow_blade goes out as blade_allowed once the parameter file holds it; a file not written leaves the step open
// with NOT_APPLIED and no restart.
TEST(Blade, AllowBladeIsToldOnlyOnceWritten) {
  for (const bool written : {true, false}) {
    Mower mower(true, BladeList().c_str());
    mower.program.SetLayer(Layer(), {});
    mower.Run(0.2);
    ASSERT_EQ(mower.Start("blade", kBladeConfirm), "");
    for (const Event& event : mower.program.TakeEvents()) EXPECT_NE(event.details.value("what", ""), "blade_allowed");
    ASSERT_TRUE(mower.program.TakeLayer());
    mower.program.LayerWritten(written, mower.now, kWall + mower.now);
    bool allowed = false;
    std::string reason;
    for (const Event& event : mower.program.TakeEvents()) {
      json details = event.details;
      details["id"] = "x";
      details["t"] = kWall;
      details["type"] = event.type;
      const auto problems = mowbite_api::CheckEvent(event.type, mowbite_api::Json(details));
      EXPECT_TRUE(problems.empty()) << event.type << ": " << (problems.empty() ? "" : problems[0]);
      if (event.type == "SETUP" && event.details["what"] == "blade_allowed") allowed = true;
      if (event.type == "SETUP_STEP" && event.details["step"] == "blade") reason = event.details.value("reason", "");
    }
    EXPECT_EQ(allowed, written);
    EXPECT_EQ(reason, written ? "" : "NOT_APPLIED");
    if (!written) {
      EXPECT_FALSE(mower.program.TakeRestart(mower.now));
    }
    EXPECT_EQ(mower.StepState("blade")["state"], "open");
  }
}

TEST(Blade, ABladeThatDoesNotStandOrRunsAgainClosesTheLock) {
  {
    BladeMower mower;
    mower.blade_stop = 1.5;
    ASSERT_EQ(mower.Start("blade", kBladeConfirm), "");
    const json end = mower.RunToEnd(10);
    EXPECT_EQ(end["state"], "failed");
    EXPECT_EQ(end["reason"], "LIMIT");
    EXPECT_EQ(end["part"], "on_off");
    EXPECT_EQ(end["value_id"], "on_off");
    EXPECT_EQ(mower.StepState("blade")["values"][1]["ok"], false);
    const json safety = mower.program.State(mower.now, {})["safety"];
    EXPECT_EQ(safety["why"], "CHECK_FAILED") << safety.dump();
    EXPECT_EQ(safety["closed"][0]["step"], "blade");
    mower.Run(0.1);
    EXPECT_FALSE(mower.program.MotionNow());
  }
  {
    BladeMower mower;
    mower.restarts_blade = true;
    ASSERT_EQ(mower.Start("blade", kBladeConfirm), "");
    const json end = mower.RunToEnd(20);
    EXPECT_EQ(end["state"], "failed");
    EXPECT_EQ(end["reason"], "STARTED_AGAIN") << end.dump();
    EXPECT_EQ(mower.program.State(mower.now, {})["safety"]["why"], "CHECK_FAILED");
  }
}

TEST(Blade, ABladeThatDoesNotComeUpFailsWithoutClosing) {
  BladeMower mower;
  ASSERT_EQ(mower.Start("blade", kBladeConfirm), "");
  // The board refuses: the blade does not turn.
  mower.capabilities.blade_allowed = false;
  const json end = mower.RunToEnd(10);
  EXPECT_EQ(end["reason"], "NO_START") << end.dump();
  EXPECT_EQ(end["rpm"], 0);
  EXPECT_EQ(mower.program.State(mower.now, {})["safety"]["closed"], json::array());
}

TEST(Blade, AnInputNotTriggeredInTimeAsksWithTheBladeOff) {
  BladeMower mower;
  ASSERT_EQ(mower.Start("blade", kBladeConfirm), "");
  const auto asks = [&mower] {
    const json step = mower.StepState("blade");
    for (const auto& input : step["inputs"]) {
      if (input["name"] == "pressed" && input["ask"] == true) return true;
    }
    return false;
  };
  for (const double end = mower.now + 40; mower.now < end && !asks();) mower.Step();
  ASSERT_TRUE(asks());
  EXPECT_EQ(mower.StepState("blade")["progress"]["n"], 4);
  ASSERT_TRUE(mower.program.MotionNow());
  EXPECT_EQ(mower.program.MotionNow()->blade, 0);
  mower.Run(0.5);
  EXPECT_EQ(mower.rpm, 0);
  json params = {{"step", "blade"}, {"version", mower.program.version()}, {"values", {{"pressed", "no"}}}};
  EXPECT_FALSE(mower.program.Answer(params, mower.now, kWall + mower.now));
  const json end = mower.RunToEnd(1);
  EXPECT_EQ(end["reason"], "NOT_DONE");
}

TEST(Blade, TheStepListHoldsItsRules) {
  json list = json::parse(BladeList());
  list["steps"].back()["confirm"] = {"blade_removed"};
  std::string why;
  EXPECT_FALSE(ParseSteps(list, &why));
  EXPECT_NE(why.find("hand_at_stop"), std::string::npos) << why;
  list = json::parse(BladeList());
  list.erase("restart");
  EXPECT_FALSE(ParseSteps(list, &why));
  EXPECT_NE(why.find("restart"), std::string::npos) << why;
  list = json::parse(BladeList());
  list["steps"].back()["parts"] = {"on_off"};
  EXPECT_FALSE(ParseSteps(list, &why));
  // Parts in their fixed order, stop, lift and bumper wait for their input.
  const auto steps = ParseSteps(json::parse(BladeList()), &why);
  ASSERT_TRUE(steps) << why;
  const Step& blade = steps->steps.back();
  ASSERT_EQ(blade.parts.size(), 6u);
  EXPECT_EQ(blade.parts[0].id, "on_off");
  EXPECT_EQ(blade.parts[3].role, "stop");
  EXPECT_EQ(blade.parts[4].role, "lift");
  EXPECT_EQ(blade.parts[5].role, "bumper");
  EXPECT_EQ(blade.parts[2].role, "");
}
