#include <gtest/gtest.h>

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
    const double kept = host ? (watchdog ? 0.5 : 1e9) : hold_without_host;
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
    program.Tick(now, kWall + now);
    if (const auto release = program.TakeRelease()) {
      releases++;
      if (release->latch_count == hardware.latch_count && !hardware.stop && !hardware.bumper_left) {
        hardware.emergency_reason = 0;
      }
    }
    if (const auto seconds = program.TakeSilence()) {
      silences++;
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
  Command setpoint;
  double setpoint_at = -1;
  double silent_until = -1;
  double gone_until = -1;
  double hearing_at = -1;
  double kept_at = -1;
  std::string silence_code;
  std::string running_step;
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
