#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <map>
#include <memory>
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
    if (fw) Board();
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
    const double kept = host ? (watchdog ? watchdog_s : 1e9) : silence_last ? setpoint_hold : hold_without_host;
    double left = 0, right = 0;
    if (!stalled && hardware.emergency_reason == 0 && setpoint_at >= 0 && now - setpoint_at <= kept) {
      left = setpoint.linear - setpoint.angular * kTrack / 2;
      right = setpoint.linear + setpoint.angular * kTrack / 2;
      if (swapped) std::swap(left, right);
    }
    wheel_left = left;
    wheel_right = right;
    if (silent_until >= 0 && now >= silent_until) {
      silent_until = -1;
      Latch(Bit("HIGH_LEVEL"));
    }
    if (gone_until >= 0 && now >= gone_until) {
      gone_until = -1;
      if (comes_back) {
        // A new driver: latched from its first frame, and until it hears its board an input missing as well; on
        // firmware it does not list with the measurement closed until a release while it hears the signal.
        hardware.emergency_reason = Bit("HIGH_LEVEL") | Bit("LATCH") | Bit("TIMEOUT_INPUTS") |
                                    (fw ? Bit("FIRMWARE_INCOMPATIBLE") : 0u);
        hardware.latch_count = 1;
        hardware.telemetry_ok = false;
        hearing_at = now + 0.8;
        odom_from = now + odom_delay;
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
      if (now >= odom_from) program.OnWheels(left * wheel_scale, right * wheel_scale, now, stamps ? now : -1);
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
      release_reasons.push_back(release->reason);
      // A release clears the board as well while stop or lift shows (tron_mcu).
      if (fw && (hardware.stop || hardware.lift)) {
        ++release_clears;
        Clear();
      }
      if (release->latch_count == hardware.latch_count && !hardware.stop && !hardware.bumper_left &&
          !(fw && (hardware.lift || hardware.bumper_right))) {
        const uint32_t closed = fw && !program.MeasureNow() ? Bit("FIRMWARE_INCOMPATIBLE") : 0u;
        hardware.emergency_reason &= closed;
      }
    }
    if (const auto seconds = program.TakeSilence()) {
      silences++;
      silence_last = program.SilenceLastSetpoint();
      silence_blade = program.SilenceWithBlade();
      if (silence_code.empty()) silent_until = now + *seconds;
      if (silence_late > 0 && silence_code.empty()) {
        answer_at = now + silence_late;
        silent_from = now;
      } else {
        program.OnSilenceAnswer(silence_code.empty(), silence_code, now, kWall + now, stamps ? now : -1);
      }
    }
    if (answer_at >= 0 && now >= answer_at) {
      answer_at = -1;
      program.OnSilenceAnswer(true, "", now, kWall + now, stamps ? silent_from : -1);
    }
    if (silent_until < 0) silence_last = false;
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
      if (event.type == "SETUP") setup_events.push_back(event.details);
    }
    if (node) {
      // What the node does after a tick: the lock, the record, the parameter file, then a restart.
      if (program.TakeLock()) program.LockWritten(true);
      if (program.TakeChanged()) saved = program.Text();
      if (const auto text = program.TakeLayer()) {
        layer_text = *text;
        program.LayerWritten(layer_ok, now, kWall + now);
      }
      if (program.TakeRestart(now)) ++restarts;
    }
  }

  void Run(double seconds) {
    for (const double end = now + seconds - 1e-9; now < end;) Step();
  }

  // Runs until the step ended, at most seconds; its SETUP_STEP.
  json RunToEnd(double seconds) {
    return EndSince(ends.size(), seconds);
  }

  // The last end once there are more than before, running at most seconds for one; null without.
  json EndSince(size_t before, double seconds) {
    for (const double end = now + seconds; now < end && ends.size() <= before;) Step();
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
      const json step = StepState(wheels_step);
      for (const auto& input : step["inputs"]) {
        if (input["name"] != "wheel" || !input["ask"].get<bool>()) continue;
        Run(0.5);
        json params = {{"step", wheels_step}, {"version", program.version()}, {"values", {{"wheel", wheel}}}};
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
    if (blade_stalled) rpm = 0;
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

  // The board under the owner measurement of firmware (fw), as tron_mcu and the board behave: unlisted firmware stays
  // latched with FIRMWARE_INCOMPATIBLE until setup/firmware_measure opens it, and closing latches it anew; stop and
  // lift stay switched after the input until a clear, which only a new clear_seq of the signal brings while the wheels
  // and the blade stand; a bumper side follows its input; hw/link_stats every second and right after a clear.
  void Board() {
    const auto signal = program.MeasureNow();
    program.TakeMeasureEnd();
    const bool none = capabilities.firmware_source == "none";
    const bool open = none && signal && signal->device_hash == capabilities.device_hash &&
                      signal->firmware == capabilities.firmware;
    if (fw_open && !open) {
      Latch(Bit("FIRMWARE_INCOMPATIBLE"));
      setpoint_at = -1;
      blade_until = -1;
    }
    fw_open = open;
    if (none && !open) hardware.emergency_reason |= Bit("FIRMWARE_INCOMPATIBLE");
    if (press_stop && !hardware.stop) {
      hardware.stop = true;
      Latch(Bit("STOP"));
    }
    if (press_lift && !hardware.lift) {
      hardware.lift = true;
      Latch(Bit("LIFT"));
    }
    // Without the signal the driver clears by itself what is let go.
    if (!press_stop && (!stop_latches || !signal)) hardware.stop = false;
    if (!press_lift && (!lift_latches || !signal)) hardware.lift = false;
    const bool left = swap_bumpers ? press_right : press_left;
    const bool right = swap_bumpers ? press_left : press_right;
    if ((left && !hardware.bumper_left) || (right && !hardware.bumper_right)) Latch(Bit("COLLISION"));
    hardware.bumper_left = left;
    hardware.bumper_right = right;
    hardware.bumper = left || right;
    if (!signal) {
      fw_seen.reset();
    } else if (!fw_seen) {
      fw_seen = signal->clear_seq;
    } else if (signal->clear_seq != *fw_seen && std::abs(wheel_left) < 0.025 && std::abs(wheel_right) < 0.025 &&
               rpm < 100) {
      fw_seen = signal->clear_seq;
      link.clear_seq = signal->clear_seq;
      ++fw_clears;
      Clear();
      SendLink();
    }
    if (link_at < 0 || now - link_at >= 1.0) SendLink();
  }

  void Clear() {
    if (!press_stop || clear_releases_held) hardware.stop = false;
    if (!press_lift || clear_releases_held) hardware.lift = false;
  }

  void SendLink() {
    const double dt = link_at < 0 ? 0 : now - link_at;
    link_at = now;
    link.stamp = now;
    for (const auto& [module, hz] : link_hz) {
      link_due[module] += hz * dt;
      link.frames[module] = static_cast<uint64_t>(link_due[module]);
    }
    checksum_due += checksum_hz * dt;
    link.checksum_errors = static_cast<uint64_t>(checksum_due);
    program.OnLinkStats(link, now);
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
  // The wheels or the blade stand whatever is asked; a new driver sends hw/wheel_odom odom_delay s after it began.
  bool stalled = false;
  bool blade_stalled = false;
  double odom_delay = 0;
  double odom_from = -1;
  int releases = 0;
  std::vector<std::string> release_reasons;
  int silences = 0;
  std::vector<json> ends;
  std::vector<std::string> motion_ends;
  std::string wheels_step = "wheels";
  std::vector<json> setup_events;
  // The odometry shows the wheels this many times as fast as they turn.
  double wheel_scale = 1;
  // The driver stamps hw/wheel_odom and the start of a silence on the clock of the test; the answer of hw/silence
  // arrives silence_late s after the silence began.
  bool stamps = true;
  double silence_late = 0;
  double answer_at = -1;
  double silent_from = -1;
  // With node: the record, the parameter file as written last, and the restarts asked for.
  bool node = false;
  bool layer_ok = true;
  std::string saved;
  std::string layer_text;
  int restarts = 0;
  // The board of Board(): the inputs as pressed, its variants, the window, the clear_seq it went by, the clears it did
  // on the signal and on a release, the wheel speeds of the last step, its link counts.
  bool fw = false;
  bool press_stop = false;
  bool press_lift = false;
  bool press_left = false;
  bool press_right = false;
  bool stop_latches = true;
  bool lift_latches = true;
  bool clear_releases_held = false;
  bool swap_bumpers = false;
  bool fw_open = false;
  std::optional<uint32_t> fw_seen;
  int fw_clears = 0;
  int release_clears = 0;
  double wheel_left = 0;
  double wheel_right = 0;
  // The board keeps the last setpoint this long without its host when the silence keeps it (s).
  double setpoint_hold = 3.6;
  bool silence_last = false;
  std::map<std::string, double> link_hz = {{"sensor_info", 100}, {"speed", 100}, {"motors", 3.4},
                                           {"version", 1.7},     {"battery", 1.7}, {"bms", 1.7}};
  std::map<std::string, double> link_due;
  double checksum_hz = 0;
  double checksum_due = 0;
  LinkCounts link;
  double link_at = -1;
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

// After answer_time the wheel of one_wheel stands and the question stays for kSetupPressedTime.
TEST(Motion, OneWheelStandsAfterItsTurnAndNoAnswerIsNotDone) {
  const auto asks = [](Mower& mower) {
    const json step = mower.StepState("wheels");
    for (const auto& input : step["inputs"]) {
      if (input["name"] == "wheel") return input["ask"].get<bool>();
    }
    return false;
  };
  const auto until_asked = [&asks](Mower& mower) {
    for (const double end = mower.now + 25; mower.now < end && !asks(mower);) mower.Step();
    return asks(mower);
  };
  Mower late;
  ASSERT_EQ(late.Start("wheels"), "");
  ASSERT_TRUE(until_asked(late));
  late.Run(10.5);
  ASSERT_TRUE(late.ends.empty()) << late.ends.back().dump();
  EXPECT_TRUE(asks(late));
  ASSERT_TRUE(late.program.CommandNow());
  EXPECT_EQ(late.program.CommandNow()->linear, 0.0);
  late.Run(20);
  EXPECT_TRUE(late.ends.empty());
  json params = {{"step", "wheels"}, {"version", late.program.version()}, {"values", {{"wheel", "right"}}}};
  EXPECT_FALSE(late.program.Answer(params, late.now, kWall + late.now));
  const json end = late.RunToEnd(10);
  EXPECT_EQ(end["state"], "ok") << end.dump();
  EXPECT_NEAR(ValueOf(late.StepState("wheels"), "one_wheel")["value"].get<double>(), 0.05, 0.005);

  Mower none;
  ASSERT_EQ(none.Start("wheels"), "");
  ASSERT_TRUE(until_asked(none));
  const double asked = none.now;
  const json failed = none.RunToEnd(45);
  EXPECT_EQ(failed["reason"], "NOT_DONE") << failed.dump();
  EXPECT_GT(none.now - asked, 10 + mowbite_api::kSetupPressedTime - 1);
  EXPECT_EQ(none.StepState("wheels")["state"], "open");
  EXPECT_FALSE(none.program.TakeLock());
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

// Standing shows kStandHold (0.3 s) after the wheels stood: a stop_max shorter than that still passes a quick stop.
TEST(Motion, AShortStopMaxJudgesWhenTheWheelsStood) {
  json list = json::parse(kList);
  for (auto& step : list["steps"]) {
    if (step["id"] == "stop_moving") step["params"]["stop_max"] = 0.2;
  }
  const std::string text = list.dump();
  Mower mower(true, text.c_str());
  ASSERT_EQ(mower.Start("stop_moving"), "");
  mower.PressWhileTurning(1, &Hardware::stop, "STOP");
  mower.PressWhileTurning(2, &Hardware::bumper_left, "COLLISION", 0.1);
  const json end = mower.RunToEnd(20);
  EXPECT_EQ(end.value("state", json()), "ok") << end.dump();
  EXPECT_LE(ValueOf(mower.StepState("stop_moving"), "stop")["value"].get<double>(), 0.1);
}

TEST(Motion, WheelsThatStoodBeforeTheTriggerMeasureNothing) {
  Mower mower;
  ASSERT_EQ(mower.Start("stop_moving"), "");
  const auto turning = [&mower] {
    const json progress = mower.StepState("stop_moving")["progress"];
    return progress.is_object() && progress["n"] == 1 && std::abs(mower.setpoint.linear) > 0.01;
  };
  for (const double end = mower.now + 10; mower.now < end && !turning();) mower.Step();
  mower.Run(0.5);
  // The wheels stop for another reason, then the input comes.
  mower.stalled = true;
  mower.Run(0.6);
  mower.hardware.stop = true;
  mower.Latch(Bit("STOP"));
  mower.Run(0.5);
  mower.hardware.stop = false;
  mower.stalled = false;
  // It ends at once, before the input is gone.
  ASSERT_FALSE(mower.ends.empty());
  const json end = mower.ends.back();
  EXPECT_EQ(end["reason"], "NO_START") << end.dump();
  EXPECT_EQ(mower.StepState("stop_moving")["state"], "failed");
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

// The answer of hw/silence comes 0.15 s late: by the stamps of the driver the stand stays as it was.
TEST(Motion, SilenceMeasuresByTheStampsOfTheDriver) {
  Mower late;
  late.silence_late = 0.15;
  ASSERT_EQ(late.Start("silence"), "");
  json end = late.RunToEnd(40);
  EXPECT_EQ(end["state"], "ok") << end.dump();
  EXPECT_NEAR(ValueOf(late.StepState("silence"), "round_1")["value"].get<double>(), 0.2, 0.06);

  // Without stamps it counts from the arrival.
  Mower arrival;
  arrival.silence_late = 0.15;
  arrival.stamps = false;
  ASSERT_EQ(arrival.Start("silence"), "");
  end = arrival.RunToEnd(40);
  EXPECT_EQ(end["state"], "ok") << end.dump();
  EXPECT_NEAR(ValueOf(arrival.StepState("silence"), "round_1")["value"].get<double>(), 0.05, 0.06);
}

TEST(Motion, DriverEndComesBackLatchedAndIsReleased) {
  Mower mower;
  ASSERT_EQ(mower.Start("driver_end"), "");
  json end = mower.RunToEnd(30);
  EXPECT_EQ(end["state"], "ok") << end.dump();
  EXPECT_NEAR(ValueOf(mower.StepState("driver_end"), "return")["value"].get<double>(), 3.8, 0.1);
  EXPECT_EQ(mower.hardware.emergency_reason, 0u);
  // The latch came from the step, not from an input: the reason says so.
  ASSERT_FALSE(mower.release_reasons.empty());
  for (const auto& reason : mower.release_reasons) EXPECT_EQ(reason, "setup: the latch of the step was checked");

  // The new driver sends its wheel speeds only after its status: the old speeds do not count.
  Mower late;
  late.odom_delay = 1.5;
  ASSERT_EQ(late.Start("driver_end"), "");
  end = late.RunToEnd(30);
  EXPECT_EQ(end["state"], "ok") << end.dump();

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

TEST(Blade, ABladeThatStoodBeforeTheTriggerMeasuresNothing) {
  BladeMower mower;
  ASSERT_EQ(mower.Start("blade", kBladeConfirm), "");
  const auto waits = [&mower] {
    const json step = mower.StepState("blade");
    return step["progress"].is_object() && step["progress"]["n"] == 4 && !step["ends_at"].is_null();
  };
  for (const double end = mower.now + 30; mower.now < end && !waits();) mower.Step();
  ASSERT_TRUE(waits());
  mower.blade_stalled = true;
  mower.Run(0.6);
  mower.hardware.stop = true;
  mower.Latch(Bit("STOP"));
  mower.Run(0.5);
  mower.hardware.stop = false;
  ASSERT_FALSE(mower.ends.empty());
  const json end = mower.ends.back();
  EXPECT_EQ(end["reason"], "NO_START") << end.dump();
}

TEST(Blade, AStepThatMovesWaitsForARestartThatIsAskedFor) {
  Mower mower(true, BladeList().c_str());
  mower.program.SetLayer(Layer(), {});
  mower.Run(0.2);
  ASSERT_EQ(mower.Start("blade", kBladeConfirm), "");
  EXPECT_EQ(mower.Start("no_command"), "BUSY");
  ASSERT_TRUE(mower.program.TakeLayer());
  mower.program.LayerWritten(true, mower.now, kWall);
  mower.program.TakeChanged();
  ASSERT_TRUE(mower.program.TakeRestart(mower.now));
  EXPECT_EQ(mower.Start("no_command"), "BUSY");
  // A restart that never comes does not block for good.
  mower.Run(121);
  EXPECT_EQ(mower.Start("blade", kBladeConfirm), "BLADE_NOT_ALLOWED");
  EXPECT_EQ(mower.Start("no_command"), "");
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

namespace {

const json kLinkConfirm = json::array({"raised", "blade_removed", "hand_at_stop"});
const char* const kEntry = "0.6.38/0.6.34/d3v1ce";

// The stage firmware of the owner measurement, shorter than the Tron's.
std::string FwList() {
  return json::parse(R"({
  "catalog": 3,
  "restart": "self",
  "steps": [
    {"id": "fw_link", "stage": "firmware", "firmware": true, "check": "link",
     "params": {"time": 10, "fast": ["sensor_info", "speed"], "fast_min": 80, "fast_max": 120, "medium": ["motors"],
                "medium_min": 2, "slow": ["version", "battery", "bms"], "slow_min": 1},
     "confirm": ["raised", "blade_removed", "hand_at_stop"]},
    {"id": "fw_stop", "stage": "firmware", "firmware": true, "check": "input", "params": {"role": "stop", "rounds": 2}},
    {"id": "fw_lift", "stage": "firmware", "firmware": true, "check": "input", "params": {"role": "lift"}},
    {"id": "fw_bumper", "stage": "firmware", "firmware": true, "check": "input", "params": {"role": "bumper"},
     "parts": [{"id": "left", "role": "bumper_left"}, {"id": "right", "role": "bumper_right"}]},
    {"id": "fw_latch", "stage": "firmware", "firmware": true, "check": "latch",
     "params": {"time": 10, "hold_min": 1, "clear_max": 0.2, "clear_every": 0.5, "clears": 2, "settle": 1,
                "rounds": 1},
     "parts": [{"id": "stop_short", "role": "stop", "mode": "short"},
               {"id": "stop_hold", "role": "stop", "mode": "press"},
               {"id": "stop_free", "role": "stop", "mode": "release"},
               {"id": "bumper_left", "role": "bumper_left", "mode": "short"},
               {"id": "lift_hold", "role": "lift", "mode": "press"},
               {"id": "lift_free", "role": "lift", "mode": "release"}]},
    {"id": "fw_wheels", "stage": "firmware", "firmware": true, "check": "wheels",
     "params": {"speed_min": 0.8, "speed_max": 1.2, "rounds": 2}, "confirm": ["raised", "hand_at_stop"]},
    {"id": "fw_setpoint", "stage": "firmware", "firmware": true, "check": "silence",
     "params": {"rounds": 2, "stop_max": 4.0, "emergency": ["HIGH_LEVEL"], "last_setpoint": "yes"},
     "confirm": ["raised", "hand_at_stop"]},
    {"id": "fw_release", "stage": "firmware", "firmware": true, "check": "firmware",
     "params": {"node": "tron_mcu", "boards": ["cutter", "chassis"], "restart_time": 30}}
  ]})")
      .dump();
}

// A Tron on firmware tron_mcu does not list (firmware_source none), latched with FIRMWARE_INCOMPATIBLE from its start.
struct FwMower : Mower {
  explicit FwMower(const std::string& source = "none", const std::string& list = FwList())
      : Mower(false, list.c_str()) {
    fw = true;
    node = true;
    wheels_step = "fw_wheels";
    capabilities.firmware = {{"cutter", "0.6.38"}, {"chassis", "0.6.34"}, {"bms", "1.0.0"}};
    capabilities.firmware_source = source;
    capabilities.device_hash = "d3v1ce";
    capabilities.silence_last_setpoint_max_s = 5.0;
    program.OnCapabilities(capabilities);
    program.SetLayer(layer, {});
    if (source == "none") hardware.emergency_reason = Bit("FIRMWARE_INCOMPATIBLE") | Bit("LATCH");
    Run(1.5);
    program.TakeEvents();
    ends.clear();
  }

  json Fw() {
    return program.State(now, {})["firmware"];
  }

  // Runs until the step shows part n (from 1) waiting with nothing in its way.
  bool WaitPart(const std::string& id, int n, double seconds = 15) {
    const auto ready = [&] {
      const json step = StepState(id);
      return step["state"] == "running" && step["progress"].is_object() && step["progress"]["n"] == n &&
             step["blocked"].empty() && hardware.emergency_reason == 0;
    };
    for (const double end = now + seconds; now < end && !ready();) Step();
    return ready();
  }

  void Press(bool Mower::*input, double hold = 0.3) {
    this->*input = true;
    Run(hold);
    this->*input = false;
  }

  // An input step, part by part and round by round.
  json PassInput(const std::string& id, const std::vector<bool Mower::*>& presses) {
    const size_t before = ends.size();
    if (const auto code = Start(id, nullptr); code != "") return {{"start", code}, {"step", id}};
    for (size_t i = 0; i < presses.size(); ++i) {
      if (!WaitPart(id, static_cast<int>(i) + 1)) {
        return ends.size() > before ? ends.back() : json{{"waiting", StepState(id)}};
      }
      Run(0.3);
      Press(presses[i]);
    }
    return EndSince(before, 15);
  }

  // The parts of fw_latch as the app asks for them; bumper: how long the left side is pressed.
  json PassLatch(double bumper = 0.3) {
    const size_t before = ends.size();
    if (const auto code = Start("fw_latch", nullptr); code != "") return {{"start", code}, {"step", "fw_latch"}};
    const auto wait = [this](int n) {
      for (const double end = now + 15; now < end;) {
        const json step = StepState("fw_latch");
        if (step["progress"].is_object() && step["progress"]["n"] == n && step["blocked"].empty()) return true;
        if (!ends.empty() && ends.back()["step"] == "fw_latch") return false;
        Step();
      }
      return false;
    };
    if (!wait(1)) return ends.empty() ? json() : ends.back();
    Run(0.3);
    Press(&Mower::press_stop);
    if (!wait(2)) return ends.back();
    Run(0.3);
    press_stop = true;
    if (!wait(3)) return ends.back();
    Run(0.2);
    press_stop = false;
    if (!wait(4)) return ends.back();
    Run(0.3);
    Press(&Mower::press_left, bumper);
    if (!wait(5)) return ends.back();
    Run(0.3);
    press_lift = true;
    if (!wait(6)) return ends.back();
    Run(0.2);
    press_lift = false;
    return EndSince(before, 15);
  }

  json PassWheels() {
    if (const auto code = Start("fw_wheels"); code != "") return {{"start", code}, {"step", "fw_wheels"}};
    for (int round = 0; round < 2; ++round) {
      if (!AnswerWheel("right")) return ends.empty() ? json() : ends.back();
    }
    return RunToEnd(30);
  }

  // Every step but fw_release; the first end that is not ok, or null.
  json PassRun() {
    const auto ok = [](const json& end) { return end.is_object() && end.value("state", "") == "ok"; };
    json end = Start("fw_link", kLinkConfirm) == "" ? RunToEnd(20) : json{{"start", "fw_link"}};
    if (!ok(end)) return end;
    end = PassInput("fw_stop", {&Mower::press_stop, &Mower::press_stop});
    if (!ok(end)) return end;
    end = PassInput("fw_lift", {&Mower::press_lift});
    if (!ok(end)) return end;
    end = PassInput("fw_bumper", {&Mower::press_left, &Mower::press_right});
    if (!ok(end)) return end;
    end = PassLatch();
    if (!ok(end)) return end;
    end = PassWheels();
    if (!ok(end)) return end;
    end = Start("fw_setpoint") == "" ? RunToEnd(60) : json{{"start", "fw_setpoint"}};
    if (!ok(end)) return end;
    return nullptr;
  }

  // The mower after the restart: the record and the parameter file as written, the driver with source.
  std::unique_ptr<FwMower> Restarted(const std::string& source) {
    Layer written;
    EXPECT_TRUE(written.Parse(layer_text, nullptr));
    auto after = std::make_unique<FwMower>(source, FwList(), written, saved);
    return after;
  }

  FwMower(const std::string& source, const std::string& list, const Layer& file, const std::string& record)
      : Mower(false, list.c_str()) {
    fw = true;
    node = true;
    wheels_step = "fw_wheels";
    EXPECT_TRUE(program.Load(record, kWall + now, nullptr));
    layer = file;
    capabilities.firmware = {{"cutter", "0.6.38"}, {"chassis", "0.6.34"}, {"bms", "1.0.0"}};
    capabilities.firmware_source = source;
    capabilities.device_hash = "d3v1ce";
    capabilities.silence_last_setpoint_max_s = 5.0;
    program.SetLayer(layer, {});
    program.OnCapabilities(capabilities);
    if (source == "none") hardware.emergency_reason = Bit("FIRMWARE_INCOMPATIBLE") | Bit("LATCH");
  }

  Layer layer;
};

}  // namespace

TEST(Firmware, TheStageHoldsItsRules) {
  std::string why;
  ASSERT_TRUE(ParseSteps(json::parse(FwList()), &why)) << why;
  const auto broken = [](const std::function<void(json&)>& change) {
    json list = json::parse(FwList());
    change(list);
    std::string why;
    EXPECT_FALSE(ParseSteps(list, &why)) << list.dump();
    return why;
  };
  EXPECT_EQ(broken([](json& l) { l["steps"][0]["firmware"] = false; }),
            "fw_link: firmware goes with the stage firmware");
  EXPECT_EQ(broken([](json& l) { l["steps"][0]["stage"] = "device"; }),
            "fw_link: firmware goes with the stage firmware");
  EXPECT_EQ(broken([](json& l) { l["steps"][1]["skippable"] = true; }), "fw_stop: firmware is not skippable");
  EXPECT_EQ(broken([](json& l) { l["steps"].erase(0); }), "fw_release comes after a step link");
  EXPECT_EQ(broken([](json& l) { l["steps"].erase(l["steps"].size() - 1); }),
            "the stage firmware has one step firmware");
  EXPECT_EQ(broken([](json& l) { l["steps"].back()["params"]["boards"] = json::array(); }),
            "fw_release names at least one board");
  EXPECT_EQ(broken([](json& l) {
              json last = l["steps"].back();
              l["steps"].erase(l["steps"].size() - 1);
              l["steps"].insert(l["steps"].begin() + 5, last);
            }),
            "fw_release is the last step with firmware");
  EXPECT_EQ(broken([](json& l) { l["steps"][4]["parts"].erase(1); }),
            "fw_latch: release comes right after a press of stop");
  EXPECT_EQ(broken([](json& l) { l["steps"][4]["parts"][3]["mode"] = "press"; }),
            "fw_latch: press of bumper_left is no part of latch");
  EXPECT_EQ(broken([](json& l) { l["steps"][4]["parts"][4]["mode"] = "short"; }),
            "fw_latch: short of lift is no part of latch");
  EXPECT_EQ(broken([](json& l) {
              l["steps"].insert(l["steps"].begin(), json{{"id", "x"}, {"stage", "device"}, {"check", "latch"}});
            }),
            "x: latch goes with firmware");
  // Its own input steps count for its wheels, those of the inputs stage do not.
  EXPECT_EQ(broken([](json& l) { l["steps"].erase(2); }),
            "fw_wheels: a step that moves the wheels or the blade comes after a step input of lift");
  const auto list = ParseSteps(json::parse(FwList()), &why);
  for (const Step& step : list->steps) EXPECT_FALSE(step.skippable) << step.id;
}

TEST(Firmware, ShownOnlyForFirmwareTheDriverDoesNotLet) {
  FwMower listed("listed");
  EXPECT_TRUE(listed.StepState("fw_link").is_null());
  EXPECT_EQ(listed.Fw()["offered"], false);
  EXPECT_EQ(listed.Start("fw_link", kLinkConfirm), "NOT_OFFERED");
  EXPECT_TRUE(listed.program.State(listed.now, {})["complete"].get<bool>());
  listed.program.SetMeasureForce(true);
  EXPECT_FALSE(listed.StepState("fw_link").is_null());
  EXPECT_EQ(listed.Fw()["force"], true);

  FwMower older("older");
  EXPECT_FALSE(older.StepState("fw_link").is_null());
  EXPECT_EQ(older.Start("fw_link", kLinkConfirm), "FIRMWARE_OLDER");
  EXPECT_EQ(older.StepState("fw_link")["blocked"][0]["reason"], "FIRMWARE_OLDER");

  FwMower none;
  EXPECT_EQ(none.Fw()["offered"], true);
  EXPECT_EQ(none.Fw()["device"], true);
  EXPECT_EQ(none.StepState("fw_stop")["firmware"], true);
  EXPECT_EQ(none.StepState("fw_stop")["safety"], false);
  EXPECT_EQ(none.program.State(none.now, {})["safety"]["firmware_source"], "none");
  // In order within a run.
  EXPECT_EQ(none.Start("fw_latch", nullptr), "LOCKED");
  none.capabilities.device_hash = "";
  none.program.OnCapabilities(none.capabilities);
  EXPECT_EQ(none.Start("fw_link", kLinkConfirm), "NO_DEVICE_ID");
}

TEST(Firmware, ARunThatPassesWritesTheEntryAndRestarts) {
  FwMower mower;
  const json end = mower.PassRun();
  ASSERT_TRUE(end.is_null()) << end.dump();
  // The program asked no clear but through the signal while an input showed.
  EXPECT_EQ(mower.release_clears, 0);
  EXPECT_GE(mower.fw_clears, 6);
  for (const char* id : {"fw_stop", "fw_wheels", "fw_setpoint"}) {
    EXPECT_EQ(mower.StepState(id)["state"], "ok") << id;
  }
  const json setpoint = mower.StepState("fw_setpoint");
  EXPECT_NEAR(ValueOf(setpoint, "round_1")["value"].get<double>(), 3.6, 0.1) << setpoint.dump();
  ASSERT_EQ(mower.Start("fw_release", nullptr), "");
  mower.Run(1.0);
  EXPECT_EQ(mower.restarts, 1);
  EXPECT_NE(mower.layer_text.find(kEntry), std::string::npos) << mower.layer_text;
  EXPECT_EQ(mower.Fw()["written"], kEntry);
  EXPECT_EQ(mower.Fw()["passed"], 1);
  // The lock never moved.
  EXPECT_EQ(mower.program.State(mower.now, {})["safety"]["closed"], json::array());

  auto after = mower.Restarted("measured");
  const json done = after->RunToEnd(5);
  EXPECT_EQ(done["state"], "ok") << done.dump();
  bool measured = false;
  for (const json& event : after->setup_events) measured = measured || event["what"] == "firmware_measured";
  EXPECT_TRUE(measured);
  const json exported = after->program.Export();
  ASSERT_EQ(exported["runs"].size(), 1u);
  EXPECT_EQ(exported["runs"][0]["result"], "passed");
  EXPECT_EQ(exported["runs"][0]["entry"], kEntry);
  EXPECT_EQ(exported["runs"][0]["link"]["checksum_errors"], 0);
  EXPECT_GT(exported["runs"][0]["link"]["frames"]["speed"].get<int>(), 1000);
  EXPECT_EQ(exported["runs"][0]["steps"]["fw_latch"]["state"], "ok");
  EXPECT_EQ(exported["device_hash"], "d3v1ce");

  // Back without the entry: the driver did not take it.
  auto refused = mower.Restarted("none");
  EXPECT_EQ(refused->RunToEnd(5)["reason"], "NOT_APPLIED");
}

TEST(Firmware, ARealFailureNeedsTwoRunsAndTakesTheEntryBack) {
  FwMower mower("measured");
  mower.program.SetMeasureForce(true);
  mower.layer.Set("tron_mcu", "measured_firmware", json::array({kEntry}));
  mower.program.SetLayer(mower.layer, {});
  mower.Run(0.2);
  mower.checksum_hz = 0.5;
  ASSERT_EQ(mower.Start("fw_link", kLinkConfirm), "");
  json end = mower.RunToEnd(20);
  EXPECT_EQ(end["reason"], "LIMIT") << end.dump();
  EXPECT_EQ(mower.Fw()["failed"], 1);
  EXPECT_EQ(mower.Fw()["needed"], 2);
  EXPECT_EQ(mower.Fw()["passed"], 0);
  mower.Run(0.5);
  EXPECT_EQ(mower.layer_text.find(kEntry), std::string::npos) << mower.layer_text;
  EXPECT_EQ(mower.restarts, 1);
  bool revoked = false;
  for (const json& event : mower.setup_events) revoked = revoked || event["what"] == "firmware_revoked";
  EXPECT_TRUE(revoked);
  EXPECT_EQ(mower.program.Export()["runs"][0]["result"], "failed");

  // After the restart without the entry: two runs in a row, the first ends RUN_AGAIN without writing.
  auto after = mower.Restarted("none");
  end = after->PassRun();
  ASSERT_TRUE(end.is_null()) << end.dump();
  ASSERT_EQ(after->Start("fw_release", nullptr), "");
  end = after->RunToEnd(2);
  EXPECT_EQ(end["reason"], "RUN_AGAIN") << end.dump();
  EXPECT_EQ(end["passed"], 1);
  EXPECT_EQ(after->StepState("fw_link")["state"], "ok");
  EXPECT_EQ(after->restarts, 0);
  end = after->PassRun();
  ASSERT_TRUE(end.is_null()) << end.dump();
  ASSERT_EQ(after->Start("fw_release", nullptr), "");
  after->Run(1.0);
  EXPECT_EQ(after->restarts, 1);
  EXPECT_EQ(after->Fw()["passed"], 2);
  EXPECT_EQ(after->Fw()["written"], kEntry);
  EXPECT_EQ(after->program.Export()["runs"].size(), 3u);

  // Forced on listed firmware a passed run writes nothing.
  FwMower listed("listed");
  listed.program.SetMeasureForce(true);
  end = listed.PassRun();
  ASSERT_TRUE(end.is_null()) << end.dump();
  ASSERT_EQ(listed.Start("fw_release", nullptr), "");
  end = listed.RunToEnd(2);
  EXPECT_EQ(end["state"], "ok") << end.dump();
  EXPECT_TRUE(listed.Fw()["written"].is_null());
  EXPECT_EQ(listed.restarts, 0);
}

TEST(Firmware, OperatorErrorsRepeatAndOnlyTooManyEndTheRun) {
  FwMower mower;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    ASSERT_EQ(mower.Start("fw_link", kLinkConfirm), "") << attempt;
    mower.Run(2);
    ASSERT_FALSE(mower.program.Abort(json::object(), mower.now, kWall + mower.now));
    mower.Run(0.5);
    const json runs = mower.program.Export()["runs"];
    ASSERT_EQ(runs.size(), 1u);
    EXPECT_EQ(runs[0]["result"], attempt < 3 ? "running" : "aborted") << runs[0].dump();
  }
  EXPECT_EQ(mower.program.Export()["runs"][0]["reason"], "OPERATOR_ERRORS fw_link");
  EXPECT_EQ(mower.Fw()["failed"], 0);
  EXPECT_EQ(mower.Fw()["needed"], 1);
}

TEST(Firmware, AResetTakesTheEntryAndEveryCount) {
  FwMower mower("measured");
  mower.layer.Set("tron_mcu", "measured_firmware", json::array({kEntry}));
  mower.program.SetLayer(mower.layer, {});
  mower.program.SetMeasureForce(true);
  mower.checksum_hz = 0.5;
  ASSERT_EQ(mower.Start("fw_link", kLinkConfirm), "");
  mower.RunToEnd(20);
  mower.Run(0.5);
  ASSERT_EQ(mower.Fw()["failed"], 1);
  const int restarts = mower.restarts;
  mower.layer.Set("tron_mcu", "measured_firmware", json::array({"0.6.40/0.6.34/d3v1ce"}));
  mower.program.SetLayer(mower.layer, {});
  ASSERT_FALSE(mower.program.Reset({{"version", mower.program.version()}}, kWall + mower.now));
  mower.Run(0.5);
  EXPECT_EQ(mower.layer_text.find("measured_firmware"), std::string::npos) << mower.layer_text;
  EXPECT_EQ(mower.restarts, restarts + 1);
  EXPECT_EQ(mower.Fw()["failed"], 0);
  EXPECT_EQ(mower.program.Export()["runs"], json::array());
}

TEST(Firmware, ABoardThatHoldsOtherwiseFails) {
  // Each board fails a part of fw_latch; the run counts as failed.
  struct Variant {
    const char* name;
    std::function<void(FwMower&)> set;
    const char* part;
  };
  const std::vector<Variant> variants = {
      {"stop does not latch", [](FwMower& m) { m.stop_latches = false; }, "stop_short"},
      {"a clear lets a held stop go", [](FwMower& m) { m.clear_releases_held = true; }, "stop_hold"},
      {"lift does not latch", [](FwMower& m) { m.lift_latches = false; }, "lift_free"},
  };
  for (const auto& variant : variants) {
    FwMower mower;
    variant.set(mower);
    ASSERT_EQ(mower.Start("fw_link", kLinkConfirm), "");
    ASSERT_EQ(mower.RunToEnd(20)["state"], "ok");
    const json stop = mower.PassInput("fw_stop", {&Mower::press_stop, &Mower::press_stop});
    ASSERT_EQ(stop["state"], "ok") << variant.name << " " << stop.dump();
    const json lift = mower.PassInput("fw_lift", {&Mower::press_lift});
    ASSERT_EQ(lift["state"], "ok") << variant.name << " " << lift.dump();
    const json bumper = mower.PassInput("fw_bumper", {&Mower::press_left, &Mower::press_right});
    ASSERT_EQ(bumper["state"], "ok") << variant.name << " " << bumper.dump();
    const json end = mower.PassLatch();
    EXPECT_EQ(end["state"], "failed") << variant.name << " " << end.dump();
    EXPECT_EQ(end.value("part", json()), variant.part) << variant.name << " " << end.dump();
    EXPECT_EQ(mower.program.Export()["runs"][0]["result"], "failed") << variant.name;
    EXPECT_EQ(mower.release_clears, 0) << variant.name;
  }

  // Swapped bumper sides: the input step sees the other side.
  FwMower swapped;
  swapped.swap_bumpers = true;
  ASSERT_EQ(swapped.Start("fw_link", kLinkConfirm), "");
  swapped.RunToEnd(20);
  swapped.PassInput("fw_stop", {&Mower::press_stop, &Mower::press_stop});
  swapped.PassInput("fw_lift", {&Mower::press_lift});
  const size_t before = swapped.ends.size();
  ASSERT_EQ(swapped.Start("fw_bumper", nullptr), "");
  ASSERT_TRUE(swapped.WaitPart("fw_bumper", 1));
  swapped.Run(0.3);
  swapped.Press(&Mower::press_left);
  const json end = swapped.EndSince(before, 70);
  EXPECT_EQ(end.value("reason", json()), "WRONG_INPUT") << end.dump();
  EXPECT_EQ(end.value("seen", json()), "bumper_right") << end.dump();
  EXPECT_EQ(swapped.program.Export()["runs"][0]["result"], "failed");
}

TEST(Firmware, RatesWheelsAndTheOwnLimitOfTheBoardAreJudged) {
  FwMower slow;
  slow.link_hz["speed"] = 50;
  ASSERT_EQ(slow.Start("fw_link", kLinkConfirm), "");
  json end = slow.RunToEnd(20);
  EXPECT_EQ(end["reason"], "LIMIT") << end.dump();
  EXPECT_EQ(ValueOf(slow.StepState("fw_link"), "speed")["ok"], false) << slow.StepState("fw_link").dump();

  // 1.3 times the speed passes the wheels of the setup (1.5), not those of the measurement (1.2).
  FwMower fast;
  fast.wheel_scale = 1.3;
  ASSERT_TRUE(fast.PassRun().is_object());
  EXPECT_EQ(fast.StepState("fw_wheels")["state"], "failed");
  EXPECT_EQ(fast.ends.back()["reason"], "LIMIT") << fast.ends.back().dump();

  // The board keeps the setpoint longer than 4 s without its host.
  FwMower holds;
  holds.setpoint_hold = 6.0;
  holds.capabilities.silence_last_setpoint_max_s = 5.0;
  holds.program.OnCapabilities(holds.capabilities);
  end = holds.PassRun();
  ASSERT_TRUE(end.is_object());
  EXPECT_EQ(end["step"], "fw_setpoint") << end.dump();
  EXPECT_EQ(end["reason"], "LIMIT") << end.dump();
}

TEST(Firmware, ABumperThatStaysIsHeldTooLongOrFails) {
  const auto latch = [](double bumper) {
    FwMower mower;
    EXPECT_EQ(mower.Start("fw_link", kLinkConfirm), "");
    EXPECT_EQ(mower.RunToEnd(20)["state"], "ok");
    EXPECT_EQ(mower.PassInput("fw_stop", {&Mower::press_stop, &Mower::press_stop})["state"], "ok");
    EXPECT_EQ(mower.PassInput("fw_lift", {&Mower::press_lift})["state"], "ok");
    EXPECT_EQ(mower.PassInput("fw_bumper", {&Mower::press_left, &Mower::press_right})["state"], "ok");
    const json end = mower.PassLatch(bumper);
    return std::make_pair(end, mower.program.Export()["runs"][0]["result"]);
  };
  // Held past fall_max (1 s) but fallen: pressed too long, a fault of the user.
  auto [end, result] = latch(1.5);
  EXPECT_EQ(end["reason"], "NOT_DONE") << end.dump();
  EXPECT_EQ(end["part"], "bumper_left") << end.dump();
  EXPECT_EQ(result, "running");
  // Still on after time (10 s): the board holds it.
  std::tie(end, result) = latch(12);
  EXPECT_EQ(end["reason"], "LIMIT") << end.dump();
  EXPECT_EQ(end["part"], "bumper_left") << end.dump();
  EXPECT_EQ(result, "failed");
}

TEST(Firmware, AnInputThatBringsAnotherFails) {
  FwMower mower;
  ASSERT_EQ(mower.Start("fw_link", kLinkConfirm), "");
  ASSERT_EQ(mower.RunToEnd(20)["state"], "ok");
  ASSERT_EQ(mower.Start("fw_stop", nullptr), "");
  ASSERT_TRUE(mower.WaitPart("fw_stop", 1));
  mower.Run(0.3);
  const size_t before = mower.ends.size();
  mower.press_lift = true;
  mower.Press(&Mower::press_stop);
  mower.press_lift = false;
  const json end = mower.EndSince(before, 15);
  EXPECT_EQ(end["reason"], "WRONG_INPUT") << end.dump();
  EXPECT_EQ(end["seen"], "lift") << end.dump();
  EXPECT_EQ(mower.program.Export()["runs"][0]["result"], "failed");
}

// The signal tells the driver whether the running step confirmed the blade off the mower.
TEST(Firmware, TheSignalSaysWhetherTheBladeIsRemoved) {
  FwMower mower;
  ASSERT_EQ(mower.Start("fw_link", kLinkConfirm), "");
  ASSERT_TRUE(mower.program.MeasureNow());
  EXPECT_TRUE(mower.program.MeasureNow()->blade_removed);
  ASSERT_EQ(mower.RunToEnd(20)["state"], "ok");
  ASSERT_EQ(mower.Start("fw_stop", nullptr), "");
  ASSERT_TRUE(mower.program.MeasureNow());
  EXPECT_FALSE(mower.program.MeasureNow()->blade_removed);
}

// The other bumper latches the collision as well: the part fails at once, not after its time.
TEST(Firmware, TheOtherBumperFailsAtOnce) {
  FwMower mower;
  ASSERT_EQ(mower.Start("fw_link", kLinkConfirm), "");
  ASSERT_EQ(mower.RunToEnd(20)["state"], "ok");
  ASSERT_EQ(mower.PassInput("fw_stop", {&Mower::press_stop, &Mower::press_stop})["state"], "ok");
  ASSERT_EQ(mower.PassInput("fw_lift", {&Mower::press_lift})["state"], "ok");
  ASSERT_EQ(mower.Start("fw_bumper", nullptr), "");
  ASSERT_TRUE(mower.WaitPart("fw_bumper", 1));
  mower.Run(0.3);
  const size_t before = mower.ends.size();
  mower.Press(&Mower::press_right);
  const json end = mower.EndSince(before, 5);
  ASSERT_TRUE(end.is_object()) << mower.StepState("fw_bumper").dump();
  EXPECT_EQ(end["reason"], "WRONG_INPUT") << end.dump();
  EXPECT_EQ(end["seen"], "bumper_right") << end.dump();
}

TEST(Firmware, AnUnknownModuleAfterTheLinkFailsTheRun) {
  FwMower mower;
  mower.link.unknown[0x51] = 3;
  ASSERT_TRUE(mower.PassRun().is_null());
  // Frames of a module fw_link did not see.
  mower.link.unknown[0x52] = 1;
  mower.Run(1.5);
  ASSERT_EQ(mower.Start("fw_release", nullptr), "");
  mower.Run(0.2);
  const json end = mower.ends.back();
  EXPECT_EQ(end["reason"], "LIMIT") << end.dump();
  EXPECT_EQ(end["value_id"], "unknown_new") << end.dump();
  EXPECT_EQ(mower.program.Export()["runs"][0]["result"], "failed");
}

TEST(Firmware, AnotherFirmwareInTheRunAbortsIt) {
  FwMower mower;
  ASSERT_EQ(mower.Start("fw_link", kLinkConfirm), "");
  ASSERT_EQ(mower.RunToEnd(20)["state"], "ok");
  // A board the entry does not name: the run goes on.
  mower.capabilities.firmware = {{"cutter", "0.6.38"}, {"chassis", "0.6.34"}, {"bms", "1.0.1"}};
  mower.program.OnCapabilities(mower.capabilities);
  mower.Run(0.2);
  ASSERT_EQ(mower.program.Export()["runs"][0]["result"], "running");
  mower.capabilities.firmware = {{"cutter", "0.6.38"}, {"chassis", "0.6.39"}, {"bms", "1.0.1"}};
  mower.program.OnCapabilities(mower.capabilities);
  mower.Run(0.2);
  const json runs = mower.program.Export()["runs"];
  EXPECT_EQ(runs[0]["result"], "aborted");
  EXPECT_EQ(runs[0]["reason"], "FIRMWARE_CHANGED");
  EXPECT_EQ(mower.Fw()["failed"], 0);
}

// The firmware list with the step driver_end before the release.
std::string FwEndList() {
  json list = json::parse(FwList());
  const json end = {{"id", "fw_driver_end"},
                    {"stage", "firmware"},
                    {"firmware", true},
                    {"check", "driver_end"},
                    {"params", {{"speed", 0.1}, {"rounds", 2}, {"emergency", {"HIGH_LEVEL"}}}},
                    {"confirm", {"raised", "hand_at_stop"}}};
  list["steps"].insert(list["steps"].end() - 1, end);
  return list.dump();
}

// The new driver starts with the measurement closed: the step waits for it, opens again and releases.
TEST(Firmware, DriverEndOpensTheMeasurementAgainForTheNewDriver) {
  FwMower mower("none", FwEndList());
  const json run = mower.PassRun();
  ASSERT_TRUE(run.is_null()) << run.dump();
  mower.Run(3.0);
  ASSERT_EQ(mower.Start("fw_driver_end", json::array({"raised", "hand_at_stop"})), "")
      << mower.StepState("fw_driver_end")["blocked"].dump();
  const json end = mower.RunToEnd(60);
  EXPECT_EQ(end["state"], "ok") << end.dump();
  EXPECT_EQ(mower.hardware.emergency_reason, 0u);
}

TEST(Firmware, TheDriverEndingTheWindowEndsTheStep) {
  FwMower mower;
  ASSERT_EQ(mower.Start("fw_link", kLinkConfirm), "");
  ASSERT_EQ(mower.RunToEnd(20)["state"], "ok");
  ASSERT_EQ(mower.Start("fw_stop", nullptr), "");
  ASSERT_TRUE(mower.WaitPart("fw_stop", 1));
  // The guard of the driver latches the firmware again (a stall of its I/O).
  mower.Latch(Bit("FIRMWARE_INCOMPATIBLE"));
  const json end = mower.RunToEnd(1);
  EXPECT_EQ(end.value("reason", json()), "NOT_OPENED") << end.dump();
  // Not the board's fault: the run goes on and the step can start again.
  EXPECT_EQ(mower.program.Export()["runs"][0]["result"], "running");
  EXPECT_EQ(mower.PassInput("fw_stop", {&Mower::press_stop, &Mower::press_stop})["state"], "ok");
}
