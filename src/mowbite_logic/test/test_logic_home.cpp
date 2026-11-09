#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

const std::string kStart = "mower_logic:idle/start_mowing";
const std::string kMowing = "mower_logic:mowing/";

// The station of the simulation and a lawn that reaches to 0.5 m in front of it, as in test_logic_docking.cpp.
const DockPose kDock{-1.0, 0.0, M_PI};
const Ring kLawn{{-0.5, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.5, 2.0}};

HardwareSnapshot Healthy() {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  hw.battery_temperature = 25;
  hw.blade.ok = true;
  hw.blade.temperature = 30;
  hw.left.ok = true;
  hw.right.ok = true;
  return hw;
}

HardwareSnapshot Volts(float voltage) {
  HardwareSnapshot hw = Healthy();
  hw.battery_voltage = voltage;
  return hw;
}

PoseSnapshot PoseAt(double x, double y, float accuracy = 0.02f, bool valid = true) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.heading = M_PI;
  pose.position_accuracy = accuracy;
  pose.orientation_accuracy = valid ? 0.01f : 999.0f;
  pose.orientation_valid = valid;
  return pose;
}

LogicConfig Tron() {
  LogicConfig config;
  config.dock_contact_x = -0.10;
  return config;
}

// IDLE in the field 2 m east of the station; with_dock: the station in the map.
TestCore Ready(LogicConfig config = Tron(), bool with_dock = true) {
  TestCore core{config};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(PoseAt(2.0, 0.5));
  core.OnMowMap({{"lawn", "Lawn", true}}, Reason());
  if (with_dock) core.OnMapDock(kDock);
  core.OnDrivableAreas({kLawn}, {});
  core.TakeEvents();
  core.TakeNotices();
  core.TakeActionsChanged();
  return core;
}

// Mowing, planning the first area.
TestCore Mowing(LogicConfig config = Tron(), bool with_dock = true) {
  TestCore core = Ready(config, with_dock);
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.TakeEvents();
  core.TakeNotices();
  return core;
}

std::vector<std::string> Types(const std::vector<Event>& events) {
  std::vector<std::string> types;
  for (const auto& e : events) types.push_back(e.type);
  return types;
}

const Event* Find(const std::vector<Event>& events, const std::string& type) {
  auto it = std::find_if(events.begin(), events.end(), [&](const Event& e) { return e.type == type; });
  return it == events.end() ? nullptr : &*it;
}

// Ticks every second from `from` up to `to`, with hw.
void Hold(TestCore& core, const HardwareSnapshot& hw, double from, double to) {
  core.OnHardwareStatus(hw);
  for (double t = from; t <= to; t += 1) core.OnTick(t);
}

}  // namespace

TEST(GoingHome, BatteryLowSendsTheMowerHome) {
  TestCore core = Mowing();
  Hold(core, Volts(20.8f), 0, 19);
  EXPECT_TRUE(core.Mowing());
  core.OnTick(20);
  EXPECT_TRUE(core.Docking());
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING", "STATE"}));
  const json& docking = events[0].details;
  EXPECT_EQ(docking["reason"], "BATTERY_LOW");
  EXPECT_EQ(docking["severity"], "info");
  EXPECT_EQ(docking["resume"], "user");
  EXPECT_NEAR(docking["battery_voltage"].get<double>(), 20.8, 1e-5);
  EXPECT_DOUBLE_EQ(docking["limit"].get<double>(), 21.0);
  EXPECT_EQ(docking["message"], "Going home: the battery stayed below 21.0 V for 20 s (at most 20.8 V)");
  EXPECT_EQ(events[1].details["state"], "DOCKING");
}

TEST(GoingHome, CriticalComesFirstAndAfterTwoSeconds) {
  TestCore core = Mowing();
  HardwareSnapshot hw = Volts(19.5f);
  hw.blade.temperature = 90;
  Hold(core, hw, 0, 1);
  EXPECT_TRUE(core.Mowing());
  core.OnTick(2);
  const auto taken = core.TakeEvents();
  const Event* docking = Find(taken, "DOCKING");
  ASSERT_TRUE(docking);
  EXPECT_EQ(docking->details["reason"], "BATTERY_CRITICAL");
  EXPECT_EQ(docking->details["severity"], "warning");
  EXPECT_NEAR(docking->details["battery_voltage"].get<double>(), 19.5, 1e-5);
  EXPECT_DOUBLE_EQ(docking->details["limit"].get<double>(), 19.7);
}

TEST(GoingHome, AHotBladeMotorSendsItHome) {
  TestCore core = Mowing();
  core.SetHomeSettings(0, 20, 60, 40);
  HardwareSnapshot hw = Healthy();
  hw.blade.temperature = 61;
  Hold(core, hw, 0, 2);
  EXPECT_TRUE(core.Mowing());
  core.OnTick(3);
  const auto taken = core.TakeEvents();
  const Event* docking = Find(taken, "DOCKING");
  ASSERT_TRUE(docking);
  EXPECT_EQ(docking->details["reason"], "MOTOR_HOT");
  EXPECT_EQ(docking->details["severity"], "warning");
  EXPECT_EQ(docking->details["temperature"], 61);
  EXPECT_DOUBLE_EQ(docking->details["limit"].get<double>(), 60.0);
}

TEST(GoingHome, RainOnlyWithARainMode) {
  HardwareSnapshot wet = Healthy();
  wet.rain = true;
  TestCore off = Mowing();
  Hold(off, wet, 0, 120);
  EXPECT_TRUE(off.Mowing());
  EXPECT_FALSE(Find(off.TakeEvents(), "DOCKING"));

  TestCore core = Mowing();
  core.SetHomeSettings(1, 20, 70, 40);
  Hold(core, wet, 0, 19);
  EXPECT_TRUE(core.Mowing());
  core.OnTick(20);
  const auto taken = core.TakeEvents();
  const Event* docking = Find(taken, "DOCKING");
  ASSERT_TRUE(docking);
  EXPECT_EQ(docking->details["reason"], "RAIN");
  EXPECT_EQ(docking->details["severity"], "info");
}

TEST(GoingHome, BatteryHotOnlyWithALimit) {
  HardwareSnapshot hot = Healthy();
  hot.battery_temperature = 60;
  TestCore off = Mowing();
  Hold(off, hot, 0, 30);
  EXPECT_TRUE(off.Mowing());

  LogicConfig config = Tron();
  config.battery_hot_temperature = 55;
  TestCore core = Mowing(config);
  Hold(core, hot, 0, 3);
  const auto taken = core.TakeEvents();
  const Event* docking = Find(taken, "DOCKING");
  ASSERT_TRUE(docking);
  EXPECT_EQ(docking->details["reason"], "BATTERY_HOT");
  EXPECT_EQ(docking->details["temperature"], 60);
  EXPECT_DOUBLE_EQ(docking->details["limit"].get<double>(), 55.0);
}

TEST(GoingHome, AlsoWhileWaitingForAGoodPosition) {
  TestCore core = Mowing();
  core.OnPoseTimeout();
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  core.TakeEvents();
  Hold(core, Volts(19.0f), 0, 2);
  EXPECT_TRUE(core.Docking());
  EXPECT_EQ(Find(core.TakeEvents(), "DOCKING")->details["reason"], "BATTERY_CRITICAL");
}

// A pause that waits for the user never goes home: only the battery ends it, as BATTERY_LOW and IDLE.
TEST(GoingHome, NeverFromAPauseThatWaitsForTheUser) {
  for (const char* how : {"manual", "emergency"}) {
    TestCore core = Mowing();
    core.SetHomeSettings(1, 20, 60, 40);
    if (std::string(how) == "manual") {
      ASSERT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
    } else {
      HardwareSnapshot stop = Healthy();
      stop.emergency = true;
      core.OnHardwareStatus(stop);
    }
    EXPECT_EQ(core.RobotState().state, "PAUSED") << how;
    core.TakeEvents();
    HardwareSnapshot hot = Healthy();
    hot.emergency = std::string(how) == "emergency";
    hot.blade.temperature = 90;
    hot.rain = true;
    Hold(core, hot, 0, 60);
    EXPECT_EQ(core.RobotState().state, "PAUSED") << how;
    EXPECT_TRUE(core.TakeEvents().empty()) << how;
    HardwareSnapshot low = hot;
    low.battery_voltage = 19.0f;
    Hold(core, low, 61, 63);
    EXPECT_TRUE(core.Idle()) << how;
    const auto events = core.TakeEvents();
    EXPECT_FALSE(Find(events, "DOCKING")) << how;
    const Event* battery = Find(events, "BATTERY_LOW");
    ASSERT_TRUE(battery) << how;
    EXPECT_DOUBLE_EQ(battery->details["limit"].get<double>(), 19.7);
  }
}

// Without a way home only the battery ends mowing; heat and rain change nothing yet (question 47 iii).
TEST(GoingHome, WithoutAWayHomeOnlyTheBattery) {
  TestCore core = Mowing(Tron(), false);
  core.SetHomeSettings(1, 20, 60, 40);
  HardwareSnapshot hot = Healthy();
  hot.blade.temperature = 90;
  hot.rain = true;
  Hold(core, hot, 0, 60);
  EXPECT_TRUE(core.Mowing());
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
  const auto notices = core.TakeNotices();
  EXPECT_EQ(std::count_if(notices.begin(), notices.end(),
                          [](const std::string& n) { return n.find("cannot go home") != std::string::npos; }),
            1);
  HardwareSnapshot low = hot;
  low.battery_voltage = 20.5f;
  Hold(core, low, 61, 81);
  EXPECT_TRUE(core.Idle());
  const auto events = core.TakeEvents();
  EXPECT_FALSE(Find(events, "DOCKING"));
  const Event* battery = Find(events, "BATTERY_LOW");
  ASSERT_TRUE(battery);
  EXPECT_DOUBLE_EQ(battery->details["limit"].get<double>(), 21.0);

  // Contacts in front: no docking yet, the same.
  LogicConfig front = Tron();
  front.dock_contact_x = 0.3;
  TestCore yf = Mowing(front);
  Hold(yf, Volts(19.0f), 0, 2);
  EXPECT_TRUE(yf.Idle());
  EXPECT_TRUE(Find(yf.TakeEvents(), "BATTERY_LOW"));
}

TEST(GoingHome, StartRefusedLowHotOrWarm) {
  TestCore core = Ready();
  core.OnHardwareStatus(Volts(20.9f));
  Reason why = core.StartProblem();
  EXPECT_EQ(why.code, "BATTERY_LOW");
  EXPECT_DOUBLE_EQ(why.fields["limit"].get<double>(), 21.0);

  HardwareSnapshot warm = Healthy();
  warm.blade.temperature = 40;
  core.OnHardwareStatus(warm);
  why = core.StartProblem();
  EXPECT_EQ(why.code, "MOTOR_HOT");
  EXPECT_EQ(why.fields["temperature"], 40);
  EXPECT_DOUBLE_EQ(why.fields["limit"].get<double>(), 40.0);
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kRejected);
  const auto taken = core.TakeEvents();
  const Event* refused = Find(taken, "MOWING_REFUSED");
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->details["reason"], "MOTOR_HOT");
  EXPECT_EQ(refused->details["severity"], "warning");
  warm.blade.temperature = 39;
  core.OnHardwareStatus(warm);
  EXPECT_TRUE(core.StartProblem().empty()) << core.StartProblem().code;

  // The battery temperature only with a limit.
  HardwareSnapshot hot = Healthy();
  hot.battery_temperature = 60;
  core.OnHardwareStatus(hot);
  EXPECT_TRUE(core.StartProblem().empty());
  LogicConfig config = Tron();
  config.battery_hot_temperature = 55;
  TestCore limited = Ready(config);
  limited.OnHardwareStatus(hot);
  EXPECT_EQ(limited.StartProblem().code, "BATTERY_HOT");
  EXPECT_DOUBLE_EQ(limited.StartProblem().fields["limit"].get<double>(), 55.0);
}

TEST(GoingHome, DockingStartAndAbortCarryManualAndJobCompleteNone) {
  TestCore core = Ready();
  ASSERT_TRUE(core.StartDocking().empty());
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING", "STATE"}));
  EXPECT_EQ(events[0].details["reason"], "MANUAL");
  EXPECT_EQ(events[0].details["message"], "Going home: sent by the app");

  // abort_mowing without a station: IDLE, no DOCKING.
  TestCore lost = Mowing(Tron(), false);
  ASSERT_EQ(lost.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  EXPECT_TRUE(lost.Idle());
  EXPECT_FALSE(Find(lost.TakeEvents(), "DOCKING"));
}

TEST(GoingHome, DockingFailsWithTheReasonOfThePosition) {
  struct Case {
    const char* code;
    std::optional<PoseSnapshot> pose;
  };
  const Case cases[] = {{"NO_POSITION", std::nullopt},
                        {"NO_HEADING", PoseAt(2.0, 0.5, 0.02f, false)},
                        {"POSITION_ACCURACY", PoseAt(2.0, 0.5, 0.4f)}};
  for (const auto& c : cases) {
    TestCore core = Ready();
    ASSERT_TRUE(core.StartDocking().empty());
    ASSERT_TRUE(core.TakeDockingLeg());
    if (c.pose) {
      core.OnPose(*c.pose);
    } else {
      core.OnPoseTimeout();
    }
    core.OnDockingLegEnded(LegEnd::kGps, Reason("POSE_LOST", "bad"));
    core.TakeEvents();
    core.OnTick(0);
    core.OnTick(10);
    EXPECT_TRUE(core.Idle()) << c.code;
    const auto taken = core.TakeEvents();
    const Event* failed = Find(taken, "DOCKING_FAILED");
    ASSERT_TRUE(failed) << c.code;
    EXPECT_EQ(failed->details["reason"], c.code);
    EXPECT_EQ(failed->details["severity"], "error");
    EXPECT_EQ(failed->details["resume"], "user");
    EXPECT_EQ(failed->details["attempts"], 1);
    if (std::string(c.code) == "POSITION_ACCURACY") {
      EXPECT_NEAR(failed->details["accuracy"].get<double>(), 0.4, 1e-6);
      EXPECT_DOUBLE_EQ(failed->details["limit"].get<double>(), 0.1);
    }
  }
}

TEST(GoingHome, BatteryVoltagesInOrder) {
  LogicConfig config;
  EXPECT_EQ(BatteryVoltagesProblem(config), "");
  config.battery_critical_voltage = 21.0;
  EXPECT_NE(BatteryVoltagesProblem(config), "");
  config = LogicConfig();
  config.battery_empty_voltage = 25.2;
  EXPECT_NE(BatteryVoltagesProblem(config), "");
  config = LogicConfig();
  config.battery_critical_voltage = 0;
  EXPECT_NE(BatteryVoltagesProblem(config), "");
}
