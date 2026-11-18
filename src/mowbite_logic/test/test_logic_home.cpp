#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mowbite_logic/logic_core.h"
#include "mowbite_time/sun.h"
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

// A lane north of where the mower stands: it has to drive to its start first.
JobPath FarLane() {
  JobPath path;
  for (int i = 0; i < 5; ++i) {
    JobPose pose;
    pose.x = 1.0 + 0.5 * i;
    pose.y = 1.8;
    pose.kind = i == 0 ? kSegmentTransit : kSegmentLane;
    path.poses.push_back(pose);
  }
  return path;
}

const Reason kNoAnswer("PLANNER_TIMEOUT", "No answer from the planner in 10 s");
const Reason kNotRunning("PLANNER_MISSING", "The planner is not running");

// Mowing with the plan of the lawn, the planner asked for the way to the start of its lane.
TestCore Approaching(LogicConfig config = Tron(), bool with_dock = true) {
  TestCore core = Mowing(config, with_dock);
  core.OnJobPlanned({FarLane()}, "fp");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.TakeEvents();
  core.TakeNotices();
  core.TakeCheckpoint();
  return core;
}

// kPlannerRetries approaches in a row without an answer, each asked for again.
void StaySilent(TestCore& core, const Reason& silent = kNoAnswer) {
  for (int i = 1; i <= kPlannerRetries; i++) {
    core.OnLegEnded(LegEnd::kCanceled, 0, silent);
    EXPECT_EQ(core.RobotState().state, "MOWING") << i;
    ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach) << i;
  }
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

// Without a way home heat and rain pause the run with their reason (question 47 iii); the battery ends it, also from
// that pause.
TEST(GoingHome, WithoutAWayHomeHeatAndRainPauseTheBatteryEnds) {
  TestCore core = Mowing(Tron(), false);
  core.SetHomeSettings(1, 20, 60, 40);
  HardwareSnapshot hot = Healthy();
  hot.blade.temperature = 90;
  hot.rain = true;
  Hold(core, hot, 0, 60);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE"}));
  EXPECT_EQ(events[0].details["reason"], "MOTOR_HOT");
  EXPECT_EQ(events[0].details["message"], "It cannot go home: the blade motor at 90 °C, 60 °C or more");
  HardwareSnapshot low = hot;
  low.battery_voltage = 20.5f;
  Hold(core, low, 61, 81);
  EXPECT_TRUE(core.Idle());
  events = core.TakeEvents();
  EXPECT_FALSE(Find(events, "DOCKING"));
  const Event* battery = Find(events, "BATTERY_LOW");
  ASSERT_TRUE(battery);
  EXPECT_DOUBLE_EQ(battery->details["limit"].get<double>(), 21.0);

  // Without charging contacts: no docking, the same.
  LogicConfig bare = Tron();
  bare.dock_contact_x = 0.0;
  TestCore other = Mowing(bare);
  Hold(other, Volts(19.0f), 0, 2);
  EXPECT_TRUE(other.Idle());
  EXPECT_TRUE(Find(other.TakeEvents(), "BATTERY_LOW"));
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

TEST(SilentPlanner, AnApproachWithoutAnAnswerIsAskedForAgain) {
  TestCore core = Approaching();
  StaySilent(core);
  // Standing in MOWING with the blade off, nothing for the app yet.
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_FALSE(core.TakeDriveCancel());
  const auto notices = core.TakeNotices();
  ASSERT_EQ(notices.size(), 3u);
  EXPECT_EQ(notices[2],
            "Mowing: no answer from the planner (No answer from the planner in 10 s), asking again: 3 of 3");
  // The fourth question is answered: on with the path, without a pause.
  core.OnLegEnded(LegEnd::kDone, 0, Reason("DONE", "done"));
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  EXPECT_TRUE(core.TakeEvents().empty());
}

TEST(SilentPlanner, AfterThreeRetriesMowingEndsAndTheMowerGoesHome) {
  for (const Reason& silent : {kNoAnswer, kNotRunning}) {
    TestCore core = Approaching();
    StaySilent(core, silent);
    core.OnLegEnded(silent.code == "PLANNER_MISSING" ? LegEnd::kFailed : LegEnd::kCanceled, 0, silent);
    EXPECT_TRUE(core.Docking()) << silent.code;
    const auto events = core.TakeEvents();
    ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING", "STATE"})) << silent.code;
    const json& docking = events[0].details;
    EXPECT_EQ(docking["reason"], silent.code);
    EXPECT_EQ(docking["severity"], "warning") << silent.code;
    EXPECT_EQ(docking["resume"], "user") << silent.code;
    EXPECT_EQ(docking["message"], "Going home: the planner does not answer") << silent.code;
    EXPECT_EQ(events[1].details["state"], "DOCKING") << silent.code;
    // The job stays for the next start.
    const auto saved = ParseCheckpoint(*core.TakeCheckpoint());
    ASSERT_TRUE(saved) << silent.code;
    EXPECT_FALSE(saved->job_id.empty()) << silent.code;
    EXPECT_EQ(docking["job_id"], saved->job_id) << silent.code;
    EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone) << silent.code;
    EXPECT_TRUE(core.TakeDockingLeg()) << silent.code;
  }
}

TEST(SilentPlanner, AnAnswerBeginsTheCountAnew) {
  TestCore core = Approaching();
  StaySilent(core);
  // "No way" is an answer: the pause of a failed approach as before.
  core.OnLegEnded(LegEnd::kFailed, 0, Reason("APPROACH_FAILED", "Transit planning failed: no way"));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE"}));
  EXPECT_EQ(events[0].details["reason"], "APPROACH_FAILED");
  ASSERT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  StaySilent(core);
  core.OnLegEnded(LegEnd::kCanceled, 0, kNoAnswer);
  EXPECT_TRUE(core.Docking());
}

TEST(SilentPlanner, AnApproachTheLogicStoppedKeepsTheCount) {
  TestCore core = Approaching();
  StaySilent(core);
  ASSERT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
  const auto cancel = core.TakeDriveCancel();
  ASSERT_TRUE(cancel);
  core.OnLegEnded(LegEnd::kCanceled, 0, *cancel);
  ASSERT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.OnLegEnded(LegEnd::kCanceled, 0, kNoAnswer);
  EXPECT_TRUE(core.Docking());
}

TEST(SilentPlanner, NeverHomeFromAPauseThatWaitsForTheUser) {
  TestCore core = Approaching();
  StaySilent(core);
  // The user pauses while the fourth question is out, and its end without an answer comes after that.
  ASSERT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
  core.TakeEvents();
  core.OnLegEnded(LegEnd::kCanceled, 0, kNoAnswer);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  // With continue the job asks once more.
  ASSERT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.OnLegEnded(LegEnd::kCanceled, 0, kNoAnswer);
  EXPECT_TRUE(core.Docking());
}

TEST(SilentPlanner, WithoutAWayHomeMowingPausesAsForAFailedApproach) {
  TestCore core = Approaching(Tron(), false);
  StaySilent(core);
  core.OnLegEnded(LegEnd::kCanceled, 0, kNoAnswer);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE"}));
  EXPECT_EQ(events[0].details["state"], "PAUSED");
  EXPECT_EQ(events[0].details["reason"], "APPROACH_FAILED");
  EXPECT_EQ(events[0].details["severity"], "error");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_EQ(events[0].details["message"], "No answer from the planner in 10 s");
  // The job goes on with continue; a planner that stays silent pauses it again at the next question.
  ASSERT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.OnLegEnded(LegEnd::kCanceled, 0, kNoAnswer);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_TRUE(core.Mowing());
}

TEST(SilentPlanner, APlanThatTimesOutSendsTheMowerHomeAtOnce) {
  const Reason late("PLANNER_TIMEOUT", "No answer from the planner in 120 s");
  TestCore core = Mowing();
  core.OnJobPlanFailed(late);
  EXPECT_TRUE(core.Docking());
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING", "STATE"}));
  EXPECT_EQ(events[0].details["reason"], "PLANNER_TIMEOUT");
  EXPECT_EQ(events[0].details["message"], "Going home: the planner does not answer");

  // Without a way home: the pause of a failed plan.
  TestCore stays = Mowing(Tron(), false);
  stays.OnJobPlanFailed(late);
  EXPECT_EQ(stays.RobotState().state, "PAUSED");
  const auto paused = stays.TakeEvents();
  ASSERT_EQ(Types(paused), (std::vector<std::string>{"STATE"}));
  EXPECT_EQ(paused[0].details["reason"], "PLANNING_FAILED");
  EXPECT_EQ(paused[0].details["message"], "Planning failed: No answer from the planner in 120 s");
}

TEST(SilentPlanner, APlannerThatIsNotRunningIsAskedForThePlanAgain) {
  TestCore core = Mowing();
  for (int i = 1; i <= kPlannerRetries; i++) {
    core.OnJobPlanFailed(kNotRunning);
    EXPECT_EQ(core.RobotState().state, "MOWING") << i;
    ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan) << i;
  }
  EXPECT_TRUE(core.TakeEvents().empty());
  // Its plan counts as an answer: three more questions before mowing ends.
  core.OnJobPlanned({FarLane()}, "fp");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  StaySilent(core, kNotRunning);
  core.OnLegEnded(LegEnd::kFailed, 0, kNotRunning);
  EXPECT_TRUE(core.Docking());
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING", "STATE"}));
  EXPECT_EQ(events[0].details["reason"], "PLANNER_MISSING");

  // A plan that never comes ends mowing like the approach.
  TestCore never = Mowing();
  for (int i = 1; i <= kPlannerRetries; i++) {
    never.OnJobPlanFailed(kNotRunning);
    ASSERT_EQ(never.NextJobStep(0).kind, JobStep::Kind::kPlan) << i;
  }
  never.OnJobPlanFailed(kNotRunning);
  EXPECT_TRUE(never.Docking());
}

// A plan the planner refused is an answer: the pause as before, and the count begins anew.
TEST(SilentPlanner, ARefusedPlanPausesAsBefore) {
  TestCore core = Mowing();
  for (int i = 1; i <= kPlannerRetries; i++) {
    core.OnJobPlanFailed(kNotRunning);
    ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan) << i;
  }
  core.OnJobPlanFailed(Reason("PLANNING_FAILED", "no lanes fit"));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE"}));
  EXPECT_EQ(events[0].details["reason"], "PLANNING_FAILED");
  EXPECT_EQ(events[0].details["message"], "Planning failed: no lanes fit");
  ASSERT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanFailed(kNotRunning);
  EXPECT_EQ(core.RobotState().state, "MOWING");
}

namespace {

// Berlin, and 2026-11-12 12:00 UTC.
constexpr double kLat = 52.52;
constexpr double kLon = 13.405;
constexpr double kNoon = 1794484800.0;

double SunsetAfter(double unix_s) {
  for (const double day : {unix_s, unix_s + 86400.0}) {
    const auto set = mowbite_time::SunTimes(kLat, kLon, day).set;
    if (set && *set > unix_s) return *set;
  }
  return 0;
}

// The wall clock this many s before today's sunset at monotonic 0.
double Before(double s) {
  return SunsetAfter(kNoon) - s;
}

// Ready with the switch on, the datum and a safe clock whose wall time at monotonic 0 is unix_s.
TestCore ReadyForTheSun(double unix_s, bool with_dock = true) {
  TestCore core = Ready(Tron(), with_dock);
  core.SetSunsetSettings(true);
  core.OnDatum(kLat, kLon);
  core.OnClock(true, "", unix_s);
  return core;
}

void Start(TestCore& core) {
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.TakeEvents();
  core.TakeNotices();
}

int CountOf(const std::vector<Event>& events, const std::string& type) {
  return static_cast<int>(std::count_if(events.begin(), events.end(), [&](const Event& e) { return e.type == type; }));
}

int Lines(const std::vector<std::string>& notices, const std::string& part) {
  return static_cast<int>(std::count_if(notices.begin(), notices.end(),
                                        [&](const std::string& n) { return n.find(part) != std::string::npos; }));
}

}  // namespace

TEST(Sunset, TheFirstSunsetAfterTheStartSendsItHomeOnce) {
  TestCore core = ReadyForTheSun(Before(100));
  Start(core);
  Hold(core, Healthy(), 0, 99);
  EXPECT_TRUE(core.Mowing());
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
  core.OnTick(100);
  EXPECT_TRUE(core.Docking());
  const auto events = core.TakeEvents();
  ASSERT_EQ(CountOf(events, "DOCKING"), 1);
  const Event* docking = Find(events, "DOCKING");
  EXPECT_EQ(docking->details["reason"], "DARK");
  EXPECT_EQ(docking->details["severity"], "info");
  EXPECT_NEAR(docking->details["sun_elevation"].get<double>(), mowbite_time::kSunsetElevation, 0.05);
  EXPECT_DOUBLE_EQ(docking->details["limit"].get<double>(), mowbite_time::kSunsetElevation);
  EXPECT_EQ(Lines(core.TakeNotices(), "sunset"), 0);
}

TEST(Sunset, AStartInTheDarkMowsOnUntilTheNextEvening) {
  const double start = Before(-3600);
  TestCore core = ReadyForTheSun(start);
  Start(core);
  Hold(core, Healthy(), 0, 600);
  const double due = SunsetAfter(start) - start;
  EXPECT_GT(due, 20 * 3600.0);
  Hold(core, Healthy(), due - 2, due - 1);
  EXPECT_TRUE(core.Mowing());
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
  core.OnTick(due);
  EXPECT_EQ(Find(core.TakeEvents(), "DOCKING")->details["reason"], "DARK");
}

TEST(Sunset, APauseByHandOverTheSunsetThenContinueMowsOn) {
  TestCore core = ReadyForTheSun(Before(100));
  Start(core);
  Hold(core, Healthy(), 0, 50);
  ASSERT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
  Hold(core, Healthy(), 51, 200);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  core.TakeEvents();
  ASSERT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  Hold(core, Healthy(), 201, 400);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
}

TEST(Sunset, NothingWithTheSwitchOff) {
  TestCore core = ReadyForTheSun(Before(100));
  core.SetSunsetSettings(false);
  Start(core);
  Hold(core, Healthy(), 0, 200);
  EXPECT_TRUE(core.Mowing());
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
  EXPECT_EQ(Lines(core.TakeNotices(), "sunset"), 0);

  // The core starts with it off: a journal without the switch replays as before.
  TestCore fresh = Mowing();
  fresh.OnDatum(kLat, kLon);
  fresh.OnClock(true, "", Before(100));
  Hold(fresh, Healthy(), 0, 200);
  EXPECT_TRUE(fresh.Mowing());
  EXPECT_EQ(Lines(fresh.TakeNotices(), "sunset"), 0);
}

TEST(Sunset, TheSwitchActsWithinTheRun) {
  TestCore off = ReadyForTheSun(Before(100));
  Start(off);
  Hold(off, Healthy(), 0, 50);
  off.SetSunsetSettings(false);
  Hold(off, Healthy(), 51, 200);
  EXPECT_TRUE(off.Mowing());
  EXPECT_FALSE(Find(off.TakeEvents(), "DOCKING"));

  TestCore on = ReadyForTheSun(Before(100));
  on.SetSunsetSettings(false);
  Start(on);
  Hold(on, Healthy(), 0, 50);
  on.SetSunsetSettings(true);
  Hold(on, Healthy(), 51, 99);
  EXPECT_TRUE(on.Mowing());
  on.OnTick(100);
  const auto events = on.TakeEvents();
  const Event* docking = Find(events, "DOCKING");
  ASSERT_NE(docking, nullptr);
  EXPECT_EQ(docking->details["reason"], "DARK");
}

TEST(Sunset, AnUnsafeClockIsToldOnceAndASafeOneSetsTheDeadline) {
  TestCore core = ReadyForTheSun(Before(100));
  core.OnClock(false, "never_synced", Before(100));
  Start(core);
  core.OnTick(0);
  auto notices = core.TakeNotices();
  EXPECT_EQ(Lines(notices, "No going home at sunset yet: the clock is not safe (never_synced)"), 1);
  core.OnClock(false, "clock_jump", Before(100));
  Hold(core, Healthy(), 1, 50);
  EXPECT_EQ(Lines(core.TakeNotices(), "sunset"), 0);
  core.OnClock(true, "", Before(100));
  Hold(core, Healthy(), 51, 99);
  EXPECT_EQ(Lines(core.TakeNotices(), "Going home at sunset after all"), 1);
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
  core.OnTick(100);
  EXPECT_EQ(Find(core.TakeEvents(), "DOCKING")->details["reason"], "DARK");

  // Unsafe the whole run: no DARK.
  TestCore never = ReadyForTheSun(Before(100));
  never.OnClock(false, "gnss_mismatch", Before(100));
  Start(never);
  Hold(never, Healthy(), 0, 300);
  EXPECT_TRUE(never.Mowing());
  EXPECT_FALSE(Find(never.TakeEvents(), "DOCKING"));
  EXPECT_EQ(Lines(never.TakeNotices(), "No going home at sunset yet"), 1);
}

TEST(Sunset, NoDatumNoDeadline) {
  TestCore core = Ready();
  core.SetSunsetSettings(true);
  core.OnClock(true, "", Before(100));
  Start(core);
  Hold(core, Healthy(), 0, 300);
  EXPECT_TRUE(core.Mowing());
  EXPECT_EQ(Lines(core.TakeNotices(), "No going home at sunset yet: no datum"), 1);
}

TEST(Sunset, RainBeforeDarkAndOneDocking) {
  TestCore core = ReadyForTheSun(Before(20));
  core.SetHomeSettings(1, 20, 70, 40);
  Start(core);
  HardwareSnapshot wet = Healthy();
  wet.rain = true;
  Hold(core, wet, 0, 20);
  const auto events = core.TakeEvents();
  ASSERT_EQ(CountOf(events, "DOCKING"), 1);
  EXPECT_EQ(Find(events, "DOCKING")->details["reason"], "RAIN");
}

TEST(Sunset, AJumpOfTheWallClockKeepsTheDeadline) {
  TestCore core = ReadyForTheSun(Before(100));
  Start(core);
  Hold(core, Healthy(), 0, 50);
  core.OnClock(true, "", Before(100) + 3600);
  Hold(core, Healthy(), 51, 99);
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
  core.OnClock(true, "", Before(100) - 3600);
  core.OnTick(100);
  EXPECT_EQ(Find(core.TakeEvents(), "DOCKING")->details["reason"], "DARK");
}

// Without a way home every reason but the battery pauses the run with it; the drive stops and the blade with it.
TEST(Sunset, WithoutAWayHomeEachReasonPauses) {
  for (const std::string reason : {"RAIN", "MOTOR_HOT", "BATTERY_HOT", "DARK"}) {
    LogicConfig config = Tron();
    config.battery_hot_temperature = 55;
    TestCore core = Mowing(config, false);
    core.SetSunsetSettings(true);
    core.OnDatum(kLat, kLon);
    core.OnClock(true, "", reason == "DARK" ? Before(30) : Before(-3600));
    core.SetHomeSettings(1, 20, 70, 40);
    core.OnJobPlanned({FarLane()}, "fp");
    ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach) << reason;
    core.TakeEvents();
    core.TakeDriveCancel();
    HardwareSnapshot hw = Healthy();
    hw.rain = reason == "RAIN";
    if (reason == "MOTOR_HOT") hw.blade.temperature = 90;
    if (reason == "BATTERY_HOT") hw.battery_temperature = 60;
    Hold(core, hw, 0, 40);
    EXPECT_EQ(core.RobotState().state, "PAUSED") << reason;
    EXPECT_TRUE(core.Mowing()) << reason;
    EXPECT_FALSE(core.BladeWanted()) << reason;
    EXPECT_TRUE(core.TakeDriveCancel()) << reason;
    const auto events = core.TakeEvents();
    ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE"})) << reason;
    EXPECT_EQ(events[0].details["state"], "PAUSED") << reason;
    EXPECT_EQ(events[0].details["reason"], reason) << reason;
    EXPECT_EQ(events[0].details["resume"], "user") << reason;
    EXPECT_EQ(events[0].details["message"].get<std::string>().rfind("It cannot go home: ", 0), 0u) << reason;
  }
}

TEST(Sunset, ContinueInTheRainWithoutAWayHomeIsRefusedUntilItStops) {
  TestCore core = Mowing(Tron(), false);
  core.SetHomeSettings(1, 20, 70, 40);
  HardwareSnapshot wet = Healthy();
  wet.rain = true;
  Hold(core, wet, 0, 25);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  core.TakeEvents();
  const CommandOutcome refused = core.Command("mowing.continue", json::object());
  EXPECT_EQ(refused.why.code, "RAIN");
  EXPECT_EQ(refused.why.message, "Not while it cannot go home: rain for 20 s");
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"MOWING_REFUSED"}));
  EXPECT_EQ(events[0].details["reason"], "RAIN");
  Hold(core, Healthy(), 26, 27);
  ASSERT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().state, "MOWING");
}

TEST(Sunset, ContinueAfterTheDarkPauseMowsOn) {
  TestCore core = ReadyForTheSun(Before(30), false);
  Start(core);
  Hold(core, Healthy(), 0, 40);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(Find(core.TakeEvents(), "STATE")->details["reason"], "DARK");
  ASSERT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  Hold(core, Healthy(), 41, 600);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  const auto events = core.TakeEvents();
  EXPECT_EQ(CountOf(events, "STATE"), 1);
  EXPECT_EQ(Find(events, "STATE")->details["state"], "MOWING");
}

TEST(Sunset, APauseForGpsWithRainAndNoWayHomeTurnsIntoTheRainPauseAsItEnds) {
  TestCore core = Mowing(Tron(), false);
  core.SetHomeSettings(1, 20, 70, 40);
  core.OnPoseTimeout();
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  core.TakeEvents();
  HardwareSnapshot wet = Healthy();
  wet.rain = true;
  Hold(core, wet, 0, 30);
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnPose(PoseAt(2.0, 0.5));
  Hold(core, wet, 31, 45);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE", "STATE"}));
  EXPECT_EQ(events[0].details["state"], "MOWING");
  EXPECT_EQ(events[1].details["state"], "PAUSED");
  EXPECT_EQ(events[1].details["reason"], "RAIN");
}

TEST(Sunset, ContinueFromAManualPauseInTheRainGoesHomeInTheSameCall) {
  TestCore core = Mowing();
  core.SetHomeSettings(1, 20, 70, 40);
  ASSERT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
  HardwareSnapshot wet = Healthy();
  wet.rain = true;
  Hold(core, wet, 0, 30);
  core.TakeEvents();
  ASSERT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  EXPECT_TRUE(core.Docking());
  EXPECT_EQ(Find(core.TakeEvents(), "DOCKING")->details["reason"], "RAIN");
}

// A new run watches the battery from its own start: a low voltage of the run before does not count.
TEST(GoingHome, ANewRunWatchesTheBatteryAnew) {
  TestCore core = Mowing(Tron(), false);
  Hold(core, Volts(19.5f), 0, 1);
  core.OnHardwareStatus(Healthy());
  ASSERT_EQ(core.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  ASSERT_TRUE(core.Idle());
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  core.TakeEvents();
  Hold(core, Volts(19.5f), 1.5, 2.5);
  EXPECT_TRUE(core.Mowing());
  EXPECT_FALSE(Find(core.TakeEvents(), "BATTERY_LOW"));
}

// Going home changes the actions, also where the reason against a start stays BUSY.
TEST(GoingHome, TheWayHomeChangesTheActions) {
  TestCore core = Mowing();
  core.TakeActionsChanged();
  Hold(core, Volts(19.5f), 0, 2);
  ASSERT_TRUE(core.Docking());
  EXPECT_EQ(core.StartProblem().code, "BUSY");
  EXPECT_TRUE(core.TakeActionsChanged());
}

// Outside of mowing the ticks do not watch the battery: no BATTERY_LOW in IDLE.
TEST(GoingHome, NothingWhileIdle) {
  TestCore core = Ready(Tron(), false);
  Hold(core, Volts(19.0f), 0, 30);
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(Find(core.TakeEvents(), "BATTERY_LOW"));
}

// A new run reckons its own sunset: the one the run before would have gone home at does not end it.
TEST(Sunset, ANewRunAfterTheSunsetMowsOnUntilTheNextEvening) {
  TestCore core = ReadyForTheSun(Before(100), false);
  Start(core);
  Hold(core, Healthy(), 0, 50);
  ASSERT_EQ(core.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  ASSERT_TRUE(core.Idle());
  Start(core);
  Hold(core, Healthy(), 200, 300);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_EQ(CountOf(core.TakeEvents(), "STATE"), 0);
}

// Each run tells once that it cannot reckon the sunset yet.
TEST(Sunset, EachRunTellsTheUnsafeClockOnce) {
  TestCore core = ReadyForTheSun(Before(100), false);
  core.OnClock(false, "never_synced", Before(100));
  Start(core);
  Hold(core, Healthy(), 0, 10);
  EXPECT_EQ(Lines(core.TakeNotices(), "No going home at sunset yet"), 1);
  ASSERT_EQ(core.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  ASSERT_TRUE(core.Idle());
  Start(core);
  Hold(core, Healthy(), 11, 20);
  EXPECT_EQ(Lines(core.TakeNotices(), "No going home at sunset yet"), 1);
}

TEST(GoingHome, NeverFromAHoldForAnObstacle) {
  HardwareSnapshot wet = Healthy();
  wet.rain = true;
  TestCore core = Mowing();
  core.SetHomeSettings(1, 20, 70, 40);
  core.SetObstacleMode(2, 2);
  core.OnObstacleStatus({true, {{1, "PERSON", true, false, 0.3, 2, 0.0, 0.0, 0.0, 0.0}}}, 0);
  ASSERT_EQ(core.RobotState().state, "PAUSED");
  core.TakeEvents();
  // A person may stand at the mower (G45): it keeps its place as in a pause for the user.
  Hold(core, wet, 0, 120);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING"));
}
