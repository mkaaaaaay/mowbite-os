#include <gtest/gtest.h>

#include <algorithm>
#include <functional>

#include "mowbite_logic/logic_core.h"

using namespace mowbite_logic;

namespace {

const std::string kStart = "mower_logic:idle/start_mowing";
const std::string kMowing = "mower_logic:mowing/";

HardwareSnapshot Healthy() {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  hw.blade.ok = true;
  hw.left.ok = true;
  hw.right.ok = true;
  return hw;
}

PoseSnapshot PoseAt(double x, double y, float accuracy = 0.02f) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.position_accuracy = accuracy;
  pose.orientation_accuracy = 0.01f;
  pose.orientation_valid = true;
  return pose;
}

std::vector<JobArea> Garden() {
  return {{"front", "Front", true}, {"bed", "Bed", false}, {"back", "Back", true}};
}

// A path along y = 2 from x0, 5 poses 0.5 m apart: a transit pose, then lane.
JobPath Lane(double x0) {
  JobPath path;
  for (int i = 0; i < 5; ++i) {
    JobPose pose;
    pose.x = x0 + 0.5 * i;
    pose.y = 2;
    pose.kind = i == 0 ? kSegmentTransit : kSegmentLane;
    path.poses.push_back(pose);
  }
  return path;
}

LogicCore Ready(LogicConfig config = LogicConfig{}) {
  LogicCore core{config};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(PoseAt(1, 2));
  core.OnMowMap(Garden(), Reason());
  core.TakeEvents();
  core.TakeActionsChanged();
  return core;
}

bool Enabled(const LogicCore& core, const std::string& prefix, const std::string& id) {
  for (const auto& group : core.Actions()) {
    if (group.prefix != prefix) continue;
    for (const auto& action : group.actions) {
      if (action.id == id) return action.enabled;
    }
  }
  ADD_FAILURE() << "missing action " << prefix << "/" << id;
  return false;
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

// Started, the first area planned with two lanes from the pose on.
LogicCore Following(LogicConfig config = LogicConfig{}) {
  LogicCore core = Ready(config);
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({Lane(1), Lane(1)}, "fp");
  const JobStep step = core.NextJobStep(0);
  EXPECT_EQ(step.kind, JobStep::Kind::kFollow);
  core.TakeEvents();
  core.TakeDriveCancel();
  core.TakeStopRequest();
  return core;
}

}  // namespace

TEST(LogicMowing, StartRefusedWithAReasonAndAnEvent) {
  struct Case {
    const char* why;
    std::function<void(LogicCore&)> spoil;
  };
  const std::vector<Case> cases = {
      {"EMERGENCY", [](LogicCore& c) { auto hw = Healthy(); hw.emergency = true; c.OnHardwareStatus(hw); }},
      {"NO_SAFETY_MEASUREMENT", [](LogicCore& c) { c.OnCapabilities(false); }},
      // On the contacts without a docking station in the map.
      {"NO_DOCK", [](LogicCore& c) { auto hw = Healthy(); hw.dock_contact = true; c.OnHardwareStatus(hw); }},
      {"RAIN", [](LogicCore& c) { auto hw = Healthy(); hw.rain = true; c.OnHardwareStatus(hw); }},
      {"BATTERY_LOW", [](LogicCore& c) { auto hw = Healthy(); hw.battery_voltage = 20; c.OnHardwareStatus(hw); }},
      {"NO_POSITION", [](LogicCore& c) { c.OnPoseTimeout(); }},
      {"POSITION_ACCURACY", [](LogicCore& c) { c.OnPose(PoseAt(1, 2, 0.5f)); }},
      {"NO_HEADING", [](LogicCore& c) { auto p = PoseAt(1, 2); p.orientation_valid = false; c.OnPose(p); }},
      {"MAP_MISMATCH", [](LogicCore& c) { c.OnMowMap(Garden(), MapReason(MapDatum::kMismatch, "map mismatch")); }},
      {"NO_MOWING_AREA", [](LogicCore& c) { c.OnMowMap({{"bed", "", false}}, Reason()); }},
      {"BUSY", [](LogicCore& c) { c.OnAction("mower_logic:idle/start_area_recording"); }},
  };
  for (const auto& test : cases) {
    LogicCore core = Ready();
    EXPECT_TRUE(Enabled(core, "mower_logic:idle", "start_mowing"));
    test.spoil(core);
    core.TakeEvents();
    EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_mowing")) << test.why;
    EXPECT_EQ(core.OnAction(kStart), ActionResult::kRejected) << test.why;
    EXPECT_FALSE(core.Mowing());
    const auto events = core.TakeEvents();
    const Event* refused = Find(events, "MOWING_REFUSED");
    ASSERT_TRUE(refused) << test.why;
    EXPECT_EQ(refused->details["reason"], test.why);
    EXPECT_EQ(refused->details["resume"], "user") << test.why;
    EXPECT_TRUE(refused->details.contains("severity")) << test.why;
    EXPECT_FALSE(refused->details["message"].get<std::string>().empty()) << test.why;
  }
}

TEST(LogicMowing, StartOpensAJobAndASessionCarriedByEveryEvent) {
  LogicCore core = Ready();
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_TRUE(core.Mowing());
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_EQ(core.RobotState().current_area, 0);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_mowing"));
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  EXPECT_TRUE(Enabled(core, "mower_logic:mowing", "pause"));
  EXPECT_FALSE(Enabled(core, "mower_logic:mowing", "continue"));
  EXPECT_TRUE(Enabled(core, "mower_logic:mowing", "abort_mowing"));
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "STATE");
  EXPECT_EQ(events[0].details["state"], "MOWING");
  const std::string job = events[0].details["job_id"];
  const std::string session = events[0].details["session_id"];
  EXPECT_EQ(job.size(), 32u);
  EXPECT_EQ(session.size(), 32u);
  EXPECT_NE(job, session);
  // Not for the app while mowing: a second start, recording, a drive.
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kRejected);
  EXPECT_FALSE(core.StartDrive(3, 3).empty());
  auto hw = Healthy();
  hw.blade.rpm = 2500;
  core.OnHardwareStatus(hw);
  const auto blades = core.TakeEvents();
  ASSERT_TRUE(Find(blades, "BLADES"));
  EXPECT_EQ(Find(blades, "BLADES")->details["job_id"], job);
  EXPECT_EQ(Find(blades, "BLADES")->details["session_id"], session);
}

TEST(LogicMowing, PlansEachMowableAreaFollowsItsPathsAndCompletesTheJob) {
  LogicCore core = Ready();
  core.OnAction(kStart);
  const std::string job = core.TakeEvents()[0].details["job_id"];
  JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(step.area_id, "front");
  auto events = core.TakeEvents();
  ASSERT_TRUE(Find(events, "AREA"));
  EXPECT_EQ(Find(events, "AREA")->details["area_name"], "Front");
  // Nothing more while the plan is on its way.
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  core.OnJobPlanned({Lane(1), Lane(5)}, "fp");
  // The first path starts at the pose: no approach.
  step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kFollow);
  EXPECT_EQ(step.start_index, 0u);
  EXPECT_EQ(step.path.poses.size(), 5u);
  EXPECT_TRUE(core.FollowerActive());
  core.OnJobProgress(3);
  EXPECT_EQ(core.RobotState().current_path, 0);
  EXPECT_EQ(core.RobotState().current_path_index, 3);
  core.OnLegEnded(LegEnd::kDone, 4, Reason("DONE", ""));
  EXPECT_EQ(core.RobotState().current_path, 1);
  // The second path starts 4 m away: approach first, then follow.
  step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kApproach);
  EXPECT_DOUBLE_EQ(step.goal.x, 5);
  core.OnPose(PoseAt(5, 2));
  core.OnLegEnded(LegEnd::kDone, 0, Reason("DONE", ""));
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  core.OnLegEnded(LegEnd::kDone, 4, Reason("DONE", ""));
  // On to the third area, the second one is not mowed.
  step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(step.area_id, "back");
  EXPECT_EQ(core.RobotState().current_area, 2);
  core.TakeEvents();
  core.OnJobPlanned({}, "fp2");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  EXPECT_FALSE(core.Mowing());
  events = core.TakeEvents();
  EXPECT_EQ(Types(events), (std::vector<std::string>{"JOB_COMPLETE", "STATE"}));
  EXPECT_EQ(events[0].details["job_id"], job);
  EXPECT_TRUE(events[0].details.contains("session_id"));
  // The finished job is gone with the session.
  EXPECT_EQ(events[1].details["state"], "IDLE");
  EXPECT_FALSE(events[1].details.contains("job_id"));
  EXPECT_FALSE(events[1].details.contains("session_id"));
  EXPECT_TRUE(core.TakeStopRequest());
  const auto saved = ParseCheckpoint(*core.TakeCheckpoint());
  ASSERT_TRUE(saved);
  EXPECT_EQ(saved->job_id, "");
  EXPECT_EQ(saved->area, 0);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "reset_job"));
}

TEST(LogicMowing, PauseStopsAtOnceAndContinueGoesOnFromWhereItStopped) {
  LogicCore core = Following();
  core.OnJobProgress(2);
  EXPECT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_TRUE(core.TakeDriveCancel());
  EXPECT_TRUE(core.TakeStopRequest());
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["state"], "PAUSED");
  EXPECT_EQ(events[0].details["reason"], "MANUAL");
  EXPECT_EQ(events[0].details["message"], "Paused by the app");
  EXPECT_EQ(events[0].details["severity"], "info");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_FALSE(Enabled(core, "mower_logic:mowing", "pause"));
  EXPECT_TRUE(Enabled(core, "mower_logic:mowing", "continue"));
  EXPECT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kRejected);
  // The follower reports where it stopped.
  core.OnLegEnded(LegEnd::kCanceled, 3, Reason("CANCELED", ""));
  EXPECT_EQ(core.RobotState().current_path_index, 3);
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  ASSERT_TRUE(core.TakeCheckpoint());
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_EQ(core.TakeEvents()[0].details["state"], "MOWING");
  core.OnPose(PoseAt(2.5, 2));
  const JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kFollow);
  EXPECT_EQ(step.start_index, 3u);
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kRejected);
}

TEST(LogicMowing, EmergencyPausesAndWaitsForContinueAfterTheReset) {
  LogicCore core = Following();
  auto hw = Healthy();
  hw.emergency = true;
  hw.emergency_reason = kEmergencyLatch | kEmergencyLift;
  core.OnHardwareStatus(hw);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_TRUE(core.TakeDriveCancel());
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_FALSE(Enabled(core, "mower_logic:mowing", "continue"));
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kRejected);
  core.OnLegEnded(LegEnd::kEmergency, 1, Reason("EMERGENCY", ""));
  core.OnHardwareStatus(Healthy());
  // Released, but it does not drive off by itself.
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  EXPECT_TRUE(Enabled(core, "mower_logic:mowing", "continue"));
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
}

TEST(LogicMowing, ABladeRefusedForAnEmergencyPausesAsTheEmergencyDoes) {
  LogicConfig config;
  config.mow_with_blade = true;
  // The same stop, once told by hw/status, once first by the answer of hw/blade.
  LogicCore by_status = Following(config);
  LogicCore by_blade = Following(config);
  by_status.OnJobProgress(1);
  by_blade.OnJobProgress(1);
  ASSERT_TRUE(by_blade.BladeWanted());
  auto hw = Healthy();
  hw.emergency = true;
  hw.emergency_reason = kEmergencyLatch | kEmergencyStop;
  by_status.OnHardwareStatus(hw);
  by_blade.OnBladeRefused("mower blocked", kEmergencyLatch | kEmergencyStop);
  EXPECT_EQ(by_blade.RobotState().state, "PAUSED");
  EXPECT_FALSE(by_blade.BladeWanted());
  EXPECT_TRUE(by_blade.TakeDriveCancel());
  const auto told_by_status = by_status.TakeEvents();
  const auto told_by_blade = by_blade.TakeEvents();
  const Event* paused = Find(told_by_blade, "STATE");
  const Event* expected = Find(told_by_status, "STATE");
  ASSERT_NE(paused, nullptr);
  ASSERT_NE(expected, nullptr);
  EXPECT_EQ(paused->details.at("reason"), "EMERGENCY");
  EXPECT_EQ(paused->details.at("message"), "Emergency: LATCH, STOP");
  for (const char* key : {"state", "reason", "message", "severity", "resume"}) {
    EXPECT_EQ(paused->details.at(key), expected->details.at(key)) << key;
  }
  // Flag and event of the emergency come with hw/status, which also refuses continue until the reset.
  EXPECT_EQ(Find(told_by_blade, "EMERGENCY"), nullptr);
  EXPECT_FALSE(by_blade.RobotState().emergency);
  by_blade.OnHardwareStatus(hw);
  EXPECT_EQ(Types(by_blade.TakeEvents()), std::vector<std::string>{"EMERGENCY"});
  EXPECT_TRUE(by_blade.RobotState().emergency);
  EXPECT_EQ(by_blade.RobotState().emergency_reason, kEmergencyLatch | kEmergencyStop);
  EXPECT_EQ(by_blade.OnAction(kMowing + "continue"), ActionResult::kRejected);
  by_blade.OnLegEnded(LegEnd::kEmergency, 1, Reason("EMERGENCY", ""));
  by_blade.OnHardwareStatus(Healthy());
  EXPECT_EQ(by_blade.RobotState().state, "PAUSED");
  EXPECT_EQ(by_blade.OnAction(kMowing + "continue"), ActionResult::kDone);
  EXPECT_EQ(by_blade.RobotState().state, "MOWING");
  // Without reason bits a refusal stays a fault of its own.
  LogicCore refused = Following(config);
  refused.OnJobProgress(1);
  refused.OnBladeRefused("mower on the dock", 0);
  const auto events = refused.TakeEvents();
  ASSERT_NE(Find(events, "STATE"), nullptr);
  EXPECT_EQ(Find(events, "STATE")->details.at("reason"), "BLADE_REFUSED");
  EXPECT_EQ(Find(events, "STATE")->details.at("message"), "The blade was refused: mower on the dock");
}

TEST(LogicMowing, TheGpsPauseCarriesTheAccuracyAndTheLimit) {
  LogicConfig config;
  LogicCore core = Following(config);
  core.OnPose(PoseAt(1, 2, 0.3f));
  core.TakeEvents();
  core.OnLegEnded(LegEnd::kGps, 2, Reason("POSITION_ACCURACY", "Position accuracy 0.30 m"));
  const auto events = core.TakeEvents();
  const Event* state = Find(events, "STATE");
  ASSERT_TRUE(state);
  EXPECT_EQ(state->details["reason"], "POSITION_ACCURACY");
  EXPECT_NEAR(state->details["accuracy"].get<double>(), 0.3, 1e-6);
  EXPECT_NEAR(state->details["limit"].get<double>(), config.max_position_accuracy, 1e-6);
}

TEST(LogicMowing, AfterBadGpsItGoesOnByItselfOnceThePositionIsGoodLongEnough) {
  LogicConfig config;
  config.gps_wait_time = 10;
  LogicCore core = Following(config);
  core.OnLegEnded(LegEnd::kGps, 2, Reason("POSITION_ACCURACY", "Position accuracy 0.30 m"));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  core.OnPose(PoseAt(1, 2, 0.3f));
  core.OnTick(100);
  core.OnTick(120);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  core.OnPose(PoseAt(2, 2));
  core.OnTick(121);
  core.OnTick(126);
  core.OnPose(PoseAt(2, 2, 0.3f));
  core.OnTick(127);
  core.OnPose(PoseAt(2, 2));
  core.OnTick(128);
  core.OnTick(137);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  core.OnTick(138);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_EQ(core.NextJobStep(138).kind, JobStep::Kind::kFollow);
  // A lost pose pauses as well; continue does not skip the wait for it.
  core.OnPoseTimeout();
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kRejected);
}

TEST(LogicMowing, GpsSettingsCountAtOnceAndOnlyIdleTakesSettings) {
  LogicCore core = Ready();
  EXPECT_TRUE(core.Idle());
  // The pose is at 0.02 m.
  core.SetGpsSettings(0.01, 10, 10);
  const Reason problem = core.StartProblem();
  EXPECT_EQ(problem.code, "POSITION_ACCURACY");
  EXPECT_DOUBLE_EQ(problem.fields["limit"].get<double>(), 0.01);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_mowing"));
  core.SetGpsSettings(0.1, 10, 10);
  EXPECT_TRUE(core.StartProblem().empty());
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_FALSE(core.Idle());
  core.OnAction(kMowing + "pause");
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_FALSE(core.Idle());
}

TEST(LogicMowing, GpsWaitTimeOfTheSettings) {
  LogicCore core = Following();
  core.SetGpsSettings(0.1, 3, 10);
  core.OnLegEnded(LegEnd::kGps, 2, Reason("POSITION_ACCURACY", "Position accuracy 0.30 m"));
  core.OnPose(PoseAt(2, 2));
  core.OnTick(200);
  core.OnTick(202.9);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  core.OnTick(203);
  EXPECT_EQ(core.RobotState().state, "MOWING");
}

TEST(LogicMowing, TheBladeOnlyOnMowSegmentsAndOnlyWhenAllowed) {
  LogicCore off = Following();
  off.OnJobProgress(2);
  EXPECT_FALSE(off.BladeWanted());

  LogicConfig config;
  config.mow_with_blade = true;
  LogicCore core = Following(config);
  // Index 0 starts the transit to the lane.
  EXPECT_FALSE(core.BladeWanted());
  core.OnJobProgress(1);
  EXPECT_TRUE(core.BladeWanted());
  auto hw = Healthy();
  hw.dock_contact = true;
  core.OnHardwareStatus(hw);
  EXPECT_FALSE(core.BladeWanted());
  core.OnHardwareStatus(Healthy());
  EXPECT_TRUE(core.BladeWanted());
  core.OnHardwareTimeout();
  EXPECT_FALSE(core.BladeWanted());
  core.OnHardwareStatus(Healthy());
  core.OnAction(kMowing + "pause");
  EXPECT_FALSE(core.BladeWanted());
  core.OnLegEnded(LegEnd::kCanceled, 1, Reason("CANCELED", ""));
  core.OnAction(kMowing + "continue");
  // Continued, but off until the follower drives again.
  EXPECT_FALSE(core.BladeWanted());
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  EXPECT_TRUE(core.BladeWanted());
  // A refusal of hw/blade pauses; the blade is off and stays off.
  core.OnBladeRefused("allow_blade is off");
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_FALSE(core.BladeWanted());
  core.OnLegEnded(LegEnd::kCanceled, 1, Reason("CANCELED", ""));
  core.OnAction(kMowing + "abort_mowing");
  EXPECT_FALSE(core.BladeWanted());
  // An approach is a transit.
  LogicCore approach = Ready(config);
  approach.OnAction(kStart);
  approach.NextJobStep(0);
  approach.OnJobPlanned({Lane(6)}, "fp");
  ASSERT_EQ(approach.NextJobStep(0).kind, JobStep::Kind::kApproach);
  EXPECT_FALSE(approach.BladeWanted());
  // Without a safety measurement never.
  approach.OnCapabilities(false);
  EXPECT_EQ(approach.RobotState().state, "PAUSED");
}

TEST(LogicMowing, WaitsForTheBladeToSpinUpOrPausesWhenItDoesNot) {
  LogicConfig config;
  config.mow_with_blade = true;
  config.mower_spinup_rpm = 2000;
  config.mower_spinup_timeout = 10;
  LogicCore core = Ready(config);
  core.OnAction(kStart);
  core.NextJobStep(0);
  JobPath path = Lane(1);
  path.poses[0].kind = kSegmentLane;
  core.OnJobPlanned({path, path}, "fp");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  EXPECT_TRUE(core.BladeWanted());
  auto hw = Healthy();
  hw.blade.rpm = 2100;
  core.OnHardwareStatus(hw);
  EXPECT_EQ(core.NextJobStep(5).kind, JobStep::Kind::kFollow);
  core.OnLegEnded(LegEnd::kDone, 4, Reason("DONE", ""));
  core.OnHardwareStatus(Healthy());
  core.TakeEvents();
  EXPECT_EQ(core.NextJobStep(20).kind, JobStep::Kind::kNone);
  EXPECT_EQ(core.NextJobStep(30).kind, JobStep::Kind::kNone);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_EQ(core.NextJobStep(30.5).kind, JobStep::Kind::kNone);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_TRUE(Find(core.TakeEvents(), "MOW_MOTOR_SPINUP_FAILED"));
}

TEST(LogicMowing, BladesEventsFollowTheRpmTheMcuReports) {
  LogicCore core = Ready();
  auto hw = Healthy();
  hw.blade.rpm = 50;
  core.OnHardwareStatus(hw);
  EXPECT_TRUE(core.TakeEvents().empty());
  hw.blade.rpm = -2400;
  core.OnHardwareStatus(hw);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "BLADES");
  EXPECT_EQ(events[0].details["enabled"], true);
  core.OnHardwareStatus(hw);
  EXPECT_TRUE(core.TakeEvents().empty());
  hw.blade.ok = false;
  core.OnHardwareStatus(hw);
  events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["enabled"], false);
}

TEST(LogicMowing, SkipPathAndSkipAreaWaitForTheLegToEnd) {
  LogicCore core = Following();
  EXPECT_EQ(core.OnAction(kMowing + "skip_path"), ActionResult::kDone);
  EXPECT_TRUE(core.TakeDriveCancel());
  EXPECT_EQ(core.RobotState().current_path, 0);
  core.OnLegEnded(LegEnd::kCanceled, 2, Reason("CANCELED", ""));
  EXPECT_EQ(core.RobotState().current_path, 1);
  EXPECT_EQ(core.RobotState().current_path_index, 0);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  core.TakeEvents();
  EXPECT_EQ(core.OnAction(kMowing + "skip_area"), ActionResult::kDone);
  core.OnLegEnded(LegEnd::kCanceled, 1, Reason("CANCELED", ""));
  const auto events = core.TakeEvents();
  ASSERT_TRUE(Find(events, "AREA_SKIPPED"));
  EXPECT_EQ(Find(events, "AREA_SKIPPED")->details["area_id"], "front");
  const JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(step.area_id, "back");
  // A skip while planning drops the plan on its way.
  core.OnAction(kMowing + "skip_area");
  core.OnJobPlanned({Lane(1)}, "late");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  EXPECT_FALSE(core.Mowing());
  EXPECT_TRUE(Find(core.TakeEvents(), "JOB_COMPLETE"));
}

TEST(LogicMowing, AbortKeepsTheJobForTheNextStartAndResetJobDropsIt) {
  LogicCore core = Following();
  core.OnJobProgress(2);
  EXPECT_EQ(core.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  EXPECT_FALSE(core.Mowing());
  EXPECT_TRUE(core.TakeDriveCancel());
  EXPECT_TRUE(core.TakeStopRequest());
  EXPECT_EQ(core.RobotState().current_area, -1);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["state"], "IDLE");
  const std::string job = events[0].details["job_id"];
  EXPECT_FALSE(events[0].details.contains("session_id"));
  // A late end of the leg changes nothing.
  core.OnLegEnded(LegEnd::kCanceled, 4, Reason("CANCELED", ""));
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "reset_job"));
  auto saved = ParseCheckpoint(*core.TakeCheckpoint());
  ASSERT_TRUE(saved);
  EXPECT_EQ(saved->job_id, job);
  EXPECT_EQ(saved->index, 2);
  // Started again: the same job in a new session, planned anew and from the same place.
  core.OnPose(PoseAt(2, 2));
  core.OnAction(kStart);
  events = core.TakeEvents();
  EXPECT_EQ(events[0].details["job_id"], job);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({Lane(1), Lane(1)}, "fp");
  JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kFollow);
  EXPECT_EQ(step.start_index, 2u);
  core.OnAction(kMowing + "abort_mowing");
  core.TakeEvents();
  EXPECT_EQ(core.OnAction("mower_logic:idle/reset_job"), ActionResult::kDone);
  events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "JOB_RESET");
  EXPECT_EQ(events[0].details["job_id"], job);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "reset_job"));
  EXPECT_EQ(core.OnAction("mower_logic:idle/reset_job"), ActionResult::kRejected);
  saved = ParseCheckpoint(*core.TakeCheckpoint());
  EXPECT_EQ(saved->job_id, "");
  EXPECT_EQ(saved->index, 0);
  core.OnAction(kStart);
  EXPECT_NE(core.TakeEvents()[0].details["job_id"], job);
}

TEST(LogicMowing, GoesOnFromARestoredCheckpoint) {
  JobProgress saved;
  saved.job_id = "J";
  saved.area = 2;
  saved.area_id = "back";
  saved.path = 1;
  saved.index = 3;
  saved.fingerprint = "fp";
  LogicCore core = Ready();
  core.RestoreJob(saved);
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "reset_job"));
  core.OnPose(PoseAt(2.5, 2));
  core.OnAction(kStart);
  EXPECT_EQ(core.TakeEvents()[0].details["job_id"], "J");
  const JobStep plan = core.NextJobStep(0);
  ASSERT_EQ(plan.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(plan.area_id, "back");
  core.OnJobPlanned({Lane(1), Lane(1)}, "fp");
  const JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kFollow);
  EXPECT_EQ(step.start_index, 3u);
  EXPECT_EQ(core.RobotState().current_path, 1);
}

TEST(LogicMowing, AFailedPlanOrLegPausesUntilContinue) {
  LogicCore core = Ready();
  core.OnAction(kStart);
  core.NextJobStep(0);
  core.OnJobPlanFailed("no lanes fit");
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  core.OnAction(kMowing + "continue");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({Lane(1)}, "fp");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  core.OnLegEnded(LegEnd::kFailed, 2, Reason("CROSS_TRACK", "Cross track 0.6 m"));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(core.RobotState().current_path_index, 2);
  EXPECT_FALSE(core.TakeDriveCancel());
  // A map that no longer fits pauses as well.
  core.OnAction(kMowing + "continue");
  core.OnMowMap(Garden(), MapReason(MapDatum::kMismatch, ""));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
}

TEST(LogicMowing, AFollowerThatStopsAnsweringPausesWithTheBladeOff) {
  // G22: the node ends the leg itself with FOLLOWER_MISSING; the job waits for the user, the blade is off.
  LogicConfig config;
  config.mow_with_blade = true;
  LogicCore core = Following(config);
  core.OnJobProgress(1);
  ASSERT_TRUE(core.BladeWanted());
  core.OnLegEnded(LegEnd::kFailed, 3,
                  Reason("FOLLOWER_MISSING", "No answer from the follower for 2.1 s").With("seconds", 2.1).With("limit", 2.0));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(core.RobotState().current_path_index, 3);
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_FALSE(core.FollowerActive());
  const auto events = core.TakeEvents();
  const Event* paused = Find(events, "STATE");
  ASSERT_TRUE(paused);
  EXPECT_EQ(paused->details["state"], "PAUSED");
  EXPECT_EQ(paused->details["reason"], "FOLLOWER_MISSING");
  EXPECT_DOUBLE_EQ(paused->details["seconds"].get<double>(), 2.1);
  EXPECT_DOUBLE_EQ(paused->details["limit"].get<double>(), 2.0);
  EXPECT_EQ(paused->details["severity"], "error");
  EXPECT_EQ(paused->details["resume"], "user");
  // Continue goes on from where it stopped.
  core.OnPose(PoseAt(2.5, 2));
  core.OnAction(kMowing + "continue");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  EXPECT_EQ(core.RobotState().current_path_index, 3);
}

TEST(LogicMowing, ALowBatteryEndsMowingAfterAWhile) {
  LogicCore core = Following();
  auto hw = Healthy();
  hw.battery_voltage = 20.5f;
  core.OnHardwareStatus(hw);
  core.OnTick(100);
  core.OnTick(109);
  EXPECT_TRUE(core.Mowing());
  core.OnTick(110);
  EXPECT_FALSE(core.Mowing());
  EXPECT_TRUE(Find(core.TakeEvents(), "BATTERY_LOW"));
  EXPECT_TRUE(core.TakeDriveCancel());
}

TEST(LogicMowing, CheckpointsWhileFollowingInItsPeriod) {
  LogicCore core = Following();
  core.TakeCheckpoint();
  core.OnJobProgress(1);
  core.OnTick(10);
  EXPECT_FALSE(core.TakeCheckpoint());
  core.OnTick(30);
  ASSERT_TRUE(core.TakeCheckpoint());
  core.OnTick(50);
  EXPECT_FALSE(core.TakeCheckpoint());
  core.OnJobProgress(2);
  core.OnTick(60);
  const auto saved = ParseCheckpoint(*core.TakeCheckpoint());
  ASSERT_TRUE(saved);
  EXPECT_EQ(saved->index, 2);
  EXPECT_EQ(saved->area_id, "front");
  EXPECT_EQ(saved->fingerprint, "fp");
}
