#include <gtest/gtest.h>

#include <algorithm>
#include <functional>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

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

TestCore Ready(LogicConfig config = LogicConfig{}) {
  TestCore core{config};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(PoseAt(1, 2));
  core.OnMowMap(Garden(), Reason());
  core.TakeEvents();
  core.TakeActionsChanged();
  return core;
}

bool Enabled(const TestCore& core, const std::string& prefix, const std::string& id) {
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
TestCore Following(LogicConfig config = LogicConfig{}) {
  TestCore core = Ready(config);
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
    std::function<void(TestCore&)> spoil;
  };
  const std::vector<Case> cases = {
      {"EMERGENCY", [](TestCore& c) { auto hw = Healthy(); hw.emergency = true; c.OnHardwareStatus(hw); }},
      {"NO_SAFETY_MEASUREMENT", [](TestCore& c) { c.OnCapabilities(false); }},
      // On the contacts without a docking station in the map.
      {"NO_DOCK", [](TestCore& c) { auto hw = Healthy(); hw.dock_contact = true; c.OnHardwareStatus(hw); }},
      {"RAIN", [](TestCore& c) { auto hw = Healthy(); hw.rain = true; c.OnHardwareStatus(hw); }},
      {"BATTERY_LOW", [](TestCore& c) { auto hw = Healthy(); hw.battery_voltage = 20; c.OnHardwareStatus(hw); }},
      {"BATTERY_LOW", [](TestCore& c) { auto hw = Healthy(); hw.battery_ok = false; c.OnHardwareStatus(hw); }},
      {"NO_POSITION", [](TestCore& c) { c.OnPoseTimeout(); }},
      {"POSITION_ACCURACY", [](TestCore& c) { c.OnPose(PoseAt(1, 2, 0.5f)); }},
      {"NO_HEADING", [](TestCore& c) { auto p = PoseAt(1, 2); p.orientation_valid = false; c.OnPose(p); }},
      {"MAP_MISMATCH", [](TestCore& c) { c.OnMowMap(Garden(), MapReason(MapDatum::kMismatch, "map mismatch")); }},
      {"NO_MOWING_AREA", [](TestCore& c) { c.OnMowMap({{"bed", "", false}}, Reason()); }},
      {"BUSY", [](TestCore& c) { c.OnAction("mower_logic:idle/start_area_recording"); }},
  };
  for (const auto& test : cases) {
    TestCore core = Ready();
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
  TestCore core = Ready();
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
  TestCore core = Ready();
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

// An area taken out of the map while it is mowed: the plan goes, the mower waits for the user.
TEST(LogicMowing, AMapChangeOfTheAreaMowedPausesAndPlansAnew) {
  TestCore core = Following();
  std::vector<JobArea> areas = Garden();
  areas.erase(areas.begin());
  core.OnMowMap(areas, Reason());
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(core.RobotState().pause_reason, "MAP_CHANGED");
  EXPECT_TRUE(core.TakeDriveCancel());
  core.OnLegEnded(LegEnd::kCanceled, 2, Reason("CANCELED", ""));
  // The same areas again change nothing more.
  core.OnMowMap(areas, Reason());
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  const JobStep next = core.NextJobStep(0);
  ASSERT_EQ(next.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(next.area_id, "back");
}

// A continue before the follower ended the leg the pause stopped: neither blade nor follower until it did.
TEST(LogicMowing, AQuickContinueWaitsForTheStoppedLeg) {
  LogicConfig config;
  config.mow_with_blade = true;
  TestCore core = Following(config);
  core.OnJobProgress(1);
  ASSERT_TRUE(core.BladeWanted());
  EXPECT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  core.OnLegEnded(LegEnd::kCanceled, 2, Reason("CANCELED", ""));
  EXPECT_NE(core.NextJobStep(0).kind, JobStep::Kind::kNone);
}

TEST(LogicMowing, PauseStopsAtOnceAndContinueGoesOnFromWhereItStopped) {
  TestCore core = Following();
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
  TestCore core = Following();
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
  TestCore by_status = Following(config);
  TestCore by_blade = Following(config);
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
  TestCore refused = Following(config);
  refused.OnJobProgress(1);
  refused.OnBladeRefused("mower on the dock", 0);
  const auto events = refused.TakeEvents();
  ASSERT_NE(Find(events, "STATE"), nullptr);
  EXPECT_EQ(Find(events, "STATE")->details.at("reason"), "BLADE_REFUSED");
  EXPECT_EQ(Find(events, "STATE")->details.at("message"), "The blade was refused: mower on the dock");
}

TEST(LogicMowing, ARefusalOfABladeThatIsNotWantedChangesNothing) {
  // Without mow_with_blade the blade is never wanted, so a late answer of hw/blade does not pause.
  TestCore core = Following();
  core.OnJobProgress(1);
  ASSERT_FALSE(core.BladeWanted());
  core.OnBladeRefused("allow_blade is off", 0);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_TRUE(core.TakeEvents().empty());
}

TEST(LogicMowing, AnEmergencyCancelsADriveOnlyWhereOneRuns) {
  auto stopped = Healthy();
  stopped.emergency = true;
  stopped.emergency_reason = kEmergencyLatch | kEmergencyStop;
  TestCore idle = Ready();
  idle.TakeDriveCancel();
  idle.OnHardwareStatus(stopped);
  EXPECT_FALSE(idle.TakeDriveCancel());
  // Mowing: once when it begins, not again when it ends before the follower told the end of its leg.
  TestCore core = Following();
  core.OnHardwareStatus(stopped);
  EXPECT_TRUE(core.TakeDriveCancel());
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.TakeDriveCancel());
}

TEST(LogicMowing, TheGpsPauseCarriesTheAccuracyAndTheLimit) {
  LogicConfig config;
  TestCore core = Following(config);
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

// The follower loses the position each time it goes on, without getting further: after kGpsResumes the user decides.
TEST(LogicMowing, GpsLostAgainAndAgainWithoutGettingOnWaitsForTheUser) {
  LogicConfig config;
  config.gps_wait_time = 10;
  TestCore core = Following(config);
  core.OnPose(PoseAt(2, 2));
  double t = 100;
  for (int i = 0; i < kGpsResumes; i++) {
    core.OnLegEnded(LegEnd::kGps, 2, Reason("POSE_LOST", "No recent pose"));
    ASSERT_EQ(core.RobotState().pause_reason, "POSE_LOST") << i;
    core.OnTick(t);
    core.OnTick(t + 10);
    t += 20;
    ASSERT_EQ(core.RobotState().state, "MOWING") << i;
    ASSERT_EQ(core.NextJobStep(t).kind, JobStep::Kind::kFollow) << i;
  }
  core.OnLegEnded(LegEnd::kGps, 2, Reason("POSE_LOST", "No recent pose"));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(core.RobotState().pause_reason, "POSITION_UNSTABLE");
  core.OnTick(t);
  core.OnTick(t + 30);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  // Getting on counts anew.
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(t + 31).kind, JobStep::Kind::kFollow);
  core.OnJobProgress(3);
  core.OnLegEnded(LegEnd::kGps, 3, Reason("POSE_LOST", "No recent pose"));
  EXPECT_EQ(core.RobotState().pause_reason, "POSE_LOST");
}

TEST(LogicMowing, AfterBadGpsItGoesOnByItselfOnceThePositionIsGoodLongEnough) {
  LogicConfig config;
  config.gps_wait_time = 10;
  TestCore core = Following(config);
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
  TestCore core = Ready();
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
  TestCore core = Following();
  core.SetGpsSettings(0.1, 3, 10);
  core.OnLegEnded(LegEnd::kGps, 2, Reason("POSITION_ACCURACY", "Position accuracy 0.30 m"));
  core.OnPose(PoseAt(2, 2));
  core.OnTick(200);
  core.OnTick(202.9);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  core.OnTick(203);
  EXPECT_EQ(core.RobotState().state, "MOWING");
}

// A contact of the station while mowing pauses for the user: the blade does not come back on its own.
TEST(LogicMowing, AContactOfTheStationPausesMowing) {
  LogicConfig config;
  config.mow_with_blade = true;
  TestCore core = Following(config);
  core.OnJobProgress(1);
  ASSERT_TRUE(core.BladeWanted());
  auto hw = Healthy();
  hw.dock_contact = true;
  core.OnHardwareStatus(hw);
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(core.RobotState().pause_reason, "DOCK_CONTACT");
  EXPECT_TRUE(core.TakeDriveCancel());
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_EQ(core.RobotState().state, "PAUSED");
}

TEST(LogicMowing, TheBladeOnlyOnMowSegmentsAndOnlyWhenAllowed) {
  TestCore off = Following();
  off.OnJobProgress(2);
  EXPECT_FALSE(off.BladeWanted());

  LogicConfig config;
  config.mow_with_blade = true;
  TestCore core = Following(config);
  // Index 0 starts the transit to the lane.
  EXPECT_FALSE(core.BladeWanted());
  core.OnJobProgress(1);
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
  TestCore approach = Ready(config);
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
  TestCore core = Ready(config);
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

TEST(LogicMowing, ASpinUpEndsWhenTheMowerHasToDriveToThePathFirst) {
  LogicConfig config;
  config.mow_with_blade = true;
  config.mower_spinup_rpm = 2000;
  config.mower_spinup_timeout = 10;
  TestCore core = Ready(config);
  core.OnAction(kStart);
  core.NextJobStep(0);
  JobPath path = Lane(1);
  path.poses[0].kind = kSegmentLane;
  core.OnJobPlanned({path, path}, "fp");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  ASSERT_TRUE(core.BladeWanted());
  // The position jumps away from the start of the path: no blade on the way back to it.
  core.OnPose(PoseAt(20, 20));
  EXPECT_EQ(core.NextJobStep(1).kind, JobStep::Kind::kApproach);
  EXPECT_FALSE(core.BladeWanted());
}

TEST(LogicMowing, BladesEventsFollowTheRpmTheMcuReports) {
  TestCore core = Ready();
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
  TestCore core = Following();
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
  TestCore core = Following();
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
  TestCore core = Ready();
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
  TestCore core = Ready();
  core.OnAction(kStart);
  core.NextJobStep(0);
  core.OnJobPlanFailed(Reason("PLANNING_FAILED", "no lanes fit"));
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
  TestCore core = Following(config);
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

// Without a docking station in the map: BATTERY_LOW and IDLE once the highest voltage of 20 s is below empty.
TEST(LogicMowing, ALowBatteryEndsMowingAfterAWhile) {
  TestCore core = Following();
  auto hw = Healthy();
  hw.battery_voltage = 20.5f;
  core.OnHardwareStatus(hw);
  for (double t = 100; t < 120; t += 1) core.OnTick(t);
  EXPECT_TRUE(core.Mowing());
  core.OnTick(120);
  EXPECT_FALSE(core.Mowing());
  const auto events = core.TakeEvents();
  const Event* low = Find(events, "BATTERY_LOW");
  ASSERT_TRUE(low);
  EXPECT_NEAR(low->details["battery_voltage"].get<double>(), 20.5, 1e-6);
  EXPECT_DOUBLE_EQ(low->details["limit"].get<double>(), 21.0);
  EXPECT_FALSE(Find(events, "DOCKING"));
  EXPECT_TRUE(core.TakeDriveCancel());
}

TEST(LogicMowing, CheckpointsWhileFollowingInItsPeriod) {
  TestCore core = Following();
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

namespace {

// n poses 0.5 m apart along x at the height y, all of one kind.
JobPath Row(double y, int n, uint8_t kind) {
  JobPath path;
  for (int i = 0; i < n; ++i) {
    JobPose pose;
    pose.x = 0.5 * i;
    pose.y = y;
    pose.kind = kind;
    path.poses.push_back(pose);
  }
  return path;
}

// A path through the given points, all of one kind.
JobPath Through(std::initializer_list<std::pair<double, double>> points, uint8_t kind) {
  JobPath path;
  for (const auto& [x, y] : points) {
    JobPose pose;
    pose.x = x;
    pose.y = y;
    pose.kind = kind;
    path.poses.push_back(pose);
  }
  return path;
}

const Reason kRefused("APPROACH_FAILED", "Transit planning failed: no way leaves the start");

// Mowing the first of the paths with the blade, skipped at its pose of the given index, where the mower stands; the
// approach to the path after it is on its way.
TestCore SkippedAt(const std::vector<JobPath>& paths, size_t index) {
  LogicConfig config;
  config.mow_with_blade = true;
  TestCore core = Ready(config);
  const std::vector<JobPose>& poses = paths[0].poses;
  core.OnPose(PoseAt(poses[0].x, poses[0].y));
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned(paths, "fp");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  EXPECT_TRUE(core.BladeWanted());
  EXPECT_EQ(core.OnAction(kMowing + "skip_path"), ActionResult::kDone);
  core.OnPose(PoseAt(poses[index].x, poses[index].y));
  core.OnLegEnded(LegEnd::kCanceled, index, Reason("CANCELED", ""));
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.TakeEvents();
  core.TakeNotices();
  core.TakeDriveCancel();
  core.TakeStopRequest();
  return core;
}

// An outline of 6 m skipped at its pose 2 (x = 1); the lane after it starts 4 m away.
TestCore SkippedOnTheOutline() { return SkippedAt({Row(2, 13, kSegmentPerimeter), Row(6, 5, kSegmentLane)}, 2); }

}  // namespace

TEST(LogicMowing, AfterASkipARefusedWayGoesOnAlongThePathLeftInPieces) {
  TestCore core = SkippedOnTheOutline();
  core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_TRUE(core.TakeEvents().empty());
  JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kSkippedPath);
  // 2 m on from pose 2: the path handed over ends at pose 6 and is a transit all along.
  EXPECT_EQ(step.start_index, 2u);
  ASSERT_EQ(step.path.poses.size(), 7u);
  EXPECT_EQ(step.path.poses.back().x, 3.0);
  for (const JobPose& pose : step.path.poses) EXPECT_EQ(pose.kind, kSegmentTransit);
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_TRUE(core.FollowerActive());
  EXPECT_EQ(core.TakeNotices().size(), 1u);
  // The job stays at the start of the next path, whatever the follower reports.
  core.OnJobProgress(4);
  EXPECT_EQ(core.RobotState().current_path, 1);
  EXPECT_EQ(core.RobotState().current_path_index, 0);
  core.OnPose(PoseAt(3, 2));
  core.OnLegEnded(LegEnd::kDone, 6, Reason("DONE", ""));
  // Asked again from there, and still no way: the next piece.
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  EXPECT_FALSE(core.BladeWanted());
  core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kSkippedPath);
  EXPECT_EQ(step.start_index, 6u);
  EXPECT_EQ(step.path.poses.size(), 11u);
  EXPECT_FALSE(core.BladeWanted());
  core.OnPose(PoseAt(5, 2));
  core.OnLegEnded(LegEnd::kDone, 10, Reason("DONE", ""));
  // Now the planner finds the way: the lane is mowed as after any approach.
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.OnPose(PoseAt(0, 6));
  core.OnLegEnded(LegEnd::kDone, 0, Reason("DONE", ""));
  step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kFollow);
  EXPECT_EQ(step.path.poses.size(), 5u);
  EXPECT_EQ(step.path.poses[1].kind, kSegmentLane);
  EXPECT_TRUE(core.BladeWanted());
  EXPECT_EQ(core.RobotState().state, "MOWING");
}

TEST(LogicMowing, AtTheEndOfThePathLeftARefusedWayPausesAsBefore) {
  TestCore core = SkippedOnTheOutline();
  core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  for (const size_t end : {size_t{6}, size_t{10}, size_t{12}}) {
    const JobStep step = core.NextJobStep(0);
    ASSERT_EQ(step.kind, JobStep::Kind::kSkippedPath);
    ASSERT_EQ(step.path.poses.size(), end + 1);
    core.OnPose(PoseAt(0.5 * static_cast<double>(end), 2));
    core.OnLegEnded(LegEnd::kDone, end, Reason("DONE", ""));
    EXPECT_EQ(core.RobotState().state, "MOWING");
    ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
    core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
    EXPECT_FALSE(core.BladeWanted());
  }
  // No pose of the path is left.
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  const auto events = core.TakeEvents();
  ASSERT_TRUE(Find(events, "STATE"));
  EXPECT_EQ(Find(events, "STATE")->details["reason"], "APPROACH_FAILED");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  // Continued there, the way is asked for again and pauses the same way.
  core.OnAction(kMowing + "continue");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
}

TEST(LogicMowing, AMowerOffThePathLeftPausesOnARefusedWay) {
  TestCore core = SkippedOnTheOutline();
  core.OnPose(PoseAt(1, 2.6));
  core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(Find(core.TakeEvents(), "STATE")->details["reason"], "APPROACH_FAILED");
  // Put back and continued: asked again, then the path left counts.
  core.OnPose(PoseAt(1, 2.2));
  core.OnAction(kMowing + "continue");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kSkippedPath);
  // Moved between the answer and the piece: the way is asked for again from there.
  TestCore moved = SkippedOnTheOutline();
  moved.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  moved.OnPose(PoseAt(1, 3));
  EXPECT_EQ(moved.NextJobStep(0).kind, JobStep::Kind::kApproach);
  moved.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  EXPECT_EQ(moved.RobotState().state, "PAUSED");
}

TEST(LogicMowing, OnlyNoWayAfterASkipGoesOnAlongThePathLeft) {
  // Any other refusal of the planner and a failure of the follower pause.
  TestCore failed = SkippedOnTheOutline();
  failed.OnLegEnded(LegEnd::kFailed, 0, kRefused);
  EXPECT_EQ(failed.RobotState().state, "PAUSED");
  // A planner that gives no answer is asked again.
  TestCore silent = SkippedOnTheOutline();
  silent.OnLegEnded(LegEnd::kFailed, 0, Reason("PLANNER_TIMEOUT", "no answer"));
  EXPECT_EQ(silent.RobotState().state, "MOWING");
  EXPECT_EQ(silent.NextJobStep(0).kind, JobStep::Kind::kApproach);
  // Without a skip there is no path left: moved in a pause, the refused way back pauses.
  TestCore paused = Following();
  paused.OnAction(kMowing + "pause");
  paused.OnLegEnded(LegEnd::kCanceled, 2, Reason("CANCELED", ""));
  paused.OnPose(PoseAt(2, 4));
  paused.OnAction(kMowing + "continue");
  ASSERT_EQ(paused.NextJobStep(0).kind, JobStep::Kind::kApproach);
  paused.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  EXPECT_EQ(paused.RobotState().state, "PAUSED");
  // The last path of a plan skipped: the next area has another plan.
  TestCore last = Ready();
  last.OnPose(PoseAt(0, 2));
  last.OnAction(kStart);
  last.NextJobStep(0);
  last.OnJobPlanned({Row(2, 13, kSegmentPerimeter)}, "fp");
  ASSERT_EQ(last.NextJobStep(0).kind, JobStep::Kind::kFollow);
  last.OnAction(kMowing + "skip_path");
  last.OnLegEnded(LegEnd::kCanceled, 2, Reason("CANCELED", ""));
  ASSERT_EQ(last.NextJobStep(0).kind, JobStep::Kind::kPlan);
  last.OnPose(PoseAt(1, 2));
  last.OnJobPlanned({Row(6, 5, kSegmentLane)}, "fp");
  ASSERT_EQ(last.NextJobStep(0).kind, JobStep::Kind::kApproach);
  last.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  EXPECT_EQ(last.RobotState().state, "PAUSED");
}

TEST(LogicMowing, APieceOfThePathLeftStopsLikeAnyLeg) {
  TestCore core = SkippedOnTheOutline();
  core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kSkippedPath);
  EXPECT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
  EXPECT_TRUE(core.TakeDriveCancel());
  EXPECT_TRUE(core.TakeStopRequest());
  core.OnPose(PoseAt(2, 2));
  core.OnLegEnded(LegEnd::kCanceled, 4, Reason("CANCELED", ""));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  // Continued: asked again, then on from the pose it got to.
  core.OnAction(kMowing + "continue");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  const JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kSkippedPath);
  EXPECT_EQ(step.start_index, 4u);
  EXPECT_EQ(step.path.poses.size(), 9u);
  // A piece the follower fails on pauses with its reason.
  core.TakeEvents();
  core.OnLegEnded(LegEnd::kFailed, 5, Reason("TIMEOUT", "stuck"));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(Find(core.TakeEvents(), "STATE")->details["reason"], "TIMEOUT");
  EXPECT_FALSE(core.BladeWanted());

  // A second skip stops the piece and skips the lane, the last path of the plan.
  TestCore twice = SkippedOnTheOutline();
  twice.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  ASSERT_EQ(twice.NextJobStep(0).kind, JobStep::Kind::kSkippedPath);
  EXPECT_EQ(twice.OnAction(kMowing + "skip_path"), ActionResult::kDone);
  EXPECT_TRUE(twice.TakeDriveCancel());
  twice.OnLegEnded(LegEnd::kCanceled, 3, Reason("CANCELED", ""));
  EXPECT_EQ(twice.NextJobStep(0).kind, JobStep::Kind::kPlan);

  // Mowing ended during a piece: the drive is canceled.
  TestCore ended = SkippedOnTheOutline();
  ended.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  ASSERT_EQ(ended.NextJobStep(0).kind, JobStep::Kind::kSkippedPath);
  EXPECT_EQ(ended.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  EXPECT_TRUE(ended.TakeDriveCancel());
  EXPECT_FALSE(ended.Mowing());
}

TEST(LogicMowing, APieceOfThePathLeftIsItsNextStretchAndWhatFitsIntoTwoMetres) {
  const JobPath lane = Row(8, 5, kSegmentLane);
  // From the first pose and along y: four stretches of 0.5 m.
  TestCore first =
      SkippedAt({Through({{0, 2}, {0, 2.5}, {0, 3}, {0, 3.5}, {0, 4}, {0, 4.5}, {0, 5}}, kSegmentPerimeter), lane}, 0);
  first.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  JobStep step = first.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kSkippedPath);
  EXPECT_EQ(step.start_index, 0u);
  ASSERT_EQ(step.path.poses.size(), 5u);
  EXPECT_EQ(step.path.poses.back().y, 4.0);
  // A stretch longer than that is driven whole, with nothing after it.
  TestCore longer = SkippedAt({Through({{0, 2}, {1, 2}, {4, 2}, {4.5, 2}, {5, 2}}, kSegmentPerimeter), lane}, 1);
  longer.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  step = longer.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kSkippedPath);
  EXPECT_EQ(step.start_index, 1u);
  EXPECT_EQ(step.path.poses.size(), 3u);
  // One pose before its end the path left still has a stretch.
  TestCore last = SkippedAt({Row(2, 13, kSegmentPerimeter), lane}, 11);
  last.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  step = last.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kSkippedPath);
  EXPECT_EQ(step.start_index, 11u);
  EXPECT_EQ(step.path.poses.size(), 13u);
}

TEST(LogicMowing, ThePathLeftCountsOnlyOnTheWayToThePathRightAfterIt) {
  TestCore core = SkippedAt({Row(2, 13, kSegmentPerimeter), Row(6, 5, kSegmentLane), Row(10, 5, kSegmentLane)}, 2);
  // The planner finds the way to the lane, which is mowed to its end.
  core.OnPose(PoseAt(0, 6));
  core.OnLegEnded(LegEnd::kDone, 0, Reason("DONE", ""));
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  core.OnLegEnded(LegEnd::kDone, 4, Reason("DONE", ""));
  // Back where the outline was left, the way to the second lane is refused.
  core.OnPose(PoseAt(1, 2));
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_EQ(Find(core.TakeEvents(), "STATE")->details["reason"], "APPROACH_FAILED");
}

TEST(LogicMowing, ALegEndWithNoLegDrivingChangesNothing) {
  TestCore core = Ready();
  core.OnPose(PoseAt(0, 2));
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.TakeEvents();
  // Waiting for the plan: a late end of the follower is none of the job's.
  core.OnLegEnded(LegEnd::kFailed, 0, Reason("TIMEOUT", "stuck"));
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_TRUE(core.TakeEvents().empty());
}

namespace {

// mowing.start with these areas in order.
json Tasks(std::initializer_list<const char*> ids, bool restart = false) {
  json tasks = json::array();
  for (const char* id : ids) tasks.push_back(json::object({{"area_id", id}}));
  json params = {{"tasks", tasks}};
  if (restart) params["restart"] = true;
  return params;
}

}  // namespace

TEST(LogicTasks, AListThatDoesNotFitIsRefusedAndNeverMowsAllAreas) {
  TestCore core = Ready();
  const json one = json::array({json::object({{"area_id", "back"}})});
  const std::vector<std::pair<json, std::string>> cases = {
      {json{{"tasks", json::array()}}, "tasks"},
      {Tasks({"back", "back"}), "tasks"},
      {Tasks({"gone"}), "tasks"},
      {json{{"tasks", "back"}}, "tasks"},
      {json{{"tasks", json::array({"back"})}}, "tasks"},
      {json{{"tasks", json::array({json::object()})}}, "tasks"},
      {json{{"tasks", json::array({json::object({{"area_id", 3}})})}}, "tasks"},
      {json{{"tasks", json::array({json::object({{"area_id", "back"}, {"angle", 30}})})}}, "tasks"},
      {json{{"restart", 1}}, "restart"},
      {json{{"areas", json::array({"back"})}}, "areas"},
      {json{{"areas", json::array({"back"})}, {"tasks", one}}, "areas"},
  };
  for (const auto& [params, field] : cases) {
    const CommandOutcome outcome = core.Command("mowing.start", params);
    EXPECT_FALSE(outcome.invalid.empty()) << params.dump();
    EXPECT_EQ(outcome.invalid_field, field) << params.dump();
    EXPECT_FALSE(outcome.accepted);
  }
  EXPECT_FALSE(core.Mowing());
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
}

TEST(LogicTasks, WithoutAMapTheStartIsRefusedForTheMap) {
  TestCore core = Ready();
  core.OnMowMap({}, Reason("MAP_UNREADABLE", "Cannot mow on the map: map/json is not readable"));
  core.TakeEvents();
  const CommandOutcome outcome = core.Command("mowing.start", Tasks({"gone"}));
  EXPECT_TRUE(outcome.invalid.empty());
  EXPECT_FALSE(outcome.accepted);
  EXPECT_EQ(outcome.why.code, "MAP_UNREADABLE");
  EXPECT_TRUE(Find(core.TakeEvents(), "MOWING_REFUSED"));
}

TEST(LogicTasks, AListWithoutAnActiveAreaIsRefused) {
  TestCore core = Ready();
  const CommandOutcome outcome = core.Command("mowing.start", Tasks({"bed"}));
  EXPECT_TRUE(outcome.invalid.empty());
  EXPECT_FALSE(outcome.accepted);
  EXPECT_EQ(outcome.why.code, "NO_MOWING_AREA");
  EXPECT_EQ(outcome.why.message, "No area of tasks is active");
  EXPECT_FALSE(core.Mowing());
}

TEST(LogicTasks, MowsTheListInItsOrderAndCompletesAfterIt) {
  TestCore core = Ready();
  ASSERT_TRUE(core.Command("mowing.start", Tasks({"back", "front"})).accepted);
  JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(step.area_id, "back");
  // current_area is the index in the map, task_index the place in the list.
  EXPECT_EQ(core.RobotState().current_area, 2);
  EXPECT_EQ(core.RobotState().task_index, 0);
  EXPECT_EQ(core.RobotState().task_count, 2);
  auto saved = ParseCheckpoint(*core.TakeCheckpoint());
  ASSERT_TRUE(saved);
  EXPECT_EQ(saved->tasks, (std::vector<std::string>{"back", "front"}));
  core.OnJobPlanned({}, "fp");
  step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(step.area_id, "front");
  EXPECT_EQ(core.RobotState().current_area, 0);
  EXPECT_EQ(core.RobotState().task_index, 1);
  core.TakeEvents();
  core.OnJobPlanned({}, "fp");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  EXPECT_FALSE(core.Mowing());
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"JOB_COMPLETE", "STATE"}));
  EXPECT_EQ(core.RobotState().task_index, -1);
  EXPECT_EQ(core.RobotState().task_count, -1);
  saved = ParseCheckpoint(*core.TakeCheckpoint());
  EXPECT_TRUE(saved->tasks.empty());
}

TEST(LogicTasks, TheSameListGoesOnAnotherOneOrRestartBeginsANewJob) {
  TestCore core = Ready();
  ASSERT_TRUE(core.Command("mowing.start", Tasks({"back"})).accepted);
  const std::string job = core.TakeEvents()[0].details["job_id"];
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({Lane(1), Lane(1)}, "fp");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  core.OnJobProgress(2);
  core.Command("mowing.abort", json::object());
  core.TakeEvents();
  // The same list: the same job from the same place.
  core.OnPose(PoseAt(2, 2));
  ASSERT_TRUE(core.Command("mowing.start", Tasks({"back"})).accepted);
  auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE"}));
  EXPECT_EQ(events[0].details["job_id"], job);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({Lane(1), Lane(1)}, "fp");
  EXPECT_EQ(core.NextJobStep(0).start_index, 2u);
  core.Command("mowing.abort", json::object());
  core.TakeEvents();
  // Another list drops the open job with JOB_RESET and begins a new one.
  ASSERT_TRUE(core.Command("mowing.start", Tasks({"front"})).accepted);
  events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"JOB_RESET", "STATE"}));
  EXPECT_EQ(events[0].details["job_id"], job);
  const std::string second = events[1].details["job_id"];
  EXPECT_NE(second, job);
  const JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(step.area_id, "front");
  core.Command("mowing.abort", json::object());
  core.TakeEvents();
  // restart begins anew with the same list.
  ASSERT_TRUE(core.Command("mowing.start", Tasks({"front"}, true)).accepted);
  events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"JOB_RESET", "STATE"}));
  EXPECT_EQ(events[0].details["job_id"], second);
  EXPECT_NE(events[1].details["job_id"], second);
  core.Command("mowing.abort", json::object());
  core.TakeEvents();
  // Without tasks every active area: another list than front alone.
  ASSERT_TRUE(core.Command("mowing.start", json::object()).accepted);
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"JOB_RESET", "STATE"}));
}

TEST(LogicTasks, TheListOfAllActiveAreasGoesOnWithAJobOverAllAreas) {
  TestCore core = Following();
  EXPECT_EQ(core.RobotState().task_index, 0);
  EXPECT_EQ(core.RobotState().task_count, 2);
  core.OnJobProgress(2);
  core.Command("mowing.abort", json::object());
  const std::string job = core.TakeEvents()[0].details["job_id"];
  ASSERT_TRUE(core.Command("mowing.start", Tasks({"front", "back"})).accepted);
  auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE"}));
  EXPECT_EQ(events[0].details["job_id"], job);
  core.Command("mowing.abort", json::object());
  core.TakeEvents();
  // Another order is another list.
  ASSERT_TRUE(core.Command("mowing.start", Tasks({"back", "front"})).accepted);
  events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"JOB_RESET", "STATE"}));
  EXPECT_EQ(events[0].details["job_id"], job);
}

TEST(LogicTasks, AListOfAllActiveAreasStaysThatListWhenTheMapGrows) {
  TestCore core = Ready();
  ASSERT_TRUE(core.Command("mowing.start", Tasks({"front", "back"})).accepted);
  EXPECT_EQ(ParseCheckpoint(*core.TakeCheckpoint())->tasks, (std::vector<std::string>{"front", "back"}));
  core.OnMowMap({{"front", "Front", true}, {"bed", "Bed", false}, {"back", "Back", true}, {"side", "Side", true}},
                Reason());
  EXPECT_EQ(core.RobotState().task_count, 2);
  // Without tasks every active area: another list now.
  core.Command("mowing.abort", json::object());
  core.TakeEvents();
  ASSERT_TRUE(core.Command("mowing.start", json::object()).accepted);
  EXPECT_TRUE(Find(core.TakeEvents(), "JOB_RESET"));
  EXPECT_EQ(core.RobotState().task_count, 3);
}

TEST(LogicTasks, SkipAreaGoesOnInTheList) {
  TestCore core = Ready();
  ASSERT_TRUE(core.Command("mowing.start", Tasks({"back", "front"})).accepted);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({Lane(1), Lane(1)}, "fp");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  core.TakeEvents();
  ASSERT_TRUE(core.Command("mowing.skip_area", json{{"area", 2}}).accepted);
  core.OnLegEnded(LegEnd::kCanceled, 1, Reason("CANCELED", ""));
  const auto events = core.TakeEvents();
  const Event* skipped = Find(events, "AREA_SKIPPED");
  ASSERT_TRUE(skipped);
  EXPECT_EQ(skipped->details["area_id"], "back");
  const JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(step.area_id, "front");
  EXPECT_EQ(core.RobotState().task_index, 1);
  core.Command("mowing.skip_area", json::object());
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  EXPECT_FALSE(core.Mowing());
  EXPECT_TRUE(Find(core.TakeEvents(), "JOB_COMPLETE"));
}

TEST(LogicTasks, ARestoredJobGoesOnInItsList) {
  JobProgress saved;
  saved.job_id = "J";
  saved.area = 0;
  saved.area_id = "front";
  saved.path = 1;
  saved.index = 3;
  saved.fingerprint = "fp";
  saved.tasks = {"back", "front"};
  saved.task = 1;
  TestCore core = Ready();
  core.RestoreJob(saved);
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "reset_job"));
  core.OnPose(PoseAt(2.5, 2));
  ASSERT_TRUE(core.Command("mowing.start", Tasks({"back", "front"})).accepted);
  EXPECT_EQ(core.TakeEvents()[0].details["job_id"], "J");
  const JobStep plan = core.NextJobStep(0);
  ASSERT_EQ(plan.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(plan.area_id, "front");
  core.OnJobPlanned({Lane(1), Lane(1)}, "fp");
  const JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kFollow);
  EXPECT_EQ(step.start_index, 3u);
  EXPECT_EQ(core.RobotState().task_index, 1);
}

// A spin-up that ended with mowing is not taken into the next start: the blade waits for its path again.
TEST(LogicMowing, ASpinUpLeftBehindDoesNotTurnTheBladeOnTheNextStart) {
  LogicConfig config;
  config.mow_with_blade = true;
  config.mower_spinup_rpm = 2000;
  config.mower_spinup_timeout = 10;
  TestCore core = Ready(config);
  core.OnAction(kStart);
  core.NextJobStep(0);
  JobPath path = Lane(1);
  path.poses[0].kind = kSegmentLane;
  core.OnJobPlanned({path, path}, "fp");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
  ASSERT_TRUE(core.BladeWanted());
  ASSERT_EQ(core.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  if (core.Docking()) core.OnAction("mower_logic:docking/abort_docking");
  ASSERT_FALSE(core.Mowing());
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_TRUE(core.Mowing());
  EXPECT_FALSE(core.BladeWanted());
}

// A pause ends with mowing: the next start mows at once.
TEST(LogicMowing, APauseDoesNotCarryOverToTheNextStart) {
  TestCore core = Following();
  ASSERT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
  ASSERT_EQ(core.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  ASSERT_FALSE(core.Mowing());
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
}

// A skip still waiting for its leg to stop ends with mowing: the next start skips nothing.
TEST(LogicMowing, ASkipLeftOpenDoesNotCarryOverToTheNextStart) {
  TestCore core = Following();
  ASSERT_EQ(core.OnAction(kMowing + "skip_area"), ActionResult::kDone);
  ASSERT_EQ(core.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  ASSERT_FALSE(core.Mowing());
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({Lane(1), Lane(1)}, "fp");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  core.TakeEvents();
  core.OnLegEnded(LegEnd::kDone, 4, Reason("DONE", "done"));
  EXPECT_FALSE(Find(core.TakeEvents(), "AREA_SKIPPED"));
  EXPECT_EQ(core.RobotState().current_area, 0);
  EXPECT_EQ(core.RobotState().current_path, 1);
}

// The path a skip left belongs to its run: after a new start an approach without a way pauses as usual.
TEST(LogicMowing, ThePathASkipLeftDoesNotCarryOverToTheNextStart) {
  TestCore core = Ready();
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({Lane(1), Lane(5)}, "fp");
  ASSERT_EQ(core.OnAction(kMowing + "skip_path"), ActionResult::kDone);
  ASSERT_EQ(core.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  ASSERT_FALSE(core.Mowing());
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({Lane(1), Lane(5)}, "fp");
  ASSERT_EQ(core.RobotState().current_path, 1);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kApproach);
  core.OnLegEnded(LegEnd::kNoWay, 0, kRefused);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
}

// A pause by hand with a good position stays as it is: the wait for GPS changes no actions.
TEST(LogicMowing, AManualPauseWithAGoodPositionChangesNoActions) {
  TestCore core = Following();
  ASSERT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
  core.TakeActionsChanged();
  for (double t = 0; t <= 30; t += 1) core.OnTick(t);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_FALSE(core.TakeActionsChanged());
}

namespace {

ObstacleSample InTheWay(const char* kind = "OBJECT", bool in_view = true, double ahead = 0.7) {
  return {true, {{7, kind, true, in_view, ahead, 2, 0.0, 0.0, 0.0, 0.0}}};
}

const ObstacleSample kFree{true, {}};

}  // namespace

TEST(LogicObstacles, AThingInTheWayPausesWithTheBladeOffAndItGoesOnOnceSeenFree) {
  TestCore core = Following();
  core.SetObstacleMode(2, 2);
  EXPECT_TRUE(core.GateWanted());
  EXPECT_TRUE(core.GateGo());
  core.OnJobProgress(2);
  core.OnObstacleStatus(InTheWay("ANIMAL"), 100);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_FALSE(core.GateGo());
  EXPECT_TRUE(core.TakeDriveCancel());
  EXPECT_TRUE(core.TakeStopRequest());
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["state"], "PAUSED");
  EXPECT_EQ(events[0].details["reason"], "OBSTACLE");
  EXPECT_EQ(events[0].details["kind"], "ANIMAL");
  EXPECT_EQ(events[0].details["resume"], "auto");
  EXPECT_EQ(core.RobotState().pause_reason, "OBSTACLE");
  EXPECT_EQ(core.RobotState().pause_kind, "ANIMAL");
  const auto notes = core.TakeObstacleNotes();
  ASSERT_EQ(notes.size(), 1u);
  EXPECT_EQ(notes[0].track_id, 7u);
  EXPECT_EQ(notes[0].action, "stopped");
  core.OnLegEnded(LegEnd::kCanceled, 3, Reason("CANCELED", ""));
  // No leg starts during the hold, also not after the tick.
  core.OnTick(101);
  EXPECT_EQ(core.NextJobStep(101).kind, JobStep::Kind::kNone);
  for (double t = 100.1; t < 110.05; t += 0.1) core.OnObstacleStatus(kFree, t);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  core.OnObstacleStatus(kFree, 110.2);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_TRUE(core.GateGo());
  core.OnPose(PoseAt(2.5, 2));
  const JobStep step = core.NextJobStep(110.2);
  ASSERT_EQ(step.kind, JobStep::Kind::kFollow);
  EXPECT_EQ(step.start_index, 3u);
}

TEST(LogicObstacles, ShowOnlyNotesAndNoPlatformProofMeansShow) {
  TestCore core = Following();
  core.SetObstacleMode(1, 2);
  EXPECT_FALSE(core.GateWanted());
  core.OnObstacleStatus(InTheWay(), 0);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  const auto notes = core.TakeObstacleNotes();
  ASSERT_EQ(notes.size(), 1u);
  EXPECT_EQ(notes[0].action, "would_stop");
  core.SetObstacleMode(2, 1);
  core.OnObstacleStatus(InTheWay(), 0.1);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_TRUE(core.GateGo());
}

TEST(LogicObstacles, ContinueGoesPastWhatNoSensorSeesOnly) {
  TestCore core = Following();
  core.SetObstacleMode(2, 2);
  core.OnObstacleStatus(InTheWay("OBJECT", true), 0);
  ASSERT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_TRUE(Enabled(core, "mower_logic:mowing", "pause"));
  EXPECT_TRUE(Enabled(core, "mower_logic:mowing", "continue"));
  core.TakeEvents();
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kRejected);
  const auto refused = core.TakeEvents();
  ASSERT_EQ(refused.size(), 1u);
  EXPECT_EQ(refused[0].type, "MOWING_REFUSED");
  EXPECT_EQ(refused[0].details["reason"], "OBSTACLE");
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  // Right at the nose, under the view: the user looked and lets it go on.
  core.OnObstacleStatus(InTheWay("OBJECT", false, 0.1), 0.1);
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  core.OnObstacleStatus(InTheWay("OBJECT", false, 0.1), 0.2);
  EXPECT_EQ(core.RobotState().state, "MOWING");
}

TEST(LogicObstacles, ManualPauseAndObstacleTakeTurns) {
  TestCore core = Following();
  core.SetObstacleMode(2, 2);
  EXPECT_EQ(core.OnAction(kMowing + "pause"), ActionResult::kDone);
  core.OnObstacleStatus(InTheWay(), 0);
  core.TakeEvents();
  // The user wanted the pause: robot_state names it, not the obstacle.
  EXPECT_EQ(core.RobotState().pause_reason, "MANUAL");
  EXPECT_EQ(core.RobotState().pause_kind, "");
  for (double t = 0.1; t < 10.3; t += 0.1) core.OnObstacleStatus(kFree, t);
  // The obstacle is gone, the pause of the user stays.
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_EQ(core.OnAction(kMowing + "continue"), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_EQ(core.RobotState().pause_reason, "");
}

namespace {

// An object 0.1 m round at (x, y) in the stop zone, and a core avoiding objects with the blade on.
ObstacleSample ObjectAt(double x, double y, uint32_t id = 7, const char* kind = "OBJECT") {
  return {true, {{id, kind, true, true, 0.7, 2, x, y, 0.1, 0.05}}};
}

TestCore Avoiding() {
  LogicConfig config;
  config.mow_with_blade = true;
  TestCore core = Following(config);
  core.SetObstacleMode(3, 3);
  core.OnJobProgress(1);
  return core;
}

// Holds the mower at the object from t on until it stood for the avoid wait; the leg ends canceled at pose 1.
void StandBefore(TestCore& core, double t, const ObstacleSample& sample) {
  core.OnObstacleStatus(sample, t);
  core.OnLegEnded(LegEnd::kCanceled, 1, Reason("CANCELED", ""));
  for (double s = t + 0.1; s < t + 3.0 - 1e-6; s += 0.1) {
    core.OnObstacleStatus(sample, s);
    EXPECT_EQ(core.NextJobStep(s).kind, JobStep::Kind::kNone);
  }
  core.OnObstacleStatus(sample, t + 3.0);
}

CutAnswer Pieces(std::vector<JobPiece> pieces) {
  CutAnswer answer;
  answer.ok = true;
  answer.pieces = std::move(pieces);
  return answer;
}

}  // namespace

TEST(LogicAvoid, AnObjectThatStandsIsDrivenAroundWithTheBladeOn) {
  TestCore core = Avoiding();
  StandBefore(core, 100, ObjectAt(2.0, 2.0));
  ASSERT_EQ(core.RobotState().state, "PAUSED");
  core.TakeEvents();
  core.TakeObstacleNotes();
  const JobStep cut = core.NextJobStep(103.0);
  ASSERT_EQ(cut.kind, JobStep::Kind::kCut);
  ASSERT_EQ(cut.circles.size(), 1u);
  EXPECT_NEAR(cut.circles[0].r, 0.5, 1e-9);
  EXPECT_EQ(cut.paths.size(), 2u);
  EXPECT_EQ(cut.start_index, 1u);
  EXPECT_FALSE(cut.area_id.empty());
  // Nothing else while the planner cuts.
  EXPECT_EQ(core.NextJobStep(103.1).kind, JobStep::Kind::kNone);
  // Pose 1 of the first path is left out (0.5 m).
  core.OnCutAnswer(Pieces({{0, 2, 4}, {1, 0, 4}}));
  EXPECT_EQ(core.RobotState().state, "MOWING");
  EXPECT_EQ(core.Circles().size(), 1u);
  const auto notes = core.TakeObstacleNotes();
  ASSERT_EQ(notes.size(), 1u);
  EXPECT_EQ(notes[0].action, "avoided");
  EXPECT_EQ(notes[0].track_id, 7u);
  EXPECT_NEAR(notes[0].keep_out_r, 0.5, 1e-9);
  EXPECT_NEAR(notes[0].left_out, 0.5, 1e-9);
  EXPECT_FALSE(notes[0].area_id.empty());
  // The object in its circle holds no more.
  core.OnObstacleStatus(ObjectAt(2.0, 2.0), 103.1);
  EXPECT_EQ(core.RobotState().state, "MOWING");
  const JobStep detour = core.NextJobStep(103.1);
  ASSERT_EQ(detour.kind, JobStep::Kind::kDetour);
  EXPECT_DOUBLE_EQ(detour.goal.x, 2.0);
  EXPECT_EQ(detour.area_id, cut.area_id);
  EXPECT_TRUE(core.BladeWanted());
  EXPECT_TRUE(core.FollowerActive());
  core.OnLegEnded(LegEnd::kDone, 0, Reason("DONE", ""));
  const JobStep piece = core.NextJobStep(110);
  ASSERT_EQ(piece.kind, JobStep::Kind::kFollow);
  EXPECT_EQ(piece.start_index, 2u);
  EXPECT_EQ(piece.path.poses.size(), 5u);
  core.OnLegEnded(LegEnd::kDone, 4, Reason("DONE", ""));
  EXPECT_EQ(core.RobotState().current_path, 1);
  // The next piece follows on without poses left out: no detour.
  EXPECT_EQ(core.NextJobStep(120).kind, JobStep::Kind::kFollow);
}

TEST(LogicAvoid, APieceEndsBeforeItsPathAndTheNextOneComesAfterADetour) {
  TestCore core = Avoiding();
  StandBefore(core, 100, ObjectAt(2.0, 2.0));
  ASSERT_EQ(core.NextJobStep(103).kind, JobStep::Kind::kCut);
  core.OnCutAnswer(Pieces({{0, 1, 3}, {1, 2, 4}}));
  ASSERT_EQ(core.NextJobStep(103.1).kind, JobStep::Kind::kDetour);
  core.OnLegEnded(LegEnd::kDone, 0, Reason("DONE", ""));
  const JobStep first = core.NextJobStep(104);
  ASSERT_EQ(first.kind, JobStep::Kind::kFollow);
  // The piece ends at pose 3.
  EXPECT_EQ(first.path.poses.size(), 4u);
  core.OnLegEnded(LegEnd::kDone, 3, Reason("DONE", ""));
  EXPECT_EQ(core.RobotState().current_path, 0);
  const JobStep detour = core.NextJobStep(110);
  ASSERT_EQ(detour.kind, JobStep::Kind::kDetour);
  EXPECT_EQ(core.RobotState().current_path, 1);
  EXPECT_DOUBLE_EQ(detour.goal.x, 2.0);
}

TEST(LogicAvoid, APersonOrAnimalIsWaitedFor) {
  for (const char* kind : {"PERSON", "ANIMAL"}) {
    TestCore core = Avoiding();
    StandBefore(core, 100, ObjectAt(2.0, 2.0, 7, kind));
    for (double t = 103; t < 400; t += 10) {
      core.OnObstacleStatus(ObjectAt(2.0, 2.0, 7, kind), t);
      EXPECT_EQ(core.NextJobStep(t).kind, JobStep::Kind::kNone) << kind;
    }
    EXPECT_EQ(core.RobotState().state, "PAUSED");
  }
}

TEST(LogicAvoid, WhatChangedWhileThePlannerCutIsNotDrivenAround) {
  TestCore core = Avoiding();
  StandBefore(core, 100, ObjectAt(2.0, 2.0));
  ASSERT_EQ(core.NextJobStep(103).kind, JobStep::Kind::kCut);
  // A person steps into the stop zone while the planner cuts.
  ObstacleSample both = ObjectAt(2.0, 2.0);
  both.tracks.push_back(ObjectAt(2.0, 2.5, 8, "PERSON").tracks[0]);
  core.OnObstacleStatus(both, 103.05);
  core.OnCutAnswer(Pieces({{0, 2, 4}, {1, 0, 4}}));
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_TRUE(core.Circles().empty());
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_EQ(core.NextJobStep(103.1).kind, JobStep::Kind::kNone);
  // Once the person is gone the object alone is asked for again.
  core.OnObstacleStatus(ObjectAt(2.0, 2.0), 103.2);
  EXPECT_EQ(core.NextJobStep(103.2).kind, JobStep::Kind::kCut);
}

TEST(LogicAvoid, WithoutAWayOrACutOfThePlannerItWaitsLikeStop) {
  CutAnswer missing;
  missing.code = "PLANNER_MISSING";
  for (const CutAnswer& answer : {missing, Pieces({})}) {
    TestCore core = Avoiding();
    StandBefore(core, 100, ObjectAt(2.0, 2.0));
    ASSERT_EQ(core.NextJobStep(103).kind, JobStep::Kind::kCut);
    core.OnCutAnswer(answer);
    EXPECT_EQ(core.RobotState().state, "PAUSED");
    EXPECT_TRUE(core.Circles().empty());
    // Not asked again in this hold.
    core.OnObstacleStatus(ObjectAt(2.0, 2.0), 110);
    EXPECT_EQ(core.NextJobStep(110).kind, JobStep::Kind::kNone);
    EXPECT_FALSE(core.BladeWanted());
  }
}

TEST(LogicAvoid, ASearchLimitIsAskedOnceMore) {
  TestCore core = Avoiding();
  StandBefore(core, 100, ObjectAt(2.0, 2.0));
  CutAnswer limit;
  limit.code = "SEARCH_LIMIT";
  ASSERT_EQ(core.NextJobStep(103).kind, JobStep::Kind::kCut);
  core.OnCutAnswer(limit);
  ASSERT_EQ(core.NextJobStep(103.5).kind, JobStep::Kind::kCut);
  core.OnCutAnswer(limit);
  EXPECT_EQ(core.NextJobStep(104).kind, JobStep::Kind::kNone);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
}

TEST(LogicAvoid, ACircleSeenFreeCutsThePlanAnewAndWithoutCirclesItIsMowedAsItIs) {
  TestCore core = Avoiding();
  StandBefore(core, 100, ObjectAt(2.0, 2.0));
  ASSERT_EQ(core.NextJobStep(103).kind, JobStep::Kind::kCut);
  core.OnCutAnswer(Pieces({{0, 2, 4}, {1, 2, 4}}));
  ASSERT_EQ(core.NextJobStep(103.1).kind, JobStep::Kind::kDetour);
  core.OnLegEnded(LegEnd::kDone, 0, Reason("DONE", ""));
  // Its place seen free while the piece is mowed.
  ASSERT_EQ(core.NextJobStep(104).kind, JobStep::Kind::kFollow);
  core.OnObstacleStatus(ObjectAt(2.0, 2.0), 104.1);
  core.OnObstacleStatus(kFree, 105);
  EXPECT_TRUE(core.Circles().empty());
  core.TakeObstacleNotes();
  core.OnLegEnded(LegEnd::kDone, 4, Reason("DONE", ""));
  // No circle left: the second path whole, from its start.
  const JobStep next = core.NextJobStep(106);
  ASSERT_EQ(next.kind, JobStep::Kind::kFollow);
  EXPECT_EQ(next.start_index, 0u);
  EXPECT_EQ(next.path.poses.size(), 5u);
}

TEST(LogicAvoid, NothingOfThePlanLeftEndsTheArea) {
  TestCore core = Avoiding();
  StandBefore(core, 100, ObjectAt(2.0, 2.0));
  ASSERT_EQ(core.NextJobStep(103).kind, JobStep::Kind::kCut);
  core.OnCutAnswer(Pieces({{0, 2, 4}}));
  ASSERT_EQ(core.NextJobStep(103.1).kind, JobStep::Kind::kDetour);
  core.OnLegEnded(LegEnd::kDone, 0, Reason("DONE", ""));
  ASSERT_EQ(core.NextJobStep(104).kind, JobStep::Kind::kFollow);
  core.OnLegEnded(LegEnd::kDone, 4, Reason("DONE", ""));
  // The second path lies behind the circle: left out, the job goes on with the next area or ends.
  const JobStep next = core.NextJobStep(110);
  EXPECT_NE(next.kind, JobStep::Kind::kFollow);
}

TEST(LogicAvoid, ADetourWithoutAWayStopsAtTheObjectUntilItGoes) {
  TestCore core = Avoiding();
  StandBefore(core, 100, ObjectAt(2.0, 2.0));
  ASSERT_EQ(core.NextJobStep(103).kind, JobStep::Kind::kCut);
  core.OnCutAnswer(Pieces({{0, 2, 4}, {1, 2, 4}}));
  ASSERT_EQ(core.NextJobStep(103.1).kind, JobStep::Kind::kDetour);
  core.OnLegEnded(LegEnd::kNoWay, 0, Reason("TRANSIT_FAILED", "no way"));
  // No piece is left out silently: the circles go, and the object holds the mower again.
  EXPECT_TRUE(core.Circles().empty());
  EXPECT_EQ(core.RobotState().current_path, 0);
  const JobStep next = core.NextJobStep(104);
  EXPECT_NE(next.kind, JobStep::Kind::kDetour);
  core.OnObstacleStatus(ObjectAt(2.0, 2.0), 104.1);
  ASSERT_EQ(core.RobotState().state, "PAUSED");
  core.OnLegEnded(LegEnd::kCanceled, 2, Reason("CANCELED", ""));
  for (double t = 104.2; t < 110; t += 0.1) {
    core.OnObstacleStatus(ObjectAt(2.0, 2.0), t);
    EXPECT_EQ(core.NextJobStep(t).kind, JobStep::Kind::kNone) << t;
  }
}

TEST(LogicAvoid, AHoldOnTheWayToALaneIsWaitedFor) {
  TestCore core = Avoiding();
  core.OnLegEnded(LegEnd::kDone, 4, Reason("DONE", ""));
  core.OnPose(PoseAt(-3, -3));
  ASSERT_EQ(core.NextJobStep(99).kind, JobStep::Kind::kApproach);
  StandBefore(core, 100, ObjectAt(2.0, 2.0));
  ASSERT_EQ(core.RobotState().state, "PAUSED");
  for (double t = 103; t < 106; t += 0.1) {
    core.OnObstacleStatus(ObjectAt(2.0, 2.0), t);
    EXPECT_EQ(core.NextJobStep(t).kind, JobStep::Kind::kNone) << t;
  }
}

TEST(LogicAvoid, CirclesCountWithinMowingOnly) {
  TestCore core = Avoiding();
  StandBefore(core, 100, ObjectAt(2.0, 2.0));
  ASSERT_EQ(core.NextJobStep(103).kind, JobStep::Kind::kCut);
  core.OnCutAnswer(Pieces({{0, 2, 4}, {1, 0, 4}}));
  ASSERT_EQ(core.Circles().size(), 1u);
  ASSERT_EQ(core.OnAction(kMowing + "abort_mowing"), ActionResult::kDone);
  EXPECT_TRUE(core.Circles().empty());
}

TEST(LogicAvoid, PiecesThatDoNotFitThePlanCountAsNoCut) {
  for (const auto& pieces : std::vector<std::vector<JobPiece>>{
           {{0, 3, 5}}, {{0, 0, 4}}, {{0, 3, 4}, {0, 4, 4}}, {{1, 0, 4}, {0, 2, 4}}, {{2, 0, 4}}, {{0, 2, 3}}}) {
    TestCore core = Avoiding();
    StandBefore(core, 100, ObjectAt(2.0, 2.0));
    ASSERT_EQ(core.NextJobStep(103).kind, JobStep::Kind::kCut);
    core.OnCutAnswer(Pieces(pieces));
    EXPECT_EQ(core.RobotState().state, "PAUSED");
    EXPECT_TRUE(core.Circles().empty());
  }
}
