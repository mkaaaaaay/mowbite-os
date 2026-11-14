#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

constexpr uint8_t kStarting = 1;
constexpr uint8_t kMeasuring = 2;
constexpr uint8_t kFault = 3;

LogicConfig Tron(double range = 1.2) {
  LogicConfig config;
  config.dock_contact_x = -0.10;
  config.marker.range = range;
  return config;
}

HardwareSnapshot Healthy(double t = 0, bool contact = false) {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  hw.dock_contact = contact;
  hw.dock_contact_raw = contact;
  hw.time_s = t;
  return hw;
}

DockMarkerSample Message(double t, uint8_t state, bool valid = false, bool yaw = false, const std::string& reason = "") {
  DockMarkerSample sample;
  sample.time = t;
  sample.state = state;
  sample.valid = valid;
  sample.yaw_valid = yaw;
  sample.reason = reason;
  return sample;
}

DockPoseSample Pose(double t, double x, double y = 0.0, double heading = M_PI, double speed = 0.0,
                    float accuracy = 0.01f) {
  DockPoseSample pose;
  pose.time = t;
  pose.x = x;
  pose.y = y;
  pose.heading = heading;
  pose.heading_valid = true;
  pose.position_accuracy = accuracy;
  pose.speed = speed;
  return pose;
}

// IDLE in front of the station without map and position, safety measured.
TestCore Ready(const LogicConfig& config = Tron()) {
  TestCore core{config};
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.TakeEvents();
  core.TakeNotices();
  return core;
}

CommandOutcome DockingTest(TestCore& core, const Reason& node_problem = Reason()) {
  return core.Command("docking.test", json{{"client", "app"}}, node_problem);
}

const Event* Find(const std::vector<Event>& events, const std::string& type) {
  auto it = std::find_if(events.begin(), events.end(), [&](const Event& e) { return e.type == type; });
  return it == events.end() ? nullptr : &*it;
}

DockingLeg Take(TestCore& core, DockingLeg::Kind kind) {
  auto leg = core.TakeDockingLeg();
  EXPECT_TRUE(leg);
  if (!leg) return DockingLeg();
  EXPECT_EQ(leg->kind, kind) << DockingLegName(leg->kind);
  EXPECT_EQ(leg->frame, LegFrame::kDock);
  EXPECT_NEAR(leg->max_cross_track, 0.10, 1e-9);
  return *leg;
}

// The test accepted and the sight begun at 100 s.
TestCore Started(const LogicConfig& config = Tron()) {
  TestCore core = Ready(config);
  EXPECT_TRUE(DockingTest(core).accepted);
  core.TakeEvents();
  core.OnTick(100.0);
  return core;
}

// The sight sees the marker with base_link at (x, y) facing heading at 100.5 s.
void See(TestCore& core, double x, double y = 0.0, double heading = M_PI) {
  core.OnDockMarker(Message(100.4, kMeasuring, true, true));
  core.OnDockPose(Pose(100.5, x, y, heading));
}

// From the line at 0.3 m out to the approach point and onto the line; the time then is 102 s.
TestCore OnTheLine(const LogicConfig& config = Tron()) {
  TestCore core = Started(config);
  See(core, -0.5);
  Take(core, DockingLeg::Kind::kOut);
  core.OnDockPose(Pose(102.0, -0.8));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  Take(core, DockingLeg::Kind::kLine);
  return core;
}

// Along the line to the gate at 0.1 m/s by the wheels, scale times that by the marker, ending y beside the line.
void DriveTheLine(TestCore& core, double scale = 1.0, double y = 0.0) {
  const double start = -0.25 - scale * 0.55;
  for (int i = 0; i <= 55; i++) {
    core.OnDockPose(Pose(102.1 + 0.1 * i, start + scale * 0.01 * i, y * i / 55.0, M_PI, 0.1));
  }
}

const Event& Failed(TestCore& core) {
  static std::vector<Event> events;
  events = core.TakeEvents();
  const Event* failed = Find(events, "DOCKING_FAILED");
  EXPECT_TRUE(failed);
  static const Event none;
  return failed ? *failed : none;
}

}  // namespace

TEST(LogicMarker, OfferedOnlyWithAMarker) {
  EXPECT_TRUE(TestCore{Tron()}.MarkerSource());
  EXPECT_FALSE(TestCore{Tron(0.0)}.MarkerSource());
}

TEST(LogicMarker, RefusedLikeDockingStart) {
  TestCore none = Ready(Tron(0.0));
  CommandOutcome out = DockingTest(none);
  EXPECT_EQ(out.why.code, "MARKER_UNAVAILABLE");
  EXPECT_EQ(out.data["event"], "DRIVE_REFUSED");
  TestCore on_dock = Ready();
  on_dock.OnHardwareStatus(Healthy(1.0, true));
  EXPECT_EQ(DockingTest(on_dock).why.code, "ALREADY_THERE");
  TestCore emergency = Ready();
  HardwareSnapshot stop = Healthy(1.0);
  stop.emergency = true;
  stop.emergency_reason = kEmergencyStop;
  emergency.OnHardwareStatus(stop);
  EXPECT_EQ(DockingTest(emergency).why.code, "EMERGENCY");
  TestCore unmeasured = Ready();
  unmeasured.OnCapabilities(false);
  EXPECT_EQ(DockingTest(unmeasured).why.code, "NO_SAFETY_MEASUREMENT");
  TestCore recording = Ready();
  recording.OnMapStatus(MapDatum::kMatch, "");
  ASSERT_TRUE(recording.Command("recording.start", json{{"client", "app"}}).accepted);
  EXPECT_EQ(DockingTest(recording).why.code, "BUSY");
  TestCore no_contacts = Ready([] {
    LogicConfig config = Tron();
    config.dock_contact_x = 0;
    return config;
  }());
  EXPECT_EQ(DockingTest(no_contacts).why.code, "NO_CHARGE_CONTACTS");
  TestCore follower = Ready();
  EXPECT_EQ(DockingTest(follower, Reason("FOLLOWER_MISSING", "The follower is not running")).why.code, "FOLLOWER_MISSING");
  for (TestCore* core : {&none, &on_dock, &emergency, &unmeasured, &follower}) {
    EXPECT_FALSE(core->MarkerWanted());
    EXPECT_EQ(core->RobotState().state, "IDLE");
  }
}

// Without map and position: DOCKING with TEST, the sight, out to the approach point, the line, the gate, the close
// part, the blind contact and the charge, every leg in the station frame and only as SOURCE_DOCK.
TEST(LogicMarker, FromTheHandStartToDocked) {
  TestCore core = Ready();
  const CommandOutcome out = DockingTest(core);
  ASSERT_TRUE(out.accepted);
  EXPECT_EQ(out.result["state"], "DOCKING");
  auto events = core.TakeEvents();
  const Event* docking = Find(events, "DOCKING");
  ASSERT_TRUE(docking);
  EXPECT_EQ(docking->details["reason"], "TEST");
  EXPECT_TRUE(core.MarkerWanted());
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnTick(100.0);
  See(core, -0.5, 0.02);
  const DockingLeg out_leg = Take(core, DockingLeg::Kind::kOut);
  EXPECT_FALSE(out_leg.reverse);
  EXPECT_TRUE(core.DockFollowerActive());
  EXPECT_FALSE(core.FollowerActive());
  core.OnDockPose(Pose(102.0, -0.8, 0.02));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  const DockingLeg line = Take(core, DockingLeg::Kind::kLine);
  EXPECT_TRUE(line.reverse);
  DriveTheLine(core, 1.0, 0.01);
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  const DockingLeg close = Take(core, DockingLeg::Kind::kClose);
  EXPECT_NEAR(close.poses.back().x, -0.07, 1e-9);
  core.OnDockPose(Pose(108.0, -0.08));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  const DockingLeg contact = Take(core, DockingLeg::Kind::kContact);
  EXPECT_NEAR(contact.poses.back().x, 0.05, 1e-9);
  for (int i = 0; i <= 6; i++) {
    const double t = 110.0 + i;
    HardwareSnapshot hw = Healthy(t, true);
    hw.battery_current = -3.0f;
    core.OnHardwareStatus(hw);
  }
  events = core.TakeEvents();
  const Event* docked = Find(events, "DOCKED");
  ASSERT_TRUE(docked);
  EXPECT_EQ(docked->details["guide"], "marker");
  EXPECT_EQ(core.RobotState().state, "IDLE");
  EXPECT_FALSE(core.MarkerWanted());
  EXPECT_FALSE(core.DockFollowerActive());
}

TEST(LogicMarker, AtTheApproachPointTheLineFirst) {
  TestCore core = Started();
  See(core, -0.8, -0.02, M_PI + 0.1);
  Take(core, DockingLeg::Kind::kLine);
}

// The pose from the marker may be worse by dock_accuracy_per_m (0.09) per m from the docked place: 0.102 m at 0.8 m,
// 0.039 m at 0.1 m.
TEST(LogicMarker, ThePoseMayBeWorseFartherFromTheStation) {
  const auto leg_after_sight = [](double x, float accuracy) {
    TestCore core = Started();
    core.OnDockMarker(Message(100.4, kMeasuring, true, true));
    core.OnDockPose(Pose(100.5, x, 0.0, M_PI, 0.0, accuracy));
    return core.TakeDockingLeg().has_value();
  };
  EXPECT_TRUE(leg_after_sight(-0.8, 0.1f));
  EXPECT_FALSE(leg_after_sight(-0.8, 0.104f));
  EXPECT_TRUE(leg_after_sight(-0.1, 0.038f));
  EXPECT_FALSE(leg_after_sight(-0.1, 0.04f));
}

TEST(LogicMarker, WithoutGrowthThePoseNeedsTheLimitOfTheStationEverywhere) {
  LogicConfig config = Tron();
  config.dock_accuracy_per_m = 0;
  TestCore core = Started(config);
  core.OnDockMarker(Message(100.4, kMeasuring, true, true));
  core.OnDockPose(Pose(100.5, -0.8, 0.0, M_PI, 0.0, 0.031f));
  EXPECT_FALSE(core.TakeDockingLeg());
}

// The sight: the source cannot measure (at once), images without a marker (after marker_wait), silence.
TEST(LogicMarker, CameraOffIsUnavailable) {
  TestCore core = Started();
  core.OnDockMarker(Message(100.5, kFault, false, false, "NOT_SET_UP"));
  core.OnTick(101.0);
  const Event& failed = Failed(core);
  EXPECT_EQ(failed.details["reason"], "dock_failed");
  EXPECT_EQ(failed.details["cause"], "MARKER_UNAVAILABLE");
  EXPECT_EQ(failed.details["detail"], "NOT_SET_UP");
  EXPECT_EQ(failed.details["guide"], "marker");
  EXPECT_EQ(failed.details["attempts"], 1);
  EXPECT_FALSE(core.MarkerWanted());
  EXPECT_FALSE(core.TakeDockingLeg());
}

TEST(LogicMarker, SilentSourceIsUnavailableWithoutDetail) {
  TestCore core = Started();
  core.OnTick(102.0);
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnTick(103.0);
  const Event& failed = Failed(core);
  EXPECT_EQ(failed.details["cause"], "MARKER_UNAVAILABLE");
  EXPECT_FALSE(failed.details.contains("detail"));
}

TEST(LogicMarker, TooDarkIsNotSeen) {
  TestCore core = Started();
  for (int i = 1; i < 50; i++) {
    const double t = 100.0 + 0.5 * i;
    core.OnDockMarker(Message(t, t < 102.0 ? kStarting : kMeasuring, false, false, t < 102.0 ? "" : "TOO_DARK"));
    core.OnTick(t);
  }
  EXPECT_EQ(core.RobotState().state, "DOCKING");
  core.OnTick(125.1);
  const Event& failed = Failed(core);
  EXPECT_EQ(failed.details["cause"], "MARKER_NOT_SEEN");
  EXPECT_EQ(failed.details["detail"], "TOO_DARK");
}

TEST(LogicMarker, SeenButNeverAccurateIsNotSeenWithItsOwnText) {
  TestCore core = Started();
  for (int i = 1; i < 50; i++) {
    const double t = 100.0 + 0.5 * i;
    core.OnDockMarker(Message(t, kMeasuring, true, true));
    core.OnDockPose(Pose(t, -0.8, 0.0, M_PI, 0.0, 0.2f));
    core.OnTick(t);
  }
  core.OnTick(125.1);
  const Event& failed = Failed(core);
  EXPECT_EQ(failed.details["cause"], "MARKER_NOT_SEEN");
  const std::string message = failed.details["message"].get<std::string>();
  EXPECT_NE(message.find("not accurate enough"), std::string::npos) << message;
  EXPECT_EQ(message.find("saw no marker"), std::string::npos) << message;
}

// A word the catalog does not have stays out of detail.
TEST(LogicMarker, UnknownWordIsNoDetail) {
  TestCore core = Started();
  core.OnDockMarker(Message(100.5, kFault, false, false, "SOMETHING_NEW"));
  core.OnTick(101.0);
  EXPECT_FALSE(Failed(core).details.contains("detail"));
}

// The hand start: the marker without a direction of its own, or the mower beside the line, ends the test at once.
TEST(LogicMarker, MarkerWithoutDirectionRefused) {
  TestCore core = Started();
  for (int i = 1; i < 50; i++) {
    const double t = 100.0 + 0.5 * i;
    core.OnDockMarker(Message(t, kMeasuring, true, false));
    core.OnTick(t);
  }
  core.OnTick(125.1);
  const Event& failed = Failed(core);
  EXPECT_EQ(failed.details["reason"], "dock_failed");
  EXPECT_FALSE(failed.details.contains("cause"));
  EXPECT_NE(failed.details["message"].get<std::string>().find("no direction"), std::string::npos);
}

// A good pose alone ends no sight: the marker has to come with its own direction since the sight began.
TEST(LogicMarker, TheSightNeedsTheDirectionOfTheMarker) {
  TestCore core = Started();
  core.OnDockPose(Pose(100.2, -0.5));
  core.OnTick(100.3);
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnDockMarker(Message(100.4, kMeasuring, true, false));
  core.OnDockPose(Pose(100.5, -0.5));
  core.OnTick(100.6);
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnDockMarker(Message(100.7, kMeasuring, true, true));
  core.OnDockPose(Pose(100.8, -0.5));
  Take(core, DockingLeg::Kind::kOut);
}

// The source reports that it cannot measure while a pose kept from before is still good: no leg, unavailable.
TEST(LogicMarker, AFaultWithAGoodPoseStartsNothing) {
  TestCore core = Started();
  core.OnDockMarker(Message(100.3, kMeasuring, true, true));
  core.OnDockMarker(Message(100.4, kFault, false, false, "NOT_SET_UP"));
  core.OnDockPose(Pose(100.5, -0.5));
  core.OnTick(100.6);
  EXPECT_FALSE(core.TakeDockingLeg());
  const Event& failed = Failed(core);
  EXPECT_EQ(failed.details["cause"], "MARKER_UNAVAILABLE");
  EXPECT_EQ(core.RobotState().state, "IDLE");
}

// A try that fails while the source cannot measure does not drive out by the pose kept from before.
TEST(LogicMarker, NoDriveOutWhileTheSourceCannotMeasure) {
  TestCore core = OnTheLine();
  core.OnDockPose(Pose(103.0, -0.6));
  core.OnDockMarker(Message(103.1, kFault, false, false, "NOT_SET_UP"));
  core.OnDockingLegEnded(LegEnd::kFailed, Reason("TIMEOUT", "stuck"));
  EXPECT_FALSE(core.TakeDockingLeg());
  EXPECT_EQ(Failed(core).details["reason"], "dock_failed");
  EXPECT_EQ(core.RobotState().state, "IDLE");
}

TEST(LogicMarker, BesideTheLineRefused) {
  TestCore core = Started();
  See(core, -0.5, 0.3);
  EXPECT_FALSE(core.TakeDockingLeg());
  const Event& failed = Failed(core);
  EXPECT_NE(failed.details["message"].get<std::string>().find("beside the line"), std::string::npos);
  EXPECT_EQ(failed.details["attempts"], 1);
  EXPECT_EQ(core.RobotState().state, "IDLE");
}

// Lost on the line: the mower stands and waits; back for gps_wait_time it goes on from there, else MARKER_LOST.
TEST(LogicMarker, LostOnTheLineGoesOnWhenBack) {
  TestCore core = OnTheLine();
  core.OnDockPose(Pose(103.0, -0.6));
  core.OnDockingLegEnded(LegEnd::kGps, Reason("POSITION_ACCURACY", "the pose is not good"));
  EXPECT_FALSE(core.DockFollowerActive());
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnTick(110.0);
  for (int i = 0; i < 10; i++) {
    const double t = 111.0 + i;
    core.OnDockPose(Pose(t, -0.6));
    core.OnTick(t);
    EXPECT_FALSE(core.TakeDockingLeg()) << t;
  }
  core.OnDockPose(Pose(121.0, -0.6));
  core.OnTick(121.0);
  Take(core, DockingLeg::Kind::kOut);
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING_RETRY"));
}

TEST(LogicMarker, LostForGoodIsMarkerLost) {
  TestCore core = OnTheLine();
  core.OnDockingLegEnded(LegEnd::kGps, Reason("POSITION_ACCURACY", "the pose is not good"));
  core.OnTick(110.0);
  for (int i = 1; i < 50; i++) {
    const double t = 110.0 + 0.5 * i;
    core.OnDockMarker(Message(t, kMeasuring, false, false, "NO_MARKER"));
    core.OnTick(t);
  }
  EXPECT_EQ(core.RobotState().state, "DOCKING");
  core.OnTick(135.1);
  const Event& failed = Failed(core);
  EXPECT_EQ(failed.details["cause"], "MARKER_LOST");
  EXPECT_EQ(failed.details["detail"], "NO_MARKER");
  EXPECT_FALSE(core.TakeDockingLeg());
}

// The gate: beside the line the try drives out by the marker and comes again; a marker of the wrong size ends it there.
TEST(LogicMarker, GateBesideTheLineTriesAgain) {
  TestCore core = OnTheLine();
  DriveTheLine(core, 1.0, 0.03);
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  Take(core, DockingLeg::Kind::kOut);
  core.OnDockPose(Pose(108.0, -0.8));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  const auto events = core.TakeEvents();
  const Event* retry = Find(events, "DOCKING_RETRY");
  ASSERT_TRUE(retry);
  EXPECT_EQ(retry->details["guide"], "marker");
  EXPECT_NE(retry->details["message"].get<std::string>().find("beside the line"), std::string::npos);
  Take(core, DockingLeg::Kind::kLine);
}

TEST(LogicMarker, WrongMarkerSizeEndsAtTheGate) {
  TestCore core = OnTheLine();
  DriveTheLine(core, 1.3);
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  EXPECT_FALSE(core.TakeDockingLeg());
  const Event& failed = Failed(core);
  EXPECT_NE(failed.details["message"].get<std::string>().find("right size"), std::string::npos);
  EXPECT_EQ(core.RobotState().state, "IDLE");
}

// The scale counts before the rest of the gate: beside the line as well, a marker of the wrong size guides no way out.
TEST(LogicMarker, WrongMarkerSizeBesideTheLineEndsAtTheGate) {
  TestCore core = OnTheLine();
  DriveTheLine(core, 1.3, 0.03);
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  EXPECT_FALSE(core.TakeDockingLeg());
  const Event& failed = Failed(core);
  EXPECT_NE(failed.details["message"].get<std::string>().find("right size"), std::string::npos);
  EXPECT_EQ(core.RobotState().state, "IDLE");
}

// A failed try without a good pose of the marker drives nowhere.
TEST(LogicMarker, NoDriveOutWithoutAGoodPose) {
  TestCore core = OnTheLine();
  core.OnDockPose(Pose(103.0, -0.6, 0.0, M_PI, 0.0, 0.5f));
  core.OnDockingLegEnded(LegEnd::kFailed, Reason("TIMEOUT", "stuck"));
  EXPECT_FALSE(core.TakeDockingLeg());
  EXPECT_EQ(Failed(core).details["reason"], "dock_failed");
}

TEST(LogicMarker, AbortEndsTheTest) {
  TestCore core = OnTheLine();
  ASSERT_TRUE(core.Command("docking.abort", json{{"client", "app"}}).accepted);
  EXPECT_FALSE(core.MarkerWanted());
  EXPECT_TRUE(core.TakeDriveCancel());
}

// The map pose has nothing to say while the test runs.
TEST(LogicMarker, MapPoseLossDoesNotStopIt) {
  TestCore core = OnTheLine();
  core.OnPoseTimeout();
  EXPECT_FALSE(core.TakeDriveCancel());
  EXPECT_TRUE(core.DockFollowerActive());
}

// Docking by the map stays on follower/cmd_vel, and its events say so.
TEST(LogicMarker, DockingByTheMapIsNoDockSource) {
  TestCore core = Ready();
  core.OnMapStatus(MapDatum::kMatch, "");
  PoseSnapshot pose;
  pose.x = 2.0;
  pose.heading = M_PI;
  pose.position_accuracy = 0.02f;
  pose.orientation_accuracy = 0.01f;
  pose.orientation_valid = true;
  core.OnPose(pose);
  core.OnMowMap({{"lawn", "Lawn", true}}, Reason());
  core.OnMapDock(DockPose{-1.0, 0.0, M_PI});
  core.OnDrivableAreas({{{-0.5, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.5, 2.0}}}, {});
  ASSERT_TRUE(core.Command("docking.start", json{{"client", "app"}}).accepted);
  const auto leg = core.TakeDockingLeg();
  ASSERT_TRUE(leg);
  EXPECT_EQ(leg.value_or(DockingLeg()).frame, LegFrame::kMap);
  EXPECT_TRUE(core.FollowerActive());
  EXPECT_FALSE(core.DockFollowerActive());
  EXPECT_FALSE(core.MarkerWanted());
  core.OnDockingLegEnded(LegEnd::kFailed, Reason("TIMEOUT", "stuck"));
  const auto events = core.TakeEvents();
  const Event* retry = Find(events, "DOCKING_RETRY");
  ASSERT_TRUE(retry);
  EXPECT_EQ(retry->details["guide"], "position");
}
