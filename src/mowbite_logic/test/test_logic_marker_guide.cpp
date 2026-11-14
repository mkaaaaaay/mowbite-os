#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

constexpr uint8_t kMeasuring = 2;
constexpr uint8_t kFault = 3;

// The station of the map: backing in going west, the mower faces east on the contacts. The line by the marker begins
// 0.8 m east of it at (-0.2, 0), the gate is at (-0.75, 0). In the station frame base_link at (x, y) of the map is at
// (-1 - x, -y), facing its heading minus pi.
const DockPose kDock{-1.0, 0.0, M_PI};
const Ring kLawn{{-0.5, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.5, 2.0}};

LogicConfig Tron(DockGuide guide = DockGuide::kRequire) {
  LogicConfig config;
  config.dock_contact_x = -0.10;
  config.marker.range = 1.2;
  config.marker.guide = guide;
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

PoseSnapshot Map(double x, double y, double heading = 0.0, float accuracy = 0.02f) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.heading = heading;
  pose.position_accuracy = accuracy;
  pose.orientation_accuracy = 0.01f;
  pose.orientation_valid = true;
  return pose;
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

// base_link in the station frame by the marker, at (x, y) facing heading in the map, with the real station shift m
// north of the recorded one.
DockPoseSample ByMarker(double t, double x, double y, double heading = 0.0, double shift = 0.0, double speed = 0.0) {
  DockPoseSample pose;
  pose.time = t;
  pose.x = -1.0 - x;
  pose.y = shift - y;
  pose.heading = std::remainder(heading - M_PI, 2 * M_PI);
  pose.heading_valid = true;
  pose.position_accuracy = 0.01f;
  pose.speed = speed;
  return pose;
}

const Event* Find(const std::vector<Event>& events, const std::string& type) {
  auto it = std::find_if(events.begin(), events.end(), [&](const Event& e) { return e.type == type; });
  return it == events.end() ? nullptr : &*it;
}

DockingLeg Take(TestCore& core, DockingLeg::Kind kind, LegFrame frame) {
  auto leg = core.TakeDockingLeg();
  EXPECT_TRUE(leg);
  if (!leg) return DockingLeg();
  EXPECT_EQ(leg->kind, kind) << DockingLegName(leg->kind);
  EXPECT_EQ(leg->frame, frame);
  return *leg;
}

// IDLE 2 m east of the station with map, lawn and station, safety measured; docking.start sent, the approach taken
// (its goal in approach, if given).
TestCore Approaching(const LogicConfig& config = Tron(), DockPose* approach = nullptr) {
  TestCore core{config};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(Map(2.0, 0.5, M_PI));
  core.OnMowMap({{"lawn", "Lawn", true}}, Reason());
  core.OnMapDock(kDock);
  core.OnDrivableAreas({kLawn}, {});
  EXPECT_TRUE(core.Command("docking.start", json{{"client", "app"}}).accepted);
  const DockingLeg leg = Take(core, DockingLeg::Kind::kApproach, LegFrame::kMap);
  if (approach) *approach = leg.goal;
  core.TakeEvents();
  return core;
}

// At the approach point after the approach, map heading as given; the sight begins at 100 s.
TestCore AtTheApproachPoint(const LogicConfig& config = Tron(), float accuracy = 0.02f) {
  TestCore core = Approaching(config);
  core.OnPose(Map(-0.2, 0.0, 0.0, accuracy));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  core.OnTick(100.0);
  return core;
}

// The marker seen at the approach point without a direction of its own, the station shift m off the recorded one.
void See(TestCore& core, double shift = 0.0) {
  core.OnDockMarker(Message(100.4, kMeasuring, true, false));
  core.OnDockPose(ByMarker(100.5, -0.2, 0.0, 0.0, shift));
}

// Along the line to the gate at 0.1 m/s, map and marker agreeing; the map heading at the gate as given.
void DriveTheLine(TestCore& core, double gate_heading = 0.0) {
  for (int i = 0; i <= 55; i++) {
    const double x = -0.2 - 0.01 * i;
    const double t = 101.1 + 0.1 * i;
    core.OnPose(Map(x, 0.0, i == 55 ? gate_heading : 0.0));
    core.OnDockPose(ByMarker(t, x, 0.0, 0.0, 0.0, 0.1));
  }
}

void Charge(TestCore& core, double from) {
  for (int i = 0; i <= 6; i++) {
    HardwareSnapshot hw = Healthy(from + i, true);
    hw.battery_current = -3.0f;
    core.OnHardwareStatus(hw);
  }
}

}  // namespace

// The line begins where the camera finds the marker: at most marker_range in front of the station.
TEST(LogicMarkerGuide, TheApproachEndsWithinRange) {
  LogicConfig config = Tron();
  config.dock.approach_distance = 2.0;
  DockPose approach;
  TestCore core = Approaching(config, &approach);
  EXPECT_NEAR(approach.x, 0.2, 1e-9);
  core.OnPose(Map(0.2, 0.0));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  core.OnTick(100.0);
  core.OnDockMarker(Message(100.4, kMeasuring, true, false));
  core.OnDockPose(ByMarker(100.5, 0.2, 0.0));
  const DockingLeg line = Take(core, DockingLeg::Kind::kLine, LegFrame::kDock);
  EXPECT_NEAR(line.poses.front().x, -1.2, 1e-9);
  LogicConfig off = Tron(DockGuide::kNone);
  off.dock.approach_distance = 2.0;
  TestCore by_map = Approaching(off, &approach);
  EXPECT_NEAR(approach.x, 1.0, 1e-9);
  by_map.OnPose(Map(1.0, 0.0));
  by_map.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  EXPECT_NEAR(Take(by_map, DockingLeg::Kind::kLine, LegFrame::kMap).poses.front().x, 1.0, 1e-9);
}

// From kMarkerRequestDistance before the approach point on, with the heading of the map as the hint; never the hint in
// docking.test, nor without a good position.
TEST(LogicMarkerGuide, AsksForTheMarkerNearTheApproachPoint) {
  TestCore core = Approaching();
  EXPECT_FALSE(core.MarkerWanted());
  core.OnPose(Map(3.0, 0.0, M_PI));
  EXPECT_FALSE(core.MarkerWanted());
  core.OnPose(Map(2.7, 0.0, M_PI - 0.1));
  EXPECT_TRUE(core.MarkerWanted());
  ASSERT_TRUE(core.DockHeadingHint());
  EXPECT_NEAR(core.DockHeadingHint().value_or(0.0), -0.1, 1e-9);
  EXPECT_FALSE(core.DockFollowerActive());
  core.OnPose(Map(2.6, 0.0, M_PI, 0.5f));
  EXPECT_FALSE(core.DockHeadingHint());
  EXPECT_TRUE(core.MarkerWanted());
  TestCore off = Approaching(Tron(DockGuide::kNone));
  off.OnPose(Map(0.0, 0.0, M_PI));
  EXPECT_FALSE(off.MarkerWanted());
  EXPECT_FALSE(off.DockHeadingHint());
}

// The approach by the map, the sight at the approach point (the marker without a direction of its own is enough here),
// the line, the gate, the close part and the contact by the marker, DOCKED with guide marker.
TEST(LogicMarkerGuide, DocksByTheMarker) {
  TestCore core = AtTheApproachPoint();
  EXPECT_TRUE(core.MarkerWanted());
  EXPECT_FALSE(core.TakeDockingLeg());
  See(core);
  const DockingLeg line = Take(core, DockingLeg::Kind::kLine, LegFrame::kDock);
  EXPECT_TRUE(line.reverse);
  EXPECT_TRUE(core.DockFollowerActive());
  EXPECT_FALSE(core.FollowerActive());
  DriveTheLine(core);
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  Take(core, DockingLeg::Kind::kClose, LegFrame::kDock);
  core.OnDockPose(ByMarker(108.0, -0.92, 0.0));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  Take(core, DockingLeg::Kind::kContact, LegFrame::kDock);
  Charge(core, 110.0);
  const auto events = core.TakeEvents();
  const Event* docked = Find(events, "DOCKED");
  ASSERT_TRUE(docked);
  EXPECT_EQ(docked->details["guide"], "marker");
  EXPECT_FALSE(Find(events, "DOCKING_WITHOUT_MARKER"));
  EXPECT_FALSE(core.MarkerWanted());
}

// The station by the marker 5 cm off the recorded one: within the limit; 20 cm: MARKER_MISMATCH, nothing drives, docking
// ends at once.
TEST(LogicMarkerGuide, TheMarkerHasToFitTheMap) {
  TestCore near = AtTheApproachPoint();
  See(near, 0.05);
  Take(near, DockingLeg::Kind::kLine, LegFrame::kDock);
  TestCore far = AtTheApproachPoint();
  See(far, 0.20);
  EXPECT_FALSE(far.TakeDockingLeg());
  const auto events = far.TakeEvents();
  const Event* failed = Find(events, "DOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "dock_failed");
  EXPECT_EQ(failed->details["cause"], "MARKER_MISMATCH");
  EXPECT_EQ(failed->details["guide"], "marker");
  EXPECT_EQ(failed->details["attempts"], 1);
  EXPECT_FALSE(Find(events, "DOCKING_RETRY"));
  EXPECT_EQ(far.RobotState().state, "IDLE");
}

// The accuracy of the map widens the limit: the map 0.18 m off with a position of 0.09 m is no mismatch, with one of
// 0.02 m it is.
TEST(LogicMarkerGuide, TheLimitGrowsWithTheAccuracyOfTheMap) {
  TestCore core = AtTheApproachPoint(Tron(), 0.09f);
  core.OnPose(Map(-0.2, 0.18, 0.0, 0.09f));
  See(core);
  Take(core, DockingLeg::Kind::kLine, LegFrame::kDock);
  TestCore exact = AtTheApproachPoint();
  exact.OnPose(Map(-0.2, 0.18));
  See(exact);
  EXPECT_FALSE(exact.TakeDockingLeg());
  EXPECT_EQ(exact.RobotState().state, "IDLE");
}

// Without a good position of the map the marker guides alone, there is nothing to compare.
TEST(LogicMarkerGuide, WithoutAGoodMapPoseNoComparison) {
  TestCore core = AtTheApproachPoint(Tron(), 0.5f);
  core.OnPose(Map(-0.2, 0.5, 0.0, 0.5f));
  See(core);
  Take(core, DockingLeg::Kind::kLine, LegFrame::kDock);
}

// At the gate also the heading: 11.5 degrees off ends docking there, nothing drives.
TEST(LogicMarkerGuide, AtTheGateTheHeadingHasToFitToo) {
  TestCore core = AtTheApproachPoint();
  See(core);
  Take(core, DockingLeg::Kind::kLine, LegFrame::kDock);
  DriveTheLine(core, 0.2);
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  EXPECT_FALSE(core.TakeDockingLeg());
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "DOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["cause"], "MARKER_MISMATCH");
  EXPECT_NE(failed->details["message"].get<std::string>().find("deg off"), std::string::npos);
  EXPECT_EQ(core.RobotState().state, "IDLE");
}

// prefer: the source cannot measure at the sight, the try docks by the position with DOCKING_WITHOUT_MARKER.
TEST(LogicMarkerGuide, PreferDocksByThePositionWhereTheCameraCannotMeasure) {
  TestCore core = AtTheApproachPoint(Tron(DockGuide::kPrefer));
  core.OnDockMarker(Message(100.5, kFault, false, false, "NOT_SET_UP"));
  core.OnTick(101.0);
  auto events = core.TakeEvents();
  const Event* without = Find(events, "DOCKING_WITHOUT_MARKER");
  ASSERT_TRUE(without);
  EXPECT_EQ(without->details["cause"], "MARKER_UNAVAILABLE");
  EXPECT_EQ(without->details["detail"], "NOT_SET_UP");
  EXPECT_EQ(without->details["severity"], "warning");
  EXPECT_EQ(without->details["resume"], "auto");
  EXPECT_FALSE(Find(events, "DOCKING_RETRY"));
  Take(core, DockingLeg::Kind::kLine, LegFrame::kMap);
  EXPECT_FALSE(core.MarkerWanted());
  EXPECT_FALSE(core.DockFollowerActive());
  EXPECT_TRUE(core.FollowerActive());
  core.OnPose(Map(-0.74, 0.01));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  Take(core, DockingLeg::Kind::kContact, LegFrame::kMap);
  Charge(core, 110.0);
  events = core.TakeEvents();
  const Event* docked = Find(events, "DOCKED");
  ASSERT_TRUE(docked);
  EXPECT_EQ(docked->details["guide"], "position");
}

// require: the same try fails, nothing drives, the next try asks for the marker again.
TEST(LogicMarkerGuide, RequireFailsTheTryWhereTheCameraCannotMeasure) {
  TestCore core = AtTheApproachPoint();
  core.OnDockMarker(Message(100.5, kFault, false, false, "NOT_SET_UP"));
  core.OnTick(101.0);
  const auto events = core.TakeEvents();
  EXPECT_FALSE(Find(events, "DOCKING_WITHOUT_MARKER"));
  const Event* retry = Find(events, "DOCKING_RETRY");
  ASSERT_TRUE(retry);
  EXPECT_EQ(retry->details["cause"], "MARKER_UNAVAILABLE");
  EXPECT_EQ(retry->details["guide"], "marker");
  EXPECT_FALSE(core.TakeDockingLeg());
  EXPECT_TRUE(core.MarkerWanted());
  core.OnTick(102.0);
  core.OnDockMarker(Message(102.4, kMeasuring, true, false));
  core.OnDockPose(ByMarker(102.5, -0.2, 0.0));
  Take(core, DockingLeg::Kind::kLine, LegFrame::kDock);
}

// The camera runs but sees no marker: never by the position, also under prefer.
TEST(LogicMarkerGuide, NotSeenNeverDocksByThePosition) {
  TestCore core = AtTheApproachPoint(Tron(DockGuide::kPrefer));
  for (int i = 1; i < 50; i++) {
    const double t = 100.0 + 0.5 * i;
    core.OnDockMarker(Message(t, kMeasuring, false, false, "TOO_DARK"));
    core.OnTick(t);
  }
  core.OnTick(125.1);
  const auto events = core.TakeEvents();
  EXPECT_FALSE(Find(events, "DOCKING_WITHOUT_MARKER"));
  const Event* retry = Find(events, "DOCKING_RETRY");
  ASSERT_TRUE(retry);
  EXPECT_EQ(retry->details["cause"], "MARKER_NOT_SEEN");
  EXPECT_EQ(retry->details["detail"], "TOO_DARK");
  EXPECT_FALSE(core.TakeDockingLeg());
  EXPECT_TRUE(core.MarkerWanted());
}

// The marker lost on the line for good: out to the approach point by the map, then the next try with a sight.
TEST(LogicMarkerGuide, LostOnTheLineDrivesOutByTheMap) {
  TestCore core = AtTheApproachPoint();
  See(core);
  Take(core, DockingLeg::Kind::kLine, LegFrame::kDock);
  core.OnPose(Map(-0.5, 0.0));
  core.OnDockingLegEnded(LegEnd::kGps, Reason("POSITION_ACCURACY", "the pose is not good"));
  core.OnTick(110.0);
  for (int i = 1; i < 50; i++) {
    const double t = 110.0 + 0.5 * i;
    core.OnDockMarker(Message(t, kMeasuring, false, false, "NO_MARKER"));
    core.OnTick(t);
  }
  core.OnTick(135.1);
  const DockingLeg out = Take(core, DockingLeg::Kind::kOut, LegFrame::kMap);
  EXPECT_FALSE(out.reverse);
  EXPECT_FALSE(core.DockFollowerActive());
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING_RETRY"));
  core.OnPose(Map(-0.2, 0.0));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  const auto events = core.TakeEvents();
  const Event* retry = Find(events, "DOCKING_RETRY");
  ASSERT_TRUE(retry);
  EXPECT_EQ(retry->details["cause"], "MARKER_LOST");
  EXPECT_EQ(retry->details["guide"], "marker");
  EXPECT_FALSE(core.TakeDockingLeg());
  EXPECT_TRUE(core.MarkerWanted());
}

// A try that fails on the line drives out by the marker where its pose is good; without that and without a good
// position of the map the mower stays and docking ends.
TEST(LogicMarkerGuide, OutByTheMarkerElseDockingEnds) {
  TestCore core = AtTheApproachPoint();
  See(core);
  Take(core, DockingLeg::Kind::kLine, LegFrame::kDock);
  core.OnPose(Map(-0.5, 0.0, 0.0, 0.5f));
  core.OnDockPose(ByMarker(103.0, -0.5, 0.0));
  core.OnDockingLegEnded(LegEnd::kFailed, Reason("TIMEOUT", "stuck"));
  Take(core, DockingLeg::Kind::kOut, LegFrame::kDock);
  EXPECT_TRUE(core.DockFollowerActive());
  TestCore lost = AtTheApproachPoint();
  See(lost);
  Take(lost, DockingLeg::Kind::kLine, LegFrame::kDock);
  lost.OnPose(Map(-0.5, 0.0, 0.0, 0.5f));
  DockPoseSample bad = ByMarker(103.0, -0.5, 0.0);
  bad.position_accuracy = 0.5f;
  lost.OnDockPose(bad);
  lost.OnDockingLegEnded(LegEnd::kFailed, Reason("TIMEOUT", "stuck"));
  EXPECT_FALSE(lost.TakeDockingLeg());
  ASSERT_TRUE(Find(lost.TakeEvents(), "DOCKING_FAILED"));
  EXPECT_EQ(lost.RobotState().state, "IDLE");
}

// The map leg of a docking that asks for the marker stays on follower/cmd_vel, and a pose of the marker meanwhile
// does not end the wait for a good position of the map.
TEST(LogicMarkerGuide, MapLegsStayMapLegs) {
  TestCore core = Approaching();
  core.OnPose(Map(1.0, 0.0, M_PI));
  ASSERT_TRUE(core.MarkerWanted());
  core.OnDockPose(ByMarker(50.0, 1.0, 0.0, M_PI));
  EXPECT_FALSE(core.DockFollowerActive());
  EXPECT_TRUE(core.FollowerActive());
  core.OnPose(Map(1.0, 0.0, M_PI, 0.5f));
  core.OnDockingLegEnded(LegEnd::kGps, Reason("POSITION_ACCURACY", "bad"));
  core.OnTick(50.0);
  core.OnDockPose(ByMarker(50.5, 1.0, 0.0, M_PI));
  core.OnTick(51.0);
  EXPECT_FALSE(core.TakeDockingLeg());
  // The map good again: poor poses of the marker between the ticks do not restart its wait.
  core.OnPose(Map(1.0, 0.0, M_PI));
  for (int i = 0; i <= 10; i++) {
    DockPoseSample poor = ByMarker(52.0 + i - 0.5, 1.0, 0.0, M_PI);
    poor.position_accuracy = 0.5f;
    core.OnDockPose(poor);
    core.OnTick(52.0 + i);
  }
  Take(core, DockingLeg::Kind::kApproach, LegFrame::kMap);
}
