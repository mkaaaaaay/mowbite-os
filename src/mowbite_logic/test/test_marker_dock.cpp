#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "mowbite_logic/marker_dock.h"

using namespace mowbite_logic;

namespace {

constexpr uint8_t kOff = 0;
constexpr uint8_t kStarting = 1;
constexpr uint8_t kMeasuring = 2;
constexpr uint8_t kFault = 3;

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
                    float accuracy = 0.01f, bool valid = true) {
  DockPoseSample pose;
  pose.time = t;
  pose.x = x;
  pose.y = y;
  pose.heading = heading;
  pose.heading_valid = valid;
  pose.position_accuracy = accuracy;
  pose.speed = speed;
  return pose;
}

MarkerConfig Tron() {
  MarkerConfig config;
  config.range = 1.2;
  config.marker_x = -0.16;
  return config;
}

double Rad(double degrees) {
  return degrees * M_PI / 180.0;
}

// A pose from the marker in the station frame with the spread of its heading (rad).
DockPoseSample At(double x, double y, double heading, double spread = 0.0) {
  DockPoseSample pose;
  pose.x = x;
  pose.y = y;
  pose.heading = heading;
  pose.heading_valid = true;
  pose.position_accuracy = 0.01f;
  pose.heading_accuracy = static_cast<float>(spread);
  return pose;
}

// The same place seen in a station frame turned by angle about the marker, as a marker yaw that errs by angle does.
DockPoseSample Turned(const DockPoseSample& pose, double angle, double spread = 0.0) {
  const double mx = 0.16;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return At(mx + c * (pose.x - mx) - s * pose.y, s * (pose.x - mx) + c * pose.y, pose.heading + angle, spread);
}


// A sight that began at 100 s, with a message every 0.1 s of this kind up to t.
MarkerWatch Sight(uint8_t state, bool valid, bool yaw, const std::string& reason, double until, bool sight = true) {
  MarkerWatch watch(Tron());
  watch.BeginWait(100.0, sight);
  for (int i = 0; 100.0 + 0.1 * i <= until + 1e-9; i++) {
    watch.OnMarker(Message(100.0 + 0.1 * i, state, valid, yaw, reason));
  }
  return watch;
}

DockingSequence StationDocking() {
  return MarkerDocking(DockConfig(), UndockConfig(), true, Tron());
}

std::string Start(const DockPoseSample& pose) {
  return TestStartProblem(StationDocking(), Tron(), pose, 0.05);
}

}  // namespace

// The source cannot measure: at once, with its reason as the detail.
TEST(MarkerWatch, FaultOrOffWithReasonIsUnavailableAtOnce) {
  MarkerWatch fault = Sight(kFault, false, false, "NOT_SET_UP", 100.5);
  EXPECT_EQ(fault.Check(100.6), MarkerWatch::Verdict::kUnavailable);
  EXPECT_EQ(fault.Detail(), "NOT_SET_UP");
  MarkerWatch off = Sight(kOff, false, false, "TOO_HOT", 100.5);
  EXPECT_EQ(off.Check(100.6), MarkerWatch::Verdict::kUnavailable);
  EXPECT_EQ(off.Detail(), "TOO_HOT");
}

TEST(MarkerWatch, StartingAndOffWithoutReasonWait) {
  EXPECT_EQ(Sight(kStarting, false, false, "", 102.0).Check(102.1), MarkerWatch::Verdict::kWaiting);
  EXPECT_EQ(Sight(kOff, false, false, "", 102.0).Check(102.1), MarkerWatch::Verdict::kWaiting);
  // Asked all along without a single image: not measuring.
  EXPECT_EQ(Sight(kStarting, false, false, "", 125.0).Check(125.1), MarkerWatch::Verdict::kUnavailable);
}

TEST(MarkerWatch, SilentForThreeSecondsIsUnavailable) {
  MarkerWatch watch(Tron());
  watch.BeginWait(100.0, true);
  EXPECT_EQ(watch.Check(102.9), MarkerWatch::Verdict::kWaiting);
  EXPECT_EQ(watch.Check(103.0), MarkerWatch::Verdict::kUnavailable);
  watch.OnMarker(Message(103.5, kMeasuring, false, false, "NO_MARKER"));
  EXPECT_EQ(watch.Check(106.0), MarkerWatch::Verdict::kWaiting);
  EXPECT_EQ(watch.Check(106.5), MarkerWatch::Verdict::kUnavailable);
  EXPECT_EQ(watch.Detail(), "NO_MARKER");
}

// Images without a marker up to marker_wait: not seen, with the reason of the last image.
TEST(MarkerWatch, ImagesWithoutMarkerAreNotSeenAfterTheWait) {
  MarkerWatch dark = Sight(kMeasuring, false, false, "TOO_DARK", 125.0);
  EXPECT_EQ(dark.Check(124.9), MarkerWatch::Verdict::kWaiting);
  EXPECT_EQ(dark.Check(125.05), MarkerWatch::Verdict::kNotSeen);
  EXPECT_EQ(dark.Detail(), "TOO_DARK");
  // A measurement before the wait does not count for it.
  MarkerWatch before(Tron());
  before.OnMarker(Message(99.0, kMeasuring, true, true));
  before.BeginWait(100.0, true);
  for (int i = 0; i <= 250; i++) before.OnMarker(Message(100.0 + 0.1 * i, kMeasuring, false, false, "NO_MARKER"));
  EXPECT_EQ(before.Check(125.05), MarkerWatch::Verdict::kNotSeen);
}

TEST(MarkerWatch, MarkerWithoutDirectionAfterTheWait) {
  EXPECT_EQ(Sight(kMeasuring, true, false, "", 125.0).Check(125.05), MarkerWatch::Verdict::kNoDirection);
  EXPECT_EQ(Sight(kMeasuring, true, true, "", 125.0).Check(125.05), MarkerWatch::Verdict::kNotSeen);
}

// On the way in a wait that runs out is lost, whatever the images say; the source failing still says so at once.
TEST(MarkerWatch, LostOnTheLine) {
  MarkerWatch hidden = Sight(kMeasuring, false, false, "NO_MARKER", 125.0, false);
  EXPECT_EQ(hidden.Check(124.9), MarkerWatch::Verdict::kWaiting);
  EXPECT_EQ(hidden.Check(125.05), MarkerWatch::Verdict::kLost);
  EXPECT_EQ(hidden.Detail(), "NO_MARKER");
  EXPECT_EQ(Sight(kFault, false, false, "NO_SIGNAL", 101.0, false).Check(101.1), MarkerWatch::Verdict::kUnavailable);
}

TEST(MarkerWatch, PoseGoodIsRecentAccurateWithHeading) {
  MarkerWatch watch(Tron());
  EXPECT_FALSE(watch.PoseGood(10.0, 0.03));
  watch.OnPose(Pose(10.0, -0.5), false);
  EXPECT_TRUE(watch.PoseGood(10.5, 0.03));
  EXPECT_FALSE(watch.PoseGood(10.51, 0.03));
  watch.OnPose(Pose(11.0, -0.5, 0.0, M_PI, 0.0, 0.031f), false);
  EXPECT_FALSE(watch.PoseGood(11.0, 0.03));
  watch.OnPose(Pose(11.1, -0.5, 0.0, M_PI, 0.0, 0.01f, false), false);
  EXPECT_FALSE(watch.PoseGood(11.1, 0.03));
  watch.Expire(11.7);
  EXPECT_FALSE(watch.pose());
}

// A pose kept by the localization counts for nothing while the source says it cannot measure, until it measures again.
TEST(MarkerWatch, NoGoodPoseWhileTheSourceCannotMeasure) {
  MarkerWatch watch(Tron());
  watch.OnPose(Pose(10.0, -0.5), false);
  watch.OnMarker(Message(10.1, kFault, false, false, "NOT_SET_UP"));
  EXPECT_TRUE(watch.SourceFault());
  EXPECT_FALSE(watch.PoseGood(10.2, 0.03));
  watch.OnMarker(Message(10.2, kOff, false, false, "TOO_HOT"));
  EXPECT_FALSE(watch.PoseGood(10.3, 0.03));
  watch.OnMarker(Message(10.3, kMeasuring, true, true));
  EXPECT_FALSE(watch.SourceFault());
  EXPECT_TRUE(watch.PoseGood(10.4, 0.03));
}

// The way by the marker against the wheels from the start of the line: within 15 %, else a marker of the wrong size.
TEST(MarkerWatch, ScaleOfTheMarkerAtTheGate) {
  const auto line = [](double scale) {
    MarkerWatch watch(Tron());
    watch.OnPose(Pose(0.0, -0.8), false);
    // 0.55 m at 0.1 m/s by the wheels, scale times that by the marker.
    for (int i = 0; i <= 55; i++) watch.OnPose(Pose(0.1 * i, -0.8 + scale * 0.01 * i, 0.0, M_PI, -0.1), true);
    watch.OnPose(Pose(5.6, -0.8 + scale * 0.55), false);
    return watch.ScaleProblem();
  };
  EXPECT_EQ(line(1.0), "");
  EXPECT_EQ(line(1.14), "");
  EXPECT_EQ(line(0.86), "");
  EXPECT_NE(line(1.25), "");
  EXPECT_NE(line(0.8), "");
  // Too short a way to compare.
  MarkerWatch short_way(Tron());
  for (int i = 0; i <= 5; i++) short_way.OnPose(Pose(0.1 * i, -0.8 + 0.02 * i, 0.0, M_PI, -0.1), true);
  EXPECT_EQ(short_way.ScaleProblem(), "");
}

// The legs by the marker: in the station frame, the line from range at most, and the contact split at the blind point.
TEST(MarkerDocking, LegsInTheStationFrame) {
  DockConfig far;
  far.approach_distance = 2.0;
  EXPECT_NEAR(MarkerDocking(far, UndockConfig(), true, Tron()).ApproachPoint().x, -1.2, 1e-9);
  const DockingSequence docking = StationDocking();
  EXPECT_NEAR(docking.ApproachPoint().x, -0.8, 1e-9);
  const DockingLeg line = docking.Line();
  EXPECT_EQ(line.frame, LegFrame::kDock);
  EXPECT_NEAR(line.max_cross_track, 0.10, 1e-9);
  EXPECT_TRUE(line.reverse);
  EXPECT_NEAR(line.poses.back().x, -0.25, 1e-9);
  const DockingLeg close = docking.AfterGate();
  ASSERT_EQ(close.kind, DockingLeg::Kind::kClose);
  EXPECT_EQ(close.frame, LegFrame::kDock);
  EXPECT_NEAR(close.poses.front().x, -0.25, 1e-9);
  EXPECT_NEAR(close.poses.back().x, -0.07, 1e-9);
  EXPECT_NEAR(close.stall_time, UndockConfig().stall_time, 1e-9);
  const DockingLeg contact = docking.Contact();
  EXPECT_EQ(contact.kind, DockingLeg::Kind::kContact);
  EXPECT_NEAR(contact.poses.front().x, -0.07, 1e-9);
  EXPECT_NEAR(contact.poses.back().x, 0.05, 1e-9);
  EXPECT_NEAR(contact.stall_time, DockConfig().contact_timeout, 1e-9);
  EXPECT_NEAR(contact.speed, DockConfig().contact_speed, 1e-9);
  const DockingLeg out = docking.Out(-0.5, 0.0, M_PI);
  EXPECT_EQ(out.frame, LegFrame::kDock);
  EXPECT_FALSE(out.reverse);
  for (const auto& pose : line.poses) EXPECT_NEAR(pose.heading, M_PI, 1e-9);
}

TEST(MarkerDocking, MapLegsStayAsTheyWere) {
  const DockingSequence docking(DockConfig(), UndockConfig(), DockPose{-1.0, 0.0, M_PI}, true);
  EXPECT_EQ(docking.Line().frame, LegFrame::kMap);
  EXPECT_EQ(docking.Line().max_cross_track, 0.0);
  EXPECT_EQ(docking.AfterGate().kind, DockingLeg::Kind::kContact);
  EXPECT_NEAR(docking.Contact().poses.front().x, -0.75, 1e-9);
}

// No guided part before the blind one where the gate lies within it.
TEST(MarkerDocking, NoCloseLegWhereTheGateIsBlind) {
  MarkerConfig blind = Tron();
  blind.blind_distance = 0.2;
  const DockingSequence docking = MarkerDocking(DockConfig(), UndockConfig(), true, blind);
  EXPECT_EQ(docking.AfterGate().kind, DockingLeg::Kind::kContact);
  EXPECT_NEAR(docking.Contact().poses.front().x, -0.25, 1e-9);
}

// The marker beside the axis of the mower: on the line facing along it none, 0.03 m beside the line 0.03 m (left
// positive), turned by 10 deg at 0.66 m from the marker 0.66 sin 10 deg; a station frame turned about the marker, as a
// marker yaw that errs does, leaves it as it is.
TEST(SideOffset, BesideTheAxisAndNotTurnedWithTheFrame) {
  const DockingSequence docking = StationDocking();
  EXPECT_NEAR(SideOffset(docking, Tron(), -0.5, 0.0, M_PI), 0.0, 1e-9);
  EXPECT_NEAR(SideOffset(docking, Tron(), -0.5, 0.03, M_PI), 0.03, 1e-9);
  EXPECT_NEAR(std::abs(SideOffset(docking, Tron(), -0.5, 0.0, M_PI + Rad(10))), 0.66 * std::sin(Rad(10)), 1e-9);
  for (const double angle : {Rad(-5), Rad(4)}) {
    const DockPoseSample turned = Turned(At(-0.5, 0.02, M_PI + Rad(3)), angle);
    EXPECT_NEAR(SideOffset(docking, Tron(), turned.x, turned.y, turned.heading),
                SideOffset(docking, Tron(), -0.5, 0.02, M_PI + Rad(3)), 1e-9);
  }
  // The docked mower has the marker where dock_marker puts it.
  MarkerConfig aside = Tron();
  aside.marker_y = 0.02;
  EXPECT_NEAR(SideOffset(docking, aside, 0.0, 0.0, M_PI), 0.0, 1e-9);
}

// The hand start of the test: on the line with the marker within 0.05 m of its axis, or at its start, facing along it,
// the line beginning on it.
TEST(TestStart, OnTheLineOrAtItsStart) {
  EXPECT_EQ(Start(At(-0.5, 0.0, M_PI)), "");
  EXPECT_EQ(Start(At(-0.5, 0.04, M_PI)), "");
  EXPECT_EQ(Start(At(-0.8, -0.03, M_PI - 0.2)), "");
  EXPECT_EQ(Start(At(-0.85, 0.0, M_PI)), "");
}

TEST(TestStart, RefusedBesideTurnedOrAway) {
  EXPECT_NE(Start(At(-0.5, 0.06, M_PI)).find("0.06 m beside the axis"), std::string::npos);
  EXPECT_NE(Start(At(-0.5, 0.3, M_PI)).find("beside the axis"), std::string::npos);
  EXPECT_NE(Start(At(-0.5, 0.0, M_PI - 0.4)).find("deg off"), std::string::npos);
  // Contacts away from the station.
  EXPECT_NE(Start(At(-0.5, 0.0, 0.0)).find("deg off"), std::string::npos);
  EXPECT_NE(Start(At(-1.1, 0.0, M_PI)).find("neither on the line"), std::string::npos);
  // At the approach point beside the line.
  EXPECT_NE(Start(At(-0.8, 0.06, M_PI)).find("begin 0.06 m beside"), std::string::npos);
  // 0.6 m out at 6 deg ends 0.06 m beside; 0.012 m on the other side of the line makes up for it.
  EXPECT_NE(Start(At(-0.2, 0.0, M_PI + 0.1)).find("would begin 0.06 m"), std::string::npos);
  EXPECT_EQ(Start(At(-0.2, 0.012, M_PI + 0.1)), "");
  EXPECT_NE(Start(At(-0.2, -0.02, M_PI + 0.1)).find("would begin"), std::string::npos);
}

// Measured on the Tron (15.11.): the marker yaw errs by 3 to 4 deg at one place, which turns the station frame about
// the marker. A mower on the line facing along it stands 0.66 sin 5 deg beside the line of such a frame, and the
// straight way out would end 0.96 sin 5 deg beside it: refused without the spread of the heading, started with it.
TEST(TestStart, AMarkerYawThatErrsCountsWithItsSpread) {
  const DockPoseSample turned = Turned(At(-0.5, 0.0, M_PI), Rad(5));
  EXPECT_GT(std::abs(StationDocking().Across(turned.x, turned.y)), 0.05);
  EXPECT_NE(Start(turned).find("would begin 0.08 m"), std::string::npos);
  EXPECT_EQ(Start(Turned(At(-0.5, 0.0, M_PI), Rad(5), Rad(4))), "");
  EXPECT_EQ(Start(Turned(At(-0.8, 0.0, M_PI), Rad(-5), Rad(4))), "");
  // The spread widens up to the limit of the follower on the line, not beyond.
  EXPECT_NE(Start(At(-0.8, 0.12, M_PI, Rad(20))).find("at most 0.10 m"), std::string::npos);
  // The marker beside the axis is measured, its spread widens nothing; nor does an unknown spread.
  EXPECT_NE(Start(At(-0.5, 0.06, M_PI, Rad(4))).find("beside the axis"), std::string::npos);
  EXPECT_NE(Start(Turned(At(-0.5, 0.0, M_PI), Rad(5), 999.0)).find("would begin"), std::string::npos);
}

// The station by the marker in the map: base_link 0.8 m east of the station of the map facing east, its pose in the
// station frame 0.8 m in front facing away; 0.1 m left of the line there (station frame, the way in going west) puts
// the station 0.1 m south of base_link; turned, the station turns with it.
TEST(StationByMarker, FromMapAndStationFrame) {
  DockPoseSample pose;
  pose.x = -0.8;
  pose.heading = M_PI;
  DockPose station = StationByMarker(-0.2, 0.0, 0.0, pose);
  EXPECT_NEAR(station.x, -1.0, 1e-9);
  EXPECT_NEAR(station.y, 0.0, 1e-9);
  EXPECT_NEAR(std::cos(station.heading), -1.0, 1e-9);
  pose.y = -0.1;
  station = StationByMarker(-0.2, 0.0, 0.0, pose);
  EXPECT_NEAR(station.y, -0.1, 1e-9);
  // A map heading 0.1 rad off turns the station by it, and moves it by 0.1 rad times the 0.8 m.
  pose.y = 0.0;
  station = StationByMarker(-0.2, 0.0, 0.1, pose);
  EXPECT_NEAR(station.x, -0.2 - 0.8 * std::cos(0.1), 1e-9);
  EXPECT_NEAR(station.y, -0.8 * std::sin(0.1), 1e-9);
  EXPECT_NEAR(std::remainder(station.heading - M_PI, 2 * M_PI), 0.1, 1e-9);
}

TEST(StationByMarker, ProblemByDistanceAndHeading) {
  const DockPose map{-1.0, 0.0, M_PI};
  EXPECT_EQ(MarkerMapProblem(DockPose{-1.0, 0.09, M_PI}, map, 0.1, 0.0), "");
  EXPECT_NE(MarkerMapProblem(DockPose{-1.0, 0.11, M_PI}, map, 0.1, 0.0).find("0.11 m"), std::string::npos);
  // The heading only where a limit is given, across the wrap of the angle.
  const DockPose turned{-1.0, 0.0, -M_PI + 0.2};
  EXPECT_EQ(MarkerMapProblem(turned, map, 0.1, 0.0), "");
  EXPECT_EQ(MarkerMapProblem(DockPose{-1.0, 0.0, -M_PI + 0.15}, map, 0.1, kMarkerMapHeading), "");
  EXPECT_NE(MarkerMapProblem(turned, map, 0.1, kMarkerMapHeading).find("11 deg"), std::string::npos);
}
