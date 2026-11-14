#include <gtest/gtest.h>

#include <cmath>

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
  return config;
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

// The hand start of the test: on the line within 0.05 m or at its start, facing along it, the way out ending on it.
TEST(TestStart, OnTheLineOrAtItsStart) {
  const DockingSequence docking = StationDocking();
  EXPECT_EQ(TestStartProblem(docking, -0.5, 0.0, M_PI, 0.05), "");
  EXPECT_EQ(TestStartProblem(docking, -0.5, 0.04, M_PI, 0.05), "");
  EXPECT_EQ(TestStartProblem(docking, -0.8, -0.03, M_PI - 0.2, 0.05), "");
  EXPECT_EQ(TestStartProblem(docking, -0.85, 0.0, M_PI, 0.05), "");
}

TEST(TestStart, RefusedBesideTurnedOrAway) {
  const DockingSequence docking = StationDocking();
  EXPECT_NE(TestStartProblem(docking, -0.5, 0.06, M_PI, 0.05).find("beside the line"), std::string::npos);
  EXPECT_NE(TestStartProblem(docking, -0.5, 0.3, M_PI, 0.05).find("beside the line"), std::string::npos);
  EXPECT_NE(TestStartProblem(docking, -0.5, 0.0, M_PI - 0.4, 0.05).find("deg off"), std::string::npos);
  // Contacts away from the station.
  EXPECT_NE(TestStartProblem(docking, -0.5, 0.0, 0.0, 0.05).find("deg off"), std::string::npos);
  EXPECT_NE(TestStartProblem(docking, -1.1, 0.0, M_PI, 0.05).find("neither on the line"), std::string::npos);
  // 0.6 m out at 6 deg ends 0.06 m beside; 0.02 m on the other side of the line makes up for it.
  EXPECT_NE(TestStartProblem(docking, -0.2, 0.0, M_PI + 0.1, 0.05).find("straight out"), std::string::npos);
  EXPECT_EQ(TestStartProblem(docking, -0.2, 0.02, M_PI + 0.1, 0.05), "");
  EXPECT_NE(TestStartProblem(docking, -0.2, -0.02, M_PI + 0.1, 0.05).find("straight out"), std::string::npos);
}
