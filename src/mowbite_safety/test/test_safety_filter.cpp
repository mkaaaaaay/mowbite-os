#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "mowbite_safety/safety_filter.h"

using namespace mowbite_safety;

namespace {

// A lawn of 10 x 6 m with a tree, a strip of navigation area east of it, and the station 1 m west of the lawn.
const char* kMap = R"({
  "areas": [
    {"id": "a", "outline": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 6}, {"x": 0, "y": 6}],
     "properties": {"type": "mow"}},
    {"id": "b", "outline": [{"x": 10, "y": 2}, {"x": 14, "y": 2}, {"x": 14, "y": 3}, {"x": 10, "y": 3}],
     "properties": {"type": "nav"}},
    {"id": "c", "outline": [{"x": 4, "y": 2}, {"x": 5, "y": 2}, {"x": 5, "y": 3}, {"x": 4, "y": 3}],
     "properties": {"type": "obstacle"}}
  ],
  "docking_stations": [{"position": {"x": -1, "y": 3}, "heading": 3.141592653589793}]
})";

// The lawn alone with the station 5 m west of it, so that the zone of the station stands apart.
const char* kFarStation = R"({
  "areas": [
    {"id": "a", "outline": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 6}, {"x": 0, "y": 6}],
     "properties": {"type": "mow"}}
  ],
  "docking_stations": [{"position": {"x": -5, "y": 3}, "heading": 3.141592653589793}]
})";

// Runs the filter in steps of 20 ms with the inputs below, like the IMU and the pose arrive.
class FilterTest : public ::testing::Test {
 protected:
  FilterTest() : filter_(SafetyConfig{}) {
    filter_.OnCapabilities(true, 0.3, 0.3);
    filter_.SetMap(kMap);
    filter_.SetMapFits(true);
    Run(0.1);
  }

  void Run(double until) {
    while (now_ < until - 1e-9) {
      now_ += 0.02;
      if (hw_) filter_.OnHardware(hw_emergency_, hw_reason_, now_);
      if (imu_) filter_.OnTilt(tilt_, now_);
      if (pose_) filter_.OnPose(x_, y_, accuracy_, heading_valid_, now_);
      if (dock_) filter_.OnDockPose(dock_x_, dock_y_, dock_accuracy_, dock_heading_valid_, now_);
      if (filter_.Tick(now_)) stops_++;
    }
  }

  Decision Cmd(Source source, double linear, double angular = 0) {
    return filter_.OnCommand(source, linear, angular, now_);
  }

  // A command every 0.1 s until `until`; true if every one passed.
  bool Hold(Source source, double linear, double until) {
    bool all = true;
    while (now_ < until - 1e-9) {
      const Decision d = Cmd(source, linear);
      all = all && d.refusal.empty() && d.out && d.out->linear != 0;
      Run(now_ + 0.1);
    }
    return all;
  }

  static bool Moves(const Decision& d) {
    return d.refusal.empty() && d.out && (d.out->linear != 0 || d.out->angular != 0);
  }

  static bool Stops(const Decision& d) {
    return d.out && d.out->linear == 0 && d.out->angular == 0;
  }

  // A filter that may drive at t: capabilities, the map, and hw/status, a level IMU sample and an accurate pose at t.
  static SafetyFilter Ready(double t, const SafetyConfig& config = SafetyConfig{}, const char* map = kMap) {
    SafetyFilter filter{config};
    filter.OnCapabilities(true, 0.3, 0.3);
    filter.SetMap(map);
    filter.SetMapFits(true);
    Feed(filter, t);
    return filter;
  }

  static void Feed(SafetyFilter& filter, double t) {
    filter.OnHardware(false, 0, t);
    filter.OnTilt(2.0, t);
    filter.OnPose(2, 2, 0.02, true, t);
  }

  SafetyFilter filter_;
  double now_ = 0;
  bool hw_ = true;
  bool hw_emergency_ = false;
  uint32_t hw_reason_ = 0;
  bool imu_ = true;
  double tilt_ = 2.0;
  bool pose_ = true;
  double x_ = 2;
  double y_ = 2;
  double accuracy_ = 0.02;
  bool heading_valid_ = true;
  // The pose from the dock marker, in the station frame: 0.8 m in front of the docked pose on its line.
  bool dock_ = false;
  double dock_x_ = -0.8;
  double dock_y_ = 0;
  double dock_accuracy_ = 0.02;
  bool dock_heading_valid_ = true;
  int stops_ = 0;
};

}  // namespace

TEST(Tilt, FromOrientationAndGravity) {
  const double half = 15.0 * M_PI / 180.0;
  EXPECT_NEAR(TiltFromOrientation(0, 0, 0, 1), 0, 1e-9);
  EXPECT_NEAR(TiltFromOrientation(std::sin(half), 0, 0, std::cos(half)), 30, 1e-9);
  EXPECT_NEAR(TiltFromOrientation(0, std::sin(half), 0, std::cos(half)), 30, 1e-9);
  // Yaw alone tilts nothing, and the length of the quaternion does not matter.
  EXPECT_NEAR(TiltFromOrientation(0, 0, std::sin(1.0), std::cos(1.0)), 0, 1e-9);
  EXPECT_NEAR(TiltFromOrientation(2 * std::sin(half), 0, 0, 2 * std::cos(half)), 30, 1e-9);
  EXPECT_TRUE(std::isnan(TiltFromOrientation(0, 0, 0, 0)));
  EXPECT_NEAR(TiltFromGravity(0, 0, 9.81), 0, 1e-9);
  EXPECT_NEAR(TiltFromGravity(0, 9.81 * std::sin(0.5), 9.81 * std::cos(0.5)), 0.5 * 180.0 / M_PI, 1e-9);
  EXPECT_NEAR(TiltFromGravity(0, 0, -9.81), 180, 1e-9);
  EXPECT_TRUE(std::isnan(TiltFromGravity(0, 0, 0)));
}

TEST(Tilt, CountsFromTheMountingOfTheImu) {
  const double deg = M_PI / 180.0;
  // An IMU 4.5 degrees nose up on level ground reads gravity tilted back: that is level for the mower.
  const double pitch = 4.5 * deg;
  const double ax = -9.81 * std::sin(pitch), az = 9.81 * std::cos(pitch);
  EXPECT_NEAR(TiltFromGravity(ax, 0, az), 4.5, 1e-9);
  EXPECT_NEAR(TiltFromGravity(ax, 0, az, 0.0, pitch), 0, 1e-9);
  // The mower 30 degrees nose down on a slope: 30 degrees off the mounting, whichever way.
  const double down = pitch - 30 * deg;
  EXPECT_NEAR(TiltFromGravity(-9.81 * std::sin(down), 0, 9.81 * std::cos(down), 0.0, pitch), 30, 1e-9);
  const double up = pitch + 30 * deg;
  EXPECT_NEAR(TiltFromGravity(-9.81 * std::sin(up), 0, 9.81 * std::cos(up), 0.0, pitch), 30, 1e-9);
  // Roll in the convention of the planner: atan2(ay, az).
  const double roll = -2.0 * deg;
  EXPECT_NEAR(TiltFromGravity(0, 9.81 * std::sin(roll), 9.81 * std::cos(roll), roll, 0.0), 0, 1e-9);
  // The orientation of an IMU that reads its mounting on level ground: turned by pitch about y.
  EXPECT_NEAR(TiltFromOrientation(0, std::sin(pitch / 2), 0, std::cos(pitch / 2), 0.0, pitch), 0, 1e-9);
  EXPECT_NEAR(TiltFromOrientation(0, std::sin(pitch / 2), 0, std::cos(pitch / 2)), 4.5, 1e-9);
}

TEST(Tilt, AnInfiniteLengthIsNoNumber) {
  // Every component finite, the length not.
  EXPECT_TRUE(std::isnan(TiltFromGravity(1e200, 0, 0)));
  EXPECT_TRUE(std::isnan(TiltFromOrientation(0, 0, 1e200, 0)));
}

TEST(Tilt, LevelAndUpsideDownForAnyMountingAlsoWhenRoundingPassesOne) {
  for (int i = 0; i < 400; i++) {
    const double roll = (i % 41 - 20) * 0.003;
    const double pitch = (i % 37 - 18) * 0.005;
    const double x = -std::sin(pitch), y = std::cos(pitch) * std::sin(roll), z = std::cos(pitch) * std::cos(roll);
    EXPECT_NEAR(TiltFromGravity(9.81 * x, 9.81 * y, 9.81 * z, roll, pitch), 0, 1e-5) << roll << " " << pitch;
    EXPECT_NEAR(TiltFromGravity(-9.81 * x, -9.81 * y, -9.81 * z, roll, pitch), 180, 1e-5) << roll << " " << pitch;
  }
}

TEST_F(FilterTest, PassesBothSourcesWithinTheLimitsOfTheDriver) {
  Decision d = Cmd(Source::kTeleop, 1.0, -2.0);
  ASSERT_TRUE(Moves(d)) << d.refusal;
  EXPECT_DOUBLE_EQ(d.out->linear, 0.3);
  EXPECT_DOUBLE_EQ(d.out->angular, -0.3);
  EXPECT_EQ(filter_.Owner(now_), Source::kTeleop);
  EXPECT_TRUE(Stops(Cmd(Source::kTeleop, 0)));
  EXPECT_EQ(filter_.Owner(now_), Source::kStop);
  Run(now_ + 1.1);
  d = Cmd(Source::kFollower, 0.2, 0.1);
  ASSERT_TRUE(Moves(d)) << d.refusal;
  EXPECT_DOUBLE_EQ(d.out->linear, 0.2);
  EXPECT_EQ(filter_.Refused(), 0u);
}

TEST_F(FilterTest, TheFollowerReversesAtMostTwelveCentimetresASecond) {
  Decision d = Cmd(Source::kFollower, -0.25, 0.2);
  ASSERT_TRUE(Moves(d)) << d.refusal;
  EXPECT_DOUBLE_EQ(d.out->linear, -0.12);
  EXPECT_DOUBLE_EQ(d.out->angular, 0.2);
  EXPECT_DOUBLE_EQ(Cmd(Source::kFollower, -0.1).out->linear, -0.1);
  EXPECT_DOUBLE_EQ(Cmd(Source::kFollower, 0.25).out->linear, 0.25);
  EXPECT_EQ(filter_.Refused(), 0u);
  // A driver limit below it holds.
  filter_.OnCapabilities(true, 0.05, 0.3);
  EXPECT_DOUBLE_EQ(Cmd(Source::kFollower, -0.25).out->linear, -0.05);
  // The remote control reverses up to the limit of the driver.
  filter_.OnCapabilities(true, 0.3, 0.3);
  EXPECT_TRUE(Stops(Cmd(Source::kFollower, 0)));
  Run(now_ + 1.1);
  d = Cmd(Source::kTeleop, -0.25);
  ASSERT_TRUE(Moves(d)) << d.refusal;
  EXPECT_DOUBLE_EQ(d.out->linear, -0.25);
}

TEST_F(FilterTest, NothingMovesWithoutHardwareStatusCapabilitiesOrImu) {
  SafetyFilter fresh{SafetyConfig{}};
  EXPECT_EQ(fresh.OnCommand(Source::kTeleop, 0.1, 0, 1.0).refusal, "emergency");
  fresh.OnHardware(false, 0, 1.0);
  // It sent during the emergency a moment ago, so it has to show a zero first.
  EXPECT_EQ(fresh.OnCommand(Source::kTeleop, 0.1, 0, 1.0).refusal, "waits for a zero or 1.0 s without commands");
  EXPECT_TRUE(Stops(fresh.OnCommand(Source::kTeleop, 0, 0, 1.0)));
  EXPECT_EQ(fresh.OnCommand(Source::kTeleop, 0.1, 0, 1.0).refusal, "no hw/capabilities from the driver");
  fresh.OnCapabilities(false, 0.3, 0.3);
  EXPECT_EQ(fresh.OnCommand(Source::kTeleop, 0.1, 0, 1.0).refusal, "no IMU sample yet");
  fresh.OnTilt(1.0, 1.0);
  EXPECT_TRUE(Moves(fresh.OnCommand(Source::kTeleop, 0.1, 0, 1.0)));
  // hw/status gone for more than 1 s counts as an emergency.
  fresh.OnTilt(1.0, 2.05);
  EXPECT_EQ(fresh.OnCommand(Source::kTeleop, 0.1, 0, 2.05).refusal, "emergency");
}

TEST_F(FilterTest, AStaleImuStopsTheMowerAndBlocksEverySource) {
  ASSERT_TRUE(Moves(Cmd(Source::kTeleop, 0.2)));
  imu_ = false;
  Run(now_ + 0.4);
  EXPECT_EQ(stops_, 0);
  Run(now_ + 0.2);
  EXPECT_EQ(stops_, 1);
  const Decision d = Cmd(Source::kTeleop, 0.2);
  EXPECT_NE(d.refusal.find("no IMU sample for"), std::string::npos) << d.refusal;
  EXPECT_EQ(filter_.Latched(), 0u);
}

TEST_F(FilterTest, TiltLatchesAfterTiltTimeAndAlsoStanding) {
  tilt_ = 35;
  Run(now_ + 0.9);
  EXPECT_EQ(filter_.Latched(), 0u);
  Run(now_ + 0.15);
  EXPECT_EQ(filter_.Latched(), kReasonTilt);
  const auto latch = filter_.TakeLatchRequest(now_);
  ASSERT_TRUE(latch.has_value());
  EXPECT_EQ(latch->bits, kReasonTilt);
  EXPECT_FALSE(latch->again);
  EXPECT_NE(latch->reason.find("more than 31 deg"), std::string::npos) << latch->reason;
  EXPECT_EQ(Cmd(Source::kTeleop, 0.1).refusal, "emergency");
}

TEST_F(FilterTest, ShortTiltSpikesDoNotLatch) {
  for (int i = 0; i < 5; i++) {
    tilt_ = 40;
    Run(now_ + 0.5);
    tilt_ = 5;
    Run(now_ + 0.1);
  }
  EXPECT_EQ(filter_.Latched(), 0u);
}

TEST_F(FilterTest, TiltStopsTheMovingSourceOnce) {
  ASSERT_TRUE(Moves(Cmd(Source::kFollower, 0.2)));
  tilt_ = 35;
  // The follower goes on sending; after the latch its commands are refused without another zero.
  EXPECT_FALSE(Hold(Source::kFollower, 0.2, now_ + 1.5));
  EXPECT_EQ(stops_, 1);
  EXPECT_EQ(filter_.Latched(), kReasonTilt);
}

TEST_F(FilterTest, ReleaseWaitsUntilTheMowerIsLevelAndTheImuSpeaks) {
  tilt_ = 35;
  Run(now_ + 1.1);
  ASSERT_EQ(filter_.Latched(), kReasonTilt);
  tilt_ = 30;
  Run(now_ + 0.1);
  Release release = filter_.OnRelease(now_);
  EXPECT_FALSE(release.allowed);
  EXPECT_EQ(release.message, "still tilted by 30 deg, below 26 deg needed");
  EXPECT_EQ(release.code, "STILL_TILTED");
  EXPECT_NEAR(release.value, 30, 1e-6);
  EXPECT_EQ(release.limit, 26);
  imu_ = false;
  tilt_ = 10;
  Run(now_ + 0.6);
  release = filter_.OnRelease(now_);
  EXPECT_FALSE(release.allowed);
  EXPECT_NE(release.message.find("the tilt is unknown"), std::string::npos) << release.message;
  EXPECT_EQ(release.code, "TILT_UNKNOWN");
  EXPECT_GT(release.value, 0.5);
  imu_ = true;
  Run(now_ + 0.1);
  release = filter_.OnRelease(now_);
  EXPECT_TRUE(release.allowed) << release.message;
  EXPECT_EQ(filter_.Latched(), 0u);
  EXPECT_FALSE(filter_.TakeLatchRequest(now_).has_value());
}

TEST_F(FilterTest, LatchIsAskedForAgainUntilHwStatusShowsIt) {
  tilt_ = 35;
  Run(now_ + 1.1);
  ASSERT_TRUE(filter_.TakeLatchRequest(now_).has_value());
  Run(now_ + 0.5);
  EXPECT_FALSE(filter_.TakeLatchRequest(now_).has_value());
  Run(now_ + 0.5);
  const auto again = filter_.TakeLatchRequest(now_);
  ASSERT_TRUE(again.has_value());
  EXPECT_TRUE(again->again);
  hw_emergency_ = true;
  hw_reason_ = 1u | kReasonTilt;
  Run(now_ + 1.1);
  EXPECT_FALSE(filter_.TakeLatchRequest(now_).has_value());
}

TEST_F(FilterTest, RestartHoldLatchesOnceAndGoesOnlyWithARelease) {
  ASSERT_TRUE(Moves(Cmd(Source::kTeleop, 0.2))) << filter_.LastRefusal();
  filter_.OnRestartHold(true);
  EXPECT_EQ(filter_.Latched(), kReasonUnexpectedRestart);
  EXPECT_TRUE(Stops(Cmd(Source::kTeleop, 0.2)));
  const auto latch = filter_.TakeLatchRequest(now_);
  ASSERT_TRUE(latch.has_value());
  EXPECT_EQ(latch->bits, kReasonUnexpectedRestart);
  EXPECT_FALSE(latch->again);
  // The file stays while the driver shows the bit: no new request.
  hw_emergency_ = true;
  hw_reason_ = 1u | kReasonUnexpectedRestart;
  for (int i = 0; i < 20; ++i) {
    filter_.OnRestartHold(true);
    Run(now_ + 0.1);
    EXPECT_FALSE(filter_.TakeLatchRequest(now_).has_value());
  }
  // The logic removes the file, then asks for the release.
  filter_.OnRestartHold(false);
  EXPECT_EQ(filter_.Latched(), kReasonUnexpectedRestart);
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  EXPECT_EQ(filter_.Latched(), 0u);
  filter_.OnRestartHold(false);
  EXPECT_EQ(filter_.Latched(), 0u);
}

TEST_F(FilterTest, ARestartHoldFileLatchesAgainOnlyWhenItComesAnew) {
  filter_.OnRestartHold(true);
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  // The release came first: the file it saw before does not latch again.
  filter_.OnRestartHold(true);
  EXPECT_EQ(filter_.Latched(), 0u);
  filter_.OnRestartHold(false);
  filter_.OnRestartHold(true);
  EXPECT_EQ(filter_.Latched(), kReasonUnexpectedRestart);
}

TEST_F(FilterTest, FenceLatchesForTheFollowerOnly) {
  // 0.5 m west of the lawn and off the line of the station: within the margin.
  x_ = -0.5;
  y_ = 1;
  Run(now_ + 0.1);
  ASSERT_TRUE(Moves(Cmd(Source::kFollower, 0.2))) << filter_.LastRefusal();
  EXPECT_NEAR(filter_.FenceDistance(now_), 0.5, 1e-9);
  x_ = -0.7;
  Run(now_ + 0.1);
  const Decision d = Cmd(Source::kFollower, 0.2);
  EXPECT_EQ(d.refusal, "base_link is 0.70 m outside of the allowed areas at (-0.70, 1.00)");
  EXPECT_TRUE(Stops(d));
  EXPECT_EQ(filter_.Latched(), kReasonFence);
  ASSERT_TRUE(filter_.TakeLatchRequest(now_).has_value());
  // Standing outside is no reason to keep the emergency; remote control has no fence.
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  Run(now_ + 0.1);
  EXPECT_TRUE(Moves(Cmd(Source::kTeleop, 0.2))) << filter_.LastRefusal();
  EXPECT_EQ(filter_.Latched(), 0u);
}

TEST_F(FilterTest, ObstaclesHaveASmallMargin) {
  x_ = 4.05;
  y_ = 2.5;
  Run(now_ + 0.1);
  EXPECT_TRUE(Moves(Cmd(Source::kFollower, 0.2))) << filter_.LastRefusal();
  x_ = 4.5;
  Run(now_ + 0.1);
  EXPECT_EQ(Cmd(Source::kFollower, 0.2).refusal, "base_link is 0.50 m inside an obstacle at (4.50, 2.50)");
  EXPECT_EQ(filter_.Latched(), kReasonFence);
}

TEST_F(FilterTest, TheLineOfTheStationIsAllowed) {
  // 0.9 m west of the lawn on the line of the station.
  x_ = -0.9;
  y_ = 3.0;
  Run(now_ + 0.1);
  EXPECT_TRUE(Moves(Cmd(Source::kFollower, -0.1))) << filter_.LastRefusal();
  EXPECT_DOUBLE_EQ(filter_.FenceDistance(now_), 0);
  // Beside the line and far from the lawn.
  y_ = 3.8;
  Run(now_ + 0.1);
  EXPECT_NE(Cmd(Source::kFollower, -0.1).refusal, "");
  EXPECT_EQ(filter_.Latched(), kReasonFence);
}

TEST_F(FilterTest, AutonomousDrivesNeedPoseAccuracyHeadingMapAndMeasurement) {
  const auto refused = [this](const std::string& why) {
    Run(now_ + 0.1);
    const Decision d = Cmd(Source::kFollower, 0.2);
    EXPECT_EQ(d.refusal, why);
    // None of these latches, and remote control still drives.
    EXPECT_EQ(filter_.Latched(), 0u);
    EXPECT_TRUE(Stops(Cmd(Source::kFollower, 0)));
    Run(now_ + 1.1);
    EXPECT_TRUE(Moves(Cmd(Source::kTeleop, 0.1))) << filter_.LastRefusal();
    EXPECT_TRUE(Stops(Cmd(Source::kTeleop, 0)));
    Run(now_ + 1.1);
  };
  accuracy_ = 0.3;
  refused("position accuracy 0.300 m, 0.100 m needed");
  filter_.SetMaxPositionAccuracy(0.5);
  Run(now_ + 0.1);
  EXPECT_TRUE(Moves(Cmd(Source::kFollower, 0.2))) << filter_.LastRefusal();
  EXPECT_TRUE(Stops(Cmd(Source::kFollower, 0)));
  Run(now_ + 1.1);
  accuracy_ = 0.02;
  heading_valid_ = false;
  refused("the heading is not aligned");
  heading_valid_ = true;
  pose_ = false;
  Run(now_ + 0.6);
  refused("no recent pose from the localization");
  pose_ = true;
  filter_.SetMapFits(false);
  refused("the map does not fit the datum (map/status)");
  filter_.SetMapFits(true);
  filter_.SetMap("{}");
  refused("map/json is not a map");
  filter_.SetMap(R"({"areas": []})");
  refused("the map has no active mow or navigation area");
  filter_.SetMap(kMap);
  filter_.OnCapabilities(false, 0.3, 0.3);
  refused("no safety measurement of this platform (hw/capabilities)");
}

TEST_F(FilterTest, OneSourceDrivesAtATime) {
  ASSERT_TRUE(Moves(Cmd(Source::kFollower, 0.2)));
  Run(now_ + 0.05);
  Decision d = Cmd(Source::kTeleop, 0.2);
  EXPECT_TRUE(d.conflict);
  EXPECT_EQ(d.refusal, "the follower drives");
  // The follower keeps going.
  EXPECT_FALSE(d.out.has_value());
  EXPECT_TRUE(Moves(Cmd(Source::kFollower, 0.2)));
  // Its zero ends the drive; the remote control sent a moment ago, so it has to show a zero first.
  EXPECT_TRUE(Stops(Cmd(Source::kFollower, 0)));
  d = Cmd(Source::kTeleop, 0.2);
  EXPECT_FALSE(d.conflict);
  EXPECT_EQ(d.refusal, "waits for a zero or 1.0 s without commands");
  EXPECT_TRUE(Stops(Cmd(Source::kTeleop, 0)));
  EXPECT_TRUE(Moves(Cmd(Source::kTeleop, 0.2)));
  // A source that went silent loses the drive after 0.5 s; the follower was quiet for longer than 1 s.
  Run(now_ + 1.1);
  EXPECT_EQ(filter_.Owner(now_), Source::kStop);
  EXPECT_TRUE(Moves(Cmd(Source::kFollower, 0.2))) << filter_.LastRefusal();
}

TEST_F(FilterTest, HeldJoystickWaitsAfterAnEmergency) {
  ASSERT_TRUE(Hold(Source::kTeleop, 0.2, now_ + 0.5));
  hw_emergency_ = true;
  Run(now_ + 0.2);
  EXPECT_FALSE(Hold(Source::kTeleop, 0.2, now_ + 0.5));
  hw_emergency_ = false;
  Run(now_ + 0.1);
  // Still held: nothing passes until a zero.
  for (int i = 0; i < 20; i++) {
    EXPECT_EQ(Cmd(Source::kTeleop, 0.2).refusal, "waits for a zero or 1.0 s without commands");
    Run(now_ + 0.1);
  }
  EXPECT_TRUE(Stops(Cmd(Source::kTeleop, 0)));
  EXPECT_TRUE(Hold(Source::kTeleop, 0.2, now_ + 0.3));
}

TEST_F(FilterTest, QuietSourceArmsAfterAnEmergency) {
  ASSERT_TRUE(Moves(Cmd(Source::kFollower, 0.2)));
  hw_emergency_ = true;
  Run(now_ + 0.3);
  hw_emergency_ = false;
  Run(now_ + 0.2);
  // The follower sent its last command 0.5 s ago.
  EXPECT_EQ(Cmd(Source::kFollower, 0.2).refusal, "waits for a zero or 1.0 s without commands");
  Run(now_ + 1.05);
  EXPECT_TRUE(Moves(Cmd(Source::kFollower, 0.2))) << filter_.LastRefusal();
}

TEST_F(FilterTest, StopAndAnyZeroStopWhateverMoves) {
  ASSERT_TRUE(Moves(Cmd(Source::kFollower, 0.2)));
  EXPECT_TRUE(Stops(Cmd(Source::kStop, 0.3)));
  EXPECT_EQ(filter_.Owner(now_), Source::kStop);
  // The follower goes on with its next path without arming again.
  EXPECT_TRUE(Moves(Cmd(Source::kFollower, 0.2)));
  EXPECT_TRUE(Stops(Cmd(Source::kTeleop, 0)));
  const Decision d = Cmd(Source::kFollower, std::nan(""));
  EXPECT_TRUE(Stops(d));
  EXPECT_EQ(d.refusal, "a command that is not a number, stopped");
}

TEST_F(FilterTest, CalibrationDrivesLikeTheFollowerForwardOnly) {
  Decision d = Cmd(Source::kCalibration, 0.2, 0.05);
  ASSERT_TRUE(Moves(d)) << d.refusal;
  EXPECT_EQ(filter_.Owner(now_), Source::kCalibration);
  // Turning on the spot is no backwards.
  EXPECT_TRUE(Moves(Cmd(Source::kCalibration, 0, -0.3)));
  // Backwards is refused with its reason, and the motion of the calibration stops at once.
  d = Cmd(Source::kCalibration, -0.1, 0.0);
  EXPECT_EQ(d.refusal, "the calibration drives forward only");
  EXPECT_TRUE(Stops(d));
  EXPECT_FALSE(d.conflict);
  EXPECT_EQ(filter_.Latched(), 0u);
  EXPECT_EQ(filter_.LastRefusal(), "calibration: the calibration drives forward only");
  // Even standing still.
  Run(now_ + 1.1);
  EXPECT_EQ(Cmd(Source::kCalibration, -0.05, 0.3).refusal, "the calibration drives forward only");
  EXPECT_TRUE(Moves(Cmd(Source::kCalibration, 0.2))) << filter_.LastRefusal();
}

TEST_F(FilterTest, CalibrationNeedsWhatAutonomousDrivesNeed) {
  const auto refused = [this](const std::string& why) {
    Run(now_ + 0.1);
    EXPECT_EQ(Cmd(Source::kCalibration, 0.2).refusal, why);
    EXPECT_EQ(filter_.Latched(), 0u);
    EXPECT_TRUE(Stops(Cmd(Source::kCalibration, 0)));
  };
  accuracy_ = 0.3;
  refused("position accuracy 0.300 m, 0.100 m needed");
  accuracy_ = 0.02;
  heading_valid_ = false;
  refused("the heading is not aligned");
  heading_valid_ = true;
  filter_.SetMapFits(false);
  refused("the map does not fit the datum (map/status)");
  filter_.SetMapFits(true);
  filter_.OnCapabilities(false, 0.3, 0.3);
  refused("no safety measurement of this platform (hw/capabilities)");
  filter_.OnCapabilities(true, 0.15, 0.3);
  Run(now_ + 0.1);
  // Within the limits of the driver.
  const Decision d = Cmd(Source::kCalibration, 0.2, 0.5);
  ASSERT_TRUE(Moves(d)) << d.refusal;
  EXPECT_DOUBLE_EQ(d.out->linear, 0.15);
  EXPECT_DOUBLE_EQ(d.out->angular, 0.3);
}

TEST_F(FilterTest, CalibrationOutsideTheAreasLatchesFence) {
  x_ = -0.7;
  y_ = 1;
  Run(now_ + 0.1);
  const Decision d = Cmd(Source::kCalibration, 0.2);
  EXPECT_EQ(d.refusal, "base_link is 0.70 m outside of the allowed areas at (-0.70, 1.00)");
  EXPECT_EQ(filter_.Latched(), kReasonFence);
}

TEST_F(FilterTest, CalibrationIsOneSourceAmongOthers) {
  ASSERT_TRUE(Moves(Cmd(Source::kFollower, 0.2)));
  Decision d = Cmd(Source::kCalibration, 0.2);
  EXPECT_TRUE(d.conflict);
  EXPECT_EQ(d.refusal, "the follower drives");
  EXPECT_TRUE(Stops(Cmd(Source::kStop, 0)));
  Run(now_ + 1.1);
  ASSERT_TRUE(Moves(Cmd(Source::kCalibration, 0.2))) << filter_.LastRefusal();
  d = Cmd(Source::kTeleop, 0.2);
  EXPECT_TRUE(d.conflict);
  EXPECT_EQ(d.refusal, "the calibration drives");
  // After an emergency the calibration has to arm again like any source.
  hw_emergency_ = true;
  Run(now_ + 0.2);
  hw_emergency_ = false;
  Run(now_ + 0.1);
  EXPECT_EQ(Cmd(Source::kCalibration, 0.2).refusal, "waits for a zero or 1.0 s without commands");
  EXPECT_TRUE(Stops(Cmd(Source::kCalibration, 0)));
  EXPECT_TRUE(Moves(Cmd(Source::kCalibration, 0.2))) << filter_.LastRefusal();
}

TEST_F(FilterTest, ATiltOfNoNumberIsNoSample) {
  ASSERT_TRUE(Moves(Cmd(Source::kTeleop, 0.2)));
  tilt_ = std::nan("");
  Run(now_ + 0.6);
  EXPECT_EQ(stops_, 1);
  EXPECT_NE(Cmd(Source::kTeleop, 0.2).refusal.find("no IMU sample for"), std::string::npos);
}

TEST_F(FilterTest, ATiltExactlyAtTheLimitDoesNotCount) {
  tilt_ = 31.0;
  Run(now_ + 1.5);
  EXPECT_EQ(filter_.Latched(), 0u);
}

TEST_F(FilterTest, TiltCountsFromTheFirstSampleOverTheLimit) {
  SafetyFilter filter = Ready(1.0);
  filter.OnTilt(35.0, 1.02);
  filter.OnTilt(35.0, 2.0);
  EXPECT_EQ(filter.Latched(), 0u);
  filter.OnTilt(35.0, 2.02);
  EXPECT_EQ(filter.Latched(), kReasonTilt);
}

// Limits at exact times, and just off them by rounding: 1.13 - 0.13 is just under 1 s, 1.07 - 0.57 just over 0.5 s.

TEST_F(FilterTest, TiltForExactlyTiltTimeLatches) {
  for (const auto& [since, now] : {std::pair{0.5, 1.5}, std::pair{0.13, 1.13}}) {
    SafetyFilter filter = Ready(0.0);
    filter.OnTilt(35.0, since);
    filter.OnTilt(35.0, now);
    EXPECT_EQ(filter.Latched(), kReasonTilt) << since;
  }
}

TEST_F(FilterTest, AnImuSampleExactlyImuTimeoutOldIsFresh) {
  for (const auto& [sample, now] : {std::pair{1.0, 1.5}, std::pair{0.57, 1.07}}) {
    SafetyFilter filter = Ready(sample);
    EXPECT_FALSE(std::isnan(filter.TiltDeg(now))) << sample;
    EXPECT_TRUE(Moves(filter.OnCommand(Source::kTeleop, 0.1, 0, now))) << sample;
  }
}

TEST_F(FilterTest, APoseExactlyPoseTimeoutOldIsRecent) {
  for (const auto& [pose, now] : {std::pair{1.0, 1.5}, std::pair{0.57, 1.07}}) {
    SafetyFilter filter = Ready(pose);
    filter.OnHardware(false, 0, now);
    filter.OnTilt(2.0, now);
    EXPECT_NEAR(filter.FenceDistance(now), 0, 1e-9) << pose;
    EXPECT_TRUE(filter.AutonomousOk(now)) << pose;
    EXPECT_TRUE(Moves(filter.OnCommand(Source::kFollower, 0.1, 0, now))) << pose;
  }
}

TEST_F(FilterTest, HwStatusExactlyHwTimeoutOldIsNoEmergency) {
  SafetyFilter filter = Ready(1.0);
  filter.OnTilt(2.0, 2.0);
  EXPECT_TRUE(Moves(filter.OnCommand(Source::kTeleop, 0.1, 0, 2.0)));
  EXPECT_EQ(filter.OnCommand(Source::kTeleop, 0.1, 0, 2.02).refusal, "emergency");
}

TEST_F(FilterTest, TheOwnerKeepsTheDriveForExactlyOwnerTimeout) {
  for (const auto& [last, now] : {std::pair{1.0, 1.5}, std::pair{0.57, 1.07}}) {
    SafetyFilter filter = Ready(last);
    ASSERT_TRUE(Moves(filter.OnCommand(Source::kTeleop, 0.1, 0, last)));
    Feed(filter, now);
    EXPECT_EQ(filter.Owner(now), Source::kTeleop) << last;
    EXPECT_TRUE(filter.OnCommand(Source::kFollower, 0.1, 0, now).conflict) << last;
    // Also between ticks.
    EXPECT_EQ(filter.Owner(now + 0.1), Source::kStop) << last;
  }
}

TEST_F(FilterTest, AnOwnerThatResumesAfterItsDriveEndedTakesItAnew) {
  // The follower drives at `last`, remote control arms with a zero, a tick comes; then the follower goes on and
  // stops. Remote control may move without another zero only if the follower kept the drive through the tick.
  const auto teleop_moves = [](double last, double tick) {
    SafetyFilter filter = Ready(last);
    EXPECT_TRUE(Moves(filter.OnCommand(Source::kFollower, 0.1, 0, last)));
    filter.OnCommand(Source::kTeleop, 0, 0, last + 0.01);
    Feed(filter, tick);
    filter.Tick(tick);
    Feed(filter, tick + 0.01);
    EXPECT_TRUE(Moves(filter.OnCommand(Source::kFollower, 0.1, 0, tick + 0.01)));
    filter.OnCommand(Source::kFollower, 0, 0, tick + 0.02);
    return Moves(filter.OnCommand(Source::kTeleop, 0.1, 0, tick + 0.03));
  };
  EXPECT_TRUE(teleop_moves(1.0, 1.2));
  EXPECT_TRUE(teleop_moves(1.0, 1.5));
  EXPECT_TRUE(teleop_moves(0.57, 1.07));
  EXPECT_FALSE(teleop_moves(1.0, 1.6));
}

TEST_F(FilterTest, ASourceQuietForArmQuietArms) {
  SafetyFilter filter = Ready(0.13);
  filter.OnHardware(true, 1, 0.13);
  EXPECT_EQ(filter.OnCommand(Source::kTeleop, 0.1, 0, 0.13).refusal, "emergency");
  Feed(filter, 1.13);
  EXPECT_TRUE(Moves(filter.OnCommand(Source::kTeleop, 0.1, 0, 1.13))) << filter.LastRefusal();
}

TEST_F(FilterTest, ALatchIsAskedForAgainAfterOneSecond) {
  SafetyFilter filter = Ready(0.13);
  filter.OnPose(-0.7, 1, 0.02, true, 0.13);
  filter.OnCommand(Source::kFollower, 0.1, 0, 0.13);
  ASSERT_TRUE(filter.TakeLatchRequest(0.13).has_value());
  EXPECT_FALSE(filter.TakeLatchRequest(1.12).has_value());
  const auto again = filter.TakeLatchRequest(1.13);
  ASSERT_TRUE(again.has_value());
  EXPECT_TRUE(again->again);
}

TEST_F(FilterTest, SilentHwStatusStopsAtTheNextTick) {
  SafetyFilter filter = Ready(1.0);
  ASSERT_TRUE(Moves(filter.OnCommand(Source::kTeleop, 0.2, 0, 1.0)));
  filter.OnTilt(2.0, 1.9);
  ASSERT_TRUE(Moves(filter.OnCommand(Source::kTeleop, 0.2, 0, 1.9)));
  filter.OnTilt(2.0, 2.0);
  EXPECT_FALSE(filter.Tick(2.0).has_value());
  filter.OnTilt(2.0, 2.01);
  const auto stop = filter.Tick(2.01);
  ASSERT_TRUE(stop.has_value());
  EXPECT_EQ(stop->linear, 0);
}

TEST_F(FilterTest, AnEmergencyBetweenTwoTicksStillDisarms) {
  ASSERT_TRUE(Moves(Cmd(Source::kTeleop, 0.2)));
  filter_.OnHardware(true, 1, now_);
  filter_.OnHardware(false, 0, now_);
  EXPECT_EQ(Cmd(Source::kTeleop, 0.2).refusal, "waits for a zero or 1.0 s without commands");
}

TEST_F(FilterTest, LimitsOfNoNumberLetNothingMove) {
  filter_.OnCapabilities(true, std::nan(""), std::numeric_limits<double>::infinity());
  const Decision d = Cmd(Source::kTeleop, 0.2, 0.2);
  ASSERT_TRUE(d.out.has_value());
  EXPECT_EQ(d.out->linear, 0);
  EXPECT_EQ(d.out->angular, 0);
}

TEST_F(FilterTest, AnAccuracyLimitOfZeroOrNoNumberIsIgnored) {
  filter_.SetMaxPositionAccuracy(0);
  filter_.SetMaxPositionAccuracy(-1);
  filter_.SetMaxPositionAccuracy(std::numeric_limits<double>::infinity());
  filter_.SetMaxPositionAccuracy(std::nan(""));
  // Exactly the limit is enough.
  accuracy_ = 0.1;
  Run(now_ + 0.1);
  EXPECT_TRUE(Moves(Cmd(Source::kFollower, 0.2))) << filter_.LastRefusal();
  accuracy_ = 0.3;
  Run(now_ + 0.1);
  EXPECT_EQ(Cmd(Source::kFollower, 0.2).refusal, "position accuracy 0.300 m, 0.100 m needed");
}

TEST_F(FilterTest, FenceDistanceIsNoNumberWithoutAMapThatFitsOrARecentPose) {
  Run(1.0);
  EXPECT_NEAR(filter_.FenceDistance(now_), 0, 1e-9);
  filter_.SetMapFits(false);
  EXPECT_TRUE(std::isnan(filter_.FenceDistance(now_)));
  filter_.SetMapFits(true);
  filter_.SetMap(R"({"areas": []})");
  EXPECT_TRUE(std::isnan(filter_.FenceDistance(now_)));
  filter_.SetMap(kMap);
  pose_ = false;
  Run(now_ + 0.6);
  EXPECT_TRUE(std::isnan(filter_.FenceDistance(now_)));
}

TEST_F(FilterTest, MarginsOfFenceAndObstacles) {
  // Exactly fence_margin outside still drives.
  x_ = -0.6;
  y_ = 1.5;
  Run(now_ + 0.1);
  EXPECT_DOUBLE_EQ(filter_.FenceDistance(now_), 0.6);
  EXPECT_TRUE(Moves(Cmd(Source::kFollower, 0.2))) << filter_.LastRefusal();
  x_ = 4.15;
  y_ = 2.5;
  Run(now_ + 0.1);
  EXPECT_EQ(Cmd(Source::kFollower, 0.2).refusal, "base_link is 0.15 m inside an obstacle at (4.15, 2.50)");
  // Exactly obstacle_margin deep still drives.
  SafetyConfig config;
  config.obstacle_margin = 0.125;
  SafetyFilter filter = Ready(1.0, config);
  filter.OnPose(4.125, 2.5, 0.02, true, 1.0);
  EXPECT_TRUE(Moves(filter.OnCommand(Source::kFollower, 0.1, 0, 1.0))) << filter.LastRefusal();
}

TEST_F(FilterTest, TheZoneOfTheStation) {
  // 2.1 m ahead of the station (east, where the mower leaves), 0.15 m behind it and 0.1 m to each side of its line.
  SafetyFilter filter = Ready(1.0, SafetyConfig{}, kFarStation);
  const auto distance = [&filter](double x, double y) {
    filter.OnPose(x, y, 0.02, true, 1.0);
    return filter.FenceDistance(1.0);
  };
  EXPECT_NEAR(distance(-3.0, 3), 0, 1e-9);
  EXPECT_NEAR(distance(-2.5, 3), 0.4, 1e-9);
  EXPECT_NEAR(distance(-5.5, 3), 0.35, 1e-9);
  EXPECT_NEAR(distance(-4, 3.5), 0.4, 1e-9);
}

TEST_F(FilterTest, TiltDuringAFenceLatchAddsItsBit) {
  x_ = -0.7;
  y_ = 1;
  Run(now_ + 0.1);
  Cmd(Source::kFollower, 0.2);
  ASSERT_EQ(filter_.Latched(), kReasonFence);
  tilt_ = 35;
  Run(now_ + 1.1);
  EXPECT_EQ(filter_.Latched(), kReasonFence | kReasonTilt);
}

TEST_F(FilterTest, AStopAZeroOrASilentSourceGetNoFurtherZero) {
  // A later emergency finds nothing moving.
  const auto emergency = [this] {
    hw_emergency_ = true;
    Run(now_ + 0.1);
    hw_emergency_ = false;
    Run(now_ + 1.1);
  };
  ASSERT_TRUE(Moves(Cmd(Source::kTeleop, 0.2)));
  EXPECT_TRUE(Stops(Cmd(Source::kStop, 0)));
  emergency();
  ASSERT_TRUE(Moves(Cmd(Source::kTeleop, 0.2)));
  EXPECT_TRUE(Stops(Cmd(Source::kTeleop, 0)));
  emergency();
  // Gone silent, the cmd_vel timeout of the driver stopped it.
  ASSERT_TRUE(Moves(Cmd(Source::kTeleop, 0.2)));
  Run(now_ + 0.6);
  emergency();
  EXPECT_EQ(stops_, 0);
}

TEST_F(FilterTest, ARefusedMovingSourceStopsOnce) {
  ASSERT_TRUE(Moves(Cmd(Source::kCalibration, 0.2)));
  EXPECT_TRUE(Stops(Cmd(Source::kCalibration, -0.1)));
  EXPECT_FALSE(Cmd(Source::kCalibration, -0.1).out.has_value());
}

TEST_F(FilterTest, AReleaseTheDriverRefusesBringsTheLatchBack) {
  ASSERT_TRUE(Moves(Cmd(Source::kTeleop, 0.2))) << filter_.LastRefusal();
  // Latched here, and the driver does not show it yet.
  filter_.OnRestartHold(true);
  filter_.OnRestartHold(false);
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  EXPECT_EQ(filter_.Latched(), 0u);
  // Refused or not answered: the emergency is back and goes to the driver again.
  filter_.OnReleaseAnswer(false, now_);
  EXPECT_EQ(filter_.Latched(), kReasonUnexpectedRestart);
  EXPECT_EQ(Cmd(Source::kTeleop, 0.2).refusal, "emergency");
  const auto latch = filter_.TakeLatchRequest(now_);
  ASSERT_TRUE(latch.has_value());
  EXPECT_EQ(latch->bits, kReasonUnexpectedRestart);
  // A second answer changes nothing, the next release goes through.
  filter_.OnReleaseAnswer(false, now_);
  EXPECT_EQ(filter_.Latched(), kReasonUnexpectedRestart);
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  filter_.OnReleaseAnswer(true, now_);
  EXPECT_EQ(filter_.Latched(), 0u);
  filter_.OnReleaseAnswer(false, now_);
  EXPECT_EQ(filter_.Latched(), 0u);
}

TEST_F(FilterTest, ARefusedReleaseOfWhatTheDriverShowsAsksItNothingNew) {
  filter_.OnRestartHold(true);
  ASSERT_TRUE(filter_.TakeLatchRequest(now_).has_value());
  hw_emergency_ = true;
  hw_reason_ = 1u | kReasonUnexpectedRestart;
  Run(now_ + 0.1);
  filter_.OnRestartHold(false);
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  filter_.OnReleaseAnswer(false, now_);
  EXPECT_EQ(filter_.Latched(), kReasonUnexpectedRestart);
  EXPECT_FALSE(filter_.TakeLatchRequest(now_).has_value());
  EXPECT_EQ(Cmd(Source::kTeleop, 0.2).refusal, "emergency");
}

TEST_F(FilterTest, AReleaseWithoutALatchLeavesNothingToBringBack) {
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  filter_.OnReleaseAnswer(false, now_);
  EXPECT_EQ(filter_.Latched(), 0u);
}

TEST_F(FilterTest, AReleaseDisarmsEverySource) {
  ASSERT_TRUE(Moves(Cmd(Source::kTeleop, 0.2)));
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  const Decision d = Cmd(Source::kTeleop, 0.2);
  EXPECT_EQ(d.refusal, "waits for a zero or 1.0 s without commands");
  EXPECT_FALSE(d.out.has_value());
}

TEST_F(FilterTest, ASourceThatTakesTheDriveDisarmsTheOthers) {
  EXPECT_TRUE(Stops(Cmd(Source::kTeleop, 0)));
  ASSERT_TRUE(Moves(Cmd(Source::kFollower, 0.2))) << filter_.LastRefusal();
  EXPECT_TRUE(Stops(Cmd(Source::kFollower, 0)));
  // Remote control armed before the follower drove, so it has to show a zero again.
  EXPECT_EQ(Cmd(Source::kTeleop, 0.2).refusal, "waits for a zero or 1.0 s without commands");
}

TEST_F(FilterTest, CountsRefusalsAndKeepsTheLast) {
  Cmd(Source::kFollower, std::nan(""));
  EXPECT_EQ(filter_.Refused(), 1u);
  EXPECT_EQ(filter_.LastRefusal(), "follower: a command that is not a number, stopped");
  Cmd(Source::kCalibration, -0.1);
  EXPECT_EQ(filter_.Refused(), 2u);
  EXPECT_EQ(filter_.LastRefusal(), "calibration: the calibration drives forward only");
}

TEST_F(FilterTest, AReleaseNamesTheAgeOfTheLastSampleAndNeedsLessThanTheReleaseAngle) {
  SafetyFilter filter = Ready(1.0);
  Release release = filter.OnRelease(1.7);
  EXPECT_EQ(release.message, "no IMU sample for 0.7 s, the tilt is unknown");
  EXPECT_DOUBLE_EQ(release.value, 0.7);
  filter.OnTilt(26.0, 1.8);
  release = filter.OnRelease(1.8);
  EXPECT_FALSE(release.allowed);
  EXPECT_EQ(release.code, "STILL_TILTED");
}

TEST_F(FilterTest, TheDockFollowerDrivesInTheStationFrameWithoutMapOrMapPose) {
  filter_.SetMap("{}");
  filter_.SetMapFits(false);
  pose_ = false;
  dock_ = true;
  Run(now_ + 0.6);
  EXPECT_FALSE(filter_.AutonomousOk(now_));
  EXPECT_TRUE(filter_.DockOk(now_));
  Decision d = Cmd(Source::kDock, -0.1, 0.05);
  ASSERT_TRUE(Moves(d)) << d.refusal;
  EXPECT_DOUBLE_EQ(d.out->linear, -0.1);
  EXPECT_DOUBLE_EQ(d.out->angular, 0.05);
  EXPECT_EQ(filter_.Owner(now_), Source::kDock);
  EXPECT_TRUE(Stops(Cmd(Source::kDock, 0)));
  // The pose from the marker lets only the dock follower drive, never the follower in the map.
  Run(now_ + 1.1);
  EXPECT_EQ(Cmd(Source::kFollower, 0.1).refusal, "no recent pose from the localization");
  EXPECT_EQ(Cmd(Source::kCalibration, 0.1).refusal, "no recent pose from the localization");
  EXPECT_EQ(filter_.Latched(), 0u);
}

TEST_F(FilterTest, TheDockFollowerNeedsARecentAccuratePoseFromTheMarkerWithAHeading) {
  dock_ = true;
  const auto refused = [this](const std::string& why) {
    Run(now_ + 0.1);
    EXPECT_FALSE(filter_.DockOk(now_));
    const Decision d = Cmd(Source::kDock, -0.1);
    EXPECT_EQ(d.refusal, why);
    // None of these latches, and remote control still drives.
    EXPECT_EQ(filter_.Latched(), 0u);
    EXPECT_TRUE(Stops(Cmd(Source::kDock, 0)));
    Run(now_ + 1.1);
    EXPECT_TRUE(Moves(Cmd(Source::kTeleop, 0.1))) << filter_.LastRefusal();
    EXPECT_TRUE(Stops(Cmd(Source::kTeleop, 0)));
    Run(now_ + 1.1);
  };
  // A good pose in the map does not stand in for the one from the marker, whose limit is its own: 0.03 m at the docked
  // place and 0.09 m per m from it, here 0.8 m.
  dock_accuracy_ = 0.103;
  refused("position accuracy from the dock marker 0.103 m, 0.102 m needed");
  filter_.SetMaxPositionAccuracy(0.5);
  refused("position accuracy from the dock marker 0.103 m, 0.102 m needed");
  dock_accuracy_ = std::numeric_limits<double>::infinity();
  refused("position accuracy from the dock marker inf m, 0.102 m needed");
  dock_accuracy_ = std::numeric_limits<double>::quiet_NaN();
  refused("position accuracy from the dock marker nan m, 0.102 m needed");
  dock_accuracy_ = 0.1;
  Run(now_ + 0.1);
  EXPECT_TRUE(Moves(Cmd(Source::kDock, -0.1))) << filter_.LastRefusal();
  EXPECT_TRUE(Stops(Cmd(Source::kDock, 0)));
  dock_heading_valid_ = false;
  refused("no heading from the dock marker");
  dock_heading_valid_ = true;
  dock_ = false;
  Run(now_ + 0.6);
  refused("no recent pose from the dock marker");
  dock_ = true;
  filter_.OnCapabilities(false, 0.3, 0.3);
  refused("no safety measurement of this platform (hw/capabilities)");
  filter_.OnCapabilities(true, 0.3, 0.3);
  Run(now_ + 0.1);
  EXPECT_TRUE(Moves(Cmd(Source::kDock, -0.1))) << filter_.LastRefusal();
}

TEST_F(FilterTest, ADockPoseExactlyPoseTimeoutOldIsRecent) {
  filter_.OnDockPose(-0.8, 0, 0.02, true, now_);
  Run(now_ + 0.5);
  EXPECT_TRUE(Moves(Cmd(Source::kDock, -0.1))) << filter_.LastRefusal();
  EXPECT_TRUE(Stops(Cmd(Source::kDock, 0)));
  Run(now_ + 0.02);
  EXPECT_EQ(Cmd(Source::kDock, -0.1).refusal, "no recent pose from the dock marker");
}

TEST_F(FilterTest, TheDockAccuracyLimitComesFromTheConfig) {
  SafetyConfig config;
  config.dock_max_position_accuracy = 0.05;
  config.dock_accuracy_per_m = 0.05;
  SafetyFilter filter = Ready(1.0, config);
  // Each moving command is followed by a zero, so that the next one starts from a stop.
  const auto refusal = [&filter](double x, double y, double accuracy) {
    filter.OnDockPose(x, y, accuracy, true, 1.0);
    const std::string why = filter.OnCommand(Source::kDock, -0.1, 0, 1.0).refusal;
    EXPECT_TRUE(Stops(filter.OnCommand(Source::kDock, 0, 0, 1.0)));
    return why;
  };
  // At the docked place the limit is dock_max_position_accuracy.
  EXPECT_EQ(refusal(0, 0, 0.049), "");
  EXPECT_EQ(refusal(0, 0, 0.051), "position accuracy from the dock marker 0.051 m, 0.050 m needed");
  // 1 m in front of it dock_accuracy_per_m worse.
  EXPECT_EQ(refusal(-1.0, 0, 0.099), "");
  EXPECT_EQ(refusal(-1.0, 0, 0.101), "position accuracy from the dock marker 0.101 m, 0.100 m needed");
}

TEST_F(FilterTest, WithoutGrowthTheDockAccuracyLimitIsTheSameEverywhere) {
  SafetyConfig config;
  config.dock_accuracy_per_m = 0;
  SafetyFilter filter = Ready(1.0, config);
  filter.OnDockPose(-0.8, 0, 0.031, true, 1.0);
  EXPECT_EQ(filter.OnCommand(Source::kDock, -0.1, 0, 1.0).refusal,
            "position accuracy from the dock marker 0.031 m, 0.030 m needed");
}

TEST_F(FilterTest, TheDockFollowerStaysInTheZoneOfTheStationInItsFrame) {
  filter_.SetMap("{}");
  dock_ = true;
  // The zone: 2.1 m in front of the docked pose (x negative), 0.15 m past it, 0.1 m to each side; FENCE beyond 0.6 m.
  const auto passes = [this](double x, double y) {
    dock_x_ = x;
    dock_y_ = y;
    Run(now_ + 0.1);
    const Decision d = Cmd(Source::kDock, -0.1);
    EXPECT_TRUE(Moves(d)) << x << ", " << y << ": " << d.refusal;
  };
  passes(0.1, 0);
  passes(-2.0, -0.05);
  passes(-0.8, 0.69);
  passes(0.74, 0);
  passes(-2.69, 0);
  dock_x_ = -0.8;
  dock_y_ = -0.75;
  Run(now_ + 0.1);
  const Decision d = Cmd(Source::kDock, -0.1);
  EXPECT_EQ(d.refusal, "base_link is 0.65 m outside of the zone of the station at (-0.80, -0.75) in its frame");
  EXPECT_TRUE(Stops(d));
  EXPECT_EQ(filter_.Latched(), kReasonFence);
  ASSERT_TRUE(filter_.TakeLatchRequest(now_).has_value());
  // Past the docked pose as well.
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  dock_x_ = 0.8;
  dock_y_ = 0;
  Run(now_ + 1.1);
  EXPECT_EQ(Cmd(Source::kDock, -0.1).refusal,
            "base_link is 0.65 m outside of the zone of the station at (0.80, 0.00) in its frame");
  EXPECT_EQ(filter_.Latched(), kReasonFence);
}

TEST_F(FilterTest, TheDockFollowerDrivesAtMostSeventeenForwardsAndTwelveBackwards) {
  dock_ = true;
  Run(now_ + 0.1);
  Decision d = Cmd(Source::kDock, 0.3, -0.5);
  ASSERT_TRUE(Moves(d)) << d.refusal;
  EXPECT_DOUBLE_EQ(d.out->linear, 0.17);
  EXPECT_DOUBLE_EQ(d.out->angular, -0.3);
  EXPECT_DOUBLE_EQ(Cmd(Source::kDock, -0.3).out->linear, -0.12);
  EXPECT_DOUBLE_EQ(Cmd(Source::kDock, 0.16).out->linear, 0.16);
  EXPECT_DOUBLE_EQ(Cmd(Source::kDock, -0.11).out->linear, -0.11);
  // A driver limit below them holds.
  filter_.OnCapabilities(true, 0.05, 0.3);
  EXPECT_DOUBLE_EQ(Cmd(Source::kDock, 0.3).out->linear, 0.05);
  EXPECT_DOUBLE_EQ(Cmd(Source::kDock, -0.3).out->linear, -0.05);
  // The follower in the map keeps the limit of the driver forwards.
  filter_.OnCapabilities(true, 0.3, 0.3);
  EXPECT_TRUE(Stops(Cmd(Source::kDock, 0)));
  Run(now_ + 1.1);
  EXPECT_DOUBLE_EQ(Cmd(Source::kFollower, 0.3).out->linear, 0.3);
}

TEST_F(FilterTest, TheDockFollowerIsOneSourceAmongOthers) {
  dock_ = true;
  Run(now_ + 0.1);
  ASSERT_TRUE(Moves(Cmd(Source::kFollower, 0.2)));
  Decision d = Cmd(Source::kDock, -0.1);
  EXPECT_TRUE(d.conflict);
  EXPECT_EQ(d.refusal, "the follower drives");
  EXPECT_TRUE(Stops(Cmd(Source::kStop, 0)));
  Run(now_ + 1.1);
  ASSERT_TRUE(Moves(Cmd(Source::kDock, -0.1))) << filter_.LastRefusal();
  d = Cmd(Source::kTeleop, 0.2);
  EXPECT_TRUE(d.conflict);
  EXPECT_EQ(d.refusal, "the dock follower drives");
  // An emergency stops it once, and it has to arm again like any source.
  const int stops = stops_;
  hw_emergency_ = true;
  Run(now_ + 0.2);
  EXPECT_EQ(stops_, stops + 1);
  hw_emergency_ = false;
  Run(now_ + 0.1);
  EXPECT_EQ(Cmd(Source::kDock, -0.1).refusal, "waits for a zero or 1.0 s without commands");
  EXPECT_TRUE(Stops(Cmd(Source::kDock, 0)));
  EXPECT_TRUE(Moves(Cmd(Source::kDock, -0.1))) << filter_.LastRefusal();
  // A tilt latches for it like for every source.
  tilt_ = 40;
  Run(now_ + 1.1);
  EXPECT_EQ(filter_.Latched(), kReasonTilt);
  EXPECT_EQ(Cmd(Source::kDock, -0.1).refusal, "emergency");
}

TEST_F(FilterTest, ACommandGrowsAtMostAtTheRampAndShrinksAtOnce) {
  SafetyConfig config;
  config.ramp_linear = 0.5;
  config.ramp_angular = 1.0;
  SafetyFilter filter = Ready(1.0, config);
  filter.OnCapabilities(true, 0.55, 0.7);
  double t = 1.0;
  const auto send = [&filter, &t](double linear, double angular) {
    Feed(filter, t);
    const Decision d = filter.OnCommand(Source::kTeleop, linear, angular, t);
    t += 0.05;
    return d.out.value_or(Twist2{-9, -9});
  };
  ASSERT_TRUE(Stops(filter.OnCommand(Source::kTeleop, 0, 0, t)));
  // From a stop one step of 0.1 s, then by the time between the commands.
  Twist2 out = send(0.55, 0.7);
  EXPECT_NEAR(out.linear, 0.05, 1e-9);
  EXPECT_NEAR(out.angular, 0.1, 1e-9);
  out = send(0.55, 0.7);
  EXPECT_NEAR(out.linear, 0.075, 1e-9);
  EXPECT_NEAR(out.angular, 0.15, 1e-9);
  for (int i = 0; i < 30; i++) out = send(0.55, 0.7);
  EXPECT_NEAR(out.linear, 0.55, 1e-9);
  EXPECT_NEAR(out.angular, 0.7, 1e-9);
  // Less goes out at once, the other direction grows from zero.
  out = send(0.1, -0.7);
  EXPECT_NEAR(out.linear, 0.1, 1e-9);
  EXPECT_NEAR(out.angular, -0.05, 1e-9);
  // A gap counts 0.1 s at most.
  t += 2.0;
  out = send(0.55, -0.7);
  EXPECT_NEAR(out.linear, 0.15, 1e-9);
  EXPECT_NEAR(out.angular, -0.15, 1e-9);
  // A zero stops at once, and the next command starts from a stop.
  EXPECT_TRUE(Stops(filter.OnCommand(Source::kTeleop, 0, 0, t)));
  out = send(0.55, 0.0);
  EXPECT_NEAR(out.linear, 0.05, 1e-9);
}

TEST_F(FilterTest, AnEmergencyStopsARampedCommandAtOnce) {
  SafetyConfig config;
  config.ramp_linear = 0.5;
  config.ramp_angular = 1.0;
  SafetyFilter filter = Ready(1.0, config);
  ASSERT_TRUE(Stops(filter.OnCommand(Source::kTeleop, 0, 0, 1.0)));
  double t = 1.0;
  for (int i = 0; i < 10; i++, t += 0.05) {
    Feed(filter, t);
    ASSERT_TRUE(Moves(filter.OnCommand(Source::kTeleop, 0.3, 0.3, t)));
  }
  filter.OnHardware(true, 1, t);
  const auto stop = filter.Tick(t);
  ASSERT_TRUE(stop);
  EXPECT_EQ(stop->linear, 0);
  EXPECT_EQ(stop->angular, 0);
  // After the release the source arms again and starts from a stop.
  Feed(filter, t + 0.05);
  ASSERT_TRUE(Stops(filter.OnCommand(Source::kTeleop, 0, 0, t + 0.05)));
  Feed(filter, t + 1.2);
  const Decision d = filter.OnCommand(Source::kTeleop, 0.3, 0.3, t + 1.2);
  ASSERT_TRUE(d.out);
  EXPECT_NEAR(d.out->linear, 0.05, 1e-9);
  EXPECT_NEAR(d.out->angular, 0.1, 1e-9);
}

TEST_F(FilterTest, AnotherSourceStartsFromAStop) {
  SafetyConfig config;
  config.ramp_linear = 0.5;
  SafetyFilter filter = Ready(1.0, config);
  ASSERT_TRUE(Stops(filter.OnCommand(Source::kTeleop, 0, 0, 1.0)));
  double t = 1.0;
  for (int i = 0; i < 10; i++, t += 0.05) {
    Feed(filter, t);
    ASSERT_TRUE(Moves(filter.OnCommand(Source::kTeleop, 0.3, 0, t)));
  }
  // Teleop stops, the follower arms and takes over after it from zero.
  ASSERT_TRUE(Stops(filter.OnCommand(Source::kTeleop, 0, 0, t)));
  ASSERT_TRUE(Stops(filter.OnCommand(Source::kFollower, 0, 0, t)));
  Feed(filter, t);
  const Decision d = filter.OnCommand(Source::kFollower, 0.3, 0, t);
  ASSERT_TRUE(d.out) << d.refusal;
  EXPECT_NEAR(d.out->linear, 0.05, 1e-9);
}

TEST_F(FilterTest, SetupMovesOnlyWithARecentMotionAndWithinItsLimits) {
  // Without setup/motion nothing of the setup passes.
  Decision d = Cmd(Source::kSetup, 0.05);
  EXPECT_FALSE(Moves(d));
  EXPECT_EQ(d.refusal, "no step of the setup program runs");
  filter_.OnSetupMotion(true, true, 0.08, 0.3, now_);
  d = Cmd(Source::kSetup, 0.2, 0.0);
  ASSERT_TRUE(Moves(d)) << d.refusal;
  EXPECT_DOUBLE_EQ(d.out->linear, 0.08);
  EXPECT_DOUBLE_EQ(Cmd(Source::kSetup, -0.2, -1.0).out->angular, -0.3);
  // The hard limits of the code hold whatever the step says.
  filter_.OnSetupMotion(true, true, 5.0, 5.0, now_);
  d = Cmd(Source::kSetup, 0.3, 0.3);
  EXPECT_DOUBLE_EQ(d.out->linear, kSetupMaxLinear);
  d = Cmd(Source::kSetup, 0.0, -0.3);
  EXPECT_DOUBLE_EQ(d.out->angular, -0.3);
  filter_.OnCapabilities(true, 0.3, 1.0);
  EXPECT_DOUBLE_EQ(Cmd(Source::kSetup, 0.0, 1.0).out->angular, kSetupMaxAngular);
  // No pose and no fence: the mower is jacked up.
  pose_ = false;
  Run(now_ + 0.3);
  filter_.OnSetupMotion(true, true, 0.1, 0.4, now_);
  EXPECT_TRUE(Moves(Cmd(Source::kSetup, 0.05))) << filter_.LastRefusal();
}

TEST_F(FilterTest, SetupStopsWhenItsMotionEndsOrGoesSilent) {
  filter_.OnSetupMotion(true, true, 0.1, 0.4, now_);
  const double began = now_;
  ASSERT_TRUE(Moves(Cmd(Source::kSetup, 0.05)));
  // Commands go on, setup/motion does not: the tick after its timeout stops the motion, commands are refused.
  const int stopped = stops_;
  while (now_ < began + 1.0) {
    Run(now_ + 0.02);
    if (stops_ != stopped) break;
    ASSERT_TRUE(Moves(Cmd(Source::kSetup, 0.05))) << filter_.LastRefusal();
  }
  EXPECT_EQ(stops_, stopped + 1);
  EXPECT_GT(now_ - began, kSetupMotionTimeout);
  EXPECT_LT(now_ - began, kSetupMotionTimeout + 0.05);
  const Decision d = Cmd(Source::kSetup, 0.05);
  EXPECT_FALSE(Moves(d));
  EXPECT_NE(d.refusal.find("no setup/motion for"), std::string::npos) << d.refusal;
  // Back again it moves; ended, the next tick stops it.
  filter_.OnSetupMotion(true, true, 0.1, 0.4, now_);
  ASSERT_TRUE(Moves(Cmd(Source::kSetup, 0.05))) << filter_.LastRefusal();
  const int stops = stops_;
  filter_.OnSetupMotion(false, false, 0, 0, now_);
  Run(now_ + 0.02);
  EXPECT_EQ(stops_, stops + 1);
  EXPECT_EQ(Cmd(Source::kSetup, 0.05).refusal, "no step of the setup program runs");
}

TEST_F(FilterTest, SetupOnTheGroundMovesWithinItsOwnLimitsWithoutPoseOrFence) {
  filter_.OnCapabilities(true, 0.5, 1.0);
  filter_.OnSetupMotion(true, false, 5.0, 5.0, now_);
  Decision d = Cmd(Source::kSetup, 0.5, 0.0);
  ASSERT_TRUE(Moves(d)) << d.refusal;
  EXPECT_DOUBLE_EQ(d.out->linear, kSetupGroundMaxLinear);
  EXPECT_DOUBLE_EQ(Cmd(Source::kSetup, -0.5, 0.0).out->linear, -kSetupGroundMaxLinear);
  EXPECT_DOUBLE_EQ(Cmd(Source::kSetup, 0.0, -1.0).out->angular, -kSetupGroundMaxAngular);
  // The step names less: that holds.
  filter_.OnSetupMotion(true, false, 0.2, 0.3, now_);
  d = Cmd(Source::kSetup, 0.5, 1.0);
  EXPECT_DOUBLE_EQ(d.out->linear, 0.2);
  EXPECT_DOUBLE_EQ(d.out->angular, 0.3);
  // Jacked up the bounds of the code are those of before.
  filter_.OnSetupMotion(true, true, 5.0, 5.0, now_);
  EXPECT_DOUBLE_EQ(Cmd(Source::kSetup, 0.5, 0.0).out->linear, kSetupMaxLinear);
  // Like the remote control: no pose and outside of every area, no fence latches.
  pose_ = false;
  Run(now_ + 0.3);
  filter_.OnSetupMotion(true, false, 0.3, 0.4, now_);
  EXPECT_TRUE(Moves(Cmd(Source::kSetup, 0.3))) << filter_.LastRefusal();
  pose_ = true;
  x_ = -20;
  Run(now_ + 0.1);
  filter_.OnSetupMotion(true, false, 0.3, 0.4, now_);
  EXPECT_TRUE(Moves(Cmd(Source::kSetup, 0.3))) << filter_.LastRefusal();
  EXPECT_EQ(filter_.Latched(), 0u);
}

TEST_F(FilterTest, AMoveOfTheSetupLastsTenSecondsAtMostThenWaitsForAZero) {
  for (const bool raised : {false, true}) {
    SCOPED_TRACE(raised ? "jacked up" : "on the ground");
    filter_.OnSetupMotion(true, raised, 0.1, 0.4, now_);
    ASSERT_TRUE(Stops(Cmd(Source::kSetup, 0.0)));
    const double began = now_;
    const int stopped = stops_;
    // Commands every 0.1 s and setup/motion every 0.2 s: the tick after the bound stops the move.
    while (now_ < began + kSetupMaxMove + 1.0 && stops_ == stopped) {
      filter_.OnSetupMotion(true, raised, 0.1, 0.4, now_);
      ASSERT_TRUE(Moves(Cmd(Source::kSetup, 0.05, 0.1))) << filter_.LastRefusal() << " at " << now_ - began;
      Run(now_ + 0.1);
    }
    EXPECT_EQ(stops_, stopped + 1);
    EXPECT_GT(now_ - began, kSetupMaxMove);
    EXPECT_LT(now_ - began, kSetupMaxMove + 0.15);
    // Moving commands stay refused, also after a pause, until a zero.
    filter_.OnSetupMotion(true, raised, 0.1, 0.4, now_);
    Decision d = Cmd(Source::kSetup, 0.05);
    EXPECT_FALSE(Moves(d));
    EXPECT_EQ(d.refusal, "a move of the setup lasts 10 s at most, it waits for a zero");
    Run(now_ + 0.5);
    filter_.OnSetupMotion(true, raised, 0.1, 0.4, now_);
    EXPECT_FALSE(Moves(Cmd(Source::kSetup, 0.05)));
    EXPECT_TRUE(Stops(Cmd(Source::kSetup, 0.0)));
    EXPECT_TRUE(Moves(Cmd(Source::kSetup, 0.05))) << filter_.LastRefusal();
    // A zero in between begins a new move.
    Hold(Source::kSetup, 0.0, now_ + 0.2);
    filter_.OnSetupMotion(false, raised, 0, 0, now_);
  }
}

TEST_F(FilterTest, TheBoundOfAMoveEndsWithItsCommandOrWithTheStep) {
  filter_.OnSetupMotion(true, false, 0.1, 0.4, now_);
  ASSERT_TRUE(Moves(Cmd(Source::kSetup, 0.05)));
  // Without commands the move ends with the timeout of its owner: the next one counts anew.
  for (int i = 0; i < 2; i++) {
    const double began = now_;
    while (now_ < began + kSetupMaxMove - 1.0) {
      filter_.OnSetupMotion(true, false, 0.1, 0.4, now_);
      ASSERT_TRUE(Moves(Cmd(Source::kSetup, 0.05))) << filter_.LastRefusal();
      Run(now_ + 0.1);
    }
    while (now_ < began + kSetupMaxMove + 1.0) {
      filter_.OnSetupMotion(true, false, 0.1, 0.4, now_);
      Run(now_ + 0.1);
    }
  }
  // Out of its bound, the end of the step takes the wait for a zero away.
  const double began = now_;
  while (now_ < began + kSetupMaxMove + 0.5) {
    filter_.OnSetupMotion(true, false, 0.1, 0.4, now_);
    Cmd(Source::kSetup, 0.05);
    Run(now_ + 0.1);
  }
  ASSERT_FALSE(Moves(Cmd(Source::kSetup, 0.05)));
  filter_.OnSetupMotion(false, false, 0, 0, now_);
  filter_.OnSetupMotion(true, false, 0.1, 0.4, now_);
  EXPECT_TRUE(Moves(Cmd(Source::kSetup, 0.05))) << filter_.LastRefusal();
}

TEST_F(FilterTest, SetupWithoutLimitsIsRefused) {
  filter_.OnSetupMotion(true, true, std::nan(""), 0.4, now_);
  EXPECT_EQ(Cmd(Source::kSetup, 0.05).refusal, "setup/motion without limits");
  filter_.OnSetupMotion(true, true, 0.1, std::nan(""), now_);
  EXPECT_EQ(Cmd(Source::kSetup, 0.05).refusal, "setup/motion without limits");
  filter_.OnSetupMotion(true, true, 0.0, 0.4, now_);
  EXPECT_EQ(Cmd(Source::kSetup, 0.05).refusal, "setup/motion without limits");
}

TEST_F(FilterTest, SetupStopsInAnEmergencyAndArmsAgainAfterIt) {
  filter_.OnSetupMotion(true, true, 0.1, 0.4, now_);
  ASSERT_TRUE(Moves(Cmd(Source::kSetup, 0.05)));
  hw_emergency_ = true;
  hw_reason_ = 1u << 2;
  const int stops = stops_;
  Run(now_ + 0.04);
  EXPECT_EQ(stops_, stops + 1);
  filter_.OnSetupMotion(true, true, 0.1, 0.4, now_);
  EXPECT_EQ(Cmd(Source::kSetup, 0.05).refusal, "emergency");
  hw_emergency_ = false;
  hw_reason_ = 0;
  Run(now_ + 0.1);
  filter_.OnSetupMotion(true, true, 0.1, 0.4, now_);
  // After the emergency it waits for a zero first.
  EXPECT_FALSE(Moves(Cmd(Source::kSetup, 0.05)));
  EXPECT_TRUE(Stops(Cmd(Source::kSetup, 0.0)));
  EXPECT_TRUE(Moves(Cmd(Source::kSetup, 0.05))) << filter_.LastRefusal();
}

TEST_F(FilterTest, SetupIsOneSourceAmongTheOthers) {
  ASSERT_TRUE(Moves(Cmd(Source::kTeleop, 0.2)));
  filter_.OnSetupMotion(true, true, 0.1, 0.4, now_);
  const Decision d = Cmd(Source::kSetup, 0.05);
  EXPECT_TRUE(d.conflict);
  EXPECT_EQ(d.refusal, "the remote control drives");
  EXPECT_STREQ(SourceName(Source::kSetup), "setup");
}

TEST_F(FilterTest, AClosedLockStopsAutonomousDrivesWhateverTheDateSays) {
  dock_ = true;
  Run(now_ + 0.2);
  ASSERT_TRUE(filter_.AutonomousOk(now_));
  ASSERT_TRUE(filter_.DockOk(now_));
  // The first look after the start finds it closed: no latch, but no autonomous drive either.
  filter_.OnSafetyLock(true, false, 2);
  EXPECT_TRUE(filter_.LockClosed());
  EXPECT_EQ(filter_.Latched(), 0u);
  EXPECT_FALSE(filter_.AutonomousOk(now_));
  EXPECT_FALSE(filter_.DockOk(now_));
  const std::string why = "the setup program closed the lock: a safety check failed";
  EXPECT_EQ(Cmd(Source::kFollower, 0.1).refusal, why);
  EXPECT_EQ(Cmd(Source::kCalibration, 0.1).refusal, why);
  EXPECT_EQ(Cmd(Source::kDock, 0.1).refusal, why);
  // Remote control and the setup program are free.
  Run(now_ + 1.1);
  EXPECT_TRUE(Moves(Cmd(Source::kTeleop, 0.2))) << filter_.LastRefusal();
  EXPECT_TRUE(Stops(Cmd(Source::kTeleop, 0)));
  filter_.OnSetupMotion(true, true, 0.08, 0.3, now_);
  EXPECT_TRUE(Moves(Cmd(Source::kSetup, 0.05))) << filter_.LastRefusal();
  EXPECT_TRUE(Stops(Cmd(Source::kSetup, 0)));
  // Opened again, the date counts.
  filter_.OnSafetyLock(false, false, 0);
  EXPECT_FALSE(filter_.LockClosed());
  EXPECT_TRUE(filter_.AutonomousOk(now_));
  EXPECT_TRUE(filter_.DockOk(now_));
}

TEST_F(FilterTest, AClosingStopsTheFollowerAtTheNextCommand) {
  ASSERT_TRUE(Moves(Cmd(Source::kFollower, 0.2))) << filter_.LastRefusal();
  filter_.OnSafetyLock(false, false, 0);
  filter_.OnSafetyLock(true, false, 1);
  EXPECT_TRUE(Stops(Cmd(Source::kFollower, 0.2)));
  EXPECT_EQ(filter_.Latched(), kReasonHighLevel);
}

TEST_F(FilterTest, AFailureLatchesHighLevelOnceAndTheReleaseLeavesTheLockClosed) {
  filter_.OnSafetyLock(false, false, 0);
  EXPECT_EQ(filter_.Latched(), 0u);
  filter_.OnSafetyLock(true, false, 1);
  EXPECT_EQ(filter_.Latched(), kReasonHighLevel);
  const auto latch = filter_.TakeLatchRequest(now_);
  ASSERT_TRUE(latch.has_value());
  EXPECT_EQ(latch->bits, kReasonHighLevel);
  EXPECT_EQ(latch->reason, "the setup program closed the lock: a safety check failed");
  // The same count again latches nothing new.
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  filter_.OnReleaseAnswer(true, now_);
  filter_.OnSafetyLock(true, false, 1);
  EXPECT_EQ(filter_.Latched(), 0u);
  EXPECT_TRUE(filter_.LockClosed());
  EXPECT_FALSE(filter_.AutonomousOk(now_));
  // The next failure, also with the lock still closed, latches again.
  filter_.OnSafetyLock(true, false, 2);
  EXPECT_EQ(filter_.Latched(), kReasonHighLevel);
}

TEST_F(FilterTest, TheFirstLookOnlyTakesTheCountAndAnUnreadableFileKeepsIt) {
  filter_.OnSafetyLock(true, false, 4);
  EXPECT_EQ(filter_.Latched(), 0u);
  // Unreadable: closed, and the count of the program starts anew after it.
  filter_.OnSafetyLock(true, true, 0);
  EXPECT_TRUE(filter_.LockClosed());
  EXPECT_EQ(filter_.Latched(), 0u);
  filter_.OnSafetyLock(true, false, 0);
  EXPECT_EQ(filter_.Latched(), 0u);
  filter_.OnSafetyLock(true, false, 1);
  EXPECT_EQ(filter_.Latched(), kReasonHighLevel);
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  filter_.OnReleaseAnswer(true, now_);
  // The same file readable again after a look that failed: nothing new.
  filter_.OnSafetyLock(true, true, 0);
  filter_.OnSafetyLock(true, false, 1);
  EXPECT_EQ(filter_.Latched(), 0u);
}

TEST_F(FilterTest, AnOpenLockKeepsTheCount) {
  filter_.OnSafetyLock(true, false, 3);
  filter_.OnSafetyLock(false, false, 0);
  EXPECT_EQ(filter_.Latched(), 0u);
  // The program keeps its count: the next failure is 4.
  filter_.OnSafetyLock(true, false, 4);
  EXPECT_EQ(filter_.Latched(), kReasonHighLevel);
}

TEST_F(FilterTest, TheSummaryClosingWithTheOldCountLatchesNothing) {
  // A failure earlier, the lock opened by a reset, then the summary closes it with the same count.
  filter_.OnSafetyLock(false, false, 0);
  filter_.OnSafetyLock(true, false, 1);
  ASSERT_EQ(filter_.Latched(), kReasonHighLevel);
  ASSERT_TRUE(filter_.OnRelease(now_).allowed);
  filter_.OnReleaseAnswer(true, now_);
  filter_.OnSafetyLock(false, false, 0);
  filter_.OnSafetyLock(true, false, 1);
  EXPECT_EQ(filter_.Latched(), 0u);
  EXPECT_TRUE(filter_.LockClosed());
  // A real failure while the file could not be read still latches.
  filter_.OnSafetyLock(true, true, 0);
  filter_.OnSafetyLock(true, false, 2);
  EXPECT_EQ(filter_.Latched(), kReasonHighLevel);
}
