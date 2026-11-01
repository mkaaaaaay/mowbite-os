#include <gtest/gtest.h>

#include <cmath>

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
  imu_ = false;
  tilt_ = 10;
  Run(now_ + 0.6);
  release = filter_.OnRelease(now_);
  EXPECT_FALSE(release.allowed);
  EXPECT_NE(release.message.find("the tilt is unknown"), std::string::npos) << release.message;
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
