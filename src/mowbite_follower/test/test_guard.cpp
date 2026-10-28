#include <gtest/gtest.h>

#include "mowbite_follower/guard.h"

using mowbite_follower::CheckGuard;
using mowbite_follower::DistanceToRing;
using mowbite_follower::Geofence;
using mowbite_follower::GuardConfig;
using mowbite_follower::GuardInput;
using mowbite_follower::InsideRing;
using mowbite_follower::Ring;
using mowbite_follower::StopReason;

namespace {

const Ring kLawn = {{0, 0}, {10, 0}, {10, 6}, {0, 6}};
const Ring kTree = {{4, 2}, {5, 2}, {5, 3}, {4, 3}};

GuardInput Healthy() {
  GuardInput in;
  in.safety_measured = true;
  in.hw_age = 0.1;
  in.emergency = false;
  in.pose_age = 0.05;
  in.pose = {2, 2, 0};
  in.position_accuracy = 0.02;
  in.heading_valid = true;
  in.cross_track = 0.05;
  return in;
}

}  // namespace

TEST(Geofence, InsideOutsideAndDistance) {
  EXPECT_TRUE(InsideRing(kLawn, 5, 3));
  EXPECT_FALSE(InsideRing(kLawn, -0.1, 3));
  Ring closed = kLawn;
  closed.push_back(closed.front());
  EXPECT_TRUE(InsideRing(closed, 9.9, 5.9));
  EXPECT_NEAR(DistanceToRing(kLawn, -0.3, 3), 0.3, 1e-9);
  EXPECT_NEAR(DistanceToRing(closed, 5, 3), 3.0, 1e-9);
}

TEST(Geofence, MarginObstaclesAndProblems) {
  const Geofence fence({kLawn}, {kTree}, 0.5);
  EXPECT_EQ(fence.Problem(), "");
  EXPECT_EQ(fence.Check(1, 1), "");
  EXPECT_EQ(fence.Check(-0.4, 3), "");
  EXPECT_EQ(fence.Check(-0.6, 3), "the mower is 0.60 m outside of the allowed areas at (-0.60, 3.00)");
  EXPECT_NE(fence.Check(4.5, 2.5).find("in an obstacle"), std::string::npos);
  EXPECT_EQ(Geofence({}, {}, 0.5).Problem(), "no allowed area");
  EXPECT_EQ(Geofence({{{0, 0}, {1, 1}}}, {}, 0.5).Problem(), "an allowed area has fewer than 3 points");
  // Two areas side by side: the gap between them is outside.
  const Ring other = {{12, 0}, {20, 0}, {20, 6}, {12, 6}};
  const Geofence two({kLawn, other}, {}, 0.5);
  EXPECT_EQ(two.Check(15, 3), "");
  EXPECT_NE(two.Check(11, 3), "");
}

TEST(Guard, HealthyPasses) {
  const Geofence fence({kLawn}, {kTree}, 0.5);
  EXPECT_EQ(CheckGuard(GuardConfig(), Healthy(), fence).reason, StopReason::kNone);
}

TEST(Guard, EachCheckStops) {
  const Geofence fence({kLawn}, {kTree}, 0.5);
  const GuardConfig config;
  auto expect = [&](GuardInput in, StopReason reason) {
    const auto result = CheckGuard(config, in, fence);
    EXPECT_EQ(result.reason, reason) << result.message;
    EXPECT_FALSE(result.message.empty());
  };
  GuardInput in = Healthy();
  in.safety_measured = false;
  expect(in, StopReason::kNotAllowed);
  in = Healthy();
  in.hw_age = 1.5;
  expect(in, StopReason::kEmergency);
  in = Healthy();
  in.emergency = true;
  expect(in, StopReason::kEmergency);
  in = Healthy();
  in.pose_age = 0.6;
  expect(in, StopReason::kPoseLost);
  in = Healthy();
  in.position_accuracy = 0.15;
  expect(in, StopReason::kPositionAccuracy);
  in = Healthy();
  in.position_accuracy = std::nanf("");
  expect(in, StopReason::kPositionAccuracy);
  in = Healthy();
  in.heading_valid = false;
  expect(in, StopReason::kHeading);
  in = Healthy();
  in.pose = {4.5, 2.5, 0};
  expect(in, StopReason::kOutside);
  in = Healthy();
  in.pose = {-1, 3, 0};
  expect(in, StopReason::kOutside);
  in = Healthy();
  in.cross_track = -0.51;
  expect(in, StopReason::kCrossTrack);
}

TEST(Guard, TheMostImportantReasonWins) {
  const Geofence fence({kLawn}, {}, 0.5);
  GuardInput in = Healthy();
  in.emergency = true;
  in.pose_age = 5;
  in.cross_track = 3;
  EXPECT_EQ(CheckGuard(GuardConfig(), in, fence).reason, StopReason::kEmergency);
  in.safety_measured = false;
  EXPECT_EQ(CheckGuard(GuardConfig(), in, fence).reason, StopReason::kNotAllowed);
}
