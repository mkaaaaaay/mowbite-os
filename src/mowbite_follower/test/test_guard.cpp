#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_follower/guard.h"

using mowbite_follower::CapAngular;
using mowbite_follower::CapLinear;
using mowbite_follower::CheckGuard;
using mowbite_follower::DistanceToRing;
using mowbite_follower::GateOpen;
using mowbite_follower::Geofence;
using mowbite_follower::GuardConfig;
using mowbite_follower::GuardInput;
using mowbite_follower::InsideRing;
using mowbite_follower::Ring;
using mowbite_follower::StopReason;
using mowbite_follower::TurnFixFresh;

namespace {

const Ring kLawn = {{0, 0}, {10, 0}, {10, 6}, {0, 6}};
const Ring kTree = {{4, 2}, {5, 2}, {5, 3}, {4, 3}};

GuardInput Healthy() {
  GuardInput in;
  in.safety_measured = true;
  in.driver_moves = true;
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
  // A point that is no number would make the area or obstacle what it is not.
  const Ring broken = {{4, 2}, {5, std::nan("")}, {5, 3}, {4, 3}};
  EXPECT_NE(Geofence({kLawn}, {broken}, 0.5).Problem(), "");
  EXPECT_NE(Geofence({broken}, {}, 0.5).Problem(), "");
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
  in.driver_moves = false;
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

// A path with a limit of its own: off it by more ends the path, wherever beside or past it the mower is.
TEST(Guard, AGoalWithItsOwnLimitStopsFartherOffThePath) {
  const Geofence fence({kLawn}, {}, 0.5);
  GuardInput in = Healthy();
  in.cross_track = 0;
  in.off_path = 0.4;
  EXPECT_EQ(CheckGuard(GuardConfig(), in, fence).reason, StopReason::kNone);
  in.max_off_path = 0.15;
  const auto result = CheckGuard(GuardConfig(), in, fence);
  EXPECT_EQ(result.reason, StopReason::kCrossTrack);
  EXPECT_EQ(result.message, "0.40 m off the path (limit of this path 0.15 m)");
  in.off_path = 0.15;
  EXPECT_EQ(CheckGuard(GuardConfig(), in, fence).reason, StopReason::kNone);
  in.off_path = 0.151;
  EXPECT_EQ(CheckGuard(GuardConfig(), in, fence).reason, StopReason::kCrossTrack);
  // A distance that is no number never passes.
  in.off_path = std::nan("");
  EXPECT_EQ(CheckGuard(GuardConfig(), in, fence).reason, StopReason::kCrossTrack);
  // The limit of the follower holds besides.
  in.off_path = 0.1;
  in.cross_track = 0.6;
  EXPECT_EQ(CheckGuard(GuardConfig(), in, fence).reason, StopReason::kCrossTrack);
  // What matters more still comes first.
  in.off_path = 0.4;
  in.emergency = true;
  EXPECT_EQ(CheckGuard(GuardConfig(), in, fence).reason, StopReason::kEmergency);
}

// A turn on the spot that slides farther than max_turn_slide from its place ends the path; without the limit and
// outside of a turn nothing does.
TEST(Guard, ATurnThatSlidesTooFarStops) {
  const Geofence fence({kLawn}, {}, 0.5);
  GuardConfig config;
  GuardInput in = Healthy();
  in.turn_slide = 0.4;
  EXPECT_EQ(CheckGuard(config, in, fence).reason, StopReason::kNone);
  config.max_turn_slide = 0.15;
  const auto result = CheckGuard(config, in, fence);
  EXPECT_EQ(result.reason, StopReason::kTurnSlide);
  EXPECT_EQ(result.message, "slid 0.40 m from the place of the turn on the spot (limit 0.15 m)");
  in.turn_slide = 0.15;
  EXPECT_EQ(CheckGuard(config, in, fence).reason, StopReason::kNone);
  in.turn_slide = std::nan("");
  EXPECT_EQ(CheckGuard(config, in, fence).reason, StopReason::kTurnSlide);
  // Outside of the area counts first.
  in.turn_slide = 0.4;
  in.pose = {-1, 3, 0};
  EXPECT_EQ(CheckGuard(config, in, fence).reason, StopReason::kOutside);
}

// The fix counts with its age now: the age it had at the pose plus the time since the pose.
TEST(Guard, TheFixOfATurnCountsWithTheAgeOfThePose) {
  EXPECT_TRUE(TurnFixFresh(0.2, 0.1, 0.5));
  EXPECT_TRUE(TurnFixFresh(0.25, 0.25, 0.5));
  EXPECT_FALSE(TurnFixFresh(0.4, 0.4, 0.5));
  EXPECT_FALSE(TurnFixFresh(0.0, 0.6, 0.5));
  EXPECT_FALSE(TurnFixFresh(0.0, 1e9, 0.5));
  EXPECT_FALSE(TurnFixFresh(std::nan(""), 0.0, 0.5));
  EXPECT_TRUE(TurnFixFresh(999, 1e9, 0));
}

TEST(Gate, APathWithGateMovesOnlyOnAFreshGo) {
  EXPECT_TRUE(GateOpen(false, -1.0, false, 0.5));
  EXPECT_TRUE(GateOpen(true, 0.1, true, 0.5));
  EXPECT_TRUE(GateOpen(true, 0.5, true, 0.5));
  // The logic holds the mower, its gate stopped coming (it hangs), or none came yet.
  EXPECT_FALSE(GateOpen(true, 0.1, false, 0.5));
  EXPECT_FALSE(GateOpen(true, 0.51, true, 0.5));
  EXPECT_FALSE(GateOpen(true, -1.0, true, 0.5));
}

TEST(Guard, APoseThatIsNoNumberStops) {
  for (int i = 0; i < 3; i++) {
    GuardInput in = Healthy();
    (i == 0 ? in.pose.x : i == 1 ? in.pose.y : in.pose.yaw) = std::nan("");
    EXPECT_EQ(CheckGuard(GuardConfig(), in, Geofence({kLawn}, {}, 0.5)).reason, StopReason::kPoseLost) << i;
  }
}

TEST(Gate, TheDriversTurnLimitKeepsTheCurve) {
  double linear = 0.3, angular = 0.6;
  CapAngular(0.4, &linear, &angular);
  EXPECT_NEAR(angular, 0.4, 1e-12);
  EXPECT_NEAR(linear, 0.2, 1e-12);
  linear = 0.0;
  angular = -0.7;
  CapAngular(0.5, &linear, &angular);
  EXPECT_NEAR(angular, -0.5, 1e-12);
  EXPECT_EQ(linear, 0.0);
}

TEST(Gate, SlowKeepsTheCurve) {
  double linear = 0.55, angular = 0.2;
  CapLinear(0.3, &linear, &angular);
  EXPECT_NEAR(linear, 0.3, 1e-12);
  EXPECT_NEAR(angular, 0.2 * 0.3 / 0.55, 1e-12);
  linear = -0.4;
  angular = -0.1;
  CapLinear(0.3, &linear, &angular);
  EXPECT_NEAR(linear, -0.3, 1e-12);
  EXPECT_NEAR(angular, -0.075, 1e-12);
  // Slower already, or turning on the spot: as it is.
  linear = 0.2;
  angular = 0.5;
  CapLinear(0.3, &linear, &angular);
  EXPECT_EQ(linear, 0.2);
  EXPECT_EQ(angular, 0.5);
  linear = 0.0;
  CapLinear(0.3, &linear, &angular);
  EXPECT_EQ(angular, 0.5);
}

namespace {

// The box of the Tron in base_link: 0.6 m ahead, 0.12 m behind, 0.235 m to each side.
const Ring kBody = {{0.6, 0.235}, {-0.12, 0.235}, {-0.12, -0.235}, {0.6, -0.235}};

}  // namespace

TEST(TurnRoom, TheSweptBodyStaysInTheLawnOrTheTurnIsRefused) {
  const Geofence fence({kLawn}, {}, 0.5);
  // In the open: any turn.
  EXPECT_EQ(fence.CheckTurn({5, 3, 0}, M_PI, kBody, 0.05), "");
  // 0.3 m beside the edge on its left: to the left the front corner (0.644 m) gets 0.34 m over it, to the right the
  // body stays in.
  const std::string left = fence.CheckTurn({5, 5.7, 0}, M_PI / 2, kBody, 0.05);
  EXPECT_NE(left.find("0.34 m out of the allowed areas"), std::string::npos) << left;
  EXPECT_EQ(fence.CheckTurn({5, 5.7, 0}, -M_PI / 2, kBody, 0.05), "");
  // Start and end in the lawn, the nose over the edge half way.
  EXPECT_NE(fence.CheckTurn({5, 5.5, 0}, M_PI, kBody, 0.05), "");
  EXPECT_EQ(fence.CheckTurn({5, 5.5, 0}, -M_PI, kBody, 0.05), "");
}

TEST(TurnRoom, TheToleranceCountsForTheCornerOfTheBody) {
  const Geofence fence({kLawn}, {}, 0.5);
  // The front corner reaches 0.644 m: 0.024 m over the edge.
  EXPECT_EQ(fence.CheckTurn({5, 5.38, 0}, M_PI / 2, kBody, 0.05), "");
  EXPECT_NE(fence.CheckTurn({5, 5.38, 0}, M_PI / 2, kBody, 0.01), "");
}

TEST(TurnRoom, NeighbouringAreasAreOneRoom) {
  const Ring west = {{0, 0}, {5, 0}, {5, 6}, {0, 6}};
  const Ring east = {{5, 0}, {10, 0}, {10, 6}, {5, 6}};
  EXPECT_EQ(Geofence({west, east}, {}, 0.5).CheckTurn({5, 3, 0}, 2 * M_PI - 0.1, kBody, 0.05), "");
}

TEST(TurnRoom, ObstaclesLargeAndSmall) {
  const Ring stone_ahead = {{2.25, 1.98}, {2.29, 1.98}, {2.29, 2.02}, {2.25, 2.02}};
  const Ring stone_left = {{1.98, 2.48}, {2.02, 2.48}, {2.02, 2.52}, {1.98, 2.52}};
  // A stone under the body already, smaller than it: no point of its edge reaches it.
  const std::string under = Geofence({kLawn}, {stone_ahead}, 0.5).CheckTurn({2, 2, 0}, 0.5, kBody, 0.05);
  EXPECT_NE(under.find("into an obstacle"), std::string::npos) << under;
  // 0.5 m to the left: the nose sweeps over it turning left, not turning right.
  const Geofence left({kLawn}, {stone_left}, 0.5);
  EXPECT_NE(left.CheckTurn({2, 2, 0}, M_PI / 2, kBody, 0.05), "");
  EXPECT_EQ(left.CheckTurn({2, 2, 0}, -M_PI / 2, kBody, 0.05), "");
  // The edge of a large obstacle 0.4 m ahead: the nose is in it already.
  EXPECT_NE(Geofence({kLawn}, {kTree}, 0.5).CheckTurn({3.6, 2.5, 0}, 0.2, kBody, 0.05), "");
  EXPECT_EQ(Geofence({kLawn}, {kTree}, 0.5).CheckTurn({3.2, 2.5, 0}, M_PI, kBody, 0.05), "");
}
