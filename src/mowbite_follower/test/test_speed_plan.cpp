#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mowbite_follower/speed_plan.h"
#include "paths.h"

using mowbite_follower::Kind;
using mowbite_follower::Path;
using mowbite_follower::Pose2;
using mowbite_follower::SpeedLimits;
using mowbite_follower::SpeedPlan;
using namespace mowbite_follower::test;

namespace {

Path Built(const std::vector<Pose2>& poses, const std::vector<Kind>& kinds = {}) {
  Path path;
  EXPECT_EQ(path.Build(poses, kinds, 15.0 * kDeg), "");
  return path;
}

}  // namespace

// F3: fast where the next metre runs straight, braking at 0.1 m/s² to creep_speed at the end.
TEST(SpeedPlan, FastOnALineBrakingToTheEnd) {
  const Path path = Built(Line(0, 0, 0, 5.0));
  const SpeedPlan plan(path, SpeedLimits());
  EXPECT_DOUBLE_EQ(plan.cap(10), 0.3);
  EXPECT_DOUBLE_EQ(plan.at_pose(10), 0.3);
  EXPECT_DOUBLE_EQ(plan.at_pose(50), 0.06);
  // 0.1 m before the end: sqrt(0.06² + 2 * 0.1 * 0.1).
  EXPECT_NEAR(plan.at_pose(49), std::sqrt(0.0036 + 0.02), 1e-12);
  EXPECT_NEAR(plan.At(49, 0.05), std::sqrt(0.0036 + 0.01), 1e-12);
  // From 0.3 m/s the brake takes (0.09 - 0.0036) / 0.2 = 0.43 m.
  EXPECT_DOUBLE_EQ(plan.at_pose(45), 0.3);
  EXPECT_LT(plan.at_pose(46), 0.3);
  EXPECT_DOUBLE_EQ(plan.Floor(10), 0.06);
}

TEST(SpeedPlan, SlowWhereThePathBendsWithinAMetre) {
  std::vector<Pose2> poses = Line(0, 0, 0, 3.0);
  Append(poses, Line(3.0, 0, 25.0 * kDeg, 3.0), true);
  const Path path = Built(poses);
  const SpeedPlan plan(path, SpeedLimits());
  EXPECT_DOUBLE_EQ(plan.cap(15), 0.3);
  EXPECT_DOUBLE_EQ(plan.cap(25), 0.15);
  // A corner: creep there.
  EXPECT_DOUBLE_EQ(plan.at_pose(30), 0.06);
}

TEST(SpeedPlan, TurnsOfThePlannerAndArcs) {
  // A lane, a U-turn of radius 0.27 marked as turn, a lane back.
  std::vector<Pose2> poses = Line(0, 0, 0, 3.0);
  std::vector<Kind> kinds(poses.size(), Kind::kLane);
  const auto turn = Arc(poses.back(), 0.27, M_PI);
  Append(poses, turn);
  kinds.insert(kinds.end(), turn.size(), Kind::kTurn);
  const auto back = Line(poses.back().x, poses.back().y, poses.back().yaw, 3.0);
  Append(poses, back, true);
  kinds.insert(kinds.end(), back.size() - 1, Kind::kLane);
  const Path path = Built(poses, kinds);
  const SpeedPlan plan(path, SpeedLimits());
  // On the arc the path turns at turn_rate at most: 0.25 * 0.27 m/s.
  for (size_t i = 31; i + 1 < 30 + turn.size(); i++) EXPECT_NEAR(plan.cap(i), 0.25 * 0.27, 0.003) << i;
  // The segment into the arc is a turn, its curvature half.
  EXPECT_LE(plan.cap(30), 0.1);
  EXPECT_LE(plan.at_pose(30), plan.cap(30));
  // Slow before the arc, braked to it in time.
  EXPECT_DOUBLE_EQ(plan.cap(28), 0.15);
  EXPECT_LE(plan.at_pose(29), 0.15);
  EXPECT_NEAR(plan.at_pose(29), std::min(0.15, std::sqrt(std::pow(plan.at_pose(30), 2) + 0.02)), 1e-12);
  // Without the kinds an arc of a large radius is slow, not a turn.
  std::vector<Pose2> wide = Line(0, 0, 0, 1.0);
  Append(wide, Arc(wide.back(), 2.0, 90.0 * kDeg));
  EXPECT_DOUBLE_EQ(SpeedPlan(Built(wide), SpeedLimits()).cap(20), 0.15);
}

TEST(SpeedPlan, NeverAboveMaxLinear) {
  SpeedLimits limits;
  limits.max_linear = 0.04;
  const Path path = Built(Line(0, 0, 0, 2.0));
  const SpeedPlan plan(path, limits);
  for (size_t i = 0; i + 1 < path.size(); i++) {
    EXPECT_LE(plan.cap(i), 0.04);
    EXPECT_LE(plan.At(i, 0.05), 0.04);
    // Below creep_speed the floor follows the cap.
    EXPECT_DOUBLE_EQ(plan.Floor(i), 0.04);
  }
}

TEST(SpeedPlan, BrakesWithItsOwnDeceleration) {
  const Path path = Built(Line(0, 0, 0, 5.0));
  SpeedLimits limits;
  limits.acceleration = 0.5;
  limits.deceleration = 0.8;
  const SpeedPlan plan(path, limits);
  // 0.02 m before the end: sqrt(0.06² + 2 * 0.8 * 0.02), whatever acceleration says.
  EXPECT_NEAR(plan.At(49, 0.02), std::sqrt(0.0036 + 0.032), 1e-12);
  EXPECT_DOUBLE_EQ(plan.at_pose(49), 0.3);
  // From 0.3 m/s the brake takes (0.09 - 0.0036) / 1.6 = 0.054 m.
  EXPECT_DOUBLE_EQ(plan.at_pose(48), 0.3);
}
