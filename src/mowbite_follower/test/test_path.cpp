#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_follower/path.h"
#include "paths.h"

using mowbite_follower::Kind;
using mowbite_follower::NormalizeAngle;
using mowbite_follower::Path;
using mowbite_follower::Pose2;
using namespace mowbite_follower::test;

namespace {

constexpr double kCorner = 15.0 * kDeg;

Path Built(const std::vector<Pose2>& poses) {
  Path path;
  EXPECT_EQ(path.Build(poses, {}, kCorner), "");
  return path;
}

}  // namespace

TEST(Path, NormalizeAngleIsHalfOpen) {
  EXPECT_NEAR(NormalizeAngle(3 * M_PI), M_PI, 1e-12);
  EXPECT_NEAR(NormalizeAngle(-M_PI), M_PI, 1e-12);
  EXPECT_NEAR(NormalizeAngle(-3.5 * M_PI), 0.5 * M_PI, 1e-12);
  EXPECT_NEAR(NormalizeAngle(0.25), 0.25, 1e-12);
}

// F1
TEST(Path, RejectsUnusablePoses) {
  Path path;
  EXPECT_EQ(path.Build(Line(0, 0, 0, 0.1), {}, kCorner), "a path needs at least 3 poses");
  const auto line = Line(0, 0, 0, 1.0);
  EXPECT_EQ(path.Build(line, {Kind::kLane}, kCorner), "kinds must be empty or one per pose");
  auto doubled = line;
  doubled.insert(doubled.begin() + 3, doubled[2]);
  EXPECT_EQ(path.Build(doubled, {}, kCorner), "pose 3 lies on the one before");
  EXPECT_EQ(path.Build({{0, 0, 0}, {0.0005, 0, 0}, {0.0009, 0, 0}}, {}, kCorner), "the poses of the path lie on one point");
  auto broken = line;
  broken[4].x = std::nan("");
  EXPECT_EQ(path.Build(broken, {}, kCorner), "pose 4 is not finite");
  broken = line;
  broken[5].yaw = INFINITY;
  EXPECT_EQ(path.Build(broken, {}, kCorner), "pose 5 is not finite");
  EXPECT_EQ(path.size(), 0u);
  EXPECT_EQ(path.Build(line, std::vector<Kind>(line.size(), Kind::kLane), kCorner), "");
  EXPECT_EQ(path.kind(3), Kind::kLane);
  EXPECT_EQ(path.Build(line, {}, kCorner), "");
  EXPECT_EQ(path.kind(3), Kind::kTransit);
}

// A pose within 1 mm of the one before is merged into it, the last one into the one before; indexes map back.
TEST(Path, MergesPosesWithinAMillimetre) {
  auto poses = Line(0, 0, 0, 1.0);
  poses.insert(poses.begin() + 4, {0.3004, 0.0003, 1.0});
  poses.push_back({1.0006, 0.0, M_PI / 2});
  Path path;
  ASSERT_EQ(path.Build(poses, {}, kCorner), "");
  EXPECT_EQ(path.size(), 11u);
  for (size_t i = 0; i < path.size(); i++) EXPECT_FALSE(path.corner(i)) << i;
  EXPECT_EQ(path.source(3), 3u);
  EXPECT_EQ(path.source(4), 5u);
  EXPECT_EQ(path.source(10), 12u);
  EXPECT_NEAR(path.pose(10).yaw, M_PI / 2, 1e-12);
  EXPECT_NEAR(path.pose(10).x, 1.0006, 1e-12);
  EXPECT_EQ(path.Merged(4), 3u);
  EXPECT_EQ(path.Merged(5), 4u);
  EXPECT_EQ(path.Merged(11), 9u);
  EXPECT_EQ(path.Merged(12), 10u);
}

TEST(Path, LengthsAndDirectionsOfTravel) {
  // Poses facing east, laid out westwards: the direction of travel is west.
  std::vector<Pose2> back = Line(0, 0, M_PI, 0.8);
  for (auto& p : back) p.yaw = 0;
  const Path path = Built(back);
  EXPECT_NEAR(path.s(8), 0.8, 1e-9);
  EXPECT_NEAR(path.length(2), 0.1, 1e-9);
  EXPECT_NEAR(std::abs(path.direction(0)), M_PI, 1e-9);
}

// F4: corners where the direction of travel turns by more than 15 deg, also the pose the planner puts 2.5 cm on.
TEST(Path, CornersOverFifteenDegrees) {
  for (const double degrees : {14.0, 16.0, 135.9, -169.4}) {
    const Path path = Built(SpinPath(degrees * kDeg));
    EXPECT_EQ(path.corner(11), std::abs(degrees) > 15.0) << degrees;
    for (size_t i = 0; i < path.size(); i++) {
      if (i != 11) {
        EXPECT_FALSE(path.corner(i)) << i;
      }
    }
    EXPECT_EQ(path.NextStop(0), std::abs(degrees) > 15.0 ? 11u : path.size() - 1);
  }
}

TEST(Path, ArcsOfTenDegreesPerPoseHaveNoCorners) {
  std::vector<Pose2> poses = Line(0, 0, 0, 1.0);
  Append(poses, Arc(poses.back(), 0.44, 210.0 * kDeg));
  const Path path = Built(poses);
  for (size_t i = 0; i < path.size(); i++) EXPECT_FALSE(path.corner(i)) << i;
  EXPECT_EQ(path.NextStop(0), path.size() - 1);
}

// The smoothed heading is the tangent of an arc at its poses and its curvature 1/r.
TEST(Path, HeadingAndCurvatureOnAnArc) {
  std::vector<Pose2> poses = Line(0, 0, 0, 1.0);
  const auto arc = Arc(poses.back(), 0.44, 180.0 * kDeg);
  Append(poses, arc);
  const Path path = Built(poses);
  for (size_t i = 12; i + 2 < path.size(); i++) {
    const auto foot = path.Project(path.pose(i).x, path.pose(i).y, i, i);
    EXPECT_NEAR(NormalizeAngle(foot.heading - path.pose(i).yaw), 0.0, 1e-9) << i;
    EXPECT_NEAR(path.curvature(i), 1.0 / 0.44, 0.01) << i;
  }
  EXPECT_NEAR(path.curvature(2), 0.0, 1e-12);
  // Right turns curve negative.
  std::vector<Pose2> right = Line(0, 0, 0, 1.0);
  Append(right, Arc(right.back(), 0.27, -90.0 * kDeg));
  EXPECT_NEAR(Built(right).curvature(12), -1.0 / 0.27, 0.02);
}

// F12: left of the direction of travel positive.
TEST(Path, CrossIsSignedLeftOfTravel) {
  const Path east = Built(Line(0, 0, 0, 2.0));
  EXPECT_NEAR(east.Project(0.2, 0.3, 0, 19).cross, 0.3, 1e-9);
  EXPECT_NEAR(east.Project(0.2, -0.4, 0, 19).cross, -0.4, 1e-9);
  std::vector<Pose2> back = Line(0, 0, M_PI, 2.0);
  for (auto& p : back) p.yaw = 0;
  // Travelling west, north is right.
  EXPECT_NEAR(Built(back).Project(-0.2, 0.3, 0, 19).cross, -0.3, 1e-9);
}

TEST(Path, ProjectionStaysInItsRangeWithOpenEnds) {
  const Path path = Built(Line(0, 0, 0, 2.0));
  auto foot = path.Project(0.55, 0.1, 3, 6);
  EXPECT_EQ(foot.segment, 5u);
  EXPECT_NEAR(foot.s, 0.55, 1e-9);
  EXPECT_NEAR(foot.along, 0.55, 1e-9);
  // Past the end of the range: along goes on, the distance to the line counts across.
  foot = path.Project(1.2, 0.1, 3, 6);
  EXPECT_EQ(foot.segment, 6u);
  EXPECT_NEAR(foot.s, 0.7, 1e-9);
  EXPECT_NEAR(foot.along, 1.2, 1e-9);
  EXPECT_NEAR(foot.cross, 0.1, 1e-9);
  // Before its start likewise.
  foot = path.Project(0.1, -0.05, 3, 6);
  EXPECT_EQ(foot.segment, 3u);
  EXPECT_NEAR(foot.along, 0.1, 1e-9);
  EXPECT_NEAR(foot.cross, -0.05, 1e-9);
  // On a pose shared by two segments the later one.
  EXPECT_EQ(path.Project(0.5, 0.0, 0, 19).segment, 5u);
}

// The U-turn of two lanes 0.54 m apart: seen from near the end of the first lane the lane back is nearest, but
// outside a range that ends on the arc.
TEST(Path, TheRangeKeepsTheLaneBackOut) {
  std::vector<Pose2> poses = Line(0, 0, 0, 3.0);
  Append(poses, Arc(poses.back(), 0.27, M_PI));
  Append(poses, Line(poses.back().x, poses.back().y, poses.back().yaw, 3.0), true);
  const Path path = Built(poses);
  EXPECT_LE(path.Project(2.5, 0.45, 20, 34).segment, 30u);
  EXPECT_GE(path.Project(2.5, 0.45, 20, path.size() - 2).segment, 49u);
}

TEST(Path, StraightAhead) {
  std::vector<Pose2> poses = Line(0, 0, 0, 2.0);
  Append(poses, Line(2.0, 0, 30.0 * kDeg, 1.0), true);
  const Path path = Built(poses);
  EXPECT_TRUE(path.StraightAhead(0, 1.0, 20.0 * kDeg));
  EXPECT_TRUE(path.StraightAhead(10, 1.0, 20.0 * kDeg));
  EXPECT_FALSE(path.StraightAhead(11, 1.0, 20.0 * kDeg));
  EXPECT_TRUE(path.StraightAhead(11, 1.0, 35.0 * kDeg));
  // Up to the end counts as straight.
  EXPECT_TRUE(path.StraightAhead(path.size() - 2, 1.0, 20.0 * kDeg));
}
