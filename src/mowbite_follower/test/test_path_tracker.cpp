#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mowbite_follower/path_tracker.h"

using mowbite_follower::Kind;
using mowbite_follower::NormalizeAngle;
using mowbite_follower::PathTracker;
using mowbite_follower::Phase;
using mowbite_follower::Pose2;
using mowbite_follower::TrackerConfig;

namespace {

constexpr double kDt = 0.05;

// Poses every spacing m along a straight line from (x, y) in direction yaw.
std::vector<Pose2> Line(double x, double y, double yaw, double length, double spacing = 0.1) {
  std::vector<Pose2> poses;
  const int n = static_cast<int>(std::round(length / spacing));
  for (int i = 0; i <= n; i++) {
    poses.push_back({x + i * spacing * std::cos(yaw), y + i * spacing * std::sin(yaw), yaw});
  }
  return poses;
}

// An arc of radius r (left turn for angle > 0) from pose p, poses every spacing m.
std::vector<Pose2> Arc(const Pose2& p, double r, double angle, double spacing = 0.1) {
  std::vector<Pose2> poses;
  const double sign = angle > 0 ? 1.0 : -1.0;
  const double cx = p.x - sign * r * std::sin(p.yaw);
  const double cy = p.y + sign * r * std::cos(p.yaw);
  const int n = std::max(2, static_cast<int>(std::ceil(std::abs(angle) * r / spacing)));
  for (int i = 1; i <= n; i++) {
    const double yaw = p.yaw + angle * i / n;
    poses.push_back({cx + sign * r * std::sin(yaw), cy - sign * r * std::cos(yaw), NormalizeAngle(yaw)});
  }
  return poses;
}

void Append(std::vector<Pose2>& path, const std::vector<Pose2>& more, bool skip_first = false) {
  path.insert(path.end(), more.begin() + (skip_first ? 1 : 0), more.end());
}

// The unicycle model of a mower without slip: base_link moves the way it faces.
struct Robot {
  Pose2 pose;
  double max_cross = 0;
  double max_linear = 0;
  double max_angular = 0;
  double max_accel = 0;
  double last_linear = 0;

  void Move(const PathTracker::Command& cmd, double dt) {
    max_linear = std::max(max_linear, std::abs(cmd.linear));
    max_angular = std::max(max_angular, std::abs(cmd.angular));
    pose.yaw = NormalizeAngle(pose.yaw + cmd.angular * dt * 0.5);
    pose.x += cmd.linear * dt * std::cos(pose.yaw);
    pose.y += cmd.linear * dt * std::sin(pose.yaw);
    pose.yaw = NormalizeAngle(pose.yaw + cmd.angular * dt * 0.5);
    last_linear = cmd.linear;
  }
};

// Drives until the tracker is done or failed, at most max_s; returns the simulated time.
double Drive(PathTracker& tracker, Robot& robot, double max_s = 300) {
  double t = 0;
  while (t < max_s && tracker.phase() != Phase::kDone && tracker.phase() != Phase::kFailed) {
    const auto cmd = tracker.Step(robot.pose, kDt);
    if (tracker.phase() == Phase::kFollowing) {
      robot.max_cross = std::max(robot.max_cross, std::abs(tracker.CrossTrack(robot.pose)));
    }
    robot.Move(cmd, kDt);
    t += kDt;
  }
  return t;
}

}  // namespace

TEST(PathTracker, RejectsUnusablePaths) {
  PathTracker tracker{TrackerConfig()};
  EXPECT_EQ(tracker.Start(Line(0, 0, 0, 0.1), {}, 0), "a path needs at least 3 poses");
  auto line = Line(0, 0, 0, 1.0);
  EXPECT_EQ(tracker.Start(line, {Kind::kLane}, 0), "kinds must be empty or one per pose");
  EXPECT_EQ(tracker.Start(line, {}, line.size() - 1), "start_index must lie before the last pose");
  auto doubled = line;
  doubled.insert(doubled.begin() + 3, doubled[2]);
  EXPECT_EQ(tracker.Start(doubled, {}, 0), "pose 3 lies on the one before");
  auto broken = line;
  broken[4].x = std::nan("");
  EXPECT_EQ(tracker.Start(broken, {}, 0), "pose 4 is not finite");
  EXPECT_EQ(tracker.phase(), Phase::kIdle);
  EXPECT_EQ(tracker.Step({}, kDt).linear, 0.0);
}

TEST(PathTracker, StraightLineStaysOnItWithinLimits) {
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 10.0), {}, 0), "");
  Robot robot;
  Drive(tracker, robot);
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_NEAR(robot.pose.x, 10.0, 0.1);
  EXPECT_NEAR(robot.pose.y, 0.0, 0.02);
  EXPECT_LT(robot.max_cross, 0.01);
  EXPECT_LE(robot.max_linear, 0.3 + 1e-9);
  EXPECT_LE(robot.max_angular, 0.3 + 1e-9);
  EXPECT_EQ(tracker.index(), 100u);
}

TEST(PathTracker, PullsBackOntoTheLineFromAnOffset) {
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 8.0), {}, 0), "");
  Robot robot;
  robot.pose = {0, 0.15, 0};
  double late_cross = 0;
  double t = 0;
  while (t < 200 && tracker.phase() != Phase::kDone && tracker.phase() != Phase::kFailed) {
    robot.Move(tracker.Step(robot.pose, kDt), kDt);
    if (robot.pose.x > 3.0) late_cross = std::max(late_cross, std::abs(robot.pose.y));
    t += kDt;
  }
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_LT(late_cross, 0.03);
}

TEST(PathTracker, ArcOfTheTurnRadius) {
  // The planner's turns: radius 0.27 m. At 0.3 rad/s the control point slows down to what the mower can turn.
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  Append(path, Arc(path.back(), 0.27, M_PI / 2));
  Append(path, Line(path.back().x, path.back().y, path.back().yaw, 1.0), true);
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(path, {}, 0), "");
  Robot robot;
  Drive(tracker, robot);
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_NEAR(robot.pose.x, path.back().x, 0.1);
  EXPECT_NEAR(robot.pose.y, path.back().y, 0.1);
  EXPECT_LT(robot.max_cross, 0.05);
  EXPECT_LE(robot.max_angular, 0.3 + 1e-9);
}

TEST(PathTracker, UTurnBetweenLanesSlowerOnTheTurn) {
  // Lane, U-turn of radius 0.27 to the left, lane back: the next lane 0.54 m over (lane_order skip).
  std::vector<Pose2> path = Line(0, 0, 0, 3.0);
  std::vector<Kind> kinds(path.size(), Kind::kLane);
  auto turn = Arc(path.back(), 0.27, M_PI);
  Append(path, turn);
  kinds.insert(kinds.end(), turn.size(), Kind::kTurn);
  auto back = Line(path.back().x, path.back().y, path.back().yaw, 3.0);
  Append(path, back, true);
  kinds.insert(kinds.end(), back.size() - 1, Kind::kLane);
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(path, kinds, 0), "");
  Robot robot;
  double turn_speed = 0;
  double t = 0;
  while (t < 300 && tracker.phase() != Phase::kDone && tracker.phase() != Phase::kFailed) {
    const auto cmd = tracker.Step(robot.pose, kDt);
    if (tracker.phase() == Phase::kFollowing) {
      robot.max_cross = std::max(robot.max_cross, std::abs(tracker.CrossTrack(robot.pose)));
      if (robot.pose.x > 3.1) turn_speed = std::max(turn_speed, cmd.linear);
    }
    robot.Move(cmd, kDt);
    t += kDt;
  }
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_NEAR(robot.pose.y, 0.54, 0.05);
  EXPECT_NEAR(robot.pose.x, 0.0, 0.1);
  EXPECT_LT(robot.max_cross, 0.08);
  // 0.3 rad/s on 0.27 m: at most about 0.08 m/s on the arc, plus what the PID adds to catch up.
  EXPECT_LT(turn_speed, 0.15);
}

TEST(PathTracker, TurnsOnTheSpotBeforeTheStart) {
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(Line(0, 0, M_PI / 2, 2.0), {}, 0), "");
  Robot robot;
  // First step: still turning, no forward motion.
  auto cmd = tracker.Step(robot.pose, kDt);
  EXPECT_EQ(tracker.phase(), Phase::kPreRotate);
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_GT(cmd.angular, 0.0);
  robot.Move(cmd, kDt);
  Drive(tracker, robot);
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_NEAR(robot.pose.y, 2.0, 0.1);
  EXPECT_NEAR(robot.pose.x, 0.0, 0.05);
}

TEST(PathTracker, GoesOnFromTheStartIndex) {
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 5.0), {}, 30), "");
  Robot robot;
  robot.pose = {3.0, 0.0, 0.0};
  Drive(tracker, robot);
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_NEAR(robot.pose.x, 5.0, 0.1);
}

TEST(PathTracker, RefusesAStartFarFromThePath) {
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 5.0), {}, 0), "");
  Robot robot;
  robot.pose = {0.0, 1.5, 0.0};
  const auto cmd = tracker.Step(robot.pose, kDt);
  EXPECT_EQ(tracker.phase(), Phase::kFailed);
  EXPECT_EQ(tracker.failure(), "the mower is too far from the start of the path");
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_EQ(cmd.angular, 0.0);
}

TEST(PathTracker, StuckMowerFails) {
  // The wheels turn, the mower does not move: the control point runs away until max_follow_distance.
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 10.0), {}, 0), "");
  Pose2 stuck;
  double t = 0;
  while (t < 60 && tracker.phase() != Phase::kFailed) {
    tracker.Step(stuck, kDt);
    t += kDt;
  }
  EXPECT_EQ(tracker.phase(), Phase::kFailed);
  EXPECT_NE(tracker.failure().find("stays behind"), std::string::npos) << tracker.failure();
  EXPECT_LT(t, 15.0);
  EXPECT_EQ(tracker.Step(stuck, kDt).linear, 0.0);
}

TEST(PathTracker, CancelStopsAtOnce) {
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 10.0), {}, 0), "");
  Robot robot;
  for (int i = 0; i < 100; i++) robot.Move(tracker.Step(robot.pose, kDt), kDt);
  EXPECT_GT(robot.last_linear, 0.0);
  tracker.Cancel();
  EXPECT_EQ(tracker.phase(), Phase::kIdle);
  const auto cmd = tracker.Step(robot.pose, kDt);
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_EQ(cmd.angular, 0.0);
}

TEST(PathTracker, CrossTrackIsSignedLeftPositive) {
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 2.0), {}, 0), "");
  EXPECT_NEAR(tracker.CrossTrack({0.2, 0.3, 0}), 0.3, 1e-9);
  EXPECT_NEAR(tracker.CrossTrack({0.2, -0.4, 0}), -0.4, 1e-9);
}

TEST(PathTracker, NeverExceedsTheLimitsEvenFarOff) {
  TrackerConfig config;
  PathTracker tracker{config};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 6.0), {}, 0), "");
  Robot robot;
  robot.pose = {0.0, 0.6, -0.5};
  Drive(tracker, robot, 120);
  EXPECT_LE(robot.max_linear, config.max_linear + 1e-9);
  EXPECT_LE(robot.max_angular, config.max_angular + 1e-9);
}
