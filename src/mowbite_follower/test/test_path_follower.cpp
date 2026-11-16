#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mowbite_follower/path_follower.h"
#include "paths.h"

using mowbite_follower::Failure;
using mowbite_follower::FollowerConfig;
using mowbite_follower::Kind;
using mowbite_follower::NormalizeAngle;
using mowbite_follower::PathFollower;
using mowbite_follower::Phase;
using mowbite_follower::Pose2;
using namespace mowbite_follower::test;

namespace {

constexpr double kDt = 0.05;

// A mower on two wheels. Without faults base_link moves the way it faces at the commanded speeds; with them, like
// the real variant of the sim: the setpoint split on a track of 0.40 m, the base turning on 0.395 m, a wheel slower
// than 0.05 m/s standing.
struct Robot {
  Pose2 pose;
  bool faults = false;
  double max_cross = 0;
  double max_linear = 0;
  double max_angular = 0;
  double max_forwards = 0;
  double last_linear = 0;
  // The share of a commanded turn on the spot the base makes: wheels that slip.
  double turn_share = 1.0;
  // The share of its speed each wheel loses on the ground while the two run against each other, and the m base_link
  // slides towards +y per rad it turns.
  double slip_left = 0;
  double slip_right = 0;
  double slide_per_rad = 0;
  // rad the wheels turned the base by their own count in the last move, m/s the slower wheel ran at least while the
  // mower drove in a turn on the spot, and m base_link got from far_from at most.
  double wheel_turn = 0;
  double slowest_wheel = 1e9;
  Pose2 far_from;
  double farthest = 0;

  void Move(const PathFollower::Command& cmd, double dt) {
    max_linear = std::max(max_linear, std::abs(cmd.linear));
    max_angular = std::max(max_angular, std::abs(cmd.angular));
    max_forwards = std::max(max_forwards, cmd.linear);
    last_linear = cmd.linear;
    double v = cmd.linear;
    double w = cmd.angular;
    if (faults) {
      double left = v - w * 0.2;
      double right = v + w * 0.2;
      if (std::abs(left) < 0.05) left = 0;
      if (std::abs(right) < 0.05) right = 0;
      v = (left + right) / 2;
      w = (right - left) / 0.395;
    }
    if (cmd.linear == 0.0) w *= turn_share;
    wheel_turn = cmd.angular * dt;
    double left = cmd.linear - cmd.angular * 0.2;
    double right = cmd.linear + cmd.angular * 0.2;
    if (left * right < 0) {
      if (cmd.linear != 0.0) slowest_wheel = std::min({slowest_wheel, std::abs(left), std::abs(right)});
      if (slip_left != 0 || slip_right != 0) {
        left *= 1.0 - slip_left;
        right *= 1.0 - slip_right;
        v = (left + right) / 2;
        w = (right - left) / 0.4;
      }
    }
    pose.yaw = NormalizeAngle(pose.yaw + w * dt * 0.5);
    pose.x += v * dt * std::cos(pose.yaw);
    pose.y += v * dt * std::sin(pose.yaw) + slide_per_rad * std::abs(w) * dt;
    pose.yaw = NormalizeAngle(pose.yaw + w * dt * 0.5);
    farthest = std::max(farthest, std::hypot(pose.x - far_from.x, pose.y - far_from.y));
  }
};

bool Running(const PathFollower& follower) {
  return follower.phase() != Phase::kDone && follower.phase() != Phase::kFailed && follower.phase() != Phase::kIdle;
}

// Drives until the follower is done or failed, at most max_s; returns the simulated time.
double Drive(PathFollower& follower, Robot& robot, double max_s = 300) {
  double t = 0;
  while (t < max_s && Running(follower)) {
    const auto cmd = follower.Step(robot.pose, kDt, robot.wheel_turn);
    if (follower.OnPath()) robot.max_cross = std::max(robot.max_cross, std::abs(follower.CrossTrack(robot.pose)));
    robot.Move(cmd, kDt);
    t += kDt;
  }
  return t;
}

std::vector<Pose2> UTurn(std::vector<Kind>* kinds) {
  // Lane, U-turn of radius 0.27 to the left, lane back: the next lane 0.54 m over (lane_order skip).
  std::vector<Pose2> path = Line(0, 0, 0, 3.0);
  kinds->assign(path.size(), Kind::kLane);
  const auto turn = Arc(path.back(), 0.27, M_PI);
  Append(path, turn);
  kinds->insert(kinds->end(), turn.size(), Kind::kTurn);
  const auto back = Line(path.back().x, path.back().y, path.back().yaw, 3.0);
  Append(path, back, true);
  kinds->insert(kinds->end(), back.size() - 1, Kind::kLane);
  return path;
}

}  // namespace

// F1
TEST(PathFollower, RejectsUnusablePaths) {
  PathFollower follower{FollowerConfig()};
  EXPECT_EQ(follower.Start(Line(0, 0, 0, 0.1), {}, 0), "a path needs at least 3 poses");
  const auto line = Line(0, 0, 0, 1.0);
  EXPECT_EQ(follower.Start(line, {Kind::kLane}, 0), "kinds must be empty or one per pose");
  EXPECT_EQ(follower.Start(line, {}, line.size() - 1), "start_index must lie before the last pose");
  auto doubled = line;
  doubled.insert(doubled.begin() + 3, doubled[2]);
  EXPECT_EQ(follower.Start(doubled, {}, 0), "pose 3 lies on the one before");
  EXPECT_EQ(follower.phase(), Phase::kIdle);
  const auto cmd = follower.Step({}, kDt);
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_EQ(cmd.angular, 0.0);
}

// F1, F8
TEST(PathFollower, StraightOnlyOnALine) {
  FollowerConfig config;
  config.straight = true;
  PathFollower follower{config};
  const std::string off = "a straight path must lie on one line, its poses facing along it";
  std::vector<Pose2> bent = Line(0, 0, 0, 0.5);
  Append(bent, Arc(bent.back(), 0.5, 0.5));
  EXPECT_EQ(follower.Start(bent, {}, 0), off);
  std::vector<Pose2> askew = Line(0, 0, 0, 0.8);
  askew[3].yaw = 0.1;
  EXPECT_EQ(follower.Start(askew, {}, 0), off);
  std::vector<Pose2> back = Line(0, 0, 0, 0.8);
  std::reverse(back.begin(), back.end());
  EXPECT_EQ(follower.Start(back, {}, 0), off);
  EXPECT_EQ(follower.Start(Line(0, 0, 2.0, 0.8), {}, 0), "");
  // Backwards the poses face against the way of travel.
  config.reverse = true;
  PathFollower backwards{config};
  for (auto& p : back) p.yaw = M_PI;
  EXPECT_EQ(backwards.Start(back, {}, 0), off);
  for (auto& p : back) p.yaw = 0;
  EXPECT_EQ(backwards.Start(back, {}, 0), "");
}

// F1: what the planner may send, two poses less than 1 mm apart (pose 132 of the full run, 01.11.), is followed and
// the index counts the poses as sent.
TEST(PathFollower, FollowsPosesLessThanAMillimetreApart) {
  auto poses = Line(0, 0, 0, 3.0);
  poses.insert(poses.begin() + 13, {1.2002, 0.0, 0.0});
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(poses, {}, 0), "");
  Robot robot;
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_EQ(follower.index(), poses.size() - 1);
  EXPECT_LT(robot.max_cross, 0.001);
  ASSERT_EQ(follower.Start(poses, {}, 20), "");
  robot.pose = {1.9, 0.0, 0.0};
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
}

// F2
TEST(PathFollower, RefusesAStartFarFromThePath) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 5.0), {}, 0), "");
  const auto cmd = follower.Step({0.0, 1.05, 0.0}, kDt);
  EXPECT_EQ(follower.phase(), Phase::kFailed);
  EXPECT_EQ(follower.failure_kind(), Failure::kStart);
  EXPECT_NE(follower.failure().find("too far from the start of the path"), std::string::npos) << follower.failure();
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_EQ(cmd.angular, 0.0);
}

// F2: within 1 m it turns towards the path on the spot and comes onto it.
TEST(PathFollower, ComesOntoThePathFromBesideTheStart) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 5.0), {}, 0), "");
  Robot robot;
  robot.pose = {0.0, 0.9, 0.0};
  const auto first = follower.Step(robot.pose, kDt);
  EXPECT_EQ(follower.phase(), Phase::kStartTurn);
  EXPECT_FALSE(follower.OnPath());
  EXPECT_EQ(first.linear, 0.0);
  EXPECT_LT(first.angular, 0.0);
  robot.Move(first, kDt);
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_NEAR(robot.pose.x, 5.0, 0.1);
  EXPECT_LT(std::abs(robot.pose.y), 0.01);
  // Counted only once on the path.
  EXPECT_LT(robot.max_cross, 0.16);
}

TEST(PathFollower, TurnsOnTheSpotBeforeTheStart) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, M_PI / 2, 2.0), {}, 0), "");
  Robot robot;
  // First step: still turning, no forward motion.
  auto cmd = follower.Step(robot.pose, kDt);
  EXPECT_EQ(follower.phase(), Phase::kStartTurn);
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_GT(cmd.angular, 0.0);
  robot.Move(cmd, kDt);
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_NEAR(robot.pose.y, 2.0, 0.1);
  EXPECT_NEAR(robot.pose.x, 0.0, 0.01);
}

// F2, F8
TEST(PathFollower, StraightNeverTurnsOnTheSpotAtTheStart) {
  FollowerConfig config;
  config.straight = true;
  PathFollower follower{config};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 0.8), {}, 0), "");
  const auto cmd = follower.Step({0, 0, 30.0 * kDeg}, kDt);
  EXPECT_EQ(follower.phase(), Phase::kFailed);
  EXPECT_EQ(follower.failure_kind(), Failure::kStart);
  EXPECT_NE(follower.failure().find("start of the path"), std::string::npos) << follower.failure();
  EXPECT_NE(follower.failure().find("30 deg"), std::string::npos) << follower.failure();
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_EQ(cmd.angular, 0.0);
}

// F3, F6, F12
TEST(PathFollower, StraightLineStaysOnItWithinLimits) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 10.0), {}, 0), "");
  Robot robot;
  size_t index = 0;
  bool monotonic = true;
  double t = 0;
  while (t < 300 && Running(follower)) {
    const auto cmd = follower.Step(robot.pose, kDt);
    if (follower.OnPath()) robot.max_cross = std::max(robot.max_cross, std::abs(follower.CrossTrack(robot.pose)));
    monotonic = monotonic && follower.index() >= index;
    index = follower.index();
    // The index is the pose the mower got to.
    if (follower.phase() == Phase::kFollowing) {
      EXPECT_LE(follower.index() * 0.1, robot.pose.x + 1e-9);
    }
    robot.Move(cmd, kDt);
    t += kDt;
  }
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_TRUE(monotonic);
  EXPECT_NEAR(robot.pose.x, 10.0, 0.02);
  EXPECT_NEAR(robot.pose.y, 0.0, 0.001);
  EXPECT_LT(robot.max_cross, 0.001);
  EXPECT_NEAR(robot.max_linear, 0.3, 1e-9);
  EXPECT_LE(robot.max_angular, 0.3 + 1e-9);
  EXPECT_EQ(follower.index(), 100u);
  // 10 m at 0.3 m/s plus speeding up and braking at 0.1 m/s²: about 36 s.
  EXPECT_LT(t, 38.0);
}

// F3: speeding up at 0.1 m/s² from the creep speed, never faster.
TEST(PathFollower, SpeedsUpAtTheAcceleration) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 10.0), {}, 0), "");
  Robot robot;
  double last = 0;
  for (int i = 0; i < 100; i++) {
    const auto cmd = follower.Step(robot.pose, kDt);
    if (i == 0) {
      EXPECT_NEAR(cmd.linear, 0.06, 1e-9);
    } else {
      EXPECT_LE(cmd.linear - last, 0.1 * kDt + 1e-9);
    }
    last = cmd.linear;
    robot.Move(cmd, kDt);
  }
  EXPECT_NEAR(last, 0.3, 1e-9);
}

TEST(PathFollower, PullsBackOntoTheLineFromAnOffset) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 8.0), {}, 0), "");
  Robot robot;
  robot.pose = {0, 0.15, 0};
  double late_cross = 0;
  double below = 0;
  double t = 0;
  while (t < 200 && Running(follower)) {
    robot.Move(follower.Step(robot.pose, kDt), kDt);
    if (robot.pose.x > 3.0) late_cross = std::max(late_cross, std::abs(robot.pose.y));
    below = std::min(below, robot.pose.y);
    t += kDt;
  }
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  // Before: under 3 cm after 3 m.
  EXPECT_LT(late_cross, 0.001);
  // Without swinging over the line.
  EXPECT_GT(below, -0.002);
}

// F3, F4: the planner's turns of radius 0.27 m, slowed down to what the mower can turn.
TEST(PathFollower, ArcOfTheTurnRadius) {
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  Append(path, Arc(path.back(), 0.27, M_PI / 2));
  Append(path, Line(path.back().x, path.back().y, path.back().yaw, 1.0), true);
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(path, {}, 0), "");
  Robot robot;
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_NEAR(robot.pose.x, path.back().x, 0.02);
  EXPECT_NEAR(robot.pose.y, path.back().y, 0.02);
  // Before: under 5 cm.
  EXPECT_LT(robot.max_cross, 0.01);
  EXPECT_LE(robot.max_angular, 0.3 + 1e-9);
}

// F3
TEST(PathFollower, UTurnBetweenLanesSlowerOnTheTurn) {
  std::vector<Kind> kinds;
  const auto path = UTurn(&kinds);
  // Driven as an arc, and as the row of corners an arc of 0.27 m is below min_arc_radius.
  for (const double min_arc_radius : {0.0, FollowerConfig().min_arc_radius}) {
    SCOPED_TRACE(min_arc_radius);
    FollowerConfig config;
    config.min_arc_radius = min_arc_radius;
    PathFollower follower{config};
    ASSERT_EQ(follower.Start(path, kinds, 0), "");
    Robot robot;
    double turn_speed = 0;
    double t = 0;
    while (t < 300 && Running(follower)) {
      const auto cmd = follower.Step(robot.pose, kDt);
      if (follower.OnPath()) {
        robot.max_cross = std::max(robot.max_cross, std::abs(follower.CrossTrack(robot.pose)));
        if (robot.pose.x > 3.05) turn_speed = std::max(turn_speed, cmd.linear);
      }
      robot.Move(cmd, kDt);
      t += kDt;
    }
    EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
    EXPECT_NEAR(robot.pose.y, 0.54, 0.01);
    EXPECT_NEAR(robot.pose.x, 0.0, 0.02);
    // Before: under 8 cm.
    EXPECT_LT(robot.max_cross, 0.01);
    // As an arc 0.25 rad/s on 0.27 m: about 0.07 m/s; between corners at most speed_turn.
    EXPECT_LT(turn_speed, min_arc_radius > 0 ? 0.1 + 1e-9 : 0.075);
  }
}

// F4: the way into the lanes of a small lawn (sim, 28.10.): a transit heading 114 deg, then a lane heading 0 deg from
// the pose where it ends.
TEST(PathFollower, StopsOnACornerAndTurnsOnTheSpot) {
  const double in = 113.9 * kDeg;
  std::vector<Pose2> path = Line(0.047, 0.329, in, 0.4);
  const Pose2 corner{path.back().x, path.back().y, 0.0};
  path.back().yaw = 0.0;
  Append(path, Line(corner.x, corner.y, 0.0, 2.0), true);
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(path, {}, 0), "");
  Robot robot;
  robot.pose = path.front();
  bool turned_on_corner = false;
  double t = 0;
  while (t < 120 && Running(follower)) {
    const auto cmd = follower.Step(robot.pose, kDt);
    if (follower.OnPath()) robot.max_cross = std::max(robot.max_cross, std::abs(follower.CrossTrack(robot.pose)));
    if (follower.phase() == Phase::kCornerTurn) {
      turned_on_corner = true;
      EXPECT_EQ(cmd.linear, 0.0);
    }
    robot.Move(cmd, kDt);
    t += kDt;
  }
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_TRUE(turned_on_corner);
  EXPECT_LT(robot.max_cross, 0.02);
  EXPECT_NEAR(robot.pose.y, corner.y, 0.01);
}

// F4: pose 59 of the small lawn with turn_radius 0.44 (sim, 29.10.), where the planner turns on the spot.
TEST(PathFollower, TurnsOnTheSpotWhereThePlannerDoes) {
  for (const double degrees : {135.9, -169.4, 16.0}) {
    SCOPED_TRACE(degrees);
    const double turn = degrees * kDeg;
    PathFollower follower{FollowerConfig()};
    ASSERT_EQ(follower.Start(SpinPath(turn), {}, 0), "");
    Robot robot;
    bool turned = false;
    bool after = false;
    Pose2 turned_at;
    double worst_heading = 0;
    double t = 0;
    while (t < 120 && Running(follower)) {
      const auto cmd = follower.Step(robot.pose, kDt);
      if (follower.phase() == Phase::kCornerTurn) {
        if (!turned) turned_at = robot.pose;
        turned = true;
        EXPECT_EQ(cmd.linear, 0.0);
        EXPECT_LE(std::abs(cmd.angular), 0.3 + 1e-9);
      } else if (turned && follower.phase() == Phase::kFollowing) {
        after = true;
      }
      if (follower.OnPath()) robot.max_cross = std::max(robot.max_cross, std::abs(follower.CrossTrack(robot.pose)));
      robot.Move(cmd, kDt);
      if (after) worst_heading = std::max(worst_heading, std::abs(NormalizeAngle(robot.pose.yaw - turn)));
      t += kDt;
    }
    EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
    EXPECT_TRUE(turned);
    // Within 10 cm before the pose of the turn, where the planner checked it, never past it.
    EXPECT_LE(turned_at.x, 1.025);
    EXPECT_GE(turned_at.x, 1.025 - 0.1);
    EXPECT_LT(std::abs(turned_at.y), 0.001);
    // Once turned, the heading stays on the new one. Before: within 25 deg.
    EXPECT_LT(worst_heading, 3.0 * kDeg);
    EXPECT_LT(robot.max_cross, 0.01);
    EXPECT_LT(t, 30.0);
  }
}

// F4
TEST(PathFollower, TurnsTheShorterWay) {
  for (const double degrees : {135.9, -135.9, 179.0, -179.0}) {
    PathFollower follower{FollowerConfig()};
    ASSERT_EQ(follower.Start(SpinPath(degrees * kDeg), {}, 0), "");
    Robot robot;
    double angular = 0;
    double t = 0;
    while (t < 60 && Running(follower)) {
      const auto cmd = follower.Step(robot.pose, kDt);
      if (follower.phase() == Phase::kCornerTurn) angular += cmd.angular * kDt;
      robot.Move(cmd, kDt);
      t += kDt;
    }
    EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
    EXPECT_GT(angular * degrees, 0.0) << degrees;
  }
}

// F4: the way round a corner of the small lawn with turn_radius 0.44: 210 deg on one arc, 10 deg per pose.
TEST(PathFollower, ArcsOfTheTurnRadiusDoNotStop) {
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  Append(path, Arc(path.back(), 0.44, 210.0 * kDeg));
  Append(path, Line(path.back().x, path.back().y, path.back().yaw, 1.0), true);
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(path, {}, 0), "");
  Robot robot;
  bool stopped = false;
  double t = 0;
  while (t < 300 && Running(follower)) {
    const auto cmd = follower.Step(robot.pose, kDt);
    if (follower.phase() != Phase::kFollowing && follower.phase() != Phase::kDone) stopped = true;
    if (follower.OnPath()) robot.max_cross = std::max(robot.max_cross, std::abs(follower.CrossTrack(robot.pose)));
    robot.Move(cmd, kDt);
    t += kDt;
  }
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_FALSE(stopped);
  // Before: under 5 cm.
  EXPECT_LT(robot.max_cross, 0.01);
  EXPECT_LE(robot.max_angular, 0.3 + 1e-9);
}

// F4: the loop of the small lawn with turn_radius 0.44 (sim, 29.10.): a turn on the spot by 76.6 deg, then 290 deg on
// an arc of 0.44 m.
TEST(PathFollower, SpinIntoAnArcOfTheTurnRadius) {
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  const double out = -76.6 * kDeg;
  path.push_back({1.025, 0.0, out});
  Append(path, Arc(path.back(), 0.44, -290.0 * kDeg));
  Append(path, Line(path.back().x, path.back().y, path.back().yaw, 1.0), true);
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(path, {}, 0), "");
  Robot robot;
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  // Before: under 6 cm.
  EXPECT_LT(robot.max_cross, 0.02);
  EXPECT_LE(robot.max_angular, 0.3 + 1e-9);
}

// F5
TEST(PathFollower, NeverDrivesForwardsFacingFarOff) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 3.0), {}, 0), "");
  Robot robot;
  Drive(follower, robot, 1.0);
  robot.pose.yaw = 50.0 * kDeg;
  auto cmd = follower.Step(robot.pose, kDt);
  EXPECT_EQ(follower.phase(), Phase::kRealign);
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_LT(cmd.angular, 0.0);
  robot.Move(cmd, kDt);
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  // 44 deg off: on it drives, steering back.
  PathFollower near{FollowerConfig()};
  ASSERT_EQ(near.Start(Line(0, 0, 0, 3.0), {}, 0), "");
  Robot other;
  Drive(near, other, 1.0);
  other.pose.yaw = 44.0 * kDeg;
  cmd = near.Step(other.pose, kDt);
  EXPECT_EQ(near.phase(), Phase::kFollowing);
  EXPECT_GT(cmd.linear, 0.0);
  EXPECT_LT(cmd.angular, 0.0);
}

// F6
TEST(PathFollower, ArrivesAtAGoalThatTurnsWithTheHeadingBefore) {
  // The last pose turns on the spot: straight onto it, then the turn at the goal.
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  path.push_back({1.025, 0.0, M_PI / 2});
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(path, {}, 0), "");
  Robot robot;
  bool goal_turn = false;
  double t = 0;
  while (t < 60 && Running(follower)) {
    const auto cmd = follower.Step(robot.pose, kDt);
    if (follower.phase() == Phase::kGoalTurn) {
      goal_turn = true;
      EXPECT_EQ(cmd.linear, 0.0);
    }
    robot.Move(cmd, kDt);
    t += kDt;
  }
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_TRUE(goal_turn);
  EXPECT_NEAR(robot.pose.x, 1.025, 0.02);
  EXPECT_LT(std::abs(robot.pose.y), 0.001);
  EXPECT_LT(std::abs(NormalizeAngle(robot.pose.yaw - M_PI / 2)), 2.0 * kDeg + 1e-9);
}

// F6: a mower pushed aside on the last metres ends beside the last pose and fails.
TEST(PathFollower, NotDoneFarBesideTheLastPose) {
  FollowerConfig config;
  config.straight = true;
  PathFollower follower{config};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 1.0), {}, 0), "");
  Robot robot;
  double t = 0;
  while (t < 60 && Running(follower)) {
    robot.Move(follower.Step(robot.pose, kDt), kDt);
    robot.pose.y += 0.004;
    t += kDt;
  }
  EXPECT_EQ(follower.phase(), Phase::kFailed);
  EXPECT_EQ(follower.failure_kind(), Failure::kTimeout);
  EXPECT_NE(follower.failure().find("from its last pose"), std::string::npos) << follower.failure();
}

// F6: done along the last stretch, whatever the mower's distance across it and without turning at the end.
TEST(PathFollower, EndToleranceCountsAlongTheLastStretch) {
  FollowerConfig config;
  config.end_tolerance = 0.02;
  PathFollower follower{config};
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  path.back().yaw = 1.0;
  ASSERT_EQ(follower.Start(path, {}, 0), "");
  Robot robot;
  robot.pose = {0.0, 0.03, 0.0};
  Drive(follower, robot, 60);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_GE(robot.pose.x, 0.98 - 1e-3);
  EXPECT_LT(robot.pose.x, 1.0);
  // Not turned to the heading of the last pose.
  EXPECT_LT(std::abs(robot.pose.yaw), 0.2);
}

// F7: onto a docking station with the contacts at the back: the poses face east, the mower backs west along them from
// 5 cm beside the line and a little turned, steering onto it.
TEST(PathFollower, BackwardsOntoALine) {
  FollowerConfig config;
  config.reverse = true;
  config.speed.max_linear = 0.1;
  PathFollower follower{config};
  std::vector<Pose2> path = Line(0, 0, M_PI, 0.8);
  for (auto& p : path) p.yaw = 0;
  ASSERT_EQ(follower.Start(path, {}, 0), "");
  Robot robot;
  robot.pose = {0.0, 0.05, 0.05};
  Drive(follower, robot, 60);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_EQ(robot.max_forwards, 0.0);
  EXPECT_LE(robot.max_linear, 0.1 + 1e-9);
  EXPECT_LE(robot.max_angular, 0.3 + 1e-9);
  EXPECT_LT(std::hypot(robot.pose.x + 0.8, robot.pose.y), 0.02);
  // Before: under 1 cm beside the line and 2 deg.
  EXPECT_LT(std::abs(robot.pose.y), 0.005);
  EXPECT_LT(std::abs(robot.pose.yaw), 2.0 * kDeg);
}

// F7, F4: backwards round a corner: turning on the spot, then on backwards, never forwards.
TEST(PathFollower, BackwardsRoundACorner) {
  FollowerConfig config;
  config.reverse = true;
  PathFollower follower{config};
  auto path = SpinPath(-90.0 * kDeg);
  for (auto& p : path) p.yaw = NormalizeAngle(p.yaw + M_PI);
  ASSERT_EQ(follower.Start(path, {}, 0), "");
  Robot robot;
  robot.pose = path.front();
  Drive(follower, robot, 120);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_EQ(robot.max_forwards, 0.0);
  EXPECT_LT(robot.max_cross, 0.01);
  EXPECT_NEAR(robot.pose.y, -1.5, 0.02);
}

// F6, F7, F9: the last 0.25 m onto the station: the line reaches 0.05 m past where the station stops the mower. With
// end_tolerance the path is never done there; the mower pushes until it counts as stuck.
TEST(PathFollower, BackwardsAgainstAStopIsNeverDone) {
  FollowerConfig config;
  config.reverse = true;
  config.speed.max_linear = 0.06;
  config.stall_time = 10.0;
  config.end_tolerance = 0.005;
  PathFollower follower{config};
  std::vector<Pose2> path = Line(-0.55, 0, M_PI, 0.3);
  for (auto& p : path) p.yaw = 0;
  ASSERT_EQ(follower.Start(path, {}, 0), "");
  Robot robot;
  robot.pose = {-0.55, 0.0, 0.0};
  double t = 0;
  double stopped_at = -1;
  while (t < 60 && Running(follower)) {
    robot.Move(follower.Step(robot.pose, kDt), kDt);
    // The station: base_link gets no further west than -0.8.
    if (robot.pose.x < -0.8) {
      robot.pose.x = -0.8;
      if (stopped_at < 0) stopped_at = t;
    }
    t += kDt;
  }
  EXPECT_EQ(follower.phase(), Phase::kFailed);
  EXPECT_NE(follower.failure().find("does not move"), std::string::npos) << follower.failure();
  ASSERT_GT(stopped_at, 0);
  // Counted from the last 5 cm it moved: up to 0.8 s before the stop at 0.06 m/s.
  EXPECT_GT(t - stopped_at, 9.0);
  EXPECT_LT(t - stopped_at, 10.5);
  EXPECT_LE(robot.max_linear, 0.06 + 1e-9);

  // Without the station it gets to the end and is done there, 5 mm before the last pose.
  ASSERT_EQ(follower.Start(path, {}, 0), "");
  Robot free;
  free.pose = {-0.55, 0.0, 0.0};
  Drive(follower, free, 60);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_LT(free.pose.x, -0.845 + 1e-3);
}

// F8: out of the docking station straight ahead without steering, at most the speed of the goal. A mower facing a
// little off the line keeps its heading and ends beside the line.
TEST(PathFollower, StraightDrivesOnWithoutSteering) {
  FollowerConfig config;
  config.straight = true;
  config.speed.max_linear = 0.15;
  PathFollower follower{config};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 0.8), {}, 0), "");
  Robot robot;
  robot.pose = {0, 0, 0.1};
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_EQ(robot.max_angular, 0.0);
  EXPECT_LE(robot.max_linear, 0.15 + 1e-9);
  EXPECT_NEAR(robot.pose.yaw, 0.1, 1e-12);
  // Done once along the line at its end, 0.08 m beside it.
  EXPECT_GE(robot.pose.x, 0.79 - 1e-9);
  EXPECT_LT(robot.pose.x, 0.8);
  EXPECT_NEAR(robot.pose.y, robot.pose.x * std::tan(0.1), 0.01);
}

// F9: wheels that cannot climb out of the hollow of the station: driven at the speed of the goal, the mower stays
// where it is.
TEST(PathFollower, StandingStillWhileDrivenEndsThePath) {
  FollowerConfig config;
  config.straight = true;
  config.speed.max_linear = 0.15;
  config.stall_time = 2.0;
  PathFollower follower{config};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 0.8), {}, 0), "");
  const Pose2 stuck;
  double t = 0;
  double max_linear = 0;
  while (t < 10 && follower.phase() != Phase::kFailed) {
    max_linear = std::max(max_linear, follower.Step(stuck, kDt).linear);
    t += kDt;
  }
  EXPECT_EQ(follower.phase(), Phase::kFailed);
  EXPECT_EQ(follower.failure_kind(), Failure::kTimeout);
  EXPECT_NE(follower.failure().find("does not move"), std::string::npos) << follower.failure();
  EXPECT_GT(t, 2.0);
  EXPECT_LT(t, 2.2);
  // It never pushed harder than the speed of the goal.
  EXPECT_NEAR(max_linear, 0.15, 1e-9);
  EXPECT_EQ(follower.Step(stuck, kDt).linear, 0.0);
}

// F9: turning on the spot does not count as standing.
TEST(PathFollower, StandingStillCountsOnlyWhileDriven) {
  FollowerConfig config;
  config.stall_time = 1.5;
  PathFollower follower{config};
  // A quarter turn on the spot first: about 6 s without moving, then the drive from standstill.
  ASSERT_EQ(follower.Start(Line(0, 0, M_PI / 2, 2.0), {}, 0), "");
  Robot robot;
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_NEAR(robot.pose.y, 2.0, 0.02);
}

// F9: without a stall time, no progress along the path ends it after progress_timeout.
TEST(PathFollower, NoProgressEndsThePath) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 10.0), {}, 0), "");
  const Pose2 stuck;
  double t = 0;
  while (t < 60 && follower.phase() != Phase::kFailed) {
    follower.Step(stuck, kDt);
    t += kDt;
  }
  EXPECT_EQ(follower.phase(), Phase::kFailed);
  EXPECT_EQ(follower.failure_kind(), Failure::kTimeout);
  EXPECT_NE(follower.failure().find("no progress"), std::string::npos) << follower.failure();
  EXPECT_GT(t, 9.9);
  EXPECT_LT(t, 10.2);
  EXPECT_EQ(follower.Step(stuck, kDt).linear, 0.0);
}

// F9: driving in circles beside the start is no progress either.
TEST(PathFollower, MovingWithoutProgressEndsThePath) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 10.0), {}, 0), "");
  double t = 0;
  while (t < 60 && follower.phase() != Phase::kFailed) {
    // Something carries the mower back and forth across the line.
    follower.Step({0.02, 0.1 * std::sin(t), 0.0}, kDt);
    t += kDt;
  }
  EXPECT_NE(follower.failure().find("no progress"), std::string::npos) << follower.failure();
}

// F10: a mower that does not turn when told to.
TEST(PathFollower, TurningTakesAtMostThePhaseTimeout) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, M_PI / 2, 2.0), {}, 0), "");
  const Pose2 stuck;
  double t = 0;
  while (t < 60 && follower.phase() != Phase::kFailed) {
    follower.Step(stuck, kDt);
    t += kDt;
  }
  EXPECT_EQ(follower.failure_kind(), Failure::kTimeout);
  EXPECT_NE(follower.failure().find("turning on the spot took longer than 20 s"), std::string::npos)
      << follower.failure();
  EXPECT_GT(t, 20.0);
  EXPECT_LT(t, 20.2);
}

// F10: turning on the spot between min_turn_rate and max_angular, to within 2 deg.
TEST(PathFollower, TurnRatesOnTheSpot) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, M_PI / 2, 2.0), {}, 0), "");
  Robot robot;
  double slowest = 1;
  auto cmd = follower.Step(robot.pose, kDt);
  while (follower.phase() == Phase::kStartTurn) {
    slowest = std::min(slowest, std::abs(cmd.angular));
    EXPECT_LE(std::abs(cmd.angular), 0.3 + 1e-9);
    robot.Move(cmd, kDt);
    cmd = follower.Step(robot.pose, kDt);
  }
  EXPECT_GE(slowest, 0.28 - 1e-9);
  EXPECT_LT(std::abs(NormalizeAngle(robot.pose.yaw - M_PI / 2)), 2.0 * kDeg);
}

// F11
TEST(PathFollower, CancelStopsAtOnce) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 10.0), {}, 0), "");
  Robot robot;
  for (int i = 0; i < 100; i++) robot.Move(follower.Step(robot.pose, kDt), kDt);
  EXPECT_GT(robot.last_linear, 0.0);
  follower.Cancel();
  EXPECT_EQ(follower.phase(), Phase::kIdle);
  const auto cmd = follower.Step(robot.pose, kDt);
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_EQ(cmd.angular, 0.0);
}

// F12
TEST(PathFollower, CrossTrackIsSignedLeftPositive) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 2.0), {}, 0), "");
  EXPECT_NEAR(follower.CrossTrack({0.2, 0.3, 0}), 0.3, 1e-9);
  EXPECT_NEAR(follower.CrossTrack({0.2, -0.4, 0}), -0.4, 1e-9);
}

// F12: a new path from the index of a canceled one goes on where it stopped.
TEST(PathFollower, GoesOnFromTheIndexItGotTo) {
  const auto line = Line(0, 0, 0, 5.0);
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(line, {}, 0), "");
  Robot robot;
  Drive(follower, robot, 12.0);
  follower.Cancel();
  const size_t index = follower.index();
  EXPECT_GT(index, 10u);
  EXPECT_LE(line[index].x, robot.pose.x);
  // A step behind the last move.
  EXPECT_GT(line[index].x, robot.pose.x - 0.1 - 0.3 * kDt);
  ASSERT_EQ(follower.Start(line, {}, index), "");
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_NEAR(robot.pose.x, 5.0, 0.02);
}

TEST(PathFollower, GoesOnFromTheStartIndex) {
  PathFollower follower{FollowerConfig()};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 5.0), {}, 30), "");
  Robot robot;
  robot.pose = {3.0, 0.0, 0.0};
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_NEAR(robot.pose.x, 5.0, 0.02);
}

TEST(PathFollower, NeverExceedsTheLimitsEvenFarOff) {
  FollowerConfig config;
  PathFollower follower{config};
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 6.0), {}, 0), "");
  Robot robot;
  robot.pose = {0.0, 0.6, -0.5};
  Drive(follower, robot, 120);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_LE(robot.max_linear, config.speed.max_linear + 1e-9);
  EXPECT_LE(robot.max_angular, config.max_angular + 1e-9);
}

// The paths above on the wheels of a real Tron: a dead band of 0.05 m/s per wheel and a track 1 % off.
TEST(PathFollower, WithTheDeadBandOfTheWheels) {
  std::vector<Kind> kinds;
  const auto uturn = UTurn(&kinds);
  std::vector<Pose2> loop = Line(0, 0, 0, 1.0);
  loop.push_back({1.025, 0.0, -76.6 * kDeg});
  Append(loop, Arc(loop.back(), 0.44, -290.0 * kDeg));
  Append(loop, Line(loop.back().x, loop.back().y, loop.back().yaw, 1.0), true);
  struct Case {
    const char* name;
    std::vector<Pose2> poses;
    std::vector<Kind> kinds;
    double limit;
  };
  // The loop of a bulb turn of the planner (full run, 01.11.): 0.13 m, far tighter than the inner wheel allows.
  std::vector<Pose2> bulb = Line(0, 0, 0, 1.0);
  Append(bulb, Arc(bulb.back(), 0.13, -M_PI));
  Append(bulb, Line(bulb.back().x, bulb.back().y, bulb.back().yaw, 1.0), true);
  for (const Case& c : {Case{"u-turn", uturn, kinds, 0.03}, Case{"loop", loop, {}, 0.03},
                        Case{"corner", SpinPath(135.9 * kDeg), {}, 0.02}, Case{"bulb", bulb, {}, 0.015}}) {
    SCOPED_TRACE(c.name);
    PathFollower follower{FollowerConfig()};
    ASSERT_EQ(follower.Start(c.poses, c.kinds, 0), "");
    Robot robot;
    robot.faults = true;
    Drive(follower, robot);
    EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
    EXPECT_LT(robot.max_cross, c.limit);
    EXPECT_LT(std::hypot(robot.pose.x - c.poses.back().x, robot.pose.y - c.poses.back().y), 0.03);
  }
}

TEST(PlannerSpeeds, UnderTheDriversLimits) {
  FollowerConfig config;
  config.speed.speed_fast = 0.3;
  config.speed.speed_slow = 0.15;
  config.speed.max_linear = 0.3;
  config.max_angular = 0.3;
  // Without capabilities the follower's own values.
  auto speeds = mowbite_follower::Effective(config, 1e9, 1e9);
  EXPECT_EQ(speeds.speed_fast, 0.3);
  EXPECT_EQ(speeds.speed_slow, 0.15);
  EXPECT_EQ(speeds.max_angular, 0.3);
  // The NX60i at max_duty 0.3: 0.163 m/s cuts speed_fast only.
  speeds = mowbite_follower::Effective(config, 0.16298, 0.3);
  EXPECT_EQ(speeds.speed_fast, 0.16298);
  EXPECT_EQ(speeds.speed_slow, 0.15);
  EXPECT_EQ(speeds.max_angular, 0.3);
  speeds = mowbite_follower::Effective(config, 0.1, 0.2);
  EXPECT_EQ(speeds.speed_fast, 0.1);
  EXPECT_EQ(speeds.speed_slow, 0.1);
  EXPECT_EQ(speeds.max_angular, 0.2);
  // A limit of the follower below the driver's holds; a limit that is no number above 0 counts as none.
  config.speed.max_linear = 0.12;
  speeds = mowbite_follower::Effective(config, std::nan(""), 0.0);
  EXPECT_EQ(speeds.speed_fast, 0.12);
  EXPECT_EQ(speeds.speed_slow, 0.12);
  EXPECT_EQ(speeds.max_angular, 0.3);
  EXPECT_EQ(mowbite_follower::ParamsJson("/mowbite_follower", {0.16298, 0.15, 0.3}),
            "{\"node\":\"/mowbite_follower\",\"params\":{\"speed_fast\":0.16298,\"speed_slow\":0.15,"
            "\"max_angular\":0.3}}");
}

// The distance for a goal with a limit of its own: to the whole path, before it started and past both ends.
TEST(PathFollower, OffPathIsTheDistanceToTheWholePath) {
  PathFollower follower{FollowerConfig()};
  EXPECT_EQ(follower.OffPath({3, 3, 0}), 0.0);
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 2.0), {}, 0), "");
  EXPECT_FALSE(follower.OnPath());
  EXPECT_NEAR(follower.OffPath({0.5, 0.2, 0}), 0.2, 1e-9);
  EXPECT_NEAR(follower.OffPath({0.5, -0.2, 1.0}), 0.2, 1e-9);
  EXPECT_NEAR(follower.OffPath({-0.3, 0.4, 0}), 0.5, 1e-9);
  EXPECT_NEAR(follower.OffPath({2.3, -0.4, 0}), 0.5, 1e-9);
  // Not only the part ahead: started in the middle, the path behind still counts.
  ASSERT_EQ(follower.Start(Line(0, 0, 0, 2.0), {}, 10), "");
  EXPECT_NEAR(follower.OffPath({0.2, 0.1, 0}), 0.1, 1e-9);
}

// Turning on the spot up to spot_max_angular, slowing down before the target by turn_deceleration; driving on the
// path stays within max_angular.
TEST(PathFollower, TurnsOnTheSpotFasterAndSlowsDownBeforeTheTarget) {
  FollowerConfig config;
  config.max_angular = 0.5;
  config.min_turn_rate = 0.45;
  config.spot_max_angular = 0.7;
  config.turn_deceleration = 1.0;
  PathFollower follower{config};
  ASSERT_EQ(follower.Start(Line(0, 0, M_PI - 0.01, 2.0), {}, 0), "");
  Robot robot;
  double t = 0;
  double fastest = 0;
  while (t < 20) {
    const auto cmd = follower.Step(robot.pose, kDt);
    if (follower.phase() != Phase::kStartTurn) break;
    const double error = std::abs(NormalizeAngle(M_PI - 0.01 - robot.pose.yaw));
    EXPECT_EQ(cmd.linear, 0.0);
    EXPECT_NEAR(std::abs(cmd.angular), std::clamp(std::sqrt(2.0 * error), 0.45, 0.7), 1e-9);
    fastest = std::max(fastest, std::abs(cmd.angular));
    robot.Move(cmd, kDt);
    t += kDt;
  }
  EXPECT_DOUBLE_EQ(fastest, 0.7);
  // 179 degrees at 0.7 rad/s and slowing down over the last 14 degrees.
  EXPECT_LT(t, 5.0);
  robot.max_angular = 0;
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  EXPECT_LE(robot.max_angular, 0.5);
}

FollowerConfig SpotConfig(double feedback) {
  FollowerConfig config;
  config.max_angular = 0.5;
  config.min_turn_rate = 0.45;
  config.spot_max_angular = 0.7;
  config.turn_deceleration = 1.0;
  config.turn_feedback = feedback;
  config.turn_feedback_max = 0.2;
  // Small turns on the spot too.
  config.corner_angle = 3.0 * M_PI / 180.0;
  return config;
}

struct SpotTurn {
  double seconds = 0;
  double fastest = 0;
  int reversals = 0;
};

// Turns on the spot by angle (rad) at the start of a line; the base makes turn_share of the commanded turn.
SpotTurn TurnOnTheSpot(const FollowerConfig& config, double angle, double turn_share) {
  PathFollower follower{config};
  EXPECT_EQ(follower.Start(Line(0, 0, angle, 2.0), {}, 0), "");
  Robot robot;
  robot.turn_share = turn_share;
  SpotTurn turn;
  double last = 0;
  while (turn.seconds < 20) {
    const auto cmd = follower.Step(robot.pose, kDt);
    if (follower.phase() != Phase::kStartTurn) break;
    EXPECT_EQ(cmd.linear, 0.0);
    if (last * cmd.angular < 0) turn.reversals++;
    last = cmd.angular;
    turn.fastest = std::max(turn.fastest, std::abs(cmd.angular));
    robot.Move(cmd, kDt);
    turn.seconds += kDt;
  }
  EXPECT_EQ(follower.phase(), Phase::kFollowing) << follower.failure();
  EXPECT_LE(std::abs(NormalizeAngle(angle - robot.pose.yaw)), config.turn_tolerance + 1e-9);
  return turn;
}

// With turn_feedback a turn the heading follows slower than commanded gets more, within its limits.
TEST(PathFollower, ASlippingTurnOnTheSpotGetsMoreCommand) {
  const double angle = 10.0 * M_PI / 180.0;
  const SpotTurn plain = TurnOnTheSpot(SpotConfig(0.0), angle, 0.6);
  const SpotTurn fed = TurnOnTheSpot(SpotConfig(1.0), angle, 0.6);
  EXPECT_DOUBLE_EQ(plain.fastest, std::sqrt(2.0 * angle));
  EXPECT_GT(fed.fastest, plain.fastest + 0.05);
  EXPECT_LE(fed.fastest, std::sqrt(2.0 * angle) + 0.2 + 1e-9);
  EXPECT_LT(fed.seconds, plain.seconds);
  EXPECT_EQ(fed.reversals, 0);
}

TEST(PathFollower, AFollowingTurnOnTheSpotGetsNoMore) {
  for (double degrees : {5.0, 10.0, 90.0, 179.0}) {
    const double angle = degrees * M_PI / 180.0;
    const SpotTurn plain = TurnOnTheSpot(SpotConfig(0.0), angle, 1.0);
    const SpotTurn fed = TurnOnTheSpot(SpotConfig(1.0), angle, 1.0);
    EXPECT_NEAR(fed.fastest, plain.fastest, 1e-9) << degrees;
    EXPECT_DOUBLE_EQ(fed.seconds, plain.seconds) << degrees;
    EXPECT_EQ(fed.reversals, 0) << degrees;
  }
}

TEST(PathFollower, TheFeedbackStaysWithinTheLimitOfTheTurn) {
  // Hardly turning at all: the full addition, but never above spot_max_angular.
  const SpotTurn far = TurnOnTheSpot(SpotConfig(1.0), 90.0 * M_PI / 180.0, 0.3);
  EXPECT_DOUBLE_EQ(far.fastest, 0.7);
  const SpotTurn near = TurnOnTheSpot(SpotConfig(1.0), 5.0 * M_PI / 180.0, 0.3);
  EXPECT_NEAR(near.fastest, 0.45 + 0.2, 1e-9);
  EXPECT_EQ(near.reversals, 0);
}

TEST(PathFollower, WithoutSpotValuesTurnsAsBefore) {
  FollowerConfig config;
  config.max_angular = 0.5;
  config.min_turn_rate = 0.3;
  PathFollower follower{config};
  ASSERT_EQ(follower.Start(Line(0, 0, 0.5, 2.0), {}, 0), "");
  const auto cmd = follower.Step(Pose2{0, 0, 0}, kDt);
  ASSERT_EQ(follower.phase(), Phase::kStartTurn);
  // heading_gain 2 * 0.5 rad, at most max_angular.
  EXPECT_NEAR(cmd.angular, 0.5, 1e-9);
  const auto near = follower.Step(Pose2{0, 0, 0.4}, kDt);
  EXPECT_NEAR(near.angular, 0.3, 1e-9);
}

FollowerConfig HoldConfig(double gain) {
  FollowerConfig config = SpotConfig(0.0);
  config.turn_hold_gain = gain;
  config.wheel_min_speed = 0.06;
  config.wheel_distance = 0.4;
  return config;
}

// Turns on the spot by angle (rad) at the start of a line with the robot as it is; the follower is left in the phase
// after the turn.
void TurnAtTheStart(PathFollower& follower, Robot& robot, double angle) {
  ASSERT_EQ(follower.Start(Line(0, 0, angle, 2.0), {}, 0), "");
  for (double t = 0; t < 20; t += kDt) {
    const auto cmd = follower.Step(robot.pose, kDt, robot.wheel_turn);
    if (follower.phase() != Phase::kStartTurn) break;
    robot.Move(cmd, kDt);
  }
}

// A wheel that slips more than the other moves base_link while it turns on the spot; with turn_hold_gain the
// follower drives against that and no wheel gets slower than wheel_min_speed.
TEST(PathFollower, HoldsThePlaceOfATurnWithWheelsThatSlipUnevenly) {
  for (double degrees : {90.0, -90.0, 179.0}) {
    const double angle = degrees * M_PI / 180.0;
    Robot loose;
    loose.slip_left = 0.5;
    PathFollower plain{HoldConfig(0.0)};
    TurnAtTheStart(plain, loose, angle);
    EXPECT_EQ(plain.phase(), Phase::kFollowing) << plain.failure();
    EXPECT_GT(loose.farthest, 0.07) << degrees;

    Robot held;
    held.slip_left = 0.5;
    PathFollower holding{HoldConfig(2.0)};
    TurnAtTheStart(holding, held, angle);
    EXPECT_EQ(holding.phase(), Phase::kFollowing) << holding.failure();
    EXPECT_LT(held.farthest, 0.035) << degrees;
    EXPECT_GE(held.slowest_wheel, 0.06 - 1e-9) << degrees;
    EXPECT_GT(holding.last_turn().hold, 0.02) << degrees;
  }
}

// A mower that stays on its place gets no drive, whatever the gain; without the track of the wheels neither.
TEST(PathFollower, ATurnThatStaysOnItsPlaceGetsNoDrive) {
  Robot robot;
  PathFollower holding{HoldConfig(2.0)};
  TurnAtTheStart(holding, robot, 2.0);
  EXPECT_EQ(robot.max_linear, 0.0);
  Robot slipping;
  slipping.slip_left = 0.5;
  FollowerConfig config = HoldConfig(2.0);
  config.wheel_distance = 0;
  PathFollower blind{config};
  TurnAtTheStart(blind, slipping, 2.0);
  EXPECT_EQ(slipping.max_linear, 0.0);
}

// Below wheel_min_speed per wheel there is no room to drive: the turn stays a turn.
TEST(PathFollower, HoldsOnlyAsFarAsTheSlowerWheelKeepsItsSpeed) {
  Robot robot;
  robot.slip_left = 0.5;
  FollowerConfig config = HoldConfig(2.0);
  // 0.7 rad/s on 0.4 m: 0.14 m/s per wheel.
  config.wheel_min_speed = 0.14;
  PathFollower follower{config};
  TurnAtTheStart(follower, robot, 2.0);
  EXPECT_EQ(robot.max_linear, 0.0);
  config.wheel_min_speed = 0.13;
  Robot tight;
  tight.slip_left = 0.5;
  PathFollower narrow{config};
  TurnAtTheStart(narrow, tight, 2.0);
  EXPECT_GT(tight.max_linear, 0.0);
  EXPECT_LE(tight.max_linear, 0.01 + 1e-9);
  EXPECT_GE(tight.slowest_wheel, 0.13 - 1e-9);
}

// Every turn on the spot leaves a record: the angles by command, wheels and heading, and how far base_link got.
TEST(PathFollower, RecordsWhatATurnOnTheSpotDid) {
  Robot robot;
  robot.slip_left = 0.4;
  robot.slip_right = 0.2;
  PathFollower follower{HoldConfig(0.0)};
  EXPECT_EQ(follower.turns(), 0u);
  TurnAtTheStart(follower, robot, M_PI / 2);
  ASSERT_EQ(follower.turns(), 1u);
  const auto& turn = follower.last_turn();
  EXPECT_EQ(turn.phase, Phase::kStartTurn);
  EXPECT_GT(turn.seconds, 2.0);
  EXPECT_NEAR(turn.turned, robot.pose.yaw, 1e-9);
  EXPECT_GT(turn.turned, 1.4);
  EXPECT_NEAR(turn.wheels, turn.commanded, 1e-9);
  // Both wheels together lose 0.3 of the turn.
  EXPECT_NEAR(turn.turned / turn.wheels, 0.7, 0.01);
  EXPECT_NEAR(turn.slide, robot.farthest, 0.005);
  EXPECT_GT(turn.slide, 0.02);
  EXPECT_EQ(turn.hold, 0.0);
  // Without the odometry of the wheels their angle is no number.
  Robot blind;
  PathFollower other{HoldConfig(0.0)};
  ASSERT_EQ(other.Start(Line(0, 0, 1.0, 2.0), {}, 0), "");
  for (double t = 0; t < 20 && other.turns() == 0; t += kDt) blind.Move(other.Step(blind.pose, kDt), kDt);
  ASSERT_EQ(other.turns(), 1u);
  EXPECT_TRUE(std::isnan(other.last_turn().wheels));
  EXPECT_NEAR(other.last_turn().turned, 1.0, 0.04);
}

// A turn that ends by its timeout or with the path is recorded too.
TEST(PathFollower, RecordsATurnThatDoesNotEnd) {
  FollowerConfig config = HoldConfig(0.0);
  config.phase_timeout = 2.0;
  PathFollower follower{config};
  ASSERT_EQ(follower.Start(Line(0, 0, 3.0, 2.0), {}, 0), "");
  Robot robot;
  robot.turn_share = 0.0;
  Drive(follower, robot);
  ASSERT_EQ(follower.phase(), Phase::kFailed);
  ASSERT_EQ(follower.turns(), 1u);
  EXPECT_NEAR(follower.last_turn().seconds, 2.0, 0.1);
  EXPECT_NEAR(follower.last_turn().turned, 0.0, 1e-9);
  EXPECT_GT(follower.last_turn().commanded, 1.0);
}

// Without a fresh fix behind the pose the mower does not turn on the spot: it stands, which is no part of the time a
// turn may take, and goes on when the fix is back.
TEST(PathFollower, DoesNotTurnOnTheSpotWithoutAFreshFix) {
  FollowerConfig config = HoldConfig(2.0);
  config.phase_timeout = 5.0;
  PathFollower follower{config};
  ASSERT_EQ(follower.Start(Line(0, 0, 2.0, 2.0), {}, 0), "");
  Robot robot;
  for (double t = 0; t < 1.0; t += kDt) robot.Move(follower.Step(robot.pose, kDt), kDt);
  ASSERT_EQ(follower.phase(), Phase::kStartTurn);
  const double turned = robot.pose.yaw;
  EXPECT_GT(turned, 0.3);
  for (double t = 0; t < 30.0; t += kDt) {
    const auto cmd = follower.Step(robot.pose, kDt, robot.wheel_turn, false);
    EXPECT_EQ(cmd.linear, 0.0);
    EXPECT_EQ(cmd.angular, 0.0);
    robot.Move(cmd, kDt);
  }
  EXPECT_EQ(follower.phase(), Phase::kStartTurn) << follower.failure();
  EXPECT_DOUBLE_EQ(robot.pose.yaw, turned);
  Drive(follower, robot);
  EXPECT_EQ(follower.phase(), Phase::kDone) << follower.failure();
  // Driving along the path needs no fresh fix of its own: the accuracy of the pose guards that.
  PathFollower straight{config};
  ASSERT_EQ(straight.Start(Line(0, 0, 0, 2.0), {}, 0), "");
  Robot other;
  const auto cmd = straight.Step(other.pose, kDt, 0.0, false);
  EXPECT_GT(cmd.linear, 0.0);
}

// The way from the place of a turn on the spot counts while it turns, not before and not after.
TEST(PathFollower, TellsHowFarATurnOnTheSpotSlid) {
  PathFollower follower{HoldConfig(0.0)};
  ASSERT_EQ(follower.Start(Line(0, 0, 2.0, 2.0), {}, 0), "");
  Robot robot;
  EXPECT_EQ(follower.TurnSlide(robot.pose), 0.0);
  robot.slide_per_rad = 0.1;
  double farthest = 0;
  for (double t = 0; t < 20; t += kDt) {
    const auto cmd = follower.Step(robot.pose, kDt);
    if (follower.phase() != Phase::kStartTurn) break;
    robot.Move(cmd, kDt);
    farthest = std::max(farthest, follower.TurnSlide(robot.pose));
    EXPECT_NEAR(follower.TurnSlide(robot.pose), robot.pose.y, 1e-9);
  }
  // 0.1 m per rad over the turn.
  EXPECT_GT(farthest, 0.15);
  ASSERT_EQ(follower.phase(), Phase::kFollowing);
  EXPECT_EQ(follower.TurnSlide(robot.pose), 0.0);
}
