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

// An arc of radius r (left turn for angle > 0) from pose p, poses every spacing m and at most every 10 deg like the
// planner's.
std::vector<Pose2> Arc(const Pose2& p, double r, double angle, double spacing = 0.1) {
  std::vector<Pose2> poses;
  const double sign = angle > 0 ? 1.0 : -1.0;
  const double cx = p.x - sign * r * std::sin(p.yaw);
  const double cy = p.y + sign * r * std::cos(p.yaw);
  const int n = std::max({2, static_cast<int>(std::ceil(std::abs(angle) * r / spacing)),
                          static_cast<int>(std::ceil(std::abs(angle) / (10.0 * M_PI / 180.0) - 1e-9))});
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
    if (tracker.OnPath()) {
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
    if (tracker.OnPath()) {
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

// The way into the lanes of a small lawn (sim, 28.10.): a transit heading 114 deg, then a lane heading 0 deg from the
// pose where it ends, which the mower leaves with 0 deg. Before, it cut this corner on a wide arc and stopped 0.5 m off
// the lane.
TEST(PathTracker, StopsOnACornerAndTurnsOnTheSpot) {
  const double in = 113.9 * M_PI / 180.0;
  std::vector<Pose2> path = Line(0.047, 0.329, in, 0.4);
  const Pose2 corner{path.back().x, path.back().y, 0.0};
  path.back().yaw = 0.0;
  Append(path, Line(corner.x, corner.y, 0.0, 2.0), true);
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(path, {}, 0), "");
  Robot robot;
  robot.pose = path.front();
  bool turned_on_corner = false;
  double t = 0;
  while (t < 120 && tracker.phase() != Phase::kDone && tracker.phase() != Phase::kFailed) {
    const auto cmd = tracker.Step(robot.pose, kDt);
    if (tracker.OnPath()) robot.max_cross = std::max(robot.max_cross, std::abs(tracker.CrossTrack(robot.pose)));
    if (tracker.phase() == Phase::kCornerRotate) {
      turned_on_corner = true;
      EXPECT_EQ(cmd.linear, 0.0);
    }
    robot.Move(cmd, kDt);
    t += kDt;
  }
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_TRUE(turned_on_corner);
  EXPECT_LT(robot.max_cross, 0.1);
  EXPECT_NEAR(robot.pose.y, corner.y, 0.05);
}

TEST(PathTracker, NeverDrivesForwardsFacingFarOff) {
  // The control point behind on the left: turning first, not an arc forwards.
  PathTracker tracker{TrackerConfig()};
  const auto line = Line(0, 0, 0, 3.0);
  ASSERT_EQ(tracker.Start(line, {}, 0), "");
  Robot robot;
  robot.pose = {0.0, 0.0, 0.0};
  Drive(tracker, robot, 1.0);
  robot.pose.yaw = 80.0 * M_PI / 180.0;
  const auto cmd = tracker.Step(robot.pose, kDt);
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_LT(cmd.angular, 0.0);
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

namespace {

// What the planner does where no arc fits: straight on, a pose 2.5 cm before the turn with the heading before, the pose
// of the turn with the new heading, then straight on from there.
std::vector<Pose2> SpinPath(double turn) {
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  path.push_back({1.025, 0.0, NormalizeAngle(turn)});
  Append(path, Line(1.025, 0.0, NormalizeAngle(turn), 1.5), true);
  return path;
}

}  // namespace

// Pose 59 of the small lawn with turn_radius 0.44 (sim, 29.10.): the mower swung back to the old heading after turning,
// then drove across the lanes and stopped 0.51 m off the path.
TEST(PathTracker, TurnsOnTheSpotWhereThePlannerDoes) {
  for (const double degrees : {135.9, -169.4, 16.0}) {
    SCOPED_TRACE(degrees);
    const double turn = degrees * M_PI / 180.0;
    PathTracker tracker{TrackerConfig()};
    ASSERT_EQ(tracker.Start(SpinPath(turn), {}, 0), "");
    Robot robot;
    bool turned = false;
    bool after = false;
    Pose2 turned_at;
    double after_time = 0;
    double control_first_second = 0;
    double worst_heading = 0;
    double t = 0;
    while (t < 120 && tracker.phase() != Phase::kDone && tracker.phase() != Phase::kFailed) {
      const auto cmd = tracker.Step(robot.pose, kDt);
      if (tracker.phase() == Phase::kCornerRotate) {
        if (!turned) turned_at = robot.pose;
        turned = true;
        EXPECT_EQ(cmd.linear, 0.0);
        EXPECT_LE(std::abs(cmd.angular), 0.3 + 1e-9);
      } else if (turned && tracker.phase() == Phase::kFollowing) {
        after = true;
      }
      if (tracker.OnPath()) robot.max_cross = std::max(robot.max_cross, std::abs(tracker.CrossTrack(robot.pose)));
      robot.Move(cmd, kDt);
      if (after) {
        // Once turned, the heading only comes closer to the new one, give or take what the PID needs for the line.
        worst_heading = std::max(worst_heading, std::abs(NormalizeAngle(robot.pose.yaw - turn)));
        if (after_time < 1.0) {
          const Pose2 control = tracker.ControlPoint();
          control_first_second = std::hypot(control.x - 1.025, control.y);
        }
        after_time += kDt;
      }
      t += kDt;
    }
    EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
    EXPECT_TRUE(turned);
    // Within 10 cm before the pose of the turn, where the planner checked it, never past it.
    EXPECT_LE(turned_at.x, 1.025 + 0.01);
    EXPECT_GE(turned_at.x, 1.025 - 0.1);
    EXPECT_LT(std::abs(turned_at.y), 0.02);
    EXPECT_LT(worst_heading, 25.0 * M_PI / 180.0);
    EXPECT_LT(robot.max_cross, 0.1);
    // From standstill: 0.1 m/s² takes the control point 5 cm from the pose of the turn in the first second.
    EXPECT_LT(control_first_second, 0.07);
    EXPECT_LT(t, 60.0);
  }
}

TEST(PathTracker, TurnsTheShorterWay) {
  for (const double degrees : {135.9, -135.9}) {
    PathTracker tracker{TrackerConfig()};
    ASSERT_EQ(tracker.Start(SpinPath(degrees * M_PI / 180.0), {}, 0), "");
    Robot robot;
    double angular = 0;
    double t = 0;
    while (t < 60 && tracker.phase() != Phase::kDone && tracker.phase() != Phase::kFailed) {
      const auto cmd = tracker.Step(robot.pose, kDt);
      if (tracker.phase() == Phase::kCornerRotate) angular += cmd.angular * kDt;
      robot.Move(cmd, kDt);
      t += kDt;
    }
    EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
    EXPECT_GT(angular * degrees, 0.0) << degrees;
  }
}

TEST(PathTracker, ArcsOfTheTurnRadiusDoNotStop) {
  // The way round a corner of the small lawn with turn_radius 0.44: 210 deg on one arc, 10 deg per pose.
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  Append(path, Arc(path.back(), 0.44, 210.0 * M_PI / 180.0));
  Append(path, Line(path.back().x, path.back().y, path.back().yaw, 1.0), true);
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(path, {}, 0), "");
  Robot robot;
  bool stopped = false;
  double t = 0;
  while (t < 300 && tracker.phase() != Phase::kDone && tracker.phase() != Phase::kFailed) {
    const auto cmd = tracker.Step(robot.pose, kDt);
    if (tracker.phase() == Phase::kCornerApproach || tracker.phase() == Phase::kCornerRotate) stopped = true;
    if (tracker.OnPath()) robot.max_cross = std::max(robot.max_cross, std::abs(tracker.CrossTrack(robot.pose)));
    robot.Move(cmd, kDt);
    t += kDt;
  }
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_FALSE(stopped);
  EXPECT_LT(robot.max_cross, 0.05);
  EXPECT_LE(robot.max_angular, 0.3 + 1e-9);
}

TEST(PathTracker, ArrivesAtAGoalThatTurnsWithTheHeadingBefore) {
  // The last pose turns on the spot: straight onto it, then the turn at the goal.
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  path.push_back({1.025, 0.0, M_PI / 2});
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(path, {}, 0), "");
  Robot robot;
  Drive(tracker, robot);
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_NEAR(robot.pose.x, 1.025, 0.1);
  EXPECT_LT(std::abs(robot.pose.y), 0.02);
  EXPECT_LT(std::abs(NormalizeAngle(robot.pose.yaw - M_PI / 2)), 20.0 * M_PI / 180.0 + 1e-9);
}

// The loop of the small lawn with turn_radius 0.44 (sim, 29.10.): a turn on the spot by 76.6 deg, then 290 deg on an arc
// of 0.44 m. With the control point turning at max_angular the mower had nothing left to correct with, fell behind,
// sped up and drifted out to 0.5 m off the path.
TEST(PathTracker, SpinIntoAnArcOfTheTurnRadius) {
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  const double out = -76.6 * M_PI / 180.0;
  path.push_back({1.025, 0.0, out});
  Append(path, Arc(path.back(), 0.44, -290.0 * M_PI / 180.0));
  Append(path, Line(path.back().x, path.back().y, path.back().yaw, 1.0), true);
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(path, {}, 0), "");
  Robot robot;
  Drive(tracker, robot);
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_LT(robot.max_cross, 0.06);
  EXPECT_LE(robot.max_angular, 0.3 + 1e-9);
}

// Out of the docking station: straight ahead without steering, at most the speed of the goal. A mower facing a little
// off the line keeps its heading and ends beside the line.
TEST(PathTracker, StraightDrivesOnWithoutSteering) {
  TrackerConfig config;
  config.straight = true;
  config.max_linear = 0.15;
  config.speed_slow = 0.15;
  PathTracker tracker{config};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 0.8), {}, 0), "");
  Robot robot;
  robot.pose = {0, 0, 0.1};
  Drive(tracker, robot);
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_EQ(robot.max_angular, 0.0);
  EXPECT_LE(robot.max_linear, 0.15 + 1e-9);
  EXPECT_NEAR(robot.pose.yaw, 0.1, 1e-12);
  // Done within max_goal_distance_error along the line, 0.07 m beside it.
  EXPECT_GE(robot.pose.x, 0.7 - 1e-9);
  EXPECT_LT(robot.pose.x, 0.8);
  EXPECT_NEAR(robot.pose.y, robot.pose.x * std::tan(0.1), 0.01);
}

TEST(PathTracker, StraightNeverTurnsOnTheSpotAtTheStart) {
  TrackerConfig config;
  config.straight = true;
  PathTracker tracker{config};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 0.8), {}, 0), "");
  const auto cmd = tracker.Step({0, 0, 30.0 * M_PI / 180.0}, kDt);
  EXPECT_EQ(tracker.phase(), Phase::kFailed);
  EXPECT_NE(tracker.failure().find("start of the path"), std::string::npos) << tracker.failure();
  EXPECT_NE(tracker.failure().find("30 deg"), std::string::npos) << tracker.failure();
  EXPECT_EQ(cmd.linear, 0.0);
  EXPECT_EQ(cmd.angular, 0.0);
}

TEST(PathTracker, StraightOnlyOnALine) {
  TrackerConfig config;
  config.straight = true;
  PathTracker tracker{config};
  std::vector<Pose2> bent = Line(0, 0, 0, 0.5);
  Append(bent, Arc(bent.back(), 0.5, 0.5));
  EXPECT_EQ(tracker.Start(bent, {}, 0), "a straight path must lie on one line, its poses facing along it");
  std::vector<Pose2> askew = Line(0, 0, 0, 0.8);
  askew[3].yaw = 0.1;
  EXPECT_EQ(tracker.Start(askew, {}, 0), "a straight path must lie on one line, its poses facing along it");
  std::vector<Pose2> back = Line(0, 0, 0, 0.8);
  std::reverse(back.begin(), back.end());
  EXPECT_EQ(tracker.Start(back, {}, 0), "a straight path must lie on one line, its poses facing along it");
  EXPECT_EQ(tracker.Start(Line(0, 0, 2.0, 0.8), {}, 0), "");
}

// Wheels that cannot climb out of the hollow of the station: driven at full speed, the mower stays where it is.
TEST(PathTracker, StandingStillWhileDrivenEndsThePath) {
  TrackerConfig config;
  config.straight = true;
  config.max_linear = 0.15;
  config.stall_time = 2.0;
  PathTracker tracker{config};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 0.8), {}, 0), "");
  const Pose2 stuck;
  double t = 0;
  double max_linear = 0;
  while (t < 10 && tracker.phase() != Phase::kFailed) {
    max_linear = std::max(max_linear, tracker.Step(stuck, kDt).linear);
    t += kDt;
  }
  EXPECT_EQ(tracker.phase(), Phase::kFailed);
  EXPECT_NE(tracker.failure().find("does not move"), std::string::npos) << tracker.failure();
  EXPECT_GT(t, 2.0);
  EXPECT_LT(t, 3.0);
  // It never pushed harder than the speed of the goal.
  EXPECT_NEAR(max_linear, 0.15, 1e-9);
  EXPECT_EQ(tracker.Step(stuck, kDt).linear, 0.0);
}

TEST(PathTracker, StandingStillCountsOnlyWhileDrivenForwards) {
  TrackerConfig config;
  config.stall_time = 1.5;
  PathTracker tracker{config};
  // A quarter turn on the spot first: 5 s without forward motion, then the drive speeding up from standstill.
  ASSERT_EQ(tracker.Start(Line(0, 0, M_PI / 2, 2.0), {}, 0), "");
  Robot robot;
  Drive(tracker, robot);
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_NEAR(robot.pose.y, 2.0, 0.1);
}

TEST(PathTracker, StandingStillIsNotCheckedWithoutAStallTime) {
  // The control point runs away until max_follow_distance, as before.
  PathTracker tracker{TrackerConfig()};
  ASSERT_EQ(tracker.Start(Line(0, 0, 0, 10.0), {}, 0), "");
  const Pose2 stuck;
  double t = 0;
  while (t < 60 && tracker.phase() != Phase::kFailed) {
    tracker.Step(stuck, kDt);
    t += kDt;
  }
  EXPECT_NE(tracker.failure().find("stays behind"), std::string::npos) << tracker.failure();
  EXPECT_GT(t, 3.0);
}

// Onto a docking station with the contacts at the back: the poses face east, the mower backs west along them from 5 cm
// beside the line and a little turned, steering onto it.
TEST(PathTracker, BackwardsOntoALine) {
  TrackerConfig config;
  config.reverse = true;
  config.max_linear = 0.1;
  PathTracker tracker{config};
  std::vector<Pose2> path = Line(0, 0, M_PI, 0.8);
  for (auto& p : path) p.yaw = 0;
  ASSERT_EQ(tracker.Start(path, {}, 0), "");
  Robot robot;
  robot.pose = {0.0, 0.05, 0.05};
  double forwards = 0;
  double t = 0;
  while (t < 60 && tracker.phase() != Phase::kDone && tracker.phase() != Phase::kFailed) {
    const auto cmd = tracker.Step(robot.pose, kDt);
    forwards = std::max(forwards, cmd.linear);
    robot.Move(cmd, kDt);
    t += kDt;
  }
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_EQ(forwards, 0.0);
  EXPECT_LE(robot.max_linear, 0.1 + 1e-9);
  EXPECT_LE(robot.max_angular, 0.3 + 1e-9);
  // Within max_goal_distance_error of the end, back on the line and facing east.
  EXPECT_LT(std::hypot(robot.pose.x + 0.8, robot.pose.y), 0.1);
  EXPECT_LT(std::abs(robot.pose.y), 0.01);
  EXPECT_LT(std::abs(robot.pose.yaw), 2.0 * M_PI / 180.0);
}

// The last 0.25 m onto the station: the line reaches 0.05 m past where the station stops the mower. With end_tolerance
// the path is never done there; the mower pushes at the speed of the goal until it counts as stuck.
TEST(PathTracker, BackwardsAgainstAStopIsNeverDone) {
  TrackerConfig config;
  config.reverse = true;
  config.max_linear = 0.06;
  config.stall_time = 10.0;
  config.end_tolerance = 0.005;
  PathTracker tracker{config};
  std::vector<Pose2> path = Line(-0.55, 0, M_PI, 0.3);
  for (auto& p : path) p.yaw = 0;
  ASSERT_EQ(tracker.Start(path, {}, 0), "");
  Robot robot;
  robot.pose = {-0.55, 0.0, 0.0};
  double t = 0;
  double stopped_at = -1;
  while (t < 60 && tracker.phase() != Phase::kDone && tracker.phase() != Phase::kFailed) {
    robot.Move(tracker.Step(robot.pose, kDt), kDt);
    // The station: base_link gets no further west than -0.8.
    if (robot.pose.x < -0.8) {
      robot.pose.x = -0.8;
      if (stopped_at < 0) stopped_at = t;
    }
    t += kDt;
  }
  EXPECT_EQ(tracker.phase(), Phase::kFailed);
  EXPECT_NE(tracker.failure().find("does not move"), std::string::npos) << tracker.failure();
  ASSERT_GT(stopped_at, 0);
  // Counted from the last 5 cm it moved: up to 0.8 s before the stop at 0.06 m/s.
  EXPECT_GT(t - stopped_at, 9.0);
  EXPECT_LT(t - stopped_at, 10.5);
  EXPECT_LE(robot.max_linear, 0.06 + 1e-9);

  // Without the station it gets to the end and is done there, 5 mm before the last pose.
  ASSERT_EQ(tracker.Start(path, {}, 0), "");
  Robot free;
  free.pose = {-0.55, 0.0, 0.0};
  Drive(tracker, free, 60);
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_LT(free.pose.x, -0.845 + 1e-3);
}

// Done along the last stretch, whatever the mower's distance across it and without turning at the end.
TEST(PathTracker, EndToleranceCountsAlongTheLastStretch) {
  TrackerConfig config;
  config.end_tolerance = 0.02;
  PathTracker tracker{config};
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  path.back().yaw = 1.0;
  ASSERT_EQ(tracker.Start(path, {}, 0), "");
  Robot robot;
  robot.pose = {0.0, 0.03, 0.0};
  Drive(tracker, robot, 60);
  EXPECT_EQ(tracker.phase(), Phase::kDone) << tracker.failure();
  EXPECT_GE(robot.pose.x, 0.98 - 1e-3);
  EXPECT_LT(robot.pose.x, 1.0);
  // Not turned to the heading of the last pose.
  EXPECT_LT(std::abs(robot.pose.yaw), 0.2);
}
