#ifndef MOWBITE_FOLLOWER_TEST_PATHS_H
#define MOWBITE_FOLLOWER_TEST_PATHS_H

#include <algorithm>
#include <cmath>
#include <vector>

#include "mowbite_follower/path.h"

namespace mowbite_follower::test {

constexpr double kDeg = M_PI / 180.0;

// Poses every spacing m along a straight line from (x, y) in direction yaw.
inline std::vector<Pose2> Line(double x, double y, double yaw, double length, double spacing = 0.1) {
  std::vector<Pose2> poses;
  const int n = static_cast<int>(std::round(length / spacing));
  for (int i = 0; i <= n; i++) {
    poses.push_back({x + i * spacing * std::cos(yaw), y + i * spacing * std::sin(yaw), yaw});
  }
  return poses;
}

// An arc of radius r from pose p (left for angle > 0) without p itself, poses every spacing m and at least every
// 10 deg, as the planner spaces them.
inline std::vector<Pose2> Arc(const Pose2& p, double r, double angle, double spacing = 0.1) {
  std::vector<Pose2> poses;
  const double side = angle > 0 ? 1.0 : -1.0;
  const double cx = p.x - side * r * std::sin(p.yaw);
  const double cy = p.y + side * r * std::cos(p.yaw);
  const int n = std::max({2, static_cast<int>(std::ceil(std::abs(angle) * r / spacing)),
                          static_cast<int>(std::ceil(std::abs(angle) / (10.0 * kDeg) - 1e-9))});
  for (int i = 1; i <= n; i++) {
    const double yaw = p.yaw + angle * i / n;
    poses.push_back({cx + side * r * std::sin(yaw), cy - side * r * std::cos(yaw), NormalizeAngle(yaw)});
  }
  return poses;
}

inline void Append(std::vector<Pose2>& path, const std::vector<Pose2>& more, bool skip_first = false) {
  path.insert(path.end(), more.begin() + (skip_first ? 1 : 0), more.end());
}

// What the planner does where no arc fits: a line, a pose 2.5 cm on with the new heading, a line on from there.
inline std::vector<Pose2> SpinPath(double turn) {
  std::vector<Pose2> path = Line(0, 0, 0, 1.0);
  path.push_back({1.025, 0.0, NormalizeAngle(turn)});
  Append(path, Line(1.025, 0.0, NormalizeAngle(turn), 1.5), true);
  return path;
}

}  // namespace mowbite_follower::test

#endif  // MOWBITE_FOLLOWER_TEST_PATHS_H
