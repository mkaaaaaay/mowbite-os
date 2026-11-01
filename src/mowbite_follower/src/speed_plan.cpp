#include "mowbite_follower/speed_plan.h"

#include <algorithm>

namespace mowbite_follower {

SpeedPlan::SpeedPlan(const Path& path, const SpeedLimits& limits) : limits_(limits) {
  const size_t n = path.size();
  cap_.assign(n - 1, 0.0);
  for (size_t i = 0; i + 1 < n; i++) {
    double cap = path.StraightAhead(i, limits.straight_length, limits.straight_angle) ? limits.speed_fast
                                                                                      : limits.speed_slow;
    // A segment belongs to the kind of the pose it leads to: the first pose of a turn ends the lane before it.
    if (path.kind(i + 1) == Kind::kTurn) cap = std::min(cap, limits.speed_turn);
    const double curvature = std::abs(path.curvature(i));
    if (curvature > 1e-6) cap = std::min(cap, limits.turn_rate / curvature);
    cap_[i] = std::min(cap, limits.max_linear);
  }
  at_pose_.assign(n, 0.0);
  at_pose_[0] = cap_[0];
  for (size_t i = 1; i + 1 < n; i++) {
    at_pose_[i] = path.corner(i) ? std::min(limits.creep_speed, std::min(cap_[i - 1], cap_[i]))
                                 : std::min(cap_[i - 1], cap_[i]);
  }
  at_pose_[n - 1] = std::min(limits.creep_speed, cap_[n - 2]);
  for (size_t i = n - 1; i-- > 0;) {
    const double braking = std::sqrt(at_pose_[i + 1] * at_pose_[i + 1] + 2.0 * limits.acceleration * path.length(i));
    at_pose_[i] = std::min(at_pose_[i], braking);
  }
}

double SpeedPlan::At(size_t segment, double remaining) const {
  const double next = at_pose_[segment + 1];
  return std::min(cap_[segment], std::sqrt(next * next + 2.0 * limits_.acceleration * std::max(remaining, 0.0)));
}

double SpeedPlan::Floor(size_t segment) const { return std::min(limits_.creep_speed, cap_[segment]); }

}  // namespace mowbite_follower
