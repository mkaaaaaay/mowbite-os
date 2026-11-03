#ifndef MOWBITE_FOLLOWER_SPEED_PLAN_H
#define MOWBITE_FOLLOWER_SPEED_PLAN_H

#include <cmath>
#include <cstddef>
#include <vector>

#include "mowbite_follower/path.h"

namespace mowbite_follower {

struct SpeedLimits {
  // m/s where the path goes on straight for straight_length m within straight_angle rad, elsewhere and on turns.
  double speed_fast = 0.3;
  double speed_slow = 0.15;
  double speed_turn = 0.1;
  double straight_length = 1.0;
  double straight_angle = 20.0 * M_PI / 180.0;
  double max_linear = 0.3;
  // rad/s the path itself may ask for on arcs; the rest up to max_angular is left to correct with.
  double turn_rate = 0.25;
  // m/s² to brake with before corners and the end.
  double acceleration = 0.1;
  // m/s the mower creeps onto corners and the end with, above the dead band of the wheels.
  double creep_speed = 0.06;
};

// The highest speed along a path, computed once per path: a cap per segment and, going back from every stop (corners
// and the end), the speed from which the mower can still brake to creep_speed there.
class SpeedPlan {
 public:
  SpeedPlan() = default;
  SpeedPlan(const Path& path, const SpeedLimits& limits);

  // m/s at most anywhere on the segment.
  double cap(size_t segment) const { return cap_[segment]; }
  // m/s at most at pose i.
  double at_pose(size_t i) const { return at_pose_[i]; }
  // m/s at most on the segment with remaining m to its end.
  double At(size_t segment, double remaining) const;
  // m/s the mower does not go below while it drives on the segment.
  double Floor(size_t segment) const;

 private:
  SpeedLimits limits_;
  std::vector<double> cap_;
  std::vector<double> at_pose_;
};

}  // namespace mowbite_follower

#endif  // MOWBITE_FOLLOWER_SPEED_PLAN_H
