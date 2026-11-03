#ifndef MOWBITE_FOLLOWER_STEERING_H
#define MOWBITE_FOLLOWER_STEERING_H

#include <cmath>

#include "mowbite_follower/path.h"

namespace mowbite_follower {

// Steering onto a path along a field of courses: beside the path the mower should head towards it at an angle that
// grows with the distance, up to approach_angle far off; on it, the course of the path. The turn rate holds the
// mower on that course and follows how the course changes as it moves, so arcs need no standing error.
struct SteeringConfig {
  // m beside the path at which the course towards it is half of approach_angle.
  double approach_distance = 0.15;
  // rad towards the path far from it.
  double approach_angle = 45.0 * M_PI / 180.0;
  // 1/s, how fast a heading error decays.
  double heading_gain = 2.0;
};

// rad, the course to hold at a foot point of the path.
double DesiredCourse(const SteeringConfig& config, const Projection& foot);

// rad/s for a mower with the heading of travel (rad) moving at speed (m/s, never negative) beside the foot point.
double TurnRate(const SteeringConfig& config, const Projection& foot, double heading, double speed);

}  // namespace mowbite_follower

#endif  // MOWBITE_FOLLOWER_STEERING_H
