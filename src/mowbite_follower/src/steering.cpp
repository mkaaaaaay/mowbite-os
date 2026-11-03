#include "mowbite_follower/steering.h"

#include <algorithm>

namespace mowbite_follower {

namespace {

// rad of course per unit of atan(cross / approach_distance).
double Slope(const SteeringConfig& config) { return config.approach_angle * 2.0 / M_PI; }

}  // namespace

double DesiredCourse(const SteeringConfig& config, const Projection& foot) {
  return NormalizeAngle(foot.heading - Slope(config) * std::atan(foot.cross / config.approach_distance));
}

double TurnRate(const SteeringConfig& config, const Projection& foot, double heading, double speed) {
  const double off_path = NormalizeAngle(heading - foot.heading);
  // How fast the mower moves away from the path to its left, and how fast the foot point turns with the path.
  const double cross_rate = speed * std::sin(off_path);
  const double inside = std::max(1.0 - foot.curvature * foot.cross, 0.2);
  const double path_rate = foot.curvature * speed * std::cos(off_path) / inside;
  const double q = foot.cross / config.approach_distance;
  const double course_rate = path_rate - Slope(config) / config.approach_distance / (1.0 + q * q) * cross_rate;
  const double error = NormalizeAngle(heading - DesiredCourse(config, foot));
  return course_rate - config.heading_gain * error;
}

}  // namespace mowbite_follower
