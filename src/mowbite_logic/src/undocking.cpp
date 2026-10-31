#include "mowbite_logic/undocking.h"

#include <algorithm>

namespace mowbite_logic {

namespace {

// The follower needs poses at most this far apart (m) on a straight path, and at least 3 of them.
constexpr double kPoseSpacing = 0.1;

}  // namespace

UndockPath PlanUndock(double x, double y, double heading, const UndockConfig& config) {
  UndockPath path;
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  const int steps = std::max(2, static_cast<int>(std::ceil(config.distance / kPoseSpacing - 1e-9)));
  for (int i = 0; i <= steps; i++) {
    const double along = config.distance * i / steps;
    path.poses.push_back({x + along * c, y + along * s, heading});
  }
  // From corridor m behind the start, so that base_link starts inside it.
  const double start_x = x - config.corridor * c;
  const double start_y = y - config.corridor * s;
  const double end_x = x + config.distance * c;
  const double end_y = y + config.distance * s;
  const double side_x = -s * config.corridor;
  const double side_y = c * config.corridor;
  path.corridor = {{start_x + side_x, start_y + side_y}, {start_x - side_x, start_y - side_y},
                   {end_x - side_x, end_y - side_y}, {end_x + side_x, end_y + side_y}};
  path.speed = config.speed;
  path.stall_time = config.stall_time;
  return path;
}

bool InsideRing(const Ring& ring, double x, double y) {
  bool inside = false;
  const size_t n = ring.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const Point2& a = ring[i];
    const Point2& b = ring[j];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
}

bool Reachable(const std::vector<Ring>& drivable, const std::vector<Ring>& obstacles, double x, double y) {
  for (const auto& ring : obstacles) {
    if (InsideRing(ring, x, y)) return false;
  }
  return std::any_of(drivable.begin(), drivable.end(), [x, y](const Ring& ring) { return InsideRing(ring, x, y); });
}

std::optional<double> CourseError(double from_x, double from_y, double heading, double x, double y,
                                  double min_distance) {
  const double dx = x - from_x;
  const double dy = y - from_y;
  if (std::hypot(dx, dy) < min_distance) return std::nullopt;
  const double error = std::atan2(dy, dx) - heading;
  return std::atan2(std::sin(error), std::cos(error));
}

}  // namespace mowbite_logic
