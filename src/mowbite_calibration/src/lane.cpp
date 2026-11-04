#include "mowbite_calibration/lane.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mowbite_calibration {

namespace {

using mowbite_safety::DistanceToRing;
using mowbite_safety::InsideRing;
using mowbite_safety::Point2;
using mowbite_safety::Region;
using mowbite_safety::Ring;

constexpr double kEpsilon = 1e-9;

// The bounds of a ring; one without points contains nothing (right below left).
struct Box {
  double left = 0;
  double bottom = 0;
  double right = -1;
  double top = -1;
  bool Contains(double x, double y) const {
    return x >= left && x <= right && y >= bottom && y <= top;
  }
};

Box BoxOf(const Ring& ring, double grow) {
  Box box;
  if (ring.empty()) return box;
  box.left = box.right = ring.front().x;
  box.bottom = box.top = ring.front().y;
  for (const Point2& p : ring) {
    box.left = std::min(box.left, p.x);
    box.bottom = std::min(box.bottom, p.y);
    box.right = std::max(box.right, p.x);
    box.top = std::max(box.top, p.y);
  }
  box.left -= grow;
  box.bottom -= grow;
  box.right += grow;
  box.top += grow;
  return box;
}

// The region ready for many points: where base_link may be, with obstacles grown by clearance.
class Field {
 public:
  Field(const Region& region, double clearance) : region_(region), clearance_(clearance) {
    for (const Ring& ring : region.allowed) allowed_.push_back(BoxOf(ring, 0));
    for (const Ring& ring : region.obstacles) obstacles_.push_back(BoxOf(ring, clearance));
  }

  bool Free(double x, double y) const {
    for (size_t i = 0; i < obstacles_.size(); i++) {
      if (!obstacles_[i].Contains(x, y)) continue;
      const Ring& ring = region_.obstacles[i];
      if (InsideRing(ring, x, y) || DistanceToRing(ring, x, y) < clearance_) return false;
    }
    for (size_t i = 0; i < allowed_.size(); i++) {
      if (allowed_[i].Contains(x, y) && InsideRing(region_.allowed[i], x, y)) return true;
    }
    return false;
  }

 private:
  const Region& region_;
  double clearance_;
  std::vector<Box> allowed_;
  std::vector<Box> obstacles_;
};

// The longest lane up to config.length along heading from (x, y) whose strip is free: rows of points across it from
// its back end forward, up to the last row before the first that is not free, and short of the station by its
// clearance. Below 0 where not even a lane of length 0 fits.
double FreeLength(const Field& field, const Region& region, double x, double y, double heading, double radius,
                  const LaneConfig& config) {
  const double ux = std::cos(heading);
  const double uy = std::sin(heading);
  // The strip runs from -radius behind base_link to end ahead of it, radius to each side.
  double end_limit = config.length + radius;
  if (region.station) {
    const double sx = region.station->x - x;
    const double sy = region.station->y - y;
    const double along = sx * ux + sy * uy;
    const double side = std::max(0.0, std::abs(-sx * uy + sy * ux) - radius);
    if (side < config.station_clearance) {
      const double reach = std::sqrt(config.station_clearance * config.station_clearance - side * side);
      // Behind the back end the distance does not depend on the length.
      if (along < -radius && std::hypot(-radius - along, side) < config.station_clearance) return -1;
      if (along >= -radius) end_limit = std::min(end_limit, along - reach);
    }
  }
  if (end_limit < radius) return -1;
  const double h = config.resolution;
  const int across = std::max(1, static_cast<int>(std::ceil(2.0 * radius / h - kEpsilon)));
  double last_free = -std::numeric_limits<double>::infinity();
  for (int row = 0;; row++) {
    const double s = std::min(-radius + row * h, end_limit);
    const double cx = x + s * ux;
    const double cy = y + s * uy;
    for (int j = 0; j <= across; j++) {
      const double t = -radius + 2.0 * radius * j / across;
      if (!field.Free(cx - t * uy, cy + t * ux)) return last_free - radius;
    }
    last_free = s;
    if (s >= end_limit - kEpsilon) break;
  }
  return last_free - radius;
}

double Floor(double length) {
  return std::floor(length * 100.0 + 1e-6) / 100.0;
}

}  // namespace

LaneChoice ChooseLane(const Region& region, double x, double y, double heading, double body_radius,
                      const LaneConfig& config) {
  const double radius = body_radius + config.margin;
  const Field field(region, config.resolution);
  LaneChoice choice;
  choice.strip_width = 2.0 * radius;
  double best = -1;
  double best_heading = heading;
  const int half = std::max(1, static_cast<int>(std::lround(M_PI / config.direction_step)));
  // Nearest first, left before right; the opposite direction once.
  for (int k = 0; k <= half; k++) {
    for (int side : {1, -1}) {
      if ((k == 0 || k == half) && side < 0) continue;
      const double direction = std::atan2(std::sin(heading + side * k * config.direction_step),
                                          std::cos(heading + side * k * config.direction_step));
      const double length = FreeLength(field, region, x, y, direction, radius, config);
      if (length >= config.length - kEpsilon) {
        choice.lane = Lane{x, y, direction, config.length};
        choice.free_length = config.length;
        choice.strip_length = config.length + 2.0 * radius;
        return choice;
      }
      if (length > best + kEpsilon) {
        best = length;
        best_heading = direction;
      }
    }
  }
  choice.free_length = std::max(0.0, Floor(best));
  if (best >= config.min_length - kEpsilon) {
    choice.lane = Lane{x, y, best_heading, std::max(config.min_length, Floor(best))};
    choice.strip_length = choice.lane->length + 2.0 * radius;
  } else {
    choice.strip_length = config.min_length + 2.0 * radius;
  }
  return choice;
}

bool LaneFits(const Region& region, const Lane& lane, double body_radius, const LaneConfig& config) {
  LaneConfig exact = config;
  exact.length = lane.length;
  const Field field(region, config.resolution);
  return FreeLength(field, region, lane.x, lane.y, lane.heading, body_radius + config.margin, exact) >=
         lane.length - kEpsilon;
}

double BodyRadius(double front, double rear, double width, const std::vector<Point2>& outline) {
  double radius = 0;
  if (outline.size() >= 3) {
    for (const Point2& p : outline) radius = std::max(radius, std::hypot(p.x, p.y));
    return radius;
  }
  return std::max(std::hypot(front, width / 2.0), std::hypot(rear, width / 2.0));
}

}  // namespace mowbite_calibration
