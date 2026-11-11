#include "mowbite_logic/direct_route.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mowbite_logic/reasons.h"
#include "mowbite_logic/undocking.h"

namespace mowbite_logic {

namespace {

// m between the samples of an edge. A point of an edge is at most half of it from a sample.
constexpr double kStep = 0.01;
// m added to the clearance the samples have to keep: more than half a step, so no point between two samples is
// closer than the clearance itself.
constexpr double kSlack = 0.01;
// m an edge has to run inside another area to count as covered by it. At least half a step: then a point on the
// outer edge of the areas always has a sample next to it that counts.
constexpr double kCover = 0.01;

// m from p to the segment ab.
double PointToSegment(const Point2& p, const Point2& a, const Point2& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  const double t = len2 > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
  return std::hypot(a.x + t * dx - p.x, a.y + t * dy - p.y);
}

double Side(const Point2& a, const Point2& b, const Point2& p) {
  return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

// m between the segments ab and cd, 0 where they cross.
double SegmentToSegment(const Point2& a, const Point2& b, const Point2& c, const Point2& d) {
  const double c_side = Side(a, b, c);
  const double d_side = Side(a, b, d);
  const double a_side = Side(c, d, a);
  const double b_side = Side(c, d, b);
  if (((c_side > 0 && d_side < 0) || (c_side < 0 && d_side > 0)) &&
      ((a_side > 0 && b_side < 0) || (a_side < 0 && b_side > 0))) {
    return 0.0;
  }
  return std::min({PointToSegment(a, c, d), PointToSegment(b, c, d), PointToSegment(c, a, b),
                   PointToSegment(d, a, b)});
}

double DistanceToRing(const Ring& ring, const Point2& p) {
  double least = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < ring.size(); i++) {
    least = std::min(least, PointToSegment(p, ring[i], ring[(i + 1) % ring.size()]));
  }
  return least;
}

// Whether p on the edge of rings[own] lies at least kCover deep inside another one of the rings.
bool Covered(const std::vector<Ring>& rings, size_t own, const Point2& p) {
  for (size_t i = 0; i < rings.size(); i++) {
    if (i != own && InsideRing(rings[i], p.x, p.y) && DistanceToRing(rings[i], p) >= kCover) return true;
  }
  return false;
}

struct Nearest {
  double distance = std::numeric_limits<double>::infinity();
  Point2 at;
};

// The sample on the edges of the rings that is closest to the line ab, among those closer than limit; with
// skip_covered a sample covered by another ring does not count.
Nearest Scan(const std::vector<Ring>& rings, bool skip_covered, const Point2& a, const Point2& b, double limit) {
  Nearest nearest;
  for (size_t r = 0; r < rings.size(); r++) {
    const Ring& ring = rings[r];
    for (size_t i = 0; i < ring.size(); i++) {
      const Point2& p = ring[i];
      const Point2& q = ring[(i + 1) % ring.size()];
      if (SegmentToSegment(p, q, a, b) >= limit) continue;
      const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(q.x - p.x, q.y - p.y) / kStep)));
      for (int k = 0; k <= steps; k++) {
        const Point2 s{p.x + (q.x - p.x) * k / steps, p.y + (q.y - p.y) * k / steps};
        const double distance = PointToSegment(s, a, b);
        if (distance >= limit || distance >= nearest.distance) continue;
        if (skip_covered && Covered(rings, r, s)) continue;
        nearest = {distance, s};
      }
    }
  }
  return nearest;
}

std::string Blocked(const char* what, const Nearest& nearest, double limit) {
  return std::string(what) + " " + Metres(nearest.distance) + " m from the way at (" + Metres(nearest.at.x) + ", " +
         Metres(nearest.at.y) + "), less than " + Metres(limit) + " m";
}

}  // namespace

DirectRouteCheck CheckDirectRoute(const Point2& from, const Point2& to, double turning_radius, double margin,
                                  const std::vector<Ring>& drivable, const std::vector<Ring>& obstacles) {
  DirectRouteCheck check;
  if (!(turning_radius > 0) || !(margin > 0) || !std::isfinite(turning_radius) || !std::isfinite(margin)) {
    check.why = "the size of the body is not known";
    return check;
  }
  if (!std::isfinite(from.x) || !std::isfinite(from.y) || !std::isfinite(to.x) || !std::isfinite(to.y)) {
    check.why = "the way has no valid ends";
    return check;
  }
  for (const auto& ring : obstacles) {
    if (InsideRing(ring, from.x, from.y)) {
      check.why = "the mower stands in an obstacle";
      return check;
    }
  }
  if (std::none_of(drivable.begin(), drivable.end(),
                   [&from](const Ring& ring) { return InsideRing(ring, from.x, from.y); })) {
    check.why = "the mower stands outside of the mowing and navigation areas";
    return check;
  }
  const double limit = turning_radius + margin + kSlack;
  const Nearest obstacle = Scan(obstacles, false, from, to, limit);
  if (obstacle.distance < limit) {
    check.why = Blocked("an obstacle", obstacle, limit);
    return check;
  }
  const Nearest edge = Scan(drivable, true, from, to, limit);
  if (edge.distance < limit) {
    check.why = Blocked("the areas end", edge, limit);
    return check;
  }
  check.free = true;
  return check;
}

}  // namespace mowbite_logic
