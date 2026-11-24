#include "mowbite_follower/guard.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace mowbite_follower {

namespace {

// A closed ring repeats its first point; the checks below work on the open one.
size_t OpenSize(const Ring& ring) {
  size_t n = ring.size();
  if (n > 1 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) n--;
  return n;
}

std::string Format(const char* format, double a, double b) {
  char text[160];
  std::snprintf(text, sizeof(text), format, a, b);
  return text;
}

// rad between the checked positions of a turning body, and m between the checked points of its edge; far below the
// tolerance of such a check (5 degrees at 0.7 m: 1 mm between the arcs).
constexpr double kTurnStep = 5.0 * M_PI / 180.0;
constexpr double kEdgeStep = 0.03;

// The points along the edges of a ring, at most kEdgeStep apart, its corners among them.
std::vector<Point2> EdgePoints(const Ring& ring) {
  const size_t n = OpenSize(ring);
  std::vector<Point2> points;
  for (size_t i = 0; i < n; i++) {
    const Point2& a = ring[i];
    const Point2& b = ring[(i + 1) % n];
    const int parts = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / kEdgeStep)));
    for (int k = 0; k < parts; k++) {
      const double t = static_cast<double>(k) / parts;
      points.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
    }
  }
  return points;
}

// Whether any point of the ring may be within reach m of (x, y), by its bounding box.
bool InReach(const Ring& ring, double x, double y, double reach) {
  double x_min = std::numeric_limits<double>::infinity();
  double y_min = x_min;
  double x_max = -x_min;
  double y_max = -x_min;
  for (const Point2& p : ring) {
    x_min = std::min(x_min, p.x);
    x_max = std::max(x_max, p.x);
    y_min = std::min(y_min, p.y);
    y_max = std::max(y_max, p.y);
  }
  return std::hypot(std::max({x_min - x, x - x_max, 0.0}), std::max({y_min - y, y - y_max, 0.0})) <= reach;
}

}  // namespace

bool InsideRing(const Ring& ring, double x, double y) {
  const size_t n = OpenSize(ring);
  bool inside = false;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const Point2& a = ring[i];
    const Point2& b = ring[j];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
}

double DistanceToRing(const Ring& ring, double x, double y) {
  const size_t n = OpenSize(ring);
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < n; i++) {
    const Point2& a = ring[i];
    const Point2& b = ring[(i + 1) % n];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    const double t = len2 > 0 ? std::clamp(((x - a.x) * dx + (y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
    best = std::min(best, std::hypot(a.x + t * dx - x, a.y + t * dy - y));
  }
  return best;
}

Geofence::Geofence(std::vector<Ring> allowed, std::vector<Ring> obstacles, double margin)
    : allowed_(std::move(allowed)), obstacles_(std::move(obstacles)), margin_(margin) {}

std::string Geofence::Problem() const {
  if (allowed_.empty()) return "no allowed area";
  const auto finite = [](const Ring& ring) {
    return std::all_of(ring.begin(), ring.end(),
                       [](const Point2& p) { return std::isfinite(p.x) && std::isfinite(p.y); });
  };
  for (const auto& ring : allowed_) {
    if (!finite(ring)) return "a point of an allowed area is no number";
  }
  for (const auto& ring : obstacles_) {
    if (!finite(ring)) return "a point of an obstacle is no number";
  }
  for (const auto& ring : allowed_) {
    if (OpenSize(ring) < 3) return "an allowed area has fewer than 3 points";
  }
  for (const auto& ring : obstacles_) {
    if (OpenSize(ring) < 3) return "an obstacle has fewer than 3 points";
  }
  return "";
}

std::string Geofence::Check(double x, double y) const {
  for (const auto& ring : obstacles_) {
    if (InsideRing(ring, x, y)) return Format("the mower is in an obstacle at (%.2f, %.2f)", x, y);
  }
  double closest = std::numeric_limits<double>::infinity();
  for (const auto& ring : allowed_) {
    if (InsideRing(ring, x, y)) return "";
    closest = std::min(closest, DistanceToRing(ring, x, y));
  }
  if (closest <= margin_) return "";
  char text[160];
  std::snprintf(text, sizeof(text), "the mower is %.2f m outside of the allowed areas at (%.2f, %.2f)", closest, x, y);
  return text;
}

std::string Geofence::CheckTurn(const Pose2& at, double delta, const Ring& body, double tolerance) const {
  double radius = 0;
  for (const Point2& p : body) radius = std::max(radius, std::hypot(p.x, p.y));
  const double reach = radius + tolerance;
  std::vector<const Ring*> allowed;
  std::vector<const Ring*> obstacles;
  for (const auto& ring : allowed_) {
    if (InReach(ring, at.x, at.y, reach)) allowed.push_back(&ring);
  }
  for (const auto& ring : obstacles_) {
    if (InReach(ring, at.x, at.y, reach)) obstacles.push_back(&ring);
  }
  const std::vector<Point2> edge = EdgePoints(body);
  // The most the body gets out of the allowed areas and into an obstacle over the whole turn.
  double out = 0;
  double into = 0;
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(delta) / kTurnStep)));
  for (int k = 0; k <= steps; k++) {
    const double yaw = at.yaw + delta * k / steps;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const auto place = [&](const Point2& p) { return Point2{at.x + c * p.x - s * p.y, at.y + s * p.x + c * p.y}; };
    for (const Point2& local : edge) {
      const Point2 p = place(local);
      for (const Ring* ring : obstacles) {
        if (InsideRing(*ring, p.x, p.y)) into = std::max(into, DistanceToRing(*ring, p.x, p.y));
      }
      double outside = reach;
      for (const Ring* ring : allowed) {
        outside = InsideRing(*ring, p.x, p.y) ? 0.0 : std::min(outside, DistanceToRing(*ring, p.x, p.y));
        if (outside == 0.0) break;
      }
      out = std::max(out, outside);
    }
    // An obstacle smaller than the body: its corners inside the body.
    Ring placed;
    for (const Point2& p : body) placed.push_back(place(p));
    for (const Ring* ring : obstacles) {
      for (const Point2& p : *ring) {
        if (InsideRing(placed, p.x, p.y)) into = std::max(into, DistanceToRing(placed, p.x, p.y));
      }
    }
  }
  const double degrees = std::abs(delta) * 180.0 / M_PI;
  if (into > tolerance) {
    return Format("turning on the spot by %.0f deg, the body would reach %.2f m into an obstacle", degrees, into);
  }
  if (out > tolerance) {
    return Format("turning on the spot by %.0f deg, the body would get %.2f m out of the allowed areas", degrees, out);
  }
  return "";
}

GuardResult CheckGuard(const GuardConfig& config, const GuardInput& in, const Geofence& fence) {
  if (!in.safety_measured) {
    return {StopReason::kNotAllowed,
            "no safety measurement of this platform (hw/capabilities): mowing and driving are blocked"};
  }
  if (!in.driver_moves) {
    return {StopReason::kNotAllowed, "the driver allows no speed (hw/capabilities): driving is blocked"};
  }
  if (in.hw_age > config.hw_timeout) return {StopReason::kEmergency, "no hw/status from the driver"};
  if (in.emergency) return {StopReason::kEmergency, "emergency"};
  if (!std::isfinite(in.pose.x) || !std::isfinite(in.pose.y) || !std::isfinite(in.pose.yaw)) {
    return {StopReason::kPoseLost, "the pose has a value that is no number"};
  }
  if (in.pose_age > config.pose_timeout) {
    return {StopReason::kPoseLost, Format("no pose for %.1f s (limit %.1f s)", in.pose_age, config.pose_timeout)};
  }
  if (!(in.position_accuracy <= config.max_position_accuracy)) {
    return {StopReason::kPositionAccuracy, Format("position accuracy %.2f m worse than %.2f m", in.position_accuracy,
                                                  config.max_position_accuracy)};
  }
  if (!in.heading_valid) return {StopReason::kHeading, "the heading is not aligned"};
  std::string outside = fence.Check(in.pose.x, in.pose.y);
  if (!outside.empty()) return {StopReason::kOutside, outside};
  if (config.max_turn_slide > 0 && !(in.turn_slide <= config.max_turn_slide)) {
    return {StopReason::kTurnSlide, Format("slid %.2f m from the place of the turn on the spot (limit %.2f m)",
                                           in.turn_slide, config.max_turn_slide)};
  }
  if (in.max_off_path > 0 && !(in.off_path <= in.max_off_path)) {
    return {StopReason::kCrossTrack, Format("%.2f m off the path (limit of this path %.2f m)", in.off_path,
                                            in.max_off_path)};
  }
  if (std::abs(in.cross_track) > config.max_cross_track) {
    return {StopReason::kCrossTrack, Format("%.2f m off the path (limit %.2f m)", std::abs(in.cross_track),
                                            config.max_cross_track)};
  }
  return {};
}

bool TurnFixFresh(double fix_age, double pose_age, double limit) {
  return !(limit > 0) || fix_age + pose_age <= limit;
}

bool GateOpen(bool gate, double age, bool go, double timeout) {
  return !gate || (go && age >= 0 && age <= timeout);
}

void CapAngular(double max_angular, double* linear, double* angular) {
  if (!(std::abs(*angular) > max_angular) || !(max_angular >= 0)) return;
  const double scale = max_angular / std::abs(*angular);
  *linear *= scale;
  *angular *= scale;
}

void CapLinear(double max_linear, double* linear, double* angular) {
  if (!(std::abs(*linear) > max_linear) || !(max_linear >= 0)) return;
  const double scale = max_linear / std::abs(*linear);
  *linear *= scale;
  *angular *= scale;
}

}  // namespace mowbite_follower
