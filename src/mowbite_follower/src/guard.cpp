#include "mowbite_follower/guard.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

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

GuardResult CheckGuard(const GuardConfig& config, const GuardInput& in, const Geofence& fence) {
  if (!in.safety_measured) {
    return {StopReason::kNotAllowed, "no safety measurement of this platform (hw/capabilities): remote control only"};
  }
  if (in.hw_age > config.hw_timeout) return {StopReason::kEmergency, "no hw/status from the driver"};
  if (in.emergency) return {StopReason::kEmergency, "emergency"};
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
  if (std::abs(in.cross_track) > config.max_cross_track) {
    return {StopReason::kCrossTrack, Format("%.2f m off the path (limit %.2f m)", std::abs(in.cross_track),
                                            config.max_cross_track)};
  }
  return {};
}

}  // namespace mowbite_follower
