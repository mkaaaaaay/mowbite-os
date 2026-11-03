#include "mowbite_follower/path.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mowbite_follower {

namespace {

// Poses closer than this (m) are merged, at the same position they are refused.
constexpr double kMerge = 1e-3;
constexpr double kSamePosition = 1e-9;

}  // namespace

double NormalizeAngle(double angle) {
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle > M_PI) angle -= 2.0 * M_PI;
  if (angle <= -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::string Path::Build(const std::vector<Pose2>& poses, const std::vector<Kind>& kinds, double corner_angle,
                        double min_radius) {
  *this = Path();
  if (poses.size() < 3) return "a path needs at least 3 poses";
  if (!kinds.empty() && kinds.size() != poses.size()) return "kinds must be empty or one per pose";
  for (size_t i = 0; i < poses.size(); i++) {
    const Pose2& p = poses[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw)) {
      return "pose " + std::to_string(i) + " is not finite";
    }
    if (i > 0 && std::hypot(p.x - poses[i - 1].x, p.y - poses[i - 1].y) < kSamePosition) {
      return "pose " + std::to_string(i) + " lies on the one before";
    }
  }
  for (size_t i = 0; i < poses.size(); i++) {
    const Kind kind = kinds.empty() ? Kind::kTransit : kinds[i];
    if (i > 0 && std::hypot(poses[i].x - poses_.back().x, poses[i].y - poses_.back().y) < kMerge) {
      // The end stays where the path ends, with the heading of its last pose.
      if (i + 1 == poses.size() && poses_.size() > 1) {
        poses_.back() = poses[i];
        source_.back() = i;
      }
      continue;
    }
    poses_.push_back(poses[i]);
    kinds_.push_back(kind);
    source_.push_back(i);
  }
  if (poses_.size() < 2) {
    *this = Path();
    return "the poses of the path lie on one point";
  }
  const size_t n = poses_.size();
  s_.assign(n, 0.0);
  length_.assign(n - 1, 0.0);
  direction_.assign(n - 1, 0.0);
  for (size_t i = 0; i + 1 < n; i++) {
    const double dx = poses_[i + 1].x - poses_[i].x;
    const double dy = poses_[i + 1].y - poses_[i].y;
    length_[i] = std::hypot(dx, dy);
    direction_[i] = std::atan2(dy, dx);
    s_[i + 1] = s_[i] + length_[i];
  }
  corner_.assign(n, false);
  heading_in_.assign(n, 0.0);
  heading_out_.assign(n, 0.0);
  heading_out_[0] = direction_[0];
  heading_in_[n - 1] = direction_[n - 2];
  for (size_t i = 1; i + 1 < n; i++) {
    const double turn = NormalizeAngle(direction_[i] - direction_[i - 1]);
    // The radius of the bend: half of the two segments over the turn.
    const double around = 0.5 * (length_[i - 1] + length_[i]);
    corner_[i] = std::abs(turn) > corner_angle || around < min_radius * std::abs(turn);
    if (corner_[i]) {
      heading_in_[i] = direction_[i - 1];
      heading_out_[i] = direction_[i];
    } else {
      // Halfway between the two segments: on an arc of evenly spaced poses that is its tangent.
      heading_in_[i] = heading_out_[i] = NormalizeAngle(direction_[i - 1] + 0.5 * turn);
    }
  }
  curvature_.assign(n - 1, 0.0);
  for (size_t i = 0; i + 1 < n; i++) {
    curvature_[i] = NormalizeAngle(heading_in_[i + 1] - heading_out_[i]) / length_[i];
  }
  return "";
}

size_t Path::Merged(size_t given) const {
  size_t i = 0;
  while (i + 1 < source_.size() && source_[i + 1] <= given) i++;
  return i;
}

size_t Path::NextStop(size_t i) const {
  for (size_t k = i + 1; k + 1 < poses_.size(); k++) {
    if (corner_[k]) return k;
  }
  return poses_.size() - 1;
}

bool Path::StraightAhead(size_t segment, double length, double angle) const {
  const double from = s_[segment];
  for (size_t i = segment; i < length_.size() && s_[i] < from + length; i++) {
    if (std::abs(NormalizeAngle(direction_[i] - direction_[segment])) > angle) return false;
  }
  return true;
}

Projection Path::Project(double x, double y, size_t first, size_t last) const {
  last = std::min(last, length_.size() - 1);
  first = std::min(first, last);
  Projection best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (size_t i = first; i <= last; i++) {
    const double c = std::cos(direction_[i]);
    const double sn = std::sin(direction_[i]);
    const double rx = x - poses_[i].x;
    const double ry = y - poses_[i].y;
    const double along = rx * c + ry * sn;
    const double t = std::clamp(along / length_[i], 0.0, 1.0);
    const double fx = poses_[i].x + t * length_[i] * c;
    const double fy = poses_[i].y + t * length_[i] * sn;
    const double distance = std::hypot(x - fx, y - fy);
    if (distance > best_distance + 1e-9) continue;
    best_distance = distance;
    best.segment = i;
    best.s = s_[i] + t * length_[i];
    const bool open_end = (i == first && along < 0) || (i == last && along > length_[i]);
    best.along = open_end ? s_[i] + along : best.s;
    const double side = c * ry - sn * rx;
    // Before or past an open end the distance to the line counts, so that it does not grow along the path.
    best.cross = open_end ? side : std::copysign(distance, side);
    best.heading = NormalizeAngle(heading_out_[i] + t * NormalizeAngle(heading_in_[i + 1] - heading_out_[i]));
    best.curvature = curvature_[i];
  }
  return best;
}

}  // namespace mowbite_follower
