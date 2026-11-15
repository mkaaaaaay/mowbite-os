#include "mowbite_localization/course_alignment.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "mowbite_localization/pose_filter.h"

namespace mowbite_localization {

CourseHeading HeadingFromCourse(const Eigen::Vector2d& velocity, double speed, double turn_rate,
                                const Eigen::Vector2d& lever, const CourseConfig& config) {
  CourseHeading result;
  if (!velocity.allFinite() || !std::isfinite(speed) || !std::isfinite(turn_rate)) return result;
  // The antenna moves with base_link plus its turn about it: in the frame of base_link that is (v - w ly, w lx).
  const Eigen::Vector2d body(speed - turn_rate * lever(1), turn_rate * lever(0));
  const double measured = velocity.norm();
  const double expected = body.norm();
  if (std::abs(speed) < config.min_speed || measured < config.min_speed) return result;
  if (std::abs(measured - expected) > config.speed_tolerance + config.speed_ratio * expected) {
    result.kind = CourseKind::kInconsistent;
    return result;
  }
  result.kind = CourseKind::kUsable;
  result.heading = WrapAngle(std::atan2(velocity(1), velocity(0)) - std::atan2(body(1), body(0)));
  return result;
}

double CourseVariance(double speed, const CourseConfig& config) {
  const double angle = config.velocity_noise / std::max(speed, config.min_speed);
  return angle * angle;
}

CourseAlignment::CourseAlignment(const CourseConfig& config) : config_(config) {
}

std::optional<double> CourseAlignment::Add(double difference, double t) {
  differences_.emplace_back(t, WrapAngle(difference));
  while (static_cast<int>(differences_.size()) > config_.run || t - differences_.front().first > config_.max_age) {
    differences_.pop_front();
  }
  if (static_cast<int>(differences_.size()) < config_.run) return std::nullopt;
  // Around the first one, so that a run across ±pi averages right.
  const double first = differences_.front().second;
  double low = 0.0, high = 0.0, sum = 0.0;
  for (const auto& entry : differences_) {
    const double relative = WrapAngle(entry.second - first);
    low = std::min(low, relative);
    high = std::max(high, relative);
    sum += relative;
  }
  if (high - low > config_.agreement) return std::nullopt;
  return WrapAngle(first + sum / config_.run);
}

CourseTrack::CourseTrack(const CourseConfig& config) : config_(config) {
}

void CourseTrack::Add(const Eigen::Vector2d& antenna, double t) {
  if (!antenna.allFinite() || !std::isfinite(t)) return;
  if (!fixes_.empty() && t < fixes_.back().first) fixes_.clear();
  fixes_.emplace_back(t, antenna);
  while (t - fixes_.front().first > config_.track_window) fixes_.pop_front();
}

TrackCheck CourseTrack::Check(const Eigen::Vector2d& velocity) const {
  if (fixes_.size() < 2 || !velocity.allFinite()) return TrackCheck::kNoTrack;
  const Eigen::Vector2d& last = fixes_.back().second;
  for (auto it = std::next(fixes_.rbegin()); it != fixes_.rend(); ++it) {
    const Eigen::Vector2d chord = last - it->second;
    if (chord.norm() < config_.track_distance) continue;
    const double angle = std::atan2(chord(0) * velocity(1) - chord(1) * velocity(0), chord.dot(velocity));
    return std::abs(angle) > config_.track_agreement ? TrackCheck::kContradicts : TrackCheck::kAgrees;
  }
  return TrackCheck::kNoTrack;
}

}  // namespace mowbite_localization
