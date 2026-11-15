#include "mowbite_localization/dock_estimator.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <vector>

#include "mowbite_localization/position_accuracy.h"

namespace mowbite_localization {

namespace {

// Clock skew between the camera and the motions that still counts as taken now.
constexpr double kFutureSlack = 0.05;

Eigen::Matrix2d Rotation(double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Eigen::Matrix2d r;
  r << c, -s, s, c;
  return r;
}

Eigen::Matrix2d PositionCovariance(const MarkerImage& image) {
  Eigen::Matrix2d covariance;
  covariance << image.covariance[0], image.covariance[1], image.covariance[3], image.covariance[4];
  return 0.5 * (covariance + covariance.transpose());
}

// Two images of the start that disagree beyond their spread: something moved or one is an outlier.
bool Disagree(const MarkerImage& a, const MarkerImage& b, const DockConfig& config) {
  const double spread =
      std::sqrt(std::max(a.covariance[0], a.covariance[4]) + std::max(b.covariance[0], b.covariance[4]));
  if (std::hypot(a.x - b.x, a.y - b.y) > std::max(config.min_outlier, 3.0 * spread)) return true;
  if (!a.yaw_valid || !b.yaw_valid) return false;
  const double yaw_spread = std::sqrt(a.covariance[8] + b.covariance[8]);
  return std::abs(WrapAngle(a.yaw - b.yaw)) > std::max(config.min_outlier_yaw, 3.0 * yaw_spread);
}

bool Usable(const MarkerImage& image) {
  for (double value : image.covariance) {
    if (!std::isfinite(value)) return false;
  }
  if (!std::isfinite(image.time) || !std::isfinite(image.x) || !std::isfinite(image.y)) return false;
  if (!(image.covariance[0] > 0.0) || !(image.covariance[4] > 0.0)) return false;
  return !image.yaw_valid || (std::isfinite(image.yaw) && image.covariance[8] > 0.0);
}

}  // namespace

DockEstimator::DockEstimator(const DockConfig& config) : config_(config), filter_(config.noise) {
}

State DockEstimator::Marker() const {
  const double docked = docked_heading_.value_or(0.0);
  const Eigen::Vector2d at = Rotation(docked) * Eigen::Vector2d(config_.marker_x, config_.marker_y);
  return State(at(0), at(1), WrapAngle(docked + config_.marker_yaw));
}

void DockEstimator::AddMotion(const Motion& motion) {
  gyro_heading_ = WrapAngle(gyro_heading_ + motion.turn);
  way_ += std::abs(motion.distance);
  if (!motion.still) {
    still_since_.reset();
    start_.clear();
  } else if (!still_since_) {
    still_since_ = motion.time;
  }
  if (!estimate_) return;
  filter_.Predict(motion.distance, motion.turn, motion.gyro_time);
  history_.Add(Moment{motion.time, motion, filter_.state(), filter_.covariance(), way_, gyro_heading_, still_since_},
               motion.time - config_.max_age - 0.5);
}

void DockEstimator::SetHint(double heading, double t) {
  if (!std::isfinite(heading) || !std::isfinite(t)) return;
  hint_offset_ = WrapAngle(heading - gyro_heading_);
  hint_time_ = t;
}

void DockEstimator::SetDockedHeading(double heading) {
  if (!std::isfinite(heading)) return;
  heading = WrapAngle(heading);
  if (docked_heading_ && *docked_heading_ == heading) return;
  docked_heading_ = heading;
  Drop();
}

MarkerResult DockEstimator::AddMarker(const MarkerImage& image, double t) {
  if (!docked_heading_) return MarkerResult::kNoFrame;
  if (!Usable(image)) return MarkerResult::kInvalid;
  if (t - image.time > config_.max_age || image.time > t + kFutureSlack) return MarkerResult::kTooOld;
  if (!estimate_) {
    if (!still_since_ || image.time < *still_since_) {
      start_.clear();
      return MarkerResult::kCollecting;
    }
    if (!start_.empty() && Disagree(start_.back(), image, config_)) start_.clear();
    start_.push_back(image);
    while (static_cast<int>(start_.size()) > config_.start_images) start_.pop_front();
    return TryStart(t) ? MarkerResult::kStarted : MarkerResult::kCollecting;
  }

  // Images go in in the order they were taken: an older one would undo what a newer one corrected.
  if (image.time < fix_time_) return MarkerResult::kTooOld;
  const auto [taken, end] = history_.From(image.time);
  if (taken == end) return MarkerResult::kTooOld;
  const State marker = Marker();
  const State& then = taken->state;
  // The marker as base_link sees it: R(heading)^T (marker - position). The Jacobian comes from the state, not from the
  // noisy image, so that images of one place tell nothing about the heading.
  const Eigen::Vector2d to_marker = marker.head<2>() - then.head<2>();
  const Eigen::Matrix2d turned = Rotation(then(2)).transpose();
  const Eigen::Vector2d innovation = Eigen::Vector2d(image.x, image.y) - turned * to_marker;
  AntennaJacobian jacobian;
  jacobian.leftCols<2>() = -turned;
  jacobian.col(2) = Rotation(then(2) + 0.5 * M_PI).transpose() * to_marker;
  const Eigen::Matrix2d noise = PositionCovariance(image);
  std::optional<double> yaw_innovation;
  if (image.yaw_valid) yaw_innovation = WrapAngle(marker(2) - image.yaw - then(2));
  if (Outlier(taken->covariance, innovation, jacobian, noise, yaw_innovation, image.covariance[8])) {
    if (++outliers_ < config_.outlier_reset) return MarkerResult::kOutlier;
    Drop();
    return MarkerResult::kDropped;
  }
  outliers_ = 0;
  // Taken standing, the first image is the measurement of this place; the next ones only confirm it. Fused they
  // would add no knowledge but feed the error of the linearization back from image to image.
  if (!taken->rest || taken->rest != fused_rest_) {
    fused_rest_ = taken->rest;
    // Fused into the estimate of the moment it was taken, which the motions since then carry on to now.
    PoseFilter at = filter_;
    at.Reset(then, taken->covariance);
    at.UpdatePosition(innovation, jacobian, noise);
    if (yaw_innovation && YawMoved(*taken)) {
      FuseYaw(at, WrapAngle(marker(2) - image.yaw - at.state()(2)), image.covariance[8], *taken);
    }
    for (auto it = taken; it != end; ++it) {
      if (it != taken) at.Predict(it->motion.distance, it->motion.turn, it->motion.gyro_time);
      it->state = at.state();
      it->covariance = at.covariance();
    }
    filter_ = at;
  }
  fix_time_ = image.time;
  fix_way_ = taken->way;
  fix_x_ = taken->state(0);
  return MarkerResult::kFused;
}

bool DockEstimator::YawMoved(const Moment& then) const {
  return !yaw_used_ || then.way - yaw_way_ >= config_.yaw_move_way ||
         std::abs(WrapAngle(then.gyro - yaw_gyro_)) >= config_.yaw_move_turn;
}

void DockEstimator::FuseYaw(PoseFilter& filter, double innovation, double variance, const Moment& then) {
  const double spread = filter.covariance()(2, 2);
  // A heading known better than one image keeps it; else the fix leaves exactly the spread of the image.
  if (spread - variance <= 1e-12) return;
  filter.UpdateHeading(innovation, spread * variance / (spread - variance));
  yaw_used_ = true;
  yaw_way_ = then.way;
  yaw_gyro_ = then.gyro;
  yaw_fixes_++;
}

bool DockEstimator::Outlier(const Covariance& p, const Eigen::Vector2d& innovation, const AntennaJacobian& jacobian,
                            const Eigen::Matrix2d& noise, std::optional<double> yaw_innovation,
                            double yaw_variance) const {
  if (innovation.norm() > config_.min_outlier) {
    const Eigen::Matrix2d s = jacobian * p * jacobian.transpose() + noise;
    if (innovation.dot(s.ldlt().solve(innovation)) > config_.position_chi2) return true;
  }
  if (yaw_innovation && std::abs(*yaw_innovation) > config_.min_outlier_yaw) {
    const double s = p(2, 2) + yaw_variance;
    if (*yaw_innovation * *yaw_innovation / s > config_.yaw_chi2) return true;
  }
  return false;
}

bool DockEstimator::TryStart(double t) {
  const int count = static_cast<int>(start_.size());
  if (count < config_.start_images) return false;
  const State marker = Marker();
  Eigen::Vector2d lever = Eigen::Vector2d::Zero();
  Eigen::Matrix2d lever_covariance = Eigen::Matrix2d::Zero();
  // Headings around that of the first image, and the variances of the directions.
  const double first = marker(2) - start_.front().yaw;
  std::vector<double> headings;
  std::vector<double> variances;
  bool all_yaw = true;
  for (const MarkerImage& image : start_) {
    lever += Eigen::Vector2d(image.x, image.y) / count;
    lever_covariance += PositionCovariance(image) / count;
    all_yaw = all_yaw && image.yaw_valid;
    headings.push_back(WrapAngle(marker(2) - image.yaw - first));
    variances.push_back(image.covariance[8]);
  }
  double heading = 0.0;
  double heading_variance = 0.0;
  if (all_yaw) {
    // Images of one place share the error of their direction: the median, as good as one image.
    std::nth_element(headings.begin(), headings.begin() + count / 2, headings.end());
    std::nth_element(variances.begin(), variances.begin() + count / 2, variances.end());
    heading = WrapAngle(first + headings[count / 2]);
    heading_variance = variances[count / 2];
  } else if (hint_offset_ && t - hint_time_ <= config_.hint_hold) {
    heading = WrapAngle(*hint_offset_ + gyro_heading_);
    heading_variance = config_.hint_sigma * config_.hint_sigma;
  } else {
    return false;
  }
  // base_link = marker - R(heading) lever, and how that moves with the heading.
  const Eigen::Matrix2d rotation = Rotation(heading);
  const Eigen::Vector2d position = marker.head<2>() - rotation * lever;
  const Eigen::Vector2d slope = -Rotation(heading + 0.5 * M_PI) * lever;
  Covariance covariance = Covariance::Zero();
  covariance.topLeftCorner<2, 2>() = rotation * (lever_covariance / count) * rotation.transpose() +
                                     slope * slope.transpose() * heading_variance;
  covariance.block<2, 1>(0, 2) = slope * heading_variance;
  covariance.block<1, 2>(2, 0) = (slope * heading_variance).transpose();
  covariance(2, 2) = heading_variance;
  filter_.Reset(State(position(0), position(1), heading), covariance);
  estimate_ = true;
  from_hint_ = !all_yaw;
  yaw_used_ = all_yaw;
  fused_rest_ = still_since_;
  yaw_way_ = way_;
  yaw_gyro_ = gyro_heading_;
  yaw_fixes_ = 0;
  outliers_ = 0;
  fix_time_ = start_.back().time;
  fix_way_ = way_;
  fix_x_ = position(0);
  start_.clear();
  history_.Clear();
  // The mower stood since the first image of the start: the estimate holds for every image taken until now.
  history_.Add(Moment{t, Motion{}, filter_.state(), filter_.covariance(), way_, gyro_heading_, still_since_}, t);
  return true;
}

void DockEstimator::AtContacts(double t) {
  if (!std::isfinite(t) || !Good(t) || filter_.state()(0) < -config_.blind_distance - config_.good_way) return;
  contact_time_ = t;
  contact_way_ = way_;
}

void DockEstimator::Drop() {
  contact_time_.reset();
  estimate_ = false;
  from_hint_ = false;
  outliers_ = 0;
  start_.clear();
  history_.Clear();
}

Pose2 DockEstimator::pose() const {
  const State& state = filter_.state();
  return Pose2{state(0), state(1), state(2)};
}

bool DockEstimator::Good(double t) const {
  if (!estimate_) return false;
  const double way = way_ - fix_way_;
  const double age = t - fix_time_;
  if (way <= config_.good_way && age <= config_.good_time) return true;
  if (contact_time_ && way_ - contact_way_ <= config_.blind_distance + config_.good_way &&
      t - *contact_time_ <= config_.final_time) {
    return true;
  }
  const bool closing = filter_.state()(0) >= -config_.blind_distance &&
                     fix_x_ >= -config_.blind_distance - config_.good_way;
  return closing && way <= config_.final_way && age <= config_.final_time;
}

double DockEstimator::PositionAccuracy(double t) const {
  if (!Good(t)) return kNoAccuracy;
  const Covariance& p = filter_.covariance();
  const double mean = 0.5 * (p(0, 0) + p(1, 1));
  const double half = 0.5 * (p(0, 0) - p(1, 1));
  return std::sqrt(std::max(0.0, mean + std::sqrt(half * half + p(0, 1) * p(0, 1))));
}

double DockEstimator::HeadingAccuracy() const {
  return estimate_ ? std::sqrt(std::max(0.0, filter_.covariance()(2, 2))) : kNoAccuracy;
}

}  // namespace mowbite_localization
