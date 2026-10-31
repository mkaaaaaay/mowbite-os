// Based on xbot_positioning.cpp and xbot_positioning_core.cpp (OpenMower), MIT License, Copyright (c) 2022 Clemens
// Elflein. See LICENSE.xbot_positioning.
#include "mowbite_localization/localization.h"

#include <algorithm>
#include <cmath>

namespace mowbite_localization {

namespace {

// Gating and variances as tuned in xbot_positioning.
constexpr double kGpsGapS = 5.0;
constexpr double kOutlierDistanceM = 5.0;
constexpr int kInliersBeforeInit = 10;
constexpr int kOutliersBeforeReset = 10;
constexpr double kInitVariance = 0.001;
constexpr double kPositionVariance = 500.0;
constexpr double kMotionVariance = 10000.0;

// The heading counts as aligned after this many motion vectors in a row within kAlignMaxError of the motion the
// filter expects, both at least kAlignMinSpeed.
constexpr int kAlignSamples = 5;
constexpr double kAlignMaxError = 10.0 * M_PI / 180.0;
constexpr double kAlignMinSpeed = 0.1;

}  // namespace

double NormalizeAngle(double angle) {
  double a = std::remainder(angle, 2.0 * M_PI);
  if (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

double YawFromQuaternion(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

Localization::Localization(const LocalizationConfig& config) : config_(config) {
  Kalman::Covariance<StateT> q;
  q.setIdentity();
  q *= config_.process_noise;
  system_.setCovariance(q);
  // The speed measurement keeps the identity covariance of the original.
  Kalman::Covariance<SpeedMeasurement<double>> r;
  r.setIdentity();
  speed_model_.setCovariance(r);
  position_model_.antenna_offset_x = motion_model_.antenna_offset_x = config_.antenna_offset_x;
  position_model_.antenna_offset_y = motion_model_.antenna_offset_y = config_.antenna_offset_y;
  Reset(0.0, 0.0, 0.0);
}

bool Localization::AddImu(double stamp_s, double gyro_z, double now_s) {
  if (!std::isfinite(stamp_s) || !std::isfinite(gyro_z)) return false;
  if (!calibrated_) {
    if (config_.skip_gyro_calibration) {
      gyro_offset_ = config_.gyro_offset;
      calibrated_ = true;
    } else if (!Calibrate(stamp_s, gyro_z, now_s)) {
      last_imu_stamp_s_ = stamp_s;
      has_imu_stamp_ = true;
      return false;
    }
  }

  const double dt = stamp_s - last_imu_stamp_s_;
  const bool can_predict = has_imu_stamp_ && dt > 0.0 && dt <= config_.max_imu_gap_s;
  last_imu_stamp_s_ = stamp_s;
  has_imu_stamp_ = true;
  if (!can_predict) return false;

  const double rate = gyro_z - gyro_offset_;
  const double vx = WheelSpeed(now_s);
  distance_since_fix_ += std::abs(vx) * dt;
  system_.SetDt(dt);
  Control<double> u;
  u.v() = vx;
  u.dtheta() = rate;
  ekf_.predict(system_, u);
  SpeedMeasurement<double> z;
  z.vx() = vx;
  z.vr() = rate;
  ekf_.update(speed_model_, z);
  KeepAngleWrapped();
  return true;
}

bool Localization::Calibrate(double stamp_s, double gyro_z, double now_s) {
  // Moving wheels would bias the offset, so the calibration starts again.
  if (std::abs(WheelSpeed(now_s)) >= config_.min_speed) {
    calibration_samples_ = 0;
    return false;
  }
  if (calibration_samples_ == 0) {
    calibration_start_s_ = stamp_s;
    calibration_sum_ = 0.0;
  }
  calibration_sum_ += gyro_z;
  calibration_samples_++;
  if (stamp_s - calibration_start_s_ < config_.gyro_calibration_s) return false;
  gyro_offset_ = calibration_sum_ / calibration_samples_;
  calibration_samples_ = 0;
  calibrated_ = true;
  return true;
}

void Localization::AddWheelSpeed(double vx, double now_s) {
  if (!std::isfinite(vx)) return;
  wheel_vx_ = vx;
  wheel_time_s_ = now_s;
}

double Localization::WheelSpeed(double now_s) const {
  return now_s - wheel_time_s_ <= config_.wheel_timeout_s ? wheel_vx_ : 0.0;
}

GpsResult Localization::AddGps(const GpsFix& fix, double now_s) {
  if (!gps_enabled_) return GpsResult::kDisabled;
  if ((fix.flags & kFlagGpsRtkFixed) == 0) return GpsResult::kNotFixed;
  if (!(fix.accuracy <= config_.max_position_accuracy)) return GpsResult::kInaccurate;
  if (!std::isfinite(fix.x) || !std::isfinite(fix.y) || !std::isfinite(fix.z)) return GpsResult::kInaccurate;

  if (now_s - last_gps_time_s_ > kGpsGapS) {
    reset_by_outliers_ = false;
    has_gps_ = false;
    valid_gps_samples_ = 0;
    gps_outliers_ = 0;
    last_gps_ = fix;
    last_gps_time_s_ = now_s;
    return GpsResult::kRestarted;
  }

  const double dx = fix.x - last_gps_.x, dy = fix.y - last_gps_.y, dz = fix.z - last_gps_.z;
  if (std::sqrt(dx * dx + dy * dy + dz * dz) >= kOutlierDistanceM) {
    if (++gps_outliers_ <= kOutliersBeforeReset) return GpsResult::kOutlier;
    last_gps_ = fix;
    last_gps_time_s_ = now_s;
    has_gps_ = false;
    valid_gps_samples_ = 0;
    gps_outliers_ = 0;
    reset_by_outliers_ = true;
    return GpsResult::kOutlierReset;
  }

  last_gps_ = fix;
  last_gps_time_s_ = now_s;
  gps_outliers_ = 0;
  valid_gps_samples_++;
  if (!has_gps_) {
    if (valid_gps_samples_ <= kInliersBeforeInit) return GpsResult::kWarmup;
    const Estimate before = GetEstimate();
    UpdatePosition(fix.x, fix.y, kInitVariance);
    has_gps_ = true;
    const Estimate after = GetEstimate();
    PositionJump jump{reset_by_outliers_, before.x, before.y, after.x, after.y};
    if (initialized_once_ && jump.distance() > config_.max_position_accuracy) jump_ = jump;
    initialized_once_ = true;
    return GpsResult::kInitialized;
  }
  UpdatePosition(fix.x, fix.y, kPositionVariance);
  if (fix.motion_valid && std::hypot(fix.motion_x, fix.motion_y) >= config_.min_speed) {
    CheckHeading(fix.motion_x, fix.motion_y);
    UpdateMotion(fix.motion_x, fix.motion_y, kMotionVariance);
  }
  return GpsResult::kFused;
}

void Localization::CheckHeading(double vx, double vy) {
  const auto expected = ExpectedMotion();
  // Slow motion gives no usable course, it neither counts nor breaks a run.
  if (std::hypot(vx, vy) < kAlignMinSpeed || std::hypot(expected.x(), expected.y()) < kAlignMinSpeed) return;
  const double error = NormalizeAngle(std::atan2(vy, vx) - std::atan2(expected.y(), expected.x()));
  heading_agreements_ = std::abs(error) <= kAlignMaxError ? heading_agreements_ + 1 : 0;
  if (heading_agreements_ >= kAlignSamples) heading_valid_ = true;
}

void Localization::SetPose(double x, double y, double theta) {
  Reset(x, y, theta);
  heading_valid_ = true;
}

std::optional<Position> Localization::SetHeading(double theta, double now_s) {
  if (!has_gps_ || !RecentGps(now_s) || !std::isfinite(theta)) return std::nullopt;
  const double c = std::cos(theta), s = std::sin(theta);
  Position base;
  base.x = last_gps_.x - (c * config_.antenna_offset_x - s * config_.antenna_offset_y);
  base.y = last_gps_.y - (s * config_.antenna_offset_x + c * config_.antenna_offset_y);
  SetPose(base.x, base.y, theta);
  return base;
}

void Localization::Reset(double x, double y, double theta) {
  StateT s;
  s.setZero();
  s.x() = x;
  s.y() = y;
  s.theta() = NormalizeAngle(theta);
  ekf_.init(s);
  Kalman::Covariance<StateT> p;
  p.setIdentity();
  ekf_.setCovariance(p);
}

void Localization::UpdatePosition(double x, double y, double variance) {
  Kalman::Covariance<PositionMeasurement<double>> r;
  r.setIdentity();
  r *= variance;
  position_model_.setCovariance(r);
  PositionMeasurement<double> z;
  z.x() = x;
  z.y() = y;
  ekf_.update(position_model_, z);
  KeepAngleWrapped();
  distance_since_fix_ = 0.0;
}

void Localization::UpdateMotion(double vx, double vy, double variance) {
  Kalman::Covariance<MotionMeasurement<double>> r;
  r.setIdentity();
  r *= variance;
  motion_model_.setCovariance(r);
  MotionMeasurement<double> z;
  z.x() = vx;
  z.y() = vy;
  ekf_.update(motion_model_, z);
  KeepAngleWrapped();
}

void Localization::KeepAngleWrapped() {
  StateT s = ekf_.getState();
  s.theta() = NormalizeAngle(s.theta());
  ekf_.init(s);
}

Estimate Localization::GetEstimate() const {
  const StateT& s = ekf_.getState();
  const auto& p = ekf_.getCovariance();
  Estimate e;
  e.x = s.x();
  e.y = s.y();
  e.theta = s.theta();
  e.vx = s.vx();
  e.vr = s.vr();
  constexpr size_t kPose[3] = {StateT::X, StateT::Y, StateT::THETA};
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) e.pose_covariance[i * 3 + j] = p(kPose[i], kPose[j]);
  }
  e.vx_variance = p(StateT::VX, StateT::VX);
  e.vr_variance = p(StateT::VR, StateT::VR);
  return e;
}

bool Localization::RecentGps(double now_s) const {
  return now_s - last_gps_time_s_ < config_.gps_timeout_s;
}

double Localization::PositionAccuracy(double now_s) const {
  if (!has_gps_ || !RecentGps(now_s)) return kNoAccuracy;
  return last_gps_.accuracy + config_.accuracy_growth_per_m * distance_since_fix_ +
         config_.accuracy_growth_per_s * std::max(0.0, now_s - last_gps_time_s_);
}

double PositionJump::distance() const {
  return std::hypot(to_x - from_x, to_y - from_y);
}

std::optional<PositionJump> Localization::TakeJump() {
  std::optional<PositionJump> out;
  out.swap(jump_);
  return out;
}

MotionMeasurement<double> Localization::ExpectedMotion() const {
  return motion_model_.h(ekf_.getState());
}

}  // namespace mowbite_localization
