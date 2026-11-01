#include "mowbite_localization/gyro_bias.h"

#include <algorithm>
#include <cmath>

namespace mowbite_localization {

GyroBias::GyroBias(const GyroBiasConfig& config)
    : config_(config),
      calibrated_(config.skip_calibration),
      offset_(config.skip_calibration ? config.initial_offset : 0.0) {
}

void GyroBias::Add(double rate, double dt, bool wheels_still) {
  if (!std::isfinite(rate) || !std::isfinite(dt) || dt <= 0.0) return;
  if (!calibrated_) {
    // Only an unbroken stretch at rest counts: moving wheels start the calibration again.
    if (!wheels_still) {
      sum_ = time_ = 0.0;
      return;
    }
    sum_ += rate * dt;
    time_ += dt;
    if (time_ >= config_.calibration_time - 1e-9) {
      offset_ = sum_ / time_;
      calibrated_ = true;
    }
    return;
  }
  if (wheels_still && std::abs(rate - offset_) < config_.still_rate) {
    still_for_ += dt;
  } else {
    still_for_ = 0.0;
  }
  if (holding()) offset_ += (rate - offset_) * std::min(1.0, dt / config_.tracking_time);
}

double GyroBias::Corrected(double rate) const {
  return holding() ? 0.0 : rate - offset_;
}

}  // namespace mowbite_localization
