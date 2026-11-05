#include "mowbite_localization/gyro_bias.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mowbite_localization {

GyroBias::GyroBias(const GyroBiasConfig& config)
    : config_(config),
      calibrated_(config.skip_calibration),
      offset_(config.skip_calibration ? std::clamp(config.initial_offset, -config.offset_limit, config.offset_limit)
                                      : 0.0) {
  if (config.skip_calibration && offset_ != config.initial_offset) {
    char text[160];
    std::snprintf(text, sizeof(text), "gyro offset %.4f rad/s from the file is beyond the limit %.4f: taking %.4f",
                  config.initial_offset, config.offset_limit, offset_);
    warnings_.push_back(text);
  }
}

std::vector<std::string> GyroBias::TakeWarnings() {
  std::vector<std::string> out;
  out.swap(warnings_);
  return out;
}

void GyroBias::Add(double rate, double dt, bool wheels_still) {
  if (!std::isfinite(rate) || !std::isfinite(dt) || dt <= 0.0) return;
  if (!calibrated_) {
    // Only an unbroken stretch at rest counts: moving wheels start the calibration again.
    if (!wheels_still) {
      sum_ = time_ = 0.0;
      return;
    }
    min_ = time_ > 0.0 ? std::min(min_, rate) : rate;
    max_ = time_ > 0.0 ? std::max(max_, rate) : rate;
    sum_ += rate * dt;
    time_ += dt;
    if (time_ >= config_.calibration_time - 1e-9) {
      const double mean = sum_ / time_;
      const double spread = std::max(max_ - mean, mean - min_);
      char text[200];
      if (spread > config_.still_spread) {
        std::snprintf(text, sizeof(text), "gyro calibration: rate %.4f to %.4f rad/s over %.1f s, more than %.4f off "
                      "its mean %.4f: the mower did not stand, new window", min_, max_, time_, config_.still_spread,
                      mean);
        warnings_.push_back(text);
      } else if (std::abs(mean) > config_.offset_limit) {
        std::snprintf(text, sizeof(text), "gyro calibration: offset %.4f rad/s above the limit %.4f (spread %.4f): "
                      "new window", mean, config_.offset_limit, spread);
        warnings_.push_back(text);
      } else {
        offset_ = mean;
        calibrated_ = true;
      }
      sum_ = time_ = 0.0;
    }
    return;
  }
  if (wheels_still && std::abs(rate - offset_) < config_.still_rate) {
    still_for_ += dt;
  } else {
    still_for_ = 0.0;
  }
  if (holding()) {
    offset_ += (rate - offset_) * std::min(1.0, dt / config_.tracking_time);
    offset_ = std::clamp(offset_, -config_.offset_limit, config_.offset_limit);
  }
}

double GyroBias::Corrected(double rate) const {
  return holding() ? 0.0 : rate - offset_;
}

}  // namespace mowbite_localization
