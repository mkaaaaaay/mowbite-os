#include "mowbite_localization/fix_gate.h"

#include <cmath>

namespace mowbite_localization {

FixGate::FixGate(const GateConfig& config) : config_(config) {
}

std::optional<FixResult> FixGate::Reject(const GnssFix& fix) const {
  if ((fix.flags & kFlagRtkFixed) == 0) return FixResult::kNotFixed;
  if (!(fix.accuracy <= config_.max_accuracy) || !std::isfinite(fix.x) || !std::isfinite(fix.y)) {
    return FixResult::kInaccurate;
  }
  return std::nullopt;
}

FixResult FixGate::StartWarmup(double t, const Eigen::Vector2d& antenna, WarmupReason reason) {
  initialized_ = false;
  warming_up_ = true;
  reason_ = reason;
  warmup_count_ = 0;
  outliers_ = 0;
  warmup_last_ = antenna;
  last_accepted_ = t;
  return FixResult::kRestarted;
}

FixResult FixGate::Classify(double t, const Eigen::Vector2d& antenna, const Eigen::Vector2d& expected) {
  const bool gap = !last_accepted_ || t - *last_accepted_ >= config_.restart_gap;
  if (gap) {
    // A warmup after outliers stays one when it is cut short.
    const WarmupReason reason = !ever_initialized_                                       ? WarmupReason::kFirst
                                : warming_up_ && reason_ == WarmupReason::kOutliers ? WarmupReason::kOutliers
                                                                                     : WarmupReason::kGap;
    return StartWarmup(t, antenna, reason);
  }
  if (warming_up_) {
    // Fixes of a warmup have to agree with each other, the estimate does not count yet.
    if ((antenna - warmup_last_).norm() >= config_.outlier_distance) return StartWarmup(t, antenna, reason_);
    warmup_last_ = antenna;
    last_accepted_ = t;
    if (++warmup_count_ <= config_.warmup_fixes) return FixResult::kWarmup;
    warming_up_ = false;
    initialized_ = ever_initialized_ = true;
    return FixResult::kInitialized;
  }
  if (!initialized_) return StartWarmup(t, antenna, ever_initialized_ ? WarmupReason::kGap : WarmupReason::kFirst);
  if ((antenna - expected).norm() >= config_.outlier_distance) {
    if (++outliers_ < config_.outlier_reset) return FixResult::kOutlier;
    StartWarmup(t, antenna, WarmupReason::kOutliers);
    return FixResult::kOutlierReset;
  }
  outliers_ = 0;
  last_accepted_ = t;
  return FixResult::kFused;
}

void FixGate::Forget() {
  initialized_ = warming_up_ = ever_initialized_ = false;
  reason_ = WarmupReason::kFirst;
  warmup_count_ = outliers_ = 0;
  last_accepted_.reset();
}

}  // namespace mowbite_localization
