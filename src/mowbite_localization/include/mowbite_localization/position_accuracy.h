#ifndef MOWBITE_LOCALIZATION_POSITION_ACCURACY_H
#define MOWBITE_LOCALIZATION_POSITION_ACCURACY_H

#include <algorithm>
#include <cmath>

namespace mowbite_localization {

// The accuracy of a position nobody knows.
constexpr double kNoAccuracy = 999.0;

struct AccuracyConfig {
  // Without a fix the accuracy grows by growth_per_m per m driven (either way), growth_per_s per s and every m the
  // wheels may have slipped; after timeout s the position counts as lost.
  double growth_per_m = 0.05;
  double growth_per_s = 0.01;
  double timeout = 10.0;
};

// The accuracy of the position: that of the last fix the estimate took, worse with the way, the slip and the time
// since.
class FixAccuracy {
 public:
  explicit FixAccuracy(const AccuracyConfig& config) : config_(config) {}

  void OnFix(double accuracy, double t) {
    accuracy_ = accuracy;
    fix_time_ = t;
    distance_ = 0.0;
    slip_ = 0.0;
    known_ = true;
  }
  void OnDistance(double distance) {
    if (std::isfinite(distance)) distance_ += std::abs(distance);
  }
  // m base_link may have moved without the wheels reporting it.
  void OnSlip(double slip) {
    if (std::isfinite(slip)) slip_ += std::abs(slip);
  }
  void Forget() { known_ = false; }
  void SetTimeout(double timeout) { config_.timeout = timeout; }
  double timeout() const { return config_.timeout; }
  // m driven since the last fix.
  double distance() const { return distance_; }

  double Value(double t) const {
    if (!known_ || t - fix_time_ >= config_.timeout) return kNoAccuracy;
    return accuracy_ + config_.growth_per_m * distance_ + slip_ + config_.growth_per_s * std::max(0.0, t - fix_time_);
  }

 private:
  AccuracyConfig config_;
  bool known_ = false;
  double accuracy_ = 0.0;
  double fix_time_ = 0.0;
  double distance_ = 0.0;
  double slip_ = 0.0;
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_POSITION_ACCURACY_H
