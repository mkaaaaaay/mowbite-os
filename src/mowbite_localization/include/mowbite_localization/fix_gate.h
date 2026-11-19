#ifndef MOWBITE_LOCALIZATION_FIX_GATE_H
#define MOWBITE_LOCALIZATION_FIX_GATE_H

#include <Eigen/Core>
#include <cstdint>
#include <optional>

namespace mowbite_localization {

// Kinds of solution as numbered in mowbite_msgs/GnssFix; only RTK fixed counts.
constexpr uint8_t kSolutionRtkFloat = 3;
constexpr uint8_t kSolutionRtkFixed = 4;

// A GNSS position of the antenna in the map frame, with the velocity of the antenna if the receiver gave one.
struct GnssFix {
  uint8_t solution = 0;
  double accuracy = 0.0;
  double x = 0.0;
  double y = 0.0;
  bool motion_valid = false;
  double motion_x = 0.0;
  double motion_y = 0.0;
  // s the motion is the mean over, up to this fix (a chord of earlier positions); 0 for a momentary one.
  double motion_window = 0.0;
};

enum class FixResult {
  kNotReady,      // the gyro is not calibrated yet
  kDisabled,      // GNSS switched off (set_gps_state)
  kNoGeometry,    // the antenna offset is not known yet
  kNotFixed,      // not RTK fixed
  kInaccurate,    // accuracy worse than the limit, or NaN
  kRestarted,     // first fix of a warmup
  kWarmup,        // agrees with the warmup so far
  kInitialized,   // ends the warmup: the estimate was set onto GNSS
  kFused,         // corrected the estimate
  kOutlier,       // too far from the estimate, dropped
  kOutlierReset,  // one outlier too many: a new warmup starts with it
  // Set by the estimator, which knows its covariance:
  kContradicting,      // nearer than an outlier but too far for the covariance of the estimate, dropped
  kContradictionReset, // one contradicting fix too many: the estimate was set onto it
};

// Why GNSS sets the estimate again at the end of a warmup.
enum class WarmupReason { kFirst, kGap, kOutliers };

struct GateConfig {
  // RTK fixed with this accuracy (m) or better counts.
  double max_accuracy = 0.1;
  // The first fix and the first after restart_gap s without one start a warmup; warmup_fixes more that agree end it.
  double restart_gap = 5.0;
  int warmup_fixes = 10;
  // A fix this far (m) from where the estimate puts the antenna is an outlier; outlier_reset in a row start a warmup.
  double outlier_distance = 5.0;
  int outlier_reset = 11;
};

// Which fixes count and what each does: a warmup before GNSS sets the estimate, then fusion with outliers dropped.
class FixGate {
 public:
  explicit FixGate(const GateConfig& config);

  // Why the fix cannot count at all, nothing if it can.
  std::optional<FixResult> Reject(const GnssFix& fix) const;
  // What a fix that can count does at time t; expected is where the estimate puts the antenna.
  FixResult Classify(double t, const Eigen::Vector2d& antenna, const Eigen::Vector2d& expected);
  // The next warmup is a first one again, e.g. after the pose was set by hand.
  void Forget();

  void SetMaxAccuracy(double max_accuracy) { config_.max_accuracy = max_accuracy; }
  // GNSS sets the estimate, it is not warming up.
  bool initialized() const { return initialized_; }
  bool warming_up() const { return warming_up_; }
  WarmupReason reason() const { return reason_; }
  // Time of the last fix that was no outlier.
  std::optional<double> last_accepted() const { return last_accepted_; }

 private:
  FixResult StartWarmup(double t, const Eigen::Vector2d& antenna, WarmupReason reason);

  GateConfig config_;
  bool initialized_ = false;
  bool warming_up_ = false;
  bool ever_initialized_ = false;
  WarmupReason reason_ = WarmupReason::kFirst;
  int warmup_count_ = 0;
  int outliers_ = 0;
  Eigen::Vector2d warmup_last_ = Eigen::Vector2d::Zero();
  std::optional<double> last_accepted_;
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_FIX_GATE_H
