#ifndef MOWBITE_LOCALIZATION_GYRO_BIAS_H
#define MOWBITE_LOCALIZATION_GYRO_BIAS_H

#include <cmath>

namespace mowbite_localization {

struct GyroBiasConfig {
  // The first offset is the mean rate over calibration_time s of samples with the wheels standing; with
  // skip_calibration it is initial_offset (rad/s) from the start.
  double calibration_time = 5.0;
  bool skip_calibration = false;
  double initial_offset = 0.0;
  // Standing: the wheels still for still_time s and the corrected rate below still_rate rad/s. Then the heading is
  // held and the offset follows the gyro with the time constant tracking_time s.
  double still_time = 1.0;
  double still_rate = 2.0 * M_PI / 180.0;
  double tracking_time = 10.0;
};

// The yaw rate offset of the gyro: measured at rest before anything is integrated, then followed whenever the mower
// stands, e.g. while the sensor warms up.
class GyroBias {
 public:
  explicit GyroBias(const GyroBiasConfig& config);

  // A sample of rate (rad/s) that covers dt s; wheels_still: the wheels report no motion. NaN is ignored.
  void Add(double rate, double dt, bool wheels_still);
  // The rate the mower turns at: rate minus the offset, 0 while it stands.
  double Corrected(double rate) const;

  bool calibrated() const { return calibrated_; }
  bool holding() const { return calibrated_ && still_for_ >= config_.still_time; }
  double offset() const { return offset_; }

 private:
  GyroBiasConfig config_;
  bool calibrated_;
  double offset_;
  // The calibration so far, and how long the mower has stood since it was calibrated.
  double sum_ = 0.0;
  double time_ = 0.0;
  double still_for_ = 0.0;
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_GYRO_BIAS_H
