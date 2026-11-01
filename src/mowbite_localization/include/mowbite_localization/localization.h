// 2D localization from gyro, wheel speed and RTK GPS. Based on xbot_positioning.cpp and xbot_positioning_core.cpp
// (OpenMower), MIT License, Copyright (c) 2022 Clemens Elflein. See LICENSE.xbot_positioning.
#ifndef MOWBITE_LOCALIZATION_LOCALIZATION_H
#define MOWBITE_LOCALIZATION_LOCALIZATION_H

#include <array>
#include <cstdint>
#include <kalman/ExtendedKalmanFilter.hpp>
#include <limits>
#include <optional>

#include "mowbite_localization/models.h"

namespace mowbite_localization {

// Flag of mowbite_msgs/AbsolutePose for an RTK fixed solution.
constexpr uint16_t kFlagGpsRtkFixed = 2;
// Accuracy reported without a usable GPS fix.
constexpr double kNoAccuracy = 999.0;

struct LocalizationConfig {
  // The robot has to stand still for this long after the start to measure the gyro offset.
  double gyro_calibration_s = 5.0;
  bool skip_gyro_calibration = false;
  // Used instead of the measured offset when the calibration is skipped.
  double gyro_offset = 0.0;
  // Motion vectors below this speed do not correct the heading.
  double min_speed = 0.01;
  // Fixes with a worse accuracy are dropped.
  double max_position_accuracy = 0.1;
  // Without a usable fix for this long the position counts as lost (gps_timeout of the logic).
  double gps_timeout_s = 10.0;
  // GPS antenna relative to the point of rotation, x forward, y left.
  double antenna_offset_x = 0.0;
  double antenna_offset_y = 0.0;
  // Variance added to every state per prediction step (one per gyro sample), as in xbot_positioning.
  double process_noise = 1.0;
  // Wheel speed older than this counts as zero.
  double wheel_timeout_s = 0.5;
  // Gyro samples further apart than this restart the integration instead of predicting over the gap.
  double max_imu_gap_s = 0.5;
  // Without a fix the accuracy grows by this much per m driven and per s. Assumptions until measured on the device.
  double accuracy_growth_per_m = 0.05;
  double accuracy_growth_per_s = 0.01;
};

enum class GpsResult {
  kDisabled,      // GPS switched off by the logic
  kNotFixed,      // no RTK fixed solution
  kInaccurate,    // accuracy above max_position_accuracy
  kRestarted,     // first fix or more than 5 s since the last one, counting starts again
  kWarmup,        // inlier, waiting for more than 10 in a row
  kInitialized,   // filter moved to the GPS position
  kFused,         // regular update
  kOutlier,       // more than 5 m from the last fix
  kOutlierReset,  // more than 10 outliers in a row, the fix is taken as the new reference
};

struct GpsFix {
  uint16_t flags = 0;
  double accuracy = 0.0;
  double x = 0.0, y = 0.0, z = 0.0;
  bool motion_valid = false;
  double motion_x = 0.0, motion_y = 0.0;
};

// The estimate moved onto GPS again by more than max_position_accuracy.
struct PositionJump {
  bool after_outliers = false;  // otherwise after a gap without fixes
  double from_x = 0.0, from_y = 0.0;
  double to_x = 0.0, to_y = 0.0;
  double distance() const;
};

struct Position {
  double x = 0.0, y = 0.0;
};

struct Estimate {
  double x = 0.0, y = 0.0, theta = 0.0, vx = 0.0, vr = 0.0;
  // Row-major covariance of x, y, theta.
  std::array<double, 9> pose_covariance{};
  double vx_variance = 0.0, vr_variance = 0.0;
};

// Wraps an angle to (-pi, pi].
double NormalizeAngle(double angle);
// Yaw of a quaternion (x, y, z, w).
double YawFromQuaternion(double x, double y, double z, double w);

class Localization {
 public:
  explicit Localization(const LocalizationConfig& config);

  // Gyro rate about z in rad/s. stamp_s is the sample time, now_s a monotonic clock. Returns true when the
  // estimate was advanced, i.e. after the gyro calibration.
  bool AddImu(double stamp_s, double gyro_z, double now_s);
  // Measured forward speed of the wheels in m/s.
  void AddWheelSpeed(double vx, double now_s);
  GpsResult AddGps(const GpsFix& fix, double now_s);

  void SetGpsEnabled(bool enabled) {
    gps_enabled_ = enabled;
  }
  // The GPS settings of the logic, taken from the next fix and the next accuracy on.
  void SetGpsLimits(double max_position_accuracy, double gps_timeout_s) {
    config_.max_position_accuracy = max_position_accuracy;
    config_.gps_timeout_s = gps_timeout_s;
  }
  // Restarts the filter at this pose with zero speed. The caller knows the heading, e.g. from the dock, so it counts
  // as aligned.
  void SetPose(double x, double y, double theta);
  // Restarts the filter facing theta with base_link where the last fix and the antenna offset put it for that heading,
  // and returns that position. Nothing changes without a recent fix the filter has taken (gps_valid).
  std::optional<Position> SetHeading(double theta, double now_s);
  // Another antenna offset (the dimensions of the mower changed). The models take it at once; with a recent fix the
  // filter restarts where that fix and the new offset put base_link for its heading, which keeps its validity. Without
  // one the next fix corrects the position.
  void SetAntennaOffset(double x, double y, double now_s);

  Estimate GetEstimate() const;
  // Accuracy of the last fix, growing with distance and time without a fix. kNoAccuracy before the filter took GPS
  // or gps_timeout_s after the last fix.
  double PositionAccuracy(double now_s) const;
  // True within gps_timeout_s after the last usable fix.
  bool RecentGps(double now_s) const;
  // Antenna velocity the filter expects, compared with the GPS motion vector.
  MotionMeasurement<double> ExpectedMotion() const;
  // The jump of the last re-initialization onto GPS, once; nothing for the first one.
  std::optional<PositionJump> TakeJump();

  bool calibrated() const {
    return calibrated_;
  }
  double gyro_offset() const {
    return gyro_offset_;
  }
  bool gps_enabled() const {
    return gps_enabled_;
  }
  bool gps_valid() const {
    return has_gps_;
  }
  // False until the GPS course and the motion the filter expects agree several times in a row while driving, or
  // until SetPose. The heading of the filter is arbitrary before.
  bool heading_valid() const {
    return heading_valid_;
  }

 private:
  using StateT = State<double>;

  void Reset(double x, double y, double theta);
  bool Calibrate(double stamp_s, double gyro_z, double now_s);
  void CheckHeading(double vx, double vy);
  double WheelSpeed(double now_s) const;
  void UpdatePosition(double x, double y, double variance);
  void UpdateMotion(double vx, double vy, double variance);
  void KeepAngleWrapped();

  LocalizationConfig config_;
  Kalman::ExtendedKalmanFilter<StateT> ekf_;
  SystemModel<double> system_;
  PositionMeasurementModel<double> position_model_;
  SpeedMeasurementModel<double> speed_model_;
  MotionMeasurementModel<double> motion_model_;

  bool calibrated_ = false;
  double gyro_offset_ = 0.0;
  double calibration_start_s_ = 0.0;
  double calibration_sum_ = 0.0;
  int calibration_samples_ = 0;
  bool has_imu_stamp_ = false;
  double last_imu_stamp_s_ = 0.0;

  double wheel_vx_ = 0.0;
  double wheel_time_s_ = -std::numeric_limits<double>::infinity();

  bool gps_enabled_ = true;
  bool has_gps_ = false;
  GpsFix last_gps_;
  double last_gps_time_s_ = -std::numeric_limits<double>::infinity();
  int valid_gps_samples_ = 0;
  int gps_outliers_ = 0;
  double distance_since_fix_ = 0.0;
  bool initialized_once_ = false;
  bool reset_by_outliers_ = false;
  std::optional<PositionJump> jump_;

  bool heading_valid_ = false;
  int heading_agreements_ = 0;
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_LOCALIZATION_H
