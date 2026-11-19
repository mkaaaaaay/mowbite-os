#ifndef MOWBITE_LOCALIZATION_ESTIMATOR_JOURNAL_H
#define MOWBITE_LOCALIZATION_ESTIMATOR_JOURNAL_H

#include <mowbite_journal/journaled.h>

#include "mowbite_localization/estimator.h"

// The journal of the Estimator (stream "localization"). A new field of a struct below needs its place in its line,
// the size check makes the build say so.

namespace mowbite_localization {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GyroBiasConfig, calibration_time, still_spread, offset_limit, skip_calibration,
                                   initial_offset, still_time, still_rate, tracking_time)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(FilterNoise, odometry, gyro)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GateConfig, max_accuracy, restart_gap, warmup_fixes, outlier_distance, outlier_reset)
// With defaults: journals before the track check have none of its fields.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CourseConfig, min_speed, speed_tolerance, speed_ratio, run, agreement,
                                                max_age, velocity_noise, track_window, track_distance, track_agreement)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AccuracyConfig, growth_per_m, growth_per_s, timeout)
// With defaults: journals before disturbed_heading_sigma, antenna_height and stale_wheel_speed have none.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(EstimatorConfig, gyro, noise, gate, course, accuracy, max_imu_gap,
                                                wheel_timeout, min_speed, motion_confirm_time, gnss_latency,
                                                heading_reset_angle, contradiction_chi2, contradiction_reset,
                                                steady_turn_rate, pose_variance, pose_heading_variance,
                                                set_heading_variance, disturbed_heading_sigma, antenna_height,
                                                stale_wheel_speed)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GnssFix, solution, accuracy, x, y, motion_valid, motion_x, motion_y)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Pose2, x, y, heading)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Jump, reason, from_x, from_y, to_x, to_y)

static_assert(sizeof(GyroBiasConfig) == 64);
static_assert(sizeof(FilterNoise) == 16);
static_assert(sizeof(GateConfig) == 40);
static_assert(sizeof(CourseConfig) == 80);
static_assert(sizeof(AccuracyConfig) == 24);
static_assert(sizeof(EstimatorConfig) == 344);
static_assert(sizeof(GnssFix) == 56);
static_assert(sizeof(Pose2) == 24);
static_assert(sizeof(Jump) == 40);

// What the node acts on besides the pose.
struct EstimatorView {
  bool calibrated;
  bool heading_valid;
  bool gps_valid;
  bool geometry_known;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EstimatorView, calibrated, heading_valid, gps_valid, geometry_known)

}  // namespace mowbite_localization

template <>
struct mowbite_journal::CoreTraits<mowbite_localization::Estimator> {
  using E = mowbite_localization::Estimator;
  static constexpr const char* kName = "localization";
  using Config = mowbite_localization::EstimatorConfig;
  MOWBITE_JOURNAL_METHODS(&E::AddImu, &E::AddWheels, &E::SetWheelScale, &E::AddFix, &E::SetGpsEnabled,
                          &E::SetGpsLimits, &E::SetAntennaOffset, &E::SetPose, &E::SetHeading, &E::TakeJump,
                          &E::TakeAligned, &E::TakeGyroWarnings, &E::TakeCourseRejections, &E::LoosenHeading,
                          &E::AddTilt, &E::SetWheelDistance);
  static mowbite_localization::EstimatorView View(const E& core) {
    return {core.calibrated(), core.heading_valid(), core.gps_valid(), core.geometry_known()};
  }
};
MOWBITE_JOURNAL_POD(mowbite_localization::EstimatorView);

#endif  // MOWBITE_LOCALIZATION_ESTIMATOR_JOURNAL_H
