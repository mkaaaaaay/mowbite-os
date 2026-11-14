#ifndef MOWBITE_LOCALIZATION_ESTIMATOR_H
#define MOWBITE_LOCALIZATION_ESTIMATOR_H

#include <Eigen/Core>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_localization/course_alignment.h"
#include "mowbite_localization/fix_gate.h"
#include "mowbite_localization/gyro_bias.h"
#include "mowbite_localization/pose_filter.h"
#include "mowbite_localization/position_accuracy.h"
#include "mowbite_localization/state_history.h"

namespace mowbite_localization {

struct EstimatorConfig {
  GyroBiasConfig gyro;
  FilterNoise noise;
  GateConfig gate;
  CourseConfig course;
  AccuracyConfig accuracy;
  // Gyro samples further apart than max_imu_gap s are not integrated; a wheel speed older than wheel_timeout s is 0.
  double max_imu_gap = 0.5;
  double wheel_timeout = 0.5;
  // Wheels slower than min_speed (m/s, and rad/s for their turn) stand. They count as moving only after
  // motion_confirm_time s above it, so single noise samples of a standing mower do not end a still window.
  double min_speed = 0.01;
  double motion_confirm_time = 0.0;
  // How old (s) a GNSS position and velocity are when they arrive.
  double gnss_latency = 0.0;
  // A valid heading that run courses in a row contradict by more than this (rad) and by more than three standard
  // deviations is set from them; a single course that far off is not fused.
  double heading_reset_angle = 5.0 * M_PI / 180.0;
  // A fix further from the expected antenna position than its covariance allows (chi-square of two axes above
  // contradiction_chi2, 99.9 %) and by more than max_position_accuracy is not fused; contradiction_reset of them in a
  // row set the position onto GNSS, e.g. after the mower was carried less than an outlier's distance.
  double contradiction_chi2 = 13.8;
  int contradiction_reset = 5;
  // While the gyro turns faster than this (rad/s), GNSS corrects the heading neither by a fix nor by a single course: a
  // fix later than gnss_latency lies behind the turn and would hold the heading back. Runs of courses still align an
  // unknown or contradicted heading.
  double steady_turn_rate = 0.05;
  // Variances after set_pose (m², rad²) and of a heading given by set_heading (rad²).
  double pose_variance = 1.0;
  double pose_heading_variance = 0.0025;
  double set_heading_variance = 0.0004;
};

// base_link in the map frame: m, and rad counterclockwise from east.
struct Pose2 {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
};

// GNSS set the estimate again, after a gap or a run of outliers, and moved it from one place to the other.
struct Jump {
  WarmupReason reason = WarmupReason::kGap;
  double from_x = 0.0;
  double from_y = 0.0;
  double to_x = 0.0;
  double to_y = 0.0;
  double distance() const;
};

// What one gyro sample moved the estimate by, so that a second estimate on the same gyro and wheels (DockEstimator)
// moves the same way: m along the mean heading, rad, and the s of gyro noise. still: the gyro offset holds, the mower
// stands.
struct Motion {
  double time = 0.0;
  double distance = 0.0;
  double turn = 0.0;
  double gyro_time = 0.0;
  bool still = false;
};

// The localization without ROS: gyro and wheel speed move the pose of base_link, RTK fixes of the antenna and its
// course correct it. Without a valid heading the position comes from the fixes alone. Times are seconds of one
// monotonic clock.
class Estimator {
 public:
  explicit Estimator(const EstimatorConfig& config);

  // A yaw rate (rad/s) at t; true when it moved the estimate, from the sample after the gyro calibration on.
  bool AddImu(double t, double rate);
  // Wheel speed (m/s) and turn rate (rad/s) at t, as the driver computed them; the wheel scale corrects both.
  void AddWheels(double t, double speed, double turn_rate);
  // The ticks per m the driver computes with against those of robot.geometry: the driver's speeds count times
  // driver/geometry. Values that are not positive leave the scale at 1.
  void SetWheelScale(double driver_ticks_per_m, double ticks_per_m);
  double wheel_scale() const { return wheel_scale_; }
  FixResult AddFix(const GnssFix& fix, double t);

  void SetGpsEnabled(bool enabled) { gps_enabled_ = enabled; }
  // max_position_accuracy and gps_timeout of the logic.
  void SetGpsLimits(double max_accuracy, double timeout);
  // The antenna offset (m, frame of base_link); with a recent fix base_link moves behind it.
  void SetAntennaOffset(double x, double y, double t);
  // Starts again at a pose with a valid heading; GNSS sets the position again after a warmup, as a first one.
  void SetPose(const Pose2& pose);
  // Sets the heading and puts base_link behind the antenna of the last fix, if the estimate took a recent one.
  std::optional<Pose2> SetHeading(double heading, double t);

  // The last jump, once.
  std::optional<Jump> TakeJump();
  // True once after the course set the heading.
  bool TakeAligned();

  Pose2 pose() const;
  const Covariance& covariance() const { return filter_.covariance(); }
  // m/s and rad/s of the last step.
  double speed() const { return speed_; }
  double turn_rate() const { return turn_rate_; }
  // The step of the last AddImu that returned true.
  const Motion& motion() const { return motion_; }
  bool calibrated() const { return bias_.calibrated(); }
  double gyro_offset() const { return bias_.offset(); }
  // Calibration windows of the gyro thrown away since the last call, for the log.
  std::vector<std::string> TakeGyroWarnings() { return bias_.TakeWarnings(); }
  bool holding() const { return bias_.holding(); }
  bool heading_valid() const { return heading_valid_; }
  // Standard deviation (rad) of the heading, kNoAccuracy while it is not valid.
  double HeadingAccuracy() const;
  // GNSS set the estimate and no warmup started since.
  bool gps_valid() const { return gate_.initialized(); }
  // A fix counted (warmup included) within gps_timeout before t.
  bool RecentGps(double t) const;
  double PositionAccuracy(double t) const;
  bool geometry_known() const { return geometry_known_; }
  const Eigen::Vector2d& antenna_offset() const { return lever_; }
  // Measured minus expected antenna position of the last fused fix (m).
  const std::optional<Eigen::Vector2d>& last_innovation() const { return last_innovation_; }

 private:
  struct LastFix {
    Eigen::Vector2d antenna;
    double time;
  };
  // The estimate and the heading of the gyro alone at a moment.
  struct Moment {
    double time;
    State state;
    double gyro_heading;
  };

  Moment MomentAt(double t) const;
  void Remember(double t);
  void ShiftHistory(const State& before);
  // Puts base_link behind the antenna of a fix; with a reason it may be a jump.
  void SetOntoFix(const Eigen::Vector2d& antenna, double accuracy, const Moment& then, double t,
                  std::optional<WarmupReason> jump_reason);
  // The fix lies further from the estimate than its covariance allows and than max_position_accuracy.
  bool Contradicts(const Eigen::Vector2d& antenna, double accuracy, const Moment& then) const;
  void Fuse(const Eigen::Vector2d& antenna, double accuracy, const Moment& then, double t);
  // True when the course of a fix at t set the heading.
  bool Course(const GnssFix& fix, const Moment& then, double t);

  EstimatorConfig config_;
  GyroBias bias_;
  PoseFilter filter_;
  FixGate gate_;
  CourseAlignment course_;
  FixAccuracy accuracy_;
  Eigen::Vector2d lever_ = Eigen::Vector2d::Zero();
  bool geometry_known_ = false;
  double wheel_scale_ = 1.0;
  bool gps_enabled_ = true;
  bool heading_valid_ = false;
  bool aligned_ = false;
  std::optional<double> last_imu_;
  std::optional<double> wheel_time_;
  std::optional<double> fast_since_;
  double wheel_speed_ = 0.0;
  double wheel_turn_ = 0.0;
  double speed_ = 0.0;
  double turn_rate_ = 0.0;
  Motion motion_;
  // The gyro integrated since the start, never corrected: courses in a row are compared on it.
  double gyro_heading_ = 0.0;
  std::optional<LastFix> last_fix_;
  int contradictions_ = 0;
  std::optional<Jump> jump_;
  std::optional<Eigen::Vector2d> last_innovation_;
  // The last moments, to compare late GNSS data with.
  StateHistory<Moment> history_;
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_ESTIMATOR_H
