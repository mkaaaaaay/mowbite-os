#include "mowbite_localization/estimator.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <utility>

namespace mowbite_localization {

namespace {

// m driven since the last fix below which a new lever moves base_link behind that fix.
constexpr double kPlaceStill = 0.01;
// s of moments kept behind gnss_latency: half the longest window of a course (1.5 s in tron_gnss) and a margin.
constexpr double kCourseWindowKeep = 1.5;

// The lever turned by a heading: where the antenna is from base_link in the map frame.
Eigen::Vector2d Turned(const Eigen::Vector2d& lever, double heading) {
  return AntennaPosition(State(0.0, 0.0, heading), lever);
}

Covariance InitialCovariance() {
  return Eigen::Vector3d(1.0, 1.0, M_PI * M_PI).asDiagonal();
}

}  // namespace

double Jump::distance() const {
  return std::hypot(to_x - from_x, to_y - from_y);
}

Estimator::Estimator(const EstimatorConfig& config)
    : config_(config),
      bias_(config.gyro),
      filter_(config.noise),
      gate_(config.gate),
      course_(config.course),
      track_(config.course),
      accuracy_(config.accuracy) {
  filter_.Reset(State::Zero(), InitialCovariance());
}

bool Estimator::AddImu(double t, double rate) {
  if (!std::isfinite(t) || !std::isfinite(rate)) return false;
  if (!last_imu_ || t <= *last_imu_) {
    last_imu_ = t;
    return false;
  }
  const double dt = t - *last_imu_;
  last_imu_ = t;
  if (dt > config_.max_imu_gap) {
    // What the mower turned and drove meanwhile is not known.
    if (bias_.calibrated()) {
      LoosenHeading();
      accuracy_.Forget();
    }
    return false;
  }
  const bool fresh = wheel_time_ && t - *wheel_time_ <= config_.wheel_timeout;
  const double speed = fresh ? wheel_speed_ : 0.0;
  const double wheel_turn = fresh ? wheel_turn_ : 0.0;
  // Without a fresh wheel report nobody knows the mower stands: the gyro offset is neither measured nor followed.
  const bool slow = fresh && std::abs(speed) < config_.min_speed && std::abs(wheel_turn) < config_.min_speed;
  if (slow || !fresh) {
    fast_since_.reset();
  } else if (!fast_since_) {
    fast_since_ = t;
  }
  const bool still = slow || (fast_since_ && t - *fast_since_ < config_.motion_confirm_time);
  const bool was_calibrated = bias_.calibrated();
  bias_.Add(rate, dt, still);
  // The sample that ends the calibration only starts the integration.
  if (!was_calibrated) return false;
  speed_ = speed;
  turn_rate_ = bias_.Corrected(rate);
  gyro_heading_ = WrapAngle(gyro_heading_ + turn_rate_ * dt);
  motion_ = Motion{t, speed * dt, turn_rate_ * dt, bias_.holding() ? 0.0 : dt, bias_.holding()};
  const double slip = fresh ? 0.5 * wheel_distance_ * std::abs(wheel_turn - turn_rate_) * dt : 0.0;
  filter_.Predict(motion_.distance, motion_.turn, motion_.gyro_time, slip);
  accuracy_.OnDistance(motion_.distance);
  accuracy_.OnSlip(fresh ? slip : config_.stale_wheel_speed * dt);
  Remember(t);
  return true;
}

void Estimator::AddWheels(double t, double speed, double turn_rate) {
  if (!std::isfinite(t) || !std::isfinite(speed) || !std::isfinite(turn_rate)) return;
  wheel_time_ = t;
  wheel_speed_ = speed * wheel_scale_;
  wheel_turn_ = turn_rate * wheel_scale_;
}

void Estimator::SetWheelScale(double driver_ticks_per_m, double ticks_per_m) {
  const bool known = std::isfinite(driver_ticks_per_m) && std::isfinite(ticks_per_m) && driver_ticks_per_m > 0 &&
                     ticks_per_m > 0;
  wheel_scale_ = known ? driver_ticks_per_m / ticks_per_m : 1.0;
}

void Estimator::SetWheelDistance(double wheel_distance) {
  wheel_distance_ = std::isfinite(wheel_distance) && wheel_distance > 0 ? wheel_distance : 0.0;
}

FixResult Estimator::AddFix(const GnssFix& fix, double t) {
  if (!bias_.calibrated()) return FixResult::kNotReady;
  if (!gps_enabled_) return FixResult::kDisabled;
  if (!geometry_known_) return FixResult::kNoGeometry;
  if (auto rejected = gate_.Reject(fix)) return *rejected;
  const Eigen::Vector2d antenna(fix.x, fix.y);
  const double then_time = t - config_.gnss_latency;
  const Moment then = MomentAt(then_time);
  const auto course_then = [&] {
    return fix.motion_window > 0.0 ? CourseMomentAt(then_time - 0.5 * fix.motion_window) : MomentAt(then_time);
  };
  FixResult result = gate_.Classify(t, antenna, AntennaPosition(then.state, Lever(then.tilt)));
  // A new warmup starts the track again; an outlier is no part of it.
  if (result == FixResult::kRestarted || result == FixResult::kOutlierReset) track_.Clear();
  if (result != FixResult::kOutlier) track_.Add(antenna, t);
  if (result != FixResult::kFused) contradictions_ = 0;
  // What the course of this fix changes the heading by.
  const double heading_before = filter_.state()(2);
  const auto turn = [&] { return WrapAngle(filter_.state()(2) - heading_before); };
  switch (result) {
    case FixResult::kRestarted:
    case FixResult::kWarmup:
      Course(fix, course_then(), t);
      break;
    case FixResult::kInitialized:
      Course(fix, course_then(), t);
      SetOntoFix(antenna, fix.accuracy, MomentAt(then_time), t, turn(),
                 gate_.reason() == WarmupReason::kFirst ? std::nullopt : std::optional(gate_.reason()));
      break;
    case FixResult::kFused: {
      // Without a heading, and when the course just set it, the dead reckoning since the last fix went the wrong
      // way: the fix sets the position.
      const bool turned = Course(fix, course_then(), t);
      if (turned || !heading_valid_) {
        contradictions_ = 0;
        SetOntoFix(antenna, fix.accuracy, MomentAt(then_time), t, turn(), std::nullopt);
      } else if (!Contradicts(antenna, fix.accuracy, MomentAt(then_time))) {
        contradictions_ = 0;
        Fuse(antenna, fix.accuracy, MomentAt(then_time), t);
      } else if (++contradictions_ < config_.contradiction_reset) {
        result = FixResult::kContradicting;
      } else {
        contradictions_ = 0;
        SetOntoFix(antenna, fix.accuracy, MomentAt(then_time), t, turn(), WarmupReason::kOutliers);
        result = FixResult::kContradictionReset;
      }
      break;
    }
    default:
      course_.Clear();
      break;
  }
  return result;
}

void Estimator::SetOntoFix(const Eigen::Vector2d& antenna, double accuracy, const Moment& then, double t, double turn,
                           std::optional<WarmupReason> jump_reason) {
  const State before = filter_.state();
  // base_link behind the antenna at the moment of the fix, moved on by what the estimate did since: that way was
  // integrated along the heading from before the course of the fix.
  const Eigen::Vector2d since = before.head<2>() - then.state.head<2>();
  const Eigen::Vector2d base = antenna - Turned(Lever(then.tilt), then.state(2)) + Turned(since, turn);
  filter_.SetPosition(base(0), base(1), accuracy * accuracy);
  ShiftHistory(before);
  accuracy_.OnFix(accuracy, t);
  last_fix_ = LastFix{antenna, t};
  if (!jump_reason) return;
  Jump jump;
  jump.reason = *jump_reason;
  jump.from_x = before(0);
  jump.from_y = before(1);
  jump.to_x = base(0);
  jump.to_y = base(1);
  if (jump.distance() > config_.gate.max_accuracy) {
    jump_ = jump;
    LoosenHeading();
  }
}

void Estimator::LoosenHeading() {
  if (!heading_valid_) return;
  const double variance = config_.disturbed_heading_sigma * config_.disturbed_heading_sigma;
  if (filter_.covariance()(2, 2) >= variance) return;
  filter_.SetHeading(filter_.state()(2), variance);
}

bool Estimator::Contradicts(const Eigen::Vector2d& antenna, double accuracy, const Moment& then) const {
  const Eigen::Vector2d lever = Lever(then.tilt);
  const Eigen::Vector2d innovation = antenna - AntennaPosition(then.state, lever);
  if (innovation.norm() <= config_.gate.max_accuracy) return false;
  const AntennaJacobian h = AntennaPositionJacobian(then.state, lever);
  const double sigma = std::max(accuracy, 0.005);
  const Eigen::Matrix2d s = h * filter_.covariance() * h.transpose() + sigma * sigma * Eigen::Matrix2d::Identity();
  return innovation.dot(s.ldlt().solve(innovation)) > config_.contradiction_chi2;
}

void Estimator::Fuse(const Eigen::Vector2d& antenna, double accuracy, const Moment& then, double t) {
  const State before = filter_.state();
  const Eigen::Vector2d lever = Lever(then.tilt);
  const Eigen::Vector2d innovation = antenna - AntennaPosition(then.state, lever);
  const double sigma = std::max(accuracy, 0.005);
  // An antenna at rest says nothing about the heading: only the fixes of a drive correct it.
  const bool heading = std::abs(speed_) >= config_.min_speed && std::abs(turn_rate_) <= config_.steady_turn_rate;
  filter_.UpdatePosition(innovation, AntennaPositionJacobian(then.state, lever), sigma * sigma, heading);
  ShiftHistory(before);
  accuracy_.OnFix(accuracy, t);
  last_fix_ = LastFix{antenna, t};
  last_innovation_ = innovation;
}

bool Estimator::Course(const GnssFix& fix, const Moment& then, double t) {
  // Without a velocity, with a window that is no time, or too slow, a fix says nothing about the heading; an
  // inconsistent course breaks the run.
  if (!fix.motion_valid || !(fix.motion_window >= 0.0) || !std::isfinite(fix.motion_window)) return false;
  const Eigen::Vector2d velocity(fix.motion_x, fix.motion_y);
  const CourseHeading course = HeadingFromCourse(velocity, speed_, turn_rate_, Lever(then.tilt), config_.course);
  if (course.kind == CourseKind::kInconsistent) course_.Clear();
  if (course.kind != CourseKind::kUsable) return false;
  switch (track_.Check(velocity)) {
    case TrackCheck::kNoTrack:
      return false;
    case TrackCheck::kContradicts:
      course_.Clear();
      ++course_rejections_;
      return false;
    case TrackCheck::kAgrees:
      break;
  }
  // A chord of positions carries the noise of its two ends only: courses over overlapping windows count each.
  const std::optional<double> heading = course.heading;
  const double variance = CourseVariance(velocity.norm(), config_.course);
  const double heading_variance = filter_.covariance()(2, 2);
  // On the gyro alone the courses of a run agree whatever the estimate did meanwhile.
  const std::optional<double> run = course_.Add(WrapAngle(*heading - then.gyro_heading), t);
  if (run) {
    // The heading the run says for now, against the estimate.
    const double from_run = WrapAngle(gyro_heading_ + *run);
    const double off = WrapAngle(from_run - filter_.state()(2));
    const double limit =
        std::max(config_.heading_reset_angle, 3.0 * std::sqrt(heading_variance + variance / config_.course.run));
    // A run of agreeing courses sets an unknown heading, and a known one that they contradict.
    if (!heading_valid_ || std::abs(off) > limit) {
      const State before = filter_.state();
      filter_.SetHeading(from_run, variance / config_.course.run);
      ShiftHistory(before);
      heading_valid_ = true;
      aligned_ = true;
      course_.Clear();
      return true;
    }
  }
  if (!heading_valid_ || std::abs(turn_rate_) > config_.steady_turn_rate) return false;
  const double difference = WrapAngle(*heading - then.state(2));
  if (std::abs(difference) > std::max(config_.heading_reset_angle, 3.0 * std::sqrt(heading_variance + variance))) {
    return false;
  }
  const State before = filter_.state();
  filter_.UpdateHeading(difference, variance);
  ShiftHistory(before);
  return false;
}

void Estimator::SetGpsLimits(double max_accuracy, double timeout) {
  // Limits that are no positive number would switch the checks off: the ones before stay.
  if (!(max_accuracy > 0) || !std::isfinite(max_accuracy) || !(timeout > 0) || !std::isfinite(timeout)) return;
  config_.gate.max_accuracy = max_accuracy;
  gate_.SetMaxAccuracy(max_accuracy);
  accuracy_.SetTimeout(timeout);
}

void Estimator::SetAntennaOffset(double x, double y, double t) {
  if (!std::isfinite(x) || !std::isfinite(y)) return;
  if (geometry_known_ && lever_ == Eigen::Vector2d(x, y)) return;
  lever_ = Eigen::Vector2d(x, y);
  geometry_known_ = true;
  // Behind the last fix only while the mower has not driven since: else the way since would be lost.
  if (StillSinceFix()) PlaceBehindLastFix(t);
}

bool Estimator::StillSinceFix() const {
  return accuracy_.distance() < kPlaceStill;
}

void Estimator::PlaceBehindLastFix(double t) {
  if (!gate_.initialized() || !last_fix_ || t - last_fix_->time >= accuracy_.timeout()) return;
  const State before = filter_.state();
  const Eigen::Vector2d base = last_fix_->antenna - Turned(Lever(TiltAt(t)), before(2));
  filter_.Shift(State(base(0) - before(0), base(1) - before(1), 0.0));
  ShiftHistory(before);
}

void Estimator::AddTilt(double t, double roll, double pitch) {
  if (!std::isfinite(t) || !std::isfinite(roll) || !std::isfinite(pitch)) return;
  tilt_ = Tilt{roll, pitch};
  tilt_time_ = t;
}

Tilt Estimator::TiltAt(double t) const {
  return tilt_time_ && std::abs(t - *tilt_time_) <= config_.max_imu_gap ? tilt_ : Tilt{};
}

Eigen::Vector2d Estimator::Lever(const Tilt& tilt) const {
  return TiltedLever(Eigen::Vector3d(lever_(0), lever_(1), config_.antenna_height), tilt);
}

void Estimator::SetPose(const Pose2& pose) {
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.heading)) return;
  const Covariance covariance =
      Eigen::Vector3d(config_.pose_variance, config_.pose_variance, config_.pose_heading_variance).asDiagonal();
  filter_.Reset(State(pose.x, pose.y, pose.heading), covariance);
  heading_valid_ = true;
  speed_ = turn_rate_ = 0.0;
  gate_.Forget();
  accuracy_.Forget();
  course_.Clear();
  history_.Clear();
  last_fix_.reset();
  contradictions_ = 0;
  jump_.reset();
}

std::optional<Pose2> Estimator::SetHeading(double heading, double t) {
  if (!std::isfinite(heading) || !gate_.initialized() || !last_fix_ || t - last_fix_->time >= accuracy_.timeout()) {
    return std::nullopt;
  }
  const State before = filter_.state();
  const Eigen::Vector2d lever = Lever(TiltAt(t));
  // The antenna stays at the last fix while the mower has not driven since, else where the estimate has it now.
  const Eigen::Vector2d antenna = StillSinceFix() ? last_fix_->antenna
                                                  : Eigen::Vector2d(before(0), before(1)) + Turned(lever, before(2));
  const Eigen::Vector2d base = antenna - Turned(lever, heading);
  filter_.SetHeading(heading, config_.set_heading_variance);
  filter_.Shift(State(base(0) - before(0), base(1) - before(1), 0.0));
  ShiftHistory(before);
  heading_valid_ = true;
  speed_ = turn_rate_ = 0.0;
  course_.Clear();
  return pose();
}

std::optional<Jump> Estimator::TakeJump() {
  std::optional<Jump> jump;
  std::swap(jump, jump_);
  return jump;
}

bool Estimator::TakeAligned() {
  return std::exchange(aligned_, false);
}

int Estimator::TakeCourseRejections() {
  return std::exchange(course_rejections_, 0);
}

Pose2 Estimator::pose() const {
  const State& state = filter_.state();
  return Pose2{state(0), state(1), state(2)};
}

double Estimator::HeadingAccuracy() const {
  return heading_valid_ ? std::sqrt(std::max(0.0, filter_.covariance()(2, 2))) : kNoAccuracy;
}

bool Estimator::RecentGps(double t) const {
  const std::optional<double> last = gate_.last_accepted();
  return (gate_.initialized() || gate_.warming_up()) && last && t - *last < accuracy_.timeout();
}

double Estimator::PositionAccuracy(double t) const {
  if (!gate_.initialized()) return kNoAccuracy;
  // Never better than the spread of the estimate itself, with that of a valid heading over the lever of the antenna
  // (at most twice the lever). Without a valid heading that part is left out: heading_valid says it instead, and the
  // mower stands on its station or drives the straight undock line, which needs none.
  const Covariance& p = filter_.covariance();
  const double lever = lever_.norm();
  const double turn = heading_valid_ ? std::min(std::sqrt(std::max(0.0, p(2, 2))) * lever, 2.0 * lever) : 0.0;
  return std::max(accuracy_.Value(t), std::sqrt(std::max(p(0, 0), p(1, 1))) + turn);
}

double Estimator::PositionAccuracyAtHeading(double t) const {
  if (!gate_.initialized()) return kNoAccuracy;
  const Covariance& p = filter_.covariance();
  return std::max(accuracy_.Value(t), std::sqrt(std::max(p(0, 0), p(1, 1))));
}

double Estimator::FixAge(double t) const {
  return gate_.initialized() && last_fix_ ? std::max(0.0, t - last_fix_->time) : kNoAccuracy;
}

Estimator::Moment Estimator::MomentAt(double t) const {
  if (config_.gnss_latency <= 0.0 || history_.empty()) return Moment{t, filter_.state(), gyro_heading_, TiltAt(t)};
  return *history_.At(t);
}

Estimator::Moment Estimator::CourseMomentAt(double t) const {
  if (history_.empty()) return Moment{t, filter_.state(), gyro_heading_, TiltAt(t)};
  return *history_.At(t);
}

void Estimator::Remember(double t) {
  history_.Add(Moment{t, filter_.state(), gyro_heading_, TiltAt(t)}, t - config_.gnss_latency - kCourseWindowKeep);
}

void Estimator::ShiftHistory(const State& before) {
  history_.Shift(before, filter_.state());
}

}  // namespace mowbite_localization
