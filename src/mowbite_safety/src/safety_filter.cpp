#include "mowbite_safety/safety_filter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace mowbite_safety {

namespace {

// Tolerance for comparing times against limits.
constexpr double kTimeEpsilon = 1e-6;
constexpr double kRadToDeg = 180.0 / M_PI;
// s between two requests of a latch that hw/status does not show yet.
constexpr double kLatchResend = 1.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// m/s the follower may reverse at most in either frame, whatever the driver allows: nothing watches behind the mower
// (G41).
constexpr double kFollowerMaxReverse = 0.12;
// m/s forwards in the station frame at most: the pose from the dock marker holds for 5 cm past an image (G42).
constexpr double kDockMaxForward = 0.17;

std::string Format(const char* format, double a, double b = 0, double c = 0) {
  char text[200];
  std::snprintf(text, sizeof(text), format, a, b, c);
  return text;
}

}  // namespace

const char* SourceName(Source source) {
  switch (source) {
    case Source::kStop: return "stop";
    case Source::kTeleop: return "remote control";
    case Source::kFollower: return "follower";
    case Source::kCalibration: return "calibration";
    case Source::kDock: return "dock follower";
  }
  return "unknown source";
}

namespace {

// The angle (degrees) between a direction of gravity in the IMU's frame and the one it reads on level ground with the
// mounting roll and pitch: roll atan2(ay, az), pitch atan2(-ax, hypot(ay, az)).
double TiltFromMounting(double gx, double gy, double gz, double roll, double pitch) {
  const double n = std::sqrt(gx * gx + gy * gy + gz * gz);
  if (!(n > 0) || !std::isfinite(n)) return kNaN;
  const double level_x = -std::sin(pitch);
  const double level_y = std::cos(pitch) * std::sin(roll);
  const double level_z = std::cos(pitch) * std::cos(roll);
  const double cos_tilt = (gx * level_x + gy * level_y + gz * level_z) / n;
  return std::acos(std::clamp(cos_tilt, -1.0, 1.0)) * kRadToDeg;
}

}  // namespace

double TiltFromOrientation(double x, double y, double z, double w, double roll, double pitch) {
  const double n = x * x + y * y + z * z + w * w;
  if (!(n > 0) || !std::isfinite(n)) return kNaN;
  // The vertical in the frame of the IMU: the last row of the rotation.
  const double gx = 2.0 * (x * z - w * y) / n;
  const double gy = 2.0 * (y * z + w * x) / n;
  const double gz = 1.0 - 2.0 * (x * x + y * y) / n;
  return TiltFromMounting(gx, gy, gz, roll, pitch);
}

double TiltFromGravity(double ax, double ay, double az, double roll, double pitch) {
  return TiltFromMounting(ax, ay, az, roll, pitch);
}

SafetyFilter::SafetyFilter(const SafetyConfig& config)
    : config_(config),
      dock_zone_(StationZone(Station{}, config.station_behind, config.station_ahead, config.station_half_width)) {
}

void SafetyFilter::OnHardware(bool emergency, uint32_t reason, double now) {
  hw_s_ = now;
  hw_emergency_ = emergency;
  hw_reason_ = reason;
  UpdateEmergency(now);
}

void SafetyFilter::OnCapabilities(bool safety_measured, double max_linear, double max_angular) {
  capabilities_ = true;
  safety_measured_ = safety_measured;
  // Limits that are no number or negative let nothing move.
  max_linear_ = std::isfinite(max_linear) ? std::max(0.0, max_linear) : 0.0;
  max_angular_ = std::isfinite(max_angular) ? std::max(0.0, max_angular) : 0.0;
}

void SafetyFilter::OnTilt(double tilt_deg, double now) {
  // A sample without an angle counts as none.
  if (!std::isfinite(tilt_deg)) return;
  imu_s_ = now;
  tilt_deg_ = tilt_deg;
  if (!(tilt_deg > config_.tilt_limit)) {
    tilt_since_.reset();
    return;
  }
  if (!tilt_since_) tilt_since_ = now;
  if ((latched_ & kReasonTilt) || now - *tilt_since_ + kTimeEpsilon < config_.tilt_time) return;
  Latch(kReasonTilt, Format("tilted by %.0f deg for %.1f s, more than %.0f deg", tilt_deg, now - *tilt_since_,
                            config_.tilt_limit));
  UpdateEmergency(now);
}

void SafetyFilter::OnPose(double x, double y, double accuracy, bool heading_valid, double now) {
  pose_ = Pose{x, y, accuracy, heading_valid, now};
}

void SafetyFilter::OnDockPose(double x, double y, double accuracy, bool heading_valid, double now) {
  dock_pose_ = Pose{x, y, accuracy, heading_valid, now};
}

void SafetyFilter::SetMap(const std::string& map_json) {
  region_ = ParseRegion(map_json, &region_error_);
  allowed_.clear();
  if (!region_) return;
  allowed_ = region_->allowed;
  if (region_->station) {
    allowed_.push_back(StationZone(*region_->station, config_.station_behind, config_.station_ahead,
                                   config_.station_half_width));
  }
}

void SafetyFilter::SetMapFits(bool fits) {
  map_fits_ = fits;
}

void SafetyFilter::SetMaxPositionAccuracy(double accuracy) {
  if (accuracy > 0 && std::isfinite(accuracy)) config_.max_position_accuracy = accuracy;
}

bool SafetyFilter::Emergency(double now) const {
  return latched_ != 0 || !hw_s_ || now - *hw_s_ > config_.hw_timeout || hw_emergency_;
}

void SafetyFilter::UpdateEmergency(double now) {
  const bool emergency = Emergency(now);
  if (emergency == emergency_) return;
  emergency_ = emergency;
  DisarmAll();
}

void SafetyFilter::DisarmAll() {
  for (auto& source : sources_) source.armed = false;
}

SafetyFilter::SourceState& SafetyFilter::State(Source source) {
  return sources_[static_cast<size_t>(source)];
}

bool SafetyFilter::ImuFresh(double now) const {
  return imu_s_ && now - *imu_s_ <= config_.imu_timeout + kTimeEpsilon;
}

std::string SafetyFilter::AutonomousProblem(double now) const {
  if (!capabilities_ || !safety_measured_) return "no safety measurement of this platform (hw/capabilities)";
  if (!pose_ || now - pose_->s > config_.pose_timeout + kTimeEpsilon) return "no recent pose from the localization";
  if (!(pose_->accuracy <= config_.max_position_accuracy)) {
    return Format("position accuracy %.3f m, %.3f m needed", pose_->accuracy, config_.max_position_accuracy);
  }
  if (!pose_->heading_valid) return "the heading is not aligned";
  if (!region_) return region_error_;
  if (!map_fits_) return "the map does not fit the datum (map/status)";
  if (allowed_.empty()) return "the map has no active mow or navigation area";
  return "";
}

bool SafetyFilter::AutonomousOk(double now) const {
  return AutonomousProblem(now).empty();
}

std::string SafetyFilter::DockProblem(double now) const {
  if (!capabilities_ || !safety_measured_) return "no safety measurement of this platform (hw/capabilities)";
  if (!dock_pose_ || now - dock_pose_->s > config_.pose_timeout + kTimeEpsilon) {
    return "no recent pose from the dock marker";
  }
  if (!(dock_pose_->accuracy <= config_.dock_max_position_accuracy)) {
    return Format("position accuracy from the dock marker %.3f m, %.3f m needed", dock_pose_->accuracy,
                  config_.dock_max_position_accuracy);
  }
  if (!dock_pose_->heading_valid) return "no heading from the dock marker";
  return "";
}

bool SafetyFilter::DockOk(double now) const {
  return DockProblem(now).empty();
}

double SafetyFilter::OutsideDistance(double x, double y) const {
  double closest = std::numeric_limits<double>::infinity();
  for (const auto& ring : allowed_) {
    if (InsideRing(ring, x, y)) return 0;
    closest = std::min(closest, DistanceToRing(ring, x, y));
  }
  return closest;
}

std::string SafetyFilter::FenceProblem(double x, double y) const {
  // Called only after AutonomousProblem found nothing, which needs region_.
  for (const auto& ring : region_->obstacles) {  // NOLINT(bugprone-unchecked-optional-access)
    if (!InsideRing(ring, x, y)) continue;
    const double depth = DistanceToRing(ring, x, y);
    if (depth > config_.obstacle_margin) {
      return Format("base_link is %.2f m inside an obstacle at (%.2f, %.2f)", depth, x, y);
    }
  }
  const double outside = OutsideDistance(x, y);
  if (outside > config_.fence_margin) {
    return Format("base_link is %.2f m outside of the allowed areas at (%.2f, %.2f)", outside, x, y);
  }
  return "";
}

std::string SafetyFilter::DockFenceProblem(double x, double y) const {
  const double outside = InsideRing(dock_zone_, x, y) ? 0.0 : DistanceToRing(dock_zone_, x, y);
  if (outside > config_.fence_margin) {
    return Format("base_link is %.2f m outside of the zone of the station at (%.2f, %.2f) in its frame", outside, x, y);
  }
  return "";
}

std::string SafetyFilter::MoveProblem(Source source, double linear, double quiet, double now, bool* conflict) {
  if (emergency_) return "emergency";
  // Backwards it would leave the strip it checked (G9: behind the mower is not watched everywhere).
  if (source == Source::kCalibration && linear < 0) return "the calibration drives forward only";
  if (owner_ != Source::kStop && owner_ != source && now - owner_s_ <= config_.owner_timeout + kTimeEpsilon) {
    *conflict = true;
    return std::string("the ") + SourceName(owner_) + " drives";
  }
  SourceState& state = State(source);
  if (!state.armed) {
    if (quiet + kTimeEpsilon < config_.arm_quiet) {
      return Format("waits for a zero or %.1f s without commands", config_.arm_quiet);
    }
    state.armed = true;
  }
  if (!capabilities_) return "no hw/capabilities from the driver";
  if (!ImuFresh(now)) return imu_s_ ? Format("no IMU sample for %.1f s", now - *imu_s_) : "no IMU sample yet";
  std::string fence;
  if (source == Source::kDock) {
    std::string problem = DockProblem(now);
    if (!problem.empty()) return problem;
    // DockProblem found nothing, so dock_pose_ is there.
    fence = DockFenceProblem(dock_pose_->x, dock_pose_->y);  // NOLINT(bugprone-unchecked-optional-access)
  } else if (source == Source::kFollower || source == Source::kCalibration) {
    std::string problem = AutonomousProblem(now);
    if (!problem.empty()) return problem;
    // AutonomousProblem found nothing, so pose_ is there.
    fence = FenceProblem(pose_->x, pose_->y);  // NOLINT(bugprone-unchecked-optional-access)
  }
  if (fence.empty()) return "";
  Latch(kReasonFence, "autonomous drive with " + fence);
  UpdateEmergency(now);
  return fence;
}

void SafetyFilter::Latch(uint32_t bits, const std::string& reason) {
  if ((latched_ & bits) == bits) return;
  latched_ |= bits;
  latch_reason_ = reason;
  latch_new_ = true;
}

Decision SafetyFilter::OnCommand(Source source, double linear, double angular, double now) {
  UpdateEmergency(now);
  Decision decision;
  if (source == Source::kStop) {
    owner_ = Source::kStop;
    moving_.reset();
    decision.out = Twist2{};
    return decision;
  }
  SourceState& state = State(source);
  const double quiet = state.last_s ? now - *state.last_s : std::numeric_limits<double>::infinity();
  state.last_s = now;
  const bool valid = std::isfinite(linear) && std::isfinite(angular);
  if (!valid || (linear == 0 && angular == 0)) {
    // A zero arms its source; it stops whatever moves, and so does a command that is no number.
    if (valid) state.armed = true;
    if (owner_ == source) owner_ = Source::kStop;
    moving_.reset();
    decision.out = Twist2{};
    if (!valid) {
      refused_++;
      last_refusal_ = std::string(SourceName(source)) + ": a command that is not a number, stopped";
      decision.refusal = "a command that is not a number, stopped";
    }
    return decision;
  }
  const std::string problem = MoveProblem(source, linear, quiet, now, &decision.conflict);
  if (!problem.empty()) {
    refused_++;
    last_refusal_ = std::string(SourceName(source)) + ": " + problem;
    decision.refusal = problem;
    // Its own motion stops at once instead of after the cmd_vel timeout of the driver.
    if (moving_ == source) {
      moving_.reset();
      decision.out = Twist2{};
    }
    return decision;
  }
  if (owner_ != source) {
    // A source takes the drive: the others have to arm again.
    for (size_t i = 0; i < sources_.size(); i++) {
      if (i != static_cast<size_t>(source)) sources_[i].armed = false;
    }
    owner_ = source;
  }
  owner_s_ = now;
  moving_ = source;
  const bool follower = source == Source::kFollower || source == Source::kDock;
  const double forward = source == Source::kDock ? std::min(max_linear_, kDockMaxForward) : max_linear_;
  const double reverse = follower ? std::min(max_linear_, kFollowerMaxReverse) : max_linear_;
  decision.out = Twist2{std::clamp(linear, -reverse, forward), std::clamp(angular, -max_angular_, max_angular_)};
  return decision;
}

std::optional<Twist2> SafetyFilter::Tick(double now) {
  UpdateEmergency(now);
  if (owner_ != Source::kStop && now - owner_s_ > config_.owner_timeout + kTimeEpsilon) owner_ = Source::kStop;
  if (!moving_) return std::nullopt;
  if (emergency_ || !ImuFresh(now)) {
    moving_.reset();
    return Twist2{};
  }
  // A source that went silent was stopped by the cmd_vel timeout of the driver.
  const auto& last = sources_[static_cast<size_t>(*moving_)].last_s;
  if (!last || now - *last > config_.owner_timeout + kTimeEpsilon) moving_.reset();
  return std::nullopt;
}

void SafetyFilter::OnRestartHold(bool present) {
  if (present && !restart_hold_) Latch(kReasonUnexpectedRestart, "unexpected restart: the mower waits for a release");
  restart_hold_ = present;
}

std::optional<LatchRequest> SafetyFilter::TakeLatchRequest(double now) {
  if (latched_ == 0) return std::nullopt;
  const bool shown = (hw_reason_ & latched_) == latched_;
  if (!latch_new_ && (shown || (latch_sent_s_ && now - *latch_sent_s_ + kTimeEpsilon < kLatchResend))) {
    return std::nullopt;
  }
  LatchRequest request{latched_, latch_reason_, !latch_new_};
  latch_new_ = false;
  latch_sent_s_ = now;
  return request;
}

Release SafetyFilter::OnRelease(double now) {
  if (!ImuFresh(now)) {
    Release refused{false, "no IMU sample yet, the tilt is unknown", "TILT_UNKNOWN"};
    if (imu_s_) {
      refused.message = Format("no IMU sample for %.1f s, the tilt is unknown", now - *imu_s_);
      refused.value = now - *imu_s_;
    }
    return refused;
  }
  if (!(tilt_deg_ < config_.tilt_release)) {
    return {false, Format("still tilted by %.0f deg, below %.0f deg needed", tilt_deg_, config_.tilt_release),
            "STILL_TILTED", tilt_deg_, config_.tilt_release};
  }
  latched_ = 0;
  latch_reason_.clear();
  latch_new_ = false;
  latch_sent_s_.reset();
  tilt_since_.reset();
  owner_ = Source::kStop;
  moving_.reset();
  DisarmAll();
  UpdateEmergency(now);
  return {true, "", ""};
}

uint32_t SafetyFilter::Latched() const {
  return latched_;
}

double SafetyFilter::TiltDeg(double now) const {
  return ImuFresh(now) ? tilt_deg_ : kNaN;
}

double SafetyFilter::FenceDistance(double now) const {
  if (!region_ || !map_fits_ || allowed_.empty() || !pose_ || now - pose_->s > config_.pose_timeout + kTimeEpsilon) {
    return kNaN;
  }
  return OutsideDistance(pose_->x, pose_->y);
}

Source SafetyFilter::Owner(double now) const {
  if (owner_ != Source::kStop && now - owner_s_ > config_.owner_timeout + kTimeEpsilon) return Source::kStop;
  return owner_;
}

uint32_t SafetyFilter::Refused() const {
  return refused_;
}

const std::string& SafetyFilter::LastRefusal() const {
  return last_refusal_;
}

}  // namespace mowbite_safety
