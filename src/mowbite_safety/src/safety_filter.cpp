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
  }
  return "unknown source";
}

double TiltFromOrientation(double x, double y, double z, double w) {
  const double n = x * x + y * y + z * z + w * w;
  if (!(n > 0) || !std::isfinite(n)) return kNaN;
  // The z axis of the body against the vertical: element (2, 2) of the rotation.
  const double cos_tilt = 1.0 - 2.0 * (x * x + y * y) / n;
  return std::acos(std::clamp(cos_tilt, -1.0, 1.0)) * kRadToDeg;
}

double TiltFromGravity(double ax, double ay, double az) {
  const double n = std::sqrt(ax * ax + ay * ay + az * az);
  if (!(n > 0) || !std::isfinite(n)) return kNaN;
  return std::acos(std::clamp(az / n, -1.0, 1.0)) * kRadToDeg;
}

SafetyFilter::SafetyFilter(const SafetyConfig& config) : config_(config) {
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
    return Format("position accuracy %.2f m, %.2f m needed", pose_->accuracy, config_.max_position_accuracy);
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

double SafetyFilter::OutsideDistance(double x, double y) const {
  double closest = std::numeric_limits<double>::infinity();
  for (const auto& ring : allowed_) {
    if (InsideRing(ring, x, y)) return 0;
    closest = std::min(closest, DistanceToRing(ring, x, y));
  }
  return closest;
}

std::string SafetyFilter::FenceProblem(double x, double y) const {
  for (const auto& ring : region_->obstacles) {
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

std::string SafetyFilter::MoveProblem(Source source, double quiet, double now, bool* conflict) {
  if (emergency_) return "emergency";
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
  if (source != Source::kFollower) return "";
  const std::string problem = AutonomousProblem(now);
  if (!problem.empty()) return problem;
  const std::string fence = FenceProblem(pose_->x, pose_->y);
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
  const std::string problem = MoveProblem(source, quiet, now, &decision.conflict);
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
  decision.out = Twist2{std::clamp(linear, -max_linear_, max_linear_), std::clamp(angular, -max_angular_, max_angular_)};
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
    return {false, imu_s_ ? Format("no IMU sample for %.1f s, the tilt is unknown", now - *imu_s_)
                          : "no IMU sample yet, the tilt is unknown"};
  }
  if (!(tilt_deg_ < config_.tilt_release)) {
    return {false, Format("still tilted by %.0f deg, below %.0f deg needed", tilt_deg_, config_.tilt_release)};
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
  return {true, ""};
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
