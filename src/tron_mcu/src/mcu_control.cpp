#include "tron_mcu/mcu_control.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace tron {

namespace {

// Tolerance for comparing loop times against deadlines.
constexpr double kTimeEpsilon = 1e-6;

bool VersionMatches(const std::string& expected, const Version& version) {
  return expected.empty() || expected == VersionString(version);
}

}  // namespace

const char* BladeResultText(BladeResult result) {
  switch (result) {
    case BladeResult::kOk: return "ok";
    case BladeResult::kNotAllowed: return "blade not allowed by the configuration";
    case BladeResult::kInvalidLease: return "lease out of range";
    case BladeResult::kInvalidSpeed: return "speed out of range";
    case BladeResult::kInvalidHeight: return "height out of range";
    case BladeResult::kBlocked: return "mower blocked";
    case BladeResult::kDocked: return "mower on the dock";
  }
  return "unknown";
}

McuControl::McuControl(const McuControlConfig& config) : config_(config) {
}

void McuControl::OnCmdVel(float linear, float angular, double now) {
  pending_cmd_ = PendingCmd{linear, angular};
  last_cmd_s_ = now;
}

void McuControl::OnSensorInfo(const SensorInfo& info, double now) {
  sensor_ = info;
  last_sensor_s_ = now;
}

void McuControl::OnDockContactSample(bool contact) {
  dock_raw_ = contact;
  dock_samples_ = contact ? std::min(dock_samples_ + 1, config_.dock_debounce_samples) : 0;
}

void McuControl::OnImu(const ImuCounts& imu, double now) {
  pending_imu_ = imu;
  last_imu_sample_s_ = now;
}

void McuControl::OnBladeRpm(int rpm, double now) {
  // Only a blade that should have stopped by now counts: not while it is on, not while it runs down.
  if (blade_on_ || !blade_off_since_s_ || config_.blade_stop_time_s <= 0 ||
      now - *blade_off_since_s_ < config_.blade_stop_time_s) {
    blade_turning_ = false;
    return;
  }
  blade_turning_ = std::abs(rpm) >= config_.blade_still_rpm;
  if (!blade_turning_) return;
  LatchEmergency(kReasonMowerNotStopped);
  // A lost off frame is the likely cause, so the off frames go out again with every such report.
  StartRepeat(blade_off_repeat_, BladeOff(), now);
}

bool McuControl::OnVersions(const VersionInfo& versions) {
  const bool ok = VersionMatches(config_.expected_cutter_version, versions.cutter_board) &&
                  VersionMatches(config_.expected_chassis_version, versions.chassis_board);
  // A mismatch also latches, so motion after a later matching frame still needs a release.
  if (!ok && firmware_ok_ != false) LatchEmergency(kReasonFirmwareIncompatible);
  firmware_ok_ = ok;
  return ok;
}

void McuControl::LatchEmergency(uint32_t reason) {
  latched_ = true;
  latch_reason_ |= reason;
  release_until_s_.reset();
}

void McuControl::ReleaseEmergency(double now) {
  release_requested_ = true;
  release_until_s_ = now + config_.release_window_s;
}

BladeResult McuControl::SetBlade(const BladeRequest& request, double now) {
  if (!request.on) {
    DropBlade(BladeOffReason::kRequest);
    return BladeResult::kOk;
  }
  if (!config_.allow_blade) return BladeResult::kNotAllowed;
  if (!std::isfinite(request.lease_s) || request.lease_s <= 0 ||
      request.lease_s > config_.blade_lease_max_s + kTimeEpsilon) {
    return BladeResult::kInvalidLease;
  }
  if (request.speed_percent > 100) return BladeResult::kInvalidSpeed;
  if (request.height_mm != 0 &&
      (request.height_mm < kMinCutterHeightMm || request.height_mm > kMaxCutterHeightMm)) {
    return BladeResult::kInvalidHeight;
  }
  // A refused renewal also ends a running lease, so the blade never outlives a refusal.
  if (Blocked(now)) {
    DropBlade(BladeOffReason::kBlocked);
    return BladeResult::kBlocked;
  }
  if (DockContact()) {
    DropBlade(BladeOffReason::kDock);
    return BladeResult::kDocked;
  }
  BladeSetting setting;
  setting.speed_percent = request.speed_percent ? request.speed_percent : config_.blade_speed_percent;
  setting.height_mm = request.height_mm ? request.height_mm : config_.mow_height_mm;
  blade_request_ = BladeLease{setting, now + request.lease_s};
  return BladeResult::kOk;
}

void McuControl::DropBlade(BladeOffReason reason) {
  if (blade_request_) blade_off_reason_ = reason;
  blade_request_.reset();
}

void McuControl::OnConnected(double now) {
  started_ = false;
  next_heartbeat_s_.reset();
  pending_cmd_.reset();
  DropBlade(BladeOffReason::kReconnect);
  // The start of the next tick sends the off frames.
  blade_on_.reset();
  // After the port reopens the MCU might run another firmware, e.g. after an update.
  firmware_ok_.reset();
  if (moving_) {
    moving_ = false;
    StartRepeat(stop_repeat_, ZeroSpeed(), now);
  }
}

bool McuControl::Blocked(double now) const {
  return latched_ || TelemetryStale(now) || NoImuYet() || FirmwareIncompatible();
}

// As hw/status shows it: an IMU still due after the start blocks without an emergency.
bool McuControl::Reported(double now) const {
  return latched_ || TelemetryStale(now) || (NoImuYet() && !ImuStarting(now)) || FirmwareIncompatible();
}

uint32_t McuControl::EmergencyReason(double now) const {
  if (!Reported(now)) return 0;
  uint32_t reason = latch_reason_;
  if (latched_) reason |= kReasonLatch;
  if (TelemetryStale(now)) reason |= kReasonTimeoutInputs;
  if (Stop(now)) reason |= kReasonStop;
  if (Lift(now)) reason |= kReasonLift;
  if (config_.bumper_latches_emergency && Bumper(now)) reason |= kReasonCollision;
  if (FirmwareIncompatible()) reason |= kReasonFirmwareIncompatible;
  if (NoImuYet() && !ImuStarting(now)) reason |= kReasonImuTimeout;
  return reason;
}

bool McuControl::DockContact() const {
  return dock_samples_ >= config_.dock_debounce_samples;
}

bool McuControl::TelemetryStale(double now) const {
  return !sensor_ || now - last_sensor_s_ > config_.telemetry_timeout_s;
}

bool McuControl::ImuStale(double now) const {
  return config_.imu_timeout_s > 0 && (!last_imu_sample_s_ || now - *last_imu_sample_s_ > config_.imu_timeout_s);
}

// Before its first sample the IMU blocks without a latch; a later gap latches in Tick, like lift.
bool McuControl::NoImuYet() const {
  return config_.imu_timeout_s > 0 && !last_imu_sample_s_;
}

// No sample yet, within the start time of the first tick.
bool McuControl::ImuStarting(double now) const {
  const double start_time = std::max(config_.imu_timeout_s, config_.imu_start_timeout_s);
  return NoImuYet() && (!start_s_ || now - *start_s_ <= start_time + kTimeEpsilon);
}

bool McuControl::FirmwareIncompatible() const {
  const bool checked = !config_.expected_cutter_version.empty() || !config_.expected_chassis_version.empty();
  return checked && firmware_ok_ != true;
}

bool McuControl::Lift(double now) const {
  return !TelemetryStale(now) && sensor_->lift;
}

bool McuControl::Stop(double now) const {
  return !TelemetryStale(now) && sensor_->stop;
}

bool McuControl::Bumper(double now) const {
  return !TelemetryStale(now) && (sensor_->bumper || sensor_->bumper_left || sensor_->bumper_right);
}

void McuControl::StartRepeat(Repeat& repeat, const SubPacket& packet, double now) {
  repeat.packet = packet;
  repeat.remaining = config_.repeat_count;
  repeat.next_s = now;
}

void McuControl::EmitRepeat(Repeat& repeat, double now, std::vector<SubPacket>& out) {
  if (repeat.remaining <= 0 || now + kTimeEpsilon < repeat.next_s) return;
  out.push_back(repeat.packet);
  repeat.remaining--;
  repeat.next_s = now + config_.repeat_interval_s;
}

SubPacket McuControl::ZeroSpeed() const {
  return *SpeedPacket(0.0f, 0.0f);
}

SubPacket McuControl::BladeOff() const {
  CutterCommand command;
  command.height_mm = config_.blade_off_height_mm;
  return CutterPacket(command);
}

std::vector<SubPacket> McuControl::Tick(double now, const WallClock& wall) {
  std::vector<SubPacket> out;

  // The blade state of the MCU is unknown when the driver takes over.
  if (!started_) {
    started_ = true;
    // A reconnect resends the off frames but does not restart the time the IMU has.
    if (!start_s_) start_s_ = now;
    StartRepeat(blade_off_repeat_, BladeOff(), now);
    blade_off_since_s_ = now;
  }

  if (!next_heartbeat_s_ || now + kTimeEpsilon >= *next_heartbeat_s_) {
    out.push_back(HeartbeatPacket(wall));
    double next = next_heartbeat_s_ ? *next_heartbeat_s_ + config_.heartbeat_period_s : now;
    next_heartbeat_s_ = next > now + kTimeEpsilon ? next : now + config_.heartbeat_period_s;
  }

  const bool lift = Lift(now);
  const bool stop = Stop(now);
  const bool bumper = config_.bumper_latches_emergency && Bumper(now);
  // Unlike a request, active flags keep a pending release open: the clear frame below is what drops them.
  const uint32_t flags = (lift ? kReasonLift : 0) | (stop ? kReasonStop : 0) | (bumper ? kReasonCollision : 0);
  if (flags) {
    latched_ = true;
    latch_reason_ |= flags;
  }
  // A gap in the IMU latches like the flags; before the first sample it only blocks.
  if (last_imu_sample_s_ && ImuStale(now)) {
    latched_ = true;
    latch_reason_ |= kReasonImuTimeout;
  }

  if (release_requested_) {
    release_requested_ = false;
    bool clear_allowed = !last_clear_s_ || now - *last_clear_s_ + kTimeEpsilon >= config_.clear_min_interval_s;
    if ((lift || stop) && clear_allowed) {
      out.push_back(SensorClearPacket());
      last_clear_s_ = now;
      // The MCU might pick up the last setpoint again once the latch is gone.
      if (last_motion_s_ && now - *last_motion_s_ < config_.clear_zero_after_motion_s) {
        StartRepeat(stop_repeat_, ZeroSpeed(), now);
      }
    }
  }
  if (release_until_s_) {
    if (!lift && !stop && !bumper && !TelemetryStale(now) && !ImuStale(now) && !blade_turning_) {
      latched_ = false;
      latch_reason_ = 0;
      release_until_s_.reset();
    } else if (now > *release_until_s_) {
      release_until_s_.reset();
    }
  }

  const bool blocked = Blocked(now);
  const bool dock_contact = DockContact();

  if (blocked) {
    pending_cmd_.reset();
    if (moving_) {
      moving_ = false;
      StartRepeat(stop_repeat_, ZeroSpeed(), now);
    }
  } else if (pending_cmd_) {
    std::optional<SubPacket> packet;
    if (pending_cmd_->linear != 0.0f || pending_cmd_->angular != 0.0f) {
      packet = SpeedPacket(pending_cmd_->linear, pending_cmd_->angular);
    }
    if (packet) {
      stop_repeat_.remaining = 0;
      out.push_back(*packet);
      moving_ = true;
      last_motion_s_ = now;
    } else if (moving_) {
      moving_ = false;
      StartRepeat(stop_repeat_, ZeroSpeed(), now);
    }
    pending_cmd_.reset();
  }

  if (moving_ && now - last_cmd_s_ > config_.cmd_vel_timeout_s) {
    moving_ = false;
    StartRepeat(stop_repeat_, ZeroSpeed(), now);
  }

  // A blocked or docked mower drops the blade request, it has to be made again afterwards.
  if (blocked) DropBlade(BladeOffReason::kBlocked);
  if (dock_contact) DropBlade(BladeOffReason::kDock);
  // Blade watchdog: a lease that was not renewed in time switches the blade off.
  if (blade_request_ && now + kTimeEpsilon >= blade_request_->until_s) DropBlade(BladeOffReason::kLeaseExpired);
  if (blade_request_ && blade_on_ != blade_request_->setting) {
    CutterCommand command;
    command.blade_on = true;
    command.blade_speed_percent = blade_request_->setting.speed_percent;
    command.height_mm = blade_request_->setting.height_mm;
    blade_off_repeat_.remaining = 0;
    out.push_back(CutterPacket(command));
    blade_on_ = blade_request_->setting;
    blade_off_since_s_.reset();
  } else if (!blade_request_ && blade_on_) {
    blade_on_.reset();
    StartRepeat(blade_off_repeat_, BladeOff(), now);
    blade_off_since_s_ = now;
  }

  if (dock_contact) {
    if (!charge_sent_ || now - last_charge_s_ + kTimeEpsilon >= config_.charge_resend_s) {
      out.push_back(ChargePacket(true));
      charge_sent_ = true;
      last_charge_s_ = now;
    }
  } else {
    charge_sent_ = false;
  }

  if (pending_imu_ && (!last_imu_s_ || now - *last_imu_s_ + kTimeEpsilon >= config_.imu_min_period_s)) {
    out.push_back(ImuPacket(*pending_imu_));
    pending_imu_.reset();
    last_imu_s_ = now;
  }

  EmitRepeat(stop_repeat_, now, out);
  EmitRepeat(blade_off_repeat_, now, out);
  return out;
}

McuControlStatus McuControl::Status(double now) const {
  McuControlStatus status;
  status.blocked = Reported(now);
  status.emergency_reason = EmergencyReason(now);
  status.latched = latched_;
  status.lift = Lift(now);
  status.stop = Stop(now);
  status.bumper = Bumper(now);
  status.telemetry_stale = TelemetryStale(now);
  status.moving = moving_;
  status.blade_on = blade_on_.has_value();
  status.blade_off_reason = blade_off_reason_;
  status.dock_contact = DockContact();
  status.dock_contact_raw = dock_raw_;
  status.firmware_incompatible = FirmwareIncompatible();
  return status;
}

}  // namespace tron
