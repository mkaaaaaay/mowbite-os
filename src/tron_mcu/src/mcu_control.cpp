#include "tron_mcu/mcu_control.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace tron {

namespace {

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
    case BladeResult::kHeld: return "mower held for an update";
  }
  return "unknown";
}

McuControl::McuControl(const McuControlConfig& config) : config_(config) {
  if (config_.start_latched) Latch(kReasonHighLevel);
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
    // Only a slower report clears a turning blade, also in the new run-down after a reconnect.
    if (std::abs(rpm) < config_.blade_still_rpm) blade_turning_ = false;
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
  Latch(reason);
  release_until_s_.reset();
}

void McuControl::Latch(uint32_t reason, bool rising) {
  // STOP counts by its debounced press in UpdateStopRelease, so a bounce is no new reason.
  if (rising || (reason & ~latch_reason_ & ~kReasonStop)) CountReason();
  latched_ = true;
  latch_reason_ |= reason;
}

void McuControl::CountReason() {
  if (++latch_count_ == 0) latch_count_ = 1;
  new_reason_ = true;
}

bool McuControl::ReleaseEmergency(double now, uint32_t latch_count) {
  if (latch_count != 0 && latch_count != latch_count_) return false;
  release_requested_ = true;
  release_until_s_ = now + config_.release_window_s;
  release_count_ = latch_count_;
  return true;
}

BladeResult McuControl::SetBlade(const BladeRequest& request, double now, uint32_t* emergency_reason) {
  if (emergency_reason) *emergency_reason = 0;
  if (!request.on) {
    DropBlade(BladeOffReason::kRequest);
    return BladeResult::kOk;
  }
  if (!config_.allow_blade) return BladeResult::kNotAllowed;
  if (!std::isfinite(request.lease_s) || request.lease_s <= 0 ||
      request.lease_s > config_.blade_lease_max_s + kDeadlineSlackS) {
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
    // An emergency says more than the hold.
    if (held_ && !Reported(now)) return BladeResult::kHeld;
    // 0 while the IMU is still due after the start: blocked, but no emergency.
    if (emergency_reason) *emergency_reason = EmergencyReason(now);
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

void McuControl::SetCharging(bool allowed, double now) {
  charge_sent_ = false;
  charge_stopped_ = !allowed;
  if (allowed) {
    charge_off_repeat_.remaining = 0;
    return;
  }
  StartRepeat(charge_off_repeat_, ChargePacket(false), now);
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
    StartRepeat(stop_repeat_, ZeroSpeedPacket(), now);
  }
  stop_release_.Interrupt();
}

void McuControl::SetHold(bool held) {
  held_ = held;
}

void McuControl::SetStopClearMark(bool written) {
  stop_clear_mark_ = written;
}

bool McuControl::StopClearReady(double now) const {
  return config_.stop_release && config_.stop_clear_period_s > 0 && latched_ && latched_since_s_ &&
         now - *latched_since_s_ + kDeadlineSlackS >= config_.stop_clear_delay_s && !release_until_s_ &&
         !blade_on_ && !blade_turning_ && !TelemetryStale(now) && !ImuStale(now) && !NoImuYet();
}

bool McuControl::Blocked(double now) const {
  return held_ || latched_ || TelemetryStale(now) || NoImuYet() || FirmwareIncompatible();
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
  return NoImuYet() && (!start_s_ || now - *start_s_ <= start_time + kDeadlineSlackS);
}

bool McuControl::FirmwareIncompatible() const {
  const bool checked = !config_.expected_cutter_version.empty() || !config_.expected_chassis_version.empty();
  return checked && firmware_ok_ != true;
}

bool McuControl::Lift(double now) const {
  return sensor_ && !TelemetryStale(now) && sensor_->lift;
}

bool McuControl::Stop(double now) const {
  return sensor_ && !TelemetryStale(now) && sensor_->stop;
}

bool McuControl::Bumper(double now) const {
  return sensor_ && !TelemetryStale(now) && (sensor_->bumper || sensor_->bumper_left || sensor_->bumper_right);
}

void McuControl::StartRepeat(Repeat& repeat, const SubPacket& packet, double now) {
  repeat.packet = packet;
  repeat.remaining = config_.repeat_count;
  repeat.next_s = now;
}

void McuControl::EmitRepeat(Repeat& repeat, double now, std::vector<SubPacket>& out) {
  if (repeat.remaining <= 0 || now + kDeadlineSlackS < repeat.next_s) return;
  out.push_back(repeat.packet);
  repeat.remaining--;
  repeat.next_s = now + config_.repeat_interval_s;
}

// Silence on cmd_vel after which the wheels get the stop.
double McuControl::StopAfter() const {
  if (config_.speed_repeat_window_s <= 0) return config_.cmd_vel_timeout_s;
  return std::min(config_.speed_repeat_window_s, config_.cmd_vel_timeout_s);
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
    if (!start_s_) {
      start_s_ = now;
      // The MCU might still run the last setpoint of the driver before this one.
      if (config_.start_latched) StartRepeat(stop_repeat_, ZeroSpeedPacket(), now);
    }
    StartRepeat(blade_off_repeat_, BladeOff(), now);
    blade_off_since_s_ = now;
  }

  if (!next_heartbeat_s_ || now + kDeadlineSlackS >= *next_heartbeat_s_) {
    out.push_back(HeartbeatPacket(wall));
    double next = next_heartbeat_s_ ? *next_heartbeat_s_ + config_.heartbeat_period_s : now;
    next_heartbeat_s_ = next > now + kDeadlineSlackS ? next : now + config_.heartbeat_period_s;
  }

  const bool latched_before = latched_;
  const bool lift = Lift(now);
  const bool stop = Stop(now);
  const bool bumper = config_.bumper_latches_emergency && Bumper(now);
  // Lift and bumper count again when they rise, also with their bit still latched.
  const bool rising = (lift && !last_lift_) || (bumper && !last_bumper_);
  last_lift_ = lift;
  last_bumper_ = bumper;
  // Unlike a request, active flags keep a pending release open: the clear frame below is what drops them.
  const uint32_t flags = (lift ? kReasonLift : 0) | (stop ? kReasonStop : 0) | (bumper ? kReasonCollision : 0);
  if (flags) Latch(flags, rising);
  // A gap in the IMU latches like the flags; before the first sample it only blocks.
  if (last_imu_sample_s_ && ImuStale(now)) Latch(kReasonImuTimeout);
  UpdateStopRelease(stop, lift, latched_before, now);

  // A new reason since the request ends it without effect, also one that came between the request and this tick.
  if (latch_count_ != release_count_) {
    release_requested_ = false;
    release_until_s_.reset();
  }
  if (release_requested_) {
    release_requested_ = false;
    bool clear_allowed = !last_clear_s_ || now - *last_clear_s_ + kDeadlineSlackS >= config_.clear_min_interval_s;
    if ((lift || stop) && clear_allowed) {
      out.push_back(SensorClearPacket());
      last_clear_s_ = now;
      // The MCU might pick up the last setpoint again once the latch is gone.
      if (last_motion_s_ && now - *last_motion_s_ < config_.clear_zero_after_motion_s) {
        StartRepeat(stop_repeat_, ZeroSpeedPacket(), now);
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
  if (!latched_) {
    latched_since_s_.reset();
  } else if (!latched_since_s_) {
    latched_since_s_ = now;
  }
  // STOP kept by the MCU during the emergency: cleared, so that letting go and a new press show (G37).
  if (stop && stop_clear_mark_ && StopClearReady(now) &&
      (!last_stop_clear_s_ || now - *last_stop_clear_s_ + kDeadlineSlackS >= config_.stop_clear_period_s)) {
    // The MCU might pick up the last setpoint again once its latch is gone.
    if (last_motion_s_ && now - *last_motion_s_ < config_.clear_zero_after_motion_s) out.push_back(ZeroSpeedPacket());
    out.push_back(SensorClearPacket());
    last_stop_clear_s_ = now;
  }

  const bool blocked = Blocked(now);
  const bool dock_contact = DockContact();

  // Charging stopped on request stays off while this contact lasts; its off frames go out before any motion.
  if (!dock_contact) charge_stopped_ = false;
  EmitRepeat(charge_off_repeat_, now, out);

  if (blocked) {
    pending_cmd_.reset();
    if (moving_) {
      moving_ = false;
      StartRepeat(stop_repeat_, ZeroSpeedPacket(), now);
    }
  } else if (pending_cmd_) {
    std::optional<SubPacket> packet;
    if (pending_cmd_->linear != 0.0f || pending_cmd_->angular != 0.0f) {
      packet = SpeedPacket(pending_cmd_->linear, pending_cmd_->angular);
    }
    if (!packet) {
      if (moving_) {
        moving_ = false;
        StartRepeat(stop_repeat_, ZeroSpeedPacket(), now);
      }
      pending_cmd_.reset();
    } else if (now + kDeadlineSlackS >= next_speed_s_) {
      stop_repeat_.remaining = 0;
      out.push_back(*packet);
      moving_ = true;
      last_motion_s_ = now;
      last_speed_ = *packet;
      next_speed_s_ = now + config_.speed_period_s;
      pending_cmd_.reset();
    }
  }

  if (moving_ && now - last_cmd_s_ > StopAfter()) {
    moving_ = false;
    StartRepeat(stop_repeat_, ZeroSpeedPacket(), now);
  }
  if (moving_ && last_speed_ && config_.speed_repeat_window_s > 0 && now + kDeadlineSlackS >= next_speed_s_) {
    out.push_back(*last_speed_);
    last_motion_s_ = now;
    next_speed_s_ = now + config_.speed_period_s;
  }

  // A blocked or docked mower drops the blade request, it has to be made again afterwards.
  if (blocked) DropBlade(BladeOffReason::kBlocked);
  if (dock_contact) DropBlade(BladeOffReason::kDock);
  // Blade watchdog: a lease that was not renewed in time switches the blade off.
  if (blade_request_ && now + kDeadlineSlackS >= blade_request_->until_s) DropBlade(BladeOffReason::kLeaseExpired);
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

  if (dock_contact && !charge_stopped_) {
    if (!charge_sent_ || now - last_charge_s_ + kDeadlineSlackS >= config_.charge_resend_s) {
      out.push_back(ChargePacket(true));
      charge_sent_ = true;
      last_charge_s_ = now;
    }
  } else {
    charge_sent_ = false;
  }

  if (pending_imu_ && (!last_imu_s_ || now - *last_imu_s_ + kDeadlineSlackS >= config_.imu_min_period_s)) {
    out.push_back(ImuPacket(*pending_imu_));
    pending_imu_.reset();
    last_imu_s_ = now;
  }

  const size_t before_stop = out.size();
  EmitRepeat(stop_repeat_, now, out);
  if (out.size() > before_stop) next_speed_s_ = now + config_.speed_period_s;
  EmitRepeat(blade_off_repeat_, now, out);
  return out;
}

void McuControl::UpdateStopRelease(bool stop, bool lift, bool latched_before, double now) {
  if (stop_release_.Sample(stop, now) && ++latch_count_ == 0) latch_count_ = 1;
  StopReleaseInput input;
  input.latched = latched_before;
  input.usable = !TelemetryStale(now) && !ImuStale(now) && !NoImuYet();
  input.lift = lift;
  input.new_reason = new_reason_;
  new_reason_ = false;
  if (!config_.stop_release) return;
  const StopPress press = stop_release_.Update(input, now);
  if (press == StopPress::kNone) return;
  ++stop_event_.seq;
  stop_event_.press = press;
  stop_event_.latch_count = press == StopPress::kLong ? latch_count_ : 0;
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
  status.held = held_;
  status.latch_count = latch_count_;
  status.stop_event = stop_event_;
  status.stop_clear_mark_wanted = !stop_clear_mark_ && Stop(now) && StopClearReady(now);
  return status;
}

std::array<McuInput, 4> McuControl::Inputs(double now) const {
  const bool fresh = sensor_ && !TelemetryStale(now);
  const char* collision = config_.bumper_latches_emergency ? "collision" : "";
  return {{{"stop", "stop", true, Stop(now)},
           {"lift", "lift", true, Lift(now)},
           {"bumper_left", collision, false, fresh && sensor_->bumper_left},
           {"bumper_right", collision, false, fresh && sensor_->bumper_right}}};
}

}  // namespace tron
