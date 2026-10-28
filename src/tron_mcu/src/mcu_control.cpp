#include "tron_mcu/mcu_control.h"

#include <algorithm>

namespace tron {

namespace {

// Tolerance for comparing loop times against deadlines.
constexpr double kTimeEpsilon = 1e-6;

}  // namespace

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
  dock_samples_ = contact ? std::min(dock_samples_ + 1, config_.dock_debounce_samples) : 0;
}

void McuControl::OnImu(const ImuCounts& imu) {
  pending_imu_ = imu;
}

void McuControl::LatchEmergency() {
  latched_ = true;
  release_until_s_.reset();
}

void McuControl::ReleaseEmergency(double now) {
  release_requested_ = true;
  release_until_s_ = now + config_.release_window_s;
}

bool McuControl::SetBlade(bool on) {
  if (on && !config_.allow_blade) return false;
  blade_request_ = on;
  return true;
}

void McuControl::OnConnected(double now) {
  started_ = false;
  next_heartbeat_s_.reset();
  pending_cmd_.reset();
  blade_request_ = false;
  blade_on_ = false;
  if (moving_) {
    moving_ = false;
    StartRepeat(stop_repeat_, ZeroSpeed(), now);
  }
}

bool McuControl::TelemetryStale(double now) const {
  return !sensor_ || now - last_sensor_s_ > config_.telemetry_timeout_s;
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
    StartRepeat(blade_off_repeat_, BladeOff(), now);
  }

  if (!next_heartbeat_s_ || now + kTimeEpsilon >= *next_heartbeat_s_) {
    out.push_back(HeartbeatPacket(wall));
    double next = next_heartbeat_s_ ? *next_heartbeat_s_ + config_.heartbeat_period_s : now;
    next_heartbeat_s_ = next > now + kTimeEpsilon ? next : now + config_.heartbeat_period_s;
  }

  const bool lift = Lift(now);
  const bool stop = Stop(now);
  const bool bumper = config_.bumper_latches_emergency && Bumper(now);
  if (lift || stop || bumper) latched_ = true;

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
    if (!lift && !stop && !bumper && !TelemetryStale(now)) {
      latched_ = false;
      release_until_s_.reset();
    } else if (now > *release_until_s_) {
      release_until_s_.reset();
    }
  }

  const bool blocked = latched_ || TelemetryStale(now);
  const bool dock_contact = dock_samples_ >= config_.dock_debounce_samples;

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
  if (blocked || dock_contact) blade_request_ = false;
  if (blade_request_ && !blade_on_) {
    CutterCommand command;
    command.blade_on = true;
    command.blade_speed_percent = config_.blade_speed_percent;
    command.height_mm = config_.mow_height_mm;
    blade_off_repeat_.remaining = 0;
    out.push_back(CutterPacket(command));
    blade_on_ = true;
  } else if (!blade_request_ && blade_on_) {
    blade_on_ = false;
    StartRepeat(blade_off_repeat_, BladeOff(), now);
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
  status.latched = latched_;
  status.lift = Lift(now);
  status.stop = Stop(now);
  status.bumper = Bumper(now);
  status.telemetry_stale = TelemetryStale(now);
  status.moving = moving_;
  status.blade_on = blade_on_;
  status.dock_contact = dock_samples_ >= config_.dock_debounce_samples;
  return status;
}

}  // namespace tron
