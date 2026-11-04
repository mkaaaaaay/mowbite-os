#include "xcore_driver/xcore_control.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "xcore_driver/services.h"

namespace xcore {

namespace {

constexpr double kTimeEpsilon = 1e-6;
constexpr double kDegreesPerRadian = 180.0 / M_PI;

double Clamp(double value, double limit) {
  if (!std::isfinite(value)) return 0;
  return std::clamp(value, -limit, limit);
}

std::optional<float> F32(const Bytes& value) {
  if (value.size() != 4) return std::nullopt;
  return Reader(value).F32();
}

std::optional<uint8_t> U8(const Bytes& value) {
  if (value.size() != 1) return std::nullopt;
  return value[0];
}

}  // namespace

const char* BladeResultText(BladeResult result) {
  switch (result) {
    case BladeResult::kOk:
      return "ok";
    case BladeResult::kNotAllowed:
      return "blade not allowed by the configuration";
    case BladeResult::kInvalidLease:
      return "lease out of range";
    case BladeResult::kInvalidSpeed:
      return "speed out of range";
    case BladeResult::kInvalidHeight:
      return "this mower has no height motor, height must be 0";
    case BladeResult::kBlocked:
      return "blocked by the emergency";
    case BladeResult::kDocked:
      return "on the dock";
  }
  return "unknown";
}

int8_t MotorStatusCode(uint8_t esc_status) {
  switch (esc_status) {
    case kEscOk:
      return 0;
    case kEscRunning:
      return 1;
    case kEscBlocked:
      return 2;
    case kEscFault:
      return -1;
    case kEscDisconnected:
      return -2;
    default:
      return -3;
  }
}

XcoreControl::XcoreControl(const ControlConfig& config, const InputMap& inputs)
    : config_(config), inputs_(inputs), odometry_(config.odometry), regulator_(config.regulator) {
}

void XcoreControl::OnCmdVel(double linear, double angular, double now) {
  Command command{Clamp(linear, config_.max_linear_speed), Clamp(angular, config_.max_angular_speed)};
  last_cmd_s_ = now;
  if (command.linear == 0 && command.angular == 0) {
    if (moving_) stop_repeat_ = Repeat{config_.repeat_count, now};
    moving_ = false;
    command_.reset();
    return;
  }
  if (Blocked(now)) return;
  bool changed = !command_ || command_->linear != command.linear || command_->angular != command.angular;
  command_ = command;
  stop_repeat_.remaining = 0;
  // A new setpoint goes out with the next tick.
  if (changed) next_drive_s_.reset();
}

BladeResult XcoreControl::SetBlade(const BladeRequest& request, double now, uint32_t* emergency_reason) {
  if (emergency_reason) *emergency_reason = 0;
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
  if (request.height_mm != 0) return BladeResult::kInvalidHeight;
  // A refused renewal also ends a running lease, so the blade never outlives a refusal.
  if (Blocked(now)) {
    DropBlade(BladeOffReason::kBlocked);
    if (emergency_reason) *emergency_reason = Reason(now);
    return BladeResult::kBlocked;
  }
  if (DockContact()) {
    DropBlade(BladeOffReason::kDock);
    return BladeResult::kDocked;
  }
  uint16_t percent = request.speed_percent ? request.speed_percent : config_.blade_speed_percent;
  float speed = std::min<uint16_t>(percent, 100) / 100.0f;
  if (!blade_ || blade_->speed != speed) next_blade_s_.reset();
  blade_ = BladeLease{speed, now + request.lease_s};
  blade_off_repeat_.remaining = 0;
  blade_off_since_s_.reset();
  return BladeResult::kOk;
}

bool XcoreControl::LatchEmergency(uint32_t bits) {
  if (bits == 0) bits = kReasonHighLevel;
  if (bits & ~kHostLatchReasons) return false;
  host_latch_ |= bits;
  clear_remaining_ = 0;
  // The set bit goes out with the next tick.
  next_emergency_s_.reset();
  return true;
}

std::optional<ReleaseRefusal> XcoreControl::ReleaseEmergency(double now) {
  if (tilt_latched_) {
    ReleaseRefusal refusal;
    if (!last_imu_s_ || now - *last_imu_s_ > config_.imu_fresh_s + kTimeEpsilon) {
      refusal.code = "TILT_UNKNOWN";
      refusal.message = "tilt latched and no fresh IMU sample to check the mower stands level";
      if (last_imu_s_) refusal.value = now - *last_imu_s_;
      return refusal;
    }
    if (tilt_deg_ >= config_.tilt_release_deg) {
      char text[120];
      std::snprintf(text, sizeof(text), "still tilted %.0f deg, a release needs less than %.0f deg", tilt_deg_,
                    config_.tilt_release_deg);
      refusal.code = "STILL_TILTED";
      refusal.message = text;
      refusal.value = tilt_deg_;
      refusal.limit = config_.tilt_release_deg;
      return refusal;
    }
  }
  tilt_latched_ = false;
  tilted_since_s_.reset();
  fault_bits_ = 0;
  uncommanded_since_s_.reset();
  overspeed_since_s_.reset();
  host_latch_ = 0;
  latched_causes_ = 0;
  if (!blade_turning_) mower_not_stopped_ = false;
  link_lost_ = false;
  stalled_ = false;
  drive_fault_ = false;
  regulator_.ClearFault();
  clear_remaining_ = config_.repeat_count;
  next_emergency_s_ = now;
  return std::nullopt;
}

void XcoreControl::Fault(uint32_t bit, const std::string& why, double now) {
  if (fault_bits_ & bit) return;
  fault_bits_ |= bit;
  errors_.push_back(why);
  // The wheels may turn without a setpoint of the driver: stops go out in any case, the latch with the next tick.
  stop_repeat_ = Repeat{config_.repeat_count, now};
  next_emergency_s_.reset();
}

void XcoreControl::LatchForeignHost(const std::string& why, double now) {
  Fault(kReasonForeignHost, why, now);
}

void XcoreControl::OnImu(double ax, double ay, double az, double now) {
  if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) return;
  // The angle between the body's z axis and gravity; past 90 deg the mower lies on its back.
  tilt_deg_ = std::atan2(std::hypot(ax, ay), az) * kDegreesPerRadian;
  last_imu_s_ = now;
  if (tilt_deg_ <= config_.tilt_limit_deg) {
    tilted_since_s_.reset();
    return;
  }
  if (!tilted_since_s_) tilted_since_s_ = now;
  if (tilt_latched_ || now - *tilted_since_s_ < config_.tilt_time_s - kTimeEpsilon) return;
  tilt_latched_ = true;
  char text[120];
  std::snprintf(text, sizeof(text), "tilted %.0f deg for %.1f s (limit %.0f deg): latched TILT", tilt_deg_,
                now - *tilted_since_s_, config_.tilt_limit_deg);
  errors_.push_back(text);
  next_emergency_s_.reset();
}

void XcoreControl::CheckWheels(const WheelTwist& twist, double now) {
  const double fastest = std::max(std::fabs(twist.left), std::fabs(twist.right));
  const bool watched = last_drive_s_ && now - *last_drive_s_ <= config_.uncommanded_window_s + kTimeEpsilon;
  if (!watched) {
    uncommanded_since_s_.reset();
    overspeed_since_s_.reset();
    return;
  }
  char text[160];
  const double limit = config_.overspeed_factor *
                       (config_.max_linear_speed + config_.max_angular_speed * config_.odometry.wheel_distance / 2);
  if (fastest > limit) {
    if (!overspeed_since_s_) overspeed_since_s_ = now;
    if (now - *overspeed_since_s_ >= config_.overspeed_time_s - kTimeEpsilon) {
      std::snprintf(text, sizeof(text), "wheel at %.2f m/s for %.1f s, above %.2f m/s: duty and speed do not fit",
                    fastest, now - *overspeed_since_s_, limit);
      Fault(kReasonWheelRunaway, text, now);
    }
  } else {
    overspeed_since_s_.reset();
  }
  const bool idle = !command_ && now - *last_drive_s_ >= config_.uncommanded_after_s - kTimeEpsilon;
  if (idle && fastest > config_.uncommanded_speed) {
    if (!uncommanded_since_s_) uncommanded_since_s_ = now;
    if (now - *uncommanded_since_s_ >= config_.uncommanded_time_s - kTimeEpsilon) {
      std::snprintf(text, sizeof(text), "wheel turns at %.2f m/s %.1f s after the last drive setpoint: no stop",
                    fastest, now - *last_drive_s_);
      Fault(kReasonWheelRunaway, text, now);
    }
  } else {
    uncommanded_since_s_.reset();
  }
}

void XcoreControl::CheckStall(double now) {
  const bool stalled = last_alive_s_ && now - *last_alive_s_ > config_.io_stall_s + kTimeEpsilon;
  if (stalled && !link_lost_) {
    char text[120];
    std::snprintf(text, sizeof(text), "I/O thread stalled for %.2f s: link counted as lost, setpoints dropped",
                  now - *last_alive_s_);
    errors_.push_back(text);
    link_lost_ = true;
    stalled_ = true;
    if (command_ || moving_) stop_repeat_ = Repeat{config_.repeat_count, now};
    command_.reset();
    moving_ = false;
    DropBlade(BladeOffReason::kBlocked);
    next_emergency_s_.reset();
  }
  last_alive_s_ = now;
}

void XcoreControl::Alive(double now) {
  last_alive_s_ = now;
}

std::vector<std::string> XcoreControl::TakeErrors() {
  std::vector<std::string> out;
  out.swap(errors_);
  return out;
}

void XcoreControl::SetServicesReady(bool ready, bool firmware_compatible) {
  services_ready_ = ready;
  firmware_compatible_ = firmware_compatible;
}

std::optional<WheelTwist> XcoreControl::OnOutput(uint16_t service_id, uint16_t channel_id, const Bytes& value,
                                                  uint64_t board_time_us, double now) {
  switch (service_id) {
    case kEmergencyService:
      if (channel_id == emergency::kOutReason && value.size() == 2) {
        board_reason_ = Reader(value).U16();
        last_emergency_s_ = now;
        if (*board_reason_ & reason::kLatch) latched_causes_ |= *board_reason_ & kLatchedCauses;
        // The board stops without the host's messages but lets go again once they come; the host keeps it stopped.
        if (*board_reason_ & reason::kTimeoutHighLevel) {
          if (board_heard_us_) link_lost_ = true;
        } else {
          board_heard_us_ = true;
        }
      }
      break;
    case kDiffDriveService: {
      bool left = channel_id <= diff_drive::kOutLeftCurrent;
      MotorReport& motor = left ? left_motor_ : right_motor_;
      switch (channel_id) {
        case diff_drive::kOutTicks:
          if (value.size() == 8) {
            Reader r(value);
            uint32_t l = r.U32();
            uint32_t rt = r.U32();
            auto twist = odometry_.OnTicks(l, rt, board_time_us);
            if (twist) {
              regulator_.OnMeasured(twist->left, twist->right, now);
              CheckWheels(*twist, now);
            }
            return twist;
          }
          break;
        case diff_drive::kOutLeftStatus:
        case diff_drive::kOutRightStatus:
          if (auto v = U8(value)) {
            motor.status = *v;
            motor.known = true;
          }
          break;
        case diff_drive::kOutLeftTemperature:
        case diff_drive::kOutRightTemperature:
          if (auto v = F32(value)) motor.temperature = *v;
          break;
        case diff_drive::kOutLeftCurrent:
        case diff_drive::kOutRightCurrent:
          if (auto v = F32(value)) motor.current = *v;
          break;
        default:
          break;
      }
      break;
    }
    case kMowerService:
      switch (channel_id) {
        case mower::kOutStatus:
          if (auto v = U8(value)) {
            blade_motor_.status = *v;
            blade_motor_.known = true;
          }
          break;
        case mower::kOutRain:
          if (auto v = U8(value)) rain_ = *v != 0;
          break;
        case mower::kOutMotorTemperature:
          if (auto v = F32(value)) blade_motor_.temperature = *v;
          break;
        case mower::kOutCurrent:
          if (auto v = F32(value)) blade_motor_.current = *v;
          break;
        case mower::kOutRpm:
          if (auto v = F32(value)) {
            blade_motor_.rpm = *v;
            bool settled = blade_off_since_s_ && now - *blade_off_since_s_ >= config_.blade_stop_time_s - kTimeEpsilon;
            if (!blade_ && settled && std::fabs(*v) >= config_.blade_still_rpm) {
              blade_turning_ = true;
              if (!mower_not_stopped_) blade_off_repeat_ = Repeat{config_.repeat_count, now};
              mower_not_stopped_ = true;
            } else if (std::fabs(*v) < config_.blade_still_rpm) {
              blade_turning_ = false;
            }
          }
          break;
        default:
          break;
      }
      break;
    case kPowerService:
      switch (channel_id) {
        case power::kOutBatteryVoltage:
          if (auto v = F32(value)) {
            battery_.voltage = *v;
            battery_.known = true;
            last_battery_s_ = now;
          }
          break;
        case power::kOutChargeVoltage:
          if (auto v = F32(value)) {
            battery_.charge_voltage = *v;
            bool raw = *v >= config_.dock_voltage;
            if (raw != dock_raw_) dock_samples_ = 0;
            dock_raw_ = raw;
            dock_samples_ = std::min(dock_samples_ + 1, config_.dock_debounce_samples);
          }
          break;
        case power::kOutChargeCurrent:
          if (auto v = F32(value)) battery_.charge_current = *v;
          break;
        case power::kOutBatteryPercentage:
          if (auto v = F32(value)) battery_.fraction = std::clamp<double>(*v, 0, 1);
          break;
        case power::kOutChargingStatus:
          battery_.charging_status = DecodeText(value);
          break;
        default:
          break;
      }
      break;
    case kInputService:
      if (channel_id == input::kOutActive && value.size() == 8) active_inputs_ = Reader(value).U64();
      break;
    default:
      break;
  }
  return std::nullopt;
}

void XcoreControl::OnServiceRestart(uint16_t service_id, double now) {
  switch (service_id) {
    case kEmergencyService:
      board_reason_.reset();
      last_emergency_s_.reset();
      next_emergency_s_.reset();
      board_heard_us_ = false;
      break;
    case kDiffDriveService:
      left_motor_ = MotorReport{};
      right_motor_ = MotorReport{};
      odometry_.Reset();
      regulator_.Reset();
      if (moving_) stop_repeat_ = Repeat{config_.repeat_count, now};
      moving_ = false;
      command_.reset();
      break;
    case kMowerService:
      blade_motor_ = MotorReport{};
      DropBlade(BladeOffReason::kReconnect);
      blade_off_repeat_ = Repeat{config_.repeat_count, now};
      break;
    case kPowerService:
      battery_ = Battery{};
      last_battery_s_.reset();
      dock_samples_ = 0;
      dock_raw_ = false;
      break;
    case kInputService:
      active_inputs_.reset();
      break;
    default:
      break;
  }
}

bool XcoreControl::Due(std::optional<double>& next, double period, double now) {
  if (next && now + kTimeEpsilon < *next) return false;
  next = now + period;
  return true;
}

InputValue XcoreControl::Twist(double linear, double angular) const {
  // The board turns index 0 and 5 into the duty of each wheel.
  return InputValue{kDiffDriveService, diff_drive::kInTwist, EncodeDoubles({linear, 0, 0, 0, 0, angular})};
}

InputValue XcoreControl::MowerSpeed(float speed) const {
  Writer w;
  w.F32(speed);
  return InputValue{kMowerService, mower::kInSpeed, w.Take()};
}

std::vector<InputValue> XcoreControl::Tick(double now) {
  std::vector<InputValue> out;
  // Before anything goes out: after a stall of this thread the board may have stopped on its own.
  CheckStall(now);
  if (regulator_.fault() && !drive_fault_) {
    drive_fault_ = true;
    errors_.push_back("wheel turns against the drive command: latched");
  }
  bool blocked = Blocked(now);

  // Emergency: the host's latch, a pending release, and in any case the heartbeat for the board.
  if (Due(next_emergency_s_, config_.emergency_period_s, now)) {
    uint16_t set = HoldsBoard() ? reason::kHighLevel | reason::kLatch : 0;
    // A board that lost its latch in a restart gets it back.
    if (!set && latched_causes_) set = reason::kLatch;
    uint16_t clear = 0;
    if (clear_remaining_ > 0 && !set) {
      // The board computes the other bits itself on every pass.
      clear = reason::kLatch | reason::kHighLevel | reason::kMowerRpmTimeout | reason::kMowerRpmLimit;
      clear_remaining_--;
      if (clear_remaining_ > 0) next_emergency_s_ = now + config_.repeat_interval_s;
    }
    Writer w;
    w.U16(set);
    w.U16(clear);
    out.push_back(InputValue{kEmergencyService, emergency::kInHighLevel, w.Take()});
  }

  // Drive: setpoints while commanded, a few stops after, then silence.
  if (command_ && (blocked || now - last_cmd_s_ > config_.cmd_vel_timeout_s + kTimeEpsilon)) {
    command_.reset();
    regulator_.Reset();
    if (moving_) stop_repeat_ = Repeat{config_.repeat_count, now};
    moving_ = false;
  }
  if (command_) {
    if (Due(next_drive_s_, config_.drive_period_s, now)) {
      DutyTwist duty = regulator_.Update(command_->linear, command_->angular, now);
      out.push_back(Twist(duty.linear, duty.angular));
      if (duty.linear != 0 || duty.angular != 0) last_drive_s_ = now;
      moving_ = true;
    }
  } else if (stop_repeat_.remaining > 0 && now + kTimeEpsilon >= stop_repeat_.next_s) {
    out.push_back(Twist(0, 0));
    stop_repeat_.remaining--;
    stop_repeat_.next_s = now + config_.repeat_interval_s;
  }

  // Blade: the lease, then the setpoint again and again, a few quick offs after it ends.
  if (blade_ && now + kTimeEpsilon >= blade_->until_s) DropBlade(BladeOffReason::kLeaseExpired);
  if (blade_ && blocked) DropBlade(BladeOffReason::kBlocked);
  if (blade_ && DockContact()) DropBlade(BladeOffReason::kDock);
  if (blade_) {
    if (Due(next_blade_s_, config_.blade_period_s, now)) out.push_back(MowerSpeed(blade_->speed));
  } else if (blade_off_repeat_.remaining > 0 && now + kTimeEpsilon >= blade_off_repeat_.next_s) {
    out.push_back(MowerSpeed(0));
    blade_off_repeat_.remaining--;
    blade_off_repeat_.next_s = now + config_.repeat_interval_s;
    next_blade_s_ = now + config_.blade_period_s;
    if (!blade_off_since_s_) blade_off_since_s_ = now;
  } else if (Due(next_blade_s_, config_.blade_period_s, now)) {
    out.push_back(MowerSpeed(0));
    if (!blade_off_since_s_) blade_off_since_s_ = now;
  }

  return out;
}

std::vector<InputValue> XcoreControl::StopInputs() const {
  return {Twist(0, 0), MowerSpeed(0)};
}

void XcoreControl::DropBlade(BladeOffReason reason) {
  if (!blade_) return;
  blade_off_reason_ = reason;
  blade_.reset();
  blade_off_repeat_ = Repeat{config_.repeat_count, 0};
  blade_off_since_s_.reset();
}

bool XcoreControl::TelemetryStale(double now) const {
  return !last_emergency_s_ || now - *last_emergency_s_ > config_.telemetry_timeout_s;
}

bool XcoreControl::Blocked(double now) const {
  return Reason(now) != 0;
}

bool XcoreControl::HoldsBoard() const {
  return host_latch_ || mower_not_stopped_ || link_lost_ || drive_fault_ || fault_bits_ || tilt_latched_;
}

uint32_t XcoreControl::Reason(double now) const {
  uint32_t bits = host_latch_;
  if (board_reason_) {
    uint32_t board = *board_reason_ & kBoardReasons;
    // The board echoes the HIGH_LEVEL the driver sends: the driver's own bits say why, HIGH_LEVEL stays for a latch
    // from above with it.
    if (HoldsBoard() && !(host_latch_ & kReasonHighLevel)) board &= ~kReasonHighLevel;
    bits |= board;
  }
  if (mower_not_stopped_) bits |= kReasonMowerNotStopped | kReasonLatch;
  if (stalled_) {
    bits |= kReasonDriverStall | kReasonLatch;
  } else if (link_lost_) {
    bits |= reason::kTimeoutHighLevel | kReasonLatch;
  }
  if (drive_fault_) bits |= kReasonWheelRunaway | kReasonLatch;
  if (fault_bits_) bits |= fault_bits_ | kReasonLatch;
  if (tilt_latched_) bits |= kReasonTilt | kReasonLatch;
  if (host_latch_ || latched_causes_) bits |= kReasonLatch;
  bits |= latched_causes_;
  if (TelemetryStale(now)) bits |= kReasonTimeoutInputs;
  if (!services_ready_) bits |= reason::kServiceNotReady;
  if (!firmware_compatible_) bits |= kReasonFirmwareIncompatible;
  return bits;
}

bool XcoreControl::InputActive(const std::vector<int>& indices) const {
  if (!active_inputs_) return false;
  for (int i : indices) {
    if (i >= 0 && i < 64 && (*active_inputs_ >> i) & 1u) return true;
  }
  return false;
}

bool XcoreControl::DockContact() const {
  return dock_raw_ && dock_samples_ >= config_.dock_debounce_samples;
}

ControlStatus XcoreControl::Status(double now) const {
  ControlStatus s;
  s.emergency_reason = Reason(now);
  s.blocked = s.emergency_reason != 0;
  s.latched = (s.emergency_reason & kReasonLatch) != 0;
  s.telemetry_ok = !TelemetryStale(now);
  s.lift = InputActive(inputs_.lift);
  s.stop = InputActive(inputs_.stop);
  s.bumper_left = InputActive(inputs_.bumper_left);
  s.bumper_right = InputActive(inputs_.bumper_right);
  s.rain = rain_;
  s.dock_contact = DockContact();
  s.dock_contact_raw = dock_raw_;
  s.moving = moving_;
  s.blade_on = blade_.has_value();
  s.blade_off_reason = blade_off_reason_;
  if (last_battery_s_ && now - *last_battery_s_ <= config_.battery_timeout_s) s.battery = battery_;
  s.blade_motor = blade_motor_;
  s.left_motor = left_motor_;
  s.right_motor = right_motor_;
  return s;
}

}  // namespace xcore
