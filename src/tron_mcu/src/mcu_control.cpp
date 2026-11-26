#include "tron_mcu/mcu_control.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <tuple>

namespace tron {

namespace {

bool VersionListed(const std::vector<std::string>& listed, const Version& version) {
  return listed.empty() || std::find(listed.begin(), listed.end(), VersionString(version)) != listed.end();
}

bool IsMotionSpeed(const SubPacket& packet) {
  return packet.type == static_cast<uint8_t>(PacketType::kHostToMcu) &&
         packet.module == static_cast<uint8_t>(Module::kSpeed) && packet.payload != ZeroSpeedPacket().payload;
}

bool Older(const Version& a, const Version& b) {
  return std::tie(a.major, a.minor, a.patch) < std::tie(b.major, b.minor, b.patch);
}

// Older than every stand of a non-empty list; an unreadable entry counts as older than anything.
bool OlderThanListed(const std::vector<std::string>& listed, const Version& version) {
  if (listed.empty()) return false;
  for (const std::string& entry : listed) {
    const std::optional<Version> stand = ParseVersion(entry);
    if (stand && !Older(version, *stand)) return false;
  }
  return true;
}

bool LowerHex(const std::string& text, size_t length) {
  return text.size() == length &&
         std::all_of(text.begin(), text.end(), [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}

// Both wheels scaled alike so that the faster one stays within max_wheel and the curve keeps its radius.
void CapWheels(float& linear, float& angular, float max_wheel) {
  const float fastest = std::abs(linear) + std::abs(angular) * kHalfTrack;
  if (!(fastest > max_wheel)) return;
  const float scale = max_wheel / fastest;
  linear *= scale;
  angular *= scale;
}

}  // namespace

const char* FirmwareSourceText(FirmwareSource source) {
  switch (source) {
    case FirmwareSource::kUnknown: return "";
    case FirmwareSource::kListed: return "listed";
    case FirmwareSource::kMeasured: return "measured";
    case FirmwareSource::kOlder: return "older";
    case FirmwareSource::kNone: return "none";
  }
  return "";
}

std::optional<Version> ParseVersion(const std::string& text) {
  int parts[3];
  size_t at = 0;
  for (int i = 0; i < 3; ++i) {
    if (i > 0) {
      if (at >= text.size() || text[at] != '.') return std::nullopt;
      ++at;
    }
    const size_t begin = at;
    int value = 0;
    while (at < text.size() && text[at] >= '0' && text[at] <= '9' && at - begin < 3) {
      value = value * 10 + (text[at] - '0');
      ++at;
    }
    if (at == begin || value > 255) return std::nullopt;
    parts[i] = value;
  }
  if (at != text.size()) return std::nullopt;
  return Version{static_cast<uint8_t>(parts[0]), static_cast<uint8_t>(parts[1]), static_cast<uint8_t>(parts[2])};
}

std::vector<FirmwarePair> MeasuredFirmware(const std::vector<std::string>& entries, const std::string& device_hash,
                                           std::vector<std::string>* malformed, int* other_devices) {
  std::vector<FirmwarePair> pairs;
  if (other_devices) *other_devices = 0;
  for (const std::string& entry : entries) {
    const size_t first = entry.find('/');
    const size_t second = first == std::string::npos ? first : entry.find('/', first + 1);
    const bool three = second != std::string::npos && entry.find('/', second + 1) == std::string::npos;
    const std::string cutter = three ? entry.substr(0, first) : "";
    const std::string chassis = three ? entry.substr(first + 1, second - first - 1) : "";
    const std::string hash = three ? entry.substr(second + 1) : "";
    if (!three || !ParseVersion(cutter) || !ParseVersion(chassis) || !LowerHex(hash, 64)) {
      if (malformed) malformed->push_back(entry);
      continue;
    }
    if (device_hash.empty() || hash != device_hash) {
      if (other_devices) ++*other_devices;
      continue;
    }
    pairs.push_back({cutter, chassis});
  }
  return pairs;
}

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
    case BladeResult::kMotorFault: return "blade motor fault";
  }
  return "unknown";
}

const char* HostTestRefusalCode(HostTestRefusal refusal) {
  switch (refusal) {
    case HostTestRefusal::kNone: return "";
    case HostTestRefusal::kOutOfRange: return "OUT_OF_RANGE";
    case HostTestRefusal::kCharging: return "CHARGING";
    case HostTestRefusal::kBusy: return "BUSY";
    case HostTestRefusal::kBlade: return "BLADE";
    case HostTestRefusal::kNoSetup: return "NO_SETUP";
    case HostTestRefusal::kLastSetpoint: return "LAST_SETPOINT";
  }
  return "";
}

namespace {

void CheckRange(const char* name, double value, bool zero_allowed, double max) {
  const bool low_ok = zero_allowed ? value >= 0.0 : value > 0.0;
  if (!std::isfinite(value) || !low_ok || value > max) {
    throw std::invalid_argument(std::string(name) + " must be " + (zero_allowed ? "from 0" : "above 0") + " to " +
                                std::to_string(max) + " s, not " + std::to_string(value));
  }
}

}  // namespace

void CheckConfig(const McuControlConfig& config, bool heartbeat_only) {
  CheckRange("cmd_vel_timeout", config.cmd_vel_timeout_s, false, heartbeat_only ? 10.0 : 1.0);
  CheckRange("telemetry_timeout", config.telemetry_timeout_s, false, 3.0);
  CheckRange("blade_lease_max", config.blade_lease_max_s, false, 5.0);
  CheckRange("imu_timeout", config.imu_timeout_s, true, 2.0);
  CheckRange("imu_start_timeout", config.imu_start_timeout_s, true, 10.0);
  CheckRange("speed_repeat_window", config.speed_repeat_window_s, true, 3.0);
  CheckRange("blade_stop_time", config.blade_stop_time_s, true, 30.0);
  CheckRange("wheel_stop_time", config.wheel_stop_time_s, true, 10.0);
  CheckRange("silence_last_setpoint_max", config.silence_last_setpoint_max_s, true, kMaxSilenceLastSetpointS);
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
  blade_rpm_high_ = std::abs(rpm) >= config_.blade_still_rpm;
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

void McuControl::OnBladeMotorStatus(int status) {
  blade_fault_ = status < 0;
  if (blade_fault_) DropBlade(BladeOffReason::kMotorFault);
}

void McuControl::OnWheelSpeed(float linear, float angular, double now) {
  // The faster wheel, with the wheel distance the MCU counts with.
  const double wheel = std::abs(linear) + std::abs(angular) * kMcuWheelDistance / 2;
  const bool fast = wheel >= config_.wheel_still_mps;
  if (!fast) {
    wheel_fast_since_s_.reset();
    wheels_turning_ = false;
  } else if (!wheel_fast_since_s_) {
    wheel_fast_since_s_ = now;
  }
  if (!wheel_stop_s_ || config_.wheel_stop_time_s <= 0 || now - *wheel_stop_s_ < config_.wheel_stop_time_s) return;
  if (!fast) {
    wheel_stop_s_.reset();
    return;
  }
  if (now - *wheel_fast_since_s_ + kDeadlineSlackS < kWheelTurningS) return;
  wheels_turning_ = true;
  LatchEmergency(kReasonMowerNotStopped);
  // A lost stop frame is the likely cause, so they go out again and the check starts anew.
  StartStop(now);
}

void McuControl::OnSetupMotion(bool active, double now) {
  if (active) {
    setup_motion_s_ = now;
  } else {
    setup_motion_s_.reset();
  }
}

bool McuControl::OnVersions(const VersionInfo& versions) {
  versions_ = versions;
  const FirmwarePair pair{VersionString(versions.cutter_board), VersionString(versions.chassis_board)};
  const auto& measured = config_.measured_firmware;
  if (VersionListed(config_.cutter_firmware, versions.cutter_board) &&
      VersionListed(config_.chassis_firmware, versions.chassis_board)) {
    firmware_source_ = FirmwareSource::kListed;
  } else if (OlderThanListed(config_.cutter_firmware, versions.cutter_board) ||
             OlderThanListed(config_.chassis_firmware, versions.chassis_board)) {
    // Older firmware is never measured: an entry for it does not count.
    firmware_source_ = FirmwareSource::kOlder;
  } else if (!config_.device_hash.empty() && std::find(measured.begin(), measured.end(), pair) != measured.end()) {
    firmware_source_ = FirmwareSource::kMeasured;
  } else {
    firmware_source_ = FirmwareSource::kNone;
  }
  const bool ok = firmware_source_ == FirmwareSource::kListed || firmware_source_ == FirmwareSource::kMeasured;
  // A mismatch also latches, so motion after a later matching frame still needs a release.
  if (!ok && firmware_ok_ != false) LatchEmergency(kReasonFirmwareIncompatible);
  firmware_ok_ = ok;
  return ok;
}

void McuControl::OnFirmwareMeasure(bool active, const std::string& cutter, const std::string& chassis,
                                   const std::string& device_hash, uint32_t clear_seq, double now) {
  measure_ = MeasureSignal{active, cutter, chassis, device_hash, clear_seq, now};
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
  // The end of a silence latches anew.
  if (silent_until_s_ || ended_) return false;
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
  if (blade_fault_) {
    DropBlade(BladeOffReason::kMotorFault);
    return BladeResult::kMotorFault;
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
  firmware_source_ = FirmwareSource::kUnknown;
  versions_.reset();
  connected_s_ = now;
  heard_ = false;
  moving_ = false;
  StartStop(now);
  stop_release_.Interrupt();
}

void McuControl::OnFrame() {
  heard_ = true;
}

void McuControl::SetHold(bool held) {
  held_ = held;
}

void McuControl::SetStopClearMark(bool written) {
  stop_clear_mark_ = written;
}

HostTestRefusal McuControl::HostTestRefused(bool blade) const {
  // Also a single sample: the stop frames afterwards would push the mower off the contacts.
  if (DockContact() || dock_raw_) return HostTestRefusal::kCharging;
  if (!blade && (blade_request_ || blade_on_ || blade_turning_ || blade_rpm_high_)) return HostTestRefusal::kBlade;
  return HostTestRefusal::kNone;
}

HostTestRefusal McuControl::Silence(double seconds, double now) {
  return StartSilence(seconds, now, false);
}

HostTestRefusal McuControl::SilenceWithBlade(double seconds, double now) {
  return StartSilence(seconds, now, true);
}

HostTestRefusal McuControl::SilenceLastSetpoint(double seconds, double now) {
  if (config_.silence_last_setpoint_max_s <= 0) return HostTestRefusal::kLastSetpoint;
  return StartSilence(seconds, now, false, true);
}

HostTestRefusal McuControl::StartSilence(double seconds, double now, bool blade, bool last_setpoint) {
  if (silent_until_s_ || armed_silence_ || ended_) return HostTestRefusal::kBusy;
  const double max = last_setpoint ? config_.silence_last_setpoint_max_s : config_.silence_max_s;
  if (!std::isfinite(seconds) || seconds < kMinSilenceS || seconds > max) return HostTestRefusal::kOutOfRange;
  // Only a running setup step may silence the driver: in normal operation the cmd_vel timeout has to stay in force.
  if (!setup_motion_s_ || now - *setup_motion_s_ > kSetupMotionFreshS + kDeadlineSlackS) {
    return HostTestRefusal::kNoSetup;
  }
  const HostTestRefusal refusal = HostTestRefused(blade);
  if (refusal != HostTestRefusal::kNone) return refusal;
  silence_stop_delay_s_ =
      last_setpoint ? std::max(config_.silence_stop_delay_s, kLastSetpointStopDelayS) : config_.silence_stop_delay_s;
  silent_from_s_.reset();
  if (last_setpoint || config_.silence_after != McuControlConfig::SilenceAfter::kNone) {
    armed_silence_ = ArmedSilence{seconds, now + kSilenceArmS, last_setpoint};
  } else if (late_heartbeat_s_) {
    armed_silence_ = ArmedSilence{seconds, now + kSilenceArmS, false, true};
  } else {
    silent_until_s_ = now + seconds;
    silent_from_s_ = last_out_s_;
  }
  return HostTestRefusal::kNone;
}

HostTestRefusal McuControl::End() {
  const HostTestRefusal refusal = HostTestRefused(false);
  if (refusal != HostTestRefusal::kNone) return refusal;
  ended_ = true;
  return HostTestRefusal::kNone;
}

void McuControl::EndSilence(double now) {
  silent_until_s_.reset();
  // A new reason, also when HIGH_LEVEL was latched before: a release asked for earlier does not apply.
  Latch(kReasonHighLevel, true);
  release_until_s_.reset();
  // The MCU might still run the last setpoint; its blade state is unknown.
  moving_ = false;
  StartStop(now, silence_stop_delay_s_);
  blade_on_.reset();
  StartRepeat(blade_off_repeat_, BladeOff(), now);
  blade_off_since_s_ = now;
  next_heartbeat_s_.reset();
  late_heartbeat_s_.reset();
  stop_release_.Interrupt();
}

bool McuControl::SilenceAfterSetpoint(double now, std::vector<SubPacket>& out) {
  auto last = out.end();
  for (auto it = out.begin(); it != out.end(); ++it) {
    if (IsMotionSpeed(*it)) last = it;
  }
  if (last == out.end()) return false;
  std::rotate(last, last + 1, out.end());
  late_heartbeat_s_.reset();
  silent_until_s_ = now + armed_silence_->seconds;
  silent_from_s_ = now;
  armed_silence_.reset();
  return true;
}

void McuControl::TieHeartbeats(double now, const WallClock& wall, std::vector<SubPacket>& out) {
  using Mode = McuControlConfig::SpeedHeartbeat;
  if (armed_silence_ && armed_silence_->last_setpoint) {
    if (SilenceAfterSetpoint(now, out)) return;
    if (now > armed_silence_->until_s) armed_silence_.reset();
  }
  bool motion = false;
  bool blade_on = false;
  bool late = false;
  for (size_t i = 0; i < out.size(); ++i) {
    const SubPacket& packet = out[i];
    if (packet.type != static_cast<uint8_t>(PacketType::kHostToMcu)) continue;
    if (packet.module == static_cast<uint8_t>(Module::kCutter) && !packet.payload.empty() && packet.payload[0]) {
      blade_on = true;
    }
    if (packet.module != static_cast<uint8_t>(Module::kSpeed)) continue;
    if (IsMotionSpeed(packet)) motion = true;
    if (config_.speed_heartbeat == Mode::kLater) late_heartbeat_s_ = now + config_.heartbeat_after_speed_s;
    if (config_.speed_heartbeat == Mode::kAfter) {
      // Last in the loop: the insert moves what packet refers to.
      out.insert(out.begin() + static_cast<std::ptrdiff_t>(++i), HeartbeatPacket(wall));
    }
  }
  if (late_heartbeat_s_ && now + kDeadlineSlackS >= *late_heartbeat_s_) {
    out.push_back(HeartbeatPacket(wall));
    late_heartbeat_s_.reset();
    late = true;
  }

  if (armed_silence_ && armed_silence_->after_heartbeat) {
    if (late || !late_heartbeat_s_) {
      silent_until_s_ = now + armed_silence_->seconds;
      silent_from_s_ = out.empty() ? last_out_s_ : std::optional<double>(now);
      armed_silence_.reset();
    }
    return;
  }
  if (!armed_silence_ || armed_silence_->last_setpoint) return;
  using After = McuControlConfig::SilenceAfter;
  const bool speed_tick = config_.speed_heartbeat == Mode::kLater ? late : motion;
  if ((config_.silence_after == After::kSpeed && speed_tick) || (config_.silence_after == After::kCutter && blade_on)) {
    silent_until_s_ = now + armed_silence_->seconds;
    silent_from_s_ = now;
    armed_silence_.reset();
  } else if (now > armed_silence_->until_s) {
    armed_silence_.reset();
  }
}

bool McuControl::StopClearReady(double now) const {
  return config_.stop_release && config_.stop_clear_period_s > 0 && latched_ && latched_since_s_ &&
         now - *latched_since_s_ + kDeadlineSlackS >= config_.stop_clear_delay_s && !release_until_s_ &&
         !blade_on_ && !blade_turning_ && !TelemetryStale(now) && !ImuStale(now) && !NoImuYet() &&
         !MeasureSignalFresh(now);
}

bool McuControl::MeasureSignalFresh(double now) const {
  return measure_ && measure_->active && now - measure_->s <= kFirmwareMeasureFreshS + kDeadlineSlackS;
}

bool McuControl::MeasureOpen(double now) const {
  return firmware_source_ == FirmwareSource::kNone && versions_ && !config_.device_hash.empty() &&
         MeasureSignalFresh(now) && measure_->device_hash == config_.device_hash &&
         measure_->cutter == VersionString(versions_->cutter_board) &&
         measure_->chassis == VersionString(versions_->chassis_board);
}

bool McuControl::MeasureClearPending(double now) const {
  return MeasureSignalFresh(now) && measure_clear_seen_ && measure_->clear_seq != *measure_clear_seen_;
}

bool McuControl::MeasureClearReady(double now) const {
  return MeasureClearPending(now) && !moving_ && !wheel_fast_since_s_ && !wheels_turning_ && !blade_request_ &&
         !blade_on_ && !blade_turning_ && !blade_rpm_high_ && !TelemetryStale(now) && (stop_clear_mark_ || !Stop(now));
}

bool McuControl::Blocked(double now) const {
  return !heard_ || silent_until_s_ || ended_ || held_ || latched_ || TelemetryStale(now) || NoImuYet() ||
         FirmwareUnconfirmed(now);
}

// As hw/status shows it: a first frame, an IMU or a version frame still due after the start blocks without an
// emergency.
bool McuControl::Reported(double now) const {
  if (!heard_) return latched_ || BoardNotHeard(now);
  return latched_ || TelemetryStale(now) || (NoImuYet() && !ImuStarting(now)) || FirmwareIncompatible(now);
}

uint32_t McuControl::EmergencyReason(double now) const {
  if (!Reported(now)) return 0;
  uint32_t reason = latch_reason_;
  if (latched_) reason |= kReasonLatch;
  if (BoardNotHeard(now)) reason |= kReasonBoardNotHeard;
  if (heard_ && TelemetryStale(now)) reason |= kReasonTimeoutInputs;
  if (Stop(now)) reason |= kReasonStop;
  if (Lift(now)) reason |= kReasonLift;
  if (config_.bumper_latches_emergency && Bumper(now)) reason |= kReasonCollision;
  if (FirmwareIncompatible(now)) reason |= kReasonFirmwareIncompatible;
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

bool McuControl::BoardNotHeard(double now) const {
  return !heard_ && (!connected_s_ || now - *connected_s_ > config_.telemetry_timeout_s + kDeadlineSlackS);
}

bool McuControl::FirmwareUnconfirmed(double now) const {
  const bool checked = !config_.cutter_firmware.empty() || !config_.chassis_firmware.empty();
  return checked && firmware_ok_ != true && !MeasureOpen(now);
}

bool McuControl::FirmwareIncompatible(double now) const {
  if (!heard_ || !FirmwareUnconfirmed(now)) return false;
  if (firmware_ok_.has_value()) return true;
  const std::optional<double> since = connected_s_ ? connected_s_ : start_s_;
  return since && now - *since > kVersionStartS + kDeadlineSlackS;
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

void McuControl::StartStop(double now, double delay) {
  StartRepeat(stop_repeat_, ZeroSpeedPacket(), now);
  stop_repeat_.next_s = now + delay;
  wheel_stop_s_ = now + delay;
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

std::vector<SubPacket> McuControl::Farewell() const {
  std::vector<SubPacket> out;
  if (!heard_) return out;
  if (moving_ || !DockContact()) out.push_back(ZeroSpeedPacket());
  out.push_back(BladeOff());
  return out;
}

std::vector<SubPacket> McuControl::GuardFrames() const {
  if (silent_until_s_ || ended_) return {};
  return Farewell();
}

SubPacket McuControl::BladeOff() const {
  CutterCommand command;
  command.height_mm = config_.blade_off_height_mm;
  return CutterPacket(command);
}

std::vector<SubPacket> McuControl::Tick(double now, const WallClock& wall) {
  std::vector<SubPacket> out;
  if (ended_) return out;
  if (silent_until_s_) {
    if (now + kDeadlineSlackS < *silent_until_s_) {
      pending_cmd_.reset();
      pending_imu_.reset();
      release_requested_ = false;
      DropBlade(BladeOffReason::kBlocked);
      return out;
    }
    EndSilence(now);
  }
  // Not a byte to a port whose board was not heard: it might be another board than the one configured.
  if (!heard_) {
    pending_cmd_.reset();
    pending_imu_.reset();
    release_requested_ = false;
    release_until_s_.reset();
    DropBlade(BladeOffReason::kBlocked);
    return out;
  }

  // The state of the MCU is unknown when the driver takes over: the blade goes off and the wheels get the stop.
  if (!started_) {
    started_ = true;
    // A reconnect resends the frames but does not restart the time the IMU has.
    if (!start_s_) {
      start_s_ = now;
      // The MCU might still run a setpoint of an earlier driver, however that one ended.
      StartStop(now);
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
      // The MCU might pick up the last setpoint again once the latch is gone: the stop goes before and after.
      out.push_back(ZeroSpeedPacket());
      out.push_back(SensorClearPacket());
      last_clear_s_ = now;
      StartStop(now);
    }
  }
  if (release_until_s_) {
    if (!lift && !stop && !bumper && !TelemetryStale(now) && !ImuStale(now) && !blade_turning_ && !wheels_turning_) {
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
    out.push_back(ZeroSpeedPacket());
    out.push_back(SensorClearPacket());
    last_stop_clear_s_ = now;
  }
  UpdateMeasure(now, out);

  const bool blocked = Blocked(now);
  const bool dock_contact = DockContact();

  // Charging stopped on request stays off while this contact lasts; its off frames go out before any motion.
  if (!dock_contact) charge_stopped_ = false;
  EmitRepeat(charge_off_repeat_, now, out);

  if (blocked) {
    pending_cmd_.reset();
    if (moving_) {
      moving_ = false;
      StartStop(now);
    }
  } else if (pending_cmd_) {
    std::optional<SubPacket> packet;
    if (pending_cmd_->linear != 0.0f || pending_cmd_->angular != 0.0f) {
      packet = SpeedPacket(pending_cmd_->linear, pending_cmd_->angular);
    }
    if (!packet) {
      if (moving_) {
        moving_ = false;
        StartStop(now);
      }
      pending_cmd_.reset();
    } else if (now + kDeadlineSlackS >= next_speed_s_) {
      if (MeasureOpen(now)) {
        float linear = pending_cmd_->linear;
        float angular = pending_cmd_->angular;
        CapWheels(linear, angular, kMeasureMaxWheelSpeed);
        packet = SpeedPacket(linear, angular);
      }
      stop_repeat_.remaining = 0;
      wheel_stop_s_.reset();
      out.push_back(*packet);
      moving_ = true;
      last_speed_ = *packet;
      next_speed_s_ = now + config_.speed_period_s;
      pending_cmd_.reset();
    }
  }

  if (moving_ && now - last_cmd_s_ > StopAfter()) {
    moving_ = false;
    StartStop(now);
  }
  if (moving_ && last_speed_ && config_.speed_repeat_window_s > 0 && now + kDeadlineSlackS >= next_speed_s_) {
    out.push_back(*last_speed_);
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
  TieHeartbeats(now, wall, out);
  if (!out.empty()) last_out_s_ = now;
  return out;
}

void McuControl::UpdateMeasure(double now, std::vector<SubPacket>& out) {
  const bool open = MeasureOpen(now);
  if (measure_open_ && !open) {
    // The measurement no longer holds: a new reason that a release asked for before does not end; blocked, the tick
    // below sends the stop and drops the blade.
    Latch(kReasonFirmwareIncompatible, true);
    release_until_s_.reset();
    if (moving_ || !DockContact()) StartStop(now);
    moving_ = false;
    pending_cmd_.reset();
  }
  measure_open_ = open;

  if (!MeasureSignalFresh(now)) {
    measure_clear_seen_.reset();
    return;
  }
  if (!measure_clear_seen_) measure_clear_seen_ = measure_->clear_seq;
  if (!MeasureClearReady(now)) return;
  // As every clear: the MCU might pick up the last setpoint again once its latch is gone.
  out.push_back(ZeroSpeedPacket());
  out.push_back(SensorClearPacket());
  measure_clear_seen_ = measure_->clear_seq;
  measure_clear_sent_ = measure_->clear_seq;
  measure_clear_s_ = now;
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
  status.firmware_incompatible = FirmwareIncompatible(now);
  status.firmware_source = firmware_source_;
  status.measuring = MeasureOpen(now);
  status.clear_seq = measure_clear_sent_;
  status.clear_s = measure_clear_s_;
  status.held = held_;
  status.latch_count = latch_count_;
  status.stop_event = stop_event_;
  status.stop_clear_mark_wanted =
      !stop_clear_mark_ && Stop(now) && (StopClearReady(now) || MeasureClearPending(now));
  status.silent = silent_until_s_.has_value();
  status.silence_armed = armed_silence_.has_value();
  status.silent_from_s = silent_from_s_;
  status.ended = ended_;
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
