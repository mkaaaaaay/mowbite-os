#include "tron_mcu/mcu_codec.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace tron {

namespace {

void PutU16(std::vector<uint8_t>& out, uint16_t value) {
  out.push_back(static_cast<uint8_t>(value & 0xFF));
  out.push_back(static_cast<uint8_t>(value >> 8));
}

void PutI16(std::vector<uint8_t>& out, int16_t value) {
  PutU16(out, static_cast<uint16_t>(value));
}

void PutF32(std::vector<uint8_t>& out, float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  for (int i = 0; i < 4; i++) out.push_back(static_cast<uint8_t>(bits >> (8 * i)));
}

uint16_t GetU16(const std::vector<uint8_t>& in, size_t at) {
  return static_cast<uint16_t>(in[at] | (in[at + 1] << 8));
}

int16_t GetI16(const std::vector<uint8_t>& in, size_t at) {
  return static_cast<int16_t>(GetU16(in, at));
}

uint32_t GetU32(const std::vector<uint8_t>& in, size_t at) {
  return static_cast<uint32_t>(in[at]) | (static_cast<uint32_t>(in[at + 1]) << 8) |
         (static_cast<uint32_t>(in[at + 2]) << 16) | (static_cast<uint32_t>(in[at + 3]) << 24);
}

float GetF32(const std::vector<uint8_t>& in, size_t at) {
  uint32_t bits = GetU32(in, at);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

SubPacket Command(Module module, std::vector<uint8_t> payload) {
  return SubPacket{static_cast<uint8_t>(PacketType::kHostToMcu), static_cast<uint8_t>(module), std::move(payload)};
}

int16_t ScaleToInt16(double value, double full_scale) {
  double counts = std::round(value * 32768.0 / full_scale);
  return static_cast<int16_t>(std::clamp(counts, -32768.0, 32767.0));
}

MotorInfo GetMotor(const std::vector<uint8_t>& in, size_t at) {
  MotorInfo motor;
  motor.rpm = GetI16(in, at);
  motor.current_raw = GetI16(in, at + 2);
  motor.voltage_raw = GetI16(in, at + 4);
  motor.temperature_c = static_cast<int8_t>(in[at + 6]);
  motor.status = static_cast<int8_t>(in[at + 7]);
  return motor;
}

Version GetVersion(const std::vector<uint8_t>& in, size_t at) {
  return Version{in[at], in[at + 1], in[at + 2]};
}

}  // namespace

uint8_t Checksum(const uint8_t* data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) sum = static_cast<uint8_t>(sum + data[i]);
  return sum;
}

std::vector<uint8_t> Escape(const std::vector<uint8_t>& raw) {
  std::vector<uint8_t> out;
  out.reserve(raw.size() + 4);
  for (uint8_t b : raw) {
    switch (b) {
      case kFrameStart: out.insert(out.end(), {kEscape, 0x01}); break;
      case kFrameEnd: out.insert(out.end(), {kEscape, 0x02}); break;
      case kEscape: out.insert(out.end(), {kEscape, 0x03}); break;
      default: out.push_back(b);
    }
  }
  return out;
}

std::vector<uint8_t> EncodeFrame(const std::vector<SubPacket>& packets) {
  std::vector<uint8_t> inner{0};
  for (const auto& packet : packets) {
    if (packet.payload.size() > 0xFF) return {};
    inner.push_back(static_cast<uint8_t>(packet.payload.size()));
    inner.push_back(packet.type);
    inner.push_back(packet.module);
    inner.insert(inner.end(), packet.payload.begin(), packet.payload.end());
  }
  size_t total_len = inner.size() - 1;
  if (packets.empty() || total_len > 0xFF) return {};
  inner[0] = static_cast<uint8_t>(total_len);
  inner.push_back(Checksum(inner.data(), inner.size()));

  std::vector<uint8_t> frame{kFrameStart};
  std::vector<uint8_t> escaped = Escape(inner);
  frame.insert(frame.end(), escaped.begin(), escaped.end());
  frame.push_back(kFrameEnd);
  return frame;
}

bool DecodeInner(const std::vector<uint8_t>& inner, std::vector<SubPacket>& packets, bool& checksum_error) {
  checksum_error = false;
  packets.clear();
  if (inner.size() < 2 || inner[0] != inner.size() - 2) return false;
  if (Checksum(inner.data(), inner.size() - 1) != inner.back()) {
    checksum_error = true;
    return false;
  }
  size_t pos = 1;
  size_t end = inner.size() - 1;
  if (pos == end) return false;
  while (pos < end) {
    if (end - pos < 3) return false;
    size_t sub_len = inner[pos];
    if (end - pos - 3 < sub_len) return false;
    SubPacket packet;
    packet.type = inner[pos + 1];
    packet.module = inner[pos + 2];
    packet.payload.assign(inner.begin() + pos + 3, inner.begin() + pos + 3 + sub_len);
    packets.push_back(std::move(packet));
    pos += 3 + sub_len;
  }
  return true;
}

FrameParser::FrameParser(size_t max_frame_bytes) : max_frame_bytes_(max_frame_bytes) {
  inner_.reserve(max_frame_bytes_);
}

std::vector<std::vector<SubPacket>> FrameParser::Push(const uint8_t* data, size_t length) {
  std::vector<std::vector<SubPacket>> out;
  for (size_t i = 0; i < length; i++) {
    uint8_t b = data[i];
    if (b == kFrameStart) {
      if (in_frame_) stats_.dropped_bytes += inner_.size() + 1;
      in_frame_ = true;
      escape_pending_ = false;
      escape_error_ = false;
      inner_.clear();
      continue;
    }
    if (!in_frame_) {
      stats_.dropped_bytes++;
      continue;
    }
    if (b == kFrameEnd) {
      Finish(out);
      continue;
    }
    if (escape_pending_) {
      escape_pending_ = false;
      switch (b) {
        case 0x01: b = kFrameStart; break;
        case 0x02: b = kFrameEnd; break;
        case 0x03: b = kEscape; break;
        default: escape_error_ = true;
      }
    } else if (b == kEscape) {
      escape_pending_ = true;
      continue;
    }
    if (inner_.size() >= max_frame_bytes_) {
      stats_.overflows++;
      stats_.dropped_bytes += inner_.size() + 1;
      in_frame_ = false;
      inner_.clear();
      continue;
    }
    inner_.push_back(b);
  }
  return out;
}

void FrameParser::Finish(std::vector<std::vector<SubPacket>>& out) {
  in_frame_ = false;
  std::vector<SubPacket> packets;
  bool checksum_error = false;
  if (escape_pending_ || escape_error_) {
    stats_.escape_errors++;
  } else if (DecodeInner(inner_, packets, checksum_error)) {
    stats_.frames++;
    out.push_back(std::move(packets));
  } else if (checksum_error) {
    stats_.checksum_errors++;
  } else {
    stats_.length_errors++;
  }
  inner_.clear();
}

SubPacket HeartbeatPacket(const WallClock& now) {
  uint16_t year = std::clamp<uint16_t>(now.year, 2000, 2255);
  std::vector<uint8_t> payload{static_cast<uint8_t>(year - 2000),
                               now.month,
                               now.day,
                               now.hour,
                               now.minute,
                               now.second,
                               static_cast<uint8_t>(now.millisecond >> 8),
                               static_cast<uint8_t>(now.millisecond & 0xFF)};
  // Module byte of the heartbeat is not documented; 0 until checked against a capture.
  return SubPacket{static_cast<uint8_t>(PacketType::kHeartbeat), 0, std::move(payload)};
}

std::optional<SubPacket> SpeedPacket(float linear, float angular) {
  if (!std::isfinite(linear) || !std::isfinite(angular)) return std::nullopt;
  std::vector<uint8_t> payload;
  PutF32(payload, std::clamp(linear, -kMaxLinearSpeed, kMaxLinearSpeed));
  PutF32(payload, std::clamp(angular, -kMaxAngularSpeed, kMaxAngularSpeed));
  return Command(Module::kSpeed, std::move(payload));
}

SubPacket CutterPacket(const CutterCommand& command) {
  std::vector<uint8_t> payload;
  payload.push_back(command.blade_on ? 1 : 0);
  payload.push_back(command.reverse ? 1 : 0);
  PutU16(payload, std::min<uint16_t>(command.blade_speed_percent, 100));
  PutU16(payload, 0);
  // Height motor: only the absolute position is used, enable, direction and speed stay 0.
  payload.push_back(0);
  payload.push_back(0);
  PutU16(payload, 0);
  PutU16(payload, std::clamp(command.height_mm, kMinCutterHeightMm, kMaxCutterHeightMm));
  return Command(Module::kCutter, std::move(payload));
}

SubPacket ChargePacket(bool start) {
  return Command(Module::kCharge, {static_cast<uint8_t>(start ? 1 : 0)});
}

SubPacket SensorClearPacket() {
  return Command(Module::kSensor, std::vector<uint8_t>(8, 0));
}

SubPacket PowerOffPacket() {
  std::vector<uint8_t> payload(8, 0);
  payload[4] = 1;
  return Command(Module::kSensor, std::move(payload));
}

ImuCounts ImuCountsFromSi(double pitch_deg, double roll_deg, double yaw_deg, const std::array<double, 3>& acc_g,
                          const std::array<double, 3>& gyro_dps) {
  ImuCounts imu;
  imu.pitch = ScaleToInt16(pitch_deg, 180.0);
  imu.roll = ScaleToInt16(roll_deg, 180.0);
  imu.yaw = ScaleToInt16(yaw_deg, 180.0);
  for (size_t i = 0; i < 3; i++) {
    imu.acc[i] = ScaleToInt16(acc_g[i], 16.0);
    imu.gyro[i] = ScaleToInt16(gyro_dps[i], 2000.0);
  }
  return imu;
}

SubPacket ImuPacket(const ImuCounts& imu) {
  std::vector<uint8_t> payload;
  PutI16(payload, imu.pitch);
  PutI16(payload, imu.roll);
  PutI16(payload, imu.yaw);
  for (int16_t v : imu.acc) PutI16(payload, v);
  for (int16_t v : imu.gyro) PutI16(payload, v);
  return Command(Module::kImu, std::move(payload));
}

std::optional<MeasuredSpeed> DecodeSpeed(const std::vector<uint8_t>& payload) {
  if (payload.size() != 8) return std::nullopt;
  return MeasuredSpeed{GetF32(payload, 0), GetF32(payload, 4)};
}

std::optional<SensorInfo> DecodeSensorInfo(const std::vector<uint8_t>& payload) {
  if (payload.size() != 10) return std::nullopt;
  SensorInfo info;
  info.bumper = payload[0] != 0;
  info.rain = payload[1] != 0;
  info.lift = payload[2] != 0;
  info.stop = payload[3] != 0;
  info.power_off = payload[4] != 0;
  info.battery_gate = payload[5] != 0;
  info.cutter_size = payload[6];
  info.press_module = payload[7] != 0;
  info.bumper_right = payload[8] != 0;
  info.bumper_left = payload[9] != 0;
  return info;
}

std::optional<BatteryInfo> DecodeBattery(const std::vector<uint8_t>& payload) {
  if (payload.size() != 11) return std::nullopt;
  BatteryInfo info;
  info.voltage_v = GetU16(payload, 0) * 0.1;
  info.current_a = GetI16(payload, 2) * 0.1;
  info.percentage = payload[4];
  info.dock_ok = static_cast<int8_t>(payload[5]);
  info.temperature_c = static_cast<int8_t>(payload[6]);
  info.error_bits = GetU32(payload, 7);
  return info;
}

std::optional<MotorsInfo> DecodeMotors(const std::vector<uint8_t>& payload) {
  if (payload.size() != 32) return std::nullopt;
  // The MCU sends the right drive motor before the left, both with negative rpm forwards (measured 07.11.).
  MotorInfo right = GetMotor(payload, 8);
  MotorInfo left = GetMotor(payload, 16);
  right.rpm = static_cast<int16_t>(-right.rpm);
  left.rpm = static_cast<int16_t>(-left.rpm);
  return MotorsInfo{GetMotor(payload, 0), left, right, GetMotor(payload, 24)};
}

std::optional<VersionInfo> DecodeVersionInfo(const std::vector<uint8_t>& payload) {
  if (payload.size() != 9) return std::nullopt;
  return VersionInfo{GetVersion(payload, 0), GetVersion(payload, 3), GetVersion(payload, 6)};
}

std::optional<Version> DecodeBmsVersion(const std::vector<uint8_t>& payload) {
  if (payload.size() != 3) return std::nullopt;
  return GetVersion(payload, 0);
}

std::optional<uint32_t> DecodeUptime(const std::vector<uint8_t>& payload) {
  if (payload.size() != 4) return std::nullopt;
  return GetU32(payload, 0);
}

std::string VersionString(const Version& version) {
  return std::to_string(version.major) + "." + std::to_string(version.minor) + "." + std::to_string(version.patch);
}

std::string DecodeLog(const std::vector<uint8_t>& payload) {
  std::string text(payload.begin(), payload.end());
  text.erase(std::find(text.begin(), text.end(), '\0'), text.end());
  return text;
}

}  // namespace tron
