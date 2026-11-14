#ifndef TRON_MCU_MCU_CODEC_H
#define TRON_MCU_MCU_CODEC_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace tron {

// Serial protocol of the cutter (gateway) board on /dev/ttyS9, 115200 8N1.
// Frame: A5 | total_len | { sub_len | type | module | payload }+ | checksum | 5A, all values little-endian.
constexpr uint8_t kFrameStart = 0xA5;
constexpr uint8_t kFrameEnd = 0x5A;
constexpr uint8_t kEscape = 0xF5;

enum class PacketType : uint8_t {
  kHostToMcu = 0x00,
  kMcuToHost = 0x08,
  kHeartbeat = 0xFF,
};

enum class Module : uint8_t {
  kCharge = 3,
  kSpeed = 4,
  kBattery = 5,
  kUptime = 7,
  kImu = 9,
  kSensor = 10,
  kCutter = 11,
  kVersion = 12,
  kMotors = 13,
  kCalibration = 18,
  kLog = 19,
  kBmsVersion = 24,
};

struct SubPacket {
  uint8_t type = 0;
  uint8_t module = 0;
  std::vector<uint8_t> payload;
};

// Limits applied by the command builders, independent of any caller.
constexpr float kMaxLinearSpeed = 0.3f;
constexpr float kMaxAngularSpeed = 0.3f;
constexpr uint16_t kMinCutterHeightMm = 30;
constexpr uint16_t kMaxCutterHeightMm = 90;
// The MCU turns speed and turn rate into wheel speeds with 0.40 m between the wheels and reports them in mm/s.
constexpr double kMcuWheelDistance = 0.40;
constexpr double kMcuTicksPerMeter = 1000.0;

// --- Framing ---

uint8_t Checksum(const uint8_t* data, size_t length);
std::vector<uint8_t> Escape(const std::vector<uint8_t>& raw);

// Returns an empty vector if the packets do not fit into one frame (total_len is one byte).
std::vector<uint8_t> EncodeFrame(const std::vector<SubPacket>& packets);

struct ParserStats {
  uint64_t frames = 0;
  uint64_t checksum_errors = 0;
  uint64_t length_errors = 0;
  uint64_t escape_errors = 0;
  uint64_t overflows = 0;
  uint64_t dropped_bytes = 0;
};

// Byte stream parser. Bytes outside a frame are dropped, a start byte inside a frame restarts it.
class FrameParser {
 public:
  explicit FrameParser(size_t max_frame_bytes = 4096);

  // Returns the sub packets of every valid frame completed by this chunk, in order.
  std::vector<std::vector<SubPacket>> Push(const uint8_t* data, size_t length);

  const ParserStats& Stats() const {
    return stats_;
  }

 private:
  void Finish(std::vector<std::vector<SubPacket>>& out);

  size_t max_frame_bytes_;
  bool in_frame_ = false;
  bool escape_pending_ = false;
  bool escape_error_ = false;
  std::vector<uint8_t> inner_;
  ParserStats stats_;
};

// Splits the unescaped bytes between start and end byte into sub packets; false on any length or checksum error.
bool DecodeInner(const std::vector<uint8_t>& inner, std::vector<SubPacket>& packets, bool& checksum_error);

// --- Host to MCU ---

struct WallClock {
  uint16_t year = 2000;
  uint8_t month = 1;
  uint8_t day = 1;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint16_t millisecond = 0;
};

SubPacket HeartbeatPacket(const WallClock& now);

// Clamped to the speed limits; returns nothing for non-finite input.
std::optional<SubPacket> SpeedPacket(float linear, float angular);
// Speed 0, 0: the stop of the wheels.
SubPacket ZeroSpeedPacket();

struct CutterCommand {
  bool blade_on = false;
  bool reverse = false;
  uint16_t blade_speed_percent = 0;
  uint16_t height_mm = kMaxCutterHeightMm;
};

// Height is always clamped to 30-90 mm.
SubPacket CutterPacket(const CutterCommand& command);

SubPacket ChargePacket(bool start);

// Clears a latched lift or stop flag; all eight bytes are zero.
SubPacket SensorClearPacket();

// Cuts the battery power of the whole mower immediately.
SubPacket PowerOffPacket();

struct ImuCounts {
  int16_t pitch = 0;
  int16_t roll = 0;
  int16_t yaw = 0;
  std::array<int16_t, 3> acc{};
  std::array<int16_t, 3> gyro{};
};

// WIT scaling: angle +-180 deg, acceleration +-16 g, rate +-2000 deg/s onto int16, saturating.
ImuCounts ImuCountsFromSi(double pitch_deg, double roll_deg, double yaw_deg, const std::array<double, 3>& acc_g,
                          const std::array<double, 3>& gyro_dps);
SubPacket ImuPacket(const ImuCounts& imu);

// --- MCU to host ---

struct MeasuredSpeed {
  float linear = 0;
  float angular = 0;
};

struct SensorInfo {
  bool bumper = false;
  bool rain = false;
  bool lift = false;
  bool stop = false;
  bool power_off = false;
  bool battery_gate = false;
  uint8_t cutter_size = 0;
  bool press_module = false;
  bool bumper_right = false;
  bool bumper_left = false;
};

struct BatteryInfo {
  double voltage_v = 0;
  // Positive while discharging.
  double current_a = 0;
  uint8_t percentage = 0;
  int8_t dock_ok = 0;
  int8_t temperature_c = 0;
  uint32_t error_bits = 0;
};

// Raw values; units of current and voltage differ between cutter and drive boards.
struct MotorInfo {
  int16_t rpm = 0;
  int16_t current_raw = 0;
  int16_t voltage_raw = 0;
  int8_t temperature_c = 0;
  int8_t status = 0;
};

struct MotorsInfo {
  MotorInfo cutter;
  // Drive motors by side, positive rpm forwards.
  MotorInfo left;
  MotorInfo right;
  // Not populated by the hardware.
  MotorInfo height;
};

struct Version {
  uint8_t major = 0;
  uint8_t minor = 0;
  uint8_t patch = 0;
};

struct VersionInfo {
  Version cutter_board;
  Version chassis_board;
  Version rtk_board;
};

// Each decoder returns nothing if the payload length does not match.
std::optional<MeasuredSpeed> DecodeSpeed(const std::vector<uint8_t>& payload);
std::optional<SensorInfo> DecodeSensorInfo(const std::vector<uint8_t>& payload);
std::optional<BatteryInfo> DecodeBattery(const std::vector<uint8_t>& payload);
std::optional<MotorsInfo> DecodeMotors(const std::vector<uint8_t>& payload);
std::optional<VersionInfo> DecodeVersionInfo(const std::vector<uint8_t>& payload);
std::optional<Version> DecodeBmsVersion(const std::vector<uint8_t>& payload);
std::optional<uint32_t> DecodeUptime(const std::vector<uint8_t>& payload);
std::string DecodeLog(const std::vector<uint8_t>& payload);

// "major.minor.patch"
std::string VersionString(const Version& version);

}  // namespace tron

#endif  // TRON_MCU_MCU_CODEC_H
