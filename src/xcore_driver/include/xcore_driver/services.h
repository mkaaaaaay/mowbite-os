#ifndef XCORE_DRIVER_SERVICES_H
#define XCORE_DRIVER_SERVICES_H

#include <cstdint>

#include "xcore_driver/schema.h"

namespace xcore {

// Service ids of the OpenMower v2 firmware.
enum ServiceId : uint16_t {
  kEmergencyService = 1,
  kDiffDriveService = 2,
  kMowerService = 3,
  kImuService = 4,
  kPowerService = 5,
  kGpsService = 6,
  kInputService = 7,
  kHighLevelService = 8,
  kBmsService = 9,
  kMetaService = 11,
};

// Channel ids the driver uses, per service.
namespace emergency {
// uint16_t[2]: reason bits to set, reason bits to clear. Every message counts as the host's heartbeat.
constexpr uint16_t kInHighLevel = 0;
constexpr uint16_t kOutReason = 0;
}  // namespace emergency

namespace diff_drive {
// double[6]: linear x, y, z in m/s and angular x, y, z in rad/s; the firmware uses [0] and [5].
constexpr uint16_t kInTwist = 0;
constexpr uint16_t kOutTwist = 0;
constexpr uint16_t kOutLeftStatus = 1;
constexpr uint16_t kOutLeftTemperature = 2;
constexpr uint16_t kOutLeftCurrent = 3;
constexpr uint16_t kOutRightStatus = 4;
constexpr uint16_t kOutRightTemperature = 5;
constexpr uint16_t kOutRightCurrent = 6;
// uint32_t[2]: left, right; free running.
constexpr uint16_t kOutTicks = 7;
constexpr uint16_t kRegTicksPerMeter = 0;
constexpr uint16_t kRegWheelDistance = 1;
}  // namespace diff_drive

namespace mower {
// float from -1 to 1.
constexpr uint16_t kInSpeed = 0;
constexpr uint16_t kOutStatus = 0;
constexpr uint16_t kOutRain = 1;
constexpr uint16_t kOutRunning = 2;
constexpr uint16_t kOutEscTemperature = 3;
constexpr uint16_t kOutMotorTemperature = 4;
constexpr uint16_t kOutCurrent = 5;
constexpr uint16_t kOutRpm = 6;
}  // namespace mower

namespace imu {
// double[9]: acceleration x, y, z in m/s², rate x, y, z in rad/s, then three more values the driver does not use.
constexpr uint16_t kOutAxes = 0;
// int8_t[3]
constexpr uint16_t kRegAxisRemap = 0;
}  // namespace imu

namespace power {
constexpr uint16_t kInChargingAllowed = 0;
constexpr uint16_t kOutChargeVoltage = 0;
constexpr uint16_t kOutChargeCurrent = 1;
constexpr uint16_t kOutBatteryVoltage = 2;
constexpr uint16_t kOutChargingStatus = 3;
constexpr uint16_t kOutChargerEnabled = 4;
constexpr uint16_t kOutBatteryPercentage = 5;
constexpr uint16_t kRegEmptyVoltage = 1;
constexpr uint16_t kRegFullVoltage = 2;
constexpr uint16_t kRegCriticalLowVoltage = 3;
constexpr uint16_t kRegCriticalHighVoltage = 4;
}  // namespace power

namespace gps {
constexpr uint16_t kInRtcm = 0;
// double[3]: latitude and longitude in degrees, height in m.
constexpr uint16_t kOutPosition = 0;
constexpr uint16_t kOutHorizontalAccuracy = 1;
constexpr uint16_t kOutVerticalAccuracy = 2;
// char[10]: "FIX", "FLOAT" or empty.
constexpr uint16_t kOutFixType = 3;
// double[3]: east, north, up in m/s.
constexpr uint16_t kOutMotionEnu = 4;
constexpr uint16_t kRegBaudrate = 0;
// uint8_t: 0 UBX, 1 NMEA.
constexpr uint16_t kRegProtocol = 1;
constexpr uint16_t kRegUart = 2;
}  // namespace gps

namespace input {
constexpr uint16_t kOutActive = 0;
constexpr uint16_t kOutEvent = 1;
constexpr uint16_t kRegConfigs = 0;
}  // namespace input

namespace meta {
// char[50]: the name of the carrier board, the other services of some boards wait for it.
constexpr uint16_t kRegRobotFirmware = 0;
}  // namespace meta

// Bits of the board's emergency reason; the driver passes bits 0 to 12 on in hw/status with the same numbers.
namespace reason {
constexpr uint16_t kLatch = 1u << 0;
constexpr uint16_t kTimeoutInputs = 1u << 1;
constexpr uint16_t kStop = 1u << 2;
constexpr uint16_t kLift = 1u << 3;
constexpr uint16_t kLiftMultiple = 1u << 4;
constexpr uint16_t kCollision = 1u << 5;
constexpr uint16_t kTimeoutHighLevel = 1u << 6;
constexpr uint16_t kHighLevel = 1u << 7;
constexpr uint16_t kServiceNotReady = 1u << 8;
constexpr uint16_t kCollisionMultiple = 1u << 9;
constexpr uint16_t kMowerRpmTimeout = 1u << 10;
constexpr uint16_t kMowerRpmLimit = 1u << 11;
constexpr uint16_t kFirmwareIncompatible = 1u << 12;
}  // namespace reason

// The channels the driver relies on, per service it claims.
const ExpectedService& ExpectedEmergency();
const ExpectedService& ExpectedDiffDrive();
const ExpectedService& ExpectedMower();
const ExpectedService& ExpectedImu();
const ExpectedService& ExpectedPower();
const ExpectedService& ExpectedGps();
const ExpectedService& ExpectedInput();
const ExpectedService& ExpectedMeta();
// nullptr for a service the driver does not claim.
const ExpectedService* ExpectedFor(uint16_t service_id);

}  // namespace xcore

#endif  // XCORE_DRIVER_SERVICES_H
