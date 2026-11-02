#include "xcore_driver/services.h"

namespace xcore {

namespace {

Channel C(uint16_t id, const char* name, const char* type) {
  return Channel{id, name, *ParseValueType(type)};
}

}  // namespace

const ExpectedService& ExpectedEmergency() {
  static const ExpectedService s{"EmergencyService",
                                 2,
                                 {C(emergency::kInHighLevel, "high level emergency", "uint16_t[2]")},
                                 {C(emergency::kOutReason, "reason", "uint16_t")},
                                 {}};
  return s;
}

const ExpectedService& ExpectedDiffDrive() {
  static const ExpectedService s{
      "DiffDriveService",
      1,
      {C(diff_drive::kInTwist, "twist", "double[6]")},
      {C(diff_drive::kOutLeftStatus, "left esc status", "uint8_t"),
       C(diff_drive::kOutLeftTemperature, "left esc temperature", "float"),
       C(diff_drive::kOutLeftCurrent, "left esc current", "float"),
       C(diff_drive::kOutRightStatus, "right esc status", "uint8_t"),
       C(diff_drive::kOutRightTemperature, "right esc temperature", "float"),
       C(diff_drive::kOutRightCurrent, "right esc current", "float"), C(diff_drive::kOutTicks, "ticks", "uint32_t[2]")},
      {C(diff_drive::kRegTicksPerMeter, "ticks per meter", "double"),
       C(diff_drive::kRegWheelDistance, "wheel distance", "double")}};
  return s;
}

const ExpectedService& ExpectedMower() {
  static const ExpectedService s{"MowerService",
                                 2,
                                 {C(mower::kInSpeed, "speed", "float")},
                                 {C(mower::kOutStatus, "status", "uint8_t"), C(mower::kOutRain, "rain", "uint8_t"),
                                  C(mower::kOutRunning, "running", "uint8_t"),
                                  C(mower::kOutEscTemperature, "esc temperature", "float"),
                                  C(mower::kOutMotorTemperature, "motor temperature", "float"),
                                  C(mower::kOutCurrent, "current", "float"), C(mower::kOutRpm, "rpm", "float")},
                                 {}};
  return s;
}

const ExpectedService& ExpectedImu() {
  static const ExpectedService s{"ImuService",
                                 1,
                                 {},
                                 {C(imu::kOutAxes, "axes", "double[9]")},
                                 {C(imu::kRegAxisRemap, "axis remap", "int8_t[3]")}};
  return s;
}

const ExpectedService& ExpectedPower() {
  static const ExpectedService s{
      "PowerService",
      1,
      {C(power::kInChargingAllowed, "charging allowed", "uint8_t")},
      {C(power::kOutChargeVoltage, "charge voltage", "float"), C(power::kOutChargeCurrent, "charge current", "float"),
       C(power::kOutBatteryVoltage, "battery voltage", "float"),
       C(power::kOutChargingStatus, "charging status", "char[25]"),
       C(power::kOutChargerEnabled, "charger enabled", "uint8_t"),
       C(power::kOutBatteryPercentage, "battery percentage", "float")},
      {C(power::kRegEmptyVoltage, "empty voltage", "float"), C(power::kRegFullVoltage, "full voltage", "float"),
       C(power::kRegCriticalLowVoltage, "critical low voltage", "float"),
       C(power::kRegCriticalHighVoltage, "critical high voltage", "float")}};
  return s;
}

const ExpectedService& ExpectedGps() {
  static const ExpectedService s{
      "GpsService",
      1,
      {C(gps::kInRtcm, "rtcm", "uint8_t[255]")},
      {C(gps::kOutPosition, "position", "double[3]"), C(gps::kOutHorizontalAccuracy, "horizontal accuracy", "double"),
       C(gps::kOutVerticalAccuracy, "vertical accuracy", "double"), C(gps::kOutFixType, "fix type", "char[10]"),
       C(gps::kOutMotionEnu, "motion enu", "double[3]")},
      {C(gps::kRegBaudrate, "baudrate", "uint32_t"), C(gps::kRegProtocol, "protocol", "uint8_t"),
       C(gps::kRegUart, "uart", "uint8_t")}};
  return s;
}

const ExpectedService& ExpectedInput() {
  static const ExpectedService s{"InputService",
                                 1,
                                 {},
                                 {C(input::kOutActive, "active", "uint64_t"), C(input::kOutEvent, "event", "uint8_t[2]")},
                                 {C(input::kRegConfigs, "configs", "blob")}};
  return s;
}

const ExpectedService& ExpectedMeta() {
  static const ExpectedService s{"MetaService", 1, {}, {}, {C(meta::kRegRobotFirmware, "robot firmware", "char[50]")}};
  return s;
}

const ExpectedService* ExpectedFor(uint16_t service_id) {
  switch (service_id) {
    case kEmergencyService:
      return &ExpectedEmergency();
    case kDiffDriveService:
      return &ExpectedDiffDrive();
    case kMowerService:
      return &ExpectedMower();
    case kImuService:
      return &ExpectedImu();
    case kPowerService:
      return &ExpectedPower();
    case kGpsService:
      return &ExpectedGps();
    case kInputService:
      return &ExpectedInput();
    case kMetaService:
      return &ExpectedMeta();
    default:
      return nullptr;
  }
}

}  // namespace xcore
