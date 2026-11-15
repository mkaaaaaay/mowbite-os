#ifndef TRON_MCU_MCU_JOURNAL_H
#define TRON_MCU_MCU_JOURNAL_H

#include <mowbite_journal/journaled.h>

#include "tron_mcu/mcu_codec.h"
#include "tron_mcu/mcu_control.h"

// The journal of McuControl (stream "mcu"): its calls come from the I/O thread, so their arguments are plain bytes.
// A new field of a struct below needs its place in its line, the size check makes the build say so.

namespace tron {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(McuControlConfig, heartbeat_period_s, cmd_vel_timeout_s, speed_repeat_window_s,
                                   speed_period_s, telemetry_timeout_s, repeat_count, repeat_interval_s,
                                   imu_min_period_s, imu_timeout_s, imu_start_timeout_s, dock_debounce_samples,
                                   charge_resend_s, allow_blade,
                                   blade_speed_percent, mow_height_mm, blade_lease_max_s, blade_off_height_mm,
                                   blade_stop_time_s, blade_still_rpm, bumper_latches_emergency, release_window_s,
                                   clear_min_interval_s, clear_zero_after_motion_s, expected_cutter_version,
                                   expected_chassis_version, stop_release, stop_clear_period_s, stop_clear_delay_s)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SensorInfo, bumper, rain, lift, stop, power_off, battery_gate, cutter_size,
                                   press_module, bumper_right, bumper_left)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImuCounts, pitch, roll, yaw, acc, gyro)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Version, major, minor, patch)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VersionInfo, cutter_board, chassis_board, rtk_board)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(WallClock, year, month, day, hour, minute, second, millisecond)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BladeRequest, on, speed_percent, height_mm, lease_s)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SubPacket, type, module, payload)

static_assert(sizeof(McuControlConfig) == 248);
static_assert(sizeof(SensorInfo) == 10);
static_assert(sizeof(ImuCounts) == 18);
static_assert(sizeof(VersionInfo) == 9);
static_assert(sizeof(WallClock) == 10);
static_assert(sizeof(BladeRequest) == 16);
static_assert(sizeof(SubPacket) == 32);

}  // namespace tron

MOWBITE_JOURNAL_POD(tron::SensorInfo);
MOWBITE_JOURNAL_POD(tron::ImuCounts);
MOWBITE_JOURNAL_POD(tron::VersionInfo);
MOWBITE_JOURNAL_POD(tron::WallClock);
MOWBITE_JOURNAL_POD(tron::BladeRequest);

// The frames of a tick without CBOR: type, module and the payload with its length.
template <>
struct mowbite_journal::Codec<tron::SubPacket> {
  static void Write(Out& out, const tron::SubPacket& packet) {
    out.Put(&packet.type, 1);
    out.Put(&packet.module, 1);
    mowbite_journal::Write(out, packet.payload);
  }
  static tron::SubPacket Read(In& in) {
    tron::SubPacket packet;
    in.Get(&packet.type, 1);
    in.Get(&packet.module, 1);
    packet.payload = mowbite_journal::Read<std::vector<uint8_t>>(in);
    return packet;
  }
};

template <>
struct mowbite_journal::CoreTraits<tron::McuControl> {
  static constexpr const char* kName = "mcu";
  using Config = tron::McuControlConfig;
  MOWBITE_JOURNAL_METHODS(&tron::McuControl::OnCmdVel, &tron::McuControl::OnSensorInfo,
                          &tron::McuControl::OnDockContactSample, &tron::McuControl::OnImu,
                          &tron::McuControl::OnBladeRpm, &tron::McuControl::OnVersions,
                          &tron::McuControl::LatchEmergency, &tron::McuControl::ReleaseEmergency,
                          &tron::McuControl::SetBlade, &tron::McuControl::SetCharging, &tron::McuControl::OnConnected,
                          &tron::McuControl::SetHold, &tron::McuControl::Tick, &tron::McuControl::SetStopClearMark);
};

#endif  // TRON_MCU_MCU_JOURNAL_H
