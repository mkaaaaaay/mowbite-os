#ifndef TRON_MCU_MCU_JOURNAL_H
#define TRON_MCU_MCU_JOURNAL_H

#include <mowbite_journal/journaled.h>

#include "tron_mcu/mcu_codec.h"
#include "tron_mcu/mcu_control.h"

// The journal of McuControl (stream "mcu"): its calls come from the I/O thread, so their arguments are plain bytes.
// A new field of a struct below needs its place in its line, the size check makes the build say so.

namespace tron {

inline void to_json(nlohmann::json& j, const McuControlConfig& c) {
  j = nlohmann::json::object();
  j["heartbeat_period_s"] = c.heartbeat_period_s;
  j["cmd_vel_timeout_s"] = c.cmd_vel_timeout_s;
  j["speed_repeat_window_s"] = c.speed_repeat_window_s;
  j["speed_period_s"] = c.speed_period_s;
  j["telemetry_timeout_s"] = c.telemetry_timeout_s;
  j["repeat_count"] = c.repeat_count;
  j["repeat_interval_s"] = c.repeat_interval_s;
  j["imu_min_period_s"] = c.imu_min_period_s;
  j["imu_timeout_s"] = c.imu_timeout_s;
  j["imu_start_timeout_s"] = c.imu_start_timeout_s;
  j["dock_debounce_samples"] = c.dock_debounce_samples;
  j["charge_resend_s"] = c.charge_resend_s;
  j["allow_blade"] = c.allow_blade;
  j["blade_speed_percent"] = c.blade_speed_percent;
  j["mow_height_mm"] = c.mow_height_mm;
  j["blade_lease_max_s"] = c.blade_lease_max_s;
  j["blade_off_height_mm"] = c.blade_off_height_mm;
  j["blade_stop_time_s"] = c.blade_stop_time_s;
  j["blade_still_rpm"] = c.blade_still_rpm;
  j["bumper_latches_emergency"] = c.bumper_latches_emergency;
  j["release_window_s"] = c.release_window_s;
  j["clear_min_interval_s"] = c.clear_min_interval_s;
  j["expected_cutter_version"] = c.expected_cutter_version;
  j["expected_chassis_version"] = c.expected_chassis_version;
  j["stop_release"] = c.stop_release;
  j["stop_clear_period_s"] = c.stop_clear_period_s;
  j["stop_clear_delay_s"] = c.stop_clear_delay_s;
  j["start_latched"] = c.start_latched;
  j["silence_max_s"] = c.silence_max_s;
  j["speed_heartbeat"] = static_cast<int>(c.speed_heartbeat);
  j["heartbeat_after_speed_s"] = c.heartbeat_after_speed_s;
  j["silence_after"] = static_cast<int>(c.silence_after);
  j["silence_stop_delay_s"] = c.silence_stop_delay_s;
  j["wheel_stop_time_s"] = c.wheel_stop_time_s;
  j["wheel_still_mps"] = c.wheel_still_mps;
}
// Fields that came later take the value the journals from before ran with; all else is required.
inline void from_json(const nlohmann::json& j, McuControlConfig& c) {
  j.at("heartbeat_period_s").get_to(c.heartbeat_period_s);
  j.at("cmd_vel_timeout_s").get_to(c.cmd_vel_timeout_s);
  j.at("speed_repeat_window_s").get_to(c.speed_repeat_window_s);
  j.at("speed_period_s").get_to(c.speed_period_s);
  j.at("telemetry_timeout_s").get_to(c.telemetry_timeout_s);
  j.at("repeat_count").get_to(c.repeat_count);
  j.at("repeat_interval_s").get_to(c.repeat_interval_s);
  j.at("imu_min_period_s").get_to(c.imu_min_period_s);
  j.at("imu_timeout_s").get_to(c.imu_timeout_s);
  j.at("imu_start_timeout_s").get_to(c.imu_start_timeout_s);
  j.at("dock_debounce_samples").get_to(c.dock_debounce_samples);
  j.at("charge_resend_s").get_to(c.charge_resend_s);
  j.at("allow_blade").get_to(c.allow_blade);
  j.at("blade_speed_percent").get_to(c.blade_speed_percent);
  j.at("mow_height_mm").get_to(c.mow_height_mm);
  j.at("blade_lease_max_s").get_to(c.blade_lease_max_s);
  j.at("blade_off_height_mm").get_to(c.blade_off_height_mm);
  j.at("blade_stop_time_s").get_to(c.blade_stop_time_s);
  j.at("blade_still_rpm").get_to(c.blade_still_rpm);
  j.at("bumper_latches_emergency").get_to(c.bumper_latches_emergency);
  j.at("release_window_s").get_to(c.release_window_s);
  j.at("clear_min_interval_s").get_to(c.clear_min_interval_s);
  j.at("expected_cutter_version").get_to(c.expected_cutter_version);
  j.at("expected_chassis_version").get_to(c.expected_chassis_version);
  j.at("stop_release").get_to(c.stop_release);
  c.stop_clear_period_s = j.value("stop_clear_period_s", McuControlConfig{}.stop_clear_period_s);
  c.stop_clear_delay_s = j.value("stop_clear_delay_s", McuControlConfig{}.stop_clear_delay_s);
  c.start_latched = j.value("start_latched", false);
  c.silence_max_s = j.value("silence_max_s", 0.0);
  c.speed_heartbeat = static_cast<McuControlConfig::SpeedHeartbeat>(j.value("speed_heartbeat", 0));
  c.heartbeat_after_speed_s = j.value("heartbeat_after_speed_s", McuControlConfig{}.heartbeat_after_speed_s);
  c.silence_after = static_cast<McuControlConfig::SilenceAfter>(j.value("silence_after", 0));
  c.silence_stop_delay_s = j.value("silence_stop_delay_s", 0.0);
  c.wheel_stop_time_s = j.value("wheel_stop_time_s", 0.0);
  c.wheel_still_mps = j.value("wheel_still_mps", McuControlConfig{}.wheel_still_mps);
}
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SensorInfo, bumper, rain, lift, stop, power_off, battery_gate, cutter_size,
                                   press_module, bumper_right, bumper_left)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImuCounts, pitch, roll, yaw, acc, gyro)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Version, major, minor, patch)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VersionInfo, cutter_board, chassis_board, rtk_board)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(WallClock, year, month, day, hour, minute, second, millisecond)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BladeRequest, on, speed_percent, height_mm, lease_s)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SubPacket, type, module, payload)

static_assert(sizeof(McuControlConfig) == 304);
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
                          &tron::McuControl::SetHold, &tron::McuControl::Tick, &tron::McuControl::SetStopClearMark,
                          &tron::McuControl::Silence, &tron::McuControl::End,
                          &tron::McuControl::SilenceWithBlade, &tron::McuControl::OnWheelSpeed,
                          &tron::McuControl::OnSetupMotion, &tron::McuControl::OnFrame);

  // Every call with the wall clock, for a copy of a journal without its place (mowbite_replay anonymize): the date of
  // a tick and of the heartbeat it sends.
  template <typename A>
  static void Anonymize(A& a) {
    a.template Args<&tron::McuControl::Tick>([](const auto& o, double&, tron::WallClock& wall) {
      int year = wall.year, month = wall.month, day = wall.day;
      o.ShiftDate(&year, &month, &day);
      wall.year = static_cast<uint16_t>(year);
      wall.month = static_cast<uint8_t>(month);
      wall.day = static_cast<uint8_t>(day);
    });
    a.template Result<&tron::McuControl::Tick>(
        [](const auto&, std::vector<tron::SubPacket>& packets, double&, tron::WallClock& wall) {
          for (auto& packet : packets) {
            if (packet.type == static_cast<uint8_t>(tron::PacketType::kHeartbeat)) packet = tron::HeartbeatPacket(wall);
          }
        });
  }
};

#endif  // TRON_MCU_MCU_JOURNAL_H
