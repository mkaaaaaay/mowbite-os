#ifndef TRON_MCU_MCU_CONTROL_H
#define TRON_MCU_MCU_CONTROL_H

#include <optional>
#include <string>
#include <vector>

#include "tron_mcu/mcu_codec.h"

namespace tron {

struct McuControlConfig {
  double heartbeat_period_s = 0.1;
  // Silence on cmd_vel after which the wheels are stopped.
  double cmd_vel_timeout_s = 0.5;
  // Without sensor info from the MCU for this long, motion and blade are blocked.
  double telemetry_timeout_s = 1.0;
  int repeat_count = 3;
  double repeat_interval_s = 0.1;
  double imu_min_period_s = 0.02;
  int dock_debounce_samples = 3;
  double charge_resend_s = 30.0;
  bool allow_blade = false;
  uint16_t blade_speed_percent = 100;
  uint16_t mow_height_mm = 60;
  uint16_t blade_off_height_mm = 90;
  bool bumper_latches_emergency = true;
  // A release waits this long for lift, stop and bumper to drop after the clear frame.
  double release_window_s = 1.0;
  double clear_min_interval_s = 1.0;
  // Zero speed follows a clear frame only if the wheels were commanded this recently.
  double clear_zero_after_motion_s = 5.0;
  // Firmware the driver was checked against, "major.minor.patch". Empty skips the check of that board. With a check,
  // motion and blade stay blocked until a version frame matches; a release does not lift it.
  std::string expected_cutter_version;
  std::string expected_chassis_version;
};

struct McuControlStatus {
  bool latched = false;
  bool lift = false;
  bool stop = false;
  bool bumper = false;
  bool telemetry_stale = true;
  bool moving = false;
  bool blade_on = false;
  bool dock_contact = false;
  // No version frame since the port opened, or one that does not match.
  bool firmware_incompatible = false;
};

// Decides what goes to the MCU and when. Time is passed in (monotonic seconds), nothing here touches the port.
// Every returned packet is sent in a frame of its own.
class McuControl {
 public:
  explicit McuControl(const McuControlConfig& config);

  void OnCmdVel(float linear, float angular, double now);
  void OnSensorInfo(const SensorInfo& info, double now);
  void OnDockContactSample(bool contact);
  // Every sample is forwarded at most once, so a dead IMU stops the stream.
  void OnImu(const ImuCounts& imu);
  // Version frame of the MCU, about once a second. Returns false if it does not match the expected firmware.
  bool OnVersions(const VersionInfo& versions);

  // Lift, stop and bumper latch the emergency as well; all of it stays until released.
  void LatchEmergency();
  // Clears a latched lift or stop on the MCU and releases the emergency once the flags are gone.
  void ReleaseEmergency(double now);
  // Returns false if the blade is not allowed by the configuration.
  bool SetBlade(bool on);
  // The MCU state is unknown after the port (re)opens: the blade goes off and moving wheels get a stop.
  void OnConnected(double now);

  std::vector<SubPacket> Tick(double now, const WallClock& wall);
  McuControlStatus Status(double now) const;

 private:
  struct Repeat {
    SubPacket packet;
    int remaining = 0;
    double next_s = 0;
  };

  bool TelemetryStale(double now) const;
  bool FirmwareIncompatible() const;
  bool Lift(double now) const;
  bool Stop(double now) const;
  bool Bumper(double now) const;
  void StartRepeat(Repeat& repeat, const SubPacket& packet, double now);
  void EmitRepeat(Repeat& repeat, double now, std::vector<SubPacket>& out);
  SubPacket ZeroSpeed() const;
  SubPacket BladeOff() const;

  McuControlConfig config_;

  struct PendingCmd {
    float linear;
    float angular;
  };
  std::optional<PendingCmd> pending_cmd_;
  double last_cmd_s_ = 0;
  bool moving_ = false;
  std::optional<double> last_motion_s_;

  std::optional<SensorInfo> sensor_;
  double last_sensor_s_ = 0;

  bool latched_ = false;
  bool release_requested_ = false;
  std::optional<double> release_until_s_;
  std::optional<double> last_clear_s_;

  bool blade_request_ = false;
  bool blade_on_ = false;

  int dock_samples_ = 0;
  bool charge_sent_ = false;
  double last_charge_s_ = 0;

  // Unknown until the first version frame after the port opened.
  std::optional<bool> firmware_ok_;

  std::optional<ImuCounts> pending_imu_;
  std::optional<double> last_imu_s_;

  std::optional<double> next_heartbeat_s_;
  bool started_ = false;

  Repeat stop_repeat_;
  Repeat blade_off_repeat_;
};

}  // namespace tron

#endif  // TRON_MCU_MCU_CONTROL_H
