#ifndef TRON_MCU_MCU_CONTROL_H
#define TRON_MCU_MCU_CONTROL_H

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_hold/restart.h"
#include "tron_mcu/mcu_codec.h"
#include "tron_mcu/stop_release.h"

namespace tron {

struct McuControlConfig {
  double heartbeat_period_s = 0.1;
  // Silence on cmd_vel after which the wheels are stopped.
  double cmd_vel_timeout_s = 0.5;
  // The MCU drops a setpoint ~0.2 s after the last SpeedData, but at 20 SpeedData/s it kept one up to 3.6 s without
  // host (measured 08.11.). So motion frames go out at most every speed_period_s, a newer command replaces the waiting
  // one; a stop goes at once. Within speed_repeat_window_s of the last cmd_vel the last non-zero setpoint goes out
  // again every speed_period_s, and the stop follows at its end if that comes before cmd_vel_timeout_s. 0 sends each
  // command once.
  double speed_repeat_window_s = 0;
  double speed_period_s = 0.1;
  // Without sensor info from the MCU for this long, motion and blade are blocked.
  double telemetry_timeout_s = 1.0;
  int repeat_count = 3;
  double repeat_interval_s = 0.1;
  double imu_min_period_s = 0.02;
  // Without an IMU sample for this long, motion and blade are blocked: the MCU knows the tilt only from the IMU the host
  // forwards. Before the first sample this only blocks, unreported for imu_start_timeout_s from the first tick; a
  // later gap latches. 0 when the IMU is not forwarded.
  double imu_timeout_s = 0;
  // The first sample may take this long after the first tick, at least imu_timeout_s: the IMU runs in another process.
  double imu_start_timeout_s = 0;
  int dock_debounce_samples = 3;
  double charge_resend_s = 30.0;
  bool allow_blade = false;
  // Defaults for a blade request that leaves speed or height at 0.
  uint16_t blade_speed_percent = 100;
  uint16_t mow_height_mm = 60;
  // Longest lease a blade request may ask for; without a renewal the blade goes off when it runs out.
  double blade_lease_max_s = 2.0;
  uint16_t blade_off_height_mm = 90;
  // The MCU does not confirm an off frame. A blade it still reports at blade_still_rpm or more this long after it went
  // off latches the emergency, and the off frames go out again. 0 turns the check off.
  double blade_stop_time_s = 5.0;
  int blade_still_rpm = 100;
  bool bumper_latches_emergency = true;
  // The driver before this one ended and the launch started this one: the MCU does not lock itself without a host and
  // might still hold a setpoint, so the emergency is latched (HIGH_LEVEL) from the start and the first tick sends the
  // stop frames.
  bool start_latched = false;
  // A release waits this long for lift, stop and bumper to drop after the clear frame.
  double release_window_s = 1.0;
  double clear_min_interval_s = 1.0;
  // Zero speed follows a clear frame only if the wheels were commanded this recently.
  double clear_zero_after_motion_s = 5.0;
  // Firmware the driver was checked against, "major.minor.patch". Empty skips the check of that board. With a check,
  // motion and blade stay blocked until a version frame matches; a release does not lift it.
  std::string expected_cutter_version;
  std::string expected_chassis_version;
  // The STOP gesture (G37) is recognised and reported in Status; the release is up to the logic.
  bool stop_release = false;
  // With stop_release: the MCU keeps stop after a press until a clear frame, so while the emergency is latched and the
  // MCU shows stop, a clear frame goes out at most every stop_clear_period_s (0: never), from stop_clear_delay_s after
  // the latch on, with the blade off and still, fresh telemetry and IMU, no release running and the mark written
  // (SetStopClearMark). A held STOP stays shown, a let go one drops, a new press shows again. The MCU no longer locks
  // itself in that time, the driver's latch is the lock.
  double stop_clear_period_s = 0;
  double stop_clear_delay_s = 1.5;
  // Longest hw/silence; 0 offers none.
  double silence_max_s = 0;
  // Measurement of how the MCU ends a setpoint without its host (H3, phase 6); only set by an override.
  // speed_heartbeat: kAfter adds a heartbeat right after each SpeedData of a tick, kLater one heartbeat_after_speed_s
  // after the last SpeedData; the heartbeats of their own schedule go on.
  enum class SpeedHeartbeat { kNone, kAfter, kLater };
  SpeedHeartbeat speed_heartbeat = SpeedHeartbeat::kNone;
  double heartbeat_after_speed_s = 0.025;
  // silence_after: hw/silence starts right after the next tick that sends a motion setpoint (with kLater: the
  // heartbeat after it), or a blade on frame; without such a tick within kSilenceArmS it is dropped.
  enum class SilenceAfter { kNone, kSpeed, kCutter };
  SilenceAfter silence_after = SilenceAfter::kNone;
  // After a silence the stop frames wait this long while heartbeats go on.
  double silence_stop_delay_s = 0;
};

// Bits of the emergency reason, numbered like EmergencyReason of OpenMower.
constexpr uint32_t kReasonLatch = 1u << 0;
constexpr uint32_t kReasonTimeoutInputs = 1u << 1;
constexpr uint32_t kReasonStop = 1u << 2;
constexpr uint32_t kReasonLift = 1u << 3;
constexpr uint32_t kReasonCollision = 1u << 5;
constexpr uint32_t kReasonHighLevel = 1u << 7;
constexpr uint32_t kReasonFirmwareIncompatible = 1u << 12;
// Own bits of hw/status.
constexpr uint32_t kReasonTilt = 1u << 13;
constexpr uint32_t kReasonFence = 1u << 14;
constexpr uint32_t kReasonStuck = 1u << 15;
constexpr uint32_t kReasonImuTimeout = 1u << 16;
constexpr uint32_t kReasonMowerNotStopped = 1u << 17;
// The bits a node above the driver may latch with over hw/emergency.
using mowbite_hold::kHostLatchReasons;

// Why the blade went off the last time it was requested or running.
enum class BladeOffReason : uint8_t {
  kNone = 0,
  kRequest = 1,
  kLeaseExpired = 2,
  kBlocked = 3,
  kDock = 4,
  kReconnect = 5,
};

struct BladeRequest {
  bool on = false;
  // 0 takes blade_speed_percent of the config, otherwise 1-100.
  uint16_t speed_percent = 0;
  // 0 takes mow_height_mm of the config, otherwise within the cutter height range.
  uint16_t height_mm = 0;
  // The blade goes off unless the next request comes within this time; at most blade_lease_max_s.
  double lease_s = 0;
};

// The last STOP gesture reported; seq counts them from 1.
struct StopEvent {
  uint32_t seq = 0;
  StopPress press = StopPress::kNone;
  // latch_count with kLong, 0 otherwise.
  uint32_t latch_count = 0;
};

// One fixed input of the Tron as hw/inputs lists it; its name is also its role in hw/status.
struct McuInput {
  const char* name = "";
  // The emergency the driver latches with it: stop, lift or collision; empty for none.
  const char* reason = "";
  // The MCU keeps it after the sensor is free, until a clear frame.
  bool held = false;
  bool active = false;
};

// Shortest hw/silence, as in the contract.
constexpr double kMinSilenceS = 0.2;
// How long an hw/silence with silence_after waits for its tick.
constexpr double kSilenceArmS = 1.0;

// Why hw/silence or hw/end was refused; kNone when it was taken.
enum class HostTestRefusal : uint8_t { kNone, kOutOfRange, kCharging, kBusy, kBlade };

// The code of the service answer: OUT_OF_RANGE, CHARGING, BUSY, BLADE; empty for kNone.
const char* HostTestRefusalCode(HostTestRefusal refusal);

enum class BladeResult { kOk, kNotAllowed, kInvalidLease, kInvalidSpeed, kInvalidHeight, kBlocked, kDocked, kHeld };

const char* BladeResultText(BladeResult result);

struct McuControlStatus {
  // Motion and blade are blocked: latched, stale telemetry, no IMU or incompatible firmware. While the IMU is still
  // due after the start, motion and blade are held without this.
  bool blocked = false;
  // Reason bits while blocked, 0 otherwise. A cause that latched stays in it until the release.
  uint32_t emergency_reason = 0;
  bool latched = false;
  bool lift = false;
  bool stop = false;
  bool bumper = false;
  bool telemetry_stale = true;
  bool moving = false;
  bool blade_on = false;
  BladeOffReason blade_off_reason = BladeOffReason::kNone;
  bool dock_contact = false;
  // The last sample, not debounced.
  bool dock_contact_raw = false;
  // No version frame since the port opened, or one that does not match.
  bool firmware_incompatible = false;
  // An update holds the mower: motion and blade blocked without an emergency.
  bool held = false;
  // Counts every new reason: a press of STOP (debounced), lift or bumper rising, a bit new in the latch. Never 0 once
  // something counted.
  uint32_t latch_count = 0;
  StopEvent stop_event;
  // The next clear frame for STOP waits for the mark (SetStopClearMark).
  bool stop_clear_mark_wanted = false;
  // hw/silence runs, or hw/end was taken: nothing goes to the MCU.
  bool silent = false;
  bool ended = false;
};

// Decides what goes to the MCU and when. Time is passed in (monotonic seconds), nothing here touches the port.
// Every returned packet is sent in a frame of its own.
class McuControl {
 public:
  explicit McuControl(const McuControlConfig& config);

  void OnCmdVel(float linear, float angular, double now);
  void OnSensorInfo(const SensorInfo& info, double now);
  void OnDockContactSample(bool contact);
  // Every sample is forwarded at most once, so a dead IMU stops the stream; imu_timeout_s counts from the last one.
  void OnImu(const ImuCounts& imu, double now);
  // Speed of the blade motor as the MCU reports it (motors frame, about once a second).
  void OnBladeRpm(int rpm, double now);
  // Version frame of the MCU, about once a second. Returns false if it does not match the expected firmware.
  bool OnVersions(const VersionInfo& versions);

  // Lift, stop and bumper latch the emergency as well; all of it stays until released, with its reason bits.
  void LatchEmergency(uint32_t reason);
  // Clears a latched lift or stop on the MCU and releases the emergency once the flags are gone, unless latch_count
  // rises before. Refused (false) when latch_count is not 0 and not the current one: another reason latched since.
  bool ReleaseEmergency(double now, uint32_t latch_count);
  // Switching off always works. Switching on, or renewing the lease with the same or another setting, is refused when
  // the configuration forbids the blade, the request is out of range, the mower is blocked or on the dock. A refusal
  // because of an emergency sets emergency_reason to its bits as hw/status shows them, every other answer to 0.
  BladeResult SetBlade(const BladeRequest& request, double now, uint32_t* emergency_reason = nullptr);
  // Not allowed: Charge 0 at once, repeat_count times, and no Charge 1 until allowed again or the contact ends, so
  // that the mower leaves the station without charging. Allowed: Charge 1 at the next tick while there is contact.
  void SetCharging(bool allowed, double now);
  // The MCU state is unknown after the port (re)opens: the blade goes off and moving wheels get a stop.
  void OnConnected(double now);
  // The hold of an update (hold.json): motion and blade blocked like before the first IMU sample, without an
  // emergency; a moving mower gets a stop, the blade goes off. Charging goes on.
  void SetHold(bool held);
  // Whether the mark of a STOP cleared during the emergency is on disk; without it no such clear frame goes out.
  void SetStopClearMark(bool written);
  // hw/silence: no frame to the MCU for seconds; commands, blade requests, releases and IMU samples meanwhile are
  // dropped. At its end the emergency latches (HIGH_LEVEL, counted), and the blade off and the stop frames go out as
  // after start_latched. Refused on the charging contacts and with the blade on or turning.
  HostTestRefusal Silence(double seconds, double now);
  // As Silence, also with the blade on or turning: the MCU has to stop it without its host. The blade request is gone
  // afterwards.
  HostTestRefusal SilenceWithBlade(double seconds, double now);
  // hw/end: from now on no frame to the MCU; the node then ends the process like a crash. Also during a silence.
  HostTestRefusal End();

  std::vector<SubPacket> Tick(double now, const WallClock& wall);
  McuControlStatus Status(double now) const;
  // STOP, lift and the two bumpers; none active without fresh telemetry.
  std::array<McuInput, 4> Inputs(double now) const;

 private:
  struct Repeat {
    SubPacket packet;
    int remaining = 0;
    double next_s = 0;
  };

  struct BladeSetting {
    uint16_t speed_percent;
    uint16_t height_mm;
    bool operator==(const BladeSetting& other) const {
      return speed_percent == other.speed_percent && height_mm == other.height_mm;
    }
    bool operator!=(const BladeSetting& other) const {
      return !(*this == other);
    }
  };
  struct BladeLease {
    BladeSetting setting;
    double until_s;
  };

  // Latches the reason bits; a bit not latched yet counts as a new reason, once per call, as does rising.
  void Latch(uint32_t reason, bool rising = false);
  void CountReason();
  void UpdateStopRelease(bool stop, bool lift, bool latched_before, double now);
  bool StopClearReady(double now) const;
  bool Blocked(double now) const;
  HostTestRefusal HostTestRefused(bool blade) const;
  HostTestRefusal StartSilence(double seconds, double now, bool blade);
  void EndSilence(double now);
  // The measurement settings of the config: heartbeats after SpeedData, the start of an armed silence.
  void TieHeartbeats(double now, const WallClock& wall, std::vector<SubPacket>& out);
  bool Reported(double now) const;
  uint32_t EmergencyReason(double now) const;
  bool DockContact() const;
  void DropBlade(BladeOffReason reason);
  bool TelemetryStale(double now) const;
  bool ImuStale(double now) const;
  bool NoImuYet() const;
  bool ImuStarting(double now) const;
  bool FirmwareIncompatible() const;
  bool Lift(double now) const;
  bool Stop(double now) const;
  bool Bumper(double now) const;
  void StartRepeat(Repeat& repeat, const SubPacket& packet, double now);
  void EmitRepeat(Repeat& repeat, double now, std::vector<SubPacket>& out);
  double StopAfter() const;
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
  // The last non-zero setpoint sent and when it goes out again.
  std::optional<SubPacket> last_speed_;
  double next_speed_s_ = 0;

  std::optional<SensorInfo> sensor_;
  double last_sensor_s_ = 0;

  bool latched_ = false;
  // Every cause that latched since the last release.
  uint32_t latch_reason_ = 0;
  bool release_requested_ = false;
  std::optional<double> release_until_s_;
  // latch_count when the release was asked for; a new reason ends the release without effect.
  uint32_t release_count_ = 0;
  uint32_t latch_count_ = 0;
  // A reason counted since the last gesture update.
  bool new_reason_ = false;
  bool last_lift_ = false;
  bool last_bumper_ = false;
  StopRelease stop_release_;
  StopEvent stop_event_;
  // Clear frames of a release, and those of STOP during the emergency.
  std::optional<double> last_clear_s_;
  std::optional<double> last_stop_clear_s_;
  // Since when the emergency is latched; empty while it is not.
  std::optional<double> latched_since_s_;
  bool stop_clear_mark_ = false;

  std::optional<BladeLease> blade_request_;
  // Setting of the last on frame, empty while the blade is off.
  std::optional<BladeSetting> blade_on_;
  BladeOffReason blade_off_reason_ = BladeOffReason::kNone;
  // When the blade was last sent off, by request or at the start; empty while it is on.
  std::optional<double> blade_off_since_s_;
  // The last blade report after blade_stop_time_s still showed it turning; only a slower report clears it.
  bool blade_turning_ = false;
  // The last blade report showed blade_still_rpm or more.
  bool blade_rpm_high_ = false;

  int dock_samples_ = 0;
  bool dock_raw_ = false;
  bool charge_sent_ = false;
  double last_charge_s_ = 0;
  // Charging was stopped on request; it stays off while this contact lasts.
  bool charge_stopped_ = false;

  // Unknown until the first version frame after the port opened.
  std::optional<bool> firmware_ok_;

  bool held_ = false;

  // End of the running hw/silence; empty while none runs.
  std::optional<double> silent_until_s_;
  // hw/silence waiting for its tick (silence_after): its length and until when it waits.
  struct ArmedSilence {
    double seconds;
    double until_s;
  };
  std::optional<ArmedSilence> armed_silence_;
  // With kLater: when the heartbeat after the last SpeedData is due.
  std::optional<double> late_heartbeat_s_;
  bool ended_ = false;

  std::optional<ImuCounts> pending_imu_;
  // Last forward to the MCU and last sample from the IMU.
  std::optional<double> last_imu_s_;
  std::optional<double> last_imu_sample_s_;

  std::optional<double> next_heartbeat_s_;
  bool started_ = false;
  // First tick after the node started.
  std::optional<double> start_s_;

  Repeat stop_repeat_;
  Repeat blade_off_repeat_;
  Repeat charge_off_repeat_;
};

}  // namespace tron

#endif  // TRON_MCU_MCU_CONTROL_H
