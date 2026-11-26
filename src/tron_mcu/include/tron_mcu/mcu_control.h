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

// A pair of board firmware an owner measured on one device (FP55).
struct FirmwarePair {
  std::string cutter;
  std::string chassis;
  bool operator==(const FirmwarePair& other) const {
    return cutter == other.cutter && chassis == other.chassis;
  }
};

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
  // The MCU does not confirm a stop either. A wheel it reports at wheel_still_mps or more for kWheelTurningS in a row,
  // wheel_stop_time_s or later after the stop frames began, latches MOWER_NOT_STOPPED and the stop frames go out
  // again; the first report of standing wheels from then on ends the check. 0 turns it off.
  double wheel_stop_time_s = 1.5;
  double wheel_still_mps = 0.025;
  bool bumper_latches_emergency = true;
  // The driver before this one ended and the launch started this one: the MCU does not lock itself without a host, so
  // the emergency is latched (HIGH_LEVEL) from the start. Every start sends the stop frames, with or without it.
  bool start_latched = false;
  // A release waits this long for lift, stop and bumper to drop after the clear frame.
  double release_window_s = 1.0;
  double clear_min_interval_s = 1.0;
  // Firmware the driver was measured with, "major.minor.patch" per entry. Empty skips the check of that board. With a
  // check, motion and blade stay blocked until a version frame names a listed firmware; a release does not lift it.
  std::vector<std::string> cutter_firmware;
  std::vector<std::string> chassis_firmware;
  // Pairs measured on this device, from measured_firmware of the driver (MeasuredFirmware). Not older than the lists.
  std::vector<FirmwarePair> measured_firmware;
  // sha256 (hex) of the id of this device; empty: no measurement opens.
  std::string device_hash;
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
  // Longest hw/silence with last_setpoint; 0 offers none.
  double silence_last_setpoint_max_s = 0;
  // How the MCU ends a setpoint without its host (H3, T9): kLater is the setting of the Tron, kAfter a measurement.
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
constexpr uint32_t kReasonBoardNotHeard = 1u << 23;
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
  kMotorFault = 6,
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
// Upper bound of silence_last_setpoint_max_s: the MCU kept a setpoint 3.6 s without host (T9).
constexpr double kMaxSilenceLastSetpointS = 5.0;
// After a silence with last_setpoint the stop frames wait this long while heartbeats go, so that a setpoint the MCU
// picks up again on a heartbeat shows.
constexpr double kLastSetpointStopDelayS = 0.5;
// setup/firmware_measure counts only this long after it came; the setup program sends it at 4 Hz.
constexpr double kFirmwareMeasureFreshS = 0.5;
// Each wheel while a measurement drives unlisted firmware (m/s).
constexpr float kMeasureMaxWheelSpeed = 0.1f;
// The MCU sends its version frame on its own at about 1.7 Hz. Until the first one after the port opened, motion and
// blade are blocked; past this time without one the block is reported as FIRMWARE_INCOMPATIBLE.
constexpr double kVersionStartS = 3.0;
// How long an hw/silence with silence_after waits for its tick.
constexpr double kSilenceArmS = 1.0;
// hw/silence is taken only this soon after an active setup/motion; the setup program sends it at 4 Hz.
constexpr double kSetupMotionFreshS = 1.0;
// How long a wheel has to turn after a stop before it counts as not stopped.
constexpr double kWheelTurningS = 0.3;

// Why hw/silence or hw/end was refused; kNone when it was taken.
enum class HostTestRefusal : uint8_t { kNone, kOutOfRange, kCharging, kBusy, kBlade, kNoSetup, kLastSetpoint };

// The code of the service answer: OUT_OF_RANGE, CHARGING, BUSY, BLADE, NO_SETUP, LAST_SETPOINT; empty for kNone.
const char* HostTestRefusalCode(HostTestRefusal refusal);

// Whether the firmware the boards report may drive, as hw/capabilities.firmware_source names it.
enum class FirmwareSource : uint8_t { kUnknown, kListed, kMeasured, kOlder, kNone };

// "", listed, measured, older, none.
const char* FirmwareSourceText(FirmwareSource source);

// "major.minor.patch" with each part 0 to 255; nothing otherwise.
std::optional<Version> ParseVersion(const std::string& text);

// The pairs of measured_firmware ("<cutter>/<chassis>/<device_hash>", hash 64 lowercase hex) for device_hash, none
// for an empty one. Entries of another form go to malformed, those of other devices are only counted.
std::vector<FirmwarePair> MeasuredFirmware(const std::vector<std::string>& entries, const std::string& device_hash,
                                           std::vector<std::string>* malformed = nullptr, int* other_devices = nullptr);

enum class BladeResult { kOk, kNotAllowed, kInvalidLease, kInvalidSpeed, kInvalidHeight, kBlocked, kDocked, kHeld,
                         kMotorFault };

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
  // A version frame that does not match, or none within kVersionStartS after the port opened.
  bool firmware_incompatible = false;
  FirmwareSource firmware_source = FirmwareSource::kUnknown;
  // setup/firmware_measure lets this firmware drive within kMeasureMaxWheelSpeed.
  bool measuring = false;
  // The last clear setup/firmware_measure asked for that went out, and when; 0 and empty before the first.
  uint32_t clear_seq = 0;
  std::optional<double> clear_s;
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

// Throws std::invalid_argument for a time that is not finite or out of its range: NaN or a huge value would switch
// the cmd_vel watchdog, the lease of the blade or the checks of telemetry and IMU off. The ranges, in s:
// cmd_vel_timeout (0, 1] (with heartbeat_only of T2 in mowbite-measure (0, 10]), telemetry_timeout (0, 3],
// blade_lease_max (0, 5], imu_timeout [0, 2] (0 without forwarding), imu_start_timeout [0, 10],
// speed_repeat_window [0, 3], blade_stop_time [0, 30], wheel_stop_time [0, 10].
void CheckConfig(const McuControlConfig& config, bool heartbeat_only = false);

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
  // Status code of the blade motor from the same frame: below zero is a fault, it switches the blade off and refuses
  // it until a report without fault.
  void OnBladeMotorStatus(int status);
  // Speed and turn rate as the MCU measures them (m/s, rad/s, about 100 Hz).
  void OnWheelSpeed(float linear, float angular, double now);
  // setup/motion of the setup program: a step that moves the wheels or the blade runs while it is active.
  void OnSetupMotion(bool active, double now);
  // Version frame of the MCU, about once a second. Returns false unless the firmware is listed or measured here.
  bool OnVersions(const VersionInfo& versions);
  // setup/firmware_measure: while it is active, at most kFirmwareMeasureFreshS old, names the firmware the boards report
  // and the device_hash of the config, and that firmware is none, motion and blade are allowed, each wheel up to
  // kMeasureMaxWheelSpeed. Once it no longer holds: stop, blade off, latched (FIRMWARE_INCOMPATIBLE). While it is
  // active and fresh, whatever the firmware, STOP is not cleared on its own; a change of clear_seq asks for one clear,
  // sent once the wheels and the blade stand. The first after a gap only sets where clear_seq stands.
  void OnFirmwareMeasure(bool active, const std::string& cutter, const std::string& chassis,
                         const std::string& device_hash, uint32_t clear_seq, double now);

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
  // The MCU state is unknown after the port (re)opens: the blade goes off and the wheels get the stop frames, but only
  // after OnFrame. Until then nothing goes out and the driver blocks, reported as BOARD_NOT_HEARD after
  // telemetry_timeout.
  void OnConnected(double now);
  // A valid frame from the MCU.
  void OnFrame();
  // The hold of an update (hold.json): motion and blade blocked like before the first IMU sample, without an
  // emergency; a moving mower gets a stop, the blade goes off. Charging goes on.
  void SetHold(bool held);
  // Whether the mark of a STOP cleared during the emergency is on disk; without it no such clear frame goes out.
  void SetStopClearMark(bool written);
  // hw/silence: no frame to the MCU for seconds; commands, blade requests, releases and IMU samples meanwhile are
  // dropped. At its end the emergency latches (HIGH_LEVEL, counted), and the blade off and the stop frames go out as
  // after start_latched. Refused without an active setup/motion within kSetupMotionFreshS, on the charging contacts and
  // with the blade on or turning.
  HostTestRefusal Silence(double seconds, double now);
  // As Silence, also with the blade on or turning: the MCU has to stop it without its host. The blade request is gone
  // afterwards.
  HostTestRefusal SilenceWithBlade(double seconds, double now);
  // As Silence, but up to silence_last_setpoint_max_s, refused with kLastSetpoint without it: starts right after the
  // next tick that sends a motion setpoint, with that SpeedData as the last frame, so that the MCU keeps it by its own
  // limit only. Its stop frames wait kLastSetpointStopDelayS while heartbeats go.
  HostTestRefusal SilenceLastSetpoint(double seconds, double now);
  // hw/end: from now on no frame to the MCU; the node then ends the process like a crash. Also during a silence.
  HostTestRefusal End();

  std::vector<SubPacket> Tick(double now, const WallClock& wall);
  // The frames when the process ends, sent repeat_count times by the node: the stop, unless the mower stands on the
  // charging contacts without a command (zero frames push it off them), and the blade off.
  std::vector<SubPacket> Farewell() const;
  // The frames the node's guard writes while its I/O thread stalls: the farewell, none during hw/silence or after
  // hw/end.
  std::vector<SubPacket> GuardFrames() const;
  McuControlStatus Status(double now) const;
  const McuControlConfig& Config() const {
    return config_;
  }
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
  // Closing of the measurement window and the clears setup/firmware_measure asks for.
  void UpdateMeasure(double now, std::vector<SubPacket>& out);
  bool StopClearReady(double now) const;
  bool Blocked(double now) const;
  HostTestRefusal HostTestRefused(bool blade) const;
  HostTestRefusal StartSilence(double seconds, double now, bool blade, bool last_setpoint = false);
  // Moves the motion setpoint of this tick to its end and starts an armed silence with last_setpoint; false without one.
  bool SilenceAfterSetpoint(double now, std::vector<SubPacket>& out);
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
  bool BoardNotHeard(double now) const;
  bool FirmwareUnconfirmed(double now) const;
  bool MeasureSignalFresh(double now) const;
  bool MeasureOpen(double now) const;
  // A clear asked for by setup/firmware_measure may go out: wheels and blade stand, and the mark is there for STOP.
  bool MeasureClearPending(double now) const;
  bool MeasureClearReady(double now) const;
  bool FirmwareIncompatible(double now) const;
  bool Lift(double now) const;
  bool Stop(double now) const;
  bool Bumper(double now) const;
  void StartRepeat(Repeat& repeat, const SubPacket& packet, double now);
  // The stop frames from now + delay on, and the check of the wheels after them.
  void StartStop(double now, double delay = 0);
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
  // The last blade motor report showed a fault.
  bool blade_fault_ = false;

  // When the stop frames began; empty while the wheels are commanded or once they stood after wheel_stop_time_s.
  std::optional<double> wheel_stop_s_;
  // Since when the reports show a wheel at wheel_still_mps or more; empty while they stand.
  std::optional<double> wheel_fast_since_s_;
  // The wheels did not stop; only a report of standing wheels clears it.
  bool wheels_turning_ = false;

  // The last active setup/motion; empty after an inactive one.
  std::optional<double> setup_motion_s_;

  int dock_samples_ = 0;
  bool dock_raw_ = false;
  bool charge_sent_ = false;
  double last_charge_s_ = 0;
  // Charging was stopped on request; it stays off while this contact lasts.
  bool charge_stopped_ = false;

  // Unknown until the first version frame after the port opened.
  std::optional<bool> firmware_ok_;
  FirmwareSource firmware_source_ = FirmwareSource::kUnknown;
  std::optional<VersionInfo> versions_;

  // The last setup/firmware_measure and when it came.
  struct MeasureSignal {
    bool active;
    std::string cutter;
    std::string chassis;
    std::string device_hash;
    uint32_t clear_seq;
    double s;
  };
  std::optional<MeasureSignal> measure_;
  // The window was open at the last tick.
  bool measure_open_ = false;
  // clear_seq of the signal already taken; empty while it is not fresh and active.
  std::optional<uint32_t> measure_clear_seen_;
  uint32_t measure_clear_sent_ = 0;
  std::optional<double> measure_clear_s_;
  // When the port opened; before that the first tick counts.
  std::optional<double> connected_s_;
  // A valid frame came since the port opened.
  bool heard_ = false;

  bool held_ = false;

  // End of the running hw/silence; empty while none runs.
  std::optional<double> silent_until_s_;
  // hw/silence waiting for its tick (silence_after): its length and until when it waits.
  struct ArmedSilence {
    double seconds;
    double until_s;
    bool last_setpoint;
  };
  // Wait of the stop frames after the running silence.
  double silence_stop_delay_s_ = 0;
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
