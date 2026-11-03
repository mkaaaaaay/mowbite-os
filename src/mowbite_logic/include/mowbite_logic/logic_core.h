#ifndef MOWBITE_LOGIC_LOGIC_CORE_H
#define MOWBITE_LOGIC_LOGIC_CORE_H

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/docking.h"
#include "mowbite_logic/mowing_job.h"
#include "mowbite_logic/reasons.h"
#include "mowbite_logic/undocking.h"

namespace mowbite_logic {

// What the logic needs from hw/status.
struct HardwareSnapshot {
  bool telemetry_ok = false;
  // Blocked by the driver; the reason bits come from it, numbered like the EMERGENCY event.
  bool emergency = false;
  int emergency_reason = 0;
  bool rain = false;
  bool dock_contact = false;
  // The contact as the last report had it, not debounced: docking stops at the first one.
  bool dock_contact_raw = false;
  bool battery_ok = false;
  float battery_voltage = 0;
  // Positive while discharging.
  float battery_current = 0;
  uint8_t battery_percentage = 0;
  int8_t battery_temperature = 0;
  struct Motor {
    bool ok = false;
    int16_t rpm = 0;
    float current = 0;
    int8_t temperature = 0;
  };
  Motor blade;
  Motor left;
  Motor right;
};

// What the logic needs from localization/pose.
struct PoseSnapshot {
  double x = 0;
  double y = 0;
  double heading = 0;
  float position_accuracy = 999;
  float orientation_accuracy = 999;
  bool orientation_valid = false;
};

// The calibration drive: before a start a position this accurate (m) with a heading and min_battery percent of
// charge; the status of the program counts for status_timeout (s), and without a sign of the drive for timeout (s) the
// logic ends it.
struct CalibrationConfig {
  double max_position_accuracy = 0.03;
  int min_battery = 30;
  double status_timeout = 2.0;
  double timeout = 2.0;
};

struct LogicConfig {
  // 6S Li-ion; empty is an assumption until the discharge curve of the Tron is measured.
  double battery_empty_voltage = 19.2;
  double battery_full_voltage = 25.2;
  // The MCU reports no charge voltage, dock contact stands in for it.
  double dock_charge_voltage = 28.0;
  // GPS counts as good up to this accuracy (m), like max_position_accuracy of OpenMower. The localization and the
  // follower take it from the logic (logic/gps_limits).
  double max_position_accuracy = 0.1;
  // Its max_position_accuracy is the one above.
  RecorderConfig recorder;
  // Mowing. The blade only with mow_with_blade, and then only on mow segments (perimeter, lanes, turns).
  bool mow_with_blade = false;
  // Before a path starts on a mow segment the blade has to reach this rpm within mower_spinup_timeout (s); 0: no wait.
  int mower_spinup_rpm = 0;
  double mower_spinup_timeout = 10.0;
  // The blade counts as running from this rpm on, for BLADES events.
  int blade_running_rpm = 100;
  // s with a good position before mowing or docking goes on after the GPS got bad; docking fails after gps_timeout s
  // without one.
  double gps_wait_time = 10.0;
  double gps_timeout = 10.0;
  // Mowing ends below this voltage (V), held for battery_critical_time (s). Assumption until the discharge curve is
  // measured: 3.5 V per cell.
  double battery_critical_voltage = 21.0;
  double battery_critical_time = 10.0;
  // m: farther from the start of a path the mower first drives there over the drivable areas.
  double approach_distance = 0.5;
  // s between checkpoints while a path is mowed.
  double checkpoint_period = 30.0;
  // m ahead of base_link where the charging contacts of the mower are. Below 0 they are behind it: it backs onto the
  // docking station and stands there facing away from the heading of the station in the map.
  double dock_contact_x = 0;
  // On the docking station mowing and drives start only after the reference check (G24): a position at least
  // dock_max_position_accuracy (m) and, with the heading of the station set, base_link within dock_reference_limit (m)
  // of the recorded station.
  double dock_max_position_accuracy = 0.03;
  double dock_reference_limit = 0.10;
  // After the reference check the mower leaves the station before mowing or a drive starts (contacts behind base_link
  // only; it drives forwards out of the station).
  UndockConfig undock;
  // After JOB_COMPLETE, abort_mowing and on docking.start the mower backs onto the station (contacts behind base_link
  // only); a try that fails also drives out like undocking.
  DockConfig dock;
  CalibrationConfig calibration;
};

// calibration/status: the problems of the program against a drive from here (room, sensors, map) and the lane it
// would take (lane_length 0 without one), with the strip it needs.
struct CalibrationStatus {
  std::vector<Reason> problems;
  double heading = 0;
  double lane_length = 0;
  double strip_length = 0;
  double strip_width = 0;
  bool running = false;
};

// The goal for calibration/run: the lane of the status the start was checked with.
struct CalibrationGoal {
  double heading = 0;
  double lane_length = 0;
};

// How the program ended a drive: DONE with the status and summary of the result, else why it stopped.
struct CalibrationOutcome {
  std::string reason;
  std::string message;
  std::string status;
  std::string summary;
};

// How an undocking ended, for the node: undocked or not, whether a drive waits for it, and why the drive or mowing
// after it did not start (empty if it did).
struct UndockOutcome {
  bool undocked = false;
  bool for_drive = false;
  Reason why;
};

struct Action {
  std::string id;
  std::string name;
  bool enabled = false;
};

struct ActionGroup {
  std::string prefix;
  std::vector<Action> actions;
};

struct RobotStateData {
  std::string state;
  std::string sub_state;
  float battery_percentage = 0;
  bool emergency = false;
  bool is_charging = false;
  bool rain_detected = false;
  // 1 at perfect accuracy, 0 at max_position_accuracy or worse, without a recent pose or without a heading.
  // OpenMower sends -1 for a bad position with a valid heading; a negative percentage is not shown here.
  float gps_percentage = 0;
  double pose_x = 0;
  double pose_y = 0;
  double pose_heading = 0;
  // 999 without a recent pose: an accuracy of 0 would read as a perfect RTK fix in the app.
  float pos_accuracy = 999;
  float heading_accuracy = 999;
  bool heading_valid = false;
  // Bits as in EMERGENCY events, 0 without emergency.
  int emergency_reason = 0;
  // While mowing: the area (counting the mow areas of the map), the path of its plan and the pose in it; else -1.
  int current_area = -1;
  int current_path = -1;
  int current_path_index = -1;
};

// What a sensor value means, numbered as in mowbite_msgs/SensorSpec.
enum class SensorMeaning : uint8_t { kUnknown = 0, kVoltage, kCurrent, kTemperature, kRevolutions, kDistance };

struct SensorDefinition {
  std::string id;
  std::string name;
  std::string unit;
  bool text = false;
  SensorMeaning meaning = SensorMeaning::kUnknown;
  // NaN where there is no limit.
  double min = std::numeric_limits<double>::quiet_NaN();
  double max = std::numeric_limits<double>::quiet_NaN();
  double critical_low = std::numeric_limits<double>::quiet_NaN();
  double critical_high = std::numeric_limits<double>::quiet_NaN();
};

struct SensorReading {
  std::string id;
  double value = 0;
  std::string text;
};

enum class ActionResult { kUnknown, kRejected, kDone, kReleaseEmergency };

struct ReasonChange {
  int from = 0;
  int to = 0;
};

// Bits of the reason in an EMERGENCY event, as the app reads them (EmergencyReason of OpenMower) and as
// hw/status carries them.
constexpr int kEmergencyLatch = 1 << 0;
constexpr int kEmergencyTimeoutInputs = 1 << 1;
constexpr int kEmergencyStop = 1 << 2;
constexpr int kEmergencyLift = 1 << 3;
constexpr int kEmergencyCollision = 1 << 5;
constexpr int kEmergencyHighLevel = 1 << 7;
constexpr int kEmergencyFirmwareIncompatible = 1 << 12;
constexpr int kEmergencyImuTimeout = 1 << 16;
constexpr int kEmergencyMowerNotStopped = 1 << 17;

// "LATCH, LIFT": the names of the bits as OpenMower logs them, in bit order.
std::string EmergencyReasonNames(int reason);

// How a drive of the follower for a job ended.
enum class LegEnd { kDone, kCanceled, kGps, kEmergency, kFailed };

// What the node does next for the job.
struct JobStep {
  enum class Kind { kNone, kPlan, kApproach, kFollow };
  Kind kind = Kind::kNone;
  // kPlan: the area to plan.
  std::string area_id;
  // kApproach: where to drive to, over the drivable areas.
  JobPose goal;
  // kFollow: the path and the pose to start at.
  JobPath path;
  size_t start_index = 0;
};

// The app repeats teleop every 100 ms while the joystick is held and sends one zero when it is let go.
constexpr double kTeleopQuietS = 1.0;

// Mower states and actions: IDLE, AREA_RECORDING (remote control and recording of areas), driving to a pose,
// UNDOCKING, MOWING and PAUSED, DOCKING, emergency and sensors.
class LogicCore {
 public:
  explicit LogicCore(const LogicConfig& config);

  void OnHardwareStatus(const HardwareSnapshot& hw);
  // hw/status stopped coming.
  void OnHardwareTimeout();
  void OnPose(const PoseSnapshot& pose);
  // localization/pose stopped coming; events keep the last position.
  void OnPoseTimeout();
  // An RTK fixed position of the GNSS antenna (gps/absolute_pose). The course probe of undocking measures the way the
  // mower moves with these: the fused pose starts from the very heading of the station the probe checks.
  void OnGnssFix(double x, double y);
  // The localization moved its estimate onto GPS again by distance_m, the event is at the new position.
  void OnPositionJump(double to_x, double to_y, double distance_m, bool after_outliers);
  ActionResult OnAction(const std::string& action_id);
  // Recording starts and saves only while the map fits the datum in use, or is still empty and takes it.
  void OnMapStatus(MapDatum datum, const std::string& reason);
  // The change of the map a recording asks for, once; the node sends it to mowbite_map.
  std::optional<MapEdit> TakeMapEdit();
  void OnMapEditResult(bool ok, const std::string& id, const std::string& code, const std::string& message);
  // Remote control reaches the wheels only while recording an area, like in OpenMower. After an emergency or a state
  // change it waits for a zero command or a pause of kTeleopQuietS, so a held joystick does not move the mower.
  // now_s is a monotonic clock.
  std::optional<std::pair<double, double>> OnTeleop(double vx, double vz, double now_s);

  // hw/capabilities: whether the platform has a safety measurement (without one only remote control, no driving) and
  // whether its driver can stop charging (charge_stop; without it undocking leaves out that step).
  void OnCapabilities(bool safety_measured, bool charge_stop = true);
  // Why a drive cannot start now (BUSY, EMERGENCY, NO_SAFETY_MEASUREMENT, position), empty if it can.
  Reason DriveProblem() const;
  // Starts a drive to (x, y): empty if it may start, else why not (event DRIVE_REFUSED). Shown as IDLE with the sub
  // state DRIVING; the node plans the way and hands it to the follower.
  Reason StartDrive(double x, double y);
  // A drive refused by the node (map, planner or follower missing): event DRIVE_REFUSED.
  void RefuseDrive(const Reason& why);
  // The drive is over: arrived, and the reason (DONE, ALREADY_THERE or why not).
  void OnDriveEnded(bool arrived, const Reason& reason);
  bool Driving() const;
  // The commands of the follower reach the wheels only while driving, or mowing on a leg and not paused, and without
  // emergency; teleop only while recording. So safety/drive has one source at a time.
  bool FollowerActive() const;
  // Once when a drive or a leg of the job has to stop at once (emergency, no pose, no safety measurement, pause, skip,
  // abort): the node cancels it, and a drive ends with this reason.
  std::optional<Reason> TakeDriveCancel();

  // The calibration drive (mowbite_calibration): IDLE with the sub state CALIBRATING. It starts only on
  // calibration.start, never by itself; the program drives on calibration/cmd_vel, which reaches the wheels only while
  // calibrating and without emergency (SOURCE_CALIBRATION), and never with the blade. It ends with one zero at an
  // emergency, without a pose or a safety measurement, on a low battery, on calibration.abort (the mower stays where
  // it is) and after CalibrationConfig::timeout without a sign of the program; jumps of the position do not end it.
  void OnCalibrationStatus(const CalibrationStatus& status, double now_s);
  // Every reason against a start now, the logic's first, then the program's (calibration.check); empty: it may start.
  std::vector<Reason> CalibrationProblems(double now_s) const;
  // The program's status while it counts, none without one.
  std::optional<CalibrationStatus> CalibrationStatusNow(double now_s) const;
  // calibration.start: empty when CALIBRATING began, else the first reason against it (also CALIBRATION_REFUSED).
  Reason StartCalibration(double now_s);
  // A start refused by the node (no program, the last drive still stopping): CALIBRATION_REFUSED.
  void RefuseCalibration(const Reason& why);
  // The goal for calibration/run, once after the start.
  std::optional<CalibrationGoal> TakeCalibrationGoal();
  // A sign of the program: the goal taken, or feedback, which also tells the phase (event CALIBRATION when it changes).
  void OnCalibrationSign(double now_s);
  void OnCalibrationFeedback(const std::string& phase, int index, double now_s);
  // The program ended the drive: CALIBRATED or CALIBRATION_FAILED, then IDLE.
  void OnCalibrationEnded(const CalibrationOutcome& outcome);
  // calibration.abort: true when a drive was running; it stops here without the program and the mower stays there.
  bool AbortCalibration(const Reason& why);
  // The watchdog of the program and the battery while it drives; true when the drive ended now.
  bool TickCalibration(double now_s);
  bool Calibrating() const;
  bool CalibrationActive() const;
  // True once when the node has to cancel the goal of calibration/run.
  bool TakeCalibrationCancel();

  // The mow areas of the map and why mowing may not plan from it (empty if it may).
  void OnMowMap(std::vector<JobArea> areas, const Reason& problem);
  // The docking station of the map, none without one.
  void OnMapDock(const std::optional<DockPose>& dock);
  // Where base_link may end leaving the station and where docking begins: the active mow and navigation areas, outside
  // of the obstacles; problem: why the map gives none (MAP_*), empty if it does.
  void OnDrivableAreas(std::vector<Ring> drivable, std::vector<Ring> obstacles, const Reason& problem = Reason());
  // undock_distance of params.set.
  void SetUndockDistance(double distance);
  // The driver reports a debounced contact with the docking station.
  bool DockContact() const;
  // On the docking station mowing (start_mowing) and drives start only after the reference check (G24): the node sets
  // the heading TakeDockCheck returns, once per check, with localization/set_heading and hands the answer to
  // OnDockCheck. A drive to (x, y) asks for its check here, after DriveProblem found nothing.
  void CheckDockBeforeDrive(double x, double y);
  std::optional<double> TakeDockCheck();
  bool DockCheckPending() const;
  // The answer of localization/set_heading: base_link with the heading of the station and its accuracy; ok false
  // without a recent fix or without an answer (message says which). Empty when the check passed and undocking
  // starts, else why mowing or the drive does not start, which also goes out as MOWING_REFUSED or DRIVE_REFUSED.
  Reason OnDockCheck(bool ok, double x, double y, float accuracy, const std::string& message);
  // UNDOCKING: first charging off where the driver can stop it (the node calls hw/charge when TakeChargeOff says so,
  // once, and hands the answer to OnChargeOff), then the path of TakeUndockPath to the follower, whose end goes to OnUndockEnded. After UNDOCKED
  // mowing or the drive starts like away from the station. Charging is allowed again (TakeChargeOn) when undocking
  // ends otherwise, as the mower may still stand on the contacts. TakeUndockOutcome tells the node once how it ended.
  bool Undocking() const;
  bool UndockingForDrive() const;
  bool TakeChargeOff();
  void OnChargeOff(bool ok, const std::string& message);
  std::optional<UndockPath> TakeUndockPath();
  void OnUndockEnded(LegEnd end, const Reason& reason);
  bool TakeChargeOn();
  std::optional<UndockOutcome> TakeUndockOutcome();
  // Stops undocking without UNDOCKING_FAILED, like abort_undocking (also logic/stop_drive for a drive).
  void AbortUndocking(const Reason& why);
  // DOCKING: back onto the station of the map after JOB_COMPLETE and abort_mowing, or on docking.start, where the
  // contacts are at the back (CanDock); else IDLE as before. The node hands the leg of TakeDockingLeg to the follower,
  // once and when nothing else is on its way, and its end to OnDockingLegEnded. On the way onto the station the first
  // raw contact stops the mower (one zero command, G18) and the debounced one ends in DOCKED. A failed try drives out
  // to the approach point, then DOCKING_RETRY or after docking_retry_count DOCKING_FAILED; a bad position waits like
  // mowing, at most gps_timeout. An emergency and abort_docking end it without DOCKING_FAILED.
  bool CanDock() const;
  bool Docking() const;
  // Why docking.start cannot start now, empty if it can: BUSY, ALREADY_THERE on the contacts, EMERGENCY or
  // NO_SAFETY_MEASUREMENT, the position, MAP_*, NO_DOCK, DOCK_UNREACHABLE.
  Reason DockingProblem() const;
  // docking.start: empty when DOCKING began, else why not (also as DRIVE_REFUSED).
  Reason StartDocking();
  std::optional<DockingLeg> TakeDockingLeg();
  void OnDockingLegEnded(LegEnd end, const Reason& reason);
  // docking_approach_distance and docking_retry_count of params.set, for the next docking.
  void SetDockingSettings(double approach_distance, int retry_count);
  // The checkpoint read at the start.
  void RestoreJob(const JobProgress& progress);
  // Why mowing cannot start now, empty if it can.
  Reason StartProblem() const;
  // A start refused by the node (planner or follower missing): event MOWING_REFUSED.
  void RefuseStart(const Reason& why);
  // robot.geometry.set changed the dimensions: event GEOMETRY_CHANGED with the keys that are other now and the keys of
  // the body below the platform's value; with the latter it is a warning with the reason BELOW_DEFAULT, else info. It
  // stops nothing, so it has no resume.
  void GeometryChanged(const std::vector<std::string>& changed, const std::vector<std::string>& below_default,
                       int version);
  bool Mowing() const;
  // IDLE without a drive: no recording, no job (also paused). Settings change only then.
  bool Idle() const;
  // GPS settings of params.set for the position, the recording, and mowing or docking on after a bad position.
  void SetGpsSettings(double max_position_accuracy, double gps_wait_time, double gps_timeout);
  // Called whenever the node has nothing on its way for the job; the step it returns is on its way until its result
  // comes back. now_s is a monotonic clock.
  JobStep NextJobStep(double now_s);
  // The plan of the area asked for by kPlan, or why there is none.
  void OnJobPlanned(std::vector<JobPath> paths, const std::string& fingerprint);
  void OnJobPlanFailed(const std::string& message);
  // The follower got to this pose of the path of kFollow.
  void OnJobProgress(size_t index);
  // The approach or path ended; index is the pose of the path it got to (kFollow only), reason why it ended.
  void OnLegEnded(LegEnd end, size_t index, const Reason& reason);
  // Timers of the job: GPS back, battery, checkpoint period.
  void OnTick(double now_s);
  // Whether the blade should turn now: mowing, not paused, a mow segment (or spinning up for one), mow_with_blade, a
  // safety measurement, telemetry and not on the dock. The node keeps the lease of hw/blade alive only while it is.
  bool BladeWanted() const;
  // hw/blade refused while the blade was wanted: mowing pauses, with reason bits of the answer as for that emergency.
  void OnBladeRefused(const std::string& message, int emergency_reason = 0);
  // The checkpoint to save, once after each change worth keeping.
  std::optional<std::string> TakeCheckpoint();

  RobotStateData RobotState() const;
  std::vector<ActionGroup> Actions() const;
  std::vector<SensorDefinition> Sensors() const;
  std::vector<SensorReading> Readings() const;

  // Events since the last call, oldest first.
  std::vector<Event> TakeEvents();
  // True once after the enabled flags of the actions changed.
  bool TakeActionsChanged();
  // Every change of the emergency reason since the last call, for the log line the app reads.
  std::vector<ReasonChange> TakeReasonChanges();
  // True once after the logic left a state that drives: the wheels get a zero command at once instead of waiting for
  // the cmd_vel watchdog of the driver.
  bool TakeStopRequest();
  // Lines for the log since the last call.
  std::vector<std::string> TakeNotices();
  bool Emergency() const;
  // The lines of map_overlay/json, none outside of AREA_RECORDING.
  json MapOverlay() const;
  bool TakeOverlayChanged();
  const AreaRecorder& recorder() const {
    return recorder_;
  }

 private:
  enum class State { kIdle, kAreaRecording, kDriving, kMowing, kUndocking, kDocking, kCalibrating };
  // Why mowing pauses; manual, emergency and fault wait for continue, GPS goes on by itself.
  static constexpr int kPauseManual = 1 << 0;
  static constexpr int kPauseEmergency = 1 << 1;
  static constexpr int kPauseGps = 1 << 2;
  static constexpr int kPauseFault = 1 << 3;
  enum class Leg { kNone, kPlanning, kApproach, kFollow };

  void SetState(State state);
  // Moves the events of the recorder into the events of the logic, at the current position.
  void PullRecorder();
  void AddEvent(const std::string& type, json details);
  void UpdateEmergency();
  int EmergencyReason() const;
  static const char* StateName(State state);
  std::string CurrentStateName() const;
  // Each empty if nothing stands in the way: BUSY; EMERGENCY or NO_SAFETY_MEASUREMENT; NO_POSITION,
  // POSITION_ACCURACY or NO_HEADING (on the docking station the stricter accuracy of the reference check and no
  // heading); NO_DOCK or DOCK_UNREACHABLE; what else keeps mowing or a drive from starting, busy or not.
  Reason Busy() const;
  Reason Unsafe() const;
  Reason AccuracyReason(float accuracy, double limit) const;
  Reason PositionProblem() const;
  Reason DockProblem() const;
  Reason MowProblem() const;
  Reason MoveProblem() const;
  // DOCK_UNREACHABLE when the way out from base_link at (x, y) on the station ends outside of the drivable areas.
  Reason UndockEndProblem(double x, double y) const;
  // DOCK_UNREACHABLE when the line onto the station begins outside of the drivable areas.
  Reason ApproachProblem() const;
  double DockAccuracyLimit() const;
  // The heading of the mower on the docking station.
  double DockedHeading() const;
  enum class DockCheck { kNone, kMowing, kDrive };
  void BeginDockCheck(DockCheck purpose);
  void BeginDrive(double x, double y);
  // Undocking from base_link at (x, y) for mowing or the drive: charging off first.
  void StartUndocking(DockCheck purpose, double x, double y);
  // Stops the follower on its way out; undocking ends once it stopped, with UNDOCKING_FAILED if failure is set.
  void StopUndocking(const std::optional<Reason>& failure, const Reason& why);
  void EndUndocking(const std::optional<Reason>& failure, const Reason& why);
  void Undocked();
  // Docking with the current settings onto the station of the map.
  DockingSequence NewDocking() const;
  // After JOB_COMPLETE and abort_mowing: DOCKING where the platform docks and the map has a station, else IDLE.
  void GoHome();
  void BeginDocking();
  // The first leg of the next try from where the mower stands, or the wait for a good position.
  void NextDockingTry();
  void OnDockingContact();
  // A try failed: counted at once, or once the mower drove out to the approach point when it stands on the line.
  void DockingTryFailed(const std::string& code, const std::string& message);
  // DOCKING_RETRY and the next try, or DOCKING_FAILED and IDLE (also when last). A copy: why may be the failure kept.
  void CountDockingFailure(Reason why, bool last = false);
  void WaitForDockingGps();
  void TickDocking(double now_s);
  // Stops the leg on its way, if any.
  void CancelDockingLeg(const Reason& why);
  void Docked();
  // IDLE; a leg on its way stops with why.
  void EndDocking(const Reason& why);
  ActionResult OnMowingAction(const std::string& name);
  void StartMowing();
  void Pause(int why, const Reason& reason);
  // Stops a drive or a leg on its way; a drive ends with the first reason given.
  void CancelDrive(const Reason& reason);
  void Unpause(int why);
  void ApplySkips();
  // Ends the job: finished (JOB_COMPLETE) or not (it goes on from the checkpoint next time); home: docking after it.
  void EndJob(bool finished, bool home);
  bool PoseGood() const;
  void CheckpointNow();
  // Ends a calibration drive here: CALIBRATION_FAILED unless done, CALIBRATION end, IDLE; cancel: the program too.
  void EndCalibration(const std::optional<Reason>& failure, bool cancel);

  LogicConfig config_;
  HardwareSnapshot hw_;
  State state_ = State::kIdle;
  bool emergency_ = false;
  // Without telemetry the mower counts as stopped until the driver reports.
  int reason_ = kEmergencyTimeoutInputs;
  // OpenMower logs the first reason as a change from none.
  int logged_reason_ = 0;
  std::vector<ReasonChange> reason_changes_;
  std::optional<PoseSnapshot> pose_;
  double last_x_ = 0;
  double last_y_ = 0;
  bool hw_seen_ = false;
  bool actions_changed_ = true;
  bool stop_requested_ = false;
  bool teleop_armed_ = false;
  bool safety_measured_ = false;
  // Until the driver says otherwise, undocking waits for charging to be off.
  bool charge_stop_ = true;
  bool drive_cancel_ = false;
  std::optional<Reason> cancel_reason_;
  double last_teleop_s_ = -std::numeric_limits<double>::infinity();
  std::vector<Event> events_;
  AreaRecorder recorder_;
  bool map_writable_ = false;
  Reason map_reason_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");

  MowingJob job_;
  Reason mow_map_problem_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");
  std::optional<DockPose> dock_;
  // The reference check on its way: what for, and whether the node has taken it yet.
  DockCheck dock_check_ = DockCheck::kNone;
  bool dock_check_due_ = false;
  // The goal of a drive from the station, for after undocking.
  double drive_x_ = 0;
  double drive_y_ = 0;
  std::vector<Ring> drivable_;
  std::vector<Ring> obstacles_;
  Reason drivable_problem_;
  // Undocking: what for, where it started and the way it faces, what the node does next for it, and once it is
  // stopping, whether that ends in UNDOCKING_FAILED.
  enum class UndockStep { kChargeOff, kDrive };
  DockCheck undock_for_ = DockCheck::kNone;
  UndockStep undock_step_ = UndockStep::kChargeOff;
  double undock_x_ = 0;
  double undock_y_ = 0;
  double undock_heading_ = 0;
  std::optional<Point2> undock_fix_start_;
  bool charge_off_due_ = false;
  bool charge_on_due_ = false;
  std::optional<UndockPath> undock_path_due_;
  bool undock_stopping_ = false;
  std::optional<Reason> undock_failure_;
  Reason undock_stop_why_;
  std::optional<UndockOutcome> undock_outcome_;
  // Docking: its tries, the leg due for the node and the kind of the one on its way (and of the last one), stopped at a
  // first contact, a failed try waiting for the mower to drive out, and the wait for a good position.
  DockingSequence docking_;
  std::optional<DockingLeg> docking_leg_due_;
  std::optional<DockingLeg::Kind> docking_leg_;
  DockingLeg::Kind docking_last_kind_ = DockingLeg::Kind::kApproach;
  bool docking_hold_ = false;
  std::optional<Reason> docking_failure_;
  bool docking_gps_wait_ = false;
  std::optional<double> docking_gps_since_;
  std::optional<double> docking_gps_good_since_;
  std::string session_id_;
  int pause_ = 0;
  Reason pause_reason_;
  Leg leg_ = Leg::kNone;
  bool approached_ = false;
  bool skip_path_ = false;
  bool skip_area_ = false;
  std::string announced_area_;
  std::optional<double> spinup_since_;
  std::optional<double> gps_good_since_;
  std::optional<double> battery_low_since_;
  double last_checkpoint_s_ = 0;
  bool checkpoint_due_ = false;
  bool blades_running_ = false;
  bool last_start_ok_ = false;
  std::vector<std::string> notices_;
  // Calibration: the program's status and when it came, the goal due for the node, the cancel of the program, its last
  // sign, the phase it reported last, and since when the battery is low.
  std::optional<CalibrationStatus> calibration_status_;
  double calibration_status_s_ = 0;
  std::optional<CalibrationGoal> calibration_goal_due_;
  bool calibration_cancel_ = false;
  double calibration_sign_s_ = 0;
  std::string calibration_phase_;
  int calibration_index_ = 0;
  std::optional<double> calibration_battery_since_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_CORE_H
