#ifndef MOWBITE_LOGIC_LOGIC_CORE_H
#define MOWBITE_LOGIC_LOGIC_CORE_H

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/docking.h"
#include "mowbite_logic/home_watch.h"
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
  // The driver holds motion and blade for an update (hold.json), also one it cannot read.
  bool hold = false;
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
  // Monotonic s when the node got the report: the check of the charge and its watch measure time with it.
  double time_s = 0;
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
  // 6S Li-ion; empty is 0 % for the app and where mowing goes home (the highest voltage of battery_low_window s below
  // it). Assumption until the discharge curve of the Tron is measured: 3.5 V per cell.
  double battery_empty_voltage = 21.0;
  double battery_full_voltage = 25.2;
  double battery_low_window = 20.0;
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
  // Mowing goes home below this voltage (V), held for battery_critical_time (s) against dips under load; below empty.
  // Assumption until the discharge curve is measured: 3.28 V per cell.
  double battery_critical_voltage = 19.7;
  double battery_critical_time = 2.0;
  // Mowing goes home with the battery this hot (°C), 0 for never, and does not start then.
  double battery_hot_temperature = 0;
  // Settings of params.set: the blade motor this hot (°C) sends mowing home, a start needs it below cold; rain_mode
  // other than 0 sends it home after rain_check_seconds of rain.
  double motor_hot_temperature = 70.0;
  double motor_cold_temperature = 40.0;
  int rain_mode = 0;
  double rain_check_seconds = 20.0;
  // m: farther from the start of a path the mower first drives there over the drivable areas.
  double approach_distance = 0.5;
  // s between checkpoints while a path is mowed.
  double checkpoint_period = 30.0;
  // m ahead of base_link where the charging contacts of the mower are. Below 0 they are behind it: it backs onto the
  // docking station and stands there facing away from the heading of the station in the map. Above 0 it drives on
  // forwards and faces that heading. 0: the platform has no charging contacts and does not dock (NO_CHARGE_CONTACTS).
  double dock_contact_x = 0;
  // On the docking station mowing and drives start only after the reference check (G24): a position at least
  // dock_max_position_accuracy (m) and, with the heading of the station set, base_link within dock_reference_limit (m)
  // of the recorded station.
  double dock_max_position_accuracy = 0.03;
  double dock_reference_limit = 0.10;
  // After the reference check the mower leaves the station before mowing or a drive starts, straight against the way
  // in: forwards with the contacts behind base_link, else backwards (PlanLeave).
  UndockConfig undock;
  // After JOB_COMPLETE, abort_mowing and on docking.start the mower drives onto the station contacts first; a try that
  // fails also drives out like undocking.
  DockConfig dock;
  // DOCKED only with the charge confirmed on the contacts, CHARGING_LOST in IDLE on the station (G19).
  ChargeConfig charge;
  CalibrationConfig calibration;
  // Seed of the job, session and event ids; the node draws it, the journal keeps it.
  uint64_t id_seed = 0;
};

// Why the battery voltages of the config do not fit (critical < empty < full), empty when they do.
std::string BatteryVoltagesProblem(const LogicConfig& config);

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
  // Only start_mowing while off: the reason of MOWING_REFUSED a start would get now.
  std::string reason = "";
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
  // While mowing: the place of the area in the areas of the job and their number; else -1.
  int task_index = -1;
  int task_count = -1;
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

// How a command of the catalog (mowbite_api::Commands) went, the same whether its action or its RPC sent it.
struct CommandOutcome {
  // The params do not fit (-32602), with the field at fault; nothing happened.
  std::string invalid;
  std::string invalid_field;
  bool accepted = false;
  // emergency.reset: the node asks the safety node, whose answer decides.
  bool release = false;
  // The answer: state and sub_state right after it and the fields of the command.
  json result = json::object();
  // Why not, and the data of the -32600: event names the group of its texts, event_id the event it also went out as.
  Reason why;
  json data = json::object();
};

struct ReasonChange {
  int from = 0;
  int to = 0;
};

// What the core asks of the node, one type per command, in the order it came up (TakeEffects).
// A drive or a leg of the job has to stop at once (emergency, no pose, no safety measurement, pause, skip, abort): the
// node cancels it, and a drive ends with why, the first reason given.
struct DriveCancel {
  Reason why;
};
// The logic left a state that drives: one zero command to the wheels at once instead of waiting for the cmd_vel
// watchdog of the driver.
struct StopRequest {};
// The goal of calibration/run is canceled.
struct CalibrationCancel {};
// hw/charge off before the mower leaves the contacts (the answer goes to OnChargeOff), or on again.
struct ChargeOff {};
struct ChargeOn {};
// localization/set_heading with the heading of the station for the reference check, whose answer goes to OnDockCheck.
struct ReferenceCheck {
  double heading = 0;
};
// The enabled flags of the actions changed.
struct ActionsChanged {};
// A line for the log.
struct Notice {
  std::string line;
};
// checkpoint.json with this text, after each change worth keeping.
struct Checkpoint {
  std::string text;
};
// The phases of a run of the schedule, numbered as in mowbite_msgs/RunStatus.
enum class RunPhase : uint8_t { kNone = 0, kAnnouncing, kChecking, kUndocking, kMowing, kEnded };
// logic/run: the run of the schedule is in a new phase; with kEnded how it ended (end_code, detail).
struct RunChanged {
  std::string run_id;
  RunPhase phase = RunPhase::kNone;
  std::string end_code;
  std::string detail;
};
// The index of a type is its number in the journal: new ones only at the end.
using Effect = std::variant<DriveCancel, StopRequest, CalibrationCancel, ChargeOff, ChargeOn, ReferenceCheck,
                            UndockPath, UndockOutcome, CalibrationGoal, MapEdit, Event, ReasonChange, ActionsChanged,
                            Notice, Checkpoint, RunChanged>;

// When the node carries an effect out in its step: what halts first, then (after os-session.json) what acts, among
// it what starts a motion, and last what is written or published.
enum class EffectPhase { kHalt, kAct, kRecord };
EffectPhase PhaseOf(const Effect& effect);
// The name of the type of an effect, for logs and the export of a journal.
const char* EffectName(const Effect& effect);

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
// The restart hold of the logic (mowbite_hold/restart.h), also without a driver.
constexpr int kEmergencyUnexpectedRestart = 1 << 21;

// What the start found about the end before it, for the event UNEXPECTED_RESTART.
struct RestartReport {
  // KERNEL_CRASH, OS_CRASH or POWER_LOSS.
  std::string cause;
  std::string detail;
  // unix s, 0 when not known.
  int64_t crashed_at = 0;
  std::string state_before;
  std::string job_id_before;
  // The mower waits for a release, also by a hold from before.
  bool hold = false;
  // {node, seconds, limit} when the watch of the container ended it for a stalled node, else null.
  json stalled;
};

// "LATCH, LIFT": the names of the bits from the catalog in bit order, one UNKNOWN for all bits without a name.
std::string EmergencyReasonNames(int reason);

// Called with every event the core adds, before it is queued; for the tests against the catalog, none by default.
using EventObserver = void (*)(const std::string& type, const json& details);
void SetEventObserver(EventObserver observer);

// What the watch of the way home takes from a report of the hardware.
HomeSample HomeSampleOf(const HardwareSnapshot& hw, double now_s);

// How a drive of the follower for a job ended. kNoWay: the planner found no way for an approach from where the mower
// stands (NO_WAY or START_OUTSIDE of PlanTransit); for any other leg it counts as kFailed.
enum class LegEnd { kDone, kCanceled, kGps, kEmergency, kFailed, kNoWay };

// What the node does next for the job.
struct JobStep {
  enum class Kind { kNone, kPlan, kApproach, kFollow, kSkippedPath };
  Kind kind = Kind::kNone;
  // kPlan: the area to plan.
  std::string area_id;
  // kApproach: where to drive to, over the drivable areas.
  JobPose goal;
  // kFollow: the path and the pose to start at. kSkippedPath: the same for a piece of the path a skip left; the path
  // ends with the piece and is a transit all along.
  JobPath path;
  size_t start_index = 0;
};

// s after the answer of the safety node within which the end of the emergency counts as released by the command.
constexpr double kReleaseSourceS = 2.0;

// A job asks the planner again this often in a row when it gave no answer; then mowing ends and the mower goes home.
constexpr int kPlannerRetries = 3;

// m of the path a skip left that the mower drives on at a time, the blade off, while the planner finds no way from
// there to the next path; after each piece the job asks for that way again.
constexpr double kSkippedPathPiece = 2.0;

// Whether the planner gave no answer: PLANNER_TIMEOUT or PLANNER_MISSING.
bool PlannerSilent(const Reason& why);

// A start of a plan of the schedule (mowbite_msgs/StartRun).
struct ScheduleRun {
  std::string run_id;
  // The list of the plan in order, the job it names; empty: every mowing area.
  std::vector<std::string> area_ids;
  // Paused or not on the map: this run leaves them out, the job keeps them.
  std::vector<std::string> pass_over;
  bool restart = false;
  bool has_angle = false;
  double angle_offset_deg = 0;
  bool dark_allowed = false;
  // Home at the next sunset.
  bool until_sunset = true;
  // Home this long (s) after the start; 0: no end.
  int64_t end_in_s = 0;
};

// s a start of the schedule is announced (S7) before the logic checks again and leaves the station.
constexpr double kAnnounceS = 10.0;

// s before the next start of the schedule within which no update hold is granted.
constexpr int64_t kHoldBeforeStartS = 1800;
// Whether the next start of the schedule (unix s, 0 for none) keeps a hold from being granted at now (unix s): within
// kHoldBeforeStartS of it, or once it is due.
bool StartBlocksHold(int64_t next_start, int64_t now);

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
  // hw/wheel_odom, which the node passes only in AREA_RECORDING: vx (m/s, negative backwards) since the one before
  // gives the way the recorder tells reversing by. More than RecorderConfig::kWheelGap after it, a value that is no
  // number, or none for that long (OnTick), is a break of the driven track.
  void OnWheelOdom(double vx, double now_s);
  bool AreaRecording() const;
  // The command the action stands for, as its RPC without client; kUnknown for an action that is not the logic's.
  ActionResult OnAction(const std::string& action_id);
  // A command by its RPC name with its params (client among them). node_problem: what only the node knows against a
  // start (planner, follower, the last drive still stopping), taken after the logic's own reasons. The events of this
  // step carry client.
  CommandOutcome Command(const std::string& name, const json& params, const Reason& node_problem = Reason());
  // The answer to an accepted command: state and sub_state as robot_state has them now, then fields.
  json CommandAnswer(const json& fields = json::object()) const;
  // The data of a refusal: event (the group of the reason), the reason with its rating and fields, state, sub_state
  // and event_id when the refusal went out as that event too.
  json RefusalData(const std::string& group, const Reason& why, const std::string& event_id = "") const;
  // Recording starts and saves only while the map fits the datum in use, or is still empty and takes it.
  void OnMapStatus(MapDatum datum, const std::string& reason);
  // The answer of mowbite_map to a MapEdit a recording asked for.
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
  // A start refused by the node (no program, the last drive still stopping): CALIBRATION_REFUSED. After a start the
  // node hands the CalibrationGoal to calibration/run.
  void RefuseCalibration(const Reason& why);
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
  // the heading of the ReferenceCheck with localization/set_heading and hands the answer to OnDockCheck. A drive to
  // (x, y) asks for its check here, after DriveProblem found nothing.
  void CheckDockBeforeDrive(double x, double y);
  bool DockCheckPending() const;
  // The answer of localization/set_heading: base_link with the heading of the station and its accuracy; ok false
  // without a recent fix or without an answer (message says which). Empty when the check passed and undocking
  // starts, else why mowing or the drive does not start, which also goes out as MOWING_REFUSED or DRIVE_REFUSED.
  Reason OnDockCheck(bool ok, double x, double y, float accuracy, const std::string& message);
  // UNDOCKING: first charging off where the driver can stop it (ChargeOff, whose answer goes to OnChargeOff), then
  // the UndockPath to the follower, whose end goes to OnUndockEnded. After UNDOCKED mowing or the drive starts like
  // away from the station. Charging is allowed again (ChargeOn) when undocking ends otherwise, as the mower may still
  // stand on the contacts. The UndockOutcome tells the node how it ended. Docking uses ChargeOff, OnChargeOff and
  // ChargeOn the same way before it drives off the contacts.
  bool Undocking() const;
  bool UndockingForDrive() const;
  void OnChargeOff(bool ok, const std::string& message);
  void OnUndockEnded(LegEnd end, const Reason& reason);
  // Stops undocking without UNDOCKING_FAILED, like abort_undocking (also logic/stop_drive for a drive).
  void AbortUndocking(const Reason& why);
  // DOCKING: back onto the station of the map after JOB_COMPLETE, abort_mowing or a reason of HomeWatch, or on
  // docking.start, where the platform has charging contacts (CanDock); else IDLE as before. Each but JOB_COMPLETE begins with
  // the event DOCKING and its reason. The node hands the leg of TakeDockingLeg to the follower,
  // once and when nothing else is on its way, and its end to OnDockingLegEnded. On the way onto the station the first
  // raw contact stops the mower (one zero command, G18); the debounced one, also at the end of a leg that failed,
  // starts the check of the charge (ChargeCheck), and only a confirmed charge ends in DOCKED. A failed try drives out
  // to the approach point, off the contacts with charging off first where the driver can stop it, then DOCKING_RETRY
  // or after docking_retry_count DOCKING_FAILED; the last try without a charge current (NO_CHARGE_CURRENT) leaves the
  // mower on the contacts, and a stable charge current there later still ends in DOCKED. A bad position waits like
  // mowing, at most gps_timeout, then DOCKING_FAILED NO_POSITION, NO_HEADING or POSITION_ACCURACY. An emergency and
  // abort_docking end it without DOCKING_FAILED. In IDLE on the contacts CHARGING_LOST tells when the contact or the
  // charge current goes away (ChargeConfig); the mower stays.
  // Where the planner gives no answer for the way to the point in front of the station (PlannerSilent as the reason
  // of the approach), the try takes the straight way (DockConfig::direct): free for the body in every heading, with
  // direct_margin and the accuracy of a good position to spare (CheckDirectRoute), it is the event DOCKING_DIRECT and
  // the leg kDirect, then the line as after the approach. Not free, or failed on the way, the try fails as
  // approach_failed with cause and direct_blocked; with the switch off only with cause.
  bool CanDock() const;
  bool Docking() const;
  // Why docking.start cannot start now, empty if it can: BUSY, NO_CHARGE_CONTACTS, ALREADY_THERE on the contacts,
  // EMERGENCY or NO_SAFETY_MEASUREMENT, the position, MAP_*, NO_DOCK, DOCK_UNREACHABLE.
  Reason DockingProblem() const;
  // docking.start: empty when DOCKING began, else why not (also as DRIVE_REFUSED).
  Reason StartDocking();
  std::optional<DockingLeg> TakeDockingLeg();
  void OnDockingLegEnded(LegEnd end, const Reason& reason);
  // docking_approach_distance and docking_retry_count of params.set, for the next docking.
  void SetDockingSettings(double approach_distance, int retry_count);
  // m from base_link to the farthest point of the body (Geometry::TurningRadius), once the dimensions are loaded and
  // after every change; until then no straight way counts as free.
  void SetTurningRadius(double radius);
  // rain_mode, rain_check_seconds and the temperatures of the blade motor of params.set, for the next run.
  void SetHomeSettings(int rain_mode, double rain_check_seconds, double motor_hot_temperature,
                       double motor_cold_temperature);
  // dock_at_sunset of params.set: a run goes home at the first sunset after its start or after the user resumed it.
  // At once in a run: off drops its deadline, on reckons the first sunset from now.
  void SetSunsetSettings(bool dock_at_sunset);
  // time/clock: whether the wall clock is safe, why not, and wall minus monotonic s when it came.
  void OnClock(bool safe, const std::string& reason, double unix_minus_mono_s);
  // gps/datum, the datum in use (deg): where the sun is reckoned.
  void OnDatum(double latitude, double longitude);
  // The checkpoint read at the start.
  void RestoreJob(const JobProgress& progress);
  // A plan of the schedule starts (S1, S3, S7): empty when its announcement began, else why not. INVALID (no event)
  // for a list the logic does not take; any other refusal is MOWING_REFUSED with source schedule. Checked in this
  // order: busy, hold, on the contacts (NOT_DOCKED), the clock (CLOCK_UNSAFE), the start problems of the areas left,
  // without dark_allowed the sun (NO_POSITION without datum, DARK), then node_problem. kAnnounceS later (OnTick) the
  // checks run again, then the reference check and undocking as for mowing.start. An emergency, a hold, leaving the
  // contacts and any command to the logic cancel the announcement (SCHEDULE_START_CANCELED); the command then runs.
  // logic/run (RunChanged) follows the run until the mower is idle again; nothing of it is kept over a restart (S5).
  Reason StartRun(const ScheduleRun& run, const Reason& node_problem, double now_s);
  // The schedule pauses areas of its run run_id while it mows, empty for all: empty when taken, else why not. The
  // current area among them is skipped (AREA_SKIPPED SCHEDULE); with none left the run ends with SCHEDULE pause and
  // the job stays open, going home as for a reason of HomeWatch, else idle where it is.
  Reason DropTasks(const std::string& run_id, const std::vector<std::string>& area_ids);
  // The angle of the lanes the open job asks of the planner (the one of the plan that began it), none without.
  std::optional<double> PlanAngle() const;
  // Why mowing of these areas (none for all) cannot start now, empty if it can.
  Reason StartProblem(const std::vector<std::string>& tasks = {}) const;
  // A start refused by the node (planner or follower missing): event MOWING_REFUSED.
  void RefuseStart(const Reason& why);
  // reset_emergency did not release: EMERGENCY_RESET_REFUSED with the reason of the driver, of S2 or of the node;
  // returns the id of the event.
  std::string RefuseReset(const Reason& why);
  // The restart hold (G35): while it stands the logic counts as in emergency with UNEXPECTED_RESTART, so nothing that
  // drives or mows starts, whatever the driver reports. The node sets it at the start and clears it on emergency.reset
  // once the file is gone.
  void SetRestartHold(bool held);
  bool RestartHeld() const {
    return restart_hold_;
  }
  // The event UNEXPECTED_RESTART, rated by cause and hold.
  void ReportRestart(const RestartReport& report);
  // The job of mowing, finished or not; empty without one.
  const std::string& JobId() const;
  // The safety node released after emergency.reset or STOP (now_s monotonic): the end of the emergency that follows
  // within kReleaseSourceS carries the source, rpc with the client of the command or button. Without that end the
  // tick after kReleaseSourceS refuses it with STILL_ACTIVE and the bits that stay.
  void OnReleaseAnswered(double now_s);
  // STOP was let go after the gesture of the driver (hw/button LONG): true in an emergency, which the node then
  // releases like emergency.reset with source button; false outside of one.
  bool OnStopRelease();
  // robot.geometry.set changed the dimensions: event GEOMETRY_CHANGED with the keys that are other now and the keys of
  // the body below the platform's value; with the latter it is a warning with the reason BELOW_DEFAULT, else info. It
  // stops nothing, so it has no resume.
  void GeometryChanged(const std::vector<std::string>& changed, const std::vector<std::string>& below_default,
                       int version);
  // map/set_recorded_body failed after robot.geometry.set changed the body: event RECORDED_BODY_KEPT, the areas
  // recorded here keep the old body. areas: how many carry the id of this installation, below 0 when not known.
  void RecordedBodyKept(int areas, const std::string& code, const std::string& message);
  bool Mowing() const;
  // IDLE without a drive: no recording, no job (also paused). Settings change only then.
  bool Idle() const;
  // The hold of an update (system.hold), until in unix s, 0 for none. While it or the hold the driver reports in
  // hw/status lasts, nothing starts the wheels or the blade: start_mowing, continue, start_area_recording, drives and
  // docking.start are refused with the reason HOLD.
  void SetHold(int64_t until);
  // until of the logic's own hold, 0 without one.
  int64_t HoldUntil() const {
    return hold_until_;
  }
  bool Held() const;
  // HOLD while held, with until when the logic holds; else STORAGE_NOT_WRITABLE while a file of the restart hold
  // cannot be written; else empty. mowing.continue checks only the hold: PAUSED holds on the disk already.
  Reason HoldProblem() const;
  // The file in data_dir of the restart hold (os-session.json, restart-hold.json) that the node cannot write, empty
  // once it can again. While one cannot, every start from IDLE is refused with STORAGE_NOT_WRITABLE; the event STORAGE
  // tells it once each time it begins.
  void SetUnwritableFile(const std::string& file);
  // The record of a start from IDLE did not reach the disk (G35), and the node carried out nothing of the start: the
  // start ends at once with the refusal of its kind and STORAGE_NOT_WRITABLE, and the mower is IDLE again.
  void AbandonStart(const std::string& file);
  // A hold is granted only in IDLE with nothing on its way, not even the reference check on the docking station.
  bool HoldAllowed() const;
  // GPS settings of params.set for the position, the recording, and mowing or docking on after a bad position.
  void SetGpsSettings(double max_position_accuracy, double gps_wait_time, double gps_timeout);
  // Called whenever the node has nothing on its way for the job; the step it returns is on its way until its result
  // comes back. now_s is a monotonic clock.
  JobStep NextJobStep(double now_s);
  // The plan of the area asked for by kPlan, or why there is none: PLANNING_FAILED with the answer of the planner,
  // PLANNER_TIMEOUT or PLANNER_MISSING without one.
  void OnJobPlanned(std::vector<JobPath> paths, const std::string& fingerprint);
  void OnJobPlanFailed(const Reason& why);
  // The follower got to this pose of the path of kFollow.
  void OnJobProgress(size_t index);
  // The approach or path ended; index is the pose of the path it got to (kFollow only), reason why it ended. An
  // approach or a plan the planner gave no answer for (PlannerSilent) is asked for again, up to kPlannerRetries times
  // in a row and never after a plan that timed out. Then the job ends, kept for the next start, and the mower goes
  // home with that reason at the event DOCKING; without a way home, or in a pause that waits for the user, mowing
  // pauses as for a failed approach or plan.
  // After a skip an approach that ended with kNoWay does not pause while the mower stands on the path it left and
  // poses of it are left: the next step is kSkippedPath, a piece of that path with the blade off, and after it the
  // approach is asked for again. For kSkippedPath index is the pose of that path the mower got to.
  void OnLegEnded(LegEnd end, size_t index, const Reason& reason);
  // Timers of the job: GPS back, going home (HomeWatch), checkpoint period. While mowing and not waiting for the user
  // the first reason that holds sends the mower home with the event DOCKING; without a way home only the battery
  // ends mowing (BATTERY_LOW, IDLE). A pause that waits for the user (manual, emergency, fault) never goes home: there
  // only the battery ends it the same way.
  void OnTick(double now_s);
  // Whether the blade should turn now: mowing, not paused, a mow segment (or spinning up for one), mow_with_blade, a
  // safety measurement, telemetry and not on the dock. The node keeps the lease of hw/blade alive only while it is.
  bool BladeWanted() const;
  // hw/blade refused while the blade was wanted: mowing pauses, with reason bits of the answer as for that emergency.
  void OnBladeRefused(const std::string& message, int emergency_reason = 0);

  RobotStateData RobotState() const;
  std::vector<ActionGroup> Actions() const;
  std::vector<SensorDefinition> Sensors() const;
  std::vector<SensorReading> Readings() const;

  // Everything the node has to do since the last call, oldest first; a checkpoint with the progress now. The next step
  // of the job (NextJobStep), of docking (TakeDockingLeg) and the overlay stay queries.
  std::vector<Effect> TakeEffects();
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
  enum class Leg { kNone, kPlanning, kApproach, kFollow, kSkippedPath };

  void SetState(State state);
  // The command after its params were checked: empty when accepted, else why not; extra fields of the answer go to
  // out->result.
  Reason RunCommand(const mowbite_api::Command& command, const json& params, const Reason& node_problem,
                    CommandOutcome* out);
  Reason RecordingCommand(const mowbite_api::Command& command, const json& params, CommandOutcome* out);
  // WRONG_STATE with state and sub_state of robot_state.
  Reason WrongState() const;
  // Moves the events of the recorder into the events of the logic, at the current position.
  void PullRecorder();
  // A break of the driven track while OnWheelOdom does not come.
  void TickRecording(double now_s);
  // Returns the id of the event.
  std::string AddEvent(const std::string& type, json details);
  // 32 letters and digits from id_seed: the ids of jobs, sessions and events.
  std::string NewId();
  // report: called for a report of the driver or its absence, whose first one only sets the state.
  void UpdateEmergency(bool report = true);
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
  // Of these areas, none for all.
  Reason MowProblem(const std::vector<std::string>& tasks) const;
  Reason MoveProblem() const;
  // DOCK_UNREACHABLE when the way out from base_link at (x, y) on the station ends outside of the drivable areas.
  Reason UndockEndProblem(double x, double y) const;
  // DOCK_UNREACHABLE when the line onto the station begins outside of the drivable areas.
  Reason ApproachProblem() const;
  double DockAccuracyLimit() const;
  // The heading of the mower on the docking station.
  double DockedHeading() const;
  // The contacts lie behind base_link: the mower backs onto the station and leaves it forwards.
  bool ContactsBack() const;
  // "straight ahead" or "straight back": the way off the station, for the messages.
  const char* LeaveWords() const;
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
  // After JOB_COMPLETE, abort_mowing and a reason to go home: DOCKING where the platform docks and the map has a
  // station, else IDLE. why: the reason of the event DOCKING, none after JOB_COMPLETE.
  void GoHome(const Reason& why = Reason());
  // Whether mowing can go home from here: a station the platform docks onto, a way to it, safe, not on it already.
  bool HomeReachable() const;
  HomeConfig HomeRules() const;
  // Samples the rules of going home while mowing, sets the deadline of the sunset and acts on the reasons that hold.
  void WatchHome(double now_s);
  // What the reasons to stop mowing do (question 47 iii); they are those of HomeWatch in its order, then dark (empty
  // when the sunset is not due). With a way home and no pause that waits for the user: home with the first. Else a
  // reason of the battery ends the run. Else a run without a pause pauses with the first and waits for the user (G11).
  void ActOnHome(const Reason& dark);
  // mowing.continue: rain or heat of HomeWatch that still holds while there is no way home.
  Reason ContinueProblem() const;
  // The user resumed: a new deadline of the sunset, and the reasons without dark act at once.
  void ResumedByUser();
  // The deadline of the sunset of this run in monotonic s, when it is empty and the switch is on.
  void TrySunset(double now_s);
  Reason DarkWhy(double now_s) const;
  // The reason of the event DOCKING for a rule of HomeWatch.
  Reason HomeWhy(HomeReason reason) const;
  // The battery ends the run where it cannot go home: BATTERY_LOW with the limit it fell below, IDLE.
  void BatteryEndsMowing(HomeReason reason);
  void BeginDocking();
  // The first leg of the next try from where the mower stands, or the wait for a good position.
  void NextDockingTry();
  void OnDockingContact();
  // A try failed: counted at once, or once the mower drove out to the approach point when it stands on the line.
  void DockingTryFailed(const Reason& why);
  // The planner gave no answer for the approach of a try: the straight way, or the try failed.
  void DockingWithoutPlanner(const Reason& silent);
  // The leg for the node; a way out from the contacts waits until charging is off, where the driver can stop it.
  void SetDockingLeg(DockingLeg leg);
  // Only while docking_out_after_charge_off_ holds the way out; OnChargeOff checks it.
  void DockingChargeOff(bool ok, const std::string& message);
  // Once the docking switched charging off: allowed again after the way out and when docking ends.
  void AllowChargeAgain();
  // The check of the charge from the debounced contact on, its samples and how it ended.
  void BeginChargeCheck();
  ChargeSample ChargeSampleNow() const;
  void OnChargeResult(ChargeCheck::Result result);
  // IDLE on the contacts: DOCKED once the charge is confirmed there, CHARGING_LOST when it stops (ChargeWatch).
  void WatchCharge();
  void ChargingLost(const Reason& why);
  // DOCKING_RETRY and the next try, or DOCKING_FAILED and IDLE (also when last). A copy: why may be the failure kept.
  void CountDockingFailure(Reason why, bool last = false);
  void WaitForDockingGps();
  void TickDocking(double now_s);
  // Stops the leg on its way, if any.
  void CancelDockingLeg(const Reason& why);
  void Docked();
  // IDLE; a leg on its way stops with why.
  void EndDocking(const Reason& why);
  // What a mowing.start asks for: the areas in order (none for all) and whether to begin anew; a start of the schedule
  // also the areas it leaves out.
  struct StartRequest {
    std::vector<std::string> tasks;
    bool restart = false;
    std::vector<std::string> pass_over;
    // Taken by a job that begins with this start, kept by one it goes on with.
    std::optional<double> angle_offset_deg;
  };
  // The start problems of the areas of the request that are not passed over; NO_MOWING_AREA when none is left.
  Reason RequestMowProblem(const StartRequest& request) const;
  // Why a start of the schedule cannot go now (StartRun without node_problem), empty if it can.
  Reason ScheduleProblem(const StartRequest& request, const ScheduleRun& run, double now_s) const;
  static StartRequest RunRequest(const ScheduleRun& run);
  // Why the list of a start of the schedule is not taken, empty if it is.
  std::string InvalidRun(const ScheduleRun& run) const;
  // The announcement ran out: checked again, then the reference check, or the refusal ends the run.
  void EndAnnouncement(double now_s);
  // An emergency, a hold or leaving the contacts cancels the announcement; a command cancels it with source rpc.
  void WatchAnnouncement();
  void CommandCancels();
  void CancelAnnouncement(const std::string& source);
  void SetRunPhase(RunPhase phase);
  // How the run ends, the first word counts; FinishRun tells it (EMERGENCY or STOPPED without one) and drops the run.
  void RunEnds(const std::string& code, const std::string& detail = "");
  void FinishRun();
  // Whether a run goes home at sunset: one of the schedule unless it may mow in the dark without until_sunset, one by
  // hand with dock_at_sunset.
  bool SunsetWanted() const;
  // AREA_SKIPPED SCHEDULE for the areas the job passed over.
  void AreasPassedOver(const std::vector<JobArea>& areas);
  // The first deadline of the run that is due: the sunset, then the end of the plan; empty when none is.
  Reason DeadlineDue(double now_s);
  // tasks and restart of mowing.start; the problem with them for -32602 and its field, empty if none. The request
  // is set only without a problem.
  std::string ReadStartRequest(const json& params, StartRequest* request, std::string* field) const;
  // mowing.start as ReadStartRequest read it: the refusal, or the reference check on the station, or mowing.
  Reason RequestStart(StartRequest request, const Reason& node_problem, CommandOutcome* out);
  // Continues the open job, or begins the one mowing.start asked for.
  void StartMowing();
  void Pause(int why, const Reason& reason);
  // Stops a drive or a leg on its way; a drive ends with the first reason given.
  void CancelDrive(const Reason& reason);
  void Unpause(int why);
  void ApplySkips();
  // The follower drives a leg of the job.
  bool LegDrives() const;
  // The path the last skip left, while the job stands at the start of the one after it, the mower within
  // approach_distance of the pose it got to on the path left, and poses of it are left; else nothing.
  const JobPath* SkippedPathHere() const;
  // Its next piece as the step of the job: the poses within kSkippedPathPiece m of the one the mower stands at, at
  // least the next one.
  JobStep SkippedPiece(const JobPath& path);
  // Ends the job: finished (JOB_COMPLETE) or not (it goes on from the checkpoint next time); home: docking after it,
  // with why as the reason of the event DOCKING.
  void EndJob(bool finished, bool home, const Reason& why = Reason());
  // The planner gave the job no answer: it asks again where retry is set and tries are left, else the mower goes
  // home; pause: the pause where it cannot.
  void JobWithoutPlanner(const Reason& silent, const Reason& pause, bool retry);
  bool PoseGood() const;
  void CheckpointNow();
  // Ends a calibration drive here: CALIBRATION_FAILED unless done, CALIBRATION end, IDLE; cancel: the program too.
  void EndCalibration(const std::optional<Reason>& failure, bool cancel);
  // STORAGE_NOT_WRITABLE with the file while one cannot be written, else empty.
  Reason StorageProblem() const;
  // The effects of one type in effects_: whether one waits; a flag added once or every one removed; the waiting one
  // replaced; every one removed.
  template <typename T>
  bool Pending() const;
  template <typename T>
  void SetDue(bool due);
  template <typename T>
  void Replace(T value);
  template <typename T>
  void Drop();
  void Notify(std::string line);

  LogicConfig config_;
  std::mt19937_64 ids_;
  HardwareSnapshot hw_;
  State state_ = State::kIdle;
  bool emergency_ = false;
  // Without telemetry the mower counts as stopped until the driver reports.
  int reason_ = kEmergencyTimeoutInputs;
  // OpenMower logs the first reason as a change from none.
  int logged_reason_ = 0;
  std::optional<PoseSnapshot> pose_;
  double last_x_ = 0;
  double last_y_ = 0;
  bool hw_seen_ = false;
  bool restart_hold_ = false;
  // The file of SetUnwritableFile.
  std::string unwritable_file_;
  bool teleop_armed_ = false;
  bool safety_measured_ = false;
  // Until the driver says otherwise, undocking waits for charging to be off.
  bool charge_stop_ = true;
  double last_teleop_s_ = -std::numeric_limits<double>::infinity();
  // What the node has not taken yet, oldest first; flags (StopRequest, ChargeOff, ...) at most once.
  std::vector<Effect> effects_;
  AreaRecorder recorder_;
  // The last OnWheelOdom.
  std::optional<double> last_wheel_s_;
  bool map_writable_ = false;
  Reason map_reason_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");

  MowingJob job_;
  // The start accepted last, until mowing begins.
  StartRequest start_request_;
  Reason mow_map_problem_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");
  std::optional<DockPose> dock_;
  // The reference check on its way and what for.
  DockCheck dock_check_ = DockCheck::kNone;
  // The goal of a drive from the station, for after undocking.
  double drive_x_ = 0;
  double drive_y_ = 0;
  std::vector<Ring> drivable_;
  std::vector<Ring> obstacles_;
  Reason drivable_problem_;
  // Undocking: what for, where it started and the way it faces, and once it is stopping, whether that ends in
  // UNDOCKING_FAILED.
  enum class UndockStep { kChargeOff, kDrive };
  DockCheck undock_for_ = DockCheck::kNone;
  UndockStep undock_step_ = UndockStep::kChargeOff;
  double undock_x_ = 0;
  double undock_y_ = 0;
  double undock_heading_ = 0;
  std::optional<Point2> undock_fix_start_;
  bool undock_stopping_ = false;
  std::optional<Reason> undock_failure_;
  Reason undock_stop_why_;
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
  // The check of the charge on the contacts and whether it ran in this try; the way out kept until charging is off;
  // whether this docking switched charging off (stopped once the driver confirmed it) and has to allow it again.
  std::optional<ChargeCheck> docking_charge_;
  bool docking_checked_ = false;
  std::optional<DockingLeg> docking_out_after_charge_off_;
  bool docking_charge_touched_ = false;
  bool docking_charge_stopped_ = false;
  // Why the planner gave no way for the try that drives straight, and the turning circle of the body for its check.
  std::string docking_direct_cause_;
  double turning_radius_ = 0;
  // Whether charging is confirmed on the station, and its loss.
  ChargeWatch charge_watch_;
  std::string session_id_;
  int pause_ = 0;
  Reason pause_reason_;
  Leg leg_ = Leg::kNone;
  bool approached_ = false;
  // Questions of the job in a row the planner gave no answer for.
  int planner_silent_ = 0;
  bool skip_path_ = false;
  bool skip_area_ = false;
  // Where the job stood when a skip left a path that was not the last of its plan; index follows the mower along it.
  std::optional<JobProgress> skipped_;
  // The planner found no way from there to the next path: a piece of the path left is the next step.
  bool drive_on_ = false;
  std::string announced_area_;
  std::optional<double> spinup_since_;
  std::optional<double> gps_good_since_;
  // The rules of going home while mowing or calibrating.
  HomeWatch home_;
  // The run of the schedule from StartRun until the mower is idle again: its request, phase, the end of the
  // announcement and of the plan (monotonic s), whether that end came, and how it ends.
  struct Run {
    ScheduleRun request;
    RunPhase phase = RunPhase::kNone;
    double announce_until = 0;
    std::optional<double> end_due;
    bool end_passed = false;
    std::string end_code;
    std::string detail;
  };
  std::optional<Run> run_;
  // Why the next skip of an area is made: MANUAL by the app, SCHEDULE for a pause of the schedule.
  Reason skip_why_;
  // The sunset: the switch, the clock and the datum it needs, and the deadline of this run (monotonic s), due once
  // more after a start, a continue, a clock or a datum. sunset_told_: the first try of the run could not reckon it.
  bool sunset_on_ = false;
  bool clock_safe_ = false;
  std::string clock_reason_ = "no_report";
  double clock_offset_ = 0;
  std::optional<std::pair<double, double>> datum_;
  std::optional<double> sunset_due_;
  bool sunset_try_ = false;
  bool sunset_tried_ = false;
  bool sunset_told_ = false;
  double last_checkpoint_s_ = 0;
  bool blades_running_ = false;
  // The reason of StartProblem() at the last actions, empty while a start would go.
  std::string last_start_reason_ = "-";
  // Calibration: the program's status and when it came, its last sign and the phase it reported last.
  std::optional<CalibrationStatus> calibration_status_;
  double calibration_status_s_ = 0;
  double calibration_sign_s_ = 0;
  std::string calibration_phase_;
  int calibration_index_ = 0;
  int64_t hold_until_ = 0;
  // The client of the command being run, for its events; the one of a skip that waits for the leg to stop.
  std::string client_;
  std::string skip_client_;
  // emergency.reset or STOP asked the safety node: the end of the emergency carries the source (rpc with its client,
  // button), until a refusal, a new emergency or kReleaseSourceS after the answer; the latest request names it.
  bool release_pending_ = false;
  std::string release_source_;
  std::string release_client_;
  std::optional<double> release_answered_s_;
};

template <typename T>
bool LogicCore::Pending() const {
  return std::any_of(effects_.begin(), effects_.end(),
                     [](const Effect& effect) { return std::holds_alternative<T>(effect); });
}

template <typename T>
void LogicCore::SetDue(bool due) {
  if (!due) {
    Drop<T>();
  } else if (!Pending<T>()) {
    effects_.push_back(T{});
  }
}

template <typename T>
void LogicCore::Replace(T value) {
  Drop<T>();
  effects_.push_back(std::move(value));
}

template <typename T>
void LogicCore::Drop() {
  effects_.erase(std::remove_if(effects_.begin(), effects_.end(),
                                [](const Effect& effect) { return std::holds_alternative<T>(effect); }),
                 effects_.end());
}

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_CORE_H
