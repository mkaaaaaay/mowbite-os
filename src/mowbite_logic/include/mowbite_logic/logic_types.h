#ifndef MOWBITE_LOGIC_LOGIC_TYPES_H
#define MOWBITE_LOGIC_LOGIC_TYPES_H

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/docking.h"
#include "mowbite_logic/home_watch.h"
#include "mowbite_logic/marker_dock.h"
#include "mowbite_logic/mowing_job.h"
#include "mowbite_logic/obstacle_part.h"
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
  // Empty is 0 % for the app and where mowing goes home (the highest voltage of battery_low_window s below it). The
  // values here are those of the tests (6S Li-ion), each platform sets its own.
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
  // The pose from the dock marker may be worse by this per m from the docked place (mowbite_safety, same value).
  double dock_accuracy_per_m = 0.09;
  double dock_reference_limit = 0.10;
  // After the reference check the mower leaves the station before mowing or a drive starts, straight against the way
  // in: forwards with the contacts behind base_link, else backwards (PlanLeave).
  UndockConfig undock;
  // After JOB_COMPLETE, abort_mowing and on docking.start the mower drives onto the station contacts first; a try that
  // fails also drives out like undocking.
  DockConfig dock;
  // Docking by the dock marker in the station frame, where the platform has one: docking.test, and with its guide
  // docking.start and going home.
  MarkerConfig marker;
  // DOCKED only with the charge confirmed on the contacts, CHARGING_LOST in IDLE on the station (G19).
  ChargeConfig charge;
  CalibrationConfig calibration;
  // The hold for obstacles (obstacle_mode stop).
  ObstacleConfig obstacle;
  // Seed of the job, session and event ids; the node draws it, the journal keeps it.
  uint64_t id_seed = 0;
};

// Why the battery voltages of the config do not fit (critical < empty < full), empty when they do.
std::string BatteryVoltagesProblem(const LogicConfig& config);
// Why a time, speed or clearance of the config is out of its range, empty when all fit. The clearances of driving
// around objects keep at least G46: 0.30 m and 2 sigma on the way around, 0.10 m beside a lane, 0.3 m/s.
std::string ConfigRangeProblem(const LogicConfig& config);

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
  // In PAUSED: the code of the reason that holds the mower (PauseReason) and its kind with OBSTACLE; else empty.
  std::string pause_reason;
  std::string pause_kind;
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

// How the latest release of the emergency ended: released, or refused with the reason and the id of its event; seq
// counts the outcomes since the start.
struct ReleaseOutcome {
  uint64_t seq = 0;
  bool released = false;
  Reason why;
  std::string event_id;
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
                            Notice, Checkpoint, RunChanged, ObstacleNote>;

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
  enum class Kind { kNone, kPlan, kApproach, kFollow, kSkippedPath, kCut, kDetour };
  Kind kind = Kind::kNone;
  // kPlan: the area to plan. kCut, kDetour: the area of the job, the only one they drive in.
  std::string area_id;
  // kApproach: where to drive to, over the drivable areas. kDetour: the same within the area, around the circles,
  // the blade on.
  JobPose goal;
  // kCut: the paths of the plan from the current one on, cut from start_index of the first, by the circles.
  std::vector<JobPath> paths;
  std::vector<AvoidCircle> circles;
  // kFollow: the path and the pose to start at. kSkippedPath: the same for a piece of the path a skip left; the path
  // ends with the piece and is a transit all along.
  JobPath path;
  size_t start_index = 0;
};

// s after the answer of the safety node within which the end of the emergency counts as released by the command.
constexpr double kReleaseSourceS = 2.0;

// A job asks the planner again this often in a row when it gave no answer; then mowing ends and the mower goes home.
constexpr int kPlannerRetries = 3;
// Pauses for GPS in a row that ended on their own without the job getting on: the next one waits for the user.
constexpr int kGpsResumes = 3;

// A cut of the plan (cut_plan) that found no way in the planner's time is asked for once more.
constexpr int kCutRetries = 1;

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

// What a mowing.start asks for: the areas in order (none for all) and whether to begin anew; a start of the schedule
// also the areas it leaves out.
struct StartRequest {
  std::vector<std::string> tasks;
  bool restart = false;
  std::vector<std::string> pass_over;
  // Taken by a job that begins with this start, kept by one it goes on with.
  std::optional<double> angle_offset_deg;
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

// The state of the logic; PAUSED is MOWING with a pause.
enum class State { kIdle, kAreaRecording, kDriving, kMowing, kUndocking, kDocking, kCalibrating, kSetup };
// Why mowing pauses; manual, emergency and fault wait for continue, GPS and an obstacle go on by themselves.
constexpr int kPauseManual = 1 << 0;
constexpr int kPauseEmergency = 1 << 1;
constexpr int kPauseGps = 1 << 2;
constexpr int kPauseFault = 1 << 3;
constexpr int kPauseObstacle = 1 << 4;
constexpr int kPauseWaitsForUser = kPauseManual | kPauseEmergency | kPauseFault;
// Pauses that never go home: the user or a person or animal may stand at the mower (G11, G45).
constexpr int kPauseKeepsPlace = kPauseWaitsForUser | kPauseObstacle;
enum class Leg { kNone, kPlanning, kApproach, kFollow, kSkippedPath, kCutting, kDetour };

// A stretch of a path of the plan the mower mows while it keeps out of the circles: poses first to last of path.
struct JobPiece {
  int path = 0;
  size_t first = 0;
  size_t last = 0;
};

// The answer of the planner's cut_plan: the pieces to mow, in order; ok false with code where it gave none.
struct CutAnswer {
  bool ok = false;
  std::string code;
  std::string message;
  std::vector<JobPiece> pieces;
};
// What the reference check on the station is for.
enum class DockCheck { kNone, kMowing, kDrive };

// What the node has not taken yet, oldest first; flags (StopRequest, ChargeOff, ...) at most once.
class EffectQueue {
 public:
  void push_back(Effect effect) {
    items_.push_back(std::move(effect));
  }
  std::vector<Effect>::iterator begin() {
    return items_.begin();
  }
  std::vector<Effect>::iterator end() {
    return items_.end();
  }
  std::vector<Effect>::const_iterator begin() const {
    return items_.begin();
  }
  std::vector<Effect>::const_iterator end() const {
    return items_.end();
  }
  // All of them, oldest first; the queue is empty after it.
  std::vector<Effect> Take() {
    std::vector<Effect> out;
    out.swap(items_);
    return out;
  }
  // The effects of one type: whether one waits; a flag added once or every one removed; the waiting one replaced;
  // every one removed.
  template <typename T>
  bool Pending() const;
  template <typename T>
  void SetDue(bool due);
  template <typename T>
  void Replace(T value);
  template <typename T>
  void Drop();

 private:
  std::vector<Effect> items_;
};

template <typename T>
bool EffectQueue::Pending() const {
  return std::any_of(items_.begin(), items_.end(),
                     [](const Effect& effect) { return std::holds_alternative<T>(effect); });
}

template <typename T>
void EffectQueue::SetDue(bool due) {
  if (!due) {
    Drop<T>();
  } else if (!Pending<T>()) {
    items_.push_back(T{});
  }
}

template <typename T>
void EffectQueue::Replace(T value) {
  Drop<T>();
  items_.push_back(std::move(value));
}

template <typename T>
void EffectQueue::Drop() {
  items_.erase(std::remove_if(items_.begin(), items_.end(),
                              [](const Effect& effect) { return std::holds_alternative<T>(effect); }),
               items_.end());
}

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_TYPES_H
