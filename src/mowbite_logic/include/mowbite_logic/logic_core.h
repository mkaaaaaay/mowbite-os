#ifndef MOWBITE_LOGIC_LOGIC_CORE_H
#define MOWBITE_LOGIC_LOGIC_CORE_H

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/mowing_job.h"
#include "mowbite_logic/reasons.h"

namespace mowbite_logic {

// What the logic needs from hw/status.
struct HardwareSnapshot {
  bool telemetry_ok = false;
  // Blocked by the driver; the reason bits come from it, numbered like the EMERGENCY event.
  bool emergency = false;
  int emergency_reason = 0;
  bool rain = false;
  bool dock_contact = false;
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

struct LogicConfig {
  // 6S Li-ion; empty is an assumption until the discharge curve of the Tron is measured.
  double battery_empty_voltage = 19.2;
  double battery_full_voltage = 25.2;
  // The MCU reports no charge voltage, dock contact stands in for it.
  double dock_charge_voltage = 28.0;
  // GPS counts as good up to this accuracy (m), like max_position_accuracy of OpenMower. The localization reads the
  // same parameter (tron.yaml, /**).
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
  // s with a good position before mowing goes on after the GPS got bad.
  double gps_wait_time = 10.0;
  // Mowing ends below this voltage (V), held for battery_critical_time (s). Assumption until the discharge curve is
  // measured: 3.5 V per cell.
  double battery_critical_voltage = 21.0;
  double battery_critical_time = 10.0;
  // m: farther from the start of a path the mower first drives there over the drivable areas.
  double approach_distance = 0.5;
  // s between checkpoints while a path is mowed.
  double checkpoint_period = 30.0;
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

constexpr uint8_t kSensorTypeString = 1;
constexpr uint8_t kSensorTypeDouble = 2;

struct SensorDefinition {
  std::string id;
  std::string name;
  std::string unit;
  uint8_t value_type = kSensorTypeDouble;
  uint8_t value_description = 0;
  bool has_min_max = false;
  double min_value = 0;
  double max_value = 0;
  bool has_critical_low = false;
  double lower_critical_value = 0;
  bool has_critical_high = false;
  double upper_critical_value = 0;
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

// Mower states and actions: IDLE, AREA_RECORDING (remote control and recording of areas), driving to a pose, MOWING
// and PAUSED, emergency and sensors.
class LogicCore {
 public:
  explicit LogicCore(const LogicConfig& config);

  void OnHardwareStatus(const HardwareSnapshot& hw);
  // hw/status stopped coming.
  void OnHardwareTimeout();
  void OnPose(const PoseSnapshot& pose);
  // localization/pose stopped coming; events keep the last position.
  void OnPoseTimeout();
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

  // hw/capabilities: whether the platform has a safety measurement. Without one only remote control, no driving.
  void OnCapabilities(bool safety_measured);
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
  // emergency; teleop only while recording. So hw/cmd_vel has one source at a time.
  bool FollowerActive() const;
  // Once when a drive or a leg of the job has to stop at once (emergency, no pose, no safety measurement, pause, skip,
  // abort): the node cancels it, and a drive ends with this reason.
  std::optional<Reason> TakeDriveCancel();

  // The mow areas of the map and why mowing may not plan from it (empty if it may).
  void OnMowMap(std::vector<JobArea> areas, const Reason& problem);
  // The checkpoint read at the start.
  void RestoreJob(const JobProgress& progress);
  // Why mowing cannot start now, empty if it can.
  Reason StartProblem() const;
  // A start refused by the node (planner or follower missing): event MOWING_REFUSED.
  void RefuseStart(const Reason& why);
  bool Mowing() const;
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
  // hw/blade refused while the blade was wanted: mowing pauses.
  void OnBladeRefused(const std::string& message);
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
  enum class State { kIdle, kAreaRecording, kDriving, kMowing };
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
  // POSITION_ACCURACY or NO_HEADING.
  Reason Busy() const;
  Reason Unsafe() const;
  Reason AccuracyReason(float accuracy) const;
  Reason PositionProblem() const;
  ActionResult OnMowingAction(const std::string& name);
  void StartMowing();
  void Pause(int why, const Reason& reason);
  // Stops a drive or a leg on its way; a drive ends with the first reason given.
  void CancelDrive(const Reason& reason);
  void Unpause(int why);
  void ApplySkips();
  // Ends the job: finished (JOB_COMPLETE) or not (it goes on from the checkpoint next time).
  void EndJob(bool finished);
  bool PoseGood() const;
  void CheckpointNow();

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
  bool drive_cancel_ = false;
  std::optional<Reason> cancel_reason_;
  double last_teleop_s_ = -std::numeric_limits<double>::infinity();
  std::vector<Event> events_;
  AreaRecorder recorder_;
  bool map_writable_ = false;
  Reason map_reason_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");

  MowingJob job_;
  Reason mow_map_problem_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");
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
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_CORE_H
