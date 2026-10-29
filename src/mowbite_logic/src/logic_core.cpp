#include "mowbite_logic/logic_core.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

namespace mowbite_logic {

namespace {

const std::string kRootPrefix = "mower_logic";
const std::string kIdlePrefix = "mower_logic:idle";
const std::string kRecordingPrefix = "mower_logic:area_recording";
const std::string kMowingPrefix = "mower_logic:mowing";

// 32 letters and digits, random: the ids of jobs and sessions.
std::string NewId() {
  static const char kAlphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<size_t> pick(0, sizeof(kAlphabet) - 2);
  std::string id(32, '0');
  for (auto& c : id) c = kAlphabet[pick(rng)];
  return id;
}

// EmergencyReason of OpenMower, one name per bit.
const char* const kReasonNames[] = {"LATCH",
                                    "TIMEOUT_INPUTS",
                                    "STOP",
                                    "LIFT",
                                    "LIFT_MULTIPLE",
                                    "COLLISION",
                                    "TIMEOUT_HIGH_LEVEL",
                                    "HIGH_LEVEL",
                                    "SERVICE_NOT_READY",
                                    "COLLISION_MULTIPLE",
                                    "MOWER_RPM_TIMEOUT",
                                    "MOWER_RPM_LIMIT",
                                    "FIRMWARE_INCOMPATIBLE"};

// SensorInfo value descriptions
constexpr uint8_t kTemperature = 1;
constexpr uint8_t kVoltage = 4;
constexpr uint8_t kCurrent = 5;
constexpr uint8_t kDistance = 7;
constexpr uint8_t kRpm = 8;

SensorDefinition Range(std::string id, std::string name, std::string unit, uint8_t description, double min,
                       double max) {
  SensorDefinition s;
  s.id = std::move(id);
  s.name = std::move(name);
  s.unit = std::move(unit);
  s.value_description = description;
  s.has_min_max = true;
  s.min_value = min;
  s.max_value = max;
  return s;
}

SensorDefinition Temperature(std::string id, std::string name, double min, double max, double critical_high) {
  SensorDefinition s = Range(std::move(id), std::move(name), "deg.C", kTemperature, min, max);
  s.has_critical_high = true;
  s.upper_critical_value = critical_high;
  return s;
}

}  // namespace

std::string EmergencyReasonNames(int reason) {
  std::string names;
  for (size_t bit = 0; bit < sizeof(kReasonNames) / sizeof(kReasonNames[0]); bit++) {
    if ((reason & (1 << bit)) == 0) continue;
    if (!names.empty()) names += ", ";
    names += kReasonNames[bit];
  }
  return names;
}

namespace {

RecorderConfig RecorderConfigOf(const LogicConfig& config) {
  RecorderConfig recorder = config.recorder;
  recorder.max_position_accuracy = config.max_position_accuracy;
  return recorder;
}

}  // namespace

LogicCore::LogicCore(const LogicConfig& config) : config_(config), recorder_(RecorderConfigOf(config)) {
  // Without telemetry the mower counts as stopped until the driver reports.
  emergency_ = true;
}

void LogicCore::OnHardwareStatus(const HardwareSnapshot& hw) {
  hw_ = hw;
  UpdateEmergency();
  // From the rpm the MCU reports, not from what was asked for.
  const bool running = hw.telemetry_ok && hw.blade.ok && std::abs(hw.blade.rpm) >= config_.blade_running_rpm;
  if (running != blades_running_) {
    blades_running_ = running;
    AddEvent("BLADES", {{"enabled", running}});
  }
}

void LogicCore::OnHardwareTimeout() {
  hw_.telemetry_ok = false;
  hw_.battery_ok = false;
  UpdateEmergency();
}

void LogicCore::OnPose(const PoseSnapshot& pose) {
  pose_ = pose;
  last_x_ = pose.x;
  last_y_ = pose.y;
  if (state_ != State::kAreaRecording) return;
  recorder_.OnPose(pose.x, pose.y, pose.position_accuracy);
  PullRecorder();
}

void LogicCore::OnPoseTimeout() {
  pose_.reset();
  if (state_ == State::kDriving) CancelDrive(Reason("POSE_LOST", "No position from the localization"));
  if (state_ == State::kMowing) Pause(kPauseGps, Reason("NO_POSITION", "No position from the localization"));
  recorder_.OnPoseLost();
  PullRecorder();
}

void LogicCore::OnMapStatus(MapDatum datum, const std::string& reason) {
  const bool writable = datum == MapDatum::kMatch || datum == MapDatum::kEmpty;
  if (writable != map_writable_) actions_changed_ = true;
  map_writable_ = writable;
  map_reason_ = writable ? Reason() : MapReason(datum, reason);
  recorder_.SetMapWritable(writable, map_reason_);
}

std::optional<MapEdit> LogicCore::TakeMapEdit() {
  return recorder_.TakeMapEdit();
}

void LogicCore::OnMapEditResult(bool ok, const std::string& id, const std::string& code,
                                const std::string& message) {
  recorder_.OnMapEditResult(ok, id, code, message);
  PullRecorder();
}

void LogicCore::PullRecorder() {
  for (auto& event : recorder_.TakeEvents()) AddEvent(event.type, std::move(event.details));
  actions_changed_ |= recorder_.TakeActionsChanged();
}

json LogicCore::MapOverlay() const {
  // The recorder is reset when the state changes, so outside of AREA_RECORDING it has no lines.
  return recorder_.Overlay();
}

bool LogicCore::TakeOverlayChanged() {
  return recorder_.TakeOverlayChanged();
}

std::vector<std::string> LogicCore::TakeNotices() {
  std::vector<std::string> out = recorder_.TakeNotices();
  out.insert(out.end(), notices_.begin(), notices_.end());
  notices_.clear();
  return out;
}

void LogicCore::OnPositionJump(double to_x, double to_y, double distance_m, bool after_outliers) {
  last_x_ = to_x;
  last_y_ = to_y;
  json details = json::object();
  PutReason(&details, "POSITION_JUMP",
            Reason(after_outliers ? "OUTLIERS" : "GAP",
                   after_outliers ? "Back on GPS after outliers" : "Back on GPS after a gap")
                .With("distance", distance_m));
  AddEvent("POSITION_JUMP", std::move(details));
}

// Every event carries the position where it happened.
void LogicCore::AddEvent(const std::string& type, json details) {
  details["x"] = last_x_;
  details["y"] = last_y_;
  if (!job_.progress().job_id.empty()) details["job_id"] = job_.progress().job_id;
  if (!session_id_.empty()) details["session_id"] = session_id_;
  events_.push_back({type, std::move(details)});
}

bool LogicCore::Emergency() const {
  return emergency_;
}

int LogicCore::EmergencyReason() const {
  return hw_.emergency_reason | (hw_.telemetry_ok ? 0 : kEmergencyTimeoutInputs);
}

void LogicCore::UpdateEmergency() {
  const bool emergency = hw_.emergency || !hw_.telemetry_ok;
  // The driver sends no reason without an emergency; one that does not block is not shown either.
  reason_ = emergency ? EmergencyReason() : 0;
  if (reason_ != logged_reason_) {
    reason_changes_.push_back({logged_reason_, reason_});
    logged_reason_ = reason_;
  }
  // The first report after the start only sets the state, it is no change worth an event.
  if (!hw_seen_) {
    hw_seen_ = true;
    actions_changed_ |= emergency != emergency_;
    emergency_ = emergency;
    return;
  }
  if (emergency != emergency_) {
    emergency_ = emergency;
    if (emergency) teleop_armed_ = false;
    const Reason why("EMERGENCY", "Emergency: " + EmergencyReasonNames(reason_));
    if (emergency && state_ == State::kDriving) CancelDrive(why);
    if (emergency && state_ == State::kMowing) Pause(kPauseEmergency, why);
    // reason stays the bits of OpenMower; resetting is up to the user.
    AddEvent("EMERGENCY", {{"emergency", emergency},
                           {"reason", reason_},
                           {"severity", emergency ? "error" : "info"},
                           {"resume", emergency ? "user" : "auto"}});
    actions_changed_ = true;
  }
}

ActionResult LogicCore::OnAction(const std::string& action_id) {
  if (action_id == kRootPrefix + "/reset_emergency") return ActionResult::kReleaseEmergency;
  if (action_id == kIdlePrefix + "/start_mowing") {
    const Reason why = StartProblem();
    if (!why.empty()) {
      RefuseStart(why);
      return ActionResult::kRejected;
    }
    StartMowing();
    return ActionResult::kDone;
  }
  if (action_id == kIdlePrefix + "/reset_job") {
    if (state_ != State::kIdle || !job_.Unfinished()) return ActionResult::kRejected;
    // The event carries the id of the dropped job, the checkpoint is saved without it.
    AddEvent("JOB_RESET", json::object());
    job_.Reset();
    CheckpointNow();
    actions_changed_ = true;
    return ActionResult::kDone;
  }
  if (action_id.rfind(kMowingPrefix + "/", 0) == 0) {
    if (state_ != State::kMowing) return ActionResult::kRejected;
    return OnMowingAction(action_id.substr(kMowingPrefix.size() + 1));
  }
  if (action_id == kIdlePrefix + "/start_area_recording") {
    if (state_ != State::kIdle || emergency_) return ActionResult::kRejected;
    if (!map_writable_) {
      json details = {{"action", "start_area_recording"}};
      PutReason(&details, "RECORDING_REFUSED", map_reason_);
      AddEvent("RECORDING_REFUSED", std::move(details));
      return ActionResult::kRejected;
    }
    SetState(State::kAreaRecording);
    return ActionResult::kDone;
  }
  if (action_id == kRecordingPrefix + "/exit_recording_mode") {
    if (state_ != State::kAreaRecording) return ActionResult::kRejected;
    SetState(State::kIdle);
    return ActionResult::kDone;
  }
  if (action_id.rfind(kRecordingPrefix + "/", 0) == 0 && state_ == State::kAreaRecording) {
    const AreaRecorder::Result result = recorder_.OnAction(action_id.substr(kRecordingPrefix.size() + 1));
    PullRecorder();
    if (result == AreaRecorder::Result::kDone) return ActionResult::kDone;
    if (result == AreaRecorder::Result::kRefused) return ActionResult::kRejected;
  }
  const bool known_prefix =
      action_id.rfind(kIdlePrefix + "/", 0) == 0 || action_id.rfind(kRecordingPrefix + "/", 0) == 0;
  return known_prefix ? ActionResult::kRejected : ActionResult::kUnknown;
}

std::optional<std::pair<double, double>> LogicCore::OnTeleop(double vx, double vz, double now_s) {
  const double quiet_s = now_s - last_teleop_s_;
  last_teleop_s_ = now_s;
  if (state_ != State::kAreaRecording || emergency_) {
    teleop_armed_ = false;
    return std::nullopt;
  }
  if (!teleop_armed_) {
    const bool zero = vx == 0.0 && vz == 0.0;
    if (!zero && quiet_s < kTeleopQuietS) return std::nullopt;
    teleop_armed_ = true;
  }
  return std::make_pair(vx, vz);
}

void LogicCore::OnCapabilities(bool safety_measured) {
  safety_measured_ = safety_measured;
  if (safety_measured) return;
  if (state_ == State::kDriving) {
    CancelDrive(Reason("NOT_ALLOWED", "No safety measurement of this platform (hw/capabilities)"));
  }
  Pause(kPauseFault, Reason("NO_SAFETY_MEASUREMENT", "No safety measurement of this platform (hw/capabilities)"));
}

Reason LogicCore::DriveProblem() const {
  Reason why = Busy();
  if (why.empty()) why = Unsafe();
  if (why.empty()) why = PositionProblem();
  return why;
}

Reason LogicCore::StartDrive(double x, double y) {
  const Reason why = DriveProblem();
  if (!why.empty()) {
    RefuseDrive(why);
    return why;
  }
  drive_cancel_ = false;
  cancel_reason_.reset();
  SetState(State::kDriving);
  AddEvent("DRIVE", {{"phase", "start"}, {"goal_x", x}, {"goal_y", y}});
  return {};
}

void LogicCore::RefuseDrive(const Reason& why) {
  json details = json::object();
  PutReason(&details, "DRIVE_REFUSED", why);
  AddEvent("DRIVE_REFUSED", std::move(details));
}

void LogicCore::OnDriveEnded(bool arrived, const Reason& reason) {
  if (state_ != State::kDriving) return;
  json details = {{"phase", "end"}, {"arrived", arrived}};
  PutReason(&details, "DRIVE", reason);
  AddEvent("DRIVE", std::move(details));
  drive_cancel_ = false;
  cancel_reason_.reset();
  SetState(State::kIdle);
}

bool LogicCore::Driving() const {
  return state_ == State::kDriving;
}

bool LogicCore::FollowerActive() const {
  if (emergency_) return false;
  if (state_ == State::kDriving) return true;
  return state_ == State::kMowing && pause_ == 0 && (leg_ == Leg::kApproach || leg_ == Leg::kFollow);
}

std::optional<Reason> LogicCore::TakeDriveCancel() {
  if (!drive_cancel_) return std::nullopt;
  drive_cancel_ = false;
  Reason reason = cancel_reason_.value_or(Reason("STOPPED", "Stopped by the logic"));
  cancel_reason_.reset();
  return reason;
}

void LogicCore::CancelDrive(const Reason& reason) {
  drive_cancel_ = true;
  if (!cancel_reason_) cancel_reason_ = reason;
}

void LogicCore::SetState(State state) {
  if (state == state_) return;
  if (state_ == State::kAreaRecording || state_ == State::kDriving || state_ == State::kMowing) stop_requested_ = true;
  // A session is one run of mowing; the job lasts until it is finished or reset.
  if (state == State::kMowing) {
    if (job_.progress().job_id.empty()) job_.SetJobId(NewId());
    session_id_ = NewId();
  }
  if (state_ == State::kMowing) {
    session_id_.clear();
    if (leg_ == Leg::kApproach || leg_ == Leg::kFollow) CancelDrive(Reason("STOPPED", "Mowing ended"));
    leg_ = Leg::kNone;
    pause_ = 0;
    pause_reason_ = Reason();
    skip_path_ = skip_area_ = false;
    spinup_since_.reset();
    gps_good_since_.reset();
    battery_low_since_.reset();
  }
  // Entering and leaving both start from nothing; what was not saved is gone, like with OpenMower.
  recorder_.Reset();
  recorder_.TakeActionsChanged();
  state_ = state;
  teleop_armed_ = false;
  json details = {{"state", CurrentStateName()}};
  if (state == State::kDriving) details["sub_state"] = "DRIVING";
  AddEvent("STATE", std::move(details));
  actions_changed_ = true;
}

std::string LogicCore::CurrentStateName() const {
  if (state_ == State::kMowing && pause_ != 0) return "PAUSED";
  return StateName(state_);
}

const char* LogicCore::StateName(State state) {
  switch (state) {
    case State::kAreaRecording: return "AREA_RECORDING";
    case State::kMowing: return "MOWING";
    // OpenMower has no state for driving without mowing; the sub state tells.
    case State::kDriving:
    case State::kIdle:
    default: return "IDLE";
  }
}

RobotStateData LogicCore::RobotState() const {
  RobotStateData s;
  s.state = CurrentStateName();
  if (state_ == State::kMowing) {
    s.current_area = job_.progress().area;
    s.current_path = job_.progress().path;
    s.current_path_index = job_.progress().index;
  }
  if (state_ == State::kAreaRecording) s.sub_state = recorder_.SubState();
  if (state_ == State::kDriving) s.sub_state = "DRIVING";
  s.emergency = emergency_;
  s.emergency_reason = reason_;
  s.is_charging = hw_.battery_ok && hw_.dock_contact;
  s.rain_detected = hw_.telemetry_ok && hw_.rain;
  if (hw_.battery_ok) {
    s.battery_percentage = std::clamp(hw_.battery_percentage / 100.0f, 0.0f, 1.0f);
  }
  if (pose_) {
    s.pose_x = pose_->x;
    s.pose_y = pose_->y;
    s.pose_heading = pose_->heading;
    s.pos_accuracy = pose_->position_accuracy;
    s.heading_accuracy = pose_->orientation_accuracy;
    s.heading_valid = pose_->orientation_valid;
    if (pose_->position_accuracy <= config_.max_position_accuracy && pose_->orientation_valid) {
      s.gps_percentage = static_cast<float>(
          1.0 - std::min(1.0, pose_->position_accuracy / config_.max_position_accuracy));
    }
  }
  return s;
}

std::vector<ActionGroup> LogicCore::Actions() const {
  const bool idle = state_ == State::kIdle;
  const bool recording = state_ == State::kAreaRecording;
  const bool mowing = state_ == State::kMowing;
  std::vector<ActionGroup> groups;
  groups.push_back({kRootPrefix, {{"reset_emergency", "Reset Emergency", true}}});
  groups.push_back({kIdlePrefix,
                    {{"start_mowing", "Start Mowing", StartProblem().empty()},
                     {"start_area_recording", "Start Area Recording", idle && !emergency_ && map_writable_},
                     {"reset_job", "Reset Job", idle && job_.Unfinished()}}});
  // Pause and continue take turns, like in OpenMower; continue waits for the emergency to be reset.
  const bool waits = (pause_ & (kPauseManual | kPauseEmergency | kPauseFault)) != 0;
  groups.push_back({kMowingPrefix,
                    {{"pause", "Pause Mowing", mowing && !waits},
                     {"continue", "Continue Mowing", mowing && waits && !emergency_},
                     {"abort_mowing", "Stop Mowing", mowing},
                     {"skip_area", "Skip Area", mowing},
                     {"skip_path", "Skip Path", mowing}}});
  // The recorder decides while recording; outside of it all are off.
  const std::vector<bool> enabled = recorder_.Enabled();
  const std::vector<std::string> names = {"Start Recording",
                                          "Stop Recording",
                                          "Save Navigation Area",
                                          "Save Mowing Area",
                                          "Exit",
                                          "Discard Area",
                                          "Record Docking point",
                                          "Enable automatic point collecting",
                                          "Disable automatic point collecting",
                                          "Collect point",
                                          "Start manual mowing",
                                          "Stop manual mowing"};
  ActionGroup recording_group{kRecordingPrefix, {}};
  for (size_t i = 0; i < names.size(); i++) {
    recording_group.actions.push_back({AreaRecorder::ActionNames()[i], names[i], recording && enabled[i]});
  }
  groups.push_back(std::move(recording_group));
  return groups;
}

std::vector<SensorDefinition> LogicCore::Sensors() const {
  std::vector<SensorDefinition> sensors;
  SensorDefinition battery =
      Range("om_v_battery", "V Battery", "V", kVoltage, config_.battery_empty_voltage, config_.battery_full_voltage);
  battery.has_critical_low = true;
  battery.lower_critical_value = config_.battery_empty_voltage;
  sensors.push_back(battery);
  // The MCU reports neither a charge voltage nor a charger state: both are derived, the names say so.
  sensors.push_back(Range("om_v_charge", "V Charge (dock contact)", "V", kVoltage, 0, config_.dock_charge_voltage + 2));
  sensors.push_back(Range("om_charge_current", "Charge Current", "A", kCurrent, 0, 7));
  SensorDefinition charge_state;
  charge_state.id = "om_charge_state";
  charge_state.name = "Charge State (derived)";
  charge_state.value_type = kSensorTypeString;
  sensors.push_back(charge_state);
  sensors.push_back(Range("om_mow_motor_rpm", "Mow Motor Revolutions", "rpm", kRpm, 0, 3500));
  sensors.push_back(Range("om_mow_motor_current", "Mow Motor Current", "A", kCurrent, 0, 5));
  // One temperature per motor board, so no separate mow ESC temperature. Own limits: the drive motors hold position
  // up to 68 degC, warn above 80.
  SensorDefinition gps_accuracy;
  gps_accuracy.id = "om_gps_accuracy";
  gps_accuracy.name = "GPS Accuracy";
  gps_accuracy.unit = "m";
  gps_accuracy.value_description = kDistance;
  sensors.push_back(gps_accuracy);
  sensors.push_back(Temperature("om_mow_motor_temp", "Mow Motor Temp", 0, 100, 80));
  sensors.push_back(Temperature("om_left_esc_temp", "Left ESC Temp", 0, 100, 80));
  sensors.push_back(Temperature("om_right_esc_temp", "Right ESC Temp", 0, 100, 80));
  // From the BMS. Own limits for Li-ion: warn above 55 degC.
  sensors.push_back(Temperature("battery_temp", "Battery Temp", -10, 70, 55));
  return sensors;
}

std::vector<SensorReading> LogicCore::Readings() const {
  std::vector<SensorReading> readings;
  if (hw_.battery_ok) {
    const double charge_current = hw_.battery_current < 0 ? -hw_.battery_current : 0.0;
    readings.push_back({"om_v_battery", hw_.battery_voltage, ""});
    readings.push_back({"om_v_charge", hw_.dock_contact ? config_.dock_charge_voltage : 0.0, ""});
    readings.push_back({"om_charge_current", charge_current, ""});
    // Texts of the charger of OpenMower v2, the app's battery script reads them.
    std::string state = "Not Charging";
    if (hw_.dock_contact && charge_current > 0.1) {
      state = "Fast Charge";
    } else if (hw_.dock_contact && hw_.battery_percentage >= 99) {
      state = "Done";
    }
    readings.push_back({"om_charge_state", 0, state});
    readings.push_back({"battery_temp", static_cast<double>(hw_.battery_temperature), ""});
  }
  if (pose_) readings.push_back({"om_gps_accuracy", pose_->position_accuracy, ""});
  if (hw_.blade.ok) {
    readings.push_back({"om_mow_motor_rpm", static_cast<double>(hw_.blade.rpm), ""});
    readings.push_back({"om_mow_motor_current", hw_.blade.current, ""});
    readings.push_back({"om_mow_motor_temp", static_cast<double>(hw_.blade.temperature), ""});
  }
  if (hw_.left.ok) readings.push_back({"om_left_esc_temp", static_cast<double>(hw_.left.temperature), ""});
  if (hw_.right.ok) readings.push_back({"om_right_esc_temp", static_cast<double>(hw_.right.temperature), ""});
  return readings;
}

std::vector<Event> LogicCore::TakeEvents() {
  std::vector<Event> out;
  out.swap(events_);
  return out;
}

bool LogicCore::TakeActionsChanged() {
  bool changed = actions_changed_;
  actions_changed_ = false;
  // Start Mowing follows the position, the battery and more, which change without an event.
  const bool start_ok = StartProblem().empty();
  if (start_ok != last_start_ok_) {
    last_start_ok_ = start_ok;
    changed = true;
  }
  return changed;
}

std::vector<ReasonChange> LogicCore::TakeReasonChanges() {
  std::vector<ReasonChange> out;
  out.swap(reason_changes_);
  return out;
}

bool LogicCore::TakeStopRequest() {
  bool stop = stop_requested_;
  stop_requested_ = false;
  return stop;
}

void LogicCore::OnMowMap(std::vector<JobArea> areas, const Reason& problem) {
  job_.SetAreas(std::move(areas));
  mow_map_problem_ = problem;
  if (state_ == State::kMowing && !problem.empty()) Pause(kPauseFault, problem);
}

void LogicCore::RestoreJob(const JobProgress& progress) {
  job_.Restore(progress);
  actions_changed_ = true;
}

bool LogicCore::PoseGood() const {
  return pose_ && pose_->position_accuracy <= config_.max_position_accuracy && pose_->orientation_valid;
}

Reason LogicCore::Busy() const {
  if (state_ == State::kIdle) return {};
  const std::string state = state_ == State::kDriving ? "driving" : CurrentStateName();
  return Reason("BUSY", "The mower is busy: " + state).With("state", state);
}

Reason LogicCore::Unsafe() const {
  if (emergency_) return Reason("EMERGENCY", "Emergency: reset it first");
  if (!safety_measured_) {
    return Reason("NO_SAFETY_MEASUREMENT",
                  "No safety measurement of this platform (hw/capabilities): mowing and driving are blocked");
  }
  return {};
}

Reason LogicCore::AccuracyReason(float accuracy) const {
  char text[160];
  std::snprintf(text, sizeof(text), "Position accuracy %.3f m is worse than %.3f m", accuracy,
                config_.max_position_accuracy);
  return Reason("POSITION_ACCURACY", text).With("accuracy", accuracy).With("limit", config_.max_position_accuracy);
}

Reason LogicCore::PositionProblem() const {
  if (!pose_) return Reason("NO_POSITION", "No position from the localization");
  if (!(pose_->position_accuracy <= config_.max_position_accuracy)) {
    return AccuracyReason(pose_->position_accuracy);
  }
  if (!pose_->orientation_valid) {
    return Reason("NO_HEADING", "The heading is not aligned yet");
  }
  return {};
}

Reason LogicCore::StartProblem() const {
  Reason why = Busy();
  if (why.empty()) why = Unsafe();
  if (!why.empty()) return why;
  // Undocking comes with docking (stage 4).
  if (hw_.dock_contact) {
    return Reason("ON_DOCK", "The mower is on the docking station: drive it off by remote control first");
  }
  if (hw_.rain) return Reason("RAIN", "Rain detected");
  if (hw_.battery_ok && hw_.battery_voltage < config_.battery_critical_voltage) {
    char text[160];
    std::snprintf(text, sizeof(text), "Battery %.1f V is below %.1f V", hw_.battery_voltage,
                  config_.battery_critical_voltage);
    return Reason("BATTERY_LOW", text)
        .With("battery_voltage", hw_.battery_voltage)
        .With("limit", config_.battery_critical_voltage);
  }
  why = PositionProblem();
  if (!why.empty()) return why;
  if (!mow_map_problem_.empty()) return mow_map_problem_;
  if (!job_.HasMowableArea()) return Reason("NO_MOWING_AREA", "The map has no active mowing area");
  return {};
}

void LogicCore::RefuseStart(const Reason& why) {
  json details = json::object();
  PutReason(&details, "MOWING_REFUSED", why);
  AddEvent("MOWING_REFUSED", std::move(details));
}

bool LogicCore::Mowing() const {
  return state_ == State::kMowing;
}

void LogicCore::StartMowing() {
  // Planned anew on every start, the map may have changed; the fingerprint keeps the place if the plan is the same.
  job_.Restore(job_.progress());
  leg_ = Leg::kNone;
  approached_ = false;
  announced_area_.clear();
  SetState(State::kMowing);
  CheckpointNow();
}

ActionResult LogicCore::OnMowingAction(const std::string& name) {
  if (name == "pause") {
    if (pause_ & kPauseManual) return ActionResult::kRejected;
    Pause(kPauseManual, Reason("MANUAL", "Paused by the app"));
    return ActionResult::kDone;
  }
  if (name == "continue") {
    const int waiting = kPauseManual | kPauseEmergency | kPauseFault;
    if (emergency_ || (pause_ & waiting) == 0) return ActionResult::kRejected;
    // Without GPS it still waits for a good position.
    Unpause(waiting);
    return ActionResult::kDone;
  }
  if (name == "abort_mowing") {
    EndJob(false);
    return ActionResult::kDone;
  }
  if (name == "skip_area" || name == "skip_path") {
    (name == "skip_area" ? skip_area_ : skip_path_) = true;
    // A leg on its way ends first, its index counts.
    if (leg_ == Leg::kApproach || leg_ == Leg::kFollow) {
      CancelDrive(Reason("STOPPED", "Skipped by the app"));
      stop_requested_ = true;
    } else {
      ApplySkips();
    }
    return ActionResult::kDone;
  }
  return ActionResult::kUnknown;
}

void LogicCore::ApplySkips() {
  if (skip_area_) {
    if (const JobArea* area = job_.CurrentArea()) {
      json details = {{"area_id", area->id}, {"area_name", area->name}};
      PutReason(&details, "AREA_SKIPPED", Reason("MANUAL", "Skipped by the app"));
      AddEvent("AREA_SKIPPED", std::move(details));
    }
    job_.SkipArea();
    // A plan on its way is for the skipped area.
    if (leg_ == Leg::kPlanning) leg_ = Leg::kNone;
  } else if (skip_path_ && job_.HasPlan()) {
    job_.SkipPath();
  }
  if (skip_area_ || skip_path_) {
    approached_ = false;
    spinup_since_.reset();
    CheckpointNow();
  }
  skip_area_ = skip_path_ = false;
}

void LogicCore::Pause(int why, const Reason& reason) {
  if (state_ != State::kMowing) return;
  const bool was_paused = pause_ != 0;
  pause_ |= why;
  if (why == kPauseGps) gps_good_since_.reset();
  if (leg_ == Leg::kApproach || leg_ == Leg::kFollow) {
    CancelDrive(reason);
    stop_requested_ = true;
  }
  spinup_since_.reset();
  if (was_paused) return;
  pause_reason_ = reason;
  notices_.push_back("Mowing paused: " + reason.message);
  json details = {{"state", "PAUSED"}};
  PutReason(&details, "STATE", reason);
  AddEvent("STATE", std::move(details));
  actions_changed_ = true;
  CheckpointNow();
}

void LogicCore::Unpause(int why) {
  if (pause_ == 0) return;
  pause_ &= ~why;
  actions_changed_ = true;
  if (pause_ != 0) return;
  pause_reason_ = Reason();
  // From where it stopped: the start of the path may be behind it now.
  approached_ = false;
  AddEvent("STATE", {{"state", "MOWING"}});
}

void LogicCore::EndJob(bool finished) {
  if (state_ != State::kMowing) return;
  if (finished) {
    AddEvent("JOB_COMPLETE", json::object());
    job_.Complete(config_.mow_angle_increment);
  }
  SetState(State::kIdle);
  CheckpointNow();
}

JobStep LogicCore::NextJobStep(double now_s) {
  JobStep step;
  if (state_ != State::kMowing || pause_ != 0 || emergency_ || leg_ != Leg::kNone) return step;
  if (!job_.HasPlan()) {
    std::string notice;
    if (!job_.SeekArea(&notice)) {
      if (!notice.empty()) notices_.push_back(notice);
      EndJob(true);
      return step;
    }
    if (!notice.empty()) notices_.push_back(notice);
    const JobArea* area = job_.CurrentArea();
    if (area->id != announced_area_) {
      announced_area_ = area->id;
      AddEvent("AREA", {{"area_id", area->id}, {"area_name", area->name}});
    }
    leg_ = Leg::kPlanning;
    step.kind = JobStep::Kind::kPlan;
    step.area_id = area->id;
    return step;
  }
  const JobPose* goal = job_.CurrentPose();
  if (!goal) return step;
  if (!pose_) {
    Pause(kPauseGps, Reason("NO_POSITION", "No position from the localization"));
    return step;
  }
  if (!approached_ && std::hypot(goal->x - pose_->x, goal->y - pose_->y) > config_.approach_distance) {
    leg_ = Leg::kApproach;
    step.kind = JobStep::Kind::kApproach;
    step.goal = *goal;
    return step;
  }
  // The blade gets up to speed before a path that starts mowing, like in OpenMower.
  if (config_.mow_with_blade && IsMowKind(goal->kind) && config_.mower_spinup_rpm > 0 &&
      std::abs(hw_.blade.rpm) < config_.mower_spinup_rpm) {
    if (!spinup_since_) spinup_since_ = now_s;
    if (now_s - *spinup_since_ > config_.mower_spinup_timeout) {
      AddEvent("MOW_MOTOR_SPINUP_FAILED", json::object());
      Pause(kPauseFault, Reason("BLADE_SPINUP_FAILED", "The blade did not reach its speed"));
    }
    return step;
  }
  spinup_since_.reset();
  leg_ = Leg::kFollow;
  step.kind = JobStep::Kind::kFollow;
  step.path = *job_.CurrentPath();
  step.start_index = static_cast<size_t>(job_.progress().index);
  return step;
}

void LogicCore::OnJobPlanned(std::vector<JobPath> paths, const std::string& fingerprint) {
  if (state_ != State::kMowing || leg_ != Leg::kPlanning) return;
  leg_ = Leg::kNone;
  job_.SetPlan(std::move(paths), fingerprint);
  approached_ = false;
  CheckpointNow();
}

void LogicCore::OnJobPlanFailed(const std::string& message) {
  if (state_ != State::kMowing || leg_ != Leg::kPlanning) return;
  leg_ = Leg::kNone;
  Pause(kPauseFault, Reason("PLANNING_FAILED", "Planning failed: " + message));
}

void LogicCore::OnJobProgress(size_t index) {
  if (state_ != State::kMowing || leg_ != Leg::kFollow) return;
  job_.OnIndex(index);
}

void LogicCore::OnLegEnded(LegEnd end, size_t index, const Reason& reason) {
  if (state_ != State::kMowing || (leg_ != Leg::kApproach && leg_ != Leg::kFollow)) return;
  const bool follow = leg_ == Leg::kFollow;
  leg_ = Leg::kNone;
  if (follow) job_.OnIndex(index);
  if (skip_area_ || skip_path_) {
    ApplySkips();
    return;
  }
  switch (end) {
    case LegEnd::kDone:
      if (follow) {
        job_.PathDone();
        approached_ = false;
        CheckpointNow();
      } else {
        approached_ = true;
      }
      break;
    case LegEnd::kCanceled: approached_ = false; break;
    case LegEnd::kGps:
      // The follower's text carries no numbers: the event gets the accuracy and the limit like a refused start.
      if (reason.code == "POSITION_ACCURACY" && pose_ && !(pose_->position_accuracy <= config_.max_position_accuracy)) {
        Pause(kPauseGps, AccuracyReason(pose_->position_accuracy));
      } else {
        Pause(kPauseGps, reason);
      }
      break;
    case LegEnd::kEmergency: Pause(kPauseEmergency, reason); break;
    case LegEnd::kFailed:
      approached_ = false;
      Pause(kPauseFault, reason);
      break;
  }
  if (follow) CheckpointNow();
}

void LogicCore::OnTick(double now_s) {
  if (state_ != State::kMowing) return;
  if (pause_ & kPauseGps) {
    if (!PoseGood()) {
      gps_good_since_.reset();
    } else if (!gps_good_since_) {
      gps_good_since_ = now_s;
    } else if (now_s - *gps_good_since_ >= config_.gps_wait_time) {
      gps_good_since_.reset();
      Unpause(kPauseGps);
    }
  }
  if (hw_.battery_ok && hw_.battery_voltage < config_.battery_critical_voltage) {
    if (!battery_low_since_) battery_low_since_ = now_s;
    if (now_s - *battery_low_since_ >= config_.battery_critical_time) {
      AddEvent("BATTERY_LOW", {{"battery_voltage", hw_.battery_voltage}, {"limit", config_.battery_critical_voltage}});
      notices_.push_back("Mowing ends: the battery is low");
      EndJob(false);
      return;
    }
  } else {
    battery_low_since_.reset();
  }
  if (leg_ == Leg::kFollow && now_s - last_checkpoint_s_ >= config_.checkpoint_period) {
    last_checkpoint_s_ = now_s;
    CheckpointNow();
  }
}

bool LogicCore::BladeWanted() const {
  if (state_ != State::kMowing || pause_ != 0 || emergency_ || !config_.mow_with_blade || !safety_measured_ ||
      !hw_.telemetry_ok || hw_.dock_contact) {
    return false;
  }
  if (spinup_since_) return true;
  const JobPose* pose = job_.CurrentPose();
  return leg_ == Leg::kFollow && pose && IsMowKind(pose->kind);
}

void LogicCore::OnBladeRefused(const std::string& message) {
  if (!BladeWanted()) return;
  Pause(kPauseFault, Reason("BLADE_REFUSED", "The blade was refused: " + message));
}

void LogicCore::CheckpointNow() {
  checkpoint_due_ = true;
}

std::optional<std::string> LogicCore::TakeCheckpoint() {
  if (!checkpoint_due_) return std::nullopt;
  checkpoint_due_ = false;
  return CheckpointText(job_.progress());
}

double LogicCore::AngleIncrementSum() const {
  return job_.progress().angle_increment_sum;
}

}  // namespace mowbite_logic
