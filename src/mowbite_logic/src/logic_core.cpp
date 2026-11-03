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
const std::string kUndockingPrefix = "mower_logic:undocking";
const std::string kDockingPrefix = "mower_logic:docking";

// 32 letters and digits, random: the ids of jobs and sessions.
std::string NewId() {
  static const char kAlphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<size_t> pick(0, sizeof(kAlphabet) - 2);
  std::string id(32, '0');
  for (auto& c : id) c = kAlphabet[pick(rng)];
  return id;
}

// EmergencyReason of OpenMower up to bit 12, then the own bits of hw/status, one name per bit.
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
                                    "FIRMWARE_INCOMPATIBLE",
                                    "TILT",
                                    "FENCE",
                                    "STUCK",
                                    "IMU_TIMEOUT",
                                    "MOWER_NOT_STOPPED"};

SensorDefinition Range(std::string id, std::string name, std::string unit, SensorMeaning meaning, double min,
                       double max) {
  SensorDefinition s;
  s.id = std::move(id);
  s.name = std::move(name);
  s.unit = std::move(unit);
  s.meaning = meaning;
  s.min = min;
  s.max = max;
  return s;
}

SensorDefinition Temperature(std::string id, std::string name, double min, double max, double critical_high) {
  SensorDefinition s = Range(std::move(id), std::move(name), "deg.C", SensorMeaning::kTemperature, min, max);
  s.critical_high = critical_high;
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

// Why mowing pauses or a drive ends for an emergency, worded alike whether hw/status or a refused blade told it.
Reason EmergencyWhy(int reason) {
  return Reason("EMERGENCY", "Emergency: " + EmergencyReasonNames(reason));
}

RecorderConfig RecorderConfigOf(const LogicConfig& config) {
  RecorderConfig recorder = config.recorder;
  recorder.max_position_accuracy = config.max_position_accuracy;
  recorder.dock_backwards = config.dock_contact_x < 0;
  return recorder;
}

std::string Metres(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.2f", value);
  return text;
}

std::string Degrees(double radians) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.0f", radians * 180.0 / M_PI);
  return text;
}

}  // namespace

LogicCore::LogicCore(const LogicConfig& config) : config_(config), recorder_(RecorderConfigOf(config)) {
  // Without telemetry the mower counts as stopped until the driver reports.
  emergency_ = true;
}

void LogicCore::OnHardwareStatus(const HardwareSnapshot& hw) {
  hw_ = hw;
  recorder_.SetDockContact(DockContact());
  UpdateEmergency();
  if (state_ == State::kDocking) OnDockingContact();
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
  recorder_.SetDockContact(false);
  UpdateEmergency();
}

void LogicCore::OnPose(const PoseSnapshot& pose) {
  pose_ = pose;
  last_x_ = pose.x;
  last_y_ = pose.y;
  if (state_ != State::kAreaRecording) return;
  recorder_.OnPose(pose.x, pose.y, pose.position_accuracy, pose.orientation_valid);
  PullRecorder();
}

void LogicCore::OnGnssFix(double x, double y) {
  if (state_ != State::kUndocking || undock_stopping_) return;
  // The first fix of the undocking is where the course is measured from; the mower stands or drives straight.
  if (!undock_fix_start_) {
    undock_fix_start_ = Point2{x, y};
    return;
  }
  if (undock_step_ != UndockStep::kDrive) return;
  const auto error = CourseError(undock_fix_start_->x, undock_fix_start_->y, undock_heading_, x, y,
                                 config_.undock.course_distance);
  if (!error || std::abs(*error) <= config_.undock.course_limit) return;
  const std::string text = "Undocking stopped: the mower drove " + Degrees(std::abs(*error)) +
                           " deg off the line of the docking station, at most " + Degrees(config_.undock.course_limit) +
                           " deg allowed: the station may be turned, or its heading was recorded wrong";
  StopUndocking(Reason("path_failed", text), Reason("STOPPED", text));
}

void LogicCore::OnPoseTimeout() {
  pose_.reset();
  if (state_ == State::kDriving) CancelDrive(Reason("POSE_LOST", "No position from the localization"));
  if (state_ == State::kMowing) Pause(kPauseGps, Reason("NO_POSITION", "No position from the localization"));
  if (state_ == State::kUndocking) {
    StopUndocking(Reason("no_gps", "Undocking stopped: no position from the localization"),
                  Reason("POSE_LOST", "No position from the localization"));
  }
  if (state_ == State::kDocking && (docking_leg_ || docking_leg_due_)) {
    CancelDockingLeg(Reason("POSE_LOST", "No position from the localization"));
    WaitForDockingGps();
  }
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
    const Reason why = EmergencyWhy(reason_);
    if (emergency && state_ == State::kDriving) CancelDrive(why);
    if (emergency && state_ == State::kMowing) Pause(kPauseEmergency, why);
    // Like OpenMower: the emergency tells, no UNDOCKING_FAILED or DOCKING_FAILED.
    if (emergency && state_ == State::kUndocking) StopUndocking(std::nullopt, why);
    if (emergency && state_ == State::kDocking) EndDocking(why);
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
    if (DockContact()) {
      BeginDockCheck(DockCheck::kMowing);
      return ActionResult::kDone;
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
  if (action_id == kUndockingPrefix + "/abort_undocking") {
    if (state_ != State::kUndocking) return ActionResult::kRejected;
    AbortUndocking(Reason("STOPPED", "Aborted by the app"));
    return ActionResult::kDone;
  }
  if (action_id == kDockingPrefix + "/abort_docking") {
    if (state_ != State::kDocking) return ActionResult::kRejected;
    notices_.push_back("Docking stopped: aborted by the app");
    EndDocking(Reason("STOPPED", "Aborted by the app"));
    return ActionResult::kDone;
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
  const bool known_prefix = action_id.rfind(kIdlePrefix + "/", 0) == 0 ||
                            action_id.rfind(kRecordingPrefix + "/", 0) == 0 ||
                            action_id.rfind(kUndockingPrefix + "/", 0) == 0 ||
                            action_id.rfind(kDockingPrefix + "/", 0) == 0;
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
  if (state_ == State::kUndocking) {
    const std::string text = "Undocking stopped: no safety measurement of this platform (hw/capabilities)";
    StopUndocking(Reason("path_failed", text), Reason("NOT_ALLOWED", text));
  }
  if (state_ == State::kDocking) {
    const bool approach = docking_last_kind_ == DockingLeg::Kind::kApproach;
    CountDockingFailure(Reason(approach ? "approach_failed" : "dock_failed",
                               "Docking stopped: no safety measurement of this platform (hw/capabilities)"),
                        true);
  }
  Pause(kPauseFault, Reason("NO_SAFETY_MEASUREMENT", "No safety measurement of this platform (hw/capabilities)"));
}

Reason LogicCore::DriveProblem() const {
  const Reason why = Busy();
  return why.empty() ? MoveProblem() : why;
}

Reason LogicCore::MoveProblem() const {
  Reason why = Unsafe();
  if (why.empty()) why = PositionProblem();
  if (why.empty()) why = DockProblem();
  return why;
}

Reason LogicCore::StartDrive(double x, double y) {
  const Reason why = DriveProblem();
  if (!why.empty()) {
    RefuseDrive(why);
    return why;
  }
  BeginDrive(x, y);
  return {};
}

void LogicCore::BeginDrive(double x, double y) {
  drive_cancel_ = false;
  cancel_reason_.reset();
  SetState(State::kDriving);
  AddEvent("DRIVE", {{"phase", "start"}, {"goal_x", x}, {"goal_y", y}});
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
  if (state_ == State::kUndocking) return undock_step_ == UndockStep::kDrive && !undock_stopping_;
  if (state_ == State::kDocking) return docking_leg_.has_value() && !docking_hold_;
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
  if (state_ == State::kAreaRecording || state_ == State::kDriving || state_ == State::kMowing ||
      state_ == State::kUndocking || state_ == State::kDocking) {
    stop_requested_ = true;
  }
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
    case State::kUndocking: return "UNDOCKING";
    case State::kDocking: return "DOCKING";
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
  groups.push_back({kUndockingPrefix, {{"abort_undocking", "Stop Undocking", state_ == State::kUndocking}}});
  groups.push_back({kDockingPrefix, {{"abort_docking", "Stop Docking", state_ == State::kDocking}}});
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
      Range("om_v_battery", "V Battery", "V", SensorMeaning::kVoltage, config_.battery_empty_voltage,
            config_.battery_full_voltage);
  battery.critical_low = config_.battery_empty_voltage;
  sensors.push_back(battery);
  // The MCU reports neither a charge voltage nor a charger state: both are derived, the names say so.
  sensors.push_back(
      Range("om_v_charge", "V Charge (dock contact)", "V", SensorMeaning::kVoltage, 0, config_.dock_charge_voltage + 2));
  sensors.push_back(Range("om_charge_current", "Charge Current", "A", SensorMeaning::kCurrent, 0, 7));
  SensorDefinition charge_state;
  charge_state.id = "om_charge_state";
  charge_state.name = "Charge State (derived)";
  charge_state.text = true;
  sensors.push_back(charge_state);
  sensors.push_back(Range("om_mow_motor_rpm", "Mow Motor Revolutions", "rpm", SensorMeaning::kRevolutions, 0, 3500));
  sensors.push_back(Range("om_mow_motor_current", "Mow Motor Current", "A", SensorMeaning::kCurrent, 0, 5));
  // One temperature per motor board, so no separate mow ESC temperature. Own limits: the drive motors hold position
  // up to 68 degC, warn above 80.
  SensorDefinition gps_accuracy;
  gps_accuracy.id = "om_gps_accuracy";
  gps_accuracy.name = "GPS Accuracy";
  gps_accuracy.unit = "m";
  gps_accuracy.meaning = SensorMeaning::kDistance;
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
  if (state_ == State::kIdle) {
    if (dock_check_ == DockCheck::kNone) return {};
    return Reason("BUSY", "The mower is busy: checking its position on the docking station").With("state", "checking");
  }
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

Reason LogicCore::AccuracyReason(float accuracy, double limit) const {
  char text[160];
  std::snprintf(text, sizeof(text), "Position accuracy %.3f m is worse than %.3f m", accuracy, limit);
  return Reason("POSITION_ACCURACY", text).With("accuracy", accuracy).With("limit", limit);
}

Reason LogicCore::PositionProblem() const {
  if (!pose_) return Reason("NO_POSITION", "No position from the localization");
  // On the docking station the reference check sets the heading, and it wants a better position.
  const double limit = DockContact() ? DockAccuracyLimit() : config_.max_position_accuracy;
  if (!(pose_->position_accuracy <= limit)) {
    Reason why = AccuracyReason(pose_->position_accuracy, limit);
    if (DockContact()) why.message += " on the docking station";
    return why;
  }
  if (!pose_->orientation_valid && !DockContact()) {
    return Reason("NO_HEADING", "The heading is not aligned yet");
  }
  return {};
}

Reason LogicCore::DockProblem() const {
  if (!DockContact()) return {};
  if (!dock_) return Reason("NO_DOCK", "The mower is on a docking station, but the map has none: record it first");
  return UndockEndProblem(dock_->x, dock_->y);
}

Reason LogicCore::UndockEndProblem(double x, double y) const {
  // Contacts in front: undocking backwards is not built, the reference check ends in ON_DOCK.
  if (config_.dock_contact_x >= 0) return {};
  const double heading = DockedHeading();
  const double distance = config_.undock.distance;
  if (Reachable(drivable_, obstacles_, x + distance * std::cos(heading), y + distance * std::sin(heading))) return {};
  return Reason("DOCK_UNREACHABLE", "Leaving the docking station " + Metres(distance) +
                                        " m straight ahead ends outside of the mowing and navigation areas: let an "
                                        "area reach there, or place the station closer to one")
      .With("distance", distance);
}

void LogicCore::OnDrivableAreas(std::vector<Ring> drivable, std::vector<Ring> obstacles, const Reason& problem) {
  drivable_ = std::move(drivable);
  obstacles_ = std::move(obstacles);
  drivable_problem_ = problem;
}

void LogicCore::SetUndockDistance(double distance) {
  config_.undock.distance = distance;
}

double LogicCore::DockAccuracyLimit() const {
  return std::min(config_.dock_max_position_accuracy, config_.max_position_accuracy);
}

bool LogicCore::DockContact() const {
  return hw_.telemetry_ok && hw_.dock_contact;
}

void LogicCore::OnMapDock(const std::optional<DockPose>& dock) {
  dock_ = dock;
}

double LogicCore::DockedHeading() const {
  const double heading = dock_ ? dock_->heading : 0.0;
  const double facing = config_.dock_contact_x < 0 ? heading + M_PI : heading;
  return std::atan2(std::sin(facing), std::cos(facing));
}

void LogicCore::BeginDockCheck(DockCheck purpose) {
  dock_check_ = purpose;
  dock_check_due_ = true;
  actions_changed_ = true;
  notices_.push_back("On the docking station: checking the position against the recorded one first");
}

void LogicCore::CheckDockBeforeDrive(double x, double y) {
  drive_x_ = x;
  drive_y_ = y;
  BeginDockCheck(DockCheck::kDrive);
}

std::optional<double> LogicCore::TakeDockCheck() {
  if (!dock_check_due_) return std::nullopt;
  dock_check_due_ = false;
  return DockedHeading();
}

bool LogicCore::DockCheckPending() const {
  return dock_check_ != DockCheck::kNone;
}

Reason LogicCore::OnDockCheck(bool ok, double x, double y, float accuracy, const std::string& message) {
  const DockCheck purpose = dock_check_;
  if (purpose == DockCheck::kNone) return Reason("STOPPED", "No reference check was on its way");
  dock_check_ = DockCheck::kNone;
  dock_check_due_ = false;
  actions_changed_ = true;
  // What changed meanwhile comes first: an emergency, the contact, then the rest of a start or a drive.
  Reason why = Unsafe();
  if (why.empty() && !DockContact()) {
    why = Reason("NO_DOCK_CONTACT", "The contact with the docking station ended during the reference check");
  }
  if (why.empty()) why = purpose == DockCheck::kMowing ? StartProblem() : DriveProblem();
  if (why.empty() && !ok) why = Reason("NO_POSITION", message.empty() ? "No answer from the localization" : message);
  if (why.empty() && !(accuracy <= DockAccuracyLimit())) {
    why = AccuracyReason(accuracy, DockAccuracyLimit());
    why.message += " on the docking station";
  }
  if (why.empty()) {
    const double distance = std::hypot(x - dock_->x, y - dock_->y);
    const double limit = config_.dock_reference_limit;
    if (distance > limit) {
      why = Reason("REFERENCE_MISMATCH", "On the docking station the position is " + Metres(distance) +
                                             " m from the recorded one, at most " + Metres(limit) +
                                             " m allowed: the corrections may have another reference, or the "
                                             "station was moved")
                .With("distance", distance)
                .With("limit", limit);
    } else {
      notices_.push_back("Reference check on the docking station passed: " + Metres(distance) +
                         " m from the recorded position");
      if (config_.dock_contact_x >= 0) {
        why = Reason("ON_DOCK", "The position on the docking station fits, but undocking backwards is not built yet: "
                                "drive the mower off by remote control first")
                  .With("distance", distance)
                  .With("limit", limit);
      } else {
        // The way out from where the mower stands.
        why = UndockEndProblem(x, y);
      }
    }
  }
  if (why.empty()) {
    StartUndocking(purpose, x, y);
    return why;
  }
  if (purpose == DockCheck::kMowing) {
    RefuseStart(why);
  } else {
    RefuseDrive(why);
  }
  return why;
}

void LogicCore::StartUndocking(DockCheck purpose, double x, double y) {
  undock_for_ = purpose;
  undock_step_ = UndockStep::kChargeOff;
  undock_x_ = x;
  undock_y_ = y;
  undock_heading_ = DockedHeading();
  undock_fix_start_.reset();
  charge_off_due_ = true;
  undock_path_due_.reset();
  undock_stopping_ = false;
  undock_failure_.reset();
  undock_stop_why_ = Reason();
  notices_.push_back("Undocking: charging off, then " + Metres(config_.undock.distance) + " m straight ahead");
  SetState(State::kUndocking);
}

bool LogicCore::Undocking() const {
  return state_ == State::kUndocking;
}

bool LogicCore::UndockingForDrive() const {
  return state_ == State::kUndocking && undock_for_ == DockCheck::kDrive;
}

bool LogicCore::TakeChargeOff() {
  const bool due = charge_off_due_;
  charge_off_due_ = false;
  return due;
}

void LogicCore::OnChargeOff(bool ok, const std::string& message) {
  if (state_ != State::kUndocking || undock_step_ != UndockStep::kChargeOff) return;
  if (!ok) {
    const std::string text = "Undocking failed: charging could not be switched off (" + message + ")";
    EndUndocking(Reason("path_failed", text), Reason("STOPPED", text));
    return;
  }
  undock_step_ = UndockStep::kDrive;
  undock_path_due_ = PlanUndock(undock_x_, undock_y_, undock_heading_, config_.undock);
}

std::optional<UndockPath> LogicCore::TakeUndockPath() {
  std::optional<UndockPath> path;
  path.swap(undock_path_due_);
  return path;
}

void LogicCore::StopUndocking(const std::optional<Reason>& failure, const Reason& why) {
  if (state_ != State::kUndocking || undock_stopping_) return;
  // Before the path went to the follower nothing moves: it ends at once.
  if (undock_step_ == UndockStep::kChargeOff || undock_path_due_) {
    EndUndocking(failure, why);
    return;
  }
  undock_stopping_ = true;
  undock_failure_ = failure;
  undock_stop_why_ = why;
  undock_path_due_.reset();
  CancelDrive(why);
  stop_requested_ = true;
  actions_changed_ = true;
}

void LogicCore::AbortUndocking(const Reason& why) {
  StopUndocking(std::nullopt, why);
}

void LogicCore::OnUndockEnded(LegEnd end, const Reason& reason) {
  if (state_ != State::kUndocking || undock_step_ != UndockStep::kDrive) return;
  if (end == LegEnd::kDone && !undock_stopping_) {
    Undocked();
    return;
  }
  // Stopped from here: the cause decided already. Otherwise the follower's: an emergency tells by itself.
  if (undock_stopping_) {
    EndUndocking(undock_failure_, undock_stop_why_);
  } else if (end == LegEnd::kEmergency) {
    EndUndocking(std::nullopt, reason);
  } else if (end == LegEnd::kGps) {
    const Reason failure("no_gps", "Undocking stopped: " + reason.message);
    EndUndocking(failure, failure);
  } else {
    const Reason failure("path_failed", "Undocking failed: " + reason.message);
    EndUndocking(failure, failure);
  }
}

void LogicCore::EndUndocking(const std::optional<Reason>& failure, const Reason& why) {
  if (failure) {
    json details = json::object();
    PutReason(&details, "UNDOCKING_FAILED", *failure);
    AddEvent("UNDOCKING_FAILED", std::move(details));
  }
  if (!failure) notices_.push_back("Undocking stopped: " + why.message);
  UndockOutcome outcome;
  outcome.for_drive = undock_for_ == DockCheck::kDrive;
  outcome.why = failure.value_or(why);
  undock_outcome_ = outcome;
  // The mower may still stand on the contacts.
  charge_on_due_ = true;
  charge_off_due_ = false;
  undock_path_due_.reset();
  undock_stopping_ = false;
  undock_for_ = DockCheck::kNone;
  SetState(State::kIdle);
}

void LogicCore::Undocked() {
  AddEvent("UNDOCKED", json::object());
  notices_.push_back("Undocked");
  UndockOutcome outcome;
  outcome.undocked = true;
  outcome.for_drive = undock_for_ == DockCheck::kDrive;
  undock_for_ = DockCheck::kNone;
  // On like a start away from the station; what changed meanwhile comes first.
  outcome.why = outcome.for_drive ? MoveProblem() : MowProblem();
  if (!outcome.why.empty()) {
    if (outcome.for_drive) {
      RefuseDrive(outcome.why);
    } else {
      RefuseStart(outcome.why);
    }
    SetState(State::kIdle);
  } else if (outcome.for_drive) {
    BeginDrive(drive_x_, drive_y_);
  } else {
    StartMowing();
  }
  undock_outcome_ = outcome;
}

bool LogicCore::TakeChargeOn() {
  const bool due = charge_on_due_;
  charge_on_due_ = false;
  return due;
}

std::optional<UndockOutcome> LogicCore::TakeUndockOutcome() {
  std::optional<UndockOutcome> outcome;
  outcome.swap(undock_outcome_);
  return outcome;
}

bool LogicCore::CanDock() const {
  return config_.dock_contact_x < 0;
}

bool LogicCore::Docking() const {
  return state_ == State::kDocking;
}

void LogicCore::SetDockingSettings(double approach_distance, int retry_count) {
  config_.dock.approach_distance = approach_distance;
  config_.dock.retry_count = retry_count;
}

DockingSequence LogicCore::NewDocking() const {
  return DockingSequence(config_.dock, config_.undock, dock_.value_or(DockPose()), DockedHeading());
}

Reason LogicCore::ApproachProblem() const {
  const Point2 p = NewDocking().ApproachPoint();
  if (Reachable(drivable_, obstacles_, p.x, p.y)) return {};
  const double distance = config_.dock.approach_distance;
  return Reason("DOCK_UNREACHABLE", "The way onto the docking station begins " + Metres(distance) +
                                        " m in front of it, outside of the mowing and navigation areas: let an area "
                                        "reach there, or place the station closer to one")
      .With("distance", distance);
}

Reason LogicCore::DockingProblem() const {
  Reason why = Busy();
  if (!why.empty()) return why;
  if (DockContact()) return Reason("ALREADY_THERE", "The mower is on the docking station already");
  why = Unsafe();
  if (why.empty()) why = PositionProblem();
  if (why.empty()) why = drivable_problem_;
  if (why.empty() && !dock_) why = Reason("NO_DOCK", "The map has no docking station: record it first");
  if (why.empty() && !CanDock()) why = Reason("NO_DOCK", "Docking onto contacts in front is not built");
  if (why.empty()) why = ApproachProblem();
  return why;
}

Reason LogicCore::StartDocking() {
  const Reason why = DockingProblem();
  if (!why.empty()) {
    RefuseDrive(why);
    return why;
  }
  BeginDocking();
  return {};
}

void LogicCore::GoHome() {
  // Never after an emergency or without the safety measurement: the mower does not drive off by itself then.
  if (!CanDock() || !dock_ || !Unsafe().empty()) {
    SetState(State::kIdle);
    return;
  }
  // On the contacts already, like OpenMower.
  if (DockContact()) {
    AddEvent("DOCKED", json::object());
    SetState(State::kIdle);
    return;
  }
  Reason why = drivable_problem_;
  if (why.empty()) why = ApproachProblem();
  if (!why.empty()) {
    // An approach that cannot succeed fails like OpenMower's: DOCKING_FAILED approach_failed, not DOCK_UNREACHABLE.
    json details = json::object();
    PutReason(&details, "DOCKING_FAILED", Reason("approach_failed", why.message).With("attempts", 1));
    AddEvent("DOCKING_FAILED", std::move(details));
    notices_.push_back("Docking failed: " + why.message);
    SetState(State::kIdle);
    return;
  }
  BeginDocking();
}

void LogicCore::BeginDocking() {
  docking_ = NewDocking();
  docking_leg_due_.reset();
  docking_leg_.reset();
  docking_last_kind_ = DockingLeg::Kind::kApproach;
  docking_hold_ = false;
  docking_failure_.reset();
  docking_gps_wait_ = false;
  docking_gps_since_.reset();
  docking_gps_good_since_.reset();
  SetState(State::kDocking);
  const Point2 p = docking_.ApproachPoint();
  notices_.push_back("Docking: to (" + Metres(p.x) + ", " + Metres(p.y) +
                     ") in front of the docking station, then backwards onto it");
  NextDockingTry();
}

void LogicCore::NextDockingTry() {
  if (!PoseGood()) {
    WaitForDockingGps();
    return;
  }
  // A failed try is counted at once where the mower does not drive out first.
  if (docking_failure_ && !docking_.OnLine(pose_->x, pose_->y)) {
    CountDockingFailure(*docking_failure_);
    return;
  }
  docking_leg_due_ = docking_.FirstLeg(pose_->x, pose_->y, pose_->heading);
}

std::optional<DockingLeg> LogicCore::TakeDockingLeg() {
  if (state_ != State::kDocking || !docking_leg_due_) return std::nullopt;
  std::optional<DockingLeg> leg;
  leg.swap(docking_leg_due_);
  docking_leg_ = leg->kind;
  docking_last_kind_ = leg->kind;
  docking_hold_ = false;
  return leg;
}

void LogicCore::OnDockingContact() {
  // On the way onto the station, or waiting there for a good position; driving out starts on the contacts.
  const bool onto =
      docking_leg_ && (*docking_leg_ == DockingLeg::Kind::kLine || *docking_leg_ == DockingLeg::Kind::kContact);
  if (!onto && !docking_gps_wait_) return;
  if (DockContact()) {
    Docked();
    return;
  }
  if (!onto) return;
  const bool raw = hw_.telemetry_ok && hw_.dock_contact_raw;
  if (raw && !docking_hold_) {
    // G18: one zero command, the driver sends three zero frames and then nothing while the contact settles.
    docking_hold_ = true;
    stop_requested_ = true;
    notices_.push_back("Docking: contact with the docking station, stopped until it is confirmed");
  } else if (!raw && docking_hold_) {
    docking_hold_ = false;
    notices_.push_back("Docking: the contact went away before it was confirmed, on slowly");
  }
}

void LogicCore::OnDockingLegEnded(LegEnd end, const Reason& reason) {
  if (state_ != State::kDocking || !docking_leg_) return;
  const DockingLeg::Kind kind = *docking_leg_;
  docking_leg_.reset();
  docking_hold_ = false;
  if (end == LegEnd::kEmergency) {
    notices_.push_back("Docking stopped: " + reason.message);
    EndDocking(reason);
    return;
  }
  if (end == LegEnd::kGps) {
    WaitForDockingGps();
    return;
  }
  if (end != LegEnd::kDone) {
    if (kind == DockingLeg::Kind::kApproach) {
      DockingTryFailed("approach_failed", "The way to the docking station failed: " + reason.message);
    } else if (kind == DockingLeg::Kind::kOut && docking_failure_) {
      CountDockingFailure(
          Reason(docking_failure_->code, docking_failure_->message + "; then driving out failed: " + reason.message));
    } else {
      DockingTryFailed("dock_failed", std::string("Docking failed on the ") + DockingLegName(kind) + ": " +
                                          reason.message);
    }
    return;
  }
  switch (kind) {
    case DockingLeg::Kind::kApproach: docking_leg_due_ = docking_.Line(); break;
    case DockingLeg::Kind::kOut:
      if (docking_failure_) {
        CountDockingFailure(*docking_failure_);
      } else {
        NextDockingTry();
      }
      break;
    case DockingLeg::Kind::kLine: {
      const std::string gate = pose_ ? docking_.CheckGate(pose_->x, pose_->y, pose_->heading,
                                                          pose_->position_accuracy, DockAccuracyLimit())
                                     : "at the gate there was no position";
      if (gate.empty()) {
        notices_.push_back("Docking: through the gate, onto the contacts");
        docking_leg_due_ = docking_.Contact();
      } else {
        DockingTryFailed("dock_failed", "Docking failed: " + gate);
      }
      break;
    }
    case DockingLeg::Kind::kContact:
      DockingTryFailed("dock_failed", "Docking failed: no contact with the docking station up to " +
                                          Metres(config_.dock.overshoot) + " m past it");
      break;
  }
}

void LogicCore::DockingTryFailed(const std::string& code, const std::string& message) {
  notices_.push_back(message);
  // Out to the approach point first; the try counts once the mower got there.
  if (pose_ && docking_last_kind_ != DockingLeg::Kind::kOut && docking_.OnLine(pose_->x, pose_->y)) {
    docking_failure_ = Reason(code, message);
    docking_leg_due_ = docking_.Out(pose_->x, pose_->y, pose_->heading);
    return;
  }
  CountDockingFailure(Reason(code, message));
}

void LogicCore::CountDockingFailure(Reason why, bool last) {
  docking_failure_.reset();
  const bool again = docking_.Failed() && !last;
  const char* type = again ? "DOCKING_RETRY" : "DOCKING_FAILED";
  json details = json::object();
  PutReason(&details, type, Reason(why).With("attempts", docking_.failures()));
  AddEvent(type, std::move(details));
  if (!again) {
    notices_.push_back("Docking failed, tries: " + std::to_string(docking_.failures()));
    EndDocking(Reason("STOPPED", why.message));
    return;
  }
  notices_.push_back("Docking: try " + std::to_string(docking_.failures() + 1) + " of " +
                     std::to_string(docking_.config().retry_count + 1));
  NextDockingTry();
}

void LogicCore::WaitForDockingGps() {
  docking_leg_due_.reset();
  docking_gps_wait_ = true;
  docking_gps_since_.reset();
  docking_gps_good_since_.reset();
  notices_.push_back("Docking waits for a good position");
}

void LogicCore::TickDocking(double now_s) {
  if (!docking_gps_wait_) return;
  if (!docking_gps_since_) docking_gps_since_ = now_s;
  if (PoseGood()) {
    if (!docking_gps_good_since_) docking_gps_good_since_ = now_s;
    if (now_s - *docking_gps_good_since_ < config_.gps_wait_time) return;
    docking_gps_wait_ = false;
    notices_.push_back("Docking goes on: the position is good again");
    NextDockingTry();
    return;
  }
  docking_gps_good_since_.reset();
  if (now_s - *docking_gps_since_ < config_.gps_timeout) return;
  char text[96];
  std::snprintf(text, sizeof(text), "Docking failed: no good position for %.0f s", config_.gps_timeout);
  const bool approach = docking_last_kind_ == DockingLeg::Kind::kApproach;
  CountDockingFailure(Reason(approach ? "approach_failed" : "dock_failed", text), true);
}

void LogicCore::CancelDockingLeg(const Reason& why) {
  docking_leg_due_.reset();
  if (!docking_leg_) return;
  docking_leg_.reset();
  docking_hold_ = false;
  CancelDrive(why);
  stop_requested_ = true;
}

void LogicCore::Docked() {
  AddEvent("DOCKED", json::object());
  notices_.push_back("Docked");
  EndDocking(Reason("STOPPED", "Docked"));
}

void LogicCore::EndDocking(const Reason& why) {
  CancelDockingLeg(why);
  docking_failure_.reset();
  docking_gps_wait_ = false;
  SetState(State::kIdle);
}

Reason LogicCore::StartProblem() const {
  const Reason why = Busy();
  return why.empty() ? MowProblem() : why;
}

Reason LogicCore::MowProblem() const {
  Reason why = Unsafe();
  if (!why.empty()) return why;
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
  return DockProblem();
}

void LogicCore::RefuseStart(const Reason& why) {
  json details = json::object();
  PutReason(&details, "MOWING_REFUSED", why);
  AddEvent("MOWING_REFUSED", std::move(details));
}

void LogicCore::GeometryChanged(const std::vector<std::string>& changed, const std::vector<std::string>& below_default,
                                int version) {
  const auto list = [](const std::vector<std::string>& names) {
    std::string text;
    for (const auto& name : names) text += (text.empty() ? "" : ", ") + name;
    return text;
  };
  json details = json::object();
  details["changed"] = changed;
  details["below_default"] = below_default;
  details["version"] = version;
  std::string message = "The dimensions of the mower changed: " + list(changed);
  if (below_default.empty()) {
    details["severity"] = "info";
  } else {
    details["reason"] = "BELOW_DEFAULT";
    details["severity"] = "warning";
    message += ". " + list(below_default) + " below the platform's value: check the measurement";
  }
  details["message"] = message;
  AddEvent("GEOMETRY_CHANGED", std::move(details));
}

bool LogicCore::Mowing() const {
  return state_ == State::kMowing;
}

bool LogicCore::Idle() const {
  return state_ == State::kIdle;
}

void LogicCore::SetGpsSettings(double max_position_accuracy, double gps_wait_time, double gps_timeout) {
  config_.max_position_accuracy = max_position_accuracy;
  config_.gps_wait_time = gps_wait_time;
  config_.gps_timeout = gps_timeout;
  recorder_.SetMaxPositionAccuracy(max_position_accuracy);
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
    EndJob(false, true);
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

void LogicCore::EndJob(bool finished, bool home) {
  if (state_ != State::kMowing) return;
  if (finished) {
    AddEvent("JOB_COMPLETE", json::object());
    job_.Complete();
  }
  if (home) {
    GoHome();
  } else {
    SetState(State::kIdle);
  }
  CheckpointNow();
}

JobStep LogicCore::NextJobStep(double now_s) {
  JobStep step;
  if (state_ != State::kMowing || pause_ != 0 || emergency_ || leg_ != Leg::kNone) return step;
  if (!job_.HasPlan()) {
    std::string notice;
    if (!job_.SeekArea(&notice)) {
      if (!notice.empty()) notices_.push_back(notice);
      EndJob(true, true);
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
        Pause(kPauseGps, AccuracyReason(pose_->position_accuracy, config_.max_position_accuracy));
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
  if (state_ == State::kDocking) TickDocking(now_s);
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
      EndJob(false, false);
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

void LogicCore::OnBladeRefused(const std::string& message, int emergency_reason) {
  if (!BladeWanted()) return;
  // The driver blocked before hw/status showed it: the pause of that emergency, its flag and event follow hw/status.
  if (emergency_reason != 0) {
    Pause(kPauseEmergency, EmergencyWhy(emergency_reason));
    return;
  }
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

}  // namespace mowbite_logic
