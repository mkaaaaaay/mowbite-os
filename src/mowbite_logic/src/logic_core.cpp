#include "mowbite_logic/logic_core.h"

#include <algorithm>

namespace mowbite_logic {

namespace {

const std::string kRootPrefix = "mower_logic";
const std::string kIdlePrefix = "mower_logic:idle";
const std::string kRecordingPrefix = "mower_logic:area_recording";

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
  recorder_.OnPoseLost();
  PullRecorder();
}

void LogicCore::OnMapStatus(MapDatum datum, const std::string& reason) {
  const bool writable = datum == MapDatum::kMatch || datum == MapDatum::kEmpty;
  std::string why = reason;
  if (!writable && why.empty()) why = "The map does not fit the datum in use";
  if (writable != map_writable_) actions_changed_ = true;
  map_writable_ = writable;
  map_reason_ = writable ? "" : why;
  recorder_.SetMapWritable(writable, map_reason_);
}

std::optional<MapEdit> LogicCore::TakeMapEdit() {
  return recorder_.TakeMapEdit();
}

void LogicCore::OnMapEditResult(bool ok, const std::string& id, const std::string& message) {
  recorder_.OnMapEditResult(ok, id, message);
  PullRecorder();
}

void LogicCore::PullRecorder() {
  for (auto& event : recorder_.TakeEvents()) AddEvent(event.type, std::move(event.details));
  actions_changed_ |= recorder_.TakeActionsChanged();
}

std::vector<std::string> LogicCore::TakeNotices() {
  return recorder_.TakeNotices();
}

void LogicCore::OnPositionJump(double to_x, double to_y, double distance_m, bool after_outliers) {
  last_x_ = to_x;
  last_y_ = to_y;
  AddEvent("POSITION_JUMP", {{"distance", distance_m}, {"reason", after_outliers ? "outliers" : "gap"}});
}

// Every event carries the position where it happened.
void LogicCore::AddEvent(const std::string& type, json details) {
  details["x"] = last_x_;
  details["y"] = last_y_;
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
    AddEvent("EMERGENCY", {{"emergency", emergency}, {"reason", reason_}});
    actions_changed_ = true;
  }
}

ActionResult LogicCore::OnAction(const std::string& action_id) {
  if (action_id == kRootPrefix + "/reset_emergency") return ActionResult::kReleaseEmergency;
  if (action_id == kIdlePrefix + "/start_area_recording") {
    if (state_ != State::kIdle || emergency_) return ActionResult::kRejected;
    if (!map_writable_) {
      AddEvent("RECORDING_REFUSED", {{"action", "start_area_recording"}, {"reason", map_reason_}});
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

void LogicCore::SetState(State state) {
  if (state == state_) return;
  if (state_ == State::kAreaRecording) stop_requested_ = true;
  // Entering and leaving both start from nothing; what was not saved is gone, like with OpenMower.
  recorder_.Reset();
  recorder_.TakeActionsChanged();
  state_ = state;
  teleop_armed_ = false;
  AddEvent("STATE", {{"state", StateName(state)}});
  actions_changed_ = true;
}

const char* LogicCore::StateName(State state) {
  switch (state) {
    case State::kAreaRecording: return "AREA_RECORDING";
    case State::kIdle:
    default: return "IDLE";
  }
}

RobotStateData LogicCore::RobotState() const {
  RobotStateData s;
  s.state = StateName(state_);
  if (state_ == State::kAreaRecording) s.sub_state = recorder_.SubState();
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
  std::vector<ActionGroup> groups;
  groups.push_back({kRootPrefix, {{"reset_emergency", "Reset Emergency", true}}});
  // Mowing needs map, localization and path following, all later stages.
  groups.push_back({kIdlePrefix,
                    {{"start_mowing", "Start Mowing", false},
                     {"start_area_recording", "Start Area Recording", idle && !emergency_ && map_writable_},
                     {"reset_job", "Reset Job", false}}});
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

}  // namespace mowbite_logic
