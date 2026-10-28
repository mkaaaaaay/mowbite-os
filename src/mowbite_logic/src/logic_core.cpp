#include "mowbite_logic/logic_core.h"

#include <algorithm>

namespace mowbite_logic {

namespace {

const std::string kRootPrefix = "mower_logic";
const std::string kIdlePrefix = "mower_logic:idle";
const std::string kRecordingPrefix = "mower_logic:area_recording";

// SensorInfo value descriptions
constexpr uint8_t kTemperature = 1;
constexpr uint8_t kVoltage = 4;
constexpr uint8_t kCurrent = 5;
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

LogicCore::LogicCore(const LogicConfig& config) : config_(config) {
  // Without telemetry the mower counts as stopped until the MCU reports.
  emergency_ = true;
}

void LogicCore::OnMcuStatus(const McuSnapshot& mcu) {
  mcu_ = mcu;
  UpdateEmergency();
}

void LogicCore::OnMcuTimeout() {
  mcu_.telemetry_ok = false;
  mcu_.battery_ok = false;
  UpdateEmergency();
}

bool LogicCore::Emergency() const {
  return emergency_;
}

int LogicCore::EmergencyReason() const {
  int reason = 0;
  if (!mcu_.telemetry_ok) reason |= kEmergencyTimeoutInputs;
  if (mcu_.emergency_latched) reason |= kEmergencyLatch;
  if (mcu_.stop) reason |= kEmergencyStop;
  if (mcu_.lift) reason |= kEmergencyLift;
  if (mcu_.bumper) reason |= kEmergencyCollision;
  return reason;
}

void LogicCore::UpdateEmergency() {
  const bool emergency = mcu_.emergency_latched || !mcu_.telemetry_ok;
  // The first report after the start only sets the state, it is no change worth an event.
  if (!mcu_seen_) {
    mcu_seen_ = true;
    actions_changed_ |= emergency != emergency_;
    emergency_ = emergency;
    return;
  }
  if (emergency != emergency_) {
    emergency_ = emergency;
    events_.push_back({"EMERGENCY", {{"emergency", emergency}, {"reason", EmergencyReason()}}});
    actions_changed_ = true;
  }
}

ActionResult LogicCore::OnAction(const std::string& action_id) {
  if (action_id == kRootPrefix + "/reset_emergency") return ActionResult::kReleaseEmergency;
  if (action_id == kIdlePrefix + "/start_area_recording") {
    if (state_ != State::kIdle || emergency_) return ActionResult::kRejected;
    SetState(State::kAreaRecording);
    return ActionResult::kDone;
  }
  if (action_id == kRecordingPrefix + "/exit_recording_mode") {
    if (state_ != State::kAreaRecording) return ActionResult::kRejected;
    SetState(State::kIdle);
    return ActionResult::kDone;
  }
  const bool known_prefix =
      action_id.rfind(kIdlePrefix + "/", 0) == 0 || action_id.rfind(kRecordingPrefix + "/", 0) == 0;
  return known_prefix ? ActionResult::kRejected : ActionResult::kUnknown;
}

std::optional<std::pair<double, double>> LogicCore::OnTeleop(double vx, double vz) const {
  if (state_ != State::kAreaRecording || emergency_) return std::nullopt;
  return std::make_pair(vx, vz);
}

void LogicCore::SetState(State state) {
  if (state == state_) return;
  state_ = state;
  events_.push_back({"STATE", {{"state", StateName(state)}}});
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
  s.emergency = emergency_;
  s.is_charging = mcu_.battery_ok && mcu_.battery_dock;
  s.rain_detected = mcu_.telemetry_ok && mcu_.rain;
  if (mcu_.battery_ok) {
    s.battery_percentage = std::clamp(mcu_.battery_percentage / 100.0f, 0.0f, 1.0f);
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
                     {"start_area_recording", "Start Area Recording", idle && !emergency_},
                     {"reset_job", "Reset Job", false}}});
  groups.push_back({kRecordingPrefix,
                    {{"start_recording", "Start Recording", false},
                     {"stop_recording", "Stop Recording", false},
                     {"finish_navigation_area", "Save Navigation Area", false},
                     {"finish_mowing_area", "Save Mowing Area", false},
                     {"exit_recording_mode", "Exit", recording},
                     {"finish_discard", "Discard Area", false},
                     {"record_dock", "Record Docking point", false},
                     {"auto_point_collecting_enable", "Enable automatic point collecting", false},
                     {"auto_point_collecting_disable", "Disable automatic point collecting", false},
                     {"collect_point", "Collect point", false},
                     {"start_manual_mowing", "Start manual mowing", false},
                     {"stop_manual_mowing", "Stop manual mowing", false}}});
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
  sensors.push_back(Temperature("om_mow_motor_temp", "Mow Motor Temp", 0, 100, 80));
  sensors.push_back(Temperature("om_left_esc_temp", "Left ESC Temp", 0, 100, 80));
  sensors.push_back(Temperature("om_right_esc_temp", "Right ESC Temp", 0, 100, 80));
  // From the BMS. Own limits for Li-ion: warn above 55 degC.
  sensors.push_back(Temperature("battery_temp", "Battery Temp", -10, 70, 55));
  return sensors;
}

std::vector<SensorReading> LogicCore::Readings() const {
  std::vector<SensorReading> readings;
  if (mcu_.battery_ok) {
    const double charge_current = mcu_.battery_current < 0 ? -mcu_.battery_current : 0.0;
    readings.push_back({"om_v_battery", mcu_.battery_voltage, ""});
    readings.push_back({"om_v_charge", mcu_.battery_dock ? config_.dock_charge_voltage : 0.0, ""});
    readings.push_back({"om_charge_current", charge_current, ""});
    // Texts of the charger of OpenMower v2, the app's battery script reads them.
    std::string state = "Not Charging";
    if (mcu_.battery_dock && charge_current > 0.1) {
      state = "Fast Charge";
    } else if (mcu_.battery_dock && mcu_.battery_percentage >= 99) {
      state = "Done";
    }
    readings.push_back({"om_charge_state", 0, state});
    readings.push_back({"battery_temp", static_cast<double>(mcu_.battery_temperature), ""});
  }
  if (mcu_.cutter.ok) {
    readings.push_back({"om_mow_motor_rpm", static_cast<double>(mcu_.cutter.rpm), ""});
    readings.push_back({"om_mow_motor_current", mcu_.cutter.current, ""});
    readings.push_back({"om_mow_motor_temp", static_cast<double>(mcu_.cutter.temperature), ""});
  }
  if (mcu_.left.ok) readings.push_back({"om_left_esc_temp", static_cast<double>(mcu_.left.temperature), ""});
  if (mcu_.right.ok) readings.push_back({"om_right_esc_temp", static_cast<double>(mcu_.right.temperature), ""});
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

}  // namespace mowbite_logic
