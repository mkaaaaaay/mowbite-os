// The sensors the logic shows (sensors/json): their definitions from the config and the readings of hw/status.
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

namespace {

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

}  // namespace mowbite_logic
