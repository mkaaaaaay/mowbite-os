// The sensors the logic shows (sensors/json): their definitions from the config and the readings of hw/status.
#include <string>
#include <utility>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

namespace {

// The name to show is the one of the interface's catalog.
std::string NameOf(const std::string& id) {
  const mowbite_api::SensorEntry* entry = mowbite_api::FindSensor(id);
  return entry ? entry->name : id;
}

SensorDefinition Range(std::string id, std::string unit, SensorMeaning meaning, double min, double max) {
  SensorDefinition s;
  s.name = NameOf(id);
  s.id = std::move(id);
  s.unit = std::move(unit);
  s.meaning = meaning;
  s.min = min;
  s.max = max;
  return s;
}

SensorDefinition Temperature(std::string id, double min, double max, double critical_high) {
  SensorDefinition s = Range(std::move(id), "deg.C", SensorMeaning::kTemperature, min, max);
  s.critical_high = critical_high;
  return s;
}

}  // namespace

std::vector<SensorDefinition> LogicCore::Sensors() const {
  std::vector<SensorDefinition> sensors;
  SensorDefinition battery = Range("battery_voltage", "V", SensorMeaning::kVoltage, config_.battery_empty_voltage,
                                   config_.battery_full_voltage);
  battery.critical_low = config_.battery_empty_voltage;
  sensors.push_back(battery);
  // The MCU reports neither a charge voltage nor a charger state: both are derived, the names say so.
  sensors.push_back(
      Range("charge_voltage", "V", SensorMeaning::kVoltage, 0, config_.dock_charge_voltage + 2));
  sensors.push_back(Range("charge_current", "A", SensorMeaning::kCurrent, 0, 7));
  SensorDefinition charge_state;
  charge_state.id = "charge_state";
  charge_state.name = NameOf(charge_state.id);
  charge_state.text = true;
  sensors.push_back(charge_state);
  sensors.push_back(Range("blade_rpm", "rpm", SensorMeaning::kRevolutions, 0, 3500));
  sensors.push_back(Range("blade_current", "A", SensorMeaning::kCurrent, 0, 5));
  // One temperature per motor board, so no separate temperature of the blade's board. Own limits: the drive motors
  // hold position up to 68 degC, warn above 80.
  SensorDefinition accuracy;
  accuracy.id = "position_accuracy";
  accuracy.name = NameOf(accuracy.id);
  accuracy.unit = "m";
  accuracy.meaning = SensorMeaning::kDistance;
  sensors.push_back(accuracy);
  sensors.push_back(Temperature("blade_temp", 0, 100, 80));
  sensors.push_back(Temperature("drive_left_temp", 0, 100, 80));
  sensors.push_back(Temperature("drive_right_temp", 0, 100, 80));
  // From the BMS. Own limits for Li-ion: warn above 55 degC.
  sensors.push_back(Temperature("battery_temp", -10, 70, 55));
  return sensors;
}

std::vector<SensorReading> LogicCore::Readings() const {
  std::vector<SensorReading> readings;
  if (hw_.battery_ok) {
    const double charge_current = hw_.battery_current < 0 ? -hw_.battery_current : 0.0;
    readings.push_back({"battery_voltage", hw_.battery_voltage, ""});
    readings.push_back({"charge_voltage", hw_.dock_contact ? config_.dock_charge_voltage : 0.0, ""});
    readings.push_back({"charge_current", charge_current, ""});
    std::string state = "idle";
    if (hw_.dock_contact && charge_current > 0.1) {
      state = "charging";
    } else if (hw_.dock_contact && hw_.battery_percentage >= 99) {
      state = "done";
    }
    readings.push_back({"charge_state", 0, state});
    readings.push_back({"battery_temp", static_cast<double>(hw_.battery_temperature), ""});
  }
  if (pose_) readings.push_back({"position_accuracy", pose_->position_accuracy, ""});
  if (hw_.blade.ok) {
    readings.push_back({"blade_rpm", static_cast<double>(hw_.blade.rpm), ""});
    readings.push_back({"blade_current", hw_.blade.current, ""});
    readings.push_back({"blade_temp", static_cast<double>(hw_.blade.temperature), ""});
  }
  if (hw_.left.ok) readings.push_back({"drive_left_temp", static_cast<double>(hw_.left.temperature), ""});
  if (hw_.right.ok) readings.push_back({"drive_right_temp", static_cast<double>(hw_.right.temperature), ""});
  return readings;
}

}  // namespace mowbite_logic
