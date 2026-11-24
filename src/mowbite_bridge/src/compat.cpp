#include "mowbite_bridge/compat.h"

#include <utility>

#include "mowbite_api/catalog.h"

namespace mowbite_bridge {

namespace {

constexpr char kSensorsHistory[] = "sensors.history";

const mowbite_api::SensorEntry* Shown(const std::string& own_id, bool openmower) {
  if (!openmower) return nullptr;
  const mowbite_api::SensorEntry* entry = mowbite_api::FindSensor(own_id);
  return entry && !entry->openmower_id.empty() ? entry : nullptr;
}

}  // namespace

SensorDescription ShownSensor(SensorDescription own, bool openmower) {
  own.key = own.id;
  if (const auto* entry = Shown(own.id, openmower)) {
    own.id = entry->openmower_id;
    own.name = entry->openmower_name;
  }
  return own;
}

std::string ShownSensorId(const std::string& own_id, bool openmower) {
  const auto* entry = Shown(own_id, openmower);
  return entry ? entry->openmower_id : own_id;
}

Json ShownSensorValue(const std::string& own_id, Json value, bool openmower) {
  if (!openmower || own_id != "charge_state" || !value.is_string()) return value;
  const std::string& state = value.get_ref<const std::string&>();
  if (state == "idle") return "Not Charging";
  if (state == "charging") return "Fast Charge";
  if (state == "done") return "Done";
  return value;
}

std::string OwnSensorId(const std::string& id) {
  const mowbite_api::SensorEntry* entry = mowbite_api::FindOpenMowerSensor(id);
  return entry ? entry->id : id;
}

Json OwnParams(const std::string& method, Json params) {
  if (method != kSensorsHistory || !params.is_object() || !params.contains("sensor_ids") ||
      !params["sensor_ids"].is_array()) {
    return params;
  }
  for (auto& id : params["sensor_ids"]) {
    if (id.is_string()) id = OwnSensorId(id.get<std::string>());
  }
  return params;
}

bool ShowsAnswer(const std::string& method, bool openmower) {
  return openmower && method == kSensorsHistory;
}

Json ShownAnswer(Json result) {
  if (!result.is_object() || !result.contains("sensors") || !result["sensors"].is_array()) return result;
  for (auto& sensor : result["sensors"]) {
    if (!sensor.is_object() || !sensor.contains("sensor_id") || !sensor["sensor_id"].is_string()) continue;
    if (const auto* entry = Shown(sensor["sensor_id"].get<std::string>(), true)) {
      sensor["sensor_id"] = entry->openmower_id;
      sensor["name"] = entry->openmower_name;
    }
  }
  return result;
}

ReasonLine EmergencyReasonLine(uint32_t before, uint32_t after, bool openmower) {
  if (after == 0) return {false, "Emergency cleared"};
  const std::string from = mowbite_api::EmergencyNames(static_cast<int>(before));
  const std::string to = mowbite_api::EmergencyNames(static_cast<int>(after));
  if (openmower) return {true, "Emergency reason changed from: '" + from + "' to: '" + to + "'"};
  return {true, "Emergency reasons now '" + to + "', before '" + from + "'"};
}

std::string BoardParams() {
  return Json{{"node", "/ll"}, {"params", {{"board", "MowBite"}}}}.dump();
}

}  // namespace mowbite_bridge
