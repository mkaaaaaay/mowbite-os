#include "mowbite_bridge/compat.h"

#include <map>
#include <utility>

#include "mowbite_api/catalog.h"

namespace mowbite_bridge {

namespace {

constexpr char kSensorsHistory[] = "sensors.history";
constexpr char kParamsSet[] = "params.set";
constexpr char kRecordingUndo[] = "recording.undo";
// The groups of OpenMower whose actions without a command the logic counts as a press it refuses.
constexpr const char* kPressGroups[] = {"mower_logic:idle/", "mower_logic:area_recording/", "mower_logic:undocking/",
                                        "mower_logic:docking/"};

const mowbite_api::SensorEntry* Shown(const std::string& own_id, bool openmower) {
  if (!openmower) return nullptr;
  const mowbite_api::SensorEntry* entry = mowbite_api::FindSensor(own_id);
  return entry && !entry->openmower_id.empty() ? entry : nullptr;
}

// A setting under the node `from` named under the node `to`, another name as it is.
std::string Renamed(const std::string& name, std::string_view from, std::string_view to) {
  if (name.size() <= from.size() || name.compare(0, from.size(), from) != 0 || name[from.size()] != '/') return name;
  return std::string(to) + name.substr(from.size());
}

Json RenamedSettings(const Json& settings, std::string_view from, std::string_view to) {
  Json out = Json::object();
  for (const auto& [name, value] : settings.items()) out[Renamed(name, from, to)] = value;
  return out;
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
  if (method == kParamsSet && params.is_object() && params.contains("params") && params["params"].is_object()) {
    params["params"] =
        RenamedSettings(params["params"], mowbite_api::kOpenMowerSettingsNode, mowbite_api::kSettingsNode);
    return params;
  }
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
  return openmower && (method == kSensorsHistory || method == kParamsSet);
}

Json ShownAnswer(const std::string& method, Json result) {
  if (method == kParamsSet) {
    if (!result.is_object()) return result;
    return RenamedSettings(result, mowbite_api::kSettingsNode, mowbite_api::kOpenMowerSettingsNode);
  }
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

std::string ShownActionsText(const std::map<std::string, std::vector<ActionItem>>& nodes, bool openmower) {
  if (!openmower) {
    std::map<std::string, std::vector<ActionItem>> own;
    for (const auto& node : nodes) own[""].insert(own[""].end(), node.second.begin(), node.second.end());
    return ActionsText(own);
  }
  // By the group of OpenMower, "<group>/<id>" there.
  std::map<std::string, std::vector<ActionItem>> groups;
  for (const auto& node : nodes) {
    for (const ActionItem& item : node.second) {
      const std::string action = mowbite_api::OpenMowerActionId(item.id);
      const size_t slash = action.rfind('/');
      if (slash == std::string::npos) continue;
      std::vector<ActionItem>& group = groups[action.substr(0, slash)];
      // Where OpenMower has them: before the undo of the recording.
      if (item.id == kRecordingUndo) {
        group.push_back({"start_manual_mowing", "Manual blade on", false});
        group.push_back({"stop_manual_mowing", "Manual blade off", false});
      }
      group.push_back({action.substr(slash + 1), item.name, item.enabled, item.reason});
    }
  }
  return ActionsText(groups);
}

std::optional<std::string> OwnAction(const std::string& sent, bool openmower) {
  if (mowbite_api::CommandOfAction(sent, nullptr)) return sent;
  const std::string own = mowbite_api::OwnActionId(sent);
  if (!own.empty()) return own;
  if (!openmower) return std::nullopt;
  for (const char* group : kPressGroups) {
    if (sent.rfind(group, 0) == 0) return sent;
  }
  return std::nullopt;
}

std::string ShownParams(std::string_view message, bool openmower) {
  if (!openmower) return std::string(message);
  Json value = Json::parse(message.begin(), message.end(), nullptr, false);
  if (!value.is_object() || value.value("node", Json()) != mowbite_api::kSettingsNode) return std::string(message);
  value["node"] = mowbite_api::kOpenMowerSettingsNode;
  return value.dump();
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
