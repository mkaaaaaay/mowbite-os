#include "mowbite_bridge/app_json.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "mowbite_api/catalog.h"

namespace mowbite_bridge {

namespace {

constexpr std::size_t kMaxTeleopBytes = 1024;

std::string Dump(const Json& j) {
  return j.dump(-1, ' ', false, Json::error_handler_t::replace);
}

int Switch(bool on) {
  return on ? 1 : 0;
}

}  // namespace

std::optional<Teleop> ReadTeleop(std::string_view payload) {
  if (payload.empty() || payload.size() > kMaxTeleopBytes) return std::nullopt;
  const Json doc = Json::from_bson(payload.begin(), payload.end(), true, false);
  if (doc.is_discarded() || !doc.is_object()) return std::nullopt;
  const auto vx = doc.find("vx");
  const auto vz = doc.find("vz");
  if (vx == doc.end() || vz == doc.end() || !vx->is_number() || !vz->is_number()) return std::nullopt;
  const Teleop teleop{vx->get<double>(), vz->get<double>()};
  if (!std::isfinite(teleop.vx) || !std::isfinite(teleop.vz)) return std::nullopt;
  return teleop;
}

std::string RobotStateText(const RobotStateView& s) {
  return Dump(Json{{"battery_percentage", s.battery_percentage},
                   {"gps_percentage", s.gps_percentage},
                   {"current_action_progress", 0.0},
                   {"current_state", s.current_state},
                   {"current_sub_state", s.current_sub_state},
                   {"current_area", s.current_area},
                   {"current_path", s.current_path},
                   {"current_path_index", s.current_path_index},
                   {"task_index", s.task_index},
                   {"task_count", s.task_count},
                   {"pause_reason", s.pause_reason},
                   {"pause_kind", s.pause_kind},
                   {"emergency", Switch(s.emergency)},
                   {"emergency_reason", s.emergency_reason},
                   {"is_charging", Switch(s.is_charging)},
                   {"rain_detected", Switch(s.rain_detected)},
                   {"hardware", Switch(s.hardware)},
                   {"pose",
                    {{"x", s.x},
                     {"y", s.y},
                     {"heading", s.heading},
                     {"pos_accuracy", s.pos_accuracy},
                     {"heading_accuracy", s.heading_accuracy},
                     {"heading_valid", Switch(s.heading_valid)}}}});
}

std::string PositionText(double x, double y, double heading, const PoseAttributes& a) {
  return Dump(Json{{"x", x},
                   {"y", y},
                   {"heading", heading},
                   {"attributes", {{"job_id", a.job_id}, {"session_id", a.session_id}, {"blades", a.blades}}}});
}

const char* MeaningName(Meaning meaning) {
  switch (meaning) {
    case Meaning::kVoltage:
      return "VOLTAGE";
    case Meaning::kCurrent:
      return "CURRENT";
    case Meaning::kTemperature:
      return "TEMPERATURE";
    case Meaning::kRevolutions:
      return "REVOLUTIONS";
    case Meaning::kUnknown:
      break;
  }
  return "UNKNOWN";
}

Json SensorInfo(const SensorDescription& s) {
  const bool range = s.min && s.max;
  return Json{{"sensor_id", s.id},
              {"sensor_name", s.name},
              {"value_type", s.is_text ? "STRING" : "DOUBLE"},
              {"value_description", MeaningName(s.meaning)},
              {"unit", s.unit},
              {"has_min_max", Switch(range)},
              {"min_value", range ? *s.min : 0.0},
              {"max_value", range ? *s.max : 0.0},
              {"has_critical_low", Switch(s.critical_low.has_value())},
              {"lower_critical_value", s.critical_low.value_or(0.0)},
              {"has_critical_high", Switch(s.critical_high.has_value())},
              {"upper_critical_value", s.critical_high.value_or(0.0)},
              {"key", s.key.empty() ? s.id : s.key}};
}

std::string SensorInfosText(const std::map<std::string, SensorDescription>& sensors) {
  Json all = Json::array();
  for (const auto& [id, sensor] : sensors) all.push_back(SensorInfo(sensor));
  return Dump(all);
}

std::string SensorNumberText(double value) {
  char text[64];
  std::snprintf(text, sizeof(text), "%f", value);
  return text;
}

Json SensorList(const std::map<std::string, SensorDescription>& sensors,
                const std::map<std::string, SensorReading>& readings, const Json& params) {
  const auto entry = [&](const SensorDescription& sensor) {
    Json j = SensorInfo(sensor);
    const auto reading = readings.find(sensor.id);
    j["value"] = reading == readings.end() ? Json() : reading->second.value;
    j["stamp"] = reading == readings.end() ? Json() : Json(reading->second.stamp);
    return j;
  };
  Json out = Json::array();
  const bool all = params.is_null() || (params.is_array() && params.empty()) ||
                   (params.is_object() && !params.contains("ids"));
  if (all) {
    for (const auto& [id, sensor] : sensors) out.push_back(entry(sensor));
    return out;
  }
  const auto ids = params.is_object() ? params.find("ids") : params.end();
  if (ids == params.end() || !ids->is_array()) throw std::invalid_argument("ids must be a list of sensor ids");
  for (const auto& id : *ids) {
    if (!id.is_string()) throw std::invalid_argument("ids must be a list of sensor ids");
  }
  for (const auto& id : *ids) {
    const std::string& wanted = id.get_ref<const std::string&>();
    auto sensor = sensors.find(wanted);
    if (sensor == sensors.end()) {
      sensor = std::find_if(sensors.begin(), sensors.end(), [&](const auto& s) { return s.second.key == wanted; });
    }
    if (sensor != sensors.end()) out.push_back(entry(sensor->second));
  }
  return out;
}

std::string ActionsText(const std::map<std::string, std::vector<ActionItem>>& groups) {
  Json all = Json::array();
  for (const auto& [prefix, items] : groups) {
    for (const auto& item : items) {
      Json entry{{"action_id", prefix + "/" + item.id}, {"action_name", item.name}, {"enabled", Switch(item.enabled)}};
      if (!item.enabled && !item.reason.empty()) entry["reason"] = item.reason;
      all.push_back(std::move(entry));
    }
  }
  return Dump(all);
}

std::string CapabilitiesText() {
  return Json{{"rpc", 1}, {"map:json", 1}, {"events", 1}}.dump(2);
}

std::string VersionText(const std::string& version) {
  return Dump(Json{{"version", version.empty() ? "UNKNOWN VERSION" : version}});
}

Json SystemInfo(const std::string& version, const std::string& platform, int mqtt_version, const Json& stop_release,
                const Json& blade_height, const std::string& update_path, bool openmower_names) {
  return Json{{"name", "mowbite-os"},
              {"version", version.empty() ? "UNKNOWN VERSION" : version},
              {"platform", platform},
              {"schema", DataSchema()},
              {"api", mowbite_api::kApi},
              {"api_break", mowbite_api::kApiBreak},
              {"mqtt", mqtt_version},
              {"stop_release", stop_release},
              {"blade_height", blade_height},
              {"update_path", update_path.empty() ? Json() : Json(update_path)},
              {"names", openmower_names ? "openmower" : "own"}};
}

Json StopRelease(double free_s, double hold_s, double max_hold_s) {
  if (!(free_s > 0)) return Json();
  return Json{{"free_s", free_s}, {"hold_s", hold_s}, {"max_hold_s", max_hold_s}};
}

Json BladeHeight(bool control, int min_mm, int max_mm, int default_mm) {
  if (!control) return Json();
  return Json{{"min_mm", min_mm}, {"max_mm", max_mm}, {"default_mm", default_mm}};
}

int DataSchema() {
  return MOWBITE_DATA_SCHEMA;
}

}  // namespace mowbite_bridge
