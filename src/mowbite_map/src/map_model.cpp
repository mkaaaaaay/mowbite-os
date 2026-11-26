#include "mowbite_map/map_model.h"

#include <climits>
#include <cmath>
#include <set>
#include <stdexcept>

#include "mowbite_api/catalog.h"

namespace mowbite_map {

namespace {

constexpr size_t kMaxIdLength = 128;
// Two points closer than this in x and y count as the same, like the app does when it closes a ring.
constexpr double kSamePoint = 1e-9;

// The reason a map is refused, naming the place in the map.
class MapError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

const std::vector<std::string> kAreaTypes = {"mow", "nav", "obstacle", "decor", "draft"};

const json* Find(const json& object, const char* key) {
  auto it = object.find(key);
  if (it == object.end() || it->is_null()) return nullptr;
  return &*it;
}

double ToNumber(const json& value, const std::string& what) {
  if (!value.is_number() || !std::isfinite(value.get<double>())) throw MapError(what + " must be a finite number");
  return value.get<double>();
}

// As params.set takes /logic/mow_height_mm: a number, rounded and within the range of the setting.
int ToHeight(const json& value, const std::string& what) {
  const auto height = mowbite_api::FitSetting("mow_height_mm", ToNumber(value, what));
  if (!height) throw MapError(what + ": no such setting");
  return static_cast<int>(*height);
}

std::string ToText(const json& value, const std::string& what) {
  if (!value.is_string()) throw MapError(what + " must be a string");
  return value.get<std::string>();
}

bool ToBool(const json& value, const std::string& what) {
  if (!value.is_boolean()) throw MapError(what + " must be true or false");
  return value.get<bool>();
}

int ToCount(const json& value, const std::string& what) {
  if (!value.is_number_integer()) throw MapError(what + " must be a whole number");
  const long long n = value.get<long long>();
  if (n < 0 || n > INT_MAX) throw MapError(what + " must be 0 or more");
  return static_cast<int>(n);
}

Point ToPoint(const json& value, const std::string& what) {
  if (!value.is_object() || !value.contains("x") || !value.contains("y")) throw MapError(what + " needs x and y");
  return {ToNumber(value.at("x"), what + ".x"), ToNumber(value.at("y"), what + ".y")};
}

std::string ReadId(const json& object, const std::string& where) {
  const json* id = Find(object, "id");
  if (!id || !id->is_string() || id->get<std::string>().empty()) throw MapError(where + " needs an id");
  std::string text = id->get<std::string>();
  if (text.size() > kMaxIdLength) throw MapError(where + ": id is longer than " + std::to_string(kMaxIdLength));
  return text;
}

// Removes the key from the properties and returns its value; null counts as not set.
std::optional<json> Take(json& properties, const char* key) {
  auto it = properties.find(key);
  if (it == properties.end()) return std::nullopt;
  json value = std::move(*it);
  properties.erase(it);
  if (value.is_null()) return std::nullopt;
  return value;
}

json ReadProperties(const json& object, const std::string& where) {
  const json* properties = Find(object, "properties");
  if (!properties) return json::object();
  if (!properties->is_object()) throw MapError(where + ": properties must be an object");
  return *properties;
}

Area ParseArea(const json& j, size_t index) {
  std::string where = "areas[" + std::to_string(index) + "]";
  if (!j.is_object()) throw MapError(where + " must be an object");
  Area area;
  area.id = ReadId(j, where);
  where += " '" + area.id + "'";

  json properties = ReadProperties(j, where);
  if (auto v = Take(properties, "name")) area.name = ToText(*v, where + ": name");
  if (auto v = Take(properties, "type")) {
    area.type = ToText(*v, where + ": type");
    if (!IsAreaType(area.type)) throw MapError(where + ": unknown type '" + area.type + "'");
  }
  if (auto v = Take(properties, "active")) area.active = ToBool(*v, where + ": active");
  if (auto v = Take(properties, "mowable")) area.mowable = ToBool(*v, where + ": mowable");
  if (auto v = Take(properties, "angle")) area.angle = ToNumber(*v, where + ": angle");
  if (auto v = Take(properties, "outline_count")) area.outline_count = ToCount(*v, where + ": outline_count");
  if (auto v = Take(properties, "outline_overlap_count")) {
    area.outline_overlap_count = ToCount(*v, where + ": outline_overlap_count");
  }
  if (auto v = Take(properties, "outline_offset")) area.outline_offset = ToNumber(*v, where + ": outline_offset");
  if (auto v = Take(properties, "mow_height_mm")) area.mow_height_mm = ToHeight(*v, where + ": mow_height_mm");
  area.other_properties = std::move(properties);

  const json* outline = Find(j, "outline");
  if (!outline || !outline->is_array()) throw MapError(where + ": outline must be an array");
  for (size_t i = 0; i < outline->size(); ++i) {
    area.outline.push_back(ToPoint((*outline)[i], where + ": outline[" + std::to_string(i) + "]"));
  }
  const auto same = [](const Point& a, const Point& b) {
    return std::abs(a.x - b.x) < kSamePoint && std::abs(a.y - b.y) < kSamePoint;
  };
  if (area.outline.size() > 1 && same(area.outline.front(), area.outline.back())) area.outline.pop_back();
  if (area.outline.size() < 3) throw MapError(where + ": outline needs at least 3 points");
  area.outline.push_back(area.outline.front());
  return area;
}

DockingStation ParseDock(const json& j, size_t index) {
  std::string where = "docking_stations[" + std::to_string(index) + "]";
  if (!j.is_object()) throw MapError(where + " must be an object");
  DockingStation dock;
  dock.id = ReadId(j, where);
  where += " '" + dock.id + "'";

  json properties = ReadProperties(j, where);
  if (auto v = Take(properties, "name")) dock.name = ToText(*v, where + ": name");
  if (auto v = Take(properties, "active")) dock.active = ToBool(*v, where + ": active");
  dock.other_properties = std::move(properties);

  const json* position = Find(j, "position");
  if (!position) throw MapError(where + " needs a position");
  dock.position = ToPoint(*position, where + ": position");
  const json* heading = Find(j, "heading");
  if (!heading) throw MapError(where + " needs a heading");
  dock.heading = ToNumber(*heading, where + ": heading");
  return dock;
}

json PointJson(const Point& p) {
  return json{{"x", p.x}, {"y", p.y}};
}

json AreaJson(const Area& area) {
  json properties = json::object();
  if (!area.name.empty()) properties["name"] = area.name;
  properties["type"] = area.type;
  if (!area.active) properties["active"] = false;
  if (!area.mowable) properties["mowable"] = false;
  if (area.angle) properties["angle"] = *area.angle;
  if (area.outline_count) properties["outline_count"] = *area.outline_count;
  if (area.outline_overlap_count) properties["outline_overlap_count"] = *area.outline_overlap_count;
  if (area.outline_offset) properties["outline_offset"] = *area.outline_offset;
  if (area.mow_height_mm) properties["mow_height_mm"] = *area.mow_height_mm;
  properties.update(area.other_properties);
  json outline = json::array();
  for (const auto& p : area.outline) outline.push_back(PointJson(p));
  json j = json::object();
  j["id"] = area.id;
  j["properties"] = std::move(properties);
  j["outline"] = std::move(outline);
  return j;
}

json DockJson(const DockingStation& dock) {
  json properties = json::object();
  if (!dock.name.empty()) properties["name"] = dock.name;
  if (!dock.active) properties["active"] = false;
  properties.update(dock.other_properties);
  json j = json::object();
  j["id"] = dock.id;
  if (!properties.empty()) j["properties"] = std::move(properties);
  j["position"] = PointJson(dock.position);
  j["heading"] = dock.heading;
  return j;
}

}  // namespace

const std::vector<std::string>& AreaPropertyNames() {
  static const std::vector<std::string> names = {
      "name",          "type",           "active",       "mowable",     "angle", "outline_count",
      "outline_overlap_count", "outline_offset", "mow_height_mm"};
  return names;
}

const std::vector<std::string>& PlannerPropertyNames() {
  static const std::vector<std::string> names = {"mow_around", "angle_min", "angle_max"};
  return names;
}

bool IsAreaType(const std::string& type) {
  for (const auto& known : kAreaTypes) {
    if (known == type) return true;
  }
  return false;
}

ParseResult ParseMap(const json& j) {
  try {
    if (!j.is_object()) throw MapError("map must be an object");
    const json* areas = Find(j, "areas");
    if (!areas || !areas->is_array()) throw MapError("areas must be an array");
    MapData map;
    std::set<std::string> ids;
    for (size_t i = 0; i < areas->size(); ++i) {
      map.areas.push_back(ParseArea((*areas)[i], i));
      if (!ids.insert(map.areas.back().id).second) {
        throw MapError("areas[" + std::to_string(i) + "]: the id '" + map.areas.back().id + "' is used twice");
      }
    }
    if (const json* docks = Find(j, "docking_stations")) {
      if (!docks->is_array()) throw MapError("docking_stations must be an array");
      ids.clear();
      for (size_t i = 0; i < docks->size(); ++i) {
        map.docking_stations.push_back(ParseDock((*docks)[i], i));
        if (!ids.insert(map.docking_stations.back().id).second) {
          throw MapError("docking_stations[" + std::to_string(i) + "]: the id '" + map.docking_stations.back().id +
                         "' is used twice");
        }
      }
    }
    return {std::move(map), ""};
  } catch (const MapError& e) {
    return {std::nullopt, e.what()};
  }
}

json ToJson(const MapData& map) {
  json areas = json::array();
  for (const auto& area : map.areas) areas.push_back(AreaJson(area));
  json docks = json::array();
  for (const auto& dock : map.docking_stations) docks.push_back(DockJson(dock));
  json j = json::object();
  j["areas"] = std::move(areas);
  j["docking_stations"] = std::move(docks);
  return j;
}

std::string ToText(const MapData& map) {
  return ToJson(map).dump(2);
}

}  // namespace mowbite_map
