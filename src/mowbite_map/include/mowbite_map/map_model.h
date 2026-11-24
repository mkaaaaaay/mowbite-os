// The map in the JSON of OpenMower (map.json on the mower, map/json for the app) as typed data.
#pragma once

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace mowbite_map {

using json = nlohmann::ordered_json;

struct Point {
  double x = 0.0;
  double y = 0.0;
};

struct Area {
  std::string id;
  std::string name;
  // mow, nav, obstacle, decor or draft
  std::string type = "draft";
  bool active = true;
  // false: still drivable, but left out when mowing
  bool mowable = true;
  // closed: the last point repeats the first
  std::vector<Point> outline;
  // rad
  std::optional<double> angle;
  std::optional<int> outline_count;
  std::optional<int> outline_overlap_count;
  std::optional<double> outline_offset;
  // mm, the cutting height of the area instead of /mower_logic/mow_height_mm; within its range.
  std::optional<int> mow_height_mm;
  // Properties this version does not know (planner settings, pin, margin, ...), written back as they are.
  json other_properties = json::object();
};

struct DockingStation {
  std::string id;
  std::string name;
  bool active = true;
  Point position;
  // rad
  double heading = 0.0;
  json other_properties = json::object();
};

struct MapData {
  std::vector<Area> areas;
  std::vector<DockingStation> docking_stations;

  bool Empty() const {
    return areas.empty() && docking_stations.empty();
  }
};

struct ParseResult {
  std::optional<MapData> map;
  // Why the map is invalid, set when map is empty.
  std::string error;
};

// Reads the shape of map/json. Outlines may be open or closed, the result has them closed.
ParseResult ParseMap(const json& j);

// Writes the shape of map/json like OpenMower: active and mowable only when false, empty name left out.
json ToJson(const MapData& map);

// The text of map.json and map/json: indented by 2 like OpenMower writes it.
std::string ToText(const MapData& map);

// The area properties this version stores, the answer of map.area_properties.
const std::vector<std::string>& AreaPropertyNames();

// The area properties the MowBite Planner reads from the map itself. They are kept always, and reported by
// map.area_properties only while some node offers planner.settings.
const std::vector<std::string>& PlannerPropertyNames();

bool IsAreaType(const std::string& type);

}  // namespace mowbite_map
