#include "mowbite_safety/region.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace mowbite_safety {

namespace {

using json = nlohmann::json;

// A closed ring repeats its first point; the checks work on the open one.
size_t OpenSize(const Ring& ring) {
  size_t n = ring.size();
  if (n > 1 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) n--;
  return n;
}

struct BadMap : std::runtime_error {
  using std::runtime_error::runtime_error;
};

Point2 ToPoint(const json& p, const std::string& where) {
  if (!p.is_object() || !p.contains("x") || !p.contains("y") || !p["x"].is_number() || !p["y"].is_number()) {
    throw BadMap(where + ": a point without numeric x and y");
  }
  const Point2 point{p["x"].get<double>(), p["y"].get<double>()};
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) throw BadMap(where + ": a point out of range");
  return point;
}

const json& Properties(const json& entry, const std::string& where) {
  static const json kNone = json::object();
  if (!entry.contains("properties")) return kNone;
  if (!entry["properties"].is_object()) throw BadMap(where + ": properties are not an object");
  return entry["properties"];
}

bool Active(const json& properties, const std::string& where) {
  if (!properties.contains("active")) return true;
  if (!properties["active"].is_boolean()) throw BadMap(where + ": active is not true or false");
  return properties["active"].get<bool>();
}

}  // namespace

bool InsideRing(const Ring& ring, double x, double y) {
  const size_t n = OpenSize(ring);
  bool inside = false;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const Point2& a = ring[i];
    const Point2& b = ring[j];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
}

double DistanceToRing(const Ring& ring, double x, double y) {
  const size_t n = OpenSize(ring);
  double nearest = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < n; i++) {
    const Point2& a = ring[i];
    const Point2& b = ring[(i + 1) % n];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double t = length2 > 0 ? std::clamp(((x - a.x) * dx + (y - a.y) * dy) / length2, 0.0, 1.0) : 0.0;
    nearest = std::min(nearest, std::hypot(a.x + t * dx - x, a.y + t * dy - y));
  }
  return nearest;
}

std::optional<Region> ParseRegion(const std::string& map_json, std::string* error) {
  const json j = json::parse(map_json, nullptr, false);
  if (j.is_discarded() || !j.is_object() || !j.contains("areas") || !j["areas"].is_array()) {
    if (error) *error = "map/json is not a map";
    return std::nullopt;
  }
  Region region;
  try {
    for (size_t i = 0; i < j["areas"].size(); i++) {
      const json& entry = j["areas"][i];
      const std::string where = "area " + std::to_string(i);
      if (!entry.is_object()) throw BadMap(where + ": not an object");
      const json& properties = Properties(entry, where);
      if (properties.contains("type") && !properties["type"].is_string()) throw BadMap(where + ": type is not a text");
      const std::string type = properties.value("type", "");
      // decor and draft areas are neither allowed nor obstacles.
      if (!Active(properties, where) || (type != "mow" && type != "nav" && type != "obstacle")) continue;
      if (!entry.contains("outline") || !entry["outline"].is_array()) throw BadMap(where + ": no outline");
      Ring ring;
      for (const auto& p : entry["outline"]) ring.push_back(ToPoint(p, where));
      if (OpenSize(ring) < 3) throw BadMap(where + ": fewer than 3 points");
      (type == "obstacle" ? region.obstacles : region.allowed).push_back(std::move(ring));
    }
    if (j.contains("docking_stations")) {
      if (!j["docking_stations"].is_array()) throw BadMap("docking_stations is not a list");
      for (size_t i = 0; i < j["docking_stations"].size(); i++) {
        const json& entry = j["docking_stations"][i];
        const std::string where = "docking station " + std::to_string(i);
        if (!entry.is_object() || !entry.contains("position") || !entry.contains("heading") ||
            !entry["heading"].is_number()) {
          throw BadMap(where + ": no position and heading");
        }
        if (!Active(Properties(entry, where), where)) continue;
        const Point2 position = ToPoint(entry["position"], where);
        const double heading = entry["heading"].get<double>();
        if (!std::isfinite(heading)) throw BadMap(where + ": heading out of range");
        // Like the logic, the first active station counts.
        if (!region.station) region.station = Station{position.x, position.y, heading};
      }
    }
  } catch (const BadMap& e) {
    if (error) *error = std::string("map/json: ") + e.what();
    return std::nullopt;
  } catch (const json::exception& e) {
    if (error) *error = std::string("map/json: ") + e.what();
    return std::nullopt;
  }
  if (error) error->clear();
  return region;
}

Ring StationZone(const Station& station, double behind, double ahead, double half_width) {
  // The mower backs in along the heading and leaves against it, so the line runs from the station against the heading.
  const double c = std::cos(station.heading);
  const double s = std::sin(station.heading);
  const double near_x = station.x + behind * c;
  const double near_y = station.y + behind * s;
  const double far_x = station.x - ahead * c;
  const double far_y = station.y - ahead * s;
  const double side_x = -s * half_width;
  const double side_y = c * half_width;
  return {{near_x + side_x, near_y + side_y},
          {near_x - side_x, near_y - side_y},
          {far_x - side_x, far_y - side_y},
          {far_x + side_x, far_y + side_y}};
}

}  // namespace mowbite_safety
