#include "mowbite_logic/geometry.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <stdexcept>

#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"

namespace mowbite_logic {

const std::vector<GeometryKey>& GeometryKeys() {
  // The keys and ranges are the interface's (x-geometry).
  static const std::vector<GeometryKey> keys = [] {
    std::vector<GeometryKey> out;
    for (const auto& key : mowbite_api::GeometryKeys()) {
      const GeometryType type =
          key.shape ? GeometryType::kShape : key.points ? GeometryType::kPoints : GeometryType::kNumber;
      out.push_back({key.name, type, key.settable, key.min, key.max, key.unit.c_str(), key.relative});
    }
    return out;
  }();
  return keys;
}

namespace {

constexpr double kEps = 1e-9;
// The body is 0.1 to 1.5 m long and wide, every point of an outline at most 1.5 m from base_link.
constexpr double kBodyMin = 0.1;
constexpr double kBodyMax = 1.5;

std::string Num(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.4g", value);
  return text;
}

const GeometryKey* FindKey(const std::string& name) {
  for (const auto& key : GeometryKeys()) {
    if (key.name == name) return &key;
  }
  return nullptr;
}

GeometryProblem Problem(const std::string& reason, const std::string& key, const std::string& message) {
  GeometryProblem problem;
  problem.reason = reason;
  problem.key = key;
  problem.message = message;
  return problem;
}

// [[x, y], ...] or nothing, why says what is wrong.
std::optional<std::vector<Point>> ParsePoints(const json& value, std::string* why,
                                              const std::string& key = "robot_outline") {
  std::vector<Point> points;
  if (!value.is_array()) {
    *why = key + " must be a list of [x, y] pairs";
    return std::nullopt;
  }
  for (const auto& item : value) {
    if (!item.is_array() || item.size() != 2 || !item[0].is_number() || !item[1].is_number()) {
      *why = key + " must be a list of [x, y] pairs";
      return std::nullopt;
    }
    points.push_back({item[0].get<double>(), item[1].get<double>()});
  }
  return points;
}

// The box round the body: x from x_min to x_max, y within plus and minus y_max.
struct Body {
  double x_min;
  double x_max;
  double y_max;
};

Body BodyOf(const std::map<std::string, double>& numbers, const std::vector<Point>& outline) {
  if (outline.empty()) {
    return {-numbers.at("robot_rear"), numbers.at("robot_front"), numbers.at("robot_width") / 2};
  }
  Body body{outline[0][0], outline[0][0], 0.0};
  for (const Point& point : outline) {
    body.x_min = std::min(body.x_min, point[0]);
    body.x_max = std::max(body.x_max, point[0]);
    body.y_max = std::max(body.y_max, std::abs(point[1]));
  }
  return body;
}

// An axis-parallel box from above: the left drive wheel, the right one mirrored at y 0.
struct Box {
  double x_min;
  double x_max;
  double y_min;
  double y_max;
};

double DistanceToSegment(const Point& a, const Point& b, double x, double y) {
  const double dx = b[0] - a[0], dy = b[1] - a[1];
  const double length = dx * dx + dy * dy;
  const double t = length > 0 ? std::clamp(((x - a[0]) * dx + (y - a[1]) * dy) / length, 0.0, 1.0) : 0.0;
  return std::hypot(a[0] + t * dx - x, a[1] + t * dy - y);
}

// Inside the polygon or on its edge.
bool Inside(const std::vector<Point>& polygon, double x, double y) {
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const Point& a = polygon[j];
    const Point& b = polygon[i];
    if (DistanceToSegment(a, b, x, y) <= kEps) return true;
    if ((a[1] > y) != (b[1] > y) && x < a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1])) inside = !inside;
  }
  return inside;
}

// The segment passes through the inside of the box, not only along its edge.
bool Enters(const Point& a, const Point& b, const Box& box) {
  const double dx = b[0] - a[0], dy = b[1] - a[1];
  const double p[] = {-dx, dx, -dy, dy};
  const double q[] = {a[0] - box.x_min - kEps, box.x_max - kEps - a[0], a[1] - box.y_min - kEps,
                      box.y_max - kEps - a[1]};
  double t0 = 0.0, t1 = 1.0;
  for (int i = 0; i < 4; ++i) {
    if (p[i] == 0.0) {
      if (q[i] < 0.0) return false;
    } else if (p[i] < 0.0) {
      t0 = std::max(t0, q[i] / p[i]);
    } else {
      t1 = std::min(t1, q[i] / p[i]);
    }
  }
  return t0 < t1;
}

// The box lies within the outline: its corners inside and no edge of the outline through it.
bool Encloses(const std::vector<Point>& outline, const Box& box) {
  for (const double x : {box.x_min, box.x_max}) {
    for (const double y : {box.y_min, box.y_max}) {
      if (!Inside(outline, x, y)) return false;
    }
  }
  for (size_t i = 0, j = outline.size() - 1; i < outline.size(); j = i++) {
    if (Enters(outline[j], outline[i], box)) return false;
  }
  return true;
}

// The key of the body that leaves out the drive wheels, if one does.
std::optional<std::string> WheelsOutside(const std::map<std::string, double>& numbers,
                                         const std::vector<Point>& outline) {
  const double radius = numbers.at("wheel_radius");
  const double half_track = numbers.at("track") / 2;
  const double half_width = numbers.at("wheel_width") / 2;
  const Box left{-radius, radius, half_track - half_width, half_track + half_width};
  const Box right{-radius, radius, -left.y_max, -left.y_min};
  if (!outline.empty()) {
    if (Encloses(outline, left) && Encloses(outline, right)) return std::nullopt;
    return "robot_outline";
  }
  if (radius > numbers.at("robot_rear") + kEps) return "robot_rear";
  if (radius > numbers.at("robot_front") + kEps) return "robot_front";
  if (left.y_max > numbers.at("robot_width") / 2 + kEps) return "robot_width";
  return std::nullopt;
}

// The body from above: the outline, without one the rectangle of width, front and rear.
std::vector<Point> BodyPolygon(const std::map<std::string, double>& numbers, const std::vector<Point>& outline) {
  if (!outline.empty()) return outline;
  const Body body = BodyOf(numbers, outline);
  return {{body.x_max, body.y_max}, {body.x_min, body.y_max}, {body.x_min, -body.y_max}, {body.x_max, -body.y_max}};
}

// [{"id", "kind", "points": [[x, y], ...], "closed"}] within the limits of the interface, or throws naming the problem.
std::vector<ShapePart> ParseShape(const json& value) {
  const auto fail = [](const std::string& what) -> std::vector<ShapePart> {
    throw std::invalid_argument("robot_shape: " + what);
  };
  if (!value.is_array()) return fail("it must be a list of parts");
  if (value.size() > static_cast<size_t>(mowbite_api::kShapeMaxParts)) {
    return fail("more than " + std::to_string(mowbite_api::kShapeMaxParts) + " parts");
  }
  std::vector<ShapePart> shape;
  std::set<std::string> ids;
  size_t total = 0;
  for (const auto& item : value) {
    if (!item.is_object() || !item.contains("id") || !item["id"].is_string() || !item.contains("kind") ||
        !item["kind"].is_string() || !item.contains("points")) {
      return fail("a part needs id, kind and points");
    }
    ShapePart part;
    part.id = item["id"].get<std::string>();
    part.kind = item["kind"].get<std::string>();
    if (part.id.empty() || part.kind.empty()) return fail("a part has an empty id or kind");
    if (!ids.insert(part.id).second) return fail("the part " + part.id + " comes twice");
    std::string why;
    auto points = ParsePoints(item["points"], &why, "robot_shape." + part.id);
    if (!points) return fail(why);
    if (points->empty()) return fail("the part " + part.id + " has no point");
    if (points->size() > static_cast<size_t>(mowbite_api::kShapeMaxPartPoints)) {
      return fail("the part " + part.id + " has more than " + std::to_string(mowbite_api::kShapeMaxPartPoints) +
                  " points");
    }
    for (const Point& point : *points) {
      if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || std::hypot(point[0], point[1]) > kBodyMax + kEps) {
        return fail("a point of the part " + part.id + " is farther than " + Num(kBodyMax) + " m from base_link");
      }
    }
    total += points->size();
    part.points = std::move(*points);
    if (item.contains("closed")) {
      if (!item["closed"].is_boolean()) return fail("closed of the part " + part.id + " is no boolean");
      part.closed = item["closed"].get<bool>();
    }
    shape.push_back(std::move(part));
  }
  if (total > static_cast<size_t>(mowbite_api::kShapeMaxPoints)) {
    return fail("more than " + std::to_string(mowbite_api::kShapeMaxPoints) + " points");
  }
  return shape;
}

GeometryProblem OutOfRange(const std::string& key, const std::string& message, double min, double max) {
  GeometryProblem problem = Problem("OUT_OF_RANGE", key, message);
  problem.data["min"] = min;
  problem.data["max"] = max;
  return problem;
}

GeometryProblem OutsideBody(const std::string& key, double value, double min, double max) {
  GeometryProblem problem =
      Problem("OUTSIDE_BODY", key,
              key + " " + Num(value) + " m is outside of the body (" + Num(min) + " to " + Num(max) + " m)");
  problem.data["min"] = min;
  problem.data["max"] = max;
  return problem;
}

}  // namespace

bool Geometry::Entries::operator==(const Entries& other) const {
  return numbers == other.numbers && outline == other.outline;
}

Geometry::Geometry(std::string platform, const json& factory) : platform_(std::move(platform)) {
  if (!factory.is_object()) throw std::invalid_argument("The geometry of the platform is no object");
  for (const auto& key : GeometryKeys()) {
    if (key.type == GeometryType::kNumber) {
      if (!factory.contains(key.name) || !factory[key.name].is_number()) {
        throw std::invalid_argument(key.name + " is missing in the geometry of the platform, or no number");
      }
      factory_.numbers[key.name] = factory[key.name].get<double>();
    } else if (key.type == GeometryType::kShape) {
      if (factory.contains(key.name)) shape_ = ParseShape(factory[key.name]);
    } else if (factory.contains(key.name)) {
      std::string why;
      auto points = ParsePoints(factory[key.name], &why, key.name);
      if (!points) throw std::invalid_argument(why);
      (key.name == "dock_footprint" ? factory_.dock_footprint : factory_.outline) = std::move(*points);
    }
  }
  if (auto problem = Check(factory_)) {
    throw std::invalid_argument("The geometry of the platform: " + problem->message);
  }
  now_ = factory_;
}

Geometry::Values Geometry::Effective(const Entries& entries) const {
  Values values = factory_;
  for (const auto& [name, value] : entries.numbers) values.numbers[name] = value;
  if (entries.outline) values.outline = *entries.outline;
  return values;
}

const std::vector<Point>& Geometry::PointsOf(const Values& values, const std::string& key) {
  return key == "dock_footprint" ? values.dock_footprint : values.outline;
}

std::pair<double, double> Geometry::Range(const GeometryKey& key) const {
  if (!key.relative) return {key.min, key.max};
  const double base = factory_.numbers.at(key.name);
  return {base + key.min, base + key.max};
}

std::optional<GeometryProblem> Geometry::Check(const Values& values) const {
  for (const auto& key : GeometryKeys()) {
    if (key.type != GeometryType::kNumber) continue;
    const double x = values.numbers.at(key.name);
    const auto [min, max] = Range(key);
    if (!std::isfinite(x) || x < min - kEps || x > max + kEps) {
      const std::string unit = std::string(" ") + key.unit;
      return OutOfRange(key.name,
                        key.name + " " + Num(x) + unit + " is outside " + Num(min) + " to " + Num(max) + unit, min,
                        max);
    }
  }
  if (!values.outline.empty()) {
    if (values.outline.size() < 3)
      return Problem("INVALID_VALUE", "robot_outline", "robot_outline needs at least 3 points");
    for (const Point& point : values.outline) {
      if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || std::abs(point[0]) > kBodyMax + kEps ||
          std::abs(point[1]) > kBodyMax + kEps) {
        return OutOfRange("robot_outline",
                          "A point of robot_outline is farther than " + Num(kBodyMax) + " m from base_link", -kBodyMax,
                          kBodyMax);
      }
    }
  }
  if (!values.dock_footprint.empty()) {
    if (values.dock_footprint.size() < 3)
      return Problem("INVALID_VALUE", "dock_footprint", "dock_footprint needs at least 3 points or none");
    for (const Point& point : values.dock_footprint) {
      if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || std::hypot(point[0], point[1]) > kBodyMax + kEps) {
        return OutOfRange("dock_footprint",
                          "A point of dock_footprint is farther than " + Num(kBodyMax) + " m from base_link",
                          -kBodyMax, kBodyMax);
      }
    }
  }
  const Body body = BodyOf(values.numbers, values.outline);
  const double length = body.x_max - body.x_min;
  const double width = 2 * body.y_max;
  const std::string length_key = values.outline.empty() ? "robot_front" : "robot_outline";
  const std::string width_key = values.outline.empty() ? "robot_width" : "robot_outline";
  if (length < kBodyMin - kEps || length > kBodyMax + kEps) {
    return OutOfRange(
        length_key,
        "The body is " + Num(length) + " m long: " + Num(kBodyMin) + " to " + Num(kBodyMax) + " m are possible",
        kBodyMin, kBodyMax);
  }
  if (width < kBodyMin - kEps || width > kBodyMax + kEps) {
    return OutOfRange(
        width_key,
        "The body is " + Num(width) + " m wide: " + Num(kBodyMin) + " to " + Num(kBodyMax) + " m are possible",
        kBodyMin, kBodyMax);
  }
  // The blade's centre and the antenna sit on the body, the cut is not wider than it.
  const struct {
    const char* key;
    double min;
    double max;
  } spots[] = {{"blade_ahead", body.x_min, body.x_max},
               {"blade_offset", -body.y_max, body.y_max},
               {"antenna_x", body.x_min, body.x_max},
               {"antenna_y", -body.y_max, body.y_max}};
  for (const auto& spot : spots) {
    const double x = values.numbers.at(spot.key);
    if (x < spot.min - kEps || x > spot.max + kEps) return OutsideBody(spot.key, x, spot.min, spot.max);
  }
  if (values.numbers.at("mower_width") > width + kEps) {
    return OutsideBody("mower_width", values.numbers.at("mower_width"), 0.0, width);
  }
  // The drive wheels from above: x within wheel_radius of the axle, track / 2 plus and minus wheel_width / 2 aside.
  if (auto key = WheelsOutside(values.numbers, values.outline)) {
    const double reach = (values.numbers.at("track") + values.numbers.at("wheel_width")) / 2;
    GeometryProblem problem =
        Problem("WHEELS_OUTSIDE", *key,
                "The body leaves out the drive wheels: they reach " + Num(values.numbers.at("wheel_radius")) +
                    " m ahead of and behind the axle and " + Num(reach) + " m to each side");
    problem.data["wheel_radius"] = values.numbers.at("wheel_radius");
    problem.data["wheel_width"] = values.numbers.at("wheel_width");
    problem.data["track"] = values.numbers.at("track");
    return problem;
  }
  return std::nullopt;
}

std::vector<std::string> Geometry::Below(const Values& factory, const Values& now) {
  std::vector<std::string> keys;
  for (const char* name : {"robot_width", "robot_front", "robot_rear"}) {
    if (now.numbers.at(name) < factory.numbers.at(name) - kEps) keys.push_back(name);
  }
  if (!now.outline.empty()) {
    const Body theirs = BodyOf(factory.numbers, factory.outline);
    const Body ours = BodyOf(now.numbers, now.outline);
    if (ours.x_max < theirs.x_max - kEps || ours.x_min > theirs.x_min + kEps || ours.y_max < theirs.y_max - kEps) {
      keys.push_back("robot_outline");
    }
  }
  return keys;
}

std::vector<std::string> Geometry::BelowDefault() const {
  return Below(factory_, now_);
}

std::vector<Point> Geometry::BodyOutline() const {
  if (!now_.outline.empty()) return now_.outline;
  const Body body = BodyOf(now_.numbers, now_.outline);
  return {{body.x_max, body.y_max}, {body.x_min, body.y_max}, {body.x_min, -body.y_max}, {body.x_max, -body.y_max}};
}

double Geometry::TurningRadius() const {
  double radius = 0;
  for (const Point& point : BodyOutline()) radius = std::max(radius, std::hypot(point[0], point[1]));
  return radius;
}

json Geometry::RecordedBody() const {
  return {{"robot_width", now_.numbers.at("robot_width")},
          {"robot_front", now_.numbers.at("robot_front")},
          {"robot_rear", now_.numbers.at("robot_rear")},
          {"robot_outline", PointsJson(now_.outline)}};
}

json Geometry::PointsJson(const std::vector<Point>& points) {
  json out = json::array();
  for (const Point& point : points) out.push_back(json::array({point[0], point[1]}));
  return out;
}

double Geometry::Value(const std::string& key) const {
  return now_.numbers.at(key);
}

bool Geometry::Load(const std::string& text, std::string* why) {
  const auto fail = [why](const std::string& message) {
    if (why) *why = message;
    return false;
  };
  const json parsed = json::parse(text, nullptr, false);
  if (!parsed.is_object()) return fail("it is no JSON object");
  int version = 0;
  if (parsed.contains("version")) {
    const json& number = parsed["version"];
    if (!number.is_number_integer() || number.get<int64_t>() < 0 || number.get<int64_t>() > 1000000000) {
      return fail("version is no whole number");
    }
    version = static_cast<int>(number.get<int64_t>());
  }
  Entries entries;
  if (parsed.contains("entries")) {
    if (!parsed["entries"].is_object()) return fail("entries is no object");
    for (const auto& item : parsed["entries"].items()) {
      const GeometryKey* key = FindKey(item.key());
      if (!key) return fail(item.key() + " is no key of the geometry");
      if (!key->settable) return fail(item.key() + " is the platform's alone");
      if (key->type == GeometryType::kNumber) {
        if (!item.value().is_number()) return fail(item.key() + " must be a number");
        entries.numbers[item.key()] = item.value().get<double>();
      } else {
        std::string problem;
        auto points = ParsePoints(item.value(), &problem);
        if (!points) return fail(problem);
        entries.outline = std::move(*points);
      }
    }
  }
  std::map<std::string, double> confirmed;
  if (parsed.contains("confirmed")) {
    if (!parsed["confirmed"].is_object()) return fail("confirmed is no object");
    for (const auto& item : parsed["confirmed"].items()) {
      const GeometryKey* key = FindKey(item.key());
      if (!key || !key->settable) return fail(item.key() + " is no key the user confirms");
      if (!item.value().is_number()) return fail("confirmed." + item.key() + " must be a number");
      confirmed[item.key()] = item.value().get<double>();
    }
  }
  json applied = parsed.value("calibration_applied", json());
  if (!applied.is_object() || !applied.value("result_id", json()).is_string()) applied = json();
  const Values values = Effective(entries);
  if (auto problem = Check(values)) return fail(problem->message);
  entries_ = std::move(entries);
  confirmed_ = std::move(confirmed);
  calibration_applied_ = std::move(applied);
  now_ = values;
  version_ = version;
  return true;
}

std::string Geometry::Text() const {
  json entries = json::object();
  for (const auto& key : GeometryKeys()) {
    if (key.type == GeometryType::kNumber) {
      const auto it = entries_.numbers.find(key.name);
      if (it != entries_.numbers.end()) entries[key.name] = it->second;
    } else if (key.settable && entries_.outline) {
      entries[key.name] = PointsJson(*entries_.outline);
    }
  }
  json file = json::object();
  file["version"] = version_;
  file["entries"] = std::move(entries);
  if (!confirmed_.empty()) file["confirmed"] = confirmed_;
  if (!calibration_applied_.is_null()) file["calibration_applied"] = calibration_applied_;
  return file.dump(2) + "\n";
}

json Geometry::Get() const {
  json settings = json::object();
  for (const auto& key : GeometryKeys()) {
    json entry = json::object();
    if (key.type == GeometryType::kNumber) {
      entry["value"] = now_.numbers.at(key.name);
      entry["default"] = factory_.numbers.at(key.name);
      entry["stored"] = entries_.numbers.count(key.name) != 0;
      const auto [min, max] = Range(key);
      entry["type"] = "number";
      entry["unit"] = key.unit;
      entry["min"] = min;
      entry["max"] = max;
    } else if (key.type == GeometryType::kShape) {
      entry["value"] = ShapeJson();
      entry["default"] = entry["value"];
      entry["stored"] = false;
      entry["type"] = "shape";
      entry["unit"] = "m";
    } else {
      entry["value"] = PointsJson(PointsOf(now_, key.name));
      entry["default"] = PointsJson(PointsOf(factory_, key.name));
      entry["stored"] = key.settable && entries_.outline.has_value();
      entry["type"] = "points";
      entry["unit"] = "m";
    }
    entry["settable"] = key.settable;
    if (key.settable) {
      const auto it = confirmed_.find(key.name);
      entry["confirmed"] = it == confirmed_.end() ? json() : json(it->second);
    }
    settings[key.name] = std::move(entry);
  }
  json out = json::object();
  out["settings"] = std::move(settings);
  out["version"] = version_;
  out["platform"] = platform_;
  return out;
}

json Geometry::Params() const {
  json out = json::object();
  for (const auto& key : GeometryKeys()) {
    if (key.type == GeometryType::kShape) continue;
    out[key.name] =
        key.type == GeometryType::kNumber ? json(now_.numbers.at(key.name)) : PointsJson(PointsOf(now_, key.name));
  }
  out["version"] = version_;
  json stored = json::array();
  json confirmed = json::array();
  for (const auto& key : GeometryKeys()) {
    if (key.type == GeometryType::kNumber ? entries_.numbers.count(key.name) != 0
                                          : key.settable && entries_.outline.has_value()) {
      stored.push_back(key.name);
    }
    if (confirmed_.count(key.name)) confirmed.push_back(key.name);
  }
  out["stored"] = std::move(stored);
  out["confirmed"] = std::move(confirmed);
  out["calibration_applied"] = calibration_applied_;
  return out;
}

GeometryChange Geometry::ApplyCalibration(const std::string& result_id, const json& settings, double now) {
  GeometryChange change;
  if (settings.empty()) {
    change.ok = true;
  } else {
    change = Set({{"version", version_}, {"settings", settings}}, now);
    if (!change.ok) return change;
  }
  json keys = json::array();
  for (const auto& item : settings.items()) keys.push_back(item.key());
  json values = json::object();
  for (const auto& key : mowbite_api::CalibrationKeys()) values[key] = Value(key);
  calibration_applied_ = {{"result_id", result_id},
                          {"t", static_cast<int64_t>(std::floor(now))},
                          {"keys", std::move(keys)},
                          {"geometry_version", version_},
                          {"values", std::move(values)}};
  change.stored = true;
  return change;
}

json Geometry::ShapeJson() const {
  json out = json::array();
  for (const ShapePart& part : shape_) {
    out.push_back({{"id", part.id}, {"kind", part.kind}, {"points", PointsJson(part.points)}, {"closed", part.closed}});
  }
  return out;
}

std::vector<ShapeOver> Geometry::Outside(const std::vector<ShapePart>& shape, const Values& values) {
  // What a ruler does not tell apart.
  constexpr double tolerance = 0.001;
  const std::vector<Point> body = BodyPolygon(values.numbers, values.outline);
  std::vector<ShapeOver> out;
  for (const ShapePart& part : shape) {
    double over = 0;
    for (const Point& point : part.points) {
      if (Inside(body, point[0], point[1])) continue;
      double distance = std::numeric_limits<double>::infinity();
      for (size_t i = 0, j = body.size() - 1; i < body.size(); j = i++) {
        distance = std::min(distance, DistanceToSegment(body[j], body[i], point[0], point[1]));
      }
      over = std::max(over, distance);
    }
    if (over > tolerance) out.push_back({part.id, over});
  }
  return out;
}

std::vector<ShapeOver> Geometry::OutsideShape() const {
  return Outside(shape_, now_);
}

GeometryChange Geometry::Set(const json& request, double now) {
  GeometryChange change;
  const auto refuse = [&change](GeometryProblem problem) {
    change.problem = std::move(problem);
    return change;
  };
  const std::string usage =
      "Expected {\"version\": N, \"settings\": {key: value}, \"reset\": [key], \"confirm\": [key]}";
  if (!request.is_object() || !request.contains("version") || !request["version"].is_number_integer()) {
    return refuse(Problem("INVALID_REQUEST", "version", usage));
  }
  for (const auto& item : request.items()) {
    if (item.key() != "version" && item.key() != "settings" && item.key() != "reset" && item.key() != "confirm") {
      return refuse(Problem("INVALID_REQUEST", item.key(), item.key() + " is no field of the request. " + usage));
    }
  }
  if (request["version"].get<int64_t>() != version_) {
    GeometryProblem problem = Problem("VERSION", "version",
                                      "The geometry is at version " + std::to_string(version_) + " now, not " +
                                          std::to_string(request["version"].get<int64_t>()) + ": read it again");
    problem.data["version"] = version_;
    return refuse(std::move(problem));
  }
  const json none_set = json::object();
  const json none_reset = json::array();
  const json& settings = request.contains("settings") ? request["settings"] : none_set;
  const json& reset = request.contains("reset") ? request["reset"] : none_reset;
  const json& confirm = request.contains("confirm") ? request["confirm"] : none_reset;
  if (!settings.is_object() || !reset.is_array() || !confirm.is_array()) {
    return refuse(Problem("INVALID_REQUEST", "", usage));
  }
  if (settings.empty() && reset.empty() && confirm.empty()) {
    return refuse(Problem("INVALID_REQUEST", "", "Nothing to change. " + usage));
  }

  // The key must exist and be the user's to set.
  const auto settable = [](const std::string& name, GeometryProblem* problem) -> const GeometryKey* {
    const GeometryKey* key = FindKey(name);
    if (!key) {
      *problem = Problem("UNKNOWN_KEY", name, name + " is no key of the geometry");
      return nullptr;
    }
    if (!key->settable) {
      *problem = Problem("NOT_SETTABLE", name, name + " is set in the file of the platform, not here");
      return nullptr;
    }
    return key;
  };

  Entries entries = entries_;
  std::map<std::string, double> confirmed = confirmed_;
  std::set<std::string> reset_keys;
  GeometryProblem problem;
  for (const auto& item : reset) {
    if (!item.is_string()) return refuse(Problem("INVALID_REQUEST", "reset", "reset lists names of keys. " + usage));
    const std::string name = item.get<std::string>();
    const GeometryKey* key = settable(name, &problem);
    if (!key) return refuse(problem);
    reset_keys.insert(name);
    confirmed.erase(name);
    if (key->type == GeometryType::kNumber) {
      entries.numbers.erase(name);
    } else {
      entries.outline.reset();
    }
  }
  for (const auto& item : settings.items()) {
    const GeometryKey* key = settable(item.key(), &problem);
    if (!key) return refuse(problem);
    if (reset_keys.count(item.key())) {
      return refuse(Problem("INVALID_REQUEST", item.key(), item.key() + " is both set and reset"));
    }
    if (key->type == GeometryType::kNumber) {
      if (!item.value().is_number())
        return refuse(Problem("INVALID_VALUE", item.key(), item.key() + " must be a number"));
      entries.numbers[item.key()] = item.value().get<double>();
    } else {
      std::string why;
      auto points = ParsePoints(item.value(), &why);
      if (!points) return refuse(Problem("INVALID_VALUE", item.key(), why));
      entries.outline = std::move(*points);
    }
    confirmed[item.key()] = now;
  }
  for (const auto& item : confirm) {
    if (!item.is_string()) {
      return refuse(Problem("INVALID_REQUEST", "confirm", "confirm lists names of keys. " + usage));
    }
    const std::string name = item.get<std::string>();
    if (!settable(name, &problem)) return refuse(problem);
    if (reset_keys.count(name)) return refuse(Problem("INVALID_REQUEST", name, name + " is both reset and confirmed"));
    confirmed[name] = now;
  }

  const Values values = Effective(entries);
  if (auto wrong = Check(values)) return refuse(*wrong);
  change.ok = true;
  if (entries == entries_ && confirmed == confirmed_) return change;
  confirmed_ = std::move(confirmed);

  for (const auto& key : GeometryKeys()) {
    if (key.type == GeometryType::kShape) continue;
    const bool differs = key.type == GeometryType::kNumber
                             ? std::abs(values.numbers.at(key.name) - now_.numbers.at(key.name)) > 1e-12
                             : PointsOf(values, key.name) != PointsOf(now_, key.name);
    if (differs) change.changed.push_back(key.name);
  }
  entries_ = std::move(entries);
  now_ = values;
  change.stored = true;
  // The version counts a change of a value, so the event and the new version always come together.
  if (!change.changed.empty()) ++version_;
  change.below_default = Below(factory_, now_);
  change.outside_shape = Outside(shape_, now_);
  return change;
}

}  // namespace mowbite_logic
