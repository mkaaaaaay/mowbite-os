#include "mowbite_logic/geometry.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <stdexcept>

#include "mowbite_api/catalog.h"

namespace mowbite_logic {

const std::vector<GeometryKey>& GeometryKeys() {
  // The keys and ranges are the interface's (x-geometry).
  static const std::vector<GeometryKey> keys = [] {
    std::vector<GeometryKey> out;
    for (const auto& key : mowbite_api::GeometryKeys()) {
      out.push_back({key.name, key.points ? GeometryType::kPoints : GeometryType::kNumber, key.settable, key.min,
                     key.max, key.unit.c_str(), key.relative});
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
std::optional<std::vector<Point>> ParsePoints(const json& value, std::string* why) {
  std::vector<Point> points;
  if (!value.is_array()) {
    *why = "robot_outline must be a list of [x, y] pairs";
    return std::nullopt;
  }
  for (const auto& item : value) {
    if (!item.is_array() || item.size() != 2 || !item[0].is_number() || !item[1].is_number()) {
      *why = "robot_outline must be a list of [x, y] pairs";
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
    } else if (factory.contains(key.name)) {
      std::string why;
      auto points = ParsePoints(factory[key.name], &why);
      if (!points) throw std::invalid_argument(why);
      factory_.outline = std::move(*points);
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
  const Values values = Effective(entries);
  if (auto problem = Check(values)) return fail(problem->message);
  entries_ = std::move(entries);
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
    } else if (entries_.outline) {
      entries[key.name] = PointsJson(*entries_.outline);
    }
  }
  json file = json::object();
  file["version"] = version_;
  file["entries"] = std::move(entries);
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
    } else {
      entry["value"] = PointsJson(now_.outline);
      entry["default"] = PointsJson(factory_.outline);
      entry["stored"] = entries_.outline.has_value();
      entry["type"] = "points";
      entry["unit"] = "m";
    }
    entry["settable"] = key.settable;
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
    out[key.name] = key.type == GeometryType::kNumber ? json(now_.numbers.at(key.name)) : PointsJson(now_.outline);
  }
  out["version"] = version_;
  json stored = json::array();
  for (const auto& key : GeometryKeys()) {
    if (key.type == GeometryType::kNumber ? entries_.numbers.count(key.name) != 0 : entries_.outline.has_value()) {
      stored.push_back(key.name);
    }
  }
  out["stored"] = std::move(stored);
  return out;
}

GeometryChange Geometry::Set(const json& request) {
  GeometryChange change;
  const auto refuse = [&change](GeometryProblem problem) {
    change.problem = std::move(problem);
    return change;
  };
  const std::string usage = "Expected {\"version\": N, \"settings\": {key: value}, \"reset\": [key]}";
  if (!request.is_object() || !request.contains("version") || !request["version"].is_number_integer()) {
    return refuse(Problem("INVALID_REQUEST", "version", usage));
  }
  for (const auto& item : request.items()) {
    if (item.key() != "version" && item.key() != "settings" && item.key() != "reset") {
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
  if (!settings.is_object() || !reset.is_array()) return refuse(Problem("INVALID_REQUEST", "", usage));
  if (settings.empty() && reset.empty()) return refuse(Problem("INVALID_REQUEST", "", "Nothing to change. " + usage));

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
  std::set<std::string> reset_keys;
  GeometryProblem problem;
  for (const auto& item : reset) {
    if (!item.is_string()) return refuse(Problem("INVALID_REQUEST", "reset", "reset lists names of keys. " + usage));
    const std::string name = item.get<std::string>();
    const GeometryKey* key = settable(name, &problem);
    if (!key) return refuse(problem);
    reset_keys.insert(name);
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
  }

  const Values values = Effective(entries);
  if (auto wrong = Check(values)) return refuse(*wrong);
  change.ok = true;
  if (entries == entries_) return change;

  for (const auto& key : GeometryKeys()) {
    const bool differs = key.type == GeometryType::kNumber
                             ? std::abs(values.numbers.at(key.name) - now_.numbers.at(key.name)) > 1e-12
                             : values.outline != now_.outline;
    if (differs) change.changed.push_back(key.name);
  }
  entries_ = std::move(entries);
  now_ = values;
  change.stored = true;
  // The version counts a change of a value, so the event and the new version always come together.
  if (!change.changed.empty()) ++version_;
  change.below_default = Below(factory_, now_);
  return change;
}

}  // namespace mowbite_logic
