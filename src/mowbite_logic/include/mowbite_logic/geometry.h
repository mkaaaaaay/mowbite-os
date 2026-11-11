// The dimensions of the mower (robot.geometry): the values of the platform's file, the entries of the user on top of
// them, the rules the entries keep, and the JSON of robot.geometry.get, of bridge/params and of the file of the
// entries. x points forward and y to the left of base_link, lengths in m; ticks_per_m in 1/m and the mounting of the
// IMU (imu_roll, imu_pitch) in rad.
#ifndef MOWBITE_LOGIC_GEOMETRY_H
#define MOWBITE_LOGIC_GEOMETRY_H

#include <array>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mowbite_logic {

using json = nlohmann::ordered_json;

// The entry of bridge/params the planner and the other nodes read the dimensions from.
constexpr const char* kGeometryNode = "/mowbite_geometry";

enum class GeometryType { kNumber, kPoints };

struct GeometryKey {
  std::string name;
  GeometryType type;
  // The user can set it; wheel_radius and dock_contact_x are the platform's alone.
  bool settable;
  // The range of a number in its unit; with relative, around the platform's value.
  double min;
  double max;
  const char* unit = "m";
  bool relative = false;
};

// All keys in the order of the answers; robot_outline is the only list.
const std::vector<GeometryKey>& GeometryKeys();

using Point = std::array<double, 2>;

// Why a request or a file was refused. reason: INVALID_REQUEST, VERSION, UNKNOWN_KEY, NOT_SETTABLE, INVALID_VALUE,
// OUT_OF_RANGE or OUTSIDE_BODY; key: what it is about, if one; data: more fields for the answer (min, max, version).
struct GeometryProblem {
  std::string reason;
  std::string key;
  std::string message;
  json data = json::object();
};

struct GeometryChange {
  bool ok = false;
  // Why not, if !ok.
  GeometryProblem problem;
  // The entries are other now: save Text() and tell the others.
  bool stored = false;
  // Keys whose value is another now.
  std::vector<std::string> changed;
  // Keys of the body below the platform's value after the change (robot_width, robot_front, robot_rear, robot_outline).
  std::vector<std::string> below_default;
};

class Geometry {
 public:
  // The platform's values: {"robot_width": 0.47, ...} with every number key of GeometryKeys(), and optionally
  // "robot_outline": [[x, y], ...]. Throws std::invalid_argument naming the problem if a number is missing, is none or
  // the values break a rule.
  Geometry(std::string platform, const json& factory);

  // The file written from Text(): {"version": N, "entries": {key: value}}. False with the reason in why if it is no
  // JSON object, has a key that is unknown or not settable, or its values break a rule; then nothing changes.
  bool Load(const std::string& text, std::string* why);
  std::string Text() const;

  // Counts up with every change of a value; entering or resetting a value that equals the platform's keeps it.
  int version() const {
    return version_;
  }
  const std::string& platform() const {
    return platform_;
  }
  // A number key, as it counts now.
  double Value(const std::string& key) const;
  // The range of a number key: min and max, around the platform's value for a relative key.
  std::pair<double, double> Range(const GeometryKey& key) const;

  // robot.geometry.get: {"settings": {key: {value, default, stored, type, unit, min, max, settable}}, "version",
  // "platform"}.
  json Get() const;
  // The "params" of the bridge/params entry: every key as it counts now (robot_outline [] without one), version, and
  // stored: the keys the user entered.
  json Params() const;

  // robot.geometry.set: {"version": N, "settings": {key: value}, "reset": [key]}. All or nothing: a version that is not
  // the current one, a key that is unknown or not settable, a value of the wrong kind or one that breaks a rule changes
  // nothing. Out-of-range values are refused, not cut to the bound.
  GeometryChange Set(const json& request);

  // Keys of the body below the platform's value now.
  std::vector<std::string> BelowDefault() const;
  // m from base_link to the farthest point of the body: the points of robot_outline, without one the corners of the
  // box of robot_width, robot_front and robot_rear. The body stays within this circle when the mower turns on the spot.
  double TurningRadius() const;

 private:
  struct Values {
    std::map<std::string, double> numbers;
    // Empty: the rectangle of robot_width, robot_front and robot_rear.
    std::vector<Point> outline;
  };
  struct Entries {
    std::map<std::string, double> numbers;
    std::optional<std::vector<Point>> outline;
    bool operator==(const Entries& other) const;
  };

  Values Effective(const Entries& entries) const;
  std::optional<GeometryProblem> Check(const Values& values) const;
  static std::vector<std::string> Below(const Values& factory, const Values& now);
  static json PointsJson(const std::vector<Point>& points);

  std::string platform_;
  Values factory_;
  Entries entries_;
  Values now_;
  int version_ = 0;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_GEOMETRY_H
