// The settings of /mower_logic an app changes with params.set: names, types, ranges and defaults of MowerLogic.cfg
// (OpenMower) as the app knows them, with bounds of MowBite OS on top. The planning values are with the planner.
#ifndef MOWBITE_LOGIC_LOGIC_PARAMS_H
#define MOWBITE_LOGIC_LOGIC_PARAMS_H

#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace mowbite_logic {

using json = nlohmann::ordered_json;

constexpr const char* kLogicParamsNode = "/mower_logic";

enum class ParamType { kInt, kDouble };

struct ParamSpec {
  std::string name;
  ParamType type;
  double min;
  double max;
  double def;
};

// The 13 settings in the order of the app's groups. max_position_accuracy at most 0.3 m, gps_timeout at least 1 s and
// undock_distance 0.5 to 2 m are bounds of MowBite OS; MowerLogic.cfg allows 1 m, 0 s and 0 to 100 m.
const std::vector<ParamSpec>& LogicParamSpecs();

struct ParamsResult {
  bool ok = false;
  // The values taken by their full name ("/mower_logic/x"), out of range ones set to the bound.
  json applied = json::object();
  // Why nothing changed.
  std::string message;
};

class LogicParams {
 public:
  // Defaults of the specs, then base ({"name": value}, e.g. from tron.yaml) for the names it has, within range.
  explicit LogicParams(const json& base = json::object());

  // A file written from Text(): known names are taken within range, others are left out. False if it is no JSON
  // object or breaks a rule between values; then nothing changes.
  bool Load(const std::string& text);
  // {"name": value} of all settings, ints as integers.
  json Values() const;
  std::string Text() const;
  double Get(const std::string& name) const;

  // The parameters of params.set: {"params": {"/mower_logic/x": value, ...}}. All or nothing: a name of another node
  // or an unknown one, a value that is no number, or motor_cold_temperature not below motor_hot_temperature changes
  // nothing.
  ParamsResult Set(const json& request);

 private:
  static double Fit(const ParamSpec& spec, double value);
  static std::string Check(const std::map<std::string, double>& values);

  std::map<std::string, double> values_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_PARAMS_H
