#include "mowbite_logic/logic_params.h"

#include <algorithm>
#include <cmath>

namespace mowbite_logic {

const std::vector<ParamSpec>& LogicParamSpecs() {
  static const std::vector<ParamSpec> specs = {
      {"rain_mode", ParamType::kInt, 0, 3, 0},
      {"rain_delay_minutes", ParamType::kInt, 30, 1440, 30},
      {"rain_check_seconds", ParamType::kInt, 20, 300, 20},
      {"automatic_mode", ParamType::kInt, 0, 2, 0},
      {"gps_timeout", ParamType::kDouble, 1.0, 60.0, 10.0},
      {"max_position_accuracy", ParamType::kDouble, 0.01, 0.3, 0.1},
      {"gps_wait_time", ParamType::kDouble, 0.0, 60.0, 10.0},
      {"docking_approach_distance", ParamType::kDouble, 0.5, 2.0, 1.5},
      {"docking_distance", ParamType::kDouble, 0.0, 100.0, 2.0},
      {"docking_retry_count", ParamType::kInt, 0, 5, 4},
      {"undock_distance", ParamType::kDouble, 0.5, 2.0, 2.0},
      {"motor_hot_temperature", ParamType::kDouble, 20.0, 150.0, 70.0},
      {"motor_cold_temperature", ParamType::kDouble, 20.0, 150.0, 40.0},
  };
  return specs;
}

namespace {

const ParamSpec* FindSpec(const std::string& name) {
  for (const auto& spec : LogicParamSpecs()) {
    if (spec.name == name) return &spec;
  }
  return nullptr;
}

json ValueJson(const ParamSpec& spec, double value) {
  return spec.type == ParamType::kInt ? json(static_cast<int64_t>(value)) : json(value);
}

}  // namespace

LogicParams::LogicParams(const json& base) {
  for (const auto& spec : LogicParamSpecs()) {
    double value = spec.def;
    if (base.is_object() && base.contains(spec.name) && base[spec.name].is_number()) {
      value = base[spec.name].get<double>();
    }
    values_[spec.name] = Fit(spec, value);
  }
}

double LogicParams::Fit(const ParamSpec& spec, double value) {
  if (spec.type == ParamType::kInt) value = std::round(value);
  return std::clamp(value, spec.min, spec.max);
}

std::string LogicParams::Check(const std::map<std::string, double>& values) {
  if (!(values.at("motor_cold_temperature") < values.at("motor_hot_temperature"))) {
    return "motor_cold_temperature must be below motor_hot_temperature";
  }
  return "";
}

bool LogicParams::Load(const std::string& text) {
  const json parsed = json::parse(text, nullptr, false);
  if (!parsed.is_object()) return false;
  auto values = values_;
  for (const auto& spec : LogicParamSpecs()) {
    if (parsed.contains(spec.name) && parsed[spec.name].is_number()) {
      values[spec.name] = Fit(spec, parsed[spec.name].get<double>());
    }
  }
  if (!Check(values).empty()) return false;
  values_ = std::move(values);
  return true;
}

json LogicParams::Values() const {
  json out = json::object();
  for (const auto& spec : LogicParamSpecs()) out[spec.name] = ValueJson(spec, values_.at(spec.name));
  return out;
}

std::string LogicParams::Text() const {
  return Values().dump(2) + "\n";
}

double LogicParams::Get(const std::string& name) const {
  return values_.at(name);
}

ParamsResult LogicParams::Set(const json& request) {
  ParamsResult result;
  if (!request.is_object() || !request.contains("params") || !request["params"].is_object() ||
      request["params"].empty()) {
    result.message = "Expected {\"params\": {name: value, ...}}";
    return result;
  }
  const std::string prefix = std::string(kLogicParamsNode) + "/";
  auto values = values_;
  json applied = json::object();
  for (const auto& item : request["params"].items()) {
    const std::string& full = item.key();
    if (full.rfind(prefix, 0) != 0) {
      // As OpenMower answers for a node without settings.
      result.message = "No dynamic_reconfigure server found for " + full;
      return result;
    }
    const ParamSpec* spec = FindSpec(full.substr(prefix.size()));
    if (!spec) {
      result.message = full + " is not a setting of MowBite OS";
      return result;
    }
    if (!item.value().is_number()) {
      result.message = full + " must be a number";
      return result;
    }
    const double value = Fit(*spec, item.value().get<double>());
    values[spec->name] = value;
    applied[full] = ValueJson(*spec, value);
  }
  result.message = Check(values);
  if (!result.message.empty()) return result;
  values_ = std::move(values);
  result.ok = true;
  result.applied = std::move(applied);
  return result;
}

}  // namespace mowbite_logic
