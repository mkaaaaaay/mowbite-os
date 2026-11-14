#include "mowbite_logic/logic_params.h"

#include <algorithm>
#include <cmath>

#include "mowbite_api/catalog.h"

namespace mowbite_logic {

const std::vector<ParamSpec>& LogicParamSpecs() {
  // The ranges are the interface's (x-settings).
  static const std::vector<ParamSpec> specs = [] {
    std::vector<ParamSpec> out;
    for (const auto& setting : mowbite_api::LogicSettings()) {
      out.push_back({setting.name, setting.integer ? ParamType::kInt : ParamType::kDouble, setting.min, setting.max,
                     setting.def});
    }
    return out;
  }();
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

uint16_t BladeHeightMm(double setting, bool control, uint16_t min_mm, uint16_t max_mm) {
  if (!control || !(setting >= 1) || min_mm == 0 || min_mm > max_mm) return 0;
  return static_cast<uint16_t>(std::clamp(std::round(setting), static_cast<double>(min_mm), static_cast<double>(max_mm)));
}

}  // namespace mowbite_logic
