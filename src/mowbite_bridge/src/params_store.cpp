// Based on publish_params in xbot_monitoring.cpp (OpenMower), MIT License, Copyright (c) 2022 Clemens Elflein.
// See LICENSE.xbot_monitoring.
#include "mowbite_bridge/params_store.h"

#include <algorithm>
#include <vector>

namespace mowbite_bridge {

namespace {

bool IsSecret(const std::string& name) {
  return name.find("password") != std::string::npos;
}

// Keys sorted like the original, which listed the names of the parameter server in order.
json Sorted(const json& params) {
  std::vector<std::string> keys;
  for (const auto& item : params.items()) keys.push_back(item.key());
  std::sort(keys.begin(), keys.end());
  json out = json::object();
  for (const auto& key : keys) out[key] = IsSecret(key) ? json(nullptr) : params[key];
  return out;
}

}  // namespace

ParamsStore::ParamsStore(const json& base) {
  if (base.is_object()) params_ = Sorted(base);
  payload_ = params_.dump();
}

bool ParamsStore::Update(const std::string& message) {
  json parsed = json::parse(message, nullptr, false);
  if (!parsed.is_object() || !parsed.contains("node") || !parsed["node"].is_string() || !parsed.contains("params") ||
      !parsed["params"].is_object()) {
    return false;
  }
  std::string node = parsed["node"].get<std::string>();
  while (!node.empty() && node.back() == '/') node.pop_back();
  if (node.empty() || node.front() != '/') node.insert(node.begin(), '/');
  for (const auto& item : parsed["params"].items()) {
    if (item.value().is_discarded() || item.value().is_binary()) continue;
    params_[node + "/" + item.key()] = item.value();
  }
  params_ = Sorted(params_);
  std::string payload = params_.dump();
  const bool changed = payload != payload_;
  payload_ = std::move(payload);
  return changed;
}

}  // namespace mowbite_bridge
