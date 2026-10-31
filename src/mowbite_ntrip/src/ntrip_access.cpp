#include "mowbite_ntrip/ntrip_access.h"

#include <algorithm>
#include <cstdint>
#include <iterator>

namespace mowbite_ntrip {

namespace {

using json = nlohmann::ordered_json;

const char* const kFields[] = {"host", "port", "mountpoint", "version", "user", "password"};

// A whole number from min to max.
std::optional<int> Whole(const json& value, int min, int max) {
  if (!value.is_number_integer()) return std::nullopt;
  const auto v = value.get<int64_t>();
  if (v < min || v > max) return std::nullopt;
  return static_cast<int>(v);
}

}  // namespace

std::vector<std::string> AccessMethods(bool write) {
  if (!write) return {kGetMethod};
  return {kGetMethod, kSetMethod, kDeleteMethod};
}

json AccessSummary(const AccessFile& file) {
  json summary = {{"configured", file.config.has_value()}};
  if (file.config) {
    summary["host"] = file.config->host;
    summary["port"] = file.config->port;
    summary["mountpoint"] = file.config->mountpoint;
    summary["version"] = file.config->version;
    summary["has_user"] = !file.config->user.empty();
    summary["has_password"] = !file.config->password.empty();
  } else if (file.exists) {
    summary["error"] = file.error;
  }
  return summary;
}

AccessChange ChangeAccess(const json& params, const std::optional<NtripConfig>& stored) {
  AccessChange change;
  auto refuse = [&change](const std::string& error) {
    change.error = error;
    return change;
  };
  if (!params.is_object()) return refuse("params: an object with host, port, mountpoint, version, user and password");
  for (const auto& item : params.items()) {
    if (std::find(std::begin(kFields), std::end(kFields), item.key()) == std::end(kFields)) {
      // The name is not repeated: a password sent as a name would come back in the answer.
      return refuse("unknown field; known are host, port, mountpoint, version, user and password");
    }
  }
  NtripConfig config;
  for (const std::string key : {"host", "mountpoint"}) {
    if (!params.contains(key)) return refuse(key + ": missing");
    if (!params[key].is_string()) return refuse(key + ": not a text");
    std::string value = params[key].get<std::string>();
    if (key == "mountpoint" && !value.empty() && value[0] == '/') value.erase(0, 1);
    const std::string problem = CheckValue(key, value);
    if (!problem.empty()) return refuse(key + ": " + problem);
    (key == "host" ? config.host : config.mountpoint) = value;
  }
  if (params.contains("port")) {
    const auto port = Whole(params["port"], 1, 65535);
    if (!port) return refuse("port: not a port number");
    config.port = *port;
  }
  if (params.contains("version")) {
    const auto version = Whole(params["version"], 1, 2);
    if (!version) return refuse("version: 1 or 2");
    config.version = *version;
  }
  const bool same_caster = stored && stored->host == config.host && stored->port == config.port;
  for (const std::string key : {"user", "password"}) {
    std::string& value = key == "user" ? config.user : config.password;
    if (params.contains(key)) {
      if (!params[key].is_string()) return refuse(key + ": not a text");
      value = params[key].get<std::string>();
      const std::string problem = CheckValue(key, value);
      if (!problem.empty()) return refuse(key + ": " + problem);
    } else if (same_caster) {
      value = key == "user" ? stored->user : stored->password;
    } else {
      return refuse(key + ": needed for a new caster (\"\" for none)");
    }
  }
  change.config = config;
  return change;
}

}  // namespace mowbite_ntrip
