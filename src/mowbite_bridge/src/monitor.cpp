// Based on xbot_monitoring.cpp and xbot_mqtt/publish.h (OpenMower), MIT License, Copyright (c) 2022 Clemens Elflein.
// See LICENSE.xbot_monitoring.
#include "mowbite_bridge/monitor.h"

#include <algorithm>
#include <random>

namespace mowbite_bridge {

size_t JsonNesting(const std::string& text) {
  size_t depth = 0, deepest = 0;
  bool in_string = false;
  for (size_t i = 0; i < text.size(); i++) {
    const char c = text[i];
    if (in_string) {
      if (c == '\\')
        i++;
      else if (c == '"')
        in_string = false;
    } else if (c == '"') {
      in_string = true;
    } else if (c == '[' || c == '{') {
      deepest = std::max(deepest, ++depth);
    } else if ((c == ']' || c == '}') && depth > 0) {
      depth--;
    }
  }
  return deepest;
}

json SystemInfo(const std::string& version, const std::string& platform) {
  return json{{"name", "mowbite-os"}, {"version", version}, {"platform", platform}, {"schema", kDataSchema}};
}

std::string RpcErrorJson(int16_t code, const std::string& message, const json& id) {
  json err = {{"jsonrpc", "2.0"}, {"error", {{"code", code}, {"message", message}}}, {"id", id}};
  return err.dump(2);
}

std::string RpcResultJson(const std::string& id, const std::string& result) {
  json parsed;
  try {
    parsed = json::parse(result);
  } catch (const json::exception& e) {
    return RpcErrorJson(kRpcInternal, "Internal error while parsing result JSON: " + std::string(e.what()), id);
  }
  json j = {{"jsonrpc", "2.0"}, {"result", parsed}, {"id", id}};
  return j.dump(2);
}

RpcParseResult ParseRpcRequest(const std::string& payload) {
  RpcParseResult out;
  auto error = [&out](int16_t code, const std::string& message, const json& id = nullptr) {
    out.kind = RpcParseResult::Kind::kError;
    out.error = RpcErrorJson(code, message, id);
    return out;
  };

  // A request nested some 100k levels deep overflows the stack when it is copied, no RPC needs more than a few.
  if (JsonNesting(payload) > 100) return error(kRpcInvalidRequest, "Request is nested too deep");

  json req;
  try {
    req = json::parse(payload);
  } catch (const json::exception&) {
    return error(kRpcInvalidJson, "Could not parse request JSON");
  }
  if (!req.is_object()) return error(kRpcInvalidRequest, "Request is not a JSON object");

  json id = req.contains("id") ? req["id"] : json(nullptr);
  if (!id.is_null() && !id.is_string()) return error(kRpcInvalidRequest, "ID is not a string", id);
  if (!req.contains("jsonrpc") || !req["jsonrpc"].is_string() || req["jsonrpc"] != "2.0") {
    return error(kRpcInvalidRequest, "Invalid JSON-RPC version");
  }
  if (!req.contains("method") || !req["method"].is_string()) {
    return error(kRpcInvalidRequest, "Method is not a string", id);
  }

  const std::string method = req["method"];
  if (method.rfind("meta.", 0) == 0 || method.rfind("ext.", 0) == 0) {
    out.kind = RpcParseResult::Kind::kIgnore;
    return out;
  }
  out.kind = RpcParseResult::Kind::kCall;
  out.call.method = method;
  out.call.params = req.contains("params") ? req["params"] : json(nullptr);
  out.call.id = id.is_null() ? "" : id.get<std::string>();
  return out;
}

Incoming ClassifyIncoming(const std::string& topic, bool retained) {
  Incoming kind = Incoming::kOther;
  if (topic == "teleop") {
    kind = Incoming::kTeleop;
  } else if (topic == "action") {
    kind = Incoming::kAction;
  } else if (topic == "rpc/request") {
    kind = Incoming::kRpcRequest;
  }
  return retained && kind != Incoming::kOther ? Incoming::kRetainedCommand : kind;
}

std::optional<std::pair<double, double>> DecodeTeleop(const std::string& payload) {
  // Only vx and vz, some 30 bytes. A large nested document overflowed the stack while decoding.
  if (payload.size() > 1024) return std::nullopt;
  try {
    json j = json::from_bson(payload.begin(), payload.end());
    if (!j.is_object() || !j.contains("vx") || !j.contains("vz") || !j["vx"].is_number() || !j["vz"].is_number()) {
      return std::nullopt;
    }
    return std::make_pair(j["vx"].get<double>(), j["vz"].get<double>());
  } catch (const json::exception&) {
    return std::nullopt;
  }
}

std::string NanoId(size_t length) {
  static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  thread_local std::mt19937 rng{std::random_device{}()};
  thread_local std::uniform_int_distribution<> dist(0, sizeof(alphabet) - 2);
  std::string id(length, '\0');
  std::generate_n(id.begin(), length, [&]() { return alphabet[dist(rng)]; });
  return id;
}

json EventPayload(const std::string& type, const json& details, double t) {
  json payload = json::object();
  payload["id"] = NanoId();
  payload["t"] = t;
  payload["type"] = type;
  if (details.is_object()) payload.update(details);
  return payload;
}

const char* ValueDescriptionName(uint8_t value_description) {
  switch (value_description) {
    case 1: return "TEMPERATURE";
    case 2: return "VELOCITY";
    case 3: return "ACCELERATION";
    case 4: return "VOLTAGE";
    case 5: return "CURRENT";
    case 6: return "PERCENT";
    case 8: return "REVOLUTIONS";
    default: return "UNKNOWN";
  }
}

}  // namespace mowbite_bridge
