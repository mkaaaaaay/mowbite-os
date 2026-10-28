// MQTT payloads of the OpenMower app interface. Based on xbot_monitoring.cpp and xbot_mqtt/publish.h (OpenMower), MIT
// License, Copyright (c) 2022 Clemens Elflein. See LICENSE.xbot_monitoring.
#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace mowbite_bridge {

using json = nlohmann::ordered_json;

constexpr int16_t kRpcInvalidJson = -32700;
constexpr int16_t kRpcInvalidRequest = -32600;
constexpr int16_t kRpcMethodNotFound = -32601;
constexpr int16_t kRpcInvalidParams = -32602;
constexpr int16_t kRpcInternal = -32603;

// Deepest nesting of [ and { in a JSON text, strings skipped.
size_t JsonNesting(const std::string& text);

std::string RpcErrorJson(int16_t code, const std::string& message, const json& id = nullptr);

// result is JSON text; an unparsable result becomes an internal error.
std::string RpcResultJson(const std::string& id, const std::string& result);

struct RpcCall {
  std::string method;
  json params;  // null without params
  std::string id;
};

struct RpcParseResult {
  enum class Kind { kCall, kError, kIgnore };
  Kind kind = Kind::kIgnore;
  RpcCall call;
  std::string error;  // response text for kError
};

// Validates a JSON-RPC 2.0 request. meta.* and ext.* are ignored, they are answered elsewhere. Whether the method is
// known is up to the caller.
RpcParseResult ParseRpcRequest(const std::string& payload);

// teleop: BSON {vx, vz}. Returns nothing for oversized or invalid payloads.
std::optional<std::pair<double, double>> DecodeTeleop(const std::string& payload);

std::string NanoId(size_t length = 32);

// {"id", "t", "type", ...details}
json EventPayload(const std::string& type, const json& details, double t);

// Name in sensor_infos/json for a SensorInfo value_description, as OpenMower writes it.
const char* ValueDescriptionName(uint8_t value_description);

}  // namespace mowbite_bridge
