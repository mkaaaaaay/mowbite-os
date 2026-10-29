// MQTT payloads of the OpenMower app interface. Based on xbot_monitoring.cpp and xbot_mqtt/publish.h (OpenMower), MIT
// License, Copyright (c) 2022 Clemens Elflein. See LICENSE.xbot_monitoring.
#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

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

enum class Incoming { kTeleop, kAction, kRpcRequest, kRetainedCommand, kOther };

// Commands from the app act only when sent. A retained one comes again after every connect and is dropped; the broker
// clears the retain flag for messages to an existing subscription, so live commands are never affected.
Incoming ClassifyIncoming(const std::string& topic, bool retained);

// teleop: BSON {vx, vz}. Returns nothing for oversized or invalid payloads.
std::optional<std::pair<double, double>> DecodeTeleop(const std::string& payload);

std::string NanoId(size_t length = 32);

// Version of the files in data_dir, raised when their layout changes.
constexpr int kDataSchema = 1;

// Answer of system.info: how the app recognizes MowBite OS. platform is empty until the driver has reported it.
json SystemInfo(const std::string& version, const std::string& platform);

// {"id", "t", "type", ...details}
json EventPayload(const std::string& type, const json& details, double t);

// Name in sensor_infos/json for a SensorInfo value_description, as OpenMower writes it.
const char* ValueDescriptionName(uint8_t value_description);

// A sensor the bridge knows: its entry of sensor_infos/json and its last value.
struct SensorEntry {
  json info;
  // A number, for a STRING sensor a text; null before the first value.
  json value = nullptr;
  // Unix s of the last value, like t of the events; null before the first value.
  json stamp = nullptr;
};

// Answer of sensors.list: the entries of sensor_infos/json with value and stamp, in the order given. params null, empty
// or without ids: all of them; {"ids": [sensor_id, ...]}: only those, unknown ids left out. Throws
// std::invalid_argument for ids that are not a list of texts and for other params.
json SensorList(const std::vector<SensorEntry>& sensors, const json& params);

}  // namespace mowbite_bridge
