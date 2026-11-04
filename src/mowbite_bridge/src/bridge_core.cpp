#include "mowbite_bridge/bridge_core.h"

#include <algorithm>

#include "mowbite_bridge/event_builder.h"
#include "mowbite_bridge/text.h"

namespace mowbite_bridge {

namespace {

constexpr char kRequestTopic[] = "rpc/request";
constexpr char kResponseTopic[] = "rpc/response";
constexpr char kEventsTopic[] = "events/json";
constexpr std::size_t kMaxDataDepth = 64;

Json IdOf(const std::string& id) {
  return id.empty() ? Json() : Json(id);
}

}  // namespace

BridgeCore::BridgeCore(BridgeConfig config) : config_(std::move(config)), params_(config_.params_json) {
  local_["rpc.ping"] = [](const Json&) { return Json("pong"); };
  local_["rpc.methods"] = [this](const Json&) { return Json(methods_.All()); };
  local_["system.info"] = [this](const Json&) { return SystemInfo(config_.software_version, platform_); };
  local_["sensors.list"] = [this](const Json& params) { return SensorList(sensors_, readings_, params); };
  local_["logs.recent"] = [this](const Json& params) { return warnings_.Recent(params); };
  for (const auto& [name, method] : local_) methods_.AddLocal(name);
  // The app's retained texts come first after a connect, in this order; sensors and map only once there are some.
  kept_order_ = {"capabilities/json", "version/json", "params/json", "sensor_infos/json", "actions/json", "map/json"};
  Keep("capabilities/json", CapabilitiesText());
  Keep("version/json", VersionText(config_.software_version));
  Keep("params/json", params_.Text());
  Keep("actions/json", ActionsText(actions_));
}

const std::vector<std::string>& BridgeCore::CommandTopics() {
  static const std::vector<std::string> topics = {"action", "teleop", kRequestTopic};
  return topics;
}

void BridgeCore::AddLocalMethod(const std::string& name, LocalMethod method) {
  local_[name] = std::move(method);
  methods_.AddLocal(name);
}

void BridgeCore::SetEventSink(EventSink sink) {
  sink_ = std::move(sink);
}

template <typename F>
void BridgeCore::Guard(const char* where, F&& f) {
  try {
    f();
  } catch (const std::exception& e) {
    Log(kLogWarn, std::string("Bridge: ") + where + " failed: " + CleanUtf8(e.what()));
  } catch (...) {
    Log(kLogWarn, std::string("Bridge: ") + where + " failed");
  }
}

void BridgeCore::Start(double t) {
  Guard("start", [&] {
    const Json booted = BuildEvent("BOOTED", Json::object(), t);
    booted_ = booted.dump(-1, ' ', false, Json::error_handler_t::replace);
    if (sink_) sink_(booted);
  });
}

void BridgeCore::Stop(double t) {
  Guard("stop", [&] {
    const Json shutdown = BuildEvent("SHUTDOWN", Json::object(), t);
    Send(kEventsTopic, shutdown.dump(-1, ' ', false, Json::error_handler_t::replace), false);
    if (sink_) sink_(shutdown);
  });
}

void BridgeCore::Connected() {
  connected_ = true;
  Log(kLogInfo, "MQTT connected");
  for (const auto& topic : kept_order_) {
    const auto text = kept_.find(topic);
    if (text != kept_.end()) out_.mqtt.push_back({topic, text->second, true, 1});
  }
  // BOOTED once per start, after the first connect.
  if (booted_) {
    out_.mqtt.push_back({kEventsTopic, *booted_, false, 0});
    booted_.reset();
  }
}

void BridgeCore::ConnectionLost() {
  if (connected_) Log(kLogWarn, "MQTT connection lost");
  connected_ = false;
}

void BridgeCore::OnMqtt(const std::string& topic, std::string_view payload, bool retained) {
  // A command the broker kept from earlier would act again at every connect.
  if (retained) return;
  Guard("MQTT input", [&] {
    if (topic == "action") {
      // A NUL byte breaks the ROS publisher for good; no action id has one.
      if (payload.find('\0') != std::string_view::npos) {
        Log(kLogWarn, "Bridge: an action with a NUL byte was dropped");
      } else {
        out_.actions.push_back(CleanUtf8(payload));
      }
    } else if (topic == "teleop") {
      if (auto teleop = ReadTeleop(payload)) out_.teleop.push_back(*teleop);
    } else if (topic == kRequestTopic) {
      const RpcReading reading = ReadRequest(payload);
      if (reading.kind == RpcReading::Kind::kError) Send(kResponseTopic, reading.reply, false);
      if (reading.kind == RpcReading::Kind::kCall) Answer(reading.call);
    }
  });
}

void BridgeCore::Answer(const RpcCall& call) {
  switch (methods_.Find(call.method)) {
    case Route::kLocal:
      try {
        Send(kResponseTopic, ResultReply(call.id, local_.at(call.method)(call.params)), false);
      } catch (const RpcFailure& e) {
        Send(kResponseTopic, ErrorReply(call.id, e.code, e.what()), false);
      } catch (const std::invalid_argument& e) {
        Send(kResponseTopic, ErrorReply(call.id, kInvalidParams, e.what()), false);
      } catch (const std::exception& e) {
        Send(kResponseTopic, ErrorReply(call.id, kInternalError, e.what()), false);
      }
      return;
    case Route::kNode:
      // Like an action: the id goes out as a ROS string.
      if (call.id.is_string() && call.id.get<std::string>().find('\0') != std::string::npos) {
        Send(kResponseTopic, ErrorReply(call.id, kInvalidRequest, "The id must not contain a NUL byte"), false);
        return;
      }
      out_.requests.push_back({call.method, call.params.is_null() ? "" : call.params.dump(),
                               call.id.is_string() ? call.id.get<std::string>() : ""});
      return;
    case Route::kUnknown:
      Send(kResponseTopic, ErrorReply(call.id, kMethodNotFound, "Method \"" + call.method + "\" not found"), false);
      return;
  }
}

void BridgeCore::OnRobotState(const RobotStateView& state) {
  Guard("robot state", [&] { Send("robot_state/json", RobotStateText(state), false); });
}

void BridgeCore::OnSensor(const SensorDescription& sensor) {
  Guard("sensor", [&] {
    const auto known = sensors_.find(sensor.id);
    if (known != sensors_.end() && SensorInfo(known->second) == SensorInfo(sensor)) return;
    sensors_[sensor.id] = sensor;
    Keep("sensor_infos/json", SensorInfosText(sensors_));
  });
}

void BridgeCore::OnSensorValue(const std::string& id, const Json& value, double stamp) {
  Guard("sensor value", [&] {
    readings_[id] = {value, stamp};
    const std::string text = value.is_string() ? value.get<std::string>() : SensorNumberText(value.get<double>());
    Send("sensors/" + id + "/data", text, false);
  });
}

void BridgeCore::OnActions(const std::string& prefix, const std::vector<ActionItem>& items) {
  Guard("actions", [&] {
    actions_[prefix] = items;
    Keep("actions/json", ActionsText(actions_));
  });
}

void BridgeCore::OnParams(std::string_view message) {
  Guard("params", [&] {
    if (params_.Apply(message)) Keep("params/json", params_.Text());
  });
}

void BridgeCore::OnPublish(const std::string& topic, const std::string& payload, bool retain) {
  Guard("publish", [&] {
    if (retain) {
      Keep(topic, payload);
    } else {
      Send(topic, payload, false);
    }
    if (topic != kEventsTopic) return;
    if (NestingDepth(payload) > kMaxDataDepth) return;
    const Json event = Json::parse(payload, nullptr, false);
    if (event.is_object()) Event(event);
  });
}

void BridgeCore::OnMap(const std::string& text) {
  Guard("map", [&] { Keep("map/json", text); });
}

void BridgeCore::OnPlatform(const std::string& platform) {
  platform_ = platform;
}

void BridgeCore::OnRpcMethods(const std::string& node, const std::vector<std::string>& methods) {
  Guard("rpc methods", [&] {
    methods_.SetNode(node, methods);
    Log(kLogInfo, CleanUtf8(node) + " registered " + std::to_string(methods.size()) + " methods");
  });
}

void BridgeCore::OnRpcResponse(const std::string& id, std::string_view result) {
  Guard("rpc response", [&] { Send(kResponseTopic, ResultReplyFromText(IdOf(id), result), false); });
}

void BridgeCore::OnRpcError(const std::string& id, int code, std::string_view message, std::string_view data) {
  Guard("rpc error", [&] {
    Json details;
    if (!data.empty()) {
      details = NestingDepth(data) > kMaxDataDepth ? Json() : Json::parse(data.begin(), data.end(), nullptr, false);
      if (details.is_discarded() || details.is_null()) details = Json(CleanUtf8(data));
    }
    Send(kResponseTopic, ErrorReply(IdOf(id), code, message, details), false);
  });
}

void BridgeCore::OnLog(double t, int level, std::string_view node, std::string_view message) {
  Guard("log", [&] { warnings_.Add(t, level, node, message); });
}

void BridgeCore::SendPosition(double x, double y, double heading, const PoseAttributes& attributes) {
  Guard("position", [&] { Send("position/json", PositionText(x, y, heading, attributes), false); });
}

Outbox BridgeCore::Take() {
  Outbox taken;
  std::swap(taken, out_);
  return taken;
}

void BridgeCore::Send(const std::string& topic, const std::string& payload, bool retain) {
  if (connected_) out_.mqtt.push_back({topic, payload, retain, retain ? 1 : 0});
}

void BridgeCore::Keep(const std::string& topic, const std::string& payload) {
  // An empty retained text clears the broker's copy; there is nothing to send again after that.
  if (payload.empty()) {
    kept_.erase(topic);
  } else {
    if (std::find(kept_order_.begin(), kept_order_.end(), topic) == kept_order_.end()) kept_order_.push_back(topic);
    kept_[topic] = payload;
  }
  Send(topic, payload, true);
}

void BridgeCore::Event(const Json& event) {
  if (sink_) sink_(event);
}

void BridgeCore::Log(int level, std::string text) {
  out_.logs.push_back({level, std::move(text)});
}

std::string RestartWarning(std::string_view note) {
  const Json j = NestingDepth(note) > kMaxDataDepth ? Json() : Json::parse(note.begin(), note.end(), nullptr, false);
  std::string text = "MowBite OS restarted after a crash";
  const auto number = [&](const char* key) -> std::optional<long long> {
    if (!j.is_object() || !j.contains(key) || !j[key].is_number_integer()) return std::nullopt;
    return j[key].get<long long>();
  };
  const auto code = number("exit_code"), restart = number("restart"), most = number("max_restarts");
  if (code) text += ": its container ended with exit code " + std::to_string(*code);
  if (restart && most) text += " (restart " + std::to_string(*restart) + " of " + std::to_string(*most) + ")";
  return text;
}

std::string RollbackWarning(std::string_view note) {
  const Json j = NestingDepth(note) > kMaxDataDepth ? Json() : Json::parse(note.begin(), note.end(), nullptr, false);
  const auto value = [&](const char* key, bool text) -> std::string {
    if (!j.is_object() || !j.contains(key)) return "?";
    if (text) return j[key].is_string() ? CleanUtf8(j[key].get<std::string>()) : "?";
    return j[key].is_number_integer() ? std::to_string(j[key].get<long long>()) : "?";
  };
  return "MowBite OS went back to an older image: data_dir is back at schema " + value("to", false) +
         " from its backup, the changes since the update (schema " + value("from", false) + ") are in " +
         value("backup", true);
}

}  // namespace mowbite_bridge
