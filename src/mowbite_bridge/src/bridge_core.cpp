#include "mowbite_bridge/bridge_core.h"

#include <algorithm>
#include <chrono>
#include <cstdio>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/event_builder.h"
#include "mowbite_bridge/text.h"

namespace mowbite_bridge {

namespace {

constexpr char kRequestTopic[] = "rpc/request";
constexpr char kResponseTopic[] = "rpc/response";
constexpr char kRequestRefused[] = "REQUEST_REFUSED";
constexpr std::size_t kMaxTokenBytes = 64;
// Return codes of a failed connect of the Paho client: its own failure, the CONNACK refusals of the protocol.
constexpr int kClientFailure = -1;
constexpr int kRefusedProtocol311 = 1;
constexpr int kRefusedProtocol5 = 0x84;
constexpr char kEventsTopic[] = "events/json";
constexpr std::size_t kMaxDataDepth = 64;
// The host offers a handful of methods: more, or longer names, would only bloat rpc.methods and METHODS_CHANGED.
constexpr std::size_t kMaxHostMethods = 32;
constexpr std::size_t kMaxMethodBytes = 64;

}  // namespace

std::string BridgeStatusText(bool online) {
  return Json{{"online", online}}.dump();
}

bool ValidResponseTopic(std::string_view topic) {
  const std::string_view prefix = "rpc/response/";
  if (topic.substr(0, prefix.size()) != prefix) return false;
  const std::string_view token = topic.substr(prefix.size());
  if (token.empty() || token.size() > kMaxTokenBytes) return false;
  return std::all_of(token.begin(), token.end(), [](char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '.' || c == '_' ||
           c == '-';
  });
}

MqttVersion::Failure MqttVersion::Classify(int return_code, bool broker_reachable) {
  if (return_code == kRefusedProtocol311 || return_code == kRefusedProtocol5) return Failure::kRefused;
  if (return_code == kClientFailure && broker_reachable) return Failure::kDropped;
  return Failure::kNetwork;
}

bool MqttVersion::OnFailure(Failure failure) {
  if (version_ != 5) return false;
  if (failure == Failure::kNetwork) {
    dropped_ = 0;
    return false;
  }
  if (failure == Failure::kDropped && ++dropped_ < 2) return false;
  version_ = 3;
  return true;
}

void MqttVersion::OnConnected() {
  dropped_ = 0;
}

BridgeCore::BridgeCore(BridgeConfig config)
    : config_(std::move(config)), params_(config_.params_json), tag_(RandomId(8) + "/") {
  local_["rpc.ping"] = [](const Json&) { return Json("pong"); };
  local_["rpc.methods"] = [this](const Json&) { return Json(methods_.All()); };
  local_["system.info"] = [this](const Json&) {
    return SystemInfo(config_.software_version, platform_, mqtt_version_, stop_release_, blade_height_);
  };
  // The catalog of the interface as JSON Schema; system.info.api tells whether a client knows it already.
  local_["system.api"] = [](const Json&) {
    return Json{{"api", mowbite_api::kApi}, {"api_break", mowbite_api::kApiBreak}, {"json_schema", mowbite_api::Schema()}};
  };
  local_["sensors.list"] = [this](const Json& params) { return SensorList(sensors_, readings_, params); };
  local_["logs.recent"] = [this](const Json& params) { return warnings_.Recent(params); };
  for (const auto& [name, method] : local_) methods_.AddLocal(name);
  told_methods_ = methods_.All();
  // The app's retained texts come first after a connect, in this order; sensors and map only once there are some.
  kept_order_ = {"capabilities/json", "version/json", "params/json",     "sensor_infos/json",
                 "actions/json",      "map/json",     kBridgeStatusTopic};
  Keep("capabilities/json", CapabilitiesText());
  Keep("version/json", VersionText(config_.software_version));
  Keep("params/json", params_.Text());
  Keep("actions/json", ActionsText(actions_));
  Keep(kBridgeStatusTopic, BridgeStatusText(true));
}

const std::vector<std::string>& BridgeCore::CommandTopics() {
  static const std::vector<std::string> topics = {"action", "teleop", kRequestTopic, kHostStatusTopic};
  return topics;
}

void BridgeCore::AddLocalMethod(const std::string& name, LocalMethod method) {
  local_[name] = std::move(method);
  methods_.AddLocal(name);
  // There from the start, no change to tell.
  const auto place = std::lower_bound(told_methods_.begin(), told_methods_.end(), name);
  if (place == told_methods_.end() || *place != name) told_methods_.insert(place, name);
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
    Keep(kBridgeStatusTopic, BridgeStatusText(false));
  });
}

void BridgeCore::Connected(int mqtt_version) {
  connected_ = true;
  mqtt_version_ = mqtt_version;
  Log(kLogInfo, mqtt_version == 5 ? "MQTT connected" : "MQTT connected with 3.1.1");
  for (const auto& topic : kept_order_) {
    const auto text = kept_.find(topic);
    if (text != kept_.end()) out_.mqtt.push_back({topic, text->second, true, 1, std::nullopt});
  }
  // BOOTED once per start, after the first connect.
  if (booted_) {
    out_.mqtt.push_back({kEventsTopic, *booted_, false, 0, std::nullopt});
    booted_.reset();
  }
}

void BridgeCore::ConnectionLost() {
  if (connected_) Log(kLogWarn, "MQTT connection lost");
  connected_ = false;
}

void BridgeCore::OnMqtt(const std::string& topic, std::string_view payload, bool retained, const ResponseRoute& route) {
  // A status, retained or not.
  if (topic == kHostStatusTopic) {
    Guard("host status", [&] { OnHostStatus(payload); });
    return;
  }
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
      Request(payload, route);
    }
  });
}

void BridgeCore::Request(std::string_view payload, const ResponseRoute& route) {
  const RpcReading reading = ReadRequest(payload);
  // meta.* and ext.* stay unanswered, whatever their properties.
  if (reading.kind == RpcReading::Kind::kSilent) return;
  Reply to{kResponseTopic, std::nullopt};
  if (route.topic) {
    // Checked before anything else: no request puts an answer on map/json or robot_state/json.
    if (!ValidResponseTopic(*route.topic)) {
      Send(kResponseTopic,
           RefuseRequest(reading.call.id, "RESPONSE_TOPIC",
                         "The response topic is not rpc/response/<token> with a token of 1 to 64 of [A-Za-z0-9._-]"),
           false);
      return;
    }
    to.topic = *route.topic;
    if (route.correlation && route.correlation->size() > kMaxCorrelationBytes) {
      SendReply(to, RefuseRequest(reading.call.id, "CORRELATION_DATA",
                                  "The correlation data is longer than " + std::to_string(kMaxCorrelationBytes) +
                                      " bytes",
                                  {{"limit", kMaxCorrelationBytes}}));
      return;
    }
    to.correlation = route.correlation;
  }
  if (reading.kind == RpcReading::Kind::kError) {
    SendReply(to, reading.reply);
  } else {
    Answer(reading.call, to);
  }
}

void BridgeCore::OnHostStatus(std::string_view payload) {
  const Json status =
      NestingDepth(payload) > kMaxDataDepth ? Json() : Json::parse(payload.begin(), payload.end(), nullptr, false);
  std::vector<std::string> methods;
  if (status.is_object() && status.value("online", Json()) == Json(true) && status.contains("methods") &&
      status["methods"].is_array()) {
    for (const auto& method : status["methods"]) {
      if (methods.size() >= kMaxHostMethods) break;
      if (method.is_string() && method.get_ref<const std::string&>().size() <= kMaxMethodBytes) {
        methods.push_back(method.get<std::string>());
      }
    }
  }
  const std::size_t before = methods_.All().size();
  methods_.SetHost(methods);
  if (methods_.All().size() != before) {
    Log(kLogInfo, "mowbite-host " + std::string(methods.empty() ? "is offline" : "answers its methods") + ": " +
                      std::to_string(methods_.All().size()) + " methods in all");
  }
  NoteMethods();
}

void BridgeCore::Answer(const RpcCall& call, const Reply& to) {
  switch (methods_.Find(call.method)) {
    case Route::kLocal:
      try {
        SendResult(to, call.id, ResultReply(call.id, local_.at(call.method)(call.params)));
      } catch (const RpcFailure& e) {
        SendReply(to, ErrorReply(call.id, e.code, e.what()));
      } catch (const std::invalid_argument& e) {
        SendReply(to, ErrorReply(call.id, kInvalidParams, e.what()));
      } catch (const std::exception& e) {
        SendReply(to, ErrorReply(call.id, kInternalError, e.what()));
      }
      return;
    case Route::kNode: {
      // The node gets an id of the bridge, the answer goes out with the client's. Without an id it gets none, unless
      // the answer has a response topic to go to.
      if (call.id.dump().size() > kMaxIdBytes) {
        SendReply(to, ErrorReply(call.id, kInvalidRequest,
                                 "The id is longer than " + std::to_string(kMaxIdBytes) + " bytes"));
        return;
      }
      const bool open = !call.id.is_null() || to.topic != kResponseTopic;
      const std::string id = open ? Open(call.id, call.method, to) : "";
      out_.requests.push_back({call.method, call.params.is_null() ? "" : call.params.dump(), id});
      return;
    }
    case Route::kHost: return;
    case Route::kUnknown:
      SendReply(to, ErrorReply(call.id, kMethodNotFound, "Method \"" + call.method + "\" not found"));
      return;
  }
}

std::string BridgeCore::RefuseRequest(const Json& id, const std::string& reason, const std::string& message,
                                      Json fields) {
  const mowbite_api::Rating rating = mowbite_api::RateReason(kRequestRefused, reason);
  Json data = {{"event", kRequestRefused}, {"state", state_}, {"sub_state", sub_state_}, {"reason", reason},
               {"message", message},       {"severity", rating.severity}, {"resume", rating.resume}};
  for (auto& [key, value] : fields.items()) data[key] = value;
  return ErrorReply(id, kInvalidRequest, message, data);
}

void BridgeCore::OnRobotState(const RobotStateView& state) {
  state_ = state.current_state;
  sub_state_ = state.current_sub_state;
  Guard("robot state", [&] { Send("robot_state/json", RobotStateText(state), false); });
}

void BridgeCore::OnSensor(const SensorDescription& sensor, const std::string& provider) {
  Guard("sensor", [&] {
    sensor_provider_[sensor.id] = provider;
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

void BridgeCore::OnPlatform(const std::string& platform, const Json& stop_release, const Json& blade_height) {
  platform_ = platform;
  stop_release_ = stop_release;
  blade_height_ = blade_height;
}

void BridgeCore::OnRpcMethods(const std::string& node, const std::vector<std::string>& methods,
                              const std::string& provider) {
  Guard("rpc methods", [&] {
    const auto known = node_provider_.find(node);
    if (known != node_provider_.end() && !known->second.empty() && !provider.empty() && known->second != provider) {
      Log(kLogWarn, CleanUtf8(node) + " registered anew from another process");
      FailCalls(methods_.Of(node));
    }
    // Calls of methods the node no longer offers get no answer from it.
    std::set<std::string> dropped = methods_.Of(node);
    for (const auto& method : methods) dropped.erase(method);
    FailCalls(dropped);
    methods_.SetNode(node, methods);
    if (methods.empty()) {
      node_provider_.erase(node);
    } else {
      node_provider_[node] = provider;
    }
    NoteMethods();
    Log(kLogInfo, CleanUtf8(node) + " registered " + std::to_string(methods.size()) + " methods");
  });
}

void BridgeCore::OnProviders(const std::set<std::string>& alive) {
  Guard("providers", [&] {
    std::map<std::string, int> missed;
    std::set<std::string> gone;
    const auto check = [&](const std::string& provider) {
      if (provider.empty() || alive.count(provider) || missed.count(provider)) return;
      const auto before = missed_.find(provider);
      const int checks = (before == missed_.end() ? 0 : before->second) + 1;
      missed[provider] = checks;
      if (checks >= kProviderChecks) gone.insert(provider);
    };
    for (const auto& [node, provider] : node_provider_) check(provider);
    for (const auto& [id, provider] : sensor_provider_) check(provider);
    missed_ = std::move(missed);
    if (gone.empty()) return;

    std::set<std::string> methods;
    for (auto node = node_provider_.begin(); node != node_provider_.end();) {
      if (!gone.count(node->second)) {
        ++node;
        continue;
      }
      const std::set<std::string>& its = methods_.Of(node->first);
      Log(kLogWarn, CleanUtf8(node->first) + " is gone, its " + std::to_string(its.size()) +
                        " methods are no longer offered");
      methods.insert(its.begin(), its.end());
      methods_.SetNode(node->first, {});
      node = node_provider_.erase(node);
    }
    FailCalls(methods);
    NoteMethods();

    std::string sensors;
    for (auto sensor = sensor_provider_.begin(); sensor != sensor_provider_.end();) {
      if (!gone.count(sensor->second)) {
        ++sensor;
        continue;
      }
      if (sensors_.erase(sensor->first) != 0) sensors += (sensors.empty() ? "" : ", ") + CleanUtf8(sensor->first);
      readings_.erase(sensor->first);
      sensor = sensor_provider_.erase(sensor);
    }
    if (sensors.empty()) return;
    Log(kLogWarn, "Sensors no longer reported, their process is gone: " + sensors);
    Keep("sensor_infos/json", SensorInfosText(sensors_));
  });
}

void BridgeCore::Tick(double t) {
  Guard("calls", [&] { ExpireCalls(Now()); });
  Guard("methods", [&] {
    if (!methods_changed_ || !connected_ || Now() - *methods_changed_ < kMethodsCollectS) return;
    methods_changed_.reset();
    const std::vector<std::string> now = methods_.All();
    Json added = Json::array(), removed = Json::array();
    for (const auto& method : now) {
      if (!std::binary_search(told_methods_.begin(), told_methods_.end(), method)) added.push_back(CleanUtf8(method));
    }
    for (const auto& method : told_methods_) {
      if (!std::binary_search(now.begin(), now.end(), method)) removed.push_back(CleanUtf8(method));
    }
    told_methods_ = now;
    if (added.empty() && removed.empty()) return;
    const Json event = BuildEvent(
        "METHODS_CHANGED", {{"added", added}, {"removed", removed}, {"severity", "info"}, {"resume", "auto"}}, t);
    Send(kEventsTopic, event.dump(), false);
    Event(event);
  });
}

void BridgeCore::OnRpcResponse(const std::string& id, std::string_view result) {
  if (id.rfind(kInternalCall, 0) == 0) return;
  Guard("rpc response", [&] {
    Json client_id;
    Reply to;
    if (Close(id, &client_id, &to)) SendResult(to, client_id, ResultReplyFromText(client_id, result));
  });
}

void BridgeCore::OnRpcError(const std::string& id, int code, std::string_view message, std::string_view data) {
  if (id.rfind(kInternalCall, 0) == 0) return;
  Guard("rpc error", [&] {
    Json client_id;
    Reply to;
    if (!Close(id, &client_id, &to)) return;
    Json details;
    if (!data.empty()) {
      details = NestingDepth(data) > kMaxDataDepth ? Json() : Json::parse(data.begin(), data.end(), nullptr, false);
      if (details.is_discarded() || details.is_null()) details = Json(CleanUtf8(data));
    }
    SendReply(to, ErrorReply(client_id, code, message, details));
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
  if (connected_) out_.mqtt.push_back({topic, payload, retain, retain ? 1 : 0, std::nullopt});
}

void BridgeCore::SendReply(const Reply& to, const std::string& payload) {
  if (connected_) out_.mqtt.push_back({to.topic, payload, false, 0, to.correlation});
}

void BridgeCore::SendResult(const Reply& to, const Json& id, const std::string& payload) {
  if (payload.size() <= kMaxReplyBytes) {
    SendReply(to, payload);
    return;
  }
  SendReply(to, ErrorReply(id, kInternalError,
                           "The answer has " + std::to_string(payload.size()) + " bytes, at most " +
                               std::to_string(kMaxReplyBytes) + " go out"));
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
  // Against the catalog, never in the way of the event: each problem once as an ERROR in the log.
  if (event.contains("type") && event["type"].is_string()) {
    for (const auto& problem : mowbite_api::CheckEvent(event["type"].get<std::string>(), event, false)) {
      if (event_problems_.size() < 256 && event_problems_.insert(problem).second) {
        Log(kLogError, "Bridge: event not as in the catalog: " + CleanUtf8(problem));
      }
    }
  }
  if (sink_) sink_(event);
}

void BridgeCore::Log(int level, std::string text) {
  out_.logs.push_back({level, std::move(text)});
}

double BridgeCore::Now() const {
  if (config_.clock) return config_.clock();
  using Seconds = std::chrono::duration<double>;
  return Seconds(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string BridgeCore::Open(const Json& id, const std::string& method, const Reply& to) {
  const double now = Now();
  ExpireCalls(now);
  if (open_.size() >= kMaxOpenCalls) {
    Log(kLogWarn, "Bridge: " + std::to_string(kMaxOpenCalls) + " calls are open, the oldest (" +
                      CleanUtf8(open_.begin()->second.method) + ") ends without the answer of its node");
    SendReply(open_.begin()->second.to,
              ErrorReply(open_.begin()->second.id, kInternalError,
                         "Too many calls are open: the answer of the node will not come through"));
    open_.erase(open_.begin());
  }
  const uint64_t number = ++next_call_;
  open_[number] = {id, method, now, to};
  return tag_ + std::to_string(number);
}

void BridgeCore::NoteMethods() {
  if (methods_.All() == told_methods_) {
    methods_changed_.reset();
  } else if (!methods_changed_) {
    methods_changed_ = Now();
  }
}

void BridgeCore::ExpireCalls(double now) {
  while (!open_.empty() && now - open_.begin()->second.since > kCallWaitS) {
    SendReply(open_.begin()->second.to,
              ErrorReply(open_.begin()->second.id, kInternalError,
                         "No answer of the node within " + std::to_string(static_cast<int>(kCallWaitS)) + " s"));
    open_.erase(open_.begin());
  }
}

void BridgeCore::FailCalls(const std::set<std::string>& methods) {
  for (auto call = open_.begin(); call != open_.end();) {
    if (!methods.count(call->second.method)) {
      ++call;
      continue;
    }
    SendReply(call->second.to, ErrorReply(call->second.id, kMethodNotFound,
                                          "The node that answers \"" + call->second.method + "\" is gone"));
    call = open_.erase(call);
  }
}

bool BridgeCore::Close(const std::string& node_id, Json* id, Reply* to) {
  if (node_id.empty()) {
    *id = nullptr;
    *to = {kResponseTopic, std::nullopt};
    return true;
  }
  // Not an id of this bridge: a call of a test or a tool straight over ROS, its answer is not for MQTT.
  if (node_id.rfind(tag_, 0) != 0) return false;
  std::optional<uint64_t> number;
  if (node_id.size() > tag_.size() && node_id.size() <= tag_.size() + 19 &&
      node_id.find_first_not_of("0123456789", tag_.size()) == std::string::npos) {
    number = std::stoull(node_id.substr(tag_.size()));
  }
  const auto open = number ? open_.find(*number) : open_.end();
  if (open == open_.end()) {
    Log(kLogWarn, "Bridge: an answer to " + CleanUtf8(node_id) + " was dropped, that call is not open (any more)");
    return false;
  }
  *id = std::move(open->second.id);
  *to = std::move(open->second.to);
  open_.erase(open);
  return true;
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
  // Written by the launch when the watch ended the container for a node (mowbite_watch).
  const Json stalled = j.is_object() ? j.value("stalled", Json()) : Json();
  if (stalled.is_object() && stalled.value("node", Json()).is_string()) {
    const auto tenths = [&](const char* key) {
      const Json value = stalled.value(key, Json());
      if (!value.is_number()) return std::string("?");
      char number[32];
      std::snprintf(number, sizeof(number), "%.1f", value.get<double>());
      return std::string(number);
    };
    text += ", ended by the watch: " + CleanUtf8(stalled["node"].get<std::string>()) + " gave no sign for " +
            tenths("seconds") + " s (limit " + tenths("limit") + " s)";
  }
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
