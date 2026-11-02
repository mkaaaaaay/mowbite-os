#ifndef MOWBITE_BRIDGE_BRIDGE_CORE_H
#define MOWBITE_BRIDGE_BRIDGE_CORE_H

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "mowbite_bridge/app_json.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_bridge/method_table.h"
#include "mowbite_bridge/param_table.h"
#include "mowbite_bridge/warn_log.h"

namespace mowbite_bridge {

struct MqttMessage {
  std::string topic;
  std::string payload;
  bool retain = false;
  int qos = 0;
};

// A call for the nodes on bridge/rpc/request: params as JSON text ("" without), id "" without.
struct RpcForward {
  std::string method;
  std::string params;
  std::string id;
};

struct LogLine {
  int level = kLogInfo;
  std::string text;
};

// Everything the node is to send since the last Take.
struct Outbox {
  std::vector<MqttMessage> mqtt;
  std::vector<std::string> actions;
  std::vector<Teleop> teleop;
  std::vector<RpcForward> requests;
  std::vector<LogLine> logs;
};

struct BridgeConfig {
  std::string software_version;
  std::string params_json = "{}";
};

// The bridge between the nodes and MQTT without ROS and without an MQTT client: takes what arrives on either side and
// collects what to send. Retained texts are kept and sent again after every connect; while there is no connection,
// everything else is dropped. No input throws: a failure becomes a WARN line.
class BridgeCore {
 public:
  using LocalMethod = std::function<Json(const Json& params)>;
  using EventSink = std::function<void(const Json& event)>;

  explicit BridgeCore(BridgeConfig config);

  // The topics to subscribe to after every connect.
  static const std::vector<std::string>& CommandTopics();

  // A method answered in the bridge; it may throw RpcFailure, or std::invalid_argument for -32602.
  void AddLocalMethod(const std::string& name, LocalMethod method);
  // Gets every event that goes to events/json, for the history and the track.
  void SetEventSink(EventSink sink);

  // BOOTED into the sink now and live after the first connect, with the same id.
  void Start(double t);
  // SHUTDOWN live and into the sink.
  void Stop(double t);
  void Connected();
  void ConnectionLost();
  bool connected() const {
    return connected_;
  }

  // A message from the broker; retained marks one the broker kept, not one sent live.
  void OnMqtt(const std::string& topic, std::string_view payload, bool retained);

  void OnRobotState(const RobotStateView& state);
  void OnSensor(const SensorDescription& sensor);
  void OnSensorValue(const std::string& id, const Json& value, double stamp);
  void OnActions(const std::string& prefix, const std::vector<ActionItem>& items);
  void OnParams(std::string_view message);
  void OnPublish(const std::string& topic, const std::string& payload, bool retain);
  void OnMap(const std::string& text);
  void OnPlatform(const std::string& platform);
  void OnRpcMethods(const std::string& node, const std::vector<std::string>& methods);
  void OnRpcResponse(const std::string& id, std::string_view result);
  void OnRpcError(const std::string& id, int code, std::string_view message, std::string_view data);
  void OnLog(double t, int level, std::string_view node, std::string_view message);
  // position/json, from the node's timer.
  void SendPosition(double x, double y, double heading, const PoseAttributes& attributes);

  Outbox Take();

 private:
  template <typename F>
  void Guard(const char* where, F&& f);
  void Answer(const RpcCall& call);
  void Send(const std::string& topic, const std::string& payload, bool retain);
  void Keep(const std::string& topic, const std::string& payload);
  void Event(const Json& event);
  void Log(int level, std::string text);

  BridgeConfig config_;
  bool connected_ = false;
  std::string platform_;
  MethodTable methods_;
  std::map<std::string, LocalMethod> local_;
  EventSink sink_;
  ParamTable params_;
  WarnLog warnings_;
  std::map<std::string, SensorDescription> sensors_;
  std::map<std::string, SensorReading> readings_;
  std::map<std::string, std::vector<ActionItem>> actions_;
  // Retained texts in the order they first came, sent again after each connect.
  std::vector<std::string> kept_order_;
  std::map<std::string, std::string> kept_;
  std::optional<std::string> booted_;
  Outbox out_;
};

// The WARN line for <data_dir>/container_exit.json, written by the launch before it starts the container again.
std::string RestartWarning(std::string_view note);

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_BRIDGE_CORE_H
