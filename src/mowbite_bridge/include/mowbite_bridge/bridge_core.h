#ifndef MOWBITE_BRIDGE_BRIDGE_CORE_H
#define MOWBITE_BRIDGE_BRIDGE_CORE_H

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "mowbite_bridge/app_json.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_bridge/method_table.h"
#include "mowbite_bridge/param_table.h"
#include "mowbite_bridge/warn_log.h"

namespace mowbite_bridge {

// Prefix of the ids of calls between nodes on bridge/rpc/request (e.g. the calibration asking the planner).
constexpr char kInternalCall[] = "os/";

// A call forwarded to a node waits this long for its answer, at most this many at once; ids of a call longer than this
// as JSON are refused.
constexpr double kCallWaitS = 900;
constexpr std::size_t kMaxOpenCalls = 512;
constexpr std::size_t kMaxIdBytes = 1024;
// Correlation data of an MQTT 5 request longer than this is refused.
constexpr std::size_t kMaxCorrelationBytes = 1024;
// Changes of rpc.methods are collected this long from the first one, then told in one METHODS_CHANGED.
constexpr double kMethodsCollectS = 1.0;
// A publisher of methods or sensors the graph lacks in this many checks in a row is gone.
constexpr int kProviderChecks = 2;
// Retained {"online": true|false} of the bridge; false is also its last will. robot_state is old while it says false.
constexpr char kBridgeStatusTopic[] = "mowbite/bridge/status";
// Retained by mowbite-host: {online, methods, ...}; the bridge stays silent on its methods while it is online.
constexpr char kHostStatusTopic[] = "mowbite/host/status";

std::string BridgeStatusText(bool online);

struct MqttMessage {
  std::string topic;
  std::string payload;
  bool retain = false;
  int qos = 0;
  // MQTT 5 correlation data of the request this answers, sent back unchanged.
  std::optional<std::string> correlation;
};

// The MQTT 5 properties of a request that say where its answer goes; both empty over 3.1.1.
struct ResponseRoute {
  std::optional<std::string> topic;
  std::optional<std::string> correlation;
};

// True for rpc/response/<token> with a token of 1 to 64 of A-Z, a-z, 0-9, '.', '_' and '-'.
bool ValidResponseTopic(std::string_view topic);

// The MQTT version the bridge connects with: 5 until the broker refuses that protocol, then 3 (3.1.1) for as long as
// the process runs.
class MqttVersion {
 public:
  enum class Failure {
    // No TCP connection to the broker, or a refusal for another reason: try again with the same version.
    kNetwork,
    // A CONNACK refusing the protocol: 1 of 3.1.1 or 0x84 of 5.
    kRefused,
    // The broker took the TCP connection and ended it before any CONNACK.
    kDropped,
  };

  // From the return code of the failed connect and whether the broker takes TCP connections right after.
  static Failure Classify(int return_code, bool broker_reachable);

  int version() const {
    return version_;
  }
  // True when this failure switches to 3: a refusal at once, an end before the CONNACK the second time in a row.
  bool OnFailure(Failure failure);
  void OnConnected();

 private:
  int version_ = 5;
  int dropped_ = 0;
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
  // Monotonic seconds for the open calls; the steady clock without one.
  std::function<double()> clock = nullptr;
};

// The bridge between the nodes and MQTT without ROS or MQTT client; retained texts are resent after every connect.
class BridgeCore {
 public:
  using LocalMethod = std::function<Json(const Json& params)>;
  using EventSink = std::function<void(const Json& event)>;

  explicit BridgeCore(BridgeConfig config);

  // The topics to subscribe to after every connect, the status of the host among them.
  static const std::vector<std::string>& CommandTopics();

  // A method answered in the bridge; it may throw RpcFailure, or std::invalid_argument for -32602.
  void AddLocalMethod(const std::string& name, LocalMethod method);
  // Gets every event that goes to events/json, for the history and the track.
  void SetEventSink(EventSink sink);

  // BOOTED into the sink now and live after the first connect, with the same id.
  void Start(double t);
  // SHUTDOWN live and into the sink, the status of the bridge offline.
  void Stop(double t);
  // mqtt_version for system.info: 5 or 3.
  void Connected(int mqtt_version = 5);
  void ConnectionLost();
  bool connected() const {
    return connected_;
  }

  // A message from the broker; retained marks one the broker kept, not one sent live. A request with a response topic
  // is answered only there, with its correlation data; one with an invalid response topic is refused on rpc/response.
  void OnMqtt(const std::string& topic, std::string_view payload, bool retained, const ResponseRoute& route = {});

  void OnRobotState(const RobotStateView& state);
  // provider names the publisher the sensor came from (OnProviders); empty: not known, the sensor stays for good.
  void OnSensor(const SensorDescription& sensor, const std::string& provider = "");
  void OnSensorValue(const std::string& id, const Json& value, double stamp);
  void OnActions(const std::string& prefix, const std::vector<ActionItem>& items);
  void OnParams(std::string_view message);
  void OnPublish(const std::string& topic, const std::string& payload, bool retain);
  void OnMap(const std::string& text);
  // From hw/capabilities: the platform, stop_release and blade_height of system.info.
  void OnPlatform(const std::string& platform, const Json& stop_release = Json(), const Json& blade_height = Json());
  // provider as for OnSensor. Another provider under the name of a node is a new instance of it: the calls its old
  // one still owes an answer get -32601.
  void OnRpcMethods(const std::string& node, const std::vector<std::string>& methods,
                    const std::string& provider = "");
  // The publishers of methods and sensors the graph has now, checked once a second. One that is missing in
  // kProviderChecks checks in a row is gone: its methods leave rpc.methods, the calls it still owes an answer get
  // -32601, its sensors leave sensor_infos/json and sensors.list.
  void OnProviders(const std::set<std::string>& alive);
  // A few times a second: METHODS_CHANGED once rpc.methods has been another list for kMethodsCollectS, held back
  // until the broker is connected. t is the time of the event.
  void Tick(double t);
  // Answers of the nodes for MQTT under the id the client sent, to where its request wanted them; calls one node makes
  // of another (ids beginning with kInternalCall) stay in ROS, an answer to a call no longer open or to an id the
  // bridge did not give is dropped.
  void OnRpcResponse(const std::string& id, std::string_view result);
  void OnRpcError(const std::string& id, int code, std::string_view message, std::string_view data);
  void OnLog(double t, int level, std::string_view node, std::string_view message);
  // position/json, from the node's timer.
  void SendPosition(double x, double y, double heading, const PoseAttributes& attributes);

  Outbox Take();

 private:
  // Where the answer to a request goes.
  struct Reply {
    std::string topic;
    std::optional<std::string> correlation;
  };

  template <typename F>
  void Guard(const char* where, F&& f);
  void Request(std::string_view payload, const ResponseRoute& route);
  void OnHostStatus(std::string_view payload);
  void Answer(const RpcCall& call, const Reply& to);
  // -32600 with data {event REQUEST_REFUSED, reason, ...}.
  std::string RefuseRequest(const Json& id, const std::string& reason, const std::string& message, Json fields = {});
  void Send(const std::string& topic, const std::string& payload, bool retain);
  void SendReply(const Reply& to, const std::string& payload);
  // A result reply, or kInternalError for one larger than kMaxReplyBytes.
  void SendResult(const Reply& to, const Json& id, const std::string& payload);
  void Keep(const std::string& topic, const std::string& payload);
  void Event(const Json& event);
  void Log(int level, std::string text);
  double Now() const;
  // The id for the node of a call with this id of the client.
  std::string Open(const Json& id, const std::string& method, const Reply& to);
  // The client's id of the call with this id for the node and where its answer goes; false when it is not open.
  bool Close(const std::string& node_id, Json* id, Reply* to);
  // Starts the time to collect when rpc.methods is not the list told last.
  void NoteMethods();
  // -32601 to every open call of one of these methods.
  void FailCalls(const std::set<std::string>& methods);

  BridgeConfig config_;
  bool connected_ = false;
  int mqtt_version_ = 5;
  std::string platform_;
  Json stop_release_;
  Json blade_height_;
  // current_state and current_sub_state of the last robot_state, for refused requests.
  std::string state_;
  std::string sub_state_;
  MethodTable methods_;
  // rpc.methods as METHODS_CHANGED told it last, sorted; the bridge's own methods count as told.
  std::vector<std::string> told_methods_;
  // Since when rpc.methods is another list, by the clock of the open calls.
  std::optional<double> methods_changed_;
  // The provider of each node with methods and of each sensor, and the checks in a row a provider was missing in.
  std::map<std::string, std::string> node_provider_;
  std::map<std::string, std::string> sensor_provider_;
  std::map<std::string, int> missed_;
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
  // The calls at the nodes by their number: the client's id, the method, since when and where the answer goes; the
  // node gets tag_ + number.
  struct OpenCall {
    Json id;
    std::string method;
    double since = 0;
    Reply to;
  };
  std::map<uint64_t, OpenCall> open_;
  std::string tag_;
  uint64_t next_call_ = 0;
  // Each problem of an event against the catalog is logged once.
  std::set<std::string> event_problems_;
  Outbox out_;
};

// The WARN line for <data_dir>/container_exit.json, written by the launch before it starts the container again.
std::string RestartWarning(std::string_view note);

// The WARN line for <data_dir>/data_rollback.json: the launch took the data back to an older schema for an older image,
// the changes since the update wait in a backup.
std::string RollbackWarning(std::string_view note);

}  // namespace mowbite_bridge

#endif  // MOWBITE_BRIDGE_BRIDGE_CORE_H
