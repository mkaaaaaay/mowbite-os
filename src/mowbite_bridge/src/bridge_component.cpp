#include <atomic>
#include <cerrno>
#include <chrono>
#include <deque>
#include <cmath>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <netdb.h>
#include <poll.h>
#include <mowbite_component/node_options.h>
#include <sys/socket.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <mqtt/async_client.h>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/waitable.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include "mowbite_bridge/bridge_core.h"
#include "mowbite_bridge/event_log.h"
#include "mowbite_bridge/pose_window.h"
#include "mowbite_bridge/system_stats.h"
#include "mowbite_bridge/track.h"
#include "mowbite_msgs/msg/action_list.hpp"
#include "mowbite_msgs/msg/hardware_capabilities.hpp"
#include "mowbite_msgs/msg/mqtt_publish.hpp"
#include "mowbite_msgs/msg/robot_pose.hpp"
#include "mowbite_msgs/msg/robot_state.hpp"
#include "mowbite_msgs/msg/rpc_error.hpp"
#include "mowbite_msgs/msg/rpc_methods.hpp"
#include "mowbite_msgs/msg/rpc_request.hpp"
#include "mowbite_msgs/msg/rpc_response.hpp"
#include "mowbite_msgs/msg/sensor_spec.hpp"
#include "mowbite_msgs/msg/sensor_value.hpp"
#include "mowbite_watch/attach.h"

namespace mowbite_bridge {

namespace {

using namespace std::chrono_literals;
namespace msg = mowbite_msgs::msg;

constexpr auto kRetryCheck = 1s;
constexpr std::size_t kMaxInbox = 1000;
constexpr double kRetrySeconds = 5.0;
constexpr auto kPositionPeriod = 150ms;
constexpr auto kSavePeriod = 30s;
constexpr auto kRestartNoteDelay = 1s;
constexpr auto kProvidersPeriod = 1s;
constexpr auto kTickPeriod = 250ms;
constexpr char kMethodsTopic[] = "bridge/rpc/methods";
constexpr char kSensorInfoTopic[] = "bridge/sensor_info";
constexpr int kReachableTimeoutMs = 1000;
constexpr char kClientId[] = "mowbite_bridge";
constexpr char kRestartNote[] = "container_exit.json";
// Left by tron.launch.py after it took data_dir back for an older image.
constexpr char kRollbackNote[] = "data_rollback.json";

double WallNow() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

rclcpp::QoS Latched(std::size_t depth) {
  return rclcpp::QoS(depth).reliable().transient_local();
}

// Distances count as unknown for the app.
Meaning MeaningOf(uint8_t meaning) {
  switch (meaning) {
    case msg::SensorSpec::MEANING_VOLTAGE: return Meaning::kVoltage;
    case msg::SensorSpec::MEANING_CURRENT: return Meaning::kCurrent;
    case msg::SensorSpec::MEANING_TEMPERATURE: return Meaning::kTemperature;
    case msg::SensorSpec::MEANING_REVOLUTIONS: return Meaning::kRevolutions;
    default: return Meaning::kUnknown;
  }
}

// The GID of a publisher as the core's name for it; empty for one without (all zero).
template <typename Bytes>
std::string ProviderName(const Bytes& gid) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string name;
  bool any = false;
  for (const uint8_t byte : gid) {
    any = any || byte != 0;
    name += kDigits[byte >> 4];
    name += kDigits[byte & 15];
  }
  return any ? name : "";
}

std::optional<double> Limit(double value) {
  if (std::isfinite(value)) return value;
  return std::nullopt;
}

SensorDescription Describe(const msg::SensorSpec& spec) {
  SensorDescription s;
  s.id = spec.id;
  s.name = spec.name;
  s.is_text = spec.kind == msg::SensorSpec::KIND_TEXT;
  s.meaning = MeaningOf(spec.meaning);
  s.unit = spec.unit;
  if (std::isfinite(spec.min) && std::isfinite(spec.max)) {
    s.min = spec.min;
    s.max = spec.max;
  }
  s.critical_low = Limit(spec.critical_low);
  s.critical_high = Limit(spec.critical_high);
  return s;
}

RobotStateView View(const msg::RobotState& m) {
  RobotStateView v;
  v.battery_percentage = m.battery_percentage;
  v.gps_percentage = m.gps_percentage;
  v.current_state = m.current_state;
  v.current_sub_state = m.current_sub_state;
  v.current_area = m.current_area;
  v.current_path = m.current_path;
  v.current_path_index = m.current_path_index;
  v.task_index = m.task_index;
  v.task_count = m.task_count;
  v.emergency = m.emergency;
  v.emergency_reason = m.emergency_reason;
  v.is_charging = m.is_charging;
  v.rain_detected = m.rain_detected;
  v.x = m.pose_x;
  v.y = m.pose_y;
  v.heading = m.pose_heading;
  v.pos_accuracy = m.pos_accuracy;
  v.heading_accuracy = m.heading_accuracy;
  v.heading_valid = m.heading_valid;
  return v;
}

std::string JoinPath(const std::string& dir, const std::string& name) {
  return dir.empty() ? "" : dir + "/" + name;
}

// Whether the broker takes a TCP connection within kReachableTimeoutMs; it tells a broker that dropped a connect from
// one that cannot be reached.
bool TakesTcp(const std::string& host, int64_t port) {
  addrinfo hints{};
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* found = nullptr;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &found) != 0) return false;
  bool reached = false;
  for (addrinfo* a = found; a && !reached; a = a->ai_next) {
    const int fd = socket(a->ai_family, a->ai_socktype | SOCK_NONBLOCK | SOCK_CLOEXEC, a->ai_protocol);
    if (fd < 0) continue;
    if (connect(fd, a->ai_addr, a->ai_addrlen) == 0) {
      reached = true;
    } else if (errno == EINPROGRESS) {
      pollfd p{fd, POLLOUT, 0};
      int error = 0;
      socklen_t length = sizeof(error);
      reached = poll(&p, 1, kReachableTimeoutMs) == 1 && getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &length) == 0 &&
                error == 0;
    }
    close(fd);
  }
  freeaddrinfo(found);
  return reached;
}

// Runs a callback in the executor once another thread has called Wake().
class Wakeup : public rclcpp::Waitable {
 public:
  Wakeup(const rclcpp::Context::SharedPtr& context, std::function<void()> callback)
      : condition_(context), callback_(std::move(callback)) {}

  // Called from the MQTT client's thread, so it must not throw there; the next wake drains what is left.
  void Wake() noexcept {
    try {
      condition_.trigger();
    } catch (...) {  // NOLINT(bugprone-empty-catch): must not throw on the client's thread
    }
  }

  size_t get_number_of_ready_guard_conditions() override { return 1; }
  void add_to_wait_set(rcl_wait_set_t& wait_set) override { condition_.add_to_wait_set(wait_set); }
  bool is_ready(const rcl_wait_set_t& wait_set) override {
    for (size_t i = 0; i < wait_set.size_of_guard_conditions; ++i) {
      if (wait_set.guard_conditions[i] == &condition_.get_rcl_guard_condition()) return true;
    }
    return false;
  }
  std::shared_ptr<void> take_data() override { return nullptr; }
  void execute(const std::shared_ptr<void>&) override { callback_(); }

 private:
  rclcpp::GuardCondition condition_;
  std::function<void()> callback_;
};

}  // namespace

// Carries messages between the nodes, MQTT and BridgeCore; the core is used from the executor only.
class BridgeNode : public rclcpp::Node {
 public:
  explicit BridgeNode(const rclcpp::NodeOptions& options)
      : Node("mowbite_bridge", mowbite_component::StartupOnly(options)),
        host_(declare_parameter("mqtt_host", std::string("127.0.0.1"))),
        port_(declare_parameter("mqtt_port", 1883)),
        data_dir_(declare_parameter("data_dir", std::string())),
        core_(BridgeConfig{declare_parameter("software_version", std::string()),
                           declare_parameter("params_json", std::string("{}"))}),
        events_(JoinPath(data_dir_, "event_history"),
                static_cast<std::size_t>(std::max<int64_t>(0, declare_parameter("event_history_days", 60))),
                static_cast<std::uintmax_t>(std::max(0.0, declare_parameter("event_history_max_mb", 20.0)) * 1e6)),
        track_(JoinPath(data_dir_, "position_history"),
               static_cast<double>(declare_parameter("position_history_days", 60)),
               static_cast<std::uintmax_t>(std::max(0.0, declare_parameter("position_history_max_mb", 50.0)) * 1e6)) {
    if (data_dir_.empty()) RCLCPP_WARN(get_logger(), "No data_dir: event history and track are not kept");
    track_.Open(WallNow());
    core_.SetEventSink([this](const Json& event) {
      const double now = WallNow();
      events_.Write(event, now);
      track_.OnEvent(event, now);
    });
    core_.AddLocalMethod("events.history", [this](const Json& p) { return events_.Read(p, WallNow()); });
    core_.AddLocalMethod("events.history.list", [this](const Json&) { return events_.List(); });
    core_.AddLocalMethod("events.history.delete", [this](const Json& p) { return events_.Erase(p); });
    core_.AddLocalMethod("position.history", [this](const Json& p) { return track_.History(p); });
    core_.AddLocalMethod("position.history.list", [this](const Json&) { return track_.List(); });
    core_.AddLocalMethod("position.history.delete", [this](const Json& p) { return track_.Erase(p); });
    core_.AddLocalMethod("system.stats", [this](const Json&) {
      SystemSources sources;
      sources.data = data_dir_;
      return SystemStats(sources);
    });
    core_.Start(WallNow());

    action_pub_ = create_publisher<std_msgs::msg::String>("bridge/action", 10);
    teleop_pub_ = create_publisher<geometry_msgs::msg::Twist>("bridge/teleop", 10);
    request_pub_ = create_publisher<msg::RpcRequest>("bridge/rpc/request", 100);

    Subscribe<msg::RobotState>("bridge/robot_state", rclcpp::QoS(10), [this](const msg::RobotState& m) {
      core_.OnRobotState(View(m));
    });
    SubscribeFrom<msg::SensorSpec>(kSensorInfoTopic, Latched(100),
                                   [this](const msg::SensorSpec& m, const std::string& provider) {
                                     text_sensors_[m.id] = m.kind == msg::SensorSpec::KIND_TEXT;
                                     core_.OnSensor(Describe(m), provider);
                                   });
    Subscribe<msg::SensorValue>("bridge/sensor_value", rclcpp::QoS(100), [this](const msg::SensorValue& m) {
      const auto known = text_sensors_.find(m.sensor_id);
      const bool text = known != text_sensors_.end() && known->second;
      core_.OnSensorValue(m.sensor_id, text ? Json(m.text) : Json(m.value), WallNow());
    });
    Subscribe<msg::ActionList>("bridge/actions", Latched(50), [this](const msg::ActionList& m) {
      std::vector<ActionItem> items;
      for (const auto& a : m.actions) items.push_back({a.id, a.name, a.enabled, a.reason});
      core_.OnActions(m.node_prefix, items);
    });
    Subscribe<std_msgs::msg::String>("bridge/params", Latched(50),
                                     [this](const std_msgs::msg::String& m) { core_.OnParams(m.data); });
    Subscribe<msg::MqttPublish>("bridge/publish", rclcpp::QoS(100), [this](const msg::MqttPublish& m) {
      core_.OnPublish(m.topic, m.payload, m.retain);
    });
    SubscribeFrom<msg::RpcMethods>(kMethodsTopic, Latched(50),
                                   [this](const msg::RpcMethods& m, const std::string& provider) {
                                     core_.OnRpcMethods(m.node_id, m.methods, provider);
                                   });
    Subscribe<msg::RpcResponse>("bridge/rpc/response", rclcpp::QoS(100), [this](const msg::RpcResponse& m) {
      core_.OnRpcResponse(m.id, m.result);
    });
    Subscribe<msg::RpcError>("bridge/rpc/error", rclcpp::QoS(100), [this](const msg::RpcError& m) {
      core_.OnRpcError(m.id, m.code, m.message, m.data);
    });
    Subscribe<msg::RobotPose>("localization/pose", rclcpp::QoS(10), [this](const msg::RobotPose& m) {
      const PlanePose pose{m.x, m.y, m.heading};
      if (std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.heading)) window_.Add(pose);
    });
    Subscribe<std_msgs::msg::String>("map/json", Latched(1),
                                     [this](const std_msgs::msg::String& m) { core_.OnMap(m.data); });
    Subscribe<msg::HardwareCapabilities>("hw/capabilities", Latched(1), [this](const msg::HardwareCapabilities& m) {
      core_.OnPlatform(m.platform,
                       StopRelease(m.stop_release_free_s, m.stop_release_hold_s, m.stop_release_max_hold_s));
    });
    // Transient local: the WARN lines from before this node met the others are in logs.recent too.
    Subscribe<rcl_interfaces::msg::Log>("/rosout", Latched(1000), [this](const rcl_interfaces::msg::Log& m) {
      core_.OnLog(rclcpp::Time(m.stamp).seconds(), m.level, m.name, m.msg);
    });

    position_timer_ = create_wall_timer(kPositionPeriod, [this] {
      Locked("position", [&] {
        const auto pose = window_.Take();
        if (!pose) return;
        track_.Add(pose->x, pose->y);
        core_.SendPosition(pose->x, pose->y, pose->heading, track_.Attributes());
      });
    });
    save_timer_ = create_wall_timer(kSavePeriod, [this] { Locked("track save", [&] { track_.Save(); }); });
    providers_timer_ = create_wall_timer(kProvidersPeriod, [this] {
      // A graph that cannot be read skips the check: it throws, and no provider counts as missing.
      Locked("providers", [&] {
        std::set<std::string> alive;
        for (const char* topic : {kMethodsTopic, kSensorInfoTopic}) {
          for (const auto& publisher : get_publishers_info_by_topic(topic)) {
            alive.insert(ProviderName(publisher.endpoint_gid()));
          }
        }
        core_.OnProviders(alive);
      });
    });
    tick_timer_ = create_wall_timer(kTickPeriod, [this] { Locked("tick", [&] { core_.Tick(WallNow()); }); });
    restart_timer_ = create_wall_timer(kRestartNoteDelay, [this] {
      restart_timer_->cancel();
      ReportRestart();
    });

    wakeup_ = std::make_shared<Wakeup>(get_node_base_interface()->get_context(), [this] { Drain(); });
    get_node_waitables_interface()->add_waitable(wakeup_, nullptr);
    MakeClient();
    retry_timer_ = create_wall_timer(kRetryCheck, [this] { TryConnect(); });
    TryConnect();
    watch_ = mowbite_watch::Attach(*this);
  }

  ~BridgeNode() override {
    stopping_ = true;
    Locked("shutdown", [&] {
      core_.Stop(WallNow());
      track_.Close();
    });
    try {
      if (client_ && client_->is_connected()) client_->disconnect(2s)->wait_for(3s);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "MQTT disconnect: %s", e.what());
    }
    client_.reset();
    retired_client_.reset();
    get_node_waitables_interface()->remove_waitable(wakeup_, nullptr);
  }

 private:
  // What the MQTT client's thread hands over to the executor; code and reachable only with kConnectFailed.
  struct Arrival {
    enum Kind { kMessage, kConnected, kLost, kConnectFailed } kind;
    std::string topic;
    std::string payload;
    bool retained = false;
    ResponseRoute route;
    int code = 0;
    bool reachable = false;
  };

  // A client for version_; the old one of a fallback is kept until the end, its thread may still be in a callback.
  void MakeClient() {
    if (client_) retired_client_ = std::move(client_);
    const int version = version_.version() == 5 ? MQTTVERSION_5 : MQTTVERSION_3_1_1;
    client_ = std::make_unique<mqtt::async_client>("tcp://" + host_ + ":" + std::to_string(port_), kClientId,
                                                   mqtt::create_options(version));
    // The client's thread only queues and wakes the executor; the client is called from the executor alone.
    client_->set_connected_handler(
        [this](const std::string&) { Post({Arrival::kConnected, "", "", false, {}, 0, false}); });
    client_->set_connection_lost_handler(
        [this](const std::string&) { Post({Arrival::kLost, "", "", false, {}, 0, false}); });
    client_->set_message_callback([this](const mqtt::const_message_ptr& m) {
      if (!m) return;
      Arrival arrival{Arrival::kMessage, m->get_topic(), m->get_payload_str(), m->is_retained(), {}, 0, false};
      const mqtt::properties& props = m->get_properties();
      if (props.contains(mqtt::property::RESPONSE_TOPIC)) {
        arrival.route.topic = mqtt::get<std::string>(props, mqtt::property::RESPONSE_TOPIC);
      }
      if (props.contains(mqtt::property::CORRELATION_DATA)) {
        arrival.route.correlation = mqtt::get<std::string>(props, mqtt::property::CORRELATION_DATA);
      }
      Post(std::move(arrival));
    });
  }

  // Beyond kMaxInbox waiting messages new ones are dropped; connection changes always go in.
  void Post(Arrival arrival) {
    {
      std::lock_guard<std::mutex> lock(inbox_mutex_);
      if (arrival.kind == Arrival::kMessage && inbox_.size() >= kMaxInbox) return;
      inbox_.push_back(std::move(arrival));
    }
    wakeup_->Wake();
  }

  void Drain() {
    std::deque<Arrival> arrivals;
    {
      std::lock_guard<std::mutex> lock(inbox_mutex_);
      arrivals.swap(inbox_);
    }
    for (const auto& a : arrivals) {
      switch (a.kind) {
        case Arrival::kConnected: OnConnected(); break;
        case Arrival::kLost: OnConnectionLost(); break;
        case Arrival::kConnectFailed: OnConnectFailed(a.code, a.reachable); break;
        case Arrival::kMessage:
          Locked("MQTT input", [&] { core_.OnMqtt(a.topic, a.payload, a.retained, a.route); });
          break;
      }
    }
  }

  // Ends a pending connect attempt; the connected handler does the rest after a success, OnConnectFailed after a
  // failure, which also ends the attempt.
  class ConnectWatch : public mqtt::iaction_listener {
   public:
    explicit ConnectWatch(BridgeNode& node) : node_(node) {}
    void on_failure(const mqtt::token& token) override {
      const int code = token.get_return_code();
      // Only a failure of the client itself can be a broker that dropped the connect; the probe waits here, not in
      // the executor.
      const bool reachable = code == MQTTASYNC_FAILURE && TakesTcp(node_.host_, node_.port_);
      node_.Post({Arrival::kConnectFailed, "", "", false, {}, code, reachable});
    }
    void on_success(const mqtt::token&) override {
      node_.connecting_ = false;
    }

   private:
    BridgeNode& node_;
  };

  template <typename M, typename F>
  void Subscribe(const std::string& topic, const rclcpp::QoS& qos, F handler) {
    subscriptions_.push_back(create_subscription<M>(topic, qos, [this, topic, handler](const M& m) {
      Locked(topic.c_str(), [&] { handler(m); });
    }));
  }

  // As Subscribe, the handler also gets the name of the publisher each message came from.
  template <typename M, typename F>
  void SubscribeFrom(const std::string& topic, const rclcpp::QoS& qos, F handler) {
    subscriptions_.push_back(
        create_subscription<M>(topic, qos, [this, topic, handler](const M& m, const rclcpp::MessageInfo& info) {
          const auto& gid = info.get_rmw_message_info().publisher_gid.data;
          Locked(topic.c_str(), [&] { handler(m, ProviderName(gid)); });
        }));
  }

  // Runs f with the core to itself, then sends what came of it; where names it should the bridge stall. Nothing that
  // arrives ends the process.
  template <typename F>
  void Locked(const char* where, F&& f) {
    if (watch_) watch_->Mark(where);
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      f();
      Flush();
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Bridge: %s failed: %s", where, e.what());
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Bridge: %s failed", where);
    }
  }

  // With mutex_ held.
  void Flush() {
    Outbox out = core_.Take();
    for (const auto& m : out.mqtt) {
      try {
        if (!client_ || !connected_) continue;
        if (m.correlation) {
          mqtt::properties props;
          props.add(mqtt::property(mqtt::property::CORRELATION_DATA, *m.correlation));
          client_->publish(mqtt::message::create(m.topic, m.payload.data(), m.payload.size(), m.qos, m.retain, props));
        } else {
          client_->publish(m.topic, m.payload.data(), m.payload.size(), m.qos, m.retain);
        }
      } catch (const mqtt::exception& e) {
        RCLCPP_DEBUG(get_logger(), "MQTT publish of %s: %s", m.topic.c_str(), e.what());
      }
    }
    for (auto& text : out.actions) {
      std_msgs::msg::String m;
      m.data = std::move(text);
      action_pub_->publish(m);
    }
    for (const auto& t : out.teleop) {
      geometry_msgs::msg::Twist m;
      m.linear.x = t.vx;
      m.angular.z = t.vz;
      teleop_pub_->publish(m);
    }
    for (auto& r : out.requests) {
      msg::RpcRequest m;
      m.method = std::move(r.method);
      m.params = std::move(r.params);
      m.id = std::move(r.id);
      request_pub_->publish(m);
    }
    for (const auto& line : out.logs) {
      if (line.level >= kLogWarn) {
        RCLCPP_WARN(get_logger(), "%s", line.text.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "%s", line.text.c_str());
      }
    }
  }

  // A new attempt kRetrySeconds after the last one while there is no connection.
  void TryConnect() {
    if (stopping_ || connected_ || connecting_) return;
    const double now = WallNow();
    if (now - last_attempt_ < kRetrySeconds) return;
    last_attempt_ = now;
    connecting_ = true;
    try {
      mqtt::connect_options options;
      if (version_.version() == 5) {
        options.set_mqtt_version(MQTTVERSION_5);
        options.set_clean_start(true);
        mqtt::properties props;
        props.add(mqtt::property(mqtt::property::SESSION_EXPIRY_INTERVAL, 0));
        options.set_properties(props);
      } else {
        options.set_mqtt_version(MQTTVERSION_3_1_1);
        options.set_clean_session(true);
      }
      options.set_will(mqtt::will_options(kBridgeStatusTopic, BridgeStatusText(false), 1, true));
      options.set_keep_alive_interval(30);
      options.set_connect_timeout(4);
      options.set_automatic_reconnect(false);
      client_->connect(options, nullptr, connect_watch_);
    } catch (const std::exception& e) {
      connecting_ = false;
      RCLCPP_DEBUG(get_logger(), "MQTT connect: %s", e.what());
    }
  }

  void OnConnectFailed(int code, bool reachable) {
    connecting_ = false;
    const auto failure = MqttVersion::Classify(code, reachable);
    if (stopping_ || !version_.OnFailure(failure)) {
      RCLCPP_DEBUG(get_logger(), "MQTT connect failed with %d", code);
      return;
    }
    RCLCPP_WARN(get_logger(), "MQTT: the broker at %s:%ld %s MQTT 5, the bridge uses 3.1.1 until it restarts: answers "
                "go to rpc/response only", host_.c_str(), static_cast<long>(port_),
                failure == MqttVersion::Failure::kRefused ? "refused" : "twice dropped the connect of");
    MakeClient();
    last_attempt_ = -1e9;
  }

  void OnConnected() {
    if (stopping_) return;
    version_.OnConnected();
    Locked("connect", [&] {
      connected_ = true;
      for (const auto& topic : BridgeCore::CommandTopics()) {
        try {
          client_->subscribe(topic, 1);
        } catch (const mqtt::exception& e) {
          RCLCPP_WARN(get_logger(), "MQTT subscribe to %s: %s", topic.c_str(), e.what());
        }
      }
      core_.Connected(version_.version());
    });
  }

  void OnConnectionLost() {
    Locked("connection lost", [&] {
      connected_ = false;
      core_.ConnectionLost();
    });
  }

  void ReportRestart() {
    ReportNote(kRestartNote, RestartWarning);
    ReportNote(kRollbackNote, RollbackWarning);
  }

  void ReportNote(const char* name, std::string (*warning)(std::string_view)) {
    const std::string path = JoinPath(data_dir_, name);
    if (path.empty()) return;
    std::ifstream in(path);
    if (!in) return;
    std::ostringstream text;
    text << in.rdbuf();
    RCLCPP_WARN(get_logger(), "%s", warning(text.str()).c_str());
  }

  const std::string host_;
  const int64_t port_;
  const std::string data_dir_;
  std::mutex mutex_;
  std::mutex inbox_mutex_;
  std::deque<Arrival> inbox_;
  BridgeCore core_;
  EventLog events_;
  MowTrack track_;
  PoseWindow window_;
  std::map<std::string, bool> text_sensors_;

  std::atomic<bool> stopping_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> connecting_{false};
  double last_attempt_ = -1e9;
  MqttVersion version_;
  ConnectWatch connect_watch_{*this};
  std::shared_ptr<Wakeup> wakeup_;
  std::unique_ptr<mqtt::async_client> client_;
  std::unique_ptr<mqtt::async_client> retired_client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
  rclcpp::Publisher<msg::RpcRequest>::SharedPtr request_pub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr position_timer_;
  rclcpp::TimerBase::SharedPtr save_timer_;
  rclcpp::TimerBase::SharedPtr providers_timer_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr restart_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_bridge::BridgeNode)
