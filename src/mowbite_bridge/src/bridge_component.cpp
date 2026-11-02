#include <atomic>
#include <chrono>
#include <deque>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <mqtt/async_client.h>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include "mowbite_bridge/bridge_core.h"
#include "mowbite_bridge/event_log.h"
#include "mowbite_bridge/pose_window.h"
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

namespace mowbite_bridge {

namespace {

using namespace std::chrono_literals;
namespace msg = mowbite_msgs::msg;

constexpr auto kRetryCheck = 1s;
constexpr auto kInboxPeriod = 20ms;
constexpr std::size_t kMaxInbox = 1000;
constexpr double kRetrySeconds = 5.0;
constexpr auto kPositionPeriod = 150ms;
constexpr auto kSavePeriod = 30s;
constexpr auto kRestartNoteDelay = 1s;
constexpr char kClientId[] = "mowbite_bridge";
constexpr char kRestartNote[] = "container_exit.json";

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

}  // namespace

// Carries messages between the nodes, MQTT and BridgeCore; the core is used from the executor only.
class BridgeNode : public rclcpp::Node {
 public:
  explicit BridgeNode(const rclcpp::NodeOptions& options)
      : Node("mowbite_bridge", options),
        host_(declare_parameter("mqtt_host", std::string("127.0.0.1"))),
        port_(declare_parameter("mqtt_port", 1883)),
        data_dir_(declare_parameter("data_dir", std::string())),
        core_(BridgeConfig{declare_parameter("software_version", std::string()),
                           declare_parameter("params_json", std::string("{}"))}),
        events_(JoinPath(data_dir_, "event_history"),
                static_cast<std::size_t>(std::max<int64_t>(0, declare_parameter("event_history_days", 60))),
                static_cast<std::uintmax_t>(std::max(0.0, declare_parameter("event_history_max_mb", 20.0)) * 1e6)),
        track_(JoinPath(data_dir_, "position_history"), declare_parameter("position_history_days", 60),
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
    core_.Start(WallNow());

    action_pub_ = create_publisher<std_msgs::msg::String>("bridge/action", 10);
    teleop_pub_ = create_publisher<geometry_msgs::msg::Twist>("bridge/teleop", 10);
    request_pub_ = create_publisher<msg::RpcRequest>("bridge/rpc/request", 100);

    Subscribe<msg::RobotState>("bridge/robot_state", rclcpp::QoS(10), [this](const msg::RobotState& m) {
      core_.OnRobotState(View(m));
    });
    Subscribe<msg::SensorSpec>("bridge/sensor_info", Latched(100), [this](const msg::SensorSpec& m) {
      text_sensors_[m.id] = m.kind == msg::SensorSpec::KIND_TEXT;
      core_.OnSensor(Describe(m));
    });
    Subscribe<msg::SensorValue>("bridge/sensor_value", rclcpp::QoS(100), [this](const msg::SensorValue& m) {
      const auto known = text_sensors_.find(m.sensor_id);
      const bool text = known != text_sensors_.end() && known->second;
      core_.OnSensorValue(m.sensor_id, text ? Json(m.text) : Json(m.value), WallNow());
    });
    Subscribe<msg::ActionList>("bridge/actions", Latched(50), [this](const msg::ActionList& m) {
      std::vector<ActionItem> items;
      for (const auto& a : m.actions) items.push_back({a.id, a.name, a.enabled});
      core_.OnActions(m.node_prefix, items);
    });
    Subscribe<std_msgs::msg::String>("bridge/params", Latched(50),
                                     [this](const std_msgs::msg::String& m) { core_.OnParams(m.data); });
    Subscribe<msg::MqttPublish>("bridge/publish", rclcpp::QoS(100), [this](const msg::MqttPublish& m) {
      core_.OnPublish(m.topic, m.payload, m.retain);
    });
    Subscribe<msg::RpcMethods>("bridge/rpc/methods", Latched(50), [this](const msg::RpcMethods& m) {
      core_.OnRpcMethods(m.node_id, m.methods);
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
      core_.OnPlatform(m.platform);
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
    restart_timer_ = create_wall_timer(kRestartNoteDelay, [this] {
      restart_timer_->cancel();
      ReportRestart();
    });

    client_ = std::make_unique<mqtt::async_client>("tcp://" + host_ + ":" + std::to_string(port_), kClientId);
    // The client's thread only queues; the client is called from the executor alone.
    client_->set_connected_handler([this](const std::string&) { Post({Arrival::kConnected, "", "", false}); });
    client_->set_connection_lost_handler([this](const std::string&) { Post({Arrival::kLost, "", "", false}); });
    client_->set_message_callback([this](mqtt::const_message_ptr m) {
      if (m) Post({Arrival::kMessage, m->get_topic(), m->get_payload_str(), m->is_retained()});
    });
    inbox_timer_ = create_wall_timer(kInboxPeriod, [this] { Drain(); });
    retry_timer_ = create_wall_timer(kRetryCheck, [this] { TryConnect(); });
    TryConnect();
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
  }

 private:
  // What the MQTT client's thread hands over to the executor.
  struct Arrival {
    enum Kind { kMessage, kConnected, kLost } kind;
    std::string topic;
    std::string payload;
    bool retained;
  };

  // Beyond kMaxInbox waiting messages new ones are dropped; connection changes always go in.
  void Post(Arrival arrival) {
    std::lock_guard<std::mutex> lock(inbox_mutex_);
    if (arrival.kind == Arrival::kMessage && inbox_.size() >= kMaxInbox) return;
    inbox_.push_back(std::move(arrival));
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
        case Arrival::kMessage:
          Locked("MQTT input", [&] { core_.OnMqtt(a.topic, a.payload, a.retained); });
          break;
      }
    }
  }

  // Ends a pending connect attempt either way; the connected handler does the rest.
  class ConnectWatch : public mqtt::iaction_listener {
   public:
    explicit ConnectWatch(std::atomic<bool>& pending) : pending_(pending) {}
    void on_failure(const mqtt::token&) override {
      pending_ = false;
    }
    void on_success(const mqtt::token&) override {
      pending_ = false;
    }

   private:
    std::atomic<bool>& pending_;
  };

  template <typename M, typename F>
  void Subscribe(const std::string& topic, const rclcpp::QoS& qos, F handler) {
    subscriptions_.push_back(create_subscription<M>(topic, qos, [this, topic, handler](const M& m) {
      Locked(topic.c_str(), [&] { handler(m); });
    }));
  }

  // Runs f with the core to itself, then sends what came of it. Nothing that arrives ends the process.
  template <typename F>
  void Locked(const char* where, F&& f) {
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
        if (client_ && connected_) client_->publish(m.topic, m.payload.data(), m.payload.size(), m.qos, m.retain);
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
      options.set_mqtt_version(MQTTVERSION_3_1_1);
      options.set_clean_session(true);
      options.set_keep_alive_interval(30);
      options.set_connect_timeout(4);
      options.set_automatic_reconnect(false);
      client_->connect(options, nullptr, connect_watch_);
    } catch (const std::exception& e) {
      connecting_ = false;
      RCLCPP_DEBUG(get_logger(), "MQTT connect: %s", e.what());
    }
  }

  void OnConnected() {
    if (stopping_) return;
    Locked("connect", [&] {
      connected_ = true;
      for (const auto& topic : BridgeCore::CommandTopics()) {
        try {
          client_->subscribe(topic, 1);
        } catch (const mqtt::exception& e) {
          RCLCPP_WARN(get_logger(), "MQTT subscribe to %s: %s", topic.c_str(), e.what());
        }
      }
      core_.Connected();
    });
  }

  void OnConnectionLost() {
    Locked("connection lost", [&] {
      connected_ = false;
      core_.ConnectionLost();
    });
  }

  void ReportRestart() {
    const std::string path = JoinPath(data_dir_, kRestartNote);
    if (path.empty()) return;
    std::ifstream in(path);
    if (!in) return;
    std::ostringstream text;
    text << in.rdbuf();
    RCLCPP_WARN(get_logger(), "%s", RestartWarning(text.str()).c_str());
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
  ConnectWatch connect_watch_{connecting_};
  std::unique_ptr<mqtt::async_client> client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
  rclcpp::Publisher<msg::RpcRequest>::SharedPtr request_pub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr position_timer_;
  rclcpp::TimerBase::SharedPtr save_timer_;
  rclcpp::TimerBase::SharedPtr restart_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr inbox_timer_;
};

}  // namespace mowbite_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_bridge::BridgeNode)
