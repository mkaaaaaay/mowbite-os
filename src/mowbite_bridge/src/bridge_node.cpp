// MQTT bridge with the interface of OpenMower's xbot_monitoring, so the MowBite app works unchanged. Based on
// xbot_monitoring.cpp (OpenMower), MIT License, Copyright (c) 2022 Clemens Elflein. See LICENSE.xbot_monitoring.
#include <mqtt/async_client.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <mowbite_msgs/msg/absolute_pose.hpp>
#include <mowbite_msgs/msg/action_list.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mowbite_msgs/msg/sensor_info.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mutex>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

#include "mowbite_bridge/event_history.h"
#include "mowbite_bridge/log_buffer.h"
#include "mowbite_bridge/monitor.h"
#include "mowbite_bridge/params_store.h"
#include "mowbite_bridge/position_history.h"
#include "mowbite_bridge/position_median.h"

namespace mowbite_bridge {

namespace {

const std::string kEventsTopic = "events/json";
const std::string kNodeId = "mowbite_bridge";

class RpcException : public std::runtime_error {
 public:
  RpcException(int16_t code, const std::string& message) : std::runtime_error(message), code_(code) {
  }
  int16_t code() const {
    return code_;
  }

 private:
  int16_t code_;
};

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

}  // namespace

class BridgeNode : public rclcpp::Node {
 public:
  explicit BridgeNode(const rclcpp::NodeOptions& options) : rclcpp::Node("mowbite_bridge", options), callback_(*this) {
    const std::string host = declare_parameter("mqtt_host", "127.0.0.1");
    const int port = declare_parameter("mqtt_port", 1883);
    const std::string data_dir = declare_parameter("data_dir", ".");
    version_ = declare_parameter("software_version", "");
    if (version_.empty()) version_ = "UNKNOWN VERSION";
    params_ = ParamsStore(json::parse(declare_parameter("params_json", "{}"), nullptr, false));

    const int history_days = declare_parameter("event_history_days", 60);
    const double history_mb = declare_parameter("event_history_max_mb", 20.0);
    event_history_.init((std::filesystem::path(data_dir) / "event_history").string(),
                        static_cast<size_t>(std::max(history_days, 1)), static_cast<uintmax_t>(history_mb * 1e6));

    local_methods_["rpc.ping"] = [](const json&) { return json("pong"); };
    local_methods_["rpc.methods"] = [this](const json&) { return MethodList(); };
    local_methods_["events.history"] = [this](const json& params) {
      if (params.is_object() && params.contains("date") && params["date"].is_string()) {
        return event_history_.getAll(params["date"].get<std::string>());
      }
      return event_history_.getAll();
    };
    local_methods_["events.history.list"] = [this](const json&) { return event_history_.listHistories(); };
    local_methods_["events.history.delete"] = [this](const json& params) {
      if (params.is_object() && params.contains("date") && params["date"].is_string()) {
        return event_history_.deleteHistory(params["date"].get<std::string>());
      }
      return event_history_.deleteHistory(std::nullopt);
    };
    local_methods_["position.history"] = [this](const json& params) {
      if (params.is_object() && params.contains("job_id") && params["job_id"].is_string()) {
        return position_history_.getHistory(params["job_id"].get<std::string>());
      }
      return position_history_.getHistory();
    };
    local_methods_["position.history.list"] = [this](const json&) { return position_history_.listHistories(); };
    local_methods_["position.history.delete"] = [this](const json& params) {
      if (params.is_object() && params.contains("job_id") && params["job_id"].is_string()) {
        return position_history_.deleteHistory(params["job_id"].get<std::string>());
      }
      return position_history_.deleteHistory(std::nullopt);
    };
    local_methods_["logs.recent"] = [this](const json& params) {
      double since = 0;
      uint8_t level = LogBuffer::WARN;
      size_t limit = 200;
      if (params.is_object()) {
        if (params.contains("since") && params["since"].is_number()) since = params["since"].get<double>();
        if (params.contains("level") && params["level"].is_string()) {
          level = LogBuffer::levelFromName(params["level"].get<std::string>());
        }
        if (params.contains("limit") && params["limit"].is_number_unsigned()) limit = params["limit"].get<size_t>();
      }
      return log_buffer_.get(since, level, limit);
    };

    const int track_days = declare_parameter("position_history_days", 60);
    const double track_mb = declare_parameter("position_history_max_mb", 50.0);
    position_history_.init(
        (std::filesystem::path(data_dir) / "position_history").string(), [this] { return now().seconds(); },
        static_cast<size_t>(std::max(track_days, 1)), static_cast<uintmax_t>(track_mb * 1e6),
        [this](const std::string& message) { RCLCPP_WARN(get_logger(), "%s", message.c_str()); });

    teleop_pub_ = create_publisher<geometry_msgs::msg::Twist>("bridge/teleop", 10);
    action_pub_ = create_publisher<std_msgs::msg::String>("bridge/action", 10);
    rpc_request_pub_ = create_publisher<mowbite_msgs::msg::RpcRequest>("bridge/rpc/request", 100);

    pose_sub_ = create_subscription<mowbite_msgs::msg::AbsolutePose>(
        "localization/pose", 10, [this](mowbite_msgs::msg::AbsolutePose::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          pose_buffer_.push_back({msg->pose.pose.position.x, msg->pose.pose.position.y, msg->vehicle_heading});
        });
    params_sub_ = create_subscription<std_msgs::msg::String>(
        "bridge/params", LatchedQos(50), [this](std_msgs::msg::String::ConstSharedPtr msg) { OnParams(msg->data); });
    robot_state_sub_ = create_subscription<mowbite_msgs::msg::RobotState>(
        "bridge/robot_state", 10, [this](mowbite_msgs::msg::RobotState::ConstSharedPtr msg) { OnRobotState(*msg); });
    sensor_info_sub_ = create_subscription<mowbite_msgs::msg::SensorInfo>(
        "bridge/sensor_info", LatchedQos(100),
        [this](mowbite_msgs::msg::SensorInfo::ConstSharedPtr msg) { OnSensorInfo(*msg); });
    sensor_value_sub_ = create_subscription<mowbite_msgs::msg::SensorValue>(
        "bridge/sensor_value", 100,
        [this](mowbite_msgs::msg::SensorValue::ConstSharedPtr msg) { OnSensorValue(*msg); });
    actions_sub_ = create_subscription<mowbite_msgs::msg::ActionList>(
        "bridge/actions", LatchedQos(50), [this](mowbite_msgs::msg::ActionList::ConstSharedPtr msg) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            actions_[msg->node_prefix] = msg->actions;
          }
          PublishActions();
        });
    methods_sub_ = create_subscription<mowbite_msgs::msg::RpcMethods>(
        "bridge/rpc/methods", LatchedQos(50), [this](mowbite_msgs::msg::RpcMethods::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          methods_[msg->node_id] = msg->methods;
          RCLCPP_INFO(get_logger(), "%s registered %zu methods", msg->node_id.c_str(), msg->methods.size());
        });
    rpc_response_sub_ = create_subscription<mowbite_msgs::msg::RpcResponse>(
        "bridge/rpc/response", 100, [this](mowbite_msgs::msg::RpcResponse::ConstSharedPtr msg) {
          TryPublish("rpc/response", RpcResultJson(msg->id, msg->result));
        });
    rpc_error_sub_ = create_subscription<mowbite_msgs::msg::RpcError>(
        "bridge/rpc/error", 100, [this](mowbite_msgs::msg::RpcError::ConstSharedPtr msg) {
          TryPublish("rpc/response", RpcErrorJson(msg->code, msg->message, msg->id));
        });
    publish_sub_ = create_subscription<mowbite_msgs::msg::MqttPublish>(
        "bridge/publish", 100, [this](mowbite_msgs::msg::MqttPublish::ConstSharedPtr msg) {
          TryPublish(msg->topic, msg->payload, msg->retain);
          if (msg->topic == kEventsTopic) {
            event_history_.add(msg->payload);
            json event = json::parse(msg->payload, nullptr, false);
            if (event.is_object()) position_history_.onEvent(event);
          }
        });
    map_sub_ = create_subscription<std_msgs::msg::String>(
        "map/json", LatchedQos(1), [this](std_msgs::msg::String::ConstSharedPtr msg) { OnMap(msg->data); });
    rosout_sub_ = create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", 100, [this](rcl_interfaces::msg::Log::ConstSharedPtr msg) {
          log_buffer_.add(rclcpp::Time(msg->stamp).seconds(), msg->level, msg->name, msg->msg);
        });

    client_ = std::make_shared<mqtt::async_client>("tcp://" + host + ":" + std::to_string(port), kNodeId);
    client_->set_callback(callback_);
    Connect();
    position_timer_ = create_wall_timer(std::chrono::milliseconds(150), [this] { PublishPosition(); });
    position_flush_timer_ = create_wall_timer(std::chrono::seconds(30), [this] { position_history_.periodicFlush(); });
    reconnect_timer_ = create_wall_timer(std::chrono::seconds(5), [this] {
      if (!client_->is_connected() && !connecting_) Connect();
    });

    PublishEvent("BOOTED");
  }

  ~BridgeNode() override {
    position_history_.flush();
    PublishEvent("SHUTDOWN");
    try {
      if (client_->is_connected()) client_->disconnect()->wait_for(std::chrono::seconds(1));
    } catch (const mqtt::exception&) {
    }
  }

 private:
  class Callback : public mqtt::callback, public mqtt::iaction_listener {
   public:
    explicit Callback(BridgeNode& node) : node_(node) {
    }
    void connected(const mqtt::string&) override {
      node_.OnConnected();
    }
    void connection_lost(const mqtt::string&) override {
      RCLCPP_WARN(node_.get_logger(), "MQTT connection lost");
    }
    void message_arrived(mqtt::const_message_ptr msg) override {
      node_.OnMessage(msg->get_topic(), msg->get_payload_str());
    }
    void on_failure(const mqtt::token&) override {
      node_.connecting_ = false;
    }
    void on_success(const mqtt::token&) override {
      node_.connecting_ = false;
    }

   private:
    BridgeNode& node_;
  };

  void Connect() {
    mqtt::connect_options options;
    options.set_clean_session(true);
    options.set_keep_alive_interval(std::chrono::seconds(30));
    options.set_max_inflight(10);
    connecting_ = true;
    try {
      client_->connect(options, nullptr, callback_);
    } catch (const mqtt::exception& e) {
      connecting_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "MQTT connect failed: %s", e.what());
    }
  }

  void OnConnected() {
    RCLCPP_INFO(get_logger(), "MQTT connected");
    json capabilities = {{"rpc", 1}, {"map:json", 1}, {"events", 1}};
    TryPublish("capabilities/json", capabilities.dump(2), true);
    TryPublish("version/json", json{{"version", version_}}.dump(), true);
    TryPublish("params/json", ParamsPayload(), true);
    PublishSensorInfos();
    PublishActions();
    std::string map;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map = map_;
    }
    if (!map.empty()) TryPublish("map/json", map, true);
    for (const char* topic : {"teleop", "action", "rpc/request"}) {
      try {
        client_->subscribe(topic, 0);
      } catch (const mqtt::exception& e) {
        RCLCPP_ERROR(get_logger(), "MQTT subscribe %s failed: %s", topic, e.what());
      }
    }
  }

  std::string ParamsPayload() {
    std::lock_guard<std::mutex> lock(mutex_);
    return params_.Payload();
  }

  void OnParams(const std::string& message) {
    bool changed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      changed = params_.Update(message);
    }
    if (changed) TryPublish("params/json", ParamsPayload(), true);
  }

  // Median of the poses since the last call; the track only records while a mowing session runs.
  void PublishPosition() {
    std::vector<PoseSample> samples;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      samples.swap(pose_buffer_);
    }
    const auto median = MedianPose(samples);
    if (!median) return;
    position_history_.addPoint(median->x, median->y);
    const json j = {
        {"x", median->x}, {"y", median->y}, {"heading", median->heading}, {"attributes", position_history_.getAttributes()}};
    TryPublish("position/json", j.dump());
  }

  void OnMessage(const std::string& topic, const std::string& payload) {
    if (topic == "teleop") {
      auto teleop = DecodeTeleop(payload);
      if (!teleop) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring invalid teleop of %zu bytes", payload.size());
        return;
      }
      geometry_msgs::msg::Twist twist;
      twist.linear.x = teleop->first;
      twist.angular.z = teleop->second;
      teleop_pub_->publish(twist);
    } else if (topic == "action") {
      RCLCPP_INFO(get_logger(), "Got action: %s", payload.c_str());
      std_msgs::msg::String msg;
      msg.data = payload;
      action_pub_->publish(msg);
    } else if (topic == "rpc/request") {
      OnRpcRequest(payload);
    }
  }

  void OnRpcRequest(const std::string& payload) {
    RpcParseResult parsed = ParseRpcRequest(payload);
    if (parsed.kind == RpcParseResult::Kind::kIgnore) return;
    if (parsed.kind == RpcParseResult::Kind::kError) {
      TryPublish("rpc/response", parsed.error);
      return;
    }
    const RpcCall& call = parsed.call;
    json id = call.id.empty() ? json(nullptr) : json(call.id);

    auto local = local_methods_.find(call.method);
    if (local != local_methods_.end()) {
      try {
        json result = local->second(call.params);
        TryPublish("rpc/response", json{{"jsonrpc", "2.0"}, {"result", result}, {"id", id}}.dump(2));
      } catch (const RpcException& e) {
        TryPublish("rpc/response", RpcErrorJson(e.code(), e.what(), id));
      } catch (const std::exception& e) {
        TryPublish("rpc/response", RpcErrorJson(kRpcInternal, e.what(), id));
      }
      return;
    }

    bool registered = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& [_, methods] : methods_) {
        if (std::find(methods.begin(), methods.end(), call.method) != methods.end()) registered = true;
      }
    }
    if (!registered) {
      TryPublish("rpc/response", RpcErrorJson(kRpcMethodNotFound, "Method \"" + call.method + "\" not found", id));
      return;
    }
    mowbite_msgs::msg::RpcRequest msg;
    msg.method = call.method;
    msg.params = call.params.is_null() ? "" : call.params.dump();
    msg.id = call.id;
    rpc_request_pub_->publish(msg);
  }

  json MethodList() {
    json methods = json::array();
    for (const auto& [name, _] : local_methods_) methods.push_back(name);
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [_, ids] : methods_) {
      for (const auto& id : ids) methods.push_back(id);
    }
    std::sort(methods.begin(), methods.end());
    return methods;
  }

  void OnRobotState(const mowbite_msgs::msg::RobotState& msg) {
    // Booleans as 0/1 like the ROS 1 messages of OpenMower; the app's scripts compare them as numbers.
    json j;
    j["battery_percentage"] = msg.battery_percentage;
    j["gps_percentage"] = msg.gps_percentage;
    j["current_action_progress"] = msg.current_action_progress;
    j["current_state"] = msg.current_state;
    j["current_sub_state"] = msg.current_sub_state;
    j["current_area"] = msg.current_area;
    j["current_path"] = msg.current_path;
    j["current_path_index"] = msg.current_path_index;
    j["emergency"] = msg.emergency ? 1 : 0;
    j["is_charging"] = msg.is_charging ? 1 : 0;
    j["rain_detected"] = msg.rain_detected ? 1 : 0;
    j["pose"]["x"] = msg.pose_x;
    j["pose"]["y"] = msg.pose_y;
    j["pose"]["heading"] = msg.pose_heading;
    j["pose"]["pos_accuracy"] = msg.pos_accuracy;
    j["pose"]["heading_accuracy"] = msg.heading_accuracy;
    j["pose"]["heading_valid"] = msg.heading_valid ? 1 : 0;
    TryPublish("robot_state/json", j.dump());
  }

  void OnSensorInfo(const mowbite_msgs::msg::SensorInfo& msg) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = sensors_.find(msg.sensor_id);
      if (it != sensors_.end() && it->second == msg) return;
      sensors_[msg.sensor_id] = msg;
    }
    PublishSensorInfos();
  }

  void OnSensorValue(const mowbite_msgs::msg::SensorValue& msg) {
    bool is_text = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = sensors_.find(msg.sensor_id);
      if (it == sensors_.end()) return;
      is_text = it->second.value_type == mowbite_msgs::msg::SensorInfo::TYPE_STRING;
    }
    TryPublish("sensors/" + msg.sensor_id + "/data", is_text ? msg.text : std::to_string(msg.value));
  }

  void PublishSensorInfos() {
    json infos = json::array();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (sensors_.empty()) return;
      for (const auto& [_, s] : sensors_) {
        json info;
        info["sensor_id"] = s.sensor_id;
        info["sensor_name"] = s.sensor_name;
        info["value_type"] = s.value_type == mowbite_msgs::msg::SensorInfo::TYPE_STRING   ? "STRING"
                             : s.value_type == mowbite_msgs::msg::SensorInfo::TYPE_DOUBLE ? "DOUBLE"
                                                                                          : "UNKNOWN";
        info["value_description"] = ValueDescriptionName(s.value_description);
        info["unit"] = s.unit;
        info["has_min_max"] = s.has_min_max ? 1 : 0;
        info["min_value"] = s.min_value;
        info["max_value"] = s.max_value;
        info["has_critical_low"] = s.has_critical_low ? 1 : 0;
        info["lower_critical_value"] = s.lower_critical_value;
        info["has_critical_high"] = s.has_critical_high ? 1 : 0;
        info["upper_critical_value"] = s.upper_critical_value;
        infos.push_back(info);
      }
    }
    TryPublish("sensor_infos/json", infos.dump(), true);
  }

  void PublishActions() {
    json actions = json::array();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& [prefix, list] : actions_) {
        for (const auto& action : list) {
          actions.push_back({{"action_id", prefix + "/" + action.action_id},
                             {"action_name", action.action_name},
                             {"enabled", action.enabled ? 1 : 0}});
        }
      }
    }
    TryPublish("actions/json", actions.dump(), true);
  }

  void OnMap(const std::string& text) {
    std::string pretty;
    try {
      pretty = json::parse(text).dump(2);
    } catch (const json::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error processing map JSON: %s", e.what());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_ = pretty;
    }
    TryPublish("map/json", pretty, true);
  }

  void PublishEvent(const std::string& type) {
    const std::string payload = EventPayload(type, nullptr, now().seconds()).dump();
    TryPublish(kEventsTopic, payload);
    event_history_.add(payload);
  }

  void TryPublish(const std::string& topic, const std::string& payload, bool retain = false) {
    try {
      // Retained data with QoS 1, so it arrives at least once.
      client_->publish(topic, payload.data(), payload.size(), retain ? 1 : 0, retain);
    } catch (const mqtt::exception&) {
      // Disconnected, dropped.
    }
  }

  std::string version_;
  ParamsStore params_;
  Callback callback_;
  std::shared_ptr<mqtt::async_client> client_;
  std::atomic<bool> connecting_{false};

  std::mutex mutex_;
  std::map<std::string, std::vector<std::string>> methods_;
  std::map<std::string, std::vector<mowbite_msgs::msg::ActionInfo>> actions_;
  std::map<std::string, mowbite_msgs::msg::SensorInfo> sensors_;
  std::string map_;

  std::map<std::string, std::function<json(const json&)>> local_methods_;
  EventHistory event_history_;
  PositionHistory position_history_;
  std::mutex pose_mutex_;
  std::vector<PoseSample> pose_buffer_;
  LogBuffer log_buffer_{1000};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcRequest>::SharedPtr rpc_request_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::AbsolutePose>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::SensorInfo>::SharedPtr sensor_info_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::ActionList>::SharedPtr actions_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcResponse>::SharedPtr rpc_response_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcError>::SharedPtr rpc_error_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
  rclcpp::TimerBase::SharedPtr position_timer_;
  rclcpp::TimerBase::SharedPtr position_flush_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

}  // namespace mowbite_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_bridge::BridgeNode)
