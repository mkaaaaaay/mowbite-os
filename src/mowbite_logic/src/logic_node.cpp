#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <mowbite_msgs/msg/absolute_pose.hpp>
#include <mowbite_msgs/msg/action_list.hpp>
#include <mowbite_msgs/msg/mcu_status.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/sensor_info.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "mowbite_bridge/monitor.h"
#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

namespace {

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

McuSnapshot::Motor ToMotor(const mowbite_msgs::msg::MotorState& m) {
  McuSnapshot::Motor motor;
  motor.ok = m.ok;
  motor.rpm = m.rpm;
  motor.current = m.current;
  motor.temperature = m.temperature;
  return motor;
}

}  // namespace

class LogicNode : public rclcpp::Node {
 public:
  explicit LogicNode(const rclcpp::NodeOptions& options) : rclcpp::Node("mowbite_logic", options), core_(LoadConfig()) {
    mcu_timeout_s_ = declare_parameter("mcu_status_timeout", 2.0);
    pose_timeout_s_ = declare_parameter("pose_timeout", 2.0);

    robot_state_pub_ = create_publisher<mowbite_msgs::msg::RobotState>("bridge/robot_state", 10);
    actions_pub_ = create_publisher<mowbite_msgs::msg::ActionList>("bridge/actions", LatchedQos(50));
    sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorInfo>("bridge/sensor_info", LatchedQos(100));
    sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("bridge/sensor_value", 100);
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // The app container rates the battery by /ll/board; any value except YardForce makes it show battery_percentage.
    params_pub_ = create_publisher<std_msgs::msg::String>(
        "bridge/params", rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local());
    std_msgs::msg::String board_msg;
    board_msg.data = nlohmann::json{{"node", "/ll"}, {"params", {{"board", declare_parameter("board", "Tron")}}}}.dump();
    params_pub_->publish(board_msg);
    emergency_client_ = create_client<std_srvs::srv::SetBool>("mcu/emergency");

    mcu_sub_ = create_subscription<mowbite_msgs::msg::McuStatus>(
        "mcu/status", 10, [this](mowbite_msgs::msg::McuStatus::ConstSharedPtr msg) { OnMcuStatus(*msg); });
    pose_sub_ = create_subscription<mowbite_msgs::msg::AbsolutePose>(
        "localization/pose", 10, [this](mowbite_msgs::msg::AbsolutePose::ConstSharedPtr msg) { OnPose(*msg); });
    action_sub_ = create_subscription<std_msgs::msg::String>(
        "bridge/action", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) { OnAction(msg->data); });
    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "bridge/teleop", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { OnTeleop(*msg); });

    for (const auto& sensor : core_.Sensors()) {
      mowbite_msgs::msg::SensorInfo info;
      info.sensor_id = sensor.id;
      info.sensor_name = sensor.name;
      info.value_type = sensor.value_type;
      info.value_description = sensor.value_description;
      info.unit = sensor.unit;
      info.has_min_max = sensor.has_min_max;
      info.min_value = sensor.min_value;
      info.max_value = sensor.max_value;
      info.has_critical_low = sensor.has_critical_low;
      info.lower_critical_value = sensor.lower_critical_value;
      info.has_critical_high = sensor.has_critical_high;
      info.upper_critical_value = sensor.upper_critical_value;
      sensor_info_pub_->publish(info);
    }

    state_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { OnStateTimer(); });
    sensor_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { PublishReadings(); });
    PublishActionsIfChanged();
  }

 private:
  LogicConfig LoadConfig() {
    LogicConfig config;
    config.battery_empty_voltage = declare_parameter("battery_empty_voltage", config.battery_empty_voltage);
    config.battery_full_voltage = declare_parameter("battery_full_voltage", config.battery_full_voltage);
    config.dock_charge_voltage = declare_parameter("dock_charge_voltage", config.dock_charge_voltage);
    config.max_position_accuracy = declare_parameter("max_position_accuracy", config.max_position_accuracy);
    return config;
  }

  void OnMcuStatus(const mowbite_msgs::msg::McuStatus& msg) {
    McuSnapshot mcu;
    mcu.telemetry_ok = msg.telemetry_ok;
    mcu.emergency_latched = msg.emergency_latched;
    mcu.lift = msg.lift;
    mcu.stop = msg.stop;
    mcu.bumper = msg.bumper;
    mcu.rain = msg.rain;
    mcu.battery_ok = msg.battery_ok;
    mcu.battery_voltage = msg.battery_voltage;
    mcu.battery_current = msg.battery_current;
    mcu.battery_percentage = msg.battery_percentage;
    mcu.battery_temperature = msg.battery_temperature;
    mcu.battery_dock = msg.battery_dock;
    mcu.cutter = ToMotor(msg.cutter);
    mcu.left = ToMotor(msg.left);
    mcu.right = ToMotor(msg.right);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      core_.OnMcuStatus(mcu);
      last_mcu_ = now();
    }
    Flush();
  }

  void OnPose(const mowbite_msgs::msg::AbsolutePose& msg) {
    PoseSnapshot pose;
    pose.x = msg.pose.pose.position.x;
    pose.y = msg.pose.pose.position.y;
    pose.heading = msg.vehicle_heading;
    pose.position_accuracy = msg.position_accuracy;
    pose.orientation_accuracy = msg.orientation_accuracy;
    pose.orientation_valid = msg.orientation_valid;
    std::lock_guard<std::mutex> lock(mutex_);
    core_.OnPose(pose);
    last_pose_ = now();
  }

  void OnAction(const std::string& action_id) {
    ActionResult result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result = core_.OnAction(action_id);
    }
    switch (result) {
      case ActionResult::kReleaseEmergency: ReleaseEmergency(); break;
      case ActionResult::kRejected: RCLCPP_WARN(get_logger(), "Action %s not possible now", action_id.c_str()); break;
      case ActionResult::kUnknown: RCLCPP_DEBUG(get_logger(), "Ignoring action %s", action_id.c_str()); break;
      case ActionResult::kDone: RCLCPP_INFO(get_logger(), "Action %s", action_id.c_str()); break;
    }
    Flush();
  }

  void ReleaseEmergency() {
    if (!emergency_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "mcu/emergency is not available, emergency stays");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    emergency_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "Emergency release requested");
  }

  void OnTeleop(const geometry_msgs::msg::Twist& msg) {
    std::optional<std::pair<double, double>> cmd;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = core_.OnTeleop(msg.linear.x, msg.angular.z, MonotonicSeconds());
    }
    if (!cmd) return;
    geometry_msgs::msg::Twist out;
    out.linear.x = cmd->first;
    out.angular.z = cmd->second;
    cmd_vel_pub_->publish(out);
  }

  void OnStateTimer() {
    RobotStateData state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!last_mcu_ || (now() - *last_mcu_).seconds() > mcu_timeout_s_) core_.OnMcuTimeout();
      if (!last_pose_ || (now() - *last_pose_).seconds() > pose_timeout_s_) core_.OnPoseTimeout();
      state = core_.RobotState();
    }
    Flush();
    mowbite_msgs::msg::RobotState msg;
    msg.current_state = state.state;
    msg.current_sub_state = state.sub_state;
    msg.current_area = -1;
    msg.current_path = -1;
    msg.current_path_index = -1;
    msg.battery_percentage = state.battery_percentage;
    msg.emergency = state.emergency;
    msg.is_charging = state.is_charging;
    msg.rain_detected = state.rain_detected;
    msg.gps_percentage = state.gps_percentage;
    msg.pose_x = state.pose_x;
    msg.pose_y = state.pose_y;
    msg.pose_heading = state.pose_heading;
    msg.pos_accuracy = state.pos_accuracy;
    msg.heading_accuracy = state.heading_accuracy;
    msg.heading_valid = state.heading_valid;
    robot_state_pub_->publish(msg);
  }

  void PublishReadings() {
    std::vector<SensorReading> readings;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      readings = core_.Readings();
    }
    for (const auto& r : readings) {
      mowbite_msgs::msg::SensorValue msg;
      msg.sensor_id = r.id;
      msg.value = r.value;
      msg.text = r.text;
      sensor_value_pub_->publish(msg);
    }
  }

  // Sends stop requests, events and changed actions collected by the core.
  void Flush() {
    std::vector<Event> events;
    bool actions_changed;
    bool stop;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      events = core_.TakeEvents();
      actions_changed = core_.TakeActionsChanged();
      stop = core_.TakeStopRequest();
    }
    if (stop) cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    for (auto& event : events) {
      mowbite_msgs::msg::MqttPublish msg;
      msg.topic = "events/json";
      msg.payload = mowbite_bridge::EventPayload(event.type, event.details, now().seconds()).dump();
      publish_pub_->publish(msg);
    }
    if (actions_changed) PublishActions();
  }

  void PublishActionsIfChanged() {
    bool changed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      changed = core_.TakeActionsChanged();
    }
    if (changed) PublishActions();
  }

  void PublishActions() {
    std::vector<ActionGroup> groups;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      groups = core_.Actions();
    }
    for (const auto& group : groups) {
      mowbite_msgs::msg::ActionList msg;
      msg.node_prefix = group.prefix;
      for (const auto& action : group.actions) {
        mowbite_msgs::msg::ActionInfo info;
        info.action_id = action.id;
        info.action_name = action.name;
        info.enabled = action.enabled;
        msg.actions.push_back(info);
      }
      actions_pub_->publish(msg);
    }
  }

  std::mutex mutex_;
  LogicCore core_;
  std::optional<rclcpp::Time> last_mcu_;
  std::optional<rclcpp::Time> last_pose_;
  double mcu_timeout_s_;
  double pose_timeout_s_;

  rclcpp::Publisher<mowbite_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::ActionList>::SharedPtr actions_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorInfo>::SharedPtr sensor_info_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr emergency_client_;
  rclcpp::Subscription<mowbite_msgs::msg::McuStatus>::SharedPtr mcu_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::AbsolutePose>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
};

}  // namespace mowbite_logic

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_logic::LogicNode)
