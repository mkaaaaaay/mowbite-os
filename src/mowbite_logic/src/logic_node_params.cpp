// The settings of the logic at the node: the parameter files, what params.set saved, mower_logic.json and
// params/json.
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

#include "logic_node.h"

namespace mowbite_logic {

// <data_dir>/params/mower_logic.json, written like the checkpoint.
std::string LogicNode::LogicParamsPath() const {
  return data_dir_.empty() ? "" : data_dir_ + "/params/mower_logic.json";
}

// The parameter files, then what params.set saved. A file that cannot be read stays as it is, the parameter files
// hold.
void LogicNode::LoadLogicParams() {
  json base = json::object();
  base["max_position_accuracy"] = get_parameter("max_position_accuracy").as_double();
  base["gps_wait_time"] = get_parameter("gps_wait_time").as_double();
  base["gps_timeout"] = declare_parameter("gps_timeout", 10.0);
  base["undock_distance"] = get_parameter("undock_distance").as_double();
  base["docking_approach_distance"] = get_parameter("docking_approach_distance").as_double();
  base["docking_retry_count"] = get_parameter("docking_retry_count").as_int();
  logic_params_ = LogicParams(base);
  const std::string path = LogicParamsPath();
  if (path.empty()) {
    RCLCPP_WARN(get_logger(), "No data_dir: what params.set changes is gone after a restart");
  } else if (std::ifstream in(path); in) {
    std::stringstream text;
    text << in.rdbuf();
    if (logic_params_.Load(text.str())) {
      RCLCPP_INFO(get_logger(), "Settings from %s", path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s is not valid, the settings of the parameter files hold", path.c_str());
    }
  }
  ApplyLogicParams();
}

// What of the settings of params.set the core uses. Called with the mutex held, or before the node runs.
void LogicNode::ApplyLogicParams() {
  core_.Call<&LogicCore::SetGpsSettings>(logic_params_.Get("max_position_accuracy"), logic_params_.Get("gps_wait_time"),
                                         logic_params_.Get("gps_timeout"));
  core_.Call<&LogicCore::SetUndockDistance>(logic_params_.Get("undock_distance"));
  core_.Call<&LogicCore::SetDockingSettings>(logic_params_.Get("docking_approach_distance"),
                                             static_cast<int>(logic_params_.Get("docking_retry_count")));
  core_.Call<&LogicCore::SetHomeSettings>(
      static_cast<int>(logic_params_.Get("rain_mode")), logic_params_.Get("rain_check_seconds"),
      logic_params_.Get("motor_hot_temperature"), logic_params_.Get("motor_cold_temperature"));
  core_.Call<&LogicCore::SetSunsetSettings>(logic_params_.Get("dock_at_sunset") != 0);
  ApplyObstacleModeLocked();
  TellRainWithoutSensor();
}

// rain_mode other than 0 does nothing on a platform without a rain sensor: told once, again after it was 0.
void LogicNode::TellRainWithoutSensor() {
  const int mode = static_cast<int>(logic_params_.Get("rain_mode"));
  if (mode == 0 || !rain_sensor_ || *rain_sensor_) {
    rain_told_ = false;
    return;
  }
  if (rain_told_) return;
  rain_told_ = true;
  // The default is no mistake of the user, another value was chosen.
  constexpr char kText[] = "rain_mode %d has no effect: this platform has no rain sensor (hw/capabilities)";
  if (mode == static_cast<int>(LogicParams().Get("rain_mode"))) {
    RCLCPP_INFO(get_logger(), kText, mode);
  } else {
    RCLCPP_WARN(get_logger(), kText, mode);
  }
}

bool LogicNode::SaveLogicParams(const LogicParams& params, std::string& error) {
  const std::string path = LogicParamsPath();
  return path.empty() || mowbite_hold::WriteFileSafely(path, params.Text(), &error);
}

void LogicNode::PublishLogicParams() {
  LogicParams params;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    params = logic_params_;
  }
  std_msgs::msg::String msg;
  msg.data = json{{"node", kLogicParamsNode}, {"params", params.Values()}}.dump();
  logic_params_pub_->publish(msg);
  mowbite_msgs::msg::GpsLimits limits;
  limits.max_position_accuracy = params.Get("max_position_accuracy");
  limits.gps_timeout = params.Get("gps_timeout");
  gps_limits_pub_->publish(limits);
}

}  // namespace mowbite_logic
