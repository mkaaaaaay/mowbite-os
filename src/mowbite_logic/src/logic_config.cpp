#include "mowbite_logic/logic_config.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <random>
#include <stdexcept>

#include <mowbite_hold/file_io.h>

namespace mowbite_logic {

double GeometryNumber(rclcpp::Node& node, const std::string& key) {
  const std::string name = "geometry." + key;
  if (!node.has_parameter(name)) node.declare_parameter(name, std::numeric_limits<double>::quiet_NaN());
  const double value = node.get_parameter(name).as_double();
  if (std::isnan(value)) throw std::invalid_argument(key + " is missing in the geometry file of the platform");
  return value;
}

LogicConfig LoadLogicConfig(rclcpp::Node& node) {
  const auto param = [&node](const char* name, auto value) { return node.declare_parameter(name, value); };
  LogicConfig config;
  config.battery_empty_voltage = param("battery_empty_voltage", config.battery_empty_voltage);
  config.battery_full_voltage = param("battery_full_voltage", config.battery_full_voltage);
  config.dock_charge_voltage = param("dock_charge_voltage", config.dock_charge_voltage);
  config.max_position_accuracy = param("max_position_accuracy", config.max_position_accuracy);
  config.recorder.min_area = param("recording_min_area", config.recorder.min_area);
  config.recorder.max_points =
      static_cast<size_t>(param("recording_max_points", static_cast<int64_t>(config.recorder.max_points)));
  config.recorder.min_dock_distance = param("recording_min_dock_distance", config.recorder.min_dock_distance);
  config.mow_with_blade = param("mow_with_blade", config.mow_with_blade);
  config.mower_spinup_rpm = static_cast<int>(param("mower_spinup_rpm", static_cast<int64_t>(config.mower_spinup_rpm)));
  config.mower_spinup_timeout = param("mower_spinup_timeout", config.mower_spinup_timeout);
  config.blade_running_rpm =
      static_cast<int>(param("blade_running_rpm", static_cast<int64_t>(config.blade_running_rpm)));
  config.gps_wait_time = param("gps_wait_time", config.gps_wait_time);
  config.battery_critical_voltage = param("battery_critical_voltage", config.battery_critical_voltage);
  config.battery_critical_time = param("battery_critical_time", config.battery_critical_time);
  config.battery_low_window = param("battery_low_window", config.battery_low_window);
  config.battery_hot_temperature = param("battery_hot_temperature", config.battery_hot_temperature);
  const std::string battery = BatteryVoltagesProblem(config);
  if (!battery.empty()) throw std::invalid_argument(battery);
  if (!(config.battery_critical_time > 0) || !(config.battery_low_window > 0) ||
      !(config.battery_hot_temperature >= 0)) {
    throw std::invalid_argument(
        "battery_critical_time and battery_low_window need to be above 0, battery_hot_temperature 0 or above");
  }
  config.approach_distance = param("approach_distance", config.approach_distance);
  config.checkpoint_period = param("checkpoint_period", config.checkpoint_period);
  // Where the charging contacts are belongs to the dimensions of the platform (geometry.dock_contact_x).
  config.dock_contact_x = GeometryNumber(node, "dock_contact_x");
  config.dock_max_position_accuracy = param("dock_max_position_accuracy", config.dock_max_position_accuracy);
  config.dock_reference_limit = param("dock_reference_limit", config.dock_reference_limit);
  // undock_distance is a setting of params.set (LoadLogicParams), the rest is the platform's.
  config.undock.distance = param("undock_distance", config.undock.distance);
  config.undock.speed = param("undock_speed", config.undock.speed);
  config.undock.stall_time = param("undock_stall_time", config.undock.stall_time);
  config.undock.course_limit = param("undock_course_limit", config.undock.course_limit * 180.0 / M_PI) * M_PI / 180.0;
  // docking_approach_distance and docking_retry_count are settings of params.set (LoadLogicParams).
  config.dock.approach_distance = param("docking_approach_distance", config.dock.approach_distance);
  config.dock.retry_count = static_cast<int>(
      param("docking_retry_count", static_cast<int64_t>(config.dock.retry_count)));
  config.dock.speed = param("docking_speed", config.dock.speed);
  config.dock.contact_speed = param("contact_speed", config.dock.contact_speed);
  config.dock.gate_distance = param("dock_gate_distance", config.dock.gate_distance);
  config.dock.gate_lateral = param("dock_gate_lateral", config.dock.gate_lateral);
  config.dock.gate_heading = param("dock_gate_heading", config.dock.gate_heading * 180.0 / M_PI) * M_PI / 180.0;
  config.dock.overshoot = param("dock_overshoot", config.dock.overshoot);
  config.dock.contact_timeout = param("contact_timeout", config.dock.contact_timeout);
  // The straight way to the station where the planner gives no answer, the platform's.
  config.dock.direct = param("docking_direct", config.dock.direct);
  config.dock.direct_margin = param("direct_margin", config.dock.direct_margin);
  if (!(config.dock.direct_margin > 0) || !std::isfinite(config.dock.direct_margin)) {
    throw std::invalid_argument("direct_margin needs to be above 0");
  }
  // The charge that confirms docking and whose loss on the station is told, the platform's.
  config.charge.current = param("charge_confirm_current", config.charge.current);
  config.charge.time = param("charge_confirm_time", config.charge.time);
  config.charge.timeout = param("charge_confirm_timeout", config.charge.timeout);
  config.charge.expected_below = param("charge_expected_below", config.charge.expected_below);
  config.charge.lost_time = param("charge_lost_time", config.charge.lost_time);
  if (!(config.charge.current > 0) || !(config.charge.time >= 0) || !(config.charge.timeout > config.charge.time) ||
      !(config.charge.lost_time > 0)) {
    throw std::invalid_argument(
        "charge_confirm_current and charge_lost_time need to be above 0, charge_confirm_timeout above "
        "charge_confirm_time");
  }
  // The calibration drive: what a start needs, and how long the program may be silent.
  config.calibration.max_position_accuracy =
      param("calibration_max_position_accuracy", config.calibration.max_position_accuracy);
  config.calibration.min_battery = static_cast<int>(
      param("calibration_min_battery", static_cast<int64_t>(config.calibration.min_battery)));
  config.calibration.status_timeout = param("calibration_status_timeout", config.calibration.status_timeout);
  config.calibration.timeout = param("calibration_timeout", config.calibration.timeout);
  config.id_seed = (static_cast<uint64_t>(std::random_device{}()) << 32) | std::random_device{}();
  return config;
}

std::string LoadRecordedBy(const std::string& data_dir, std::string* problem) {
  const std::string path = data_dir.empty() ? "" : data_dir + "/recorded_by";
  if (!path.empty()) {
    std::ifstream in(path);
    std::string id;
    if (in >> id && id.size() == 32 && id.find_first_not_of("0123456789abcdef") == std::string::npos) return id;
  }
  std::random_device source;
  char hex[33];
  for (size_t i = 0; i < 4; i++) std::snprintf(hex + 8 * i, 9, "%08x", static_cast<unsigned>(source()));
  std::string error;
  if (!path.empty() && !mowbite_hold::WriteFileSafely(path, std::string(hex) + "\n", &error)) {
    *problem = "Cannot write " + path + " (" + error + ")";
  }
  return hex;
}

}  // namespace mowbite_logic
