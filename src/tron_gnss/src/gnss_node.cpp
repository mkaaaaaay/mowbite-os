#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <mowbite_msgs/msg/absolute_pose.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <vector>

#include "tron_gnss/gnss_tracker.h"
#include "tron_gnss/nmea.h"
#include "tron_mcu/serial_port.h"

namespace tron_gnss {

namespace {

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

// Reads NMEA from the receiver and publishes gps/absolute_pose. Never writes to the port.
class GnssNode : public rclcpp::Node {
 public:
  explicit GnssNode(const rclcpp::NodeOptions& options) : rclcpp::Node("tron_gnss", options) {
    port_name_ = declare_parameter("port", "/dev/ttyS4");
    baud_ = static_cast<int>(declare_parameter("baud", 115200));
    frame_id_ = declare_parameter("frame_id", "map");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double lat = declare_parameter("datum_lat", nan);
    const double lon = declare_parameter("datum_long", nan);
    const double height = declare_parameter("datum_height", nan);

    tron::GnssConfig config;
    config.accuracy_rtk_fixed_m = declare_parameter("accuracy_rtk_fixed", config.accuracy_rtk_fixed_m);
    config.accuracy_rtk_float_m = declare_parameter("accuracy_rtk_float", config.accuracy_rtk_float_m);
    config.accuracy_dgps_m = declare_parameter("accuracy_dgps", config.accuracy_dgps_m);
    config.accuracy_single_m = declare_parameter("accuracy_single", config.accuracy_single_m);
    config.motion_max_age_s = declare_parameter("motion_max_age", config.motion_max_age_s);

    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(height) || std::abs(lat) > 90.0 ||
        std::abs(lon) > 180.0) {
      RCLCPP_ERROR(get_logger(), "datum_lat, datum_long and datum_height are not set, no pose is published");
    } else {
      tracker_.emplace(config, tron::LocalFrame(lat, lon, height));
    }

    pose_pub_ = create_publisher<mowbite_msgs::msg::AbsolutePose>("gps/absolute_pose", 10);
    // The app reads the datum from params/json, the bridge collects it from this latched topic.
    if (std::isfinite(lat) && std::isfinite(lon) && std::abs(lat) <= 90.0 && std::abs(lon) <= 180.0) {
      params_pub_ = create_publisher<std_msgs::msg::String>(
          "bridge/params", rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local());
      char text[160];
      std::snprintf(text, sizeof(text), "{\"node\":\"/tron_gnss\",\"params\":{\"datum_lat\":%.15g,\"datum_long\":%.15g}}",
                    lat, lon);
      std_msgs::msg::String msg;
      msg.data = text;
      params_pub_->publish(msg);
    }
    io_thread_ = std::thread([this] { IoLoop(); });
  }

  ~GnssNode() override {
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
  }

 private:
  void IoLoop() {
    uint8_t buffer[512];
    double next_open = 0;
    double last_data = MonotonicSeconds();
    while (running_ && rclcpp::ok()) {
      double now = MonotonicSeconds();
      if (!port_.IsOpen()) {
        if (now >= next_open) {
          next_open = now + 1.0;
          TryOpen();
          last_data = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      ssize_t n = port_.Read(buffer, sizeof(buffer), 100);
      if (n < 0) {
        RCLCPP_ERROR(get_logger(), "Read error on %s, closing", port_name_.c_str());
        port_.Close();
        continue;
      }
      now = MonotonicSeconds();
      if (n > 0) {
        last_data = now;
        HandleBytes(buffer, static_cast<size_t>(n), now);
      } else if (now - last_data > 5.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "No data from %s for %.0f s", port_name_.c_str(),
                             now - last_data);
      }
    }
  }

  void TryOpen() {
    auto users = tron::ProcessesUsing(port_name_);
    if (!users.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "%s is open in another process (pid %d), not opening it",
                            port_name_.c_str(), users.front());
      return;
    }
    std::string error;
    if (!port_.Open(port_name_, baud_, error)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Cannot open %s: %s", port_name_.c_str(), error.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Opened %s at %d baud, read only", port_name_.c_str(), baud_);
  }

  void HandleBytes(const uint8_t* data, size_t size, double now) {
    std::vector<tron::NmeaMessage> messages;
    parser_.Feed(data, size, messages);
    if (!tracker_) return;
    for (const auto& message : messages) {
      auto pose = tracker_->Update(message, now);
      if (pose) Publish(*pose);
    }
    if (parser_.checksum_errors() + parser_.malformed() != reported_errors_) {
      reported_errors_ = parser_.checksum_errors() + parser_.malformed();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "NMEA errors so far: %lu checksum, %lu malformed",
                           static_cast<unsigned long>(parser_.checksum_errors()),
                           static_cast<unsigned long>(parser_.malformed()));
    }
  }

  void Publish(const tron::GnssPose& pose) {
    mowbite_msgs::msg::AbsolutePose msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.sensor_stamp = pose.sensor_stamp_ms;
    msg.received_stamp = static_cast<uint32_t>(MonotonicSeconds() * 1000.0);
    msg.source = mowbite_msgs::msg::AbsolutePose::SOURCE_GPS;
    msg.flags = pose.flags;
    msg.orientation_valid = 0;
    msg.motion_vector_valid = pose.motion_valid ? 1 : 0;
    msg.position_accuracy = static_cast<float>(pose.accuracy_m);
    msg.orientation_accuracy = 0.0f;
    msg.pose.pose.position.x = pose.position.east;
    msg.pose.pose.position.y = pose.position.north;
    msg.pose.pose.position.z = pose.position.up;
    msg.pose.pose.orientation.w = 1.0;
    msg.motion_vector.x = pose.motion_east_mps;
    msg.motion_vector.y = pose.motion_north_mps;
    msg.vehicle_heading = 0.0;
    msg.motion_heading = pose.motion_heading;
    pose_pub_->publish(msg);
  }

  std::string port_name_;
  int baud_;
  std::string frame_id_;
  tron::SerialPort port_;
  tron::NmeaParser parser_;
  std::optional<tron::GnssTracker> tracker_;
  uint64_t reported_errors_ = 0;
  rclcpp::Publisher<mowbite_msgs::msg::AbsolutePose>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  std::atomic<bool> running_{true};
  std::thread io_thread_;
};

}  // namespace tron_gnss

RCLCPP_COMPONENTS_REGISTER_NODE(tron_gnss::GnssNode)
