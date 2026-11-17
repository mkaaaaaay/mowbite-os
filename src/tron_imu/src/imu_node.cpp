#include <atomic>
#include <chrono>
#include <memory>
#include <mowbite_component/node_options.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>
#include <vector>

#include "mowbite_serial/serial_port.h"
#include "mowbite_watch/attach.h"
#include "tron_imu/wit_imu.h"
#include "tron_imu/wit_parser.h"

namespace tron_imu {

namespace {

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

// Reads the WIT sensor and publishes imu/data. Never writes to the port.
class ImuNode : public rclcpp::Node {
 public:
  explicit ImuNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("tron_imu", mowbite_component::StartupOnly(options)) {
    port_name_ = declare_parameter("port", "/dev/ttyS1");
    baud_ = static_cast<int>(declare_parameter("baud", 115200));
    frame_id_ = declare_parameter("frame_id", "base_link");
    tron::WitImuConfig config;
    config.mount_roll_deg = declare_parameter("mount_roll_deg", config.mount_roll_deg);
    config.mount_pitch_deg = declare_parameter("mount_pitch_deg", config.mount_pitch_deg);
    config.mount_yaw_deg = declare_parameter("mount_yaw_deg", config.mount_yaw_deg);
    // The 6-axis yaw is integrated from the gyro and drifts: tilt is trusted, yaw is not.
    tilt_variance_ = declare_parameter("tilt_variance", 0.0004);
    yaw_variance_ = declare_parameter("yaw_variance", 1000.0);
    imu_ = std::make_unique<tron::WitImu>(config);

    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    watch_ = mowbite_watch::Attach(*this);
    io_thread_ = std::thread([this] { IoLoop(); });
  }

  ~ImuNode() override {
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
        HandleBytes(buffer, static_cast<size_t>(n));
      } else if (now - last_data > 1.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "No data from %s for %.0f s", port_name_.c_str(),
                             now - last_data);
      }
    }
  }

  void TryOpen() {
    auto users = mowbite_serial::ProcessesUsing(port_name_);
    if (!users.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "%s is open in another process (pid %d), not opening it",
                            port_name_.c_str(), users.front());
      return;
    }
    std::string error;
    if (!port_.Open(port_name_, baud_, error, true)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Cannot open %s: %s", port_name_.c_str(), error.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Opened %s at %d baud, read only", port_name_.c_str(), baud_);
  }

  void HandleBytes(const uint8_t* data, size_t size) {
    packets_.clear();
    parser_.Feed(data, size, packets_);
    for (const auto& packet : packets_) {
      auto sample = imu_->Update(packet);
      if (sample && Plausible(*sample)) Publish(*sample);
    }
    if (parser_.checksum_errors() != reported_errors_) {
      reported_errors_ = parser_.checksum_errors();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "WIT checksum errors so far: %lu",
                           static_cast<unsigned long>(reported_errors_));
    }
    if (parser_.unknown_types() && !unknown_logged_) {
      unknown_logged_ = true;
      RCLCPP_INFO(get_logger(), "Skipping WIT packets of type 0x%02x and other types; 0x51-0x53 are used",
                  parser_.last_unknown_type());
    }
  }

  // Upside down at rest no sample goes out: the driver and the safety stop on the missing IMU.
  bool Plausible(const tron::ImuSample& sample) {
    const bool was = upside_down_.upside_down();
    const bool now = upside_down_.Update(sample, MonotonicSeconds());
    if (now && !was) {
      RCLCPP_ERROR(get_logger(), "IMU upside down (a_z %.1f m/s2 at rest): check mount_roll_deg; no imu/data until it "
                   "reads upright", sample.acceleration[2]);
    } else if (was && !now) {
      RCLCPP_INFO(get_logger(), "IMU upright again (a_z %.1f m/s2): imu/data goes on", sample.acceleration[2]);
    }
    return !now;
  }

  void Publish(const tron::ImuSample& s) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.orientation.x = s.orientation[0];
    msg.orientation.y = s.orientation[1];
    msg.orientation.z = s.orientation[2];
    msg.orientation.w = s.orientation[3];
    msg.orientation_covariance = {tilt_variance_, 0, 0, 0, tilt_variance_, 0, 0, 0, yaw_variance_};
    msg.linear_acceleration.x = s.acceleration[0];
    msg.linear_acceleration.y = s.acceleration[1];
    msg.linear_acceleration.z = s.acceleration[2];
    msg.angular_velocity.x = s.angular_velocity[0];
    msg.angular_velocity.y = s.angular_velocity[1];
    msg.angular_velocity.z = s.angular_velocity[2];
    // Not measured separately; the sensor data sheet gives no values.
    msg.linear_acceleration_covariance[0] = -1.0;
    msg.angular_velocity_covariance[0] = -1.0;
    pub_->publish(msg);
  }

  std::string port_name_;
  int baud_;
  std::string frame_id_;
  double tilt_variance_;
  double yaw_variance_;
  mowbite_serial::SerialPort port_;
  tron::WitParser parser_;
  std::unique_ptr<tron::WitImu> imu_;
  std::vector<tron::WitPacket> packets_;
  uint64_t reported_errors_ = 0;
  bool unknown_logged_ = false;
  tron::UpsideDownCheck upside_down_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::atomic<bool> running_{true};
  std::thread io_thread_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace tron_imu

RCLCPP_COMPONENTS_REGISTER_NODE(tron_imu::ImuNode)
