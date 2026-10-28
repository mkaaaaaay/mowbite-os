#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mowbite_msgs/msg/mcu_status.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>

#include "tron_mcu/mcu_codec.h"
#include "tron_mcu/mcu_control.h"
#include "tron_mcu/serial_port.h"

namespace tron_mcu {

namespace {

using tron::Module;

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kStandardGravity = 9.80665;
// Battery and motor reports not refreshed within this time are treated as absent.
constexpr double kReportMaxAge = 3.0;

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

tron::WallClock LocalWallClock() {
  using namespace std::chrono;
  auto now = system_clock::now();
  std::time_t seconds = system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&seconds, &local);
  tron::WallClock wall;
  wall.year = static_cast<uint16_t>(local.tm_year + 1900);
  wall.month = static_cast<uint8_t>(local.tm_mon + 1);
  wall.day = static_cast<uint8_t>(local.tm_mday);
  wall.hour = static_cast<uint8_t>(local.tm_hour);
  wall.minute = static_cast<uint8_t>(local.tm_min);
  wall.second = static_cast<uint8_t>(local.tm_sec);
  wall.millisecond = static_cast<uint16_t>(duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000);
  return wall;
}

std::string VersionString(const tron::Version& v) {
  std::ostringstream s;
  s << static_cast<int>(v.major) << "." << static_cast<int>(v.minor) << "." << static_cast<int>(v.patch);
  return s.str();
}

}  // namespace

class McuNode : public rclcpp::Node {
 public:
  explicit McuNode(const rclcpp::NodeOptions& options) : rclcpp::Node("tron_mcu", options), control_(LoadConfig()) {
    port_name_ = declare_parameter("port", "/dev/ttyS9");
    measured_linear_scale_ = declare_parameter("measured_linear_scale", 1.0);
    measured_angular_scale_ = declare_parameter("measured_angular_scale", 1.0);
    drive_current_scale_ = declare_parameter("drive_current_scale", 0.001);
    imu_roll_offset_deg_ = declare_parameter("imu_roll_offset_deg", 0.0);
    imu_pitch_offset_deg_ = declare_parameter("imu_pitch_offset_deg", 0.0);
    expected_cutter_version_ = declare_parameter("expected_cutter_version", "");
    expected_chassis_version_ = declare_parameter("expected_chassis_version", "");

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("wheel_twist", 10);
    status_pub_ = create_publisher<mowbite_msgs::msg::McuStatus>("mcu/status", 10);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { OnCmdVel(*msg); });
    if (declare_parameter("send_imu", true)) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImu(*msg); });
    }
    emergency_srv_ = create_service<std_srvs::srv::SetBool>(
        "mcu/emergency", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                std_srvs::srv::SetBool::Response::SharedPtr res) { OnEmergency(*req, *res); });
    blade_srv_ = create_service<std_srvs::srv::SetBool>(
        "mcu/blade", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr res) { OnBlade(*req, *res); });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { PublishStatus(); });

    io_thread_ = std::thread([this] { IoLoop(); });
  }

  ~McuNode() override {
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
  }

 private:
  tron::McuControlConfig LoadConfig() {
    tron::McuControlConfig config;
    config.cmd_vel_timeout_s = declare_parameter("cmd_vel_timeout", config.cmd_vel_timeout_s);
    config.telemetry_timeout_s = declare_parameter("telemetry_timeout", config.telemetry_timeout_s);
    config.allow_blade = declare_parameter("allow_blade", false);
    config.blade_speed_percent = static_cast<uint16_t>(declare_parameter("blade_speed_percent", 100));
    config.mow_height_mm = static_cast<uint16_t>(declare_parameter("mow_height_mm", 60));
    config.bumper_latches_emergency = declare_parameter("bumper_latches_emergency", true);
    RCLCPP_INFO(get_logger(), "allow_blade: %d, cmd_vel_timeout: %.2f s, mow_height: %u mm", config.allow_blade,
                config.cmd_vel_timeout_s, config.mow_height_mm);
    return config;
  }

  // Owns the port: reads telemetry, runs the control tick every 10 ms and writes all frames.
  void IoLoop() {
    uint8_t buffer[512];
    double next_tick = 0;
    double next_open = 0;
    while (running_ && rclcpp::ok()) {
      double now = MonotonicSeconds();
      if (!port_.IsOpen() && now >= next_open) {
        next_open = now + 1.0;
        TryOpen();
      }
      if (port_.IsOpen()) {
        int wait_ms = std::clamp(static_cast<int>((next_tick - now) * 1000.0), 0, 5);
        ssize_t n = port_.Read(buffer, sizeof(buffer), wait_ms);
        if (n < 0) {
          RCLCPP_ERROR(get_logger(), "Read error on %s, closing", port_name_.c_str());
          port_.Close();
        } else if (n > 0) {
          HandleBytes(buffer, static_cast<size_t>(n));
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }

      now = MonotonicSeconds();
      // Nothing is computed while the port is closed, so nothing is lost either.
      if (port_.IsOpen() && now >= next_tick) {
        next_tick = now + 0.01;
        std::vector<tron::SubPacket> packets;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          packets = control_.Tick(now, LocalWallClock());
        }
        Send(packets);
      }
    }
    SendShutdownFrames();
  }

  void TryOpen() {
    auto users = tron::ProcessesUsing(port_name_);
    if (!users.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "%s is open in another process (pid %d), not opening it",
                            port_name_.c_str(), users.front());
      return;
    }
    std::string error;
    if (!port_.Open(port_name_, 115200, error)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Cannot open %s: %s", port_name_.c_str(), error.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      control_.OnConnected(MonotonicSeconds());
    }
    RCLCPP_INFO(get_logger(), "Opened %s", port_name_.c_str());
  }

  void Send(const std::vector<tron::SubPacket>& packets) {
    for (const auto& packet : packets) {
      if (!port_.Write(tron::EncodeFrame({packet}))) {
        RCLCPP_ERROR(get_logger(), "Write error on %s, closing", port_name_.c_str());
        port_.Close();
        return;
      }
    }
  }

  void SendShutdownFrames() {
    if (!port_.IsOpen()) return;
    // Stop wheels if they were commanded, switch the blade off and lift the deck.
    std::vector<tron::SubPacket> packets;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (control_.Status(MonotonicSeconds()).moving) packets.push_back(*tron::SpeedPacket(0.0f, 0.0f));
    }
    packets.push_back(tron::CutterPacket(tron::CutterCommand{}));
    Send(packets);
    port_.Close();
  }

  void HandleBytes(const uint8_t* data, size_t length) {
    std::vector<tron::MeasuredSpeed> speeds;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      double now = MonotonicSeconds();
      for (const auto& frame : parser_.Push(data, length)) {
        for (const auto& packet : frame) {
          if (packet.type != static_cast<uint8_t>(tron::PacketType::kMcuToHost)) continue;
          if (packet.module == static_cast<uint8_t>(Module::kSpeed)) {
            if (auto speed = tron::DecodeSpeed(packet.payload)) speeds.push_back(*speed);
          } else {
            HandleTelemetry(packet, now);
          }
        }
      }
    }
    for (const auto& speed : speeds) PublishTwist(speed);
  }

  // Called with mutex_ held.
  void HandleTelemetry(const tron::SubPacket& packet, double now) {
    switch (static_cast<Module>(packet.module)) {
      case Module::kSensor:
        if (auto info = tron::DecodeSensorInfo(packet.payload)) {
          control_.OnSensorInfo(*info, now);
          sensor_ = *info;
        }
        break;
      case Module::kBattery:
        if (auto battery = tron::DecodeBattery(packet.payload)) {
          control_.OnDockContactSample(battery->dock_ok == 1);
          battery_ = *battery;
          battery_s_ = now;
        }
        break;
      case Module::kMotors:
        if (auto motors = tron::DecodeMotors(packet.payload)) {
          motors_ = *motors;
          motors_s_ = now;
        }
        break;
      case Module::kVersion:
        if (auto versions = tron::DecodeVersionInfo(packet.payload)) CheckVersions(*versions);
        break;
      case Module::kBmsVersion:
        if (auto bms = tron::DecodeBmsVersion(packet.payload)) bms_version_ = VersionString(*bms);
        break;
      case Module::kLog: RCLCPP_INFO(get_logger(), "MCU log: %s", tron::DecodeLog(packet.payload).c_str()); break;
      case Module::kUptime:
      case Module::kCalibration: break;
      default:
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 10000, "Unhandled MCU module %d with %zu bytes",
                              packet.module, packet.payload.size());
    }
  }

  // Called with mutex_ held.
  void CheckVersions(const tron::VersionInfo& versions) {
    std::string cutter = VersionString(versions.cutter_board);
    std::string chassis = VersionString(versions.chassis_board);
    if (cutter != cutter_version_ || chassis != chassis_version_) {
      RCLCPP_INFO(get_logger(), "MCU versions: cutter %s, chassis %s", cutter.c_str(), chassis.c_str());
      cutter_version_ = cutter;
      chassis_version_ = chassis;
    }
    bool cutter_ok = expected_cutter_version_.empty() || cutter == expected_cutter_version_;
    bool chassis_ok = expected_chassis_version_.empty() || chassis == expected_chassis_version_;
    if (!cutter_ok || !chassis_ok) {
      // A firmware the driver was not checked against keeps the emergency latched.
      control_.LatchEmergency();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Unexpected MCU firmware: cutter %s, chassis %s",
                            cutter.c_str(), chassis.c_str());
    }
  }

  void PublishTwist(const tron::MeasuredSpeed& speed) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = speed.linear * measured_linear_scale_;
    msg.twist.angular.z = speed.angular * measured_angular_scale_;
    twist_pub_->publish(msg);
  }

  mowbite_msgs::msg::MotorState MotorMessage(const tron::MotorInfo& motor, bool fresh, double current_scale) const {
    mowbite_msgs::msg::MotorState msg;
    msg.ok = fresh;
    if (!fresh) return msg;
    msg.rpm = motor.rpm;
    msg.current = static_cast<float>(motor.current_raw * current_scale);
    msg.temperature = motor.temperature_c;
    msg.status = motor.status;
    return msg;
  }

  void PublishStatus() {
    mowbite_msgs::msg::McuStatus msg;
    msg.stamp = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      double now_s = MonotonicSeconds();
      const auto state = control_.Status(now_s);
      msg.telemetry_ok = !state.telemetry_stale;
      msg.emergency_latched = state.latched;
      msg.lift = state.lift;
      msg.stop = state.stop;
      msg.bumper = state.bumper;
      msg.bumper_left = msg.telemetry_ok && sensor_.bumper_left;
      msg.bumper_right = msg.telemetry_ok && sensor_.bumper_right;
      msg.rain = msg.telemetry_ok && sensor_.rain;
      msg.cutter_size = sensor_.cutter_size;
      msg.moving = state.moving;
      msg.blade_on = state.blade_on;
      msg.dock_contact = state.dock_contact;

      msg.battery_ok = battery_s_ && now_s - *battery_s_ < kReportMaxAge;
      if (msg.battery_ok) {
        msg.battery_voltage = static_cast<float>(battery_.voltage_v);
        msg.battery_current = static_cast<float>(battery_.current_a);
        msg.battery_percentage = battery_.percentage;
        msg.battery_temperature = battery_.temperature_c;
        msg.battery_dock = battery_.dock_ok == 1;
        msg.battery_error_bits = battery_.error_bits;
      }

      bool motors_ok = motors_s_ && now_s - *motors_s_ < kReportMaxAge;
      // Cutter current comes in 10 mA steps.
      msg.cutter = MotorMessage(motors_.cutter, motors_ok, 0.01);
      msg.left = MotorMessage(motors_.left, motors_ok, drive_current_scale_);
      msg.right = MotorMessage(motors_.right, motors_ok, drive_current_scale_);

      msg.cutter_version = cutter_version_;
      msg.chassis_version = chassis_version_;
      msg.bms_version = bms_version_;
      const auto& stats = parser_.Stats();
      msg.frames_ok = stats.frames;
      msg.frame_errors = stats.checksum_errors + stats.length_errors + stats.escape_errors + stats.overflows;
    }
    status_pub_->publish(msg);
  }

  void OnCmdVel(const geometry_msgs::msg::Twist& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    control_.OnCmdVel(static_cast<float>(msg.linear.x), static_cast<float>(msg.angular.z), MonotonicSeconds());
  }

  void OnImu(const sensor_msgs::msg::Imu& msg) {
    double roll, pitch, yaw = 0;
    const auto& a = msg.linear_acceleration;
    if (msg.orientation_covariance[0] >= 0) {
      const auto& q = msg.orientation;
      roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
      pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
      yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    } else {
      // No orientation in the message: tilt from gravity.
      roll = std::atan2(a.y, a.z);
      pitch = std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z));
    }
    // Like the vendor, the rate fields stay 0.
    auto counts = tron::ImuCountsFromSi(
        pitch * kRadToDeg + imu_pitch_offset_deg_, roll * kRadToDeg + imu_roll_offset_deg_, yaw * kRadToDeg,
        {a.x / kStandardGravity, a.y / kStandardGravity, a.z / kStandardGravity}, {0.0, 0.0, 0.0});
    std::lock_guard<std::mutex> lock(mutex_);
    control_.OnImu(counts);
  }

  void OnEmergency(const std_srvs::srv::SetBool::Request& req, std_srvs::srv::SetBool::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (req.data) {
      RCLCPP_WARN(get_logger(), "Emergency latched by request");
      control_.LatchEmergency();
      res.message = "latched";
    } else {
      control_.ReleaseEmergency(MonotonicSeconds());
      res.message = "release requested";
    }
    res.success = true;
  }

  void OnBlade(const std_srvs::srv::SetBool::Request& req, std_srvs::srv::SetBool::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    res.success = control_.SetBlade(req.data);
    res.message = res.success ? "ok" : "allow_blade is false";
  }

  std::mutex mutex_;
  tron::McuControl control_;
  tron::FrameParser parser_;
  tron::SensorInfo sensor_;
  tron::BatteryInfo battery_;
  std::optional<double> battery_s_;
  tron::MotorsInfo motors_;
  std::optional<double> motors_s_;
  std::string cutter_version_;
  std::string chassis_version_;
  std::string bms_version_;

  tron::SerialPort port_;
  std::string port_name_;
  std::atomic<bool> running_{true};
  std::thread io_thread_;

  double measured_linear_scale_;
  double measured_angular_scale_;
  double drive_current_scale_;
  double imu_roll_offset_deg_;
  double imu_pitch_offset_deg_;
  std::string expected_cutter_version_;
  std::string expected_chassis_version_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::McuStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr blade_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace tron_mcu

RCLCPP_COMPONENTS_REGISTER_NODE(tron_mcu::McuNode)
