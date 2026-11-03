#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mowbite_msgs/msg/robot_pose.hpp>
#include <mowbite_msgs/msg/drive_command.hpp>
#include <mowbite_msgs/msg/gps_limits.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/map_status.hpp>
#include <mowbite_msgs/msg/safety_status.hpp>
#include <mowbite_msgs/srv/set_emergency.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <stdexcept>
#include <std_msgs/msg/string.hpp>

#include "mowbite_safety/safety_filter.h"

namespace mowbite_safety {

namespace {

using mowbite_msgs::msg::DriveCommand;
using mowbite_msgs::msg::HardwareStatus;
using mowbite_msgs::srv::SetEmergency;

static_assert(kReasonTilt == HardwareStatus::REASON_TILT);
static_assert(kReasonFence == HardwareStatus::REASON_FENCE);
static_assert(static_cast<uint8_t>(Source::kStop) == DriveCommand::SOURCE_STOP);
static_assert(static_cast<uint8_t>(Source::kTeleop) == DriveCommand::SOURCE_TELEOP);
static_assert(static_cast<uint8_t>(Source::kFollower) == DriveCommand::SOURCE_FOLLOWER);

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

rclcpp::QoS LatchedQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

std::string BitNames(uint32_t bits) {
  std::string names;
  if (bits & kReasonTilt) names = "TILT";
  if (bits & kReasonFence) names += names.empty() ? "FENCE" : ", FENCE";
  return names;
}

}  // namespace

// S2, the only writer of hw/cmd_vel: passes the drive commands of the logic (safety/drive) that SafetyFilter allows,
// latches TILT and FENCE over hw/emergency, and lets a release of the logic (safety/release_emergency) through to the
// driver only while the mower is not tilted.
class SafetyNode : public rclcpp::Node {
 public:
  explicit SafetyNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_safety", options), filter_(LoadConfig()) {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("hw/cmd_vel", 10);
    status_pub_ = create_publisher<mowbite_msgs::msg::SafetyStatus>("safety/status", 10);
    emergency_client_ = create_client<SetEmergency>("hw/emergency");

    drive_sub_ = create_subscription<DriveCommand>(
        "safety/drive", 10, [this](DriveCommand::ConstSharedPtr msg) { OnDrive(*msg); });
    hw_sub_ = create_subscription<HardwareStatus>("hw/status", 10, [this](HardwareStatus::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      // Without telemetry the driver cannot stop the mower either.
      filter_.OnHardware(msg->emergency || !msg->telemetry_ok, msg->emergency_reason, MonotonicSeconds());
    });
    caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", LatchedQos(), [this](mowbite_msgs::msg::HardwareCapabilities::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          filter_.OnCapabilities(!msg->safety_measurement_date.empty(), msg->max_linear_speed, msg->max_angular_speed);
        });
    imu_in_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImu(*msg); });
    pose_sub_ = create_subscription<mowbite_msgs::msg::RobotPose>(
        "localization/pose", 10, [this](mowbite_msgs::msg::RobotPose::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          filter_.OnPose(msg->x, msg->y, msg->position_accuracy, msg->heading_valid, MonotonicSeconds());
        });
    map_sub_ = create_subscription<std_msgs::msg::String>(
        "map/json", LatchedQos(), [this](std_msgs::msg::String::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          filter_.SetMap(msg->data);
        });
    map_status_sub_ = create_subscription<mowbite_msgs::msg::MapStatus>(
        "map/status", LatchedQos(), [this](mowbite_msgs::msg::MapStatus::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          filter_.SetMapFits(msg->datum_state == mowbite_msgs::msg::MapStatus::DATUM_MATCH);
        });
    // max_position_accuracy of tron.yaml holds until the logic tells its own (params.set).
    limits_sub_ = create_subscription<mowbite_msgs::msg::GpsLimits>(
        "logic/gps_limits", LatchedQos(), [this](mowbite_msgs::msg::GpsLimits::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          filter_.SetMaxPositionAccuracy(msg->max_position_accuracy);
        });
    release_srv_ = create_service<SetEmergency>(
        "safety/release_emergency",
        [this](const SetEmergency::Request::SharedPtr req, SetEmergency::Response::SharedPtr res) {
          OnRelease(*req, *res);
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { OnTimer(); });
  }

 private:
  SafetyConfig LoadConfig() {
    SafetyConfig config;
    config.tilt_limit = declare_parameter("tilt_limit", config.tilt_limit);
    config.tilt_time = declare_parameter("tilt_time", config.tilt_time);
    config.tilt_release = declare_parameter("tilt_release", config.tilt_release);
    config.fence_margin = declare_parameter("fence_margin", config.fence_margin);
    config.imu_timeout = declare_parameter("imu_timeout", config.imu_timeout);
    config.pose_timeout = declare_parameter("pose_timeout", config.pose_timeout);
    config.hw_timeout = declare_parameter("hw_status_timeout", config.hw_timeout);
    config.max_position_accuracy = declare_parameter("max_position_accuracy", config.max_position_accuracy);
    // Values that would let the filter pass everything or nothing keep it from starting; without it nothing moves.
    if (!(config.tilt_release > 0 && config.tilt_release < config.tilt_limit && config.tilt_limit < 90) ||
        !(config.tilt_time >= 0) || !(config.fence_margin >= 0) || !(config.imu_timeout > 0) ||
        !(config.pose_timeout > 0) || !(config.hw_timeout > 0)) {
      throw std::invalid_argument("mowbite_safety: tilt_release < tilt_limit < 90 and timeouts above 0 needed");
    }
    RCLCPP_INFO(get_logger(), "Tilt limit %.1f deg for %.1f s, release below %.1f deg; fence %.2f m beyond the areas",
                config.tilt_limit, config.tilt_time, config.tilt_release, config.fence_margin);
    return config;
  }

  void OnDrive(const DriveCommand& msg) {
    if (msg.source > DriveCommand::SOURCE_FOLLOWER) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Drive command of unknown source %u: stopped",
                            msg.source);
    }
    // An unknown source stops like the logic.
    const Source source = msg.source > DriveCommand::SOURCE_FOLLOWER ? Source::kStop : static_cast<Source>(msg.source);
    Decision decision;
    std::optional<LatchRequest> latch;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now = MonotonicSeconds();
      decision = filter_.OnCommand(source, msg.twist.linear.x, msg.twist.angular.z, now);
      latch = filter_.TakeLatchRequest(now);
    }
    if (decision.out) Publish(*decision.out);
    if (latch) SendLatch(*latch);
    if (decision.refusal.empty()) return;
    if (decision.conflict) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Refused the %s: %s (one source at a time)",
                            SourceName(source), decision.refusal.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Refused the %s: %s", SourceName(source),
                           decision.refusal.c_str());
    }
  }

  void OnImu(const sensor_msgs::msg::Imu& msg) {
    // Without an orientation in the message: the tilt from gravity.
    const double tilt =
        msg.orientation_covariance[0] >= 0
            ? TiltFromOrientation(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            : TiltFromGravity(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    std::optional<Twist2> stop;
    std::optional<LatchRequest> latch;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now = MonotonicSeconds();
      filter_.OnTilt(tilt, now);
      stop = filter_.Tick(now);
      latch = filter_.TakeLatchRequest(now);
    }
    if (stop) Publish(*stop);
    if (latch) SendLatch(*latch);
  }

  void OnRelease(const SetEmergency::Request& req, SetEmergency::Response& res) {
    if (req.latch) {
      res.success = false;
      res.message = "safety/release_emergency only releases";
      return;
    }
    if (!emergency_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "hw/emergency is not available, emergency stays");
      res.success = false;
      res.message = "hw/emergency is not available";
      return;
    }
    Release release;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      release = filter_.OnRelease(MonotonicSeconds());
    }
    if (!release.allowed) {
      RCLCPP_WARN(get_logger(), "Emergency stays: %s", release.message.c_str());
      res.success = false;
      res.message = release.message;
      return;
    }
    auto request = std::make_shared<SetEmergency::Request>();
    request->latch = false;
    request->reason = req.reason;
    emergency_client_->async_send_request(request);
    res.success = true;
    res.message = "release requested";
  }

  void OnTimer() {
    std::optional<Twist2> stop;
    std::optional<LatchRequest> latch;
    mowbite_msgs::msg::SafetyStatus status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now = MonotonicSeconds();
      stop = filter_.Tick(now);
      latch = filter_.TakeLatchRequest(now);
      status.latched = filter_.Latched();
      status.tilt_deg = static_cast<float>(filter_.TiltDeg(now));
      status.fence_distance = static_cast<float>(filter_.FenceDistance(now));
      status.autonomous_ok = filter_.AutonomousOk(now);
      status.owner = static_cast<uint8_t>(filter_.Owner(now));
      status.refused = filter_.Refused();
      status.last_refusal = filter_.LastRefusal();
    }
    if (stop) Publish(*stop);
    if (latch) SendLatch(*latch);
    status.stamp = now();
    status_pub_->publish(status);
  }

  void Publish(const Twist2& twist) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = twist.linear;
    msg.angular.z = twist.angular;
    cmd_pub_->publish(msg);
  }

  void SendLatch(const LatchRequest& latch) {
    if (latch.again) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "hw/status does not show %s yet, latching again",
                           BitNames(latch.bits).c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Emergency %s: %s", BitNames(latch.bits).c_str(), latch.reason.c_str());
    }
    // The filter blocks by its own latch meanwhile and asks again every second.
    if (!emergency_client_->service_is_ready()) return;
    auto request = std::make_shared<SetEmergency::Request>();
    request->latch = true;
    request->bits = latch.bits;
    request->reason = latch.reason;
    emergency_client_->async_send_request(request, [this](rclcpp::Client<SetEmergency>::SharedFuture future) {
      const auto response = future.get();
      if (!response->success) {
        RCLCPP_ERROR(get_logger(), "hw/emergency refused the latch: %s", response->message.c_str());
      }
    });
  }

  std::mutex mutex_;
  SafetyFilter filter_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Client<SetEmergency>::SharedPtr emergency_client_;
  rclcpp::Subscription<DriveCommand>::SharedPtr drive_sub_;
  rclcpp::Subscription<HardwareStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr caps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_in_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MapStatus>::SharedPtr map_status_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::GpsLimits>::SharedPtr limits_sub_;
  rclcpp::Service<SetEmergency>::SharedPtr release_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mowbite_safety

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_safety::SafetyNode)
