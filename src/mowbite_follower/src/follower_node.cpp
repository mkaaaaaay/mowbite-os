#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mowbite_msgs/action/follow_path.hpp>
#include <mowbite_msgs/msg/absolute_pose.hpp>
#include <mowbite_msgs/msg/gps_limits.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "mowbite_follower/guard.h"
#include "mowbite_follower/path_follower.h"

namespace mowbite_follower {

namespace {

using FollowPath = mowbite_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

double Yaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double Degrees(double radians) { return radians * 180.0 / M_PI; }
double Radians(double degrees) { return degrees * M_PI / 180.0; }

Ring ToRing(const geometry_msgs::msg::Polygon& polygon) {
  Ring ring;
  for (const auto& p : polygon.points) ring.push_back({p.x, p.y});
  return ring;
}

}  // namespace

// Follows the paths of the follow_path action on follower/cmd_vel; the logic passes them on to mowbite_safety. Stops by
// sending one zero and then nothing, the cmd_vel watchdog of the driver does the rest.
class FollowerNode : public rclcpp::Node {
 public:
  explicit FollowerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("mowbite_follower", options) {
    FollowerConfig& c = follower_config_;
    SpeedLimits& v = c.speed;
    v.speed_fast = declare_parameter("speed_fast", v.speed_fast);
    v.speed_slow = declare_parameter("speed_slow", v.speed_slow);
    v.speed_turn = declare_parameter("speed_turn", v.speed_turn);
    v.acceleration = declare_parameter("acceleration", v.acceleration);
    v.turn_rate = declare_parameter("turn_rate", v.turn_rate);
    v.max_linear = declare_parameter("max_linear", v.max_linear);
    v.creep_speed = declare_parameter("creep_speed", v.creep_speed);
    c.max_angular = declare_parameter("max_angular", c.max_angular);
    c.min_turn_rate = declare_parameter("min_turn_rate", c.min_turn_rate);
    c.max_start_distance = declare_parameter("max_start_distance", c.max_start_distance);
    c.goal_tolerance = declare_parameter("goal_tolerance", c.goal_tolerance);
    // Angles in degrees.
    c.corner_angle = Radians(declare_parameter("corner_angle", Degrees(c.corner_angle)));
    c.max_drive_angle = Radians(declare_parameter("max_drive_angle", Degrees(c.max_drive_angle)));
    c.steering.approach_distance = declare_parameter("approach_distance", c.steering.approach_distance);
    c.steering.approach_angle = Radians(declare_parameter("approach_angle", Degrees(c.steering.approach_angle)));
    c.steering.heading_gain = declare_parameter("heading_gain", c.steering.heading_gain);
    c.phase_timeout = declare_parameter("phase_timeout", c.phase_timeout);
    c.progress_timeout = declare_parameter("progress_timeout", c.progress_timeout);
    guard_config_.pose_timeout = declare_parameter("pose_timeout", guard_config_.pose_timeout);
    guard_config_.hw_timeout = declare_parameter("hw_status_timeout", guard_config_.hw_timeout);
    guard_config_.max_position_accuracy = declare_parameter("max_position_accuracy", guard_config_.max_position_accuracy);
    guard_config_.max_cross_track = declare_parameter("max_cross_track", guard_config_.max_cross_track);
    geofence_margin_ = declare_parameter("geofence_margin", 0.5);
    const double rate = declare_parameter("control_rate", 20.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("follower/cmd_vel", 10);
    pose_sub_ = create_subscription<mowbite_msgs::msg::AbsolutePose>(
        "localization/pose", 10, [this](mowbite_msgs::msg::AbsolutePose::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          pose_ = Pose2{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->vehicle_heading};
          position_accuracy_ = msg->position_accuracy;
          heading_valid_ = msg->orientation_valid != 0;
          pose_time_ = MonotonicSeconds();
        });
    hw_sub_ = create_subscription<mowbite_msgs::msg::HardwareStatus>(
        "hw/status", 10, [this](mowbite_msgs::msg::HardwareStatus::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          // Without telemetry the driver cannot stop the mower either.
          emergency_ = msg->emergency || !msg->telemetry_ok;
          hw_time_ = MonotonicSeconds();
        });
    caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](mowbite_msgs::msg::HardwareCapabilities::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          safety_measured_ = !msg->safety_measurement_date.empty();
          // The tighter of the driver's limits and our own.
          if (msg->max_linear_speed > 0) driver_max_linear_ = msg->max_linear_speed;
          if (msg->max_angular_speed > 0) driver_max_angular_ = msg->max_angular_speed;
        });
    // max_position_accuracy of tron.yaml holds until the logic tells its own (params.set).
    limits_sub_ = create_subscription<mowbite_msgs::msg::GpsLimits>(
        "logic/gps_limits", rclcpp::QoS(1).reliable().transient_local(),
        [this](mowbite_msgs::msg::GpsLimits::ConstSharedPtr msg) {
          if (!(msg->max_position_accuracy > 0) || !std::isfinite(msg->max_position_accuracy)) {
            RCLCPP_ERROR(get_logger(), "Ignored accuracy limit %g m from the logic", msg->max_position_accuracy);
            return;
          }
          {
            std::lock_guard<std::mutex> lock(mutex_);
            guard_config_.max_position_accuracy = msg->max_position_accuracy;
          }
          RCLCPP_INFO(get_logger(), "Accuracy limit %.3f m", msg->max_position_accuracy);
        });

    server_ = rclcpp_action::create_server<FollowPath>(
        this, "follow_path",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowPath::Goal>) {
          std::lock_guard<std::mutex> lock(mutex_);
          // One path at a time: the logic cancels the old one before it sends a new one.
          if (active_) {
            RCLCPP_WARN(get_logger(), "Rejecting a path while another one is followed");
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const std::shared_ptr<GoalHandle> handle) { OnAccepted(handle); });

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { OnTimer(); });
  }

 private:
  void OnAccepted(const std::shared_ptr<GoalHandle>& handle) {
    const auto goal = handle->get_goal();
    std::vector<Pose2> poses;
    for (const auto& p : goal->poses) poses.push_back({p.position.x, p.position.y, Yaw(p.orientation)});
    std::vector<Kind> kinds;
    std::string problem;
    for (uint8_t k : goal->kinds) {
      if (k > static_cast<uint8_t>(Kind::kTransit)) problem = "unknown kind " + std::to_string(k);
      kinds.push_back(static_cast<Kind>(k));
    }
    std::vector<Ring> allowed;
    std::vector<Ring> obstacles;
    for (const auto& p : goal->allowed_areas) allowed.push_back(ToRing(p));
    for (const auto& p : goal->obstacles) obstacles.push_back(ToRing(p));

    std::lock_guard<std::mutex> lock(mutex_);
    FollowerConfig config = follower_config_;
    SpeedLimits& v = config.speed;
    v.max_linear = std::min(v.max_linear, driver_max_linear_);
    if (goal->max_speed > 0) v.max_linear = std::min(v.max_linear, static_cast<double>(goal->max_speed));
    config.max_angular = std::min(config.max_angular, driver_max_angular_);
    // The path keeps its share of max_angular when the driver allows less.
    v.turn_rate *= config.max_angular / follower_config_.max_angular;
    config.stall_time = goal->stall_time > 0 ? goal->stall_time : 0.0;
    config.straight = goal->straight;
    config.reverse = goal->reverse;
    config.end_tolerance = goal->end_tolerance > 0 ? goal->end_tolerance : 0.0;
    follower_ = std::make_unique<PathFollower>(config);
    fence_ = Geofence(std::move(allowed), std::move(obstacles), geofence_margin_);
    if (problem.empty()) problem = fence_.Problem();
    if (problem.empty()) problem = follower_->Start(poses, kinds, goal->start_index);
    active_ = handle;
    last_step_ = MonotonicSeconds();
    last_feedback_ = 0;
    if (!problem.empty()) {
      Finish(FollowPath::Result::REASON_INVALID, "invalid path: " + problem, goal->start_index);
      return;
    }
    RCLCPP_INFO(get_logger(), "Following %zu poses from %u%s%s, at most %.2f m/s", poses.size(), goal->start_index,
                goal->reverse ? " backwards" : "", goal->straight ? " straight without steering" : "",
                config.speed.max_linear);
  }

  void OnTimer() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) return;
    const double now = MonotonicSeconds();
    const double dt = now - last_step_;
    last_step_ = now;
    if (active_->is_canceling()) {
      Finish(FollowPath::Result::REASON_CANCELED, "canceled", follower_->index());
      return;
    }
    GuardInput in;
    in.safety_measured = safety_measured_;
    in.hw_age = hw_time_ ? now - *hw_time_ : 1e9;
    in.emergency = emergency_;
    in.pose_age = pose_time_ ? now - *pose_time_ : 1e9;
    in.pose = pose_;
    in.position_accuracy = position_accuracy_;
    in.heading_valid = heading_valid_;
    // Off the path counts only while on it; before the start the follower checks the distance itself.
    in.cross_track = follower_->OnPath() ? follower_->CrossTrack(pose_) : 0.0;
    const GuardResult guard = CheckGuard(guard_config_, in, fence_);
    if (guard.reason != StopReason::kNone) {
      Finish(static_cast<uint8_t>(guard.reason), guard.message, follower_->index());
      return;
    }
    const PathFollower::Command cmd = follower_->Step(pose_, dt);
    if (follower_->phase() == Phase::kFailed) {
      const bool start = follower_->failure_kind() == Failure::kStart;
      Finish(start ? FollowPath::Result::REASON_INVALID : FollowPath::Result::REASON_TIMEOUT, follower_->failure(),
             follower_->index());
      return;
    }
    if (follower_->phase() == Phase::kDone) {
      Finish(FollowPath::Result::REASON_DONE, "done", follower_->index());
      return;
    }
    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd.linear;
    twist.angular.z = cmd.angular;
    cmd_pub_->publish(twist);
    if (now - last_feedback_ >= 0.5) {
      last_feedback_ = now;
      auto feedback = std::make_shared<FollowPath::Feedback>();
      feedback->index = static_cast<uint32_t>(follower_->index());
      feedback->cross_track = static_cast<float>(follower_->CrossTrack(pose_));
      active_->publish_feedback(feedback);
    }
  }

  // Ends the active goal with one zero command. Called with the mutex held.
  void Finish(uint8_t reason, const std::string& message, size_t index) {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    follower_->Cancel();
    auto result = std::make_shared<FollowPath::Result>();
    result->reason = reason;
    result->message = message;
    result->index = static_cast<uint32_t>(index);
    if (reason == FollowPath::Result::REASON_DONE) {
      RCLCPP_INFO(get_logger(), "Path done at pose %zu", index);
      active_->succeed(result);
    } else if (reason == FollowPath::Result::REASON_CANCELED) {
      RCLCPP_INFO(get_logger(), "Path canceled at pose %zu", index);
      active_->canceled(result);
    } else {
      RCLCPP_WARN(get_logger(), "Stopped at pose %zu: %s", index, message.c_str());
      active_->abort(result);
    }
    active_.reset();
  }

  std::mutex mutex_;
  FollowerConfig follower_config_;
  GuardConfig guard_config_;
  double geofence_margin_ = 0.5;
  std::unique_ptr<PathFollower> follower_;
  Geofence fence_;
  std::shared_ptr<GoalHandle> active_;
  double last_step_ = 0;
  double last_feedback_ = 0;

  Pose2 pose_;
  float position_accuracy_ = 999;
  bool heading_valid_ = false;
  std::optional<double> pose_time_;
  bool emergency_ = true;
  std::optional<double> hw_time_;
  bool safety_measured_ = false;
  double driver_max_linear_ = 1e9;
  double driver_max_angular_ = 1e9;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::AbsolutePose>::SharedPtr pose_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr caps_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::GpsLimits>::SharedPtr limits_sub_;
  rclcpp_action::Server<FollowPath>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mowbite_follower

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_follower::FollowerNode)
