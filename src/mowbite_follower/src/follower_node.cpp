#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mowbite_component/node_options.h>
#include <mowbite_msgs/action/follow_path.hpp>
#include <mowbite_msgs/msg/robot_pose.hpp>
#include <mowbite_msgs/msg/gps_limits.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include "mowbite_follower/guard.h"
#include "mowbite_follower/path_follower.h"
#include "mowbite_watch/attach.h"

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

// Follows the paths of the follow_path action on follower/cmd_vel by localization/pose, or in the station frame on
// follower/dock_cmd_vel by localization/dock_pose; the logic passes them on to mowbite_safety. Stops by sending one
// zero and then nothing, the cmd_vel watchdog of the driver does the rest.
class FollowerNode : public rclcpp::Node {
 public:
  explicit FollowerNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_follower", mowbite_component::StartupOnly(options)) {
    FollowerConfig& c = follower_config_;
    SpeedLimits& v = c.speed;
    v.speed_fast = declare_parameter("speed_fast", v.speed_fast);
    v.speed_slow = declare_parameter("speed_slow", v.speed_slow);
    v.speed_turn = declare_parameter("speed_turn", v.speed_turn);
    v.acceleration = declare_parameter("acceleration", v.acceleration);
    v.deceleration = declare_parameter("deceleration", v.acceleration);
    v.turn_rate = declare_parameter("turn_rate", v.turn_rate);
    v.max_linear = declare_parameter("max_linear", v.max_linear);
    v.creep_speed = declare_parameter("creep_speed", v.creep_speed);
    c.max_angular = declare_parameter("max_angular", c.max_angular);
    c.min_turn_rate = declare_parameter("min_turn_rate", c.min_turn_rate);
    c.spot_max_angular = declare_parameter("spot_max_angular", c.spot_max_angular);
    c.turn_deceleration = declare_parameter("turn_deceleration", c.turn_deceleration);
    c.max_start_distance = declare_parameter("max_start_distance", c.max_start_distance);
    c.goal_tolerance = declare_parameter("goal_tolerance", c.goal_tolerance);
    // Angles in degrees.
    c.corner_angle = Radians(declare_parameter("corner_angle", Degrees(c.corner_angle)));
    c.max_drive_angle = Radians(declare_parameter("max_drive_angle", Degrees(c.max_drive_angle)));
    c.min_arc_radius = declare_parameter("min_arc_radius", c.min_arc_radius);
    c.steering.approach_distance = declare_parameter("approach_distance", c.steering.approach_distance);
    c.steering.approach_angle = Radians(declare_parameter("approach_angle", Degrees(c.steering.approach_angle)));
    c.steering.heading_gain = declare_parameter("heading_gain", c.steering.heading_gain);
    c.phase_timeout = declare_parameter("phase_timeout", c.phase_timeout);
    c.progress_timeout = declare_parameter("progress_timeout", c.progress_timeout);
    guard_config_.pose_timeout = declare_parameter("pose_timeout", guard_config_.pose_timeout);
    guard_config_.hw_timeout = declare_parameter("hw_status_timeout", guard_config_.hw_timeout);
    guard_config_.max_position_accuracy = declare_parameter("max_position_accuracy", guard_config_.max_position_accuracy);
    dock_max_position_accuracy_ = declare_parameter("dock_max_position_accuracy", 0.03);
    dock_accuracy_per_m_ = declare_parameter("dock_accuracy_per_m", 0.09);
    if (!(dock_max_position_accuracy_ > 0) || !(dock_accuracy_per_m_ >= 0) || !(v.acceleration > 0) ||
        !(v.deceleration > 0) || !(c.spot_max_angular >= 0) || !(c.turn_deceleration >= 0)) {
      throw std::invalid_argument(
          "mowbite_follower: dock_max_position_accuracy, acceleration and deceleration above 0, dock_accuracy_per_m, "
          "spot_max_angular and turn_deceleration from 0 needed");
    }
    guard_config_.max_cross_track = declare_parameter("max_cross_track", guard_config_.max_cross_track);
    geofence_margin_ = declare_parameter("geofence_margin", 0.5);
    const double rate = declare_parameter("control_rate", 20.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("follower/cmd_vel", 10);
    dock_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("follower/dock_cmd_vel", 10);
    // The speeds the planner plans with, latched: whoever starts later gets the last. Sent once the driver's limits
    // are known, so the planner never sees the follower's own ones above them.
    params_pub_ = create_publisher<std_msgs::msg::String>(
        "bridge/params", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    pose_sub_ = create_subscription<mowbite_msgs::msg::RobotPose>(
        "localization/pose", 10, [this](const mowbite_msgs::msg::RobotPose::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          map_pose_.Set(*msg, MonotonicSeconds());
        });
    dock_pose_sub_ = create_subscription<mowbite_msgs::msg::RobotPose>(
        "localization/dock_pose", 10, [this](const mowbite_msgs::msg::RobotPose::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          dock_pose_.Set(*msg, MonotonicSeconds());
        });
    hw_sub_ = create_subscription<mowbite_msgs::msg::HardwareStatus>(
        "hw/status", 10, [this](const mowbite_msgs::msg::HardwareStatus::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          // Without telemetry the driver cannot stop the mower either.
          emergency_ = msg->emergency || !msg->telemetry_ok;
          hw_time_ = MonotonicSeconds();
        });
    caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](const mowbite_msgs::msg::HardwareCapabilities::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          safety_measured_ = !msg->safety_measurement_date.empty();
          // A limit of 0 (max_duty 0) lets nothing move: the guard stops every path, the planner gets no speeds.
          driver_moves_ = msg->max_linear_speed > 0 && msg->max_angular_speed > 0;
          // The tighter of the driver's limits and our own.
          if (msg->max_linear_speed > 0) driver_max_linear_ = msg->max_linear_speed;
          if (msg->max_angular_speed > 0) driver_max_angular_ = msg->max_angular_speed;
          if (driver_moves_) {
            PublishParams();
          } else {
            RCLCPP_WARN(get_logger(), "The driver allows no speed (%.3f m/s, %.3f rad/s): no paths, no planner speeds",
                        msg->max_linear_speed, msg->max_angular_speed);
          }
        });
    // max_position_accuracy of tron.yaml holds until the logic tells its own (params.set).
    limits_sub_ = create_subscription<mowbite_msgs::msg::GpsLimits>(
        "logic/gps_limits", rclcpp::QoS(1).reliable().transient_local(),
        [this](const mowbite_msgs::msg::GpsLimits::ConstSharedPtr& msg) {
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
        [this](const rclcpp_action::GoalUUID&, const std::shared_ptr<const FollowPath::Goal>&) {
          std::lock_guard<std::mutex> lock(mutex_);
          // One path at a time: the logic cancels the old one before it sends a new one.
          if (active_) {
            RCLCPP_WARN(get_logger(), "Rejecting a path while another one is followed");
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const std::shared_ptr<GoalHandle>& handle) { OnAccepted(handle); });

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { OnTimer(); });
    watch_ = mowbite_watch::Attach(*this);
  }

 private:
  // Publishes the effective speeds when they differ from the last ones sent.
  void PublishParams() {
    std_msgs::msg::String msg;
    msg.data = ParamsJson(get_fully_qualified_name(),
                          Effective(follower_config_, driver_max_linear_, driver_max_angular_));
    if (msg.data == last_params_) return;
    last_params_ = msg.data;
    params_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Planner speeds %s", msg.data.c_str());
  }

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
    // 0 or a limit: a value that is no number must not switch the check off.
    if (!(goal->max_cross_track >= 0) || !std::isfinite(goal->max_cross_track)) {
      problem = "max_cross_track must be 0 or more";
    }
    if (goal->frame > FollowPath::Goal::FRAME_DOCK) problem = "unknown frame " + std::to_string(goal->frame);
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
    // Faster turns on the spot only in the map: in the station frame the marker has to stay in view.
    config.spot_max_angular = goal->frame == FollowPath::Goal::FRAME_DOCK || config.spot_max_angular <= 0
                                  ? config.max_angular
                                  : std::min(config.spot_max_angular, driver_max_angular_);
    config.stall_time = goal->stall_time > 0 ? goal->stall_time : 0.0;
    config.straight = goal->straight;
    config.reverse = goal->reverse;
    config.end_tolerance = goal->end_tolerance > 0 ? goal->end_tolerance : 0.0;
    max_off_path_ = goal->max_cross_track;
    dock_frame_ = goal->frame == FollowPath::Goal::FRAME_DOCK;
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
    RCLCPP_INFO(get_logger(), "Following %zu poses from %u%s%s%s, at most %.2f m/s", poses.size(), goal->start_index,
                dock_frame_ ? " in the station frame" : "", goal->reverse ? " backwards" : "",
                goal->straight ? " straight without steering" : "", config.speed.max_linear);
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
    // The pose and accuracy limit of the frame of the path; guard and controller are the same in both.
    const PoseInput& pose = dock_frame_ ? dock_pose_ : map_pose_;
    GuardConfig guard_config = guard_config_;
    if (dock_frame_) {
      guard_config.max_position_accuracy =
          dock_max_position_accuracy_ + dock_accuracy_per_m_ * std::hypot(pose.pose.x, pose.pose.y);
    }
    GuardInput in;
    in.safety_measured = safety_measured_;
    in.driver_moves = driver_moves_;
    in.hw_age = hw_time_ ? now - *hw_time_ : 1e9;
    in.emergency = emergency_;
    in.pose_age = pose.time ? now - *pose.time : 1e9;
    in.pose = pose.pose;
    in.position_accuracy = pose.accuracy;
    in.heading_valid = pose.heading_valid;
    // Off the path counts only while on it; before the start the follower checks the distance itself.
    in.cross_track = follower_->OnPath() ? follower_->CrossTrack(pose.pose) : 0.0;
    // The limit of the goal counts from the first command and around the whole path.
    in.off_path = max_off_path_ > 0 ? follower_->OffPath(pose.pose) : 0.0;
    in.max_off_path = max_off_path_;
    const GuardResult guard = CheckGuard(guard_config, in, fence_);
    if (guard.reason != StopReason::kNone) {
      Finish(static_cast<uint8_t>(guard.reason), guard.message, follower_->index());
      return;
    }
    const PathFollower::Command cmd = follower_->Step(pose.pose, dt);
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
    CmdPublisher().publish(twist);
    if (now - last_feedback_ >= 0.5) {
      last_feedback_ = now;
      auto feedback = std::make_shared<FollowPath::Feedback>();
      feedback->index = static_cast<uint32_t>(follower_->index());
      feedback->cross_track = static_cast<float>(follower_->CrossTrack(pose.pose));
      active_->publish_feedback(feedback);
    }
  }

  // The topic of the frame of the active path. Called with the mutex held.
  rclcpp::Publisher<geometry_msgs::msg::Twist>& CmdPublisher() { return dock_frame_ ? *dock_cmd_pub_ : *cmd_pub_; }

  // Ends the active goal with one zero command. Called with the mutex held.
  void Finish(uint8_t reason, const std::string& message, size_t index) {
    CmdPublisher().publish(geometry_msgs::msg::Twist());
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

  // The last pose of one frame.
  struct PoseInput {
    Pose2 pose;
    float accuracy = 999;
    bool heading_valid = false;
    std::optional<double> time;

    void Set(const mowbite_msgs::msg::RobotPose& msg, double now) {
      pose = Pose2{msg.x, msg.y, msg.heading};
      accuracy = msg.position_accuracy;
      heading_valid = msg.heading_valid;
      time = now;
    }
  };

  std::mutex mutex_;
  FollowerConfig follower_config_;
  GuardConfig guard_config_;
  // m localization/dock_pose has to be accurate to at least on a path in the station frame: at the docked place, and
  // worse by that per m away from it (the frame turns with the error in the direction of the marker).
  double dock_max_position_accuracy_ = 0.03;
  double dock_accuracy_per_m_ = 0.09;
  double geofence_margin_ = 0.5;
  // max_cross_track of the active goal, 0 for none.
  double max_off_path_ = 0;
  // The active goal is in the station frame.
  bool dock_frame_ = false;
  std::unique_ptr<PathFollower> follower_;
  Geofence fence_;
  std::shared_ptr<GoalHandle> active_;
  double last_step_ = 0;
  double last_feedback_ = 0;

  PoseInput map_pose_;
  PoseInput dock_pose_;
  bool emergency_ = true;
  std::optional<double> hw_time_;
  bool safety_measured_ = false;
  bool driver_moves_ = false;
  double driver_max_linear_ = 1e9;
  double driver_max_angular_ = 1e9;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dock_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  std::string last_params_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotPose>::SharedPtr dock_pose_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr caps_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::GpsLimits>::SharedPtr limits_sub_;
  rclcpp_action::Server<FollowPath>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_follower

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_follower::FollowerNode)
