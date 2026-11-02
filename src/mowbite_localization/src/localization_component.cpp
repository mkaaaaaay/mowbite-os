#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <mowbite_msgs/msg/gnss_fix.hpp>
#include <mowbite_msgs/msg/gps_limits.hpp>
#include <mowbite_msgs/msg/position_jump.hpp>
#include <mowbite_msgs/msg/robot_pose.hpp>
#include <mowbite_msgs/srv/set_heading.hpp>
#include <mowbite_msgs/srv/set_pose.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

#include "mowbite_localization/estimator.h"

namespace mowbite_localization {

namespace {

using GnssMsg = mowbite_msgs::msg::GnssFix;
using PoseMsg = mowbite_msgs::msg::RobotPose;
static_assert(kSolutionRtkFixed == GnssMsg::RTK_FIXED);
static_assert(kSolutionRtkFloat == GnssMsg::RTK_FLOAT);
using mowbite_msgs::msg::PositionJump;
using GyroMsg = sensor_msgs::msg::Imu;
using OdometryMsg = nav_msgs::msg::Odometry;

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

geometry_msgs::msg::Quaternion YawQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

double YawOf(const geometry_msgs::msg::Quaternion& q) {
  const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
  const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(sin_yaw, cos_yaw);
}

}  // namespace

// The pose of base_link from gyro, wheel speed and RTK GNSS (Estimator), on localization/pose with every gyro sample,
// odom and TF. The antenna offset comes from the geometry of the logic on bridge/params, the GNSS limits from
// logic/gps_limits.
class LocalizationNode : public rclcpp::Node {
 public:
  explicit LocalizationNode(const rclcpp::NodeOptions& options)
      : Node("mowbite_localization", options), estimator_(ReadConfig()) {
    frame_id_ = declare_parameter("frame_id", "map");
    child_frame_id_ = declare_parameter("child_frame_id", "base_link");
    if (declare_parameter("publish_tf", true)) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pose_pub_ = create_publisher<PoseMsg>("localization/pose", 10);
    odometry_pub_ = create_publisher<OdometryMsg>("odom", 10);
    jump_pub_ = create_publisher<PositionJump>("localization/jump", 10);
    params_pub_ = create_publisher<std_msgs::msg::String>("bridge/params", LatchedQos(1));

    gyro_sub_ = create_subscription<GyroMsg>(
        "imu/data", rclcpp::SensorDataQoS(), [this](GyroMsg::ConstSharedPtr msg) { OnImu(*msg); });
    wheel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "hw/wheel_odom", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          estimator_.AddWheels(MonotonicSeconds(), msg->twist.linear.x, msg->twist.angular.z);
        });
    gps_sub_ = create_subscription<GnssMsg>(
        "gps/absolute_pose", 10, [this](GnssMsg::ConstSharedPtr msg) { OnGps(*msg); });
    limits_sub_ = create_subscription<mowbite_msgs::msg::GpsLimits>(
        "logic/gps_limits", LatchedQos(1), [this](mowbite_msgs::msg::GpsLimits::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          estimator_.SetGpsLimits(msg->max_position_accuracy, msg->gps_timeout);
          RCLCPP_INFO(get_logger(), "GPS limits: accuracy %.3f m, timeout %.1f s", msg->max_position_accuracy,
                      msg->gps_timeout);
        });
    // Several nodes leave their parameters latched here; the geometry of the logic is the one that counts.
    params_sub_ = create_subscription<std_msgs::msg::String>(
        "bridge/params", LatchedQos(50), [this](std_msgs::msg::String::ConstSharedPtr msg) { OnParams(msg->data); });

    set_pose_srv_ = create_service<mowbite_msgs::srv::SetPose>(
        "localization/set_pose", [this](const mowbite_msgs::srv::SetPose::Request::SharedPtr req,
                                        mowbite_msgs::srv::SetPose::Response::SharedPtr) { OnSetPose(*req); });
    set_heading_srv_ = create_service<mowbite_msgs::srv::SetHeading>(
        "localization/set_heading", [this](const mowbite_msgs::srv::SetHeading::Request::SharedPtr req,
                                           mowbite_msgs::srv::SetHeading::Response::SharedPtr res) {
          OnSetHeading(*req, *res);
        });
    set_gps_srv_ = create_service<std_srvs::srv::SetBool>(
        "localization/set_gps_state", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                             std_srvs::srv::SetBool::Response::SharedPtr res) {
          std::lock_guard<std::mutex> lock(mutex_);
          estimator_.SetGpsEnabled(req->data);
          res->success = true;
          res->message = req->data ? "GPS on" : "GPS off";
          RCLCPP_INFO(get_logger(), "GPS %s", req->data ? "on" : "off");
        });

    if (!estimator_.calibrated()) {
      RCLCPP_INFO(get_logger(), "Gyro calibration: keep the mower still for %.0f s",
                  get_parameter("gyro_calibration_time").as_double());
    }
  }

 private:
  EstimatorConfig ReadConfig() {
    EstimatorConfig config;
    config.gyro.calibration_time = declare_parameter("gyro_calibration_time", config.gyro.calibration_time);
    config.gyro.skip_calibration = declare_parameter("skip_gyro_calibration", config.gyro.skip_calibration);
    config.gyro.initial_offset = declare_parameter("gyro_offset", config.gyro.initial_offset);
    config.max_imu_gap = declare_parameter("max_imu_gap", config.max_imu_gap);
    config.wheel_timeout = declare_parameter("wheel_timeout", config.wheel_timeout);
    config.min_speed = declare_parameter("min_speed", config.min_speed);
    config.accuracy.growth_per_m = declare_parameter("accuracy_growth_per_m", config.accuracy.growth_per_m);
    config.accuracy.growth_per_s = declare_parameter("accuracy_growth_per_s", config.accuracy.growth_per_s);
    config.accuracy.timeout = declare_parameter("gps_timeout", config.accuracy.timeout);
    config.gate.max_accuracy = declare_parameter("max_position_accuracy", config.gate.max_accuracy);
    config.noise.odometry = declare_parameter("odometry_noise", config.noise.odometry);
    config.noise.gyro = declare_parameter("gyro_noise", config.noise.gyro);
    config.gnss_latency = declare_parameter("gnss_latency", config.gnss_latency);
    return config;
  }

  void OnImu(const GyroMsg& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = MonotonicSeconds();
    const bool was_calibrated = estimator_.calibrated();
    const bool moved = estimator_.AddImu(now, msg.angular_velocity.z);
    if (!was_calibrated && estimator_.calibrated()) {
      RCLCPP_INFO(get_logger(), "Gyro offset: %.5f rad/s", estimator_.gyro_offset());
    }
    if (moved) Publish(now);
  }

  void OnGps(const GnssMsg& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    GnssFix fix;
    fix.solution = msg.solution;
    fix.accuracy = msg.accuracy;
    fix.x = msg.x;
    fix.y = msg.y;
    fix.motion_valid = msg.velocity_valid;
    fix.motion_x = msg.velocity_east;
    fix.motion_y = msg.velocity_north;
    const FixResult result = estimator_.AddFix(fix, MonotonicSeconds());
    const GateConfig gate;
    if (result == FixResult::kRestarted && had_gps_) {
      RCLCPP_WARN(get_logger(), "GPS warmup again: waiting for %d fixes that agree", gate.warmup_fixes);
    } else if (result == FixResult::kOutlierReset) {
      RCLCPP_WARN(get_logger(), "GPS disagreed with the estimate %d times in a row, waiting for %d fixes that agree",
                  gate.outlier_reset, gate.warmup_fixes);
    } else if (result == FixResult::kContradictionReset) {
      RCLCPP_WARN(get_logger(), "GPS contradicted the estimate %d times in a row: position set onto GPS",
                  EstimatorConfig().contradiction_reset);
    } else if (result == FixResult::kInitialized) {
      had_gps_ = true;
      const Pose2 pose = estimator_.pose();
      RCLCPP_INFO(get_logger(), "Position set onto GPS: %.3f, %.3f", pose.x, pose.y);
    }
    if (estimator_.TakeAligned()) RCLCPP_INFO(get_logger(), "Heading aligned with the GPS course");
    if (auto jump = estimator_.TakeJump()) PublishJump(*jump);
  }

  void OnParams(const std::string& text) {
    nlohmann::json json = nlohmann::json::parse(text, nullptr, false);
    if (!json.is_object() || json.value("node", "") != "/mowbite_geometry" || !json.contains("params")) return;
    const nlohmann::json& params = json["params"];
    if (!params.is_object() || !params.contains("antenna_x") || !params.contains("antenna_y") ||
        !params["antenna_x"].is_number() || !params["antenna_y"].is_number()) {
      return;
    }
    const double x = params["antenna_x"].get<double>();
    const double y = params["antenna_y"].get<double>();
    bool known, changed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      known = estimator_.geometry_known();
      changed = !known || estimator_.antenna_offset() != Eigen::Vector2d(x, y);
      estimator_.SetAntennaOffset(x, y, MonotonicSeconds());
    }
    if (changed) {
      RCLCPP_INFO(get_logger(), "Antenna offset: %.3f, %.3f%s", x, y, known ? " (the geometry changed)" : "");
    }
    // Under the name of OpenMower's localization, which the app reads.
    nlohmann::json own = {{"node", "/xbot_positioning"},
                          {"params", {{"antenna_offset_x", x}, {"antenna_offset_y", y}}}};
    std_msgs::msg::String msg;
    msg.data = own.dump();
    params_pub_->publish(msg);
  }

  void OnSetPose(const mowbite_msgs::srv::SetPose::Request& req) {
    std::lock_guard<std::mutex> lock(mutex_);
    estimator_.SetPose(Pose2{req.pose.position.x, req.pose.position.y, YawOf(req.pose.orientation)});
    const Pose2 pose = estimator_.pose();
    RCLCPP_INFO(get_logger(), "Pose set to %.3f, %.3f, heading %.3f", pose.x, pose.y, pose.heading);
    Publish(MonotonicSeconds());
  }

  void OnSetHeading(const mowbite_msgs::srv::SetHeading::Request& req, mowbite_msgs::srv::SetHeading::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = MonotonicSeconds();
    const std::optional<Pose2> pose = estimator_.SetHeading(req.heading, now);
    if (!pose) {
      res.success = false;
      res.message = std::isfinite(req.heading) ? "no recent RTK fix taken by the localization"
                                               : "the heading is not a number";
      return;
    }
    res.success = true;
    res.x = pose->x;
    res.y = pose->y;
    res.position_accuracy = static_cast<float>(estimator_.PositionAccuracy(now));
    RCLCPP_INFO(get_logger(), "Heading set to %.3f, base_link at %.3f, %.3f", pose->heading, pose->x, pose->y);
    Publish(now);
  }

  void Publish(double now) {
    const Pose2 pose = estimator_.pose();
    const Covariance& p = estimator_.covariance();
    const rclcpp::Time stamp = this->now();

    PoseMsg msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.x = pose.x;
    msg.y = pose.y;
    msg.heading = WrapAngle(pose.heading);
    msg.heading_valid = estimator_.heading_valid();
    msg.position_accuracy = static_cast<float>(estimator_.PositionAccuracy(now));
    msg.heading_accuracy = static_cast<float>(estimator_.HeadingAccuracy());
    msg.gnss_recent = estimator_.RecentGps(now);
    msg.speed = estimator_.speed();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) msg.covariance[3 * r + c] = p(r, c);
    }
    pose_pub_->publish(msg);

    OdometryMsg odometry;
    odometry.header = msg.header;
    odometry.child_frame_id = child_frame_id_;
    odometry.pose.pose.position.x = pose.x;
    odometry.pose.pose.position.y = pose.y;
    odometry.pose.pose.orientation = YawQuaternion(pose.heading);
    // x, y and yaw of the 6 x 6 covariance (x, y, z, roll, pitch, yaw).
    const int index[3] = {0, 1, 5};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) odometry.pose.covariance[6 * index[r] + index[c]] = p(r, c);
    }
    odometry.twist.twist.linear.x = msg.speed;
    odometry.twist.twist.angular.z = estimator_.turn_rate();
    odometry_pub_->publish(odometry);

    if (tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = msg.header;
      tf.child_frame_id = child_frame_id_;
      tf.transform.translation.x = pose.x;
      tf.transform.translation.y = pose.y;
      tf.transform.rotation = odometry.pose.pose.orientation;
      tf_->sendTransform(tf);
    }
  }

  void PublishJump(const Jump& jump) {
    PositionJump msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.reason = jump.reason == WarmupReason::kOutliers ? PositionJump::REASON_OUTLIERS : PositionJump::REASON_GAP;
    msg.before.x = jump.from_x;
    msg.before.y = jump.from_y;
    msg.after.x = jump.to_x;
    msg.after.y = jump.to_y;
    msg.distance = jump.distance();
    jump_pub_->publish(msg);
    RCLCPP_WARN(get_logger(), "Position moved %.2f m onto GPS after %s", jump.distance(),
                jump.reason == WarmupReason::kOutliers ? "a run of outliers" : "a gap without fixes");
  }

  std::mutex mutex_;
  Estimator estimator_;
  bool had_gps_ = false;
  std::string frame_id_;
  std::string child_frame_id_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<PoseMsg>::SharedPtr pose_pub_;
  rclcpp::Publisher<OdometryMsg>::SharedPtr odometry_pub_;
  rclcpp::Publisher<PositionJump>::SharedPtr jump_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  rclcpp::Subscription<GyroMsg>::SharedPtr gyro_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_sub_;
  rclcpp::Subscription<GnssMsg>::SharedPtr gps_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::GpsLimits>::SharedPtr limits_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
  rclcpp::Service<mowbite_msgs::srv::SetPose>::SharedPtr set_pose_srv_;
  rclcpp::Service<mowbite_msgs::srv::SetHeading>::SharedPtr set_heading_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_gps_srv_;
};

}  // namespace mowbite_localization

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_localization::LocalizationNode)
