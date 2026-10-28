// ROS 2 node of the localization. Based on xbot_positioning.cpp (OpenMower), MIT License, Copyright (c) 2022 Clemens
// Elflein. See LICENSE.xbot_positioning.
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <mowbite_msgs/msg/absolute_pose.hpp>
#include <mowbite_msgs/srv/set_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "mowbite_localization/localization.h"

namespace mowbite_localization {

namespace {

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

// Fuses imu/data, wheel_twist and gps/absolute_pose. Publishes odom, localization/pose and the map -> base_link
// transform on every gyro sample.
class LocalizationNode : public rclcpp::Node {
 public:
  explicit LocalizationNode(const rclcpp::NodeOptions& options) : rclcpp::Node("mowbite_localization", options) {
    LocalizationConfig config;
    config.gyro_calibration_s = declare_parameter("gyro_calibration_time", config.gyro_calibration_s);
    config.skip_gyro_calibration = declare_parameter("skip_gyro_calibration", config.skip_gyro_calibration);
    config.gyro_offset = declare_parameter("gyro_offset", config.gyro_offset);
    config.min_speed = declare_parameter("min_speed", config.min_speed);
    config.max_gps_accuracy = declare_parameter("max_gps_accuracy", config.max_gps_accuracy);
    config.antenna_offset_x = declare_parameter("antenna_offset_x", config.antenna_offset_x);
    config.antenna_offset_y = declare_parameter("antenna_offset_y", config.antenna_offset_y);
    config.process_noise = declare_parameter("process_noise", config.process_noise);
    config.wheel_timeout_s = declare_parameter("wheel_timeout", config.wheel_timeout_s);
    config.max_imu_gap_s = declare_parameter("max_imu_gap", config.max_imu_gap_s);
    frame_id_ = declare_parameter("frame_id", "map");
    child_frame_id_ = declare_parameter("child_frame_id", "base_link");
    const bool publish_tf = declare_parameter("publish_tf", true);
    debug_ = declare_parameter("debug", false);
    max_gps_accuracy_ = config.max_gps_accuracy;
    localization_ = std::make_unique<Localization>(config);

    RCLCPP_INFO(get_logger(), "Antenna offset: %.3f, %.3f", config.antenna_offset_x, config.antenna_offset_y);
    if (config.skip_gyro_calibration) {
      RCLCPP_WARN(get_logger(), "Skipping the gyro calibration, offset %.5f rad/s", config.gyro_offset);
    } else {
      RCLCPP_INFO(get_logger(), "Gyro calibration: keep the mower still for %.0f s", config.gyro_calibration_s);
    }

    // The app shows the antenna offset from params/json, the bridge collects it from this latched topic.
    params_pub_ = create_publisher<std_msgs::msg::String>(
        "bridge/params", rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local());
    char text[160];
    std::snprintf(text, sizeof(text),
                  "{\"node\":\"/xbot_positioning\",\"params\":{\"antenna_offset_x\":%.15g,\"antenna_offset_y\":%.15g}}",
                  config.antenna_offset_x, config.antenna_offset_y);
    std_msgs::msg::String params_msg;
    params_msg.data = text;
    params_pub_->publish(params_msg);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pose_pub_ = create_publisher<mowbite_msgs::msg::AbsolutePose>("localization/pose", 10);
    if (publish_tf) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    if (debug_) {
      motion_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("localization/debug/expected_motion", 10);
    }

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS(),
                                                          [this](const sensor_msgs::msg::Imu& msg) { OnImu(msg); });
    wheel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "wheel_twist", 10, [this](const geometry_msgs::msg::TwistStamped& msg) {
          localization_->AddWheelSpeed(msg.twist.linear.x, MonotonicSeconds());
        });
    gps_sub_ = create_subscription<mowbite_msgs::msg::AbsolutePose>(
        "gps/absolute_pose", 10, [this](const mowbite_msgs::msg::AbsolutePose& msg) { OnGps(msg); });

    gps_srv_ = create_service<std_srvs::srv::SetBool>(
        "localization/set_gps_state", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          localization_->SetGpsEnabled(request->data);
          RCLCPP_INFO(get_logger(), "GPS %s", request->data ? "enabled" : "disabled");
          response->success = true;
        });
    pose_srv_ = create_service<mowbite_msgs::srv::SetPose>(
        "localization/set_pose", [this](const std::shared_ptr<mowbite_msgs::srv::SetPose::Request> request,
                                        std::shared_ptr<mowbite_msgs::srv::SetPose::Response>) {
          const auto& p = request->pose;
          const double yaw = YawFromQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
          localization_->SetPose(p.position.x, p.position.y, yaw);
          RCLCPP_INFO(get_logger(), "Pose set to %.2f, %.2f, %.3f rad", p.position.x, p.position.y, yaw);
        });
  }

 private:
  void OnImu(const sensor_msgs::msg::Imu& msg) {
    const bool was_calibrated = localization_->calibrated();
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const bool advanced = localization_->AddImu(stamp, msg.angular_velocity.z, MonotonicSeconds());
    if (!was_calibrated && localization_->calibrated()) {
      RCLCPP_INFO(get_logger(), "Gyro offset: %.5f rad/s", localization_->gyro_offset());
    }
    if (advanced) Publish(msg.header.stamp);
  }

  void OnGps(const mowbite_msgs::msg::AbsolutePose& msg) {
    GpsFix fix;
    fix.flags = msg.flags;
    fix.accuracy = msg.position_accuracy;
    fix.x = msg.pose.pose.position.x;
    fix.y = msg.pose.pose.position.y;
    fix.z = msg.pose.pose.position.z;
    fix.motion_valid = msg.motion_vector_valid != 0;
    fix.motion_x = msg.motion_vector.x;
    fix.motion_y = msg.motion_vector.y;
    const auto result = localization_->AddGps(fix, MonotonicSeconds());
    switch (result) {
      case GpsResult::kNotFixed:
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "Dropped GPS update, not RTK fixed");
        break;
      case GpsResult::kInaccurate:
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "Dropped GPS update, accuracy %.3f m above %.3f m",
                             fix.accuracy, max_gps_accuracy_);
        break;
      case GpsResult::kRestarted:
        if (had_fix_) RCLCPP_WARN(get_logger(), "No usable GPS fix for more than 5 s, waiting for 10 inliers again");
        had_fix_ = true;
        break;
      case GpsResult::kInitialized:
        RCLCPP_INFO(get_logger(), "GPS valid, moved the filter to %.2f, %.2f", fix.x, fix.y);
        break;
      case GpsResult::kOutlier:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "GPS outlier at %.2f, %.2f", fix.x, fix.y);
        break;
      case GpsResult::kOutlierReset:
        RCLCPP_ERROR(get_logger(), "Too many GPS outliers, taking the current fix as valid");
        break;
      case GpsResult::kDisabled:
      case GpsResult::kWarmup:
      case GpsResult::kFused: break;
    }
    if (debug_ && result == GpsResult::kFused) {
      const auto m = localization_->ExpectedMotion();
      geometry_msgs::msg::Vector3Stamped dbg;
      dbg.header.stamp = msg.header.stamp;
      dbg.header.frame_id = frame_id_;
      dbg.vector.x = m.x();
      dbg.vector.y = m.y();
      motion_pub_->publish(dbg);
    }
  }

  void Publish(const builtin_interfaces::msg::Time& stamp) {
    const Estimate e = localization_->GetEstimate();
    const double now_s = MonotonicSeconds();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = e.x;
    odom.pose.pose.position.y = e.y;
    odom.pose.pose.orientation.z = std::sin(e.theta / 2.0);
    odom.pose.pose.orientation.w = std::cos(e.theta / 2.0);
    // 6x6 over x, y, z, roll, pitch, yaw; only the planar part is estimated.
    constexpr size_t kIndex[3] = {0, 1, 5};
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 3; ++j) odom.pose.covariance[kIndex[i] * 6 + kIndex[j]] = e.pose_covariance[i * 3 + j];
    }
    odom.twist.twist.linear.x = e.vx;
    odom.twist.twist.angular.z = e.vr;
    odom.twist.covariance[0] = e.vx_variance;
    odom.twist.covariance[35] = e.vr_variance;
    odom_pub_->publish(odom);

    if (tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header = odom.header;
      t.child_frame_id = child_frame_id_;
      t.transform.translation.x = e.x;
      t.transform.translation.y = e.y;
      t.transform.rotation = odom.pose.pose.orientation;
      tf_->sendTransform(t);
    }

    mowbite_msgs::msg::AbsolutePose pose;
    pose.header = odom.header;
    pose.source = mowbite_msgs::msg::AbsolutePose::SOURCE_SENSOR_FUSION;
    pose.flags = mowbite_msgs::msg::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;
    if (localization_->RecentGps(now_s)) {
      pose.flags |= mowbite_msgs::msg::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;
    }
    pose.orientation_valid = 1;
    pose.motion_vector_valid = 0;
    pose.position_accuracy = static_cast<float>(localization_->PositionAccuracy(now_s));
    // Fixed value as in the original until the filter variance is calibrated against the device.
    pose.orientation_accuracy = 0.01f;
    pose.pose = odom.pose;
    pose.vehicle_heading = e.theta;
    pose.motion_heading = e.theta;
    pose_pub_->publish(pose);
  }

  std::unique_ptr<Localization> localization_;
  std::string frame_id_;
  std::string child_frame_id_;
  double max_gps_accuracy_ = 0.0;
  bool debug_ = false;
  bool had_fix_ = false;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::AbsolutePose>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr motion_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::AbsolutePose>::SharedPtr gps_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gps_srv_;
  rclcpp::Service<mowbite_msgs::srv::SetPose>::SharedPtr pose_srv_;
};

}  // namespace mowbite_localization

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_localization::LocalizationNode)
