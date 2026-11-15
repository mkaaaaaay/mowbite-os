#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <mowbite_component/node_options.h>
#include <mowbite_msgs/msg/dock_marker.hpp>
#include <mowbite_msgs/msg/gnss_fix.hpp>
#include <mowbite_msgs/msg/gps_limits.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/position_jump.hpp>
#include <mowbite_msgs/msg/robot_pose.hpp>
#include <mowbite_msgs/srv/set_heading.hpp>
#include <mowbite_msgs/srv/set_pose.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <mowbite_journal/node_options.h>
#include <tf2_ros/transform_broadcaster.h>

#include "mowbite_localization/dock_estimator_journal.h"
#include "mowbite_localization/estimator.h"
#include "mowbite_localization/estimator_journal.h"
#include "mowbite_watch/attach.h"

namespace mowbite_localization {

namespace {

using GnssMsg = mowbite_msgs::msg::GnssFix;
using PoseMsg = mowbite_msgs::msg::RobotPose;
static_assert(kSolutionRtkFixed == GnssMsg::RTK_FIXED);
static_assert(kSolutionRtkFloat == GnssMsg::RTK_FLOAT);
using mowbite_msgs::msg::PositionJump;
using GyroMsg = sensor_msgs::msg::Imu;
using OdometryMsg = nav_msgs::msg::Odometry;
using DockMarkerMsg = mowbite_msgs::msg::DockMarker;

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
// logic/gps_limits. On a platform with a dock marker (marker_range above 0) also the pose in the station frame
// (DockEstimator) from hw/dock_marker on localization/dock_pose, on the same motions, with every gyro sample while
// there is an estimate; logic/dock_heading_hint gives its start heading where the marker has no direction yet.
class LocalizationNode : public rclcpp::Node {
 public:
  explicit LocalizationNode(const rclcpp::NodeOptions& options)
      : Node("mowbite_localization", mowbite_component::StartupOnly(options)),
        journal_(mowbite_journal::OpenFromParameters(*this)),
        estimator_(journal_, ReadConfig()),
        journal_watch_(journal_) {
    journal_timer_ = create_wall_timer(std::chrono::seconds(10), [this] { journal_watch_.Check(get_logger()); });
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
          estimator_.Call<&Estimator::AddWheels>(MonotonicSeconds(), msg->twist.linear.x, msg->twist.angular.z);
        });
    gps_sub_ = create_subscription<GnssMsg>(
        "gps/absolute_pose", 10, [this](GnssMsg::ConstSharedPtr msg) { OnGps(*msg); });
    limits_sub_ = create_subscription<mowbite_msgs::msg::GpsLimits>(
        "logic/gps_limits", LatchedQos(1), [this](mowbite_msgs::msg::GpsLimits::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          estimator_.Call<&Estimator::SetGpsLimits>(msg->max_position_accuracy, msg->gps_timeout);
          RCLCPP_INFO(get_logger(), "GPS limits: accuracy %.3f m, timeout %.1f s", msg->max_position_accuracy,
                      msg->gps_timeout);
        });
    // The ticks per m the driver computes the wheel speeds with; robot.geometry holds the true ones.
    caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", LatchedQos(1), [this](mowbite_msgs::msg::HardwareCapabilities::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          driver_ticks_per_m_ = msg->wheel_ticks_per_m;
          ApplyWheelScale();
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
          estimator_.Call<&Estimator::SetGpsEnabled>(req->data);
          res->success = true;
          res->message = req->data ? "GPS on" : "GPS off";
          RCLCPP_INFO(get_logger(), "GPS %s", req->data ? "on" : "off");
        });

    for (const auto& warning : estimator_.Call<&Estimator::TakeGyroWarnings>()) {
      RCLCPP_WARN(get_logger(), "%s", warning.c_str());
    }
    if (!estimator_.View().calibrated()) {
      RCLCPP_INFO(get_logger(), "Gyro calibration: keep the mower still for %.0f s",
                  get_parameter("gyro_calibration_time").as_double());
    }
    if (declare_parameter("marker_range", 0.0) > 0.0) {
      StartDock();
    } else {
      RCLCPP_INFO(get_logger(), "No dock marker on this platform (marker_range 0): no localization/dock_pose");
    }
    watch_ = mowbite_watch::Attach(*this);
  }

 private:
  EstimatorConfig ReadConfig() {
    EstimatorConfig config;
    config.gyro.calibration_time = declare_parameter("gyro_calibration_time", config.gyro.calibration_time);
    config.gyro.skip_calibration = declare_parameter("skip_gyro_calibration", config.gyro.skip_calibration);
    config.gyro.initial_offset = declare_parameter("gyro_offset", config.gyro.initial_offset);
    config.gyro.still_spread = declare_parameter("gyro_still_spread", config.gyro.still_spread);
    config.gyro.offset_limit = declare_parameter("gyro_offset_limit", config.gyro.offset_limit);
    if (!(config.gyro.still_spread > 0) || !(config.gyro.offset_limit >= 0) ||
        !std::isfinite(config.gyro.offset_limit) || !std::isfinite(config.gyro.initial_offset)) {
      throw std::invalid_argument(
          "gyro_still_spread must be above 0, gyro_offset_limit a number of 0 or more and gyro_offset a number");
    }
    config.max_imu_gap = declare_parameter("max_imu_gap", config.max_imu_gap);
    config.wheel_timeout = declare_parameter("wheel_timeout", config.wheel_timeout);
    config.min_speed = declare_parameter("min_speed", config.min_speed);
    config.motion_confirm_time = declare_parameter("motion_confirm_time", config.motion_confirm_time);
    config.accuracy.growth_per_m = declare_parameter("accuracy_growth_per_m", config.accuracy.growth_per_m);
    config.accuracy.growth_per_s = declare_parameter("accuracy_growth_per_s", config.accuracy.growth_per_s);
    config.accuracy.timeout = declare_parameter("gps_timeout", config.accuracy.timeout);
    config.gate.max_accuracy = declare_parameter("max_position_accuracy", config.gate.max_accuracy);
    config.noise.odometry = declare_parameter("odometry_noise", config.noise.odometry);
    config.noise.gyro = declare_parameter("gyro_noise", config.noise.gyro);
    config.gnss_latency = declare_parameter("gnss_latency", config.gnss_latency);
    return config;
  }

  // The platform values of the marker stand in the platform file for all nodes, the final stretch of the docking with
  // them: dock_overshoot and contact_timeout of the logic.
  DockConfig ReadDockConfig() {
    DockConfig config;
    config.noise.odometry = get_parameter("odometry_noise").as_double();
    config.noise.gyro = get_parameter("gyro_noise").as_double();
    const std::vector<double> marker = declare_parameter<std::vector<double>>("dock_marker");
    config.max_age = declare_parameter<double>("marker_max_age");
    config.blind_distance = declare_parameter<double>("marker_blind_distance");
    const double overshoot = declare_parameter<double>("dock_overshoot");
    const double contact_timeout = declare_parameter<double>("contact_timeout");
    const bool finite = std::all_of(marker.begin(), marker.end(), [](double v) { return std::isfinite(v); });
    if (marker.size() != 3 || !finite || !(config.max_age > 0) || !(config.blind_distance >= 0) ||
        !(overshoot >= 0) || !(contact_timeout > 0)) {
      throw std::invalid_argument("dock_marker must be [x, y, yaw], marker_max_age and contact_timeout above 0, "
                                  "marker_blind_distance and dock_overshoot 0 or more");
    }
    config.marker_x = marker[0];
    config.marker_y = marker[1];
    config.marker_yaw = marker[2];
    config.final_way = config.blind_distance + overshoot + 0.02;
    config.final_time = contact_timeout + 2.0;
    config.hint_hold = DockMarkerMsg::REQUEST_HOLD_S;
    return config;
  }

  void StartDock() {
    dock_ = std::make_unique<mowbite_journal::Journaled<DockEstimator>>(journal_, ReadDockConfig());
    dock_pub_ = create_publisher<PoseMsg>("localization/dock_pose", 10);
    marker_sub_ = create_subscription<DockMarkerMsg>(
        "hw/dock_marker", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        [this](const DockMarkerMsg::ConstSharedPtr& msg) { OnMarker(*msg); });
    hint_sub_ = create_subscription<std_msgs::msg::Float64>(
        "logic/dock_heading_hint", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        [this](const std_msgs::msg::Float64::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          dock_->Call<&DockEstimator::SetHint>(msg->data, MonotonicSeconds());
        });
  }

  void OnMarker(const DockMarkerMsg& msg) {
    if (msg.state != DockMarkerMsg::STATE_MEASURING || !msg.valid) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = MonotonicSeconds();
    // The time of the image on the clock of the motions: its age on the ROS clock taken from now.
    const double age = (this->now() - rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type())).seconds();
    MarkerImage image;
    image.time = now - age;
    image.x = msg.x;
    image.y = msg.y;
    image.yaw = msg.yaw;
    image.yaw_valid = msg.yaw_valid;
    std::copy(msg.covariance.begin(), msg.covariance.end(), image.covariance.begin());
    switch (dock_->Call<&DockEstimator::AddMarker>(image, now)) {
      case MarkerResult::kStarted:
        RCLCPP_INFO(get_logger(), "Dock pose set from the marker, heading from the %s",
                    dock_->View().heading_from_hint() ? "hint of the logic" : "marker");
        break;
      case MarkerResult::kDropped:
        RCLCPP_WARN(get_logger(), "The dock marker contradicted the dock pose %d times in a row: dock pose dropped",
                    DockConfig().outlier_reset);
        dock_dropped_ = true;
        PublishDock(now);
        break;
      case MarkerResult::kNoFrame:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "Dock marker ignored: no charging contacts in the geometry (dock_contact_x)");
        break;
      case MarkerResult::kTooOld:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Dock marker dropped: %.3f s old on arrival", age);
        break;
      default:
        break;
    }
  }

  void OnImu(const GyroMsg& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = MonotonicSeconds();
    const bool was_calibrated = estimator_.View().calibrated();
    const bool moved = estimator_.Call<&Estimator::AddImu>(now, msg.angular_velocity.z);
    for (const auto& warning : estimator_.Call<&Estimator::TakeGyroWarnings>()) {
      RCLCPP_WARN(get_logger(), "%s", warning.c_str());
    }
    if (!was_calibrated && estimator_.View().calibrated()) {
      RCLCPP_INFO(get_logger(), "Gyro offset: %.5f rad/s", estimator_.View().gyro_offset());
    }
    if (!moved) return;
    if (dock_) dock_->Call<&DockEstimator::AddMotion>(estimator_.View().motion());
    Publish(now);
    PublishDock(now);
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
    const FixResult result = estimator_.Call<&Estimator::AddFix>(fix, MonotonicSeconds());
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
      const Pose2 pose = estimator_.View().pose();
      RCLCPP_INFO(get_logger(), "Position set onto GPS: %.3f, %.3f", pose.x, pose.y);
    }
    if (estimator_.Call<&Estimator::TakeAligned>()) RCLCPP_INFO(get_logger(), "Heading aligned with the GPS course");
    WarnCourseRejections();
    if (auto jump = estimator_.Call<&Estimator::TakeJump>()) PublishJump(*jump);
  }

  // At most every 10 s, with the count since the last warning.
  void WarnCourseRejections() {
    rejected_courses_ += estimator_.Call<&Estimator::TakeCourseRejections>();
    const double now = MonotonicSeconds();
    if (rejected_courses_ == 0 || (course_warned_ && now - *course_warned_ < 10.0)) return;
    RCLCPP_WARN(get_logger(), "GPS course against the track of the fixes, dropped: %d times", rejected_courses_);
    rejected_courses_ = 0;
    course_warned_ = now;
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
      // The docked mower faces the way in with the contacts in front, against it with them behind; 0 is no contacts.
      if (dock_ && params.contains("dock_contact_x") && params["dock_contact_x"].is_number()) {
        const double contact_x = params["dock_contact_x"].get<double>();
        if (contact_x != 0.0) dock_->Call<&DockEstimator::SetDockedHeading>(contact_x < 0.0 ? M_PI : 0.0);
      }
      if (params.contains("ticks_per_m") && params["ticks_per_m"].is_number()) {
        ticks_per_m_ = params["ticks_per_m"].get<double>();
        ApplyWheelScale();
      }
      known = estimator_.View().geometry_known();
      changed = !known || estimator_.View().antenna_offset() != Eigen::Vector2d(x, y);
      estimator_.Call<&Estimator::SetAntennaOffset>(x, y, MonotonicSeconds());
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

  // Called with the mutex held.
  void ApplyWheelScale() {
    const double before = estimator_.View().wheel_scale();
    estimator_.Call<&Estimator::SetWheelScale>(driver_ticks_per_m_, ticks_per_m_);
    if (estimator_.View().wheel_scale() != before) {
      RCLCPP_INFO(get_logger(), "Wheel speeds count %.4f times (driver %.1f, geometry %.1f ticks per m)",
                  estimator_.View().wheel_scale(), driver_ticks_per_m_, ticks_per_m_);
    }
  }

  void OnSetPose(const mowbite_msgs::srv::SetPose::Request& req) {
    std::lock_guard<std::mutex> lock(mutex_);
    estimator_.Call<&Estimator::SetPose>(Pose2{req.pose.position.x, req.pose.position.y, YawOf(req.pose.orientation)});
    const Pose2 pose = estimator_.View().pose();
    RCLCPP_INFO(get_logger(), "Pose set to %.3f, %.3f, heading %.3f", pose.x, pose.y, pose.heading);
    Publish(MonotonicSeconds());
  }

  void OnSetHeading(const mowbite_msgs::srv::SetHeading::Request& req, mowbite_msgs::srv::SetHeading::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = MonotonicSeconds();
    const std::optional<Pose2> pose = estimator_.Call<&Estimator::SetHeading>(req.heading, now);
    if (!pose) {
      res.success = false;
      res.message = std::isfinite(req.heading) ? "no recent RTK fix taken by the localization"
                                               : "the heading is not a number";
      return;
    }
    res.success = true;
    res.x = pose->x;
    res.y = pose->y;
    res.position_accuracy = static_cast<float>(estimator_.View().PositionAccuracy(now));
    RCLCPP_INFO(get_logger(), "Heading set to %.3f, base_link at %.3f, %.3f", pose->heading, pose->x, pose->y);
    Publish(now);
  }

  void Publish(double now) {
    const Pose2 pose = estimator_.View().pose();
    const Covariance& p = estimator_.View().covariance();
    const rclcpp::Time stamp = this->now();

    PoseMsg msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.x = pose.x;
    msg.y = pose.y;
    msg.heading = WrapAngle(pose.heading);
    msg.heading_valid = estimator_.View().heading_valid();
    msg.position_accuracy = static_cast<float>(estimator_.View().PositionAccuracy(now));
    msg.heading_accuracy = static_cast<float>(estimator_.View().HeadingAccuracy());
    msg.gnss_recent = estimator_.View().RecentGps(now);
    msg.speed = estimator_.View().speed();
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
    odometry.twist.twist.angular.z = estimator_.View().turn_rate();
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

  // localization/dock_pose while there is an estimate, and once with the accuracy unknown when it was dropped. Called
  // with the mutex held.
  void PublishDock(double now) {
    if (!dock_ || (!dock_->View().has_estimate() && !dock_dropped_)) return;
    dock_dropped_ = false;
    const DockEstimator& dock = dock_->View();
    const Pose2 pose = dock.pose();
    const Covariance& p = dock.covariance();
    PoseMsg msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "dock";
    msg.x = pose.x;
    msg.y = pose.y;
    msg.heading = WrapAngle(pose.heading);
    msg.heading_valid = dock.has_estimate();
    msg.position_accuracy = static_cast<float>(dock.PositionAccuracy(now));
    msg.heading_accuracy = static_cast<float>(dock.HeadingAccuracy());
    msg.gnss_recent = false;
    msg.speed = estimator_.View().speed();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) msg.covariance[3 * r + c] = p(r, c);
    }
    dock_pub_->publish(msg);
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
  // Null without journal_dir.
  mowbite_journal::Journal* journal_;
  mowbite_journal::Journaled<Estimator> estimator_;
  mowbite_journal::JournalWatch journal_watch_;
  rclcpp::TimerBase::SharedPtr journal_timer_;
  bool had_gps_ = false;
  int rejected_courses_ = 0;
  std::optional<double> course_warned_;
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
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr caps_sub_;
  double driver_ticks_per_m_ = 0;
  double ticks_per_m_ = 0;
  rclcpp::Service<mowbite_msgs::srv::SetPose>::SharedPtr set_pose_srv_;
  rclcpp::Service<mowbite_msgs::srv::SetHeading>::SharedPtr set_heading_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_gps_srv_;
  // Null without a dock marker on the platform.
  std::unique_ptr<mowbite_journal::Journaled<DockEstimator>> dock_;
  bool dock_dropped_ = false;
  rclcpp::Publisher<PoseMsg>::SharedPtr dock_pub_;
  rclcpp::Subscription<DockMarkerMsg>::SharedPtr marker_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hint_sub_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_localization

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_localization::LocalizationNode)
