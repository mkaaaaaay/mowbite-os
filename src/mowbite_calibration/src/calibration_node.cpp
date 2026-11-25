#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <map>
#include <memory>
#include <mowbite_component/node_options.h>
#include <mowbite_msgs/action/calibrate.hpp>
#include <mowbite_msgs/msg/calibration_status.hpp>
#include <mowbite_msgs/msg/gnss_fix.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/map_status.hpp>
#include <mowbite_msgs/msg/position_jump.hpp>
#include <mowbite_msgs/msg/robot_pose.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <stdexcept>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include "mowbite_calibration/lane.h"
#include "mowbite_calibration/program.h"
#include "mowbite_calibration/recording.h"
#include "mowbite_calibration/result.h"
#include "mowbite_watch/attach.h"

namespace mowbite_calibration {

namespace {

using Calibrate = mowbite_msgs::action::Calibrate;
using GoalHandle = rclcpp_action::ServerGoalHandle<Calibrate>;
using mowbite_msgs::msg::GnssFix;

// Calls of the planner from here carry ids with this prefix; the bridge keeps their answers from MQTT.
constexpr const char* kCallPrefix = "os/calibration/";
constexpr const char* kEvaluate = "planner.calib.evaluate";
// The mounting of the IMU may be this far off as long as it is not measured (rad), as robot.geometry allows.
constexpr double kMountAllowance = 0.087;
// The lane is chosen again when the mower moved or turned this much, or the map or the dimensions changed.
constexpr double kLaneMove = 0.05;
constexpr double kLaneTurn = 2.0 * M_PI / 180.0;
constexpr double kLaneRefresh = 5.0;
// The pose goes into the recording at most this often (s).
constexpr double kPosePeriod = 0.1;

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

double WallSeconds() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

double Round(double value, double scale) {
  return std::round(value * scale) / scale;
}

const char* FixName(uint8_t solution) {
  switch (solution) {
    case GnssFix::RTK_FIXED: return "fixed";
    case GnssFix::RTK_FLOAT: return "float";
    case GnssFix::SINGLE:
    case GnssFix::DGPS: return "single";
    default: return "none";
  }
}

std::string Fields(const std::vector<std::pair<std::string, double>>& numbers) {
  json fields = json::object();
  for (const auto& [name, value] : numbers) {
    if (std::isfinite(value)) fields[name] = Round(value, 1000.0);
  }
  return fields.dump();
}

mowbite_msgs::msg::CalibrationProblem ProblemMsg(const Problem& problem) {
  mowbite_msgs::msg::CalibrationProblem msg;
  msg.reason = problem.reason;
  msg.message = problem.message;
  msg.fields = Fields(problem.numbers);
  return msg;
}

// What the program needs from the dimensions of the logic (/mowbite_geometry on bridge/params).
struct Dimensions {
  double body_radius = 0;
  double ticks_per_m = 0;
  double track = 0;
  double antenna_x = 0;
  double antenna_y = 0;
  double imu_roll = 0;
  double imu_pitch = 0;
  // imu_roll or imu_pitch entered by the user, so measured.
  bool imu_measured = false;
  int version = 0;
  // The last calibration.apply, null before one.
  json applied;
};

std::optional<Dimensions> ReadDimensions(const std::string& text) {
  const json parsed = json::parse(text, nullptr, false);
  if (!parsed.is_object() || !parsed.contains("node") || parsed["node"] != "/mowbite_geometry" ||
      !parsed.contains("params")) {
    return std::nullopt;
  }
  const json& p = parsed["params"];
  if (!p.is_object()) return std::nullopt;
  const auto number = [&p](const char* key) -> std::optional<double> {
    if (!p.contains(key) || !p[key].is_number()) return std::nullopt;
    const double value = p[key].get<double>();
    return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
  };
  Dimensions d;
  const auto front = number("robot_front"), rear = number("robot_rear"), width = number("robot_width");
  const auto ticks = number("ticks_per_m"), track = number("track"), ax = number("antenna_x"),
             ay = number("antenna_y"), roll = number("imu_roll"), pitch = number("imu_pitch");
  if (!front || !rear || !width || !ticks || !track || !ax || !ay || !roll || !pitch) return std::nullopt;
  std::vector<mowbite_safety::Point2> outline;
  if (p.contains("robot_outline") && p["robot_outline"].is_array()) {
    for (const auto& point : p["robot_outline"]) {
      if (point.is_array() && point.size() == 2 && point[0].is_number() && point[1].is_number()) {
        outline.push_back({point[0].get<double>(), point[1].get<double>()});
      }
    }
  }
  d.body_radius = BodyRadius(*front, *rear, *width, outline);
  d.ticks_per_m = *ticks;
  d.track = *track;
  d.antenna_x = *ax;
  d.antenna_y = *ay;
  d.imu_roll = *roll;
  d.imu_pitch = *pitch;
  if (p.contains("stored") && p["stored"].is_array()) {
    for (const auto& key : p["stored"]) {
      if (key == "imu_roll" || key == "imu_pitch") d.imu_measured = true;
    }
  }
  d.version = p.contains("version") && p["version"].is_number_integer() ? p["version"].get<int>() : 0;
  if (p.contains("calibration_applied") && p["calibration_applied"].is_object()) d.applied = p["calibration_applied"];
  return d;
}

}  // namespace

// The calibration drive (Kalibrierfahrt) on calibration/run: the program on calibration/cmd_vel, which the logic passes
// on to S2 while it calibrates; the recording in <data_dir>/calibration/recordings, its evaluation by the planner
// (planner.calib.evaluate) and the result in <data_dir>/calibration/result.json for calibration.result. Twice a second
// calibration/status tells the logic what keeps a drive from starting here and the lane it would take.
class CalibrationNode : public rclcpp::Node {
 public:
  explicit CalibrationNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_calibration", mowbite_component::StartupOnly(options)), program_(ProgramConfig()) {
    ProgramConfig config;
    config.spin_rate = declare_parameter("spin_rate", config.spin_rate);
    config.turn_min_ratio = declare_parameter("turn_min_ratio", config.turn_min_ratio);
    config.line_min_speed = declare_parameter("line_min_speed", config.line_min_speed);
    if (!(config.spin_rate > 0) || !(config.turn_min_ratio > 0 && config.turn_min_ratio < 1) ||
        !(config.line_min_speed > 0)) {
      throw std::invalid_argument(
          "mowbite_calibration: spin_rate and line_min_speed above 0 and turn_min_ratio between 0 and 1 needed");
    }
    program_ = Program(config);
    data_dir_ = declare_parameter("data_dir", std::string());
    software_ = declare_parameter("software_version", std::string());
    evaluate_timeout_ = declare_parameter("evaluate_timeout", 120.0);
    limits_.max_runs = static_cast<size_t>(declare_parameter("recordings_keep", 5));
    limits_.max_bytes = static_cast<uint64_t>(declare_parameter("recordings_max_mb", 20.0) * 1000.0 * 1000.0);
    limits_.max_days = declare_parameter("recordings_keep_days", 60.0);
    if (data_dir_.empty()) RCLCPP_WARN(get_logger(), "No data_dir: drives are neither recorded nor evaluated");
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("calibration/cmd_vel", 10);
    result_pub_ = create_publisher<std_msgs::msg::String>("calibration/result", LatchedQos(1));
    LoadResult();
    status_pub_ = create_publisher<mowbite_msgs::msg::CalibrationStatus>("calibration/status", 10);
    request_pub_ = create_publisher<mowbite_msgs::msg::RpcRequest>("bridge/rpc/request", 100);
    response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));

    imu_in_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { OnImu(*msg); });
    wheel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "hw/wheel_odom", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) { OnWheels(*msg); });
    gnss_sub_ = create_subscription<GnssFix>("gps/absolute_pose", 10,
                                             [this](GnssFix::ConstSharedPtr msg) { OnGnss(*msg); });
    pose_sub_ = create_subscription<mowbite_msgs::msg::RobotPose>(
        "localization/pose", 10, [this](mowbite_msgs::msg::RobotPose::ConstSharedPtr msg) { OnPose(*msg); });
    jump_sub_ = create_subscription<mowbite_msgs::msg::PositionJump>(
        "localization/jump", 10, [this](mowbite_msgs::msg::PositionJump::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          program_.OnPositionJump(msg->distance, MonotonicSeconds());
        });
    hw_sub_ = create_subscription<mowbite_msgs::msg::HardwareStatus>(
        "hw/status", 10, [this](mowbite_msgs::msg::HardwareStatus::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          program_.OnEmergency(msg->emergency, MonotonicSeconds());
        });
    caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", LatchedQos(1), [this](mowbite_msgs::msg::HardwareCapabilities::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          caps_ = *msg;
          if (!program_.Running()) program_.SetSpeedLimits(msg->max_linear_speed, msg->max_angular_speed);
        });
    map_sub_ = create_subscription<std_msgs::msg::String>(
        "map/json", LatchedQos(1), [this](std_msgs::msg::String::ConstSharedPtr msg) { OnMap(msg->data); });
    map_status_sub_ = create_subscription<mowbite_msgs::msg::MapStatus>(
        "map/status", LatchedQos(1), [this](mowbite_msgs::msg::MapStatus::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          map_fits_ = msg->datum_state == mowbite_msgs::msg::MapStatus::DATUM_MATCH;
          lane_s_.reset();
        });
    params_sub_ = create_subscription<std_msgs::msg::String>(
        "bridge/params", LatchedQos(50), [this](std_msgs::msg::String::ConstSharedPtr msg) { OnParams(msg->data); });
    methods_sub_ = create_subscription<mowbite_msgs::msg::RpcMethods>(
        "bridge/rpc/methods", LatchedQos(50), [this](mowbite_msgs::msg::RpcMethods::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          offers_[msg->node_id] =
              std::find(msg->methods.begin(), msg->methods.end(), kEvaluate) != msg->methods.end();
        });
    request_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](mowbite_msgs::msg::RpcRequest::ConstSharedPtr msg) { OnRequest(*msg); });
    response_sub_ = create_subscription<mowbite_msgs::msg::RpcResponse>(
        "bridge/rpc/response", 100, [this](mowbite_msgs::msg::RpcResponse::ConstSharedPtr msg) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!Awaited(msg->id)) return;
          }
          OnAnswer(msg->id, json::parse(msg->result, nullptr, false), std::nullopt);
        });
    error_sub_ = create_subscription<mowbite_msgs::msg::RpcError>(
        "bridge/rpc/error", 100, [this](mowbite_msgs::msg::RpcError::ConstSharedPtr msg) {
          OnAnswer(msg->id, json(), std::make_pair(msg->code, msg->message));
        });

    server_ = rclcpp_action::create_server<Calibrate>(
        this, "calibration/run",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Calibrate::Goal>) {
          std::lock_guard<std::mutex> lock(mutex_);
          // One drive at a time, its evaluation included.
          return goal_ ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const std::shared_ptr<GoalHandle> handle) {
          std::lock_guard<std::mutex> lock(mutex_);
          goal_ = handle;
          start_due_ = true;
        });

    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_calibration";
    methods.methods = {"calibration.result"};
    methods_pub_->publish(methods);

    tick_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { OnTick(); });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { PublishStatus(); });
    watch_ = mowbite_watch::Attach(*this);
  }

 private:
  void OnImu(const sensor_msgs::msg::Imu& msg) {
    ImuSample sample;
    sample.gx = msg.angular_velocity.x;
    sample.gy = msg.angular_velocity.y;
    sample.gz = msg.angular_velocity.z;
    sample.ax = msg.linear_acceleration.x;
    sample.ay = msg.linear_acceleration.y;
    sample.az = msg.linear_acceleration.z;
    std::lock_guard<std::mutex> lock(mutex_);
    imu_frame_ = msg.header.frame_id.empty() ? "base_link" : msg.header.frame_id;
    program_.OnImu(sample, MonotonicSeconds());
    if (recording_.open()) {
      Record("imu", {{"gx", Round(sample.gx, 1e6)},
                     {"gy", Round(sample.gy, 1e6)},
                     {"gz", Round(sample.gz, 1e6)},
                     {"ax", Round(sample.ax, 1e5)},
                     {"ay", Round(sample.ay, 1e5)},
                     {"az", Round(sample.az, 1e5)}});
    }
  }

  void OnWheels(const geometry_msgs::msg::TwistStamped& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    program_.OnWheels(msg.twist.linear.x, msg.twist.angular.z, MonotonicSeconds());
    if (recording_.open()) {
      Record("twist", {{"v", Round(msg.twist.linear.x, 1e5)}, {"w", Round(msg.twist.angular.z, 1e5)}});
    }
  }

  void OnGnss(const GnssFix& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    program_.OnGnss({msg.x, msg.y, msg.accuracy, msg.solution == GnssFix::RTK_FIXED}, MonotonicSeconds());
    if (!recording_.open()) return;
    json fields = {{"x", Round(msg.x, 1e4)}, {"y", Round(msg.y, 1e4)},  {"z", Round(msg.z, 1e4)},
                   {"acc", Round(msg.accuracy, 1e4)}, {"fix", FixName(msg.solution)}, {"vh", nullptr},
                   {"mh", nullptr}};
    if (msg.velocity_valid && std::isfinite(msg.velocity_east) && std::isfinite(msg.velocity_north)) {
      fields["ve"] = Round(msg.velocity_east, 1e4);
      fields["vn"] = Round(msg.velocity_north, 1e4);
      // The course over ground, from east counterclockwise, while the antenna moves.
      if (std::hypot(msg.velocity_east, msg.velocity_north) > 0.05) {
        fields["vh"] = Round(std::atan2(msg.velocity_north, msg.velocity_east), 1e5);
      }
    }
    // Only a receiver time in UTC can be set against other clocks.
    if (caps_ && caps_->gnss_receiver_time) fields["rt"] = msg.receiver_time_ms / 1000.0;
    Record("gps", fields);
  }

  void OnPose(const mowbite_msgs::msg::RobotPose& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = MonotonicSeconds();
    PoseSample pose{msg.x, msg.y, msg.heading, msg.heading_valid, msg.position_accuracy};
    program_.OnPose(pose, now);
    pose_ = pose;
    pose_s_ = now;
    if (recording_.open() && (!pose_recorded_s_ || now - *pose_recorded_s_ >= kPosePeriod - 1e-3)) {
      pose_recorded_s_ = now;
      Record("pose", {{"x", Round(msg.x, 1e4)}, {"y", Round(msg.y, 1e4)}, {"yaw", Round(msg.heading, 1e5)}});
    }
  }

  void OnMap(const std::string& text) {
    std::string error;
    auto region = mowbite_safety::ParseRegion(text, &error);
    std::lock_guard<std::mutex> lock(mutex_);
    region_ = std::move(region);
    map_error_ = region_ ? "" : error;
    lane_s_.reset();
  }

  void OnParams(const std::string& text) {
    auto dimensions = ReadDimensions(text);
    if (!dimensions) return;
    std::lock_guard<std::mutex> lock(mutex_);
    dimensions_ = dimensions;
    program_.SetMounting(dimensions->imu_roll, dimensions->imu_pitch,
                         dimensions->imu_measured ? 0.0 : kMountAllowance);
    lane_s_.reset();
  }

  // Called with the mutex held.
  void Record(const char* kind, const json& fields) {
    if (!recording_.Line(RecordingTime(), kind, fields) || recording_.open() || recording_.finished().empty()) return;
    RCLCPP_WARN(get_logger(), "The recording reached %.0f MB and ended: %s", limits_.max_bytes / 1e6,
                recording_.finished().c_str());
  }

  // The lane from where the mower stands, again only when it moved, turned, or the map or dimensions changed.
  // Called with the mutex held.
  void UpdateLane(double now) {
    if (!pose_ || !region_ || !map_fits_ || !dimensions_) {
      lane_.reset();
      lane_s_.reset();
      return;
    }
    if (lane_s_ && lane_ && now - *lane_s_ < kLaneRefresh &&
        std::hypot(pose_->x - lane_at_.x, pose_->y - lane_at_.y) < kLaneMove &&
        std::abs(std::remainder(pose_->heading - lane_at_.heading, 2 * M_PI)) < kLaneTurn) {
      return;
    }
    lane_ = ChooseLane(*region_, pose_->x, pose_->y, pose_->heading, dimensions_->body_radius, lane_config_);
    lane_at_ = *pose_;
    lane_s_ = now;
  }

  // What keeps a drive from starting here, besides the program's own readiness. Called with the mutex held.
  std::vector<Problem> RoomProblems() const {
    if (!caps_) return {{"SENSOR_MISSING", "No capabilities from the driver yet", {}}};
    if (!dimensions_) return {{"NO_ROOM", "The dimensions of the mower are not known yet", {}}};
    if (!region_) {
      return {{"NO_ROOM", map_error_.empty() ? "No map yet" : "The map cannot be read: " + map_error_, {}}};
    }
    if (!map_fits_) return {{"NO_ROOM", "The map does not fit the position of the GNSS", {}}};
    if (!pose_) return {{"NO_ROOM", "No position from the localization", {}}};
    if (lane_ && lane_->lane) return {};
    const double free = lane_ ? lane_->free_length : 0.0;
    char text[200];
    std::snprintf(text, sizeof(text), "No room for a calibration drive here: %.1f m free, %.1f m needed", free,
                  lane_config_.min_length);
    return {{"NO_ROOM",
             text,
             {{"free_length", free},
              {"length", lane_config_.min_length},
              {"strip_length", lane_ ? lane_->strip_length : 0.0},
              {"strip_width", lane_ ? lane_->strip_width : 0.0}}}};
  }

  void PublishStatus() {
    mowbite_msgs::msg::CalibrationStatus msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double now = MonotonicSeconds();
      msg.running = goal_ != nullptr;
      if (!goal_) UpdateLane(now);
      for (const auto& problem : program_.Readiness(now)) msg.problems.push_back(ProblemMsg(problem));
      for (const auto& problem : RoomProblems()) msg.problems.push_back(ProblemMsg(problem));
      if (lane_) {
        msg.strip_length = lane_->strip_length;
        msg.strip_width = lane_->strip_width;
        if (lane_->lane) {
          msg.heading = lane_->lane->heading;
          msg.lane_length = lane_->lane->length;
          msg.rounds = program_.config().rounds;
          msg.expected_s = program_.ExpectedSeconds(*lane_->lane);
        }
      }
    }
    msg.stamp = now();
    status_pub_->publish(msg);
  }

  // Called with the mutex held.
  void Send(const Twist2& twist) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = twist.linear;
    msg.angular.z = twist.angular;
    cmd_pub_->publish(msg);
    if (recording_.open()) Record("cmd", {{"v", Round(twist.linear, 1e5)}, {"w", Round(twist.angular, 1e5)}});
  }

  void OnTick() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!goal_) return;
    const double now = MonotonicSeconds();
    if (start_due_) {
      start_due_ = false;
      StartDrive(now);
      if (!goal_) return;
    }
    if (program_.Running()) {
      std::optional<Twist2> out =
          goal_->is_canceling() ? program_.Stop("Stopped by the logic", now) : program_.Tick(now);
      if (out) Send(*out);
      for (const auto& edge : program_.TakeEdges()) {
        if (recording_.open()) {
          Record("phase", {{"name", PhaseName(edge.phase)}, {"edge", edge.start ? "start" : "end"}});
        }
      }
      if (!program_.Running()) DriveEnded(now);
    } else if (evaluating_) {
      if (goal_->is_canceling()) {
        RCLCPP_WARN(get_logger(), "Calibration: stopped while the planner evaluated, the recording stays");
        Finish(json(), "STOPPED", "Stopped while the planner evaluated the recording");
        return;
      }
      if (now - evaluate_s_ > evaluate_timeout_) {
        RCLCPP_WARN(get_logger(), "Calibration: the planner did not answer within %.0f s", evaluate_timeout_);
        Finish(json(), "", "");
        return;
      }
    }
    if (goal_ && now - feedback_s_ >= 0.25) {
      feedback_s_ = now;
      auto feedback = std::make_shared<Calibrate::Feedback>();
      feedback->phase = evaluating_ ? "evaluate" : PhaseName(program_.phase());
      feedback->index = static_cast<uint8_t>(std::clamp(program_.round(), 0, 255));
      feedback->progress = static_cast<float>(evaluating_ ? 1.0 : program_.Progress(now));
      goal_->publish_feedback(feedback);
    }
  }

  // The goal of the logic: the lane it checked, from where base_link stands now. Called with the mutex held.
  void StartDrive(double now) {
    const auto goal = goal_->get_goal();
    ending_ = Ending();
    ending_.id = "calib-" + std::to_string(static_cast<long long>(WallSeconds()));
    if (!pose_ || !region_ || !map_fits_ || !dimensions_ || !caps_) {
      Refuse("NO_ROOM", "No map, position or dimensions to check the lane with");
      return;
    }
    Lane lane{pose_->x, pose_->y, goal->heading, goal->lane_length};
    if (!(goal->lane_length > 0) || !LaneFits(*region_, lane, dimensions_->body_radius, lane_config_)) {
      Refuse("NO_ROOM", "The lane does not fit where the mower stands now");
      return;
    }
    program_.SetSpeedLimits(caps_->max_linear_speed, caps_->max_angular_speed);
    if (!program_.Start(lane, now)) {
      Refuse("BUSY", "A calibration drive runs already");
      return;
    }
    ending_.geometry_version = dimensions_->version;
    OpenRecording(lane);
    evaluating_ = false;
    feedback_s_ = 0;
    pose_recorded_s_.reset();
    RCLCPP_INFO(get_logger(), "Calibration: lanes of %.1f m at %.0f deg, %s", lane.length, lane.heading * 180 / M_PI,
                recording_.open() ? "recording" : "not recorded");
  }

  // Called with the mutex held.
  void Refuse(const std::string& reason, const std::string& message) {
    RCLCPP_WARN(get_logger(), "Calibration refused: %s (%s)", reason.c_str(), message.c_str());
    ending_.reason = reason;
    ending_.message = message;
    ending_.time = WallSeconds();
    auto result = std::make_shared<Calibrate::Result>();
    result->reason = reason;
    result->message = message;
    result->status = "failed";
    result->summary = "The drive did not start: " + reason;
    goal_->abort(result);
    goal_.reset();
  }

  // The time of a line: the wall clock at the start of the recording, then the monotonic clock, so that a clock
  // set meanwhile does not break the time axis the planner evaluates.
  double RecordingTime() const {
    return recording_wall_ + (MonotonicSeconds() - recording_mono_);
  }

  // Called with the mutex held.
  void OpenRecording(const Lane& lane) {
    recorded_ = false;
    if (data_dir_.empty()) return;
    recording_wall_ = WallSeconds();
    recording_mono_ = MonotonicSeconds();
    const ProgramConfig& config = program_.config();
    json head = json::object();
    head["mode"] = "program";
    head["started"] = Round(recording_wall_, 1000.0);
    head["params"] = {{"ticks_per_m", dimensions_->ticks_per_m},
                      {"wheel_distance_m", dimensions_->track},
                      {"antenna_offset_x", dimensions_->antenna_x},
                      {"antenna_offset_y", dimensions_->antenna_y}};
    if (caps_->wheel_ticks_per_m > 0 && caps_->wheel_distance > 0) {
      head["basis"] = {{"ticks_per_m", caps_->wheel_ticks_per_m}, {"wheel_distance_m", caps_->wheel_distance}};
    }
    head["imu_frame"] = imu_frame_;
    head["platform"] = caps_->platform;
    head["imu"] = {{"roll", dimensions_->imu_roll}, {"pitch", dimensions_->imu_pitch}};
    head["program"] = {{"rounds", config.rounds},
                       {"lane_length", Round(lane.length, 1e3)},
                       {"heading", Round(lane.heading, 1e5)},
                       {"line_speed", config.line_speed},
                       {"spin_rate", config.spin_rate},
                       {"spin_angle", Round(config.spin_angle, 1e5)}};
    if (!software_.empty()) head["software"] = software_;
    std::string why;
    recorded_ = recording_.Open(RecordingsDir(), head, recording_wall_, limits_, &why);
    if (!recorded_) RCLCPP_ERROR(get_logger(), "Calibration: cannot record the drive: %s", why.c_str());
  }

  // The program stopped: end the recording, then the planner evaluates it, if it can. Called with the mutex held.
  void DriveEnded(double now) {
    const Problem outcome = program_.outcome().value_or(Problem{"STOPPED", "The program ended", {}});
    ending_.reason = outcome.reason;
    ending_.message = outcome.message;
    ending_.time = WallSeconds();
    std::string why = "error";
    if (outcome.reason == "DONE") why = "enough";
    if (outcome.reason == "STOPPED") why = "stop";
    if (recording_.open()) {
      ending_.recording = recording_.End(RecordingTime(), why, outcome.reason == "DONE" ? "" : outcome.reason);
      if (ending_.recording.empty()) RCLCPP_ERROR(get_logger(), "Calibration: the recording could not be finished");
      if (recording_.dropped() > 0) {
        RCLCPP_WARN(get_logger(), "Calibration: %zu lines were left out of the recording", recording_.dropped());
      }
    } else if (recorded_ && recording_.finished() == recording_.path()) {
      // Ended by its size already.
      ending_.recording = recording_.finished();
    }
    if (!ending_.recording.empty()) Prune(RecordingsDir(), WallSeconds(), limits_, ending_.recording);
    RCLCPP_INFO(get_logger(), "Calibration drive ended: %s (%s)", outcome.reason.c_str(), outcome.message.c_str());
    if (outcome.reason != "DONE" || ending_.recording.empty() || !PlannerOffers()) {
      if (outcome.reason == "DONE" && !ending_.recording.empty()) {
        RCLCPP_WARN(get_logger(), "Calibration: no planner to evaluate %s; it can be evaluated later",
                    ending_.recording.c_str());
      }
      Finish(json(), "", "");
      return;
    }
    // planner.calib.evaluate {path} over the bridge; the answer comes on bridge/rpc/response or error.
    call_id_ = std::string(kCallPrefix) + std::to_string(++calls_);
    mowbite_msgs::msg::RpcRequest request;
    request.method = kEvaluate;
    request.params = json{{"path", ending_.recording}}.dump();
    request.id = call_id_;
    request_pub_->publish(request);
    evaluating_ = true;
    evaluate_s_ = now;
    RCLCPP_INFO(get_logger(), "Calibration: the planner evaluates %s", ending_.recording.c_str());
  }

  // Called with the mutex held.
  bool PlannerOffers() const {
    for (const auto& [node, offers] : offers_) {
      if (offers) return true;
    }
    return false;
  }

  // An answer to the evaluation asked for. Called with the mutex held.
  bool Awaited(const std::string& id) const {
    return evaluating_ && id == call_id_;
  }

  void OnAnswer(const std::string& id, const json& result, const std::optional<std::pair<int, std::string>>& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!Awaited(id)) return;
    if (error) {
      RCLCPP_WARN(get_logger(), "Calibration: %s failed: %s (%d)", kEvaluate, error->second.c_str(), error->first);
      // Without the method there is no evaluation; any other error is the planner's verdict on the data.
      Finish(error->first == mowbite_msgs::msg::RpcError::ERROR_METHOD_NOT_FOUND ? json()
                                                                                 : json{{"error", error->second}},
             "", "");
      return;
    }
    if (!result.is_object()) {
      Finish(json{{"error", "the planner answered with no object"}}, "", "");
      return;
    }
    Finish(result, "", "");
  }

  // The result for calibration.result and the logic; reason and message override those of the drive for the logic
  // (a stop while evaluating). Called with the mutex held.
  void Finish(const json& evaluation, const std::string& reason, const std::string& message) {
    evaluating_ = false;
    call_id_.clear();
    json composed = Compose(ending_, evaluation);
    std::string why;
    if (!data_dir_.empty() && !WriteWhole(ResultPath(), composed.dump(2) + "\n", &why)) {
      RCLCPP_ERROR(get_logger(), "Calibration: cannot save the result: %s", why.c_str());
    }
    result_ = composed;
    PublishResult();
    auto result = std::make_shared<Calibrate::Result>();
    result->reason = reason.empty() ? ending_.reason : reason;
    result->message = message.empty() ? ending_.message : message;
    result->status = composed.value("status", "");
    result->summary = composed.value("summary", "");
    result->recording = ending_.recording;
    RCLCPP_INFO(get_logger(), "Calibration: %s, %s", result->status.c_str(), result->summary.c_str());
    if (!goal_) return;
    if (goal_->is_canceling()) {
      goal_->canceled(result);
    } else if (result->reason == "DONE") {
      goal_->succeed(result);
    } else {
      goal_->abort(result);
    }
    goal_.reset();
  }

  void OnRequest(const mowbite_msgs::msg::RpcRequest& request) {
    if (request.method != "calibration.result") return;
    mowbite_msgs::msg::RpcResponse msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      json answer = result_;
      // applied: the last calibration.apply when it took from this result.
      if (answer.is_object()) {
        const json applied = dimensions_ ? dimensions_->applied : json();
        const bool ours = applied.is_object() && applied.value("result_id", json()) == answer.value("id", json());
        answer["applied"] = ours ? applied : json();
      }
      msg.result = answer.dump(-1, ' ', false, json::error_handler_t::replace);
    }
    msg.id = request.id;
    response_pub_->publish(msg);
  }

  void LoadResult() {
    if (data_dir_.empty()) return;
    std::ifstream in(ResultPath());
    if (!in) return;
    std::stringstream text;
    text << in.rdbuf();
    const json parsed = json::parse(text.str(), nullptr, false);
    if (parsed.is_object()) {
      result_ = parsed;
      PublishResult();
    } else {
      RCLCPP_ERROR(get_logger(), "%s is not valid and is not offered", ResultPath().c_str());
    }
  }

  // calibration/result for the logic (calibration.apply) and the setup program. Called with the mutex held or before
  // the callbacks run.
  void PublishResult() {
    std_msgs::msg::String msg;
    msg.data = result_.dump(-1, ' ', false, json::error_handler_t::replace);
    result_pub_->publish(msg);
  }

  std::string RecordingsDir() const { return data_dir_ + "/calibration/recordings"; }
  std::string ResultPath() const { return data_dir_ + "/calibration/result.json"; }

  std::mutex mutex_;
  Program program_;
  LaneConfig lane_config_;
  RecordingLimits limits_;
  Recording recording_;
  double recording_wall_ = 0;
  double recording_mono_ = 0;
  // The drive that runs has a recording (it may have ended by its size).
  bool recorded_ = false;
  std::string data_dir_;
  std::string software_;
  double evaluate_timeout_ = 120.0;
  std::string imu_frame_ = "base_link";
  std::optional<mowbite_msgs::msg::HardwareCapabilities> caps_;
  std::optional<Dimensions> dimensions_;
  std::optional<mowbite_safety::Region> region_;
  std::string map_error_;
  bool map_fits_ = false;
  std::optional<PoseSample> pose_;
  std::optional<double> pose_s_;
  std::optional<double> pose_recorded_s_;
  std::optional<LaneChoice> lane_;
  PoseSample lane_at_;
  std::optional<double> lane_s_;
  std::map<std::string, bool> offers_;
  std::shared_ptr<GoalHandle> goal_;
  bool start_due_ = false;
  bool evaluating_ = false;
  double evaluate_s_ = 0;
  double feedback_s_ = 0;
  std::string call_id_;
  uint64_t calls_ = 0;
  Ending ending_;
  json result_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::CalibrationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcRequest>::SharedPtr request_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_in_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_sub_;
  rclcpp::Subscription<GnssFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::PositionJump>::SharedPtr jump_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr caps_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MapStatus>::SharedPtr map_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcMethods>::SharedPtr methods_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcResponse>::SharedPtr response_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcError>::SharedPtr error_sub_;
  rclcpp_action::Server<Calibrate>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace mowbite_calibration

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_calibration::CalibrationNode)
