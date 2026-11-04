#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mowbite_msgs/action/follow_path.hpp>
#include <mowbite_msgs/msg/action_list.hpp>
#include <mowbite_msgs/msg/drive_command.hpp>
#include <mowbite_msgs/msg/gnss_fix.hpp>
#include <mowbite_msgs/msg/gps_limits.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/map_status.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/outline.hpp>
#include <mowbite_msgs/msg/position_jump.hpp>
#include <mowbite_msgs/msg/robot_pose.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mowbite_msgs/srv/add_area.hpp>
#include <mowbite_msgs/srv/drive_to.hpp>
#include <mowbite_msgs/srv/set_blade.hpp>
#include <mowbite_msgs/srv/set_charge.hpp>
#include <mowbite_msgs/srv/set_dock.hpp>
#include <mowbite_msgs/srv/set_emergency.hpp>
#include <mowbite_msgs/srv/set_heading.hpp>
#include <mowbite_hold/hold.h>
#include <map>
#include <mowbite_planner_interfaces/srv/plan_coverage.hpp>
#include <mowbite_planner_interfaces/srv/plan_transit.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mowbite_bridge/event_builder.h"
#include "mowbite_logic/geometry.h"
#include "mowbite_logic/logic_core.h"
#include "mowbite_logic/logic_params.h"
#include "mowbite_logic/mowing_plan.h"

namespace mowbite_logic {

namespace {

using mowbite_msgs::msg::SensorSpec;
static_assert(static_cast<uint8_t>(SensorMeaning::kUnknown) == SensorSpec::MEANING_UNKNOWN);
static_assert(static_cast<uint8_t>(SensorMeaning::kVoltage) == SensorSpec::MEANING_VOLTAGE);
static_assert(static_cast<uint8_t>(SensorMeaning::kCurrent) == SensorSpec::MEANING_CURRENT);
static_assert(static_cast<uint8_t>(SensorMeaning::kTemperature) == SensorSpec::MEANING_TEMPERATURE);
static_assert(static_cast<uint8_t>(SensorMeaning::kRevolutions) == SensorSpec::MEANING_REVOLUTIONS);
static_assert(static_cast<uint8_t>(SensorMeaning::kDistance) == SensorSpec::MEANING_DISTANCE);

// S2 answers a release within 2 s (its own limit for the driver); later counts as no answer.
constexpr double kReleaseTimeoutS = 3.0;
// mowbite/hold goes out again this often while nothing changes.
constexpr double kHoldMirrorS = 30.0;

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

int64_t WallSeconds() {
  using namespace std::chrono;
  return duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
}

// The text becomes the file whole or not at all: written next to it, synced, renamed over it, the folder synced.
bool WriteFileSafely(const std::string& path, const std::string& text, std::string* error) {
  std::error_code ec;
  const std::filesystem::path folder = std::filesystem::path(path).parent_path();
  std::filesystem::create_directories(folder, ec);
  if (ec) {
    *error = "cannot create the folder of " + path + ": " + ec.message();
    return false;
  }
  const std::string temp = path + ".new";
  const int fd = ::open(temp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    *error = "cannot write " + temp + ": " + std::strerror(errno);
    return false;
  }
  size_t done = 0;
  while (done < text.size()) {
    const ssize_t n = ::write(fd, text.data() + done, text.size() - done);
    if (n < 0 && errno == EINTR) continue;
    if (n < 0) {
      *error = "cannot write " + temp + ": " + std::strerror(errno);
      ::close(fd);
      return false;
    }
    done += static_cast<size_t>(n);
  }
  const bool synced = ::fsync(fd) == 0;
  ::close(fd);
  if (!synced) {
    *error = "cannot sync " + temp + ": " + std::strerror(errno);
    return false;
  }
  if (std::rename(temp.c_str(), path.c_str()) != 0) {
    *error = "cannot replace " + path + ": " + std::strerror(errno);
    return false;
  }
  const int folder_fd = ::open(folder.c_str(), O_RDONLY | O_DIRECTORY);
  if (folder_fd >= 0) {
    ::fsync(folder_fd);
    ::close(folder_fd);
  }
  return true;
}

MapDatum ToDatum(uint8_t state) {
  using mowbite_msgs::msg::MapStatus;
  switch (state) {
    case MapStatus::DATUM_EMPTY: return MapDatum::kEmpty;
    case MapStatus::DATUM_MATCH: return MapDatum::kMatch;
    case MapStatus::DATUM_MISMATCH: return MapDatum::kMismatch;
    case MapStatus::DATUM_UNBOUND: return MapDatum::kUnbound;
    default: return MapDatum::kUnknown;
  }
}

geometry_msgs::msg::Polygon ToPolygon(const Ring& ring) {
  geometry_msgs::msg::Polygon polygon;
  for (const auto& p : ring) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(p.x);
    point.y = static_cast<float>(p.y);
    polygon.points.push_back(point);
  }
  return polygon;
}

rcl_interfaces::msg::Parameter IntegerParameter(const std::string& name, int64_t value) {
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = name;
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  parameter.value.integer_value = value;
  return parameter;
}

rcl_interfaces::msg::Parameter DoubleParameter(const std::string& name, double value) {
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = name;
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  parameter.value.double_value = value;
  return parameter;
}

geometry_msgs::msg::Pose PoseAt(double x, double y, double heading) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(heading / 2);
  pose.orientation.w = std::cos(heading / 2);
  return pose;
}

double Yaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// How a drive of the follower ended, for the job: a bad position goes on by itself, the rest waits for continue.
LegEnd ToLegEnd(uint8_t reason) {
  using Result = mowbite_msgs::action::FollowPath::Result;
  switch (reason) {
    case Result::REASON_DONE: return LegEnd::kDone;
    case Result::REASON_CANCELED: return LegEnd::kCanceled;
    case Result::REASON_EMERGENCY: return LegEnd::kEmergency;
    case Result::REASON_POSE_LOST:
    case Result::REASON_POSITION_ACCURACY:
    case Result::REASON_HEADING: return LegEnd::kGps;
    default: return LegEnd::kFailed;
  }
}

// The name of a result of the follower, as in FollowPath.action.
std::string FollowerCode(uint8_t reason) {
  using Result = mowbite_msgs::action::FollowPath::Result;
  switch (reason) {
    case Result::REASON_DONE: return "DONE";
    case Result::REASON_CANCELED: return "CANCELED";
    case Result::REASON_INVALID: return "INVALID";
    case Result::REASON_NOT_ALLOWED: return "NOT_ALLOWED";
    case Result::REASON_EMERGENCY: return "EMERGENCY";
    case Result::REASON_POSE_LOST: return "POSE_LOST";
    case Result::REASON_POSITION_ACCURACY: return "POSITION_ACCURACY";
    case Result::REASON_HEADING: return "HEADING";
    case Result::REASON_CROSS_TRACK: return "CROSS_TRACK";
    case Result::REASON_OUTSIDE: return "OUTSIDE";
    case Result::REASON_TIMEOUT: return "TIMEOUT";
    default: return "INVALID";
  }
}

Reason PlannerMissing() {
  return Reason("PLANNER_MISSING", "The planner is not running");
}

Reason FollowerMissing() {
  return Reason("FOLLOWER_MISSING", "The follower is not running");
}

HardwareSnapshot::Motor ToMotor(const mowbite_msgs::msg::MotorState& m) {
  HardwareSnapshot::Motor motor;
  motor.ok = m.ok;
  motor.rpm = m.rpm;
  motor.current = m.current;
  motor.temperature = m.temperature;
  return motor;
}

}  // namespace

class LogicNode : public rclcpp::Node {
  using FollowPath = mowbite_msgs::action::FollowPath;
  using FollowHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  // What the follower drives for: logic/drive_to, the approach to a path of the job, that path, out of the station, or
  // a leg of docking.
  enum Purpose { kDriveTo = 0, kApproach = 1, kMow = 2, kUndock = 3, kDock = 4 };

 public:
  explicit LogicNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_logic", options), core_(LoadConfig()), geometry_(FactoryGeometry()) {
    hw_timeout_s_ = declare_parameter("hw_status_timeout", 2.0);
    pose_timeout_s_ = declare_parameter("pose_timeout", 2.0);
    // mowbite_map answers at once; a recording waits this long (s) before it counts as not saved.
    map_edit_timeout_s_ = declare_parameter("map_edit_timeout", 5.0);
    // The checkpoint of the mowing job and the settings of params.set; empty: none, a restart starts over with
    // tron.yaml.
    data_dir_ = declare_parameter("data_dir", std::string());
    // system.hold of the updater; the drivers read the same file themselves.
    hold_path_ = data_dir_.empty() ? "" : data_dir_ + "/hold.json";
    RestoreHold();
    LoadLogicParams();
    LoadGeometryEntries();
    // The blade: a lease of blade_lease (s), renewed every blade_renew (s) while the logic wants it to turn.
    blade_lease_s_ = declare_parameter("blade_lease", 1.0);
    blade_renew_s_ = declare_parameter("blade_renew", 0.5);
    // G22: no sign of the follower (goal taken, feedback every 0.5 s) for this long (s) while it has a path: the path
    // ends here, without waiting for the follower, and the mower and the blade stop.
    follower_timeout_s_ = declare_parameter("follower_timeout", 2.0);
    blade_client_ = create_client<mowbite_msgs::srv::SetBlade>("hw/blade");
    RestoreCheckpoint();

    robot_state_pub_ = create_publisher<mowbite_msgs::msg::RobotState>("bridge/robot_state", 10);
    actions_pub_ = create_publisher<mowbite_msgs::msg::ActionList>("bridge/actions", LatchedQos(50));
    sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorSpec>("bridge/sensor_info", LatchedQos(100));
    sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("bridge/sensor_value", 100);
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);
    // Every drive command goes to mowbite_safety (S2), the only writer of hw/cmd_vel, with its source: teleop while
    // recording, the follower while driving (LogicCore decides), stops of the logic.
    drive_pub_ = create_publisher<mowbite_msgs::msg::DriveCommand>("safety/drive", 10);

    // The app container rates the battery by /ll/board; any value except YardForce makes it show battery_percentage.
    params_pub_ = create_publisher<std_msgs::msg::String>(
        "bridge/params", rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local());
    std_msgs::msg::String board_msg;
    board_msg.data = nlohmann::json{{"node", "/ll"}, {"params", {{"board", declare_parameter("board", "Tron")}}}}.dump();
    params_pub_->publish(board_msg);
    // Publishers of their own, so a run of params.set never pushes /ll/board out of the history above.
    logic_params_pub_ = create_publisher<std_msgs::msg::String>(
        "bridge/params", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    // The dimensions of the mower for the planner and the localization, latched: whoever starts later gets the last.
    geometry_pub_ = create_publisher<std_msgs::msg::String>(
        "bridge/params", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    gps_limits_pub_ =
        create_publisher<mowbite_msgs::msg::GpsLimits>("logic/gps_limits", rclcpp::QoS(1).reliable().transient_local());
    // Settings of other nodes (the NTRIP access) change only while this is true, like params.set.
    idle_pub_ = create_publisher<std_msgs::msg::Bool>("logic/idle", rclcpp::QoS(1).reliable().transient_local());
    // The hold of an update for nodes that refuse changes of data in it (map, NTRIP access): until in unix s, 0 none.
    hold_pub_ = create_publisher<std_msgs::msg::Int64>("logic/hold", LatchedQos(1));
    PublishLogicParams();
    PublishGeometry();
    // Releases go through mowbite_safety, which keeps the emergency while the mower is tilted.
    emergency_client_ = create_client<mowbite_msgs::srv::SetEmergency>("safety/release_emergency");
    add_area_client_ = create_client<mowbite_msgs::srv::AddArea>("map/add_area");
    set_dock_client_ = create_client<mowbite_msgs::srv::SetDock>("map/set_dock");
    // The reference check on the docking station (G24); without an answer in dock_check_timeout s nothing starts.
    set_heading_client_ = create_client<mowbite_msgs::srv::SetHeading>("localization/set_heading");
    dock_check_timeout_s_ = declare_parameter("dock_check_timeout", 2.0);
    // Charging off before undocking; without an answer in charge_timeout s the mower stays on the station.
    charge_client_ = create_client<mowbite_msgs::srv::SetCharge>("hw/charge");
    charge_timeout_s_ = declare_parameter("charge_timeout", 2.0);

    // mowing.plan: asks the planner process for the paths of an area, answers through the bridge.
    plan_timeout_s_ = declare_parameter("plan_timeout", 30.0);
    plan_client_ = create_client<mowbite_planner_interfaces::srv::PlanCoverage>("mowbite_planner/plan_coverage");
    rpc_response_pub_ = create_publisher<mowbite_msgs::msg::RpcResponse>("bridge/rpc/response", 100);
    rpc_error_pub_ = create_publisher<mowbite_msgs::msg::RpcError>("bridge/rpc/error", 100);
    rpc_methods_pub_ = create_publisher<mowbite_msgs::msg::RpcMethods>("bridge/rpc/methods", LatchedQos(50));
    rpc_sub_ = create_subscription<mowbite_msgs::msg::RpcRequest>(
        "bridge/rpc/request", 100, [this](mowbite_msgs::msg::RpcRequest::ConstSharedPtr msg) { OnRpcRequest(*msg); });
    map_sub_ = create_subscription<std_msgs::msg::String>(
        "map/json", LatchedQos(1), [this](std_msgs::msg::String::ConstSharedPtr msg) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            planner_.SetMap(msg->data);
            core_.OnMowMap(planner_.MowAreas(), planner_.MapProblem());
            core_.OnMapDock(planner_.Dock());
            UpdateDrivableAreas();
          }
          Flush();
        });
    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_logic";
    methods.methods = {"mowing.plan", "params.set", "robot.geometry.get", "robot.geometry.set", "system.hold"};
    // "Home" from IDLE, only where the OS docks (contacts at the back).
    if (core_.CanDock()) methods.methods.push_back("docking.start");
    rpc_methods_pub_->publish(methods);

    hw_sub_ = create_subscription<mowbite_msgs::msg::HardwareStatus>(
        "hw/status", 10, [this](mowbite_msgs::msg::HardwareStatus::ConstSharedPtr msg) { OnHardwareStatus(*msg); });
    pose_sub_ = create_subscription<mowbite_msgs::msg::RobotPose>(
        "localization/pose", 10, [this](mowbite_msgs::msg::RobotPose::ConstSharedPtr msg) { OnPose(*msg); });
    // RTK fixed positions of the antenna for the course probe of undocking.
    gnss_sub_ = create_subscription<mowbite_msgs::msg::GnssFix>(
        "gps/absolute_pose", 10, [this](mowbite_msgs::msg::GnssFix::ConstSharedPtr msg) {
          if (msg->solution != mowbite_msgs::msg::GnssFix::RTK_FIXED) return;
          bool undocking;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            undocking = core_.Undocking();
            core_.OnGnssFix(msg->x, msg->y);
          }
          if (undocking) Flush();
        });
    jump_sub_ = create_subscription<mowbite_msgs::msg::PositionJump>(
        "localization/jump", 10, [this](mowbite_msgs::msg::PositionJump::ConstSharedPtr msg) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            core_.OnPositionJump(msg->after.x, msg->after.y, msg->distance,
                                 msg->reason == mowbite_msgs::msg::PositionJump::REASON_OUTLIERS);
          }
          Flush();
        });
    map_status_sub_ = create_subscription<mowbite_msgs::msg::MapStatus>(
        "map/status", LatchedQos(1), [this](mowbite_msgs::msg::MapStatus::ConstSharedPtr msg) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            core_.OnMapStatus(ToDatum(msg->datum_state), msg->reason);
            planner_.SetMapStatus(msg->mowing_allowed,
                                  msg->mowing_allowed ? Reason() : MapReason(ToDatum(msg->datum_state), msg->reason));
            core_.OnMowMap(planner_.MowAreas(), planner_.MapProblem());
            UpdateDrivableAreas();
          }
          Flush();
        });
    action_sub_ = create_subscription<std_msgs::msg::String>(
        "bridge/action", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) { OnAction(msg->data); });
    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "bridge/teleop", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { OnTeleop(*msg); });
    follower_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "follower/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          bool active;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            active = core_.FollowerActive();
          }
          if (active) SendDrive(mowbite_msgs::msg::DriveCommand::SOURCE_FOLLOWER, *msg);
        });
    caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
        "hw/capabilities", LatchedQos(1), [this](mowbite_msgs::msg::HardwareCapabilities::ConstSharedPtr msg) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            core_.OnCapabilities(!msg->safety_measurement_date.empty(), msg->charge_stop);
            blade_lease_max_s_ = msg->blade_lease_max_s;
          }
          Flush();
        });

    // Driving to a pose: the planner plans the way, the follower drives it. A ROS service only, not for the app yet.
    transit_max_outside_ = declare_parameter("transit_max_outside", 0.3);
    transit_client_ = create_client<mowbite_planner_interfaces::srv::PlanTransit>("mowbite_planner/plan_transit");
    follow_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
    drive_srv_ = create_service<mowbite_msgs::srv::DriveTo>(
        "logic/drive_to", [this](std::shared_ptr<rmw_request_id_t> header,
                                 std::shared_ptr<mowbite_msgs::srv::DriveTo::Request> request) {
          OnDriveTo(header, *request);
        });
    stop_drive_srv_ = create_service<std_srvs::srv::Trigger>(
        "logic/stop_drive", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            // Leaving the station for a drive is part of the drive.
            const bool undocking = core_.UndockingForDrive();
            response->success = undocking || (core_.Driving() && drive_.purpose == Purpose::kDriveTo);
            response->message = response->success ? "stopping" : "not driving";
            if (undocking) {
              core_.AbortUndocking(Reason("STOPPED", "Stopped by logic/stop_drive"));
            } else if (response->success) {
              CancelDrive(Reason("STOPPED", "Stopped by logic/stop_drive"));
            }
          }
          Flush();
        });

    for (const auto& sensor : core_.Sensors()) {
      mowbite_msgs::msg::SensorSpec spec;
      spec.id = sensor.id;
      spec.name = sensor.name;
      spec.kind = sensor.text ? mowbite_msgs::msg::SensorSpec::KIND_TEXT : mowbite_msgs::msg::SensorSpec::KIND_NUMBER;
      spec.meaning = static_cast<uint8_t>(sensor.meaning);
      spec.unit = sensor.unit;
      spec.min = sensor.min;
      spec.max = sensor.max;
      spec.critical_low = sensor.critical_low;
      spec.critical_high = sensor.critical_high;
      sensor_info_pub_->publish(spec);
    }

    state_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { OnStateTimer(); });
    sensor_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { PublishReadings(); });
    // At most twice a second: a long outline is a large message. Retained, so a late app sees the lines too.
    overlay_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { PublishOverlay(); });
    plan_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      ExpirePlans();
      Flush();
    });
    blade_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { UpdateBlade(); });
    PublishActionsIfChanged();
    PublishIdleIfChanged();
    PublishHoldIfChanged();
  }

 private:
  LogicConfig LoadConfig() {
    LogicConfig config;
    config.battery_empty_voltage = declare_parameter("battery_empty_voltage", config.battery_empty_voltage);
    config.battery_full_voltage = declare_parameter("battery_full_voltage", config.battery_full_voltage);
    config.dock_charge_voltage = declare_parameter("dock_charge_voltage", config.dock_charge_voltage);
    config.max_position_accuracy = declare_parameter("max_position_accuracy", config.max_position_accuracy);
    config.recorder.min_area = declare_parameter("recording_min_area", config.recorder.min_area);
    config.recorder.max_points =
        static_cast<size_t>(declare_parameter("recording_max_points", static_cast<int64_t>(config.recorder.max_points)));
    config.recorder.min_dock_distance = declare_parameter("recording_min_dock_distance", config.recorder.min_dock_distance);
    config.mow_with_blade = declare_parameter("mow_with_blade", config.mow_with_blade);
    config.mower_spinup_rpm =
        static_cast<int>(declare_parameter("mower_spinup_rpm", static_cast<int64_t>(config.mower_spinup_rpm)));
    config.mower_spinup_timeout = declare_parameter("mower_spinup_timeout", config.mower_spinup_timeout);
    config.blade_running_rpm =
        static_cast<int>(declare_parameter("blade_running_rpm", static_cast<int64_t>(config.blade_running_rpm)));
    config.gps_wait_time = declare_parameter("gps_wait_time", config.gps_wait_time);
    config.battery_critical_voltage = declare_parameter("battery_critical_voltage", config.battery_critical_voltage);
    config.battery_critical_time = declare_parameter("battery_critical_time", config.battery_critical_time);
    config.approach_distance = declare_parameter("approach_distance", config.approach_distance);
    config.checkpoint_period = declare_parameter("checkpoint_period", config.checkpoint_period);
    // Where the charging contacts are belongs to the dimensions of the platform (geometry.dock_contact_x).
    config.dock_contact_x = GeometryNumber("dock_contact_x");
    config.dock_max_position_accuracy = declare_parameter("dock_max_position_accuracy", config.dock_max_position_accuracy);
    config.dock_reference_limit = declare_parameter("dock_reference_limit", config.dock_reference_limit);
    // undock_distance is a setting of params.set (LoadLogicParams), the rest is the platform's.
    config.undock.distance = declare_parameter("undock_distance", config.undock.distance);
    config.undock.speed = declare_parameter("undock_speed", config.undock.speed);
    config.undock.stall_time = declare_parameter("undock_stall_time", config.undock.stall_time);
    config.undock.course_limit =
        declare_parameter("undock_course_limit", config.undock.course_limit * 180.0 / M_PI) * M_PI / 180.0;
    // docking_approach_distance and docking_retry_count are settings of params.set (LoadLogicParams).
    config.dock.approach_distance = declare_parameter("docking_approach_distance", config.dock.approach_distance);
    config.dock.retry_count = static_cast<int>(
        declare_parameter("docking_retry_count", static_cast<int64_t>(config.dock.retry_count)));
    config.dock.speed = declare_parameter("docking_speed", config.dock.speed);
    config.dock.contact_speed = declare_parameter("contact_speed", config.dock.contact_speed);
    config.dock.gate_distance = declare_parameter("dock_gate_distance", config.dock.gate_distance);
    config.dock.gate_lateral = declare_parameter("dock_gate_lateral", config.dock.gate_lateral);
    config.dock.gate_heading =
        declare_parameter("dock_gate_heading", config.dock.gate_heading * 180.0 / M_PI) * M_PI / 180.0;
    config.dock.overshoot = declare_parameter("dock_overshoot", config.dock.overshoot);
    config.dock.contact_timeout = declare_parameter("contact_timeout", config.dock.contact_timeout);
    return config;
  }

  void OnHardwareStatus(const mowbite_msgs::msg::HardwareStatus& msg) {
    HardwareSnapshot hw;
    hw.telemetry_ok = msg.telemetry_ok;
    hw.emergency = msg.emergency;
    hw.emergency_reason = static_cast<int>(msg.emergency_reason);
    hw.rain = msg.rain;
    hw.dock_contact = msg.dock_contact;
    hw.dock_contact_raw = msg.dock_contact_raw;
    hw.hold = msg.hold;
    hw.battery_ok = msg.battery_source != mowbite_msgs::msg::HardwareStatus::BATTERY_SOURCE_NONE;
    hw.battery_voltage = msg.battery_voltage;
    hw.battery_current = msg.battery_current;
    hw.battery_percentage = msg.battery_percentage;
    hw.battery_temperature = msg.battery_temperature;
    hw.blade = ToMotor(msg.blade_motor);
    hw.left = ToMotor(msg.left_motor);
    hw.right = ToMotor(msg.right_motor);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      core_.OnHardwareStatus(hw);
      last_hw_ = MonotonicSeconds();
    }
    Flush();
  }

  void OnPose(const mowbite_msgs::msg::RobotPose& msg) {
    PoseSnapshot pose;
    pose.x = msg.x;
    pose.y = msg.y;
    pose.heading = msg.heading;
    pose.position_accuracy = msg.position_accuracy;
    pose.orientation_accuracy = msg.heading_accuracy;
    pose.orientation_valid = msg.heading_valid;
    std::lock_guard<std::mutex> lock(mutex_);
    core_.OnPose(pose);
    last_pose_ = MonotonicSeconds();
  }

  void OnAction(const std::string& action_id) {
    ActionResult result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Reason missing;
      if (action_id == "mower_logic:idle/start_mowing" && core_.StartProblem().empty()) {
        if (!plan_client_->service_is_ready() || !transit_client_->service_is_ready()) missing = PlannerMissing();
        if (!follow_client_->action_server_is_ready()) missing = FollowerMissing();
        // The node still ends a leg of the last job.
        if (drive_.active || job_plan_.active) {
          missing = Reason("STILL_STOPPING", "The last job is still stopping, try again in a moment");
        }
      }
      if (!missing.empty()) {
        core_.RefuseStart(missing);
        result = ActionResult::kRejected;
      } else {
        result = core_.OnAction(action_id);
      }
    }
    switch (result) {
      case ActionResult::kReleaseEmergency: ReleaseEmergency(); break;
      case ActionResult::kRejected: RCLCPP_WARN(get_logger(), "Action %s not possible now", action_id.c_str()); break;
      case ActionResult::kUnknown: RCLCPP_DEBUG(get_logger(), "Ignoring action %s", action_id.c_str()); break;
      case ActionResult::kDone: RCLCPP_INFO(get_logger(), "Action %s", action_id.c_str()); break;
    }
    Flush();
  }

  // The answer comes from S2 once the driver answered; a refusal goes to the app as EMERGENCY_RESET_REFUSED.
  void ReleaseEmergency() {
    if (!emergency_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "safety/release_emergency is not available, emergency stays");
      {
        std::lock_guard<std::mutex> lock(mutex_);
        core_.RefuseReset(Reason("SAFETY_MISSING", "safety/release_emergency is not available"));
      }
      Flush();
      return;
    }
    auto request = std::make_shared<mowbite_msgs::srv::SetEmergency::Request>();
    request->latch = false;
    request->reason = "reset_emergency from the app";
    std::lock_guard<std::mutex> lock(mutex_);
    // A second reset while one is pending replaces it; only the newest answer counts.
    if (release_sent_) emergency_client_->remove_pending_request(release_request_id_);
    release_sent_ = MonotonicSeconds();
    release_request_id_ = emergency_client_->async_send_request(
        request, [this](rclcpp::Client<mowbite_msgs::srv::SetEmergency>::SharedFuture future) {
          const auto response = future.get();
          {
            std::lock_guard<std::mutex> lock(mutex_);
            release_sent_.reset();
            if (response->success) return;
            RCLCPP_WARN(get_logger(), "Emergency stays (%s): %s", response->code.c_str(), response->message.c_str());
            core_.RefuseReset(ResetRefusal(response->code, response->message, response->value, response->limit));
          }
          Flush();
        }).request_id;
    RCLCPP_INFO(get_logger(), "Emergency release requested");
  }

  void SendDrive(uint8_t source, const geometry_msgs::msg::Twist& twist) {
    mowbite_msgs::msg::DriveCommand msg;
    msg.source = source;
    msg.twist = twist;
    drive_pub_->publish(msg);
  }

  // A zero command at once instead of waiting for the cmd_vel watchdog of the driver.
  void SendStop() {
    SendDrive(mowbite_msgs::msg::DriveCommand::SOURCE_STOP, geometry_msgs::msg::Twist());
  }

  void OnTeleop(const geometry_msgs::msg::Twist& msg) {
    std::optional<std::pair<double, double>> cmd;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = core_.OnTeleop(msg.linear.x, msg.angular.z, MonotonicSeconds());
    }
    if (!cmd) return;
    geometry_msgs::msg::Twist out;
    out.linear.x = cmd->first;
    out.angular.z = cmd->second;
    SendDrive(mowbite_msgs::msg::DriveCommand::SOURCE_TELEOP, out);
  }

  void OnDriveTo(const std::shared_ptr<rmw_request_id_t>& header, const mowbite_msgs::srv::DriveTo::Request& request) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::vector<Ring> drivable;
      std::vector<Ring> obstacles;
      // What the core checks first (busy, emergency, position), then what only the node knows.
      Reason why = core_.DriveProblem();
      if (why.empty()) why = planner_.TransitAreas(&drivable, &obstacles);
      if (why.empty() && !transit_client_->service_is_ready()) why = PlannerMissing();
      if (why.empty() && !follow_client_->action_server_is_ready()) why = FollowerMissing();
      // The follower may still be letting go of the last path (right after docking): a new one would be refused.
      if (why.empty() && (drive_.active || job_plan_.active)) {
        why = Reason("STILL_STOPPING", "The last drive is still stopping, try again in a moment");
      }
      const bool from_dock = why.empty() && core_.DockContact();
      if (!why.empty()) {
        core_.RefuseDrive(why);
      } else if (from_dock) {
        // From the docking station the reference check and undocking come first; the request is answered once the
        // drive is on its way after them, or with why not.
        core_.CheckDockBeforeDrive(request.x, request.y);
        dock_drive_ = DockDrive{header, request};
        RCLCPP_INFO(get_logger(), "Drive to (%.2f, %.2f) from the docking station: checking the position first",
                    request.x, request.y);
      } else {
        why = core_.StartDrive(request.x, request.y);
      }
      if (!why.empty()) {
        RCLCPP_WARN(get_logger(), "Drive refused: %s (%s)", why.code.c_str(), why.message.c_str());
        RespondDrive(header, false, why.message);
      } else if (!from_dock) {
        PlanDrive(request, header);
      }
    }
    Flush();
  }

  // Plans the way of the drive the core has started (state DRIVING). Called with the mutex held.
  void PlanDrive(const mowbite_msgs::srv::DriveTo::Request& request, const std::shared_ptr<rmw_request_id_t>& header) {
    std::vector<Ring> drivable;
    std::vector<Ring> obstacles;
    Reason why = planner_.TransitAreas(&drivable, &obstacles);
    if (why.empty() && !transit_client_->service_is_ready()) why = PlannerMissing();
    if (!why.empty()) {
      RCLCPP_WARN(get_logger(), "Drive ended: %s (%s)", why.code.c_str(), why.message.c_str());
      RespondDrive(header, false, why.message);
      core_.OnDriveEnded(false, why);
      return;
    }
    const RobotStateData state = core_.RobotState();
    auto plan = std::make_shared<mowbite_planner_interfaces::srv::PlanTransit::Request>();
    plan->start = PoseAt(state.pose_x, state.pose_y, state.pose_heading);
    plan->goal = PoseAt(request.x, request.y, request.heading);
    plan->use_goal_heading = request.use_heading;
    for (const auto& ring : drivable) plan->drivable_areas.push_back(ToPolygon(ring));
    for (const auto& ring : obstacles) plan->obstacles.push_back(ToPolygon(ring));
    plan->max_outside = transit_max_outside_;
    drive_ = Drive();
    drive_.purpose = Purpose::kDriveTo;
    drive_.seq = ++drive_seq_;
    drive_.active = true;
    drive_.header = header;
    drive_.planning = true;
    drive_.sent = MonotonicSeconds();
    drive_.allowed = plan->drivable_areas;
    drive_.obstacles = plan->obstacles;
    const uint64_t seq = drive_.seq;
    drive_.request_id =
        transit_client_
            ->async_send_request(plan,
                                 [this, seq](rclcpp::Client<mowbite_planner_interfaces::srv::PlanTransit>::SharedFuture f) {
                                   OnTransit(seq, *f.get());
                                 })
            .request_id;
    RCLCPP_INFO(get_logger(), "Drive to (%.2f, %.2f): planning the way", request.x, request.y);
  }

  // The drivable areas of the map for the core, which lets undocking end and docking begin only in one of them. Called
  // with the mutex held.
  void UpdateDrivableAreas() {
    std::vector<Ring> drivable;
    std::vector<Ring> obstacles;
    const Reason problem = planner_.TransitAreas(&drivable, &obstacles);
    if (!problem.empty()) {
      drivable.clear();
      obstacles.clear();
    }
    core_.OnDrivableAreas(std::move(drivable), std::move(obstacles), problem);
  }

  void OnTransit(uint64_t seq, const mowbite_planner_interfaces::srv::PlanTransit::Response& response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (seq != drive_.seq || !drive_.planning) return;
      drive_.planning = false;
      if (!response.success) {
        EndDrive(false, Reason("TRANSIT_FAILED", "Transit planning failed: " + response.message),
                 FollowPath::Result::REASON_INVALID, 0);
      } else if (response.path.poses.empty()) {
        // Already there; the planner says so in its message.
        RespondDrive(drive_.header, true, "already there: " + response.message);
        drive_.header.reset();
        EndDrive(true, Reason("ALREADY_THERE", "Already there"), FollowPath::Result::REASON_DONE, 0);
      } else {
        FollowPath::Goal goal;
        for (const auto& pose : response.path.poses) goal.poses.push_back(pose.pose);
        goal.allowed_areas = drive_.allowed;
        goal.obstacles = drive_.obstacles;
        char text[64];
        std::snprintf(text, sizeof(text), "driving %.1f m", response.path_length);
        SendFollow(goal, text);
      }
    }
    Flush();
  }

  // Hands a path to the follower for the current drive. Called with the mutex held.
  void SendFollow(const FollowPath::Goal& goal, const std::string& what) {
    const uint64_t seq = drive_.seq;
    typename rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback = [this, seq, what](const FollowHandle::SharedPtr& handle) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (seq != drive_.seq) return;
        if (!handle) {
          EndDrive(false, Reason("FOLLOWER_REFUSED", "The follower did not take the path"),
                   FollowPath::Result::REASON_INVALID, 0);
        } else {
          drive_.handle = handle;
          drive_.follower_sign = MonotonicSeconds();
          RCLCPP_INFO(get_logger(), "%s: %s", PurposeName(drive_.purpose), what.c_str());
          RespondDrive(drive_.header, true, what);
          drive_.header.reset();
          // An emergency or a pause while the goal was on its way.
          if (drive_.cancel_pending) follow_client_->async_cancel_goal(handle);
        }
      }
      Flush();
    };
    options.feedback_callback = [this, seq](FollowHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (seq != drive_.seq) return;
      drive_.follower_sign = MonotonicSeconds();
      drive_.follower_index = feedback->index;
      if (drive_.purpose != Purpose::kMow) return;
      core_.OnJobProgress(drive_.PathIndex(feedback->index));
    };
    options.result_callback = [this, seq](const FollowHandle::WrappedResult& wrapped) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (seq != drive_.seq) return;
        const auto& result = wrapped.result;
        uint8_t reason = result ? result->reason : FollowPath::Result::REASON_CANCELED;
        uint32_t index = result ? result->index : drive_.follower_index;
        std::string message = result ? result->message : "No result from the follower";
        // A goal the follower dropped without ending it (gone, or another goal took its place) comes back canceled
        // with an empty result: never a path done.
        if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED && reason == FollowPath::Result::REASON_DONE) {
          reason = FollowPath::Result::REASON_CANCELED;
          index = drive_.follower_index;
          message = "no result from the follower";
        }
        Reason why(FollowerCode(reason), message);
        if (reason == FollowPath::Result::REASON_CANCELED && drive_.cancel_reason) {
          // Stopped from here: the cause, not the cancel of the follower.
          why = Reason(*drive_.cancel_reason).WithMessage(drive_.cancel_reason->message + " (" + message + ")");
        } else if (reason == FollowPath::Result::REASON_CANCELED) {
          // A cancel the logic did not ask for (G22): the path stops for the user, as when the follower does not answer.
          why = Reason("FOLLOWER_MISSING", "The follower dropped the path (" + message + ")");
          reason = FollowPath::Result::REASON_TIMEOUT;
        }
        EndDrive(reason == FollowPath::Result::REASON_DONE, why, reason, index);
      }
      Flush();
    };
    drive_.follower_sign = MonotonicSeconds();
    drive_.follower_index = goal.start_index;
    follow_client_->async_send_goal(goal, options);
  }

  // G22: a follower that stops answering ends its path here, without waiting for its result, so that the logic does
  // not keep the blade turning on the spot. True when it ended one. Called with the mutex held.
  bool CheckFollowerLocked() {
    if (!drive_.active || drive_.planning || drive_.follower_sign <= 0) return false;
    const double quiet = MonotonicSeconds() - drive_.follower_sign;
    if (quiet <= follower_timeout_s_) return false;
    char text[96];
    std::snprintf(text, sizeof(text), "No answer from the follower for %.1f s", quiet);
    // Stopped from here already: the cause stays, the follower only never confirmed it.
    const bool canceled = drive_.cancel_reason.has_value();
    Reason why = Reason("FOLLOWER_MISSING", text).With("seconds", quiet).With("limit", follower_timeout_s_);
    if (canceled) why = Reason(*drive_.cancel_reason).WithMessage(drive_.cancel_reason->message + " (" + text + ")");
    try {
      if (drive_.handle) follow_client_->async_cancel_goal(drive_.handle);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError&) {
      // The client has dropped the goal already; there is nothing left to cancel.
    }
    EndDrive(false, why, canceled ? FollowPath::Result::REASON_CANCELED : FollowPath::Result::REASON_TIMEOUT,
             drive_.follower_index);
    return true;
  }

  // Stops the drive: cancels the follower, or drops the way still being planned. Called with the mutex held.
  void CancelDrive(const Reason& why) {
    if (!drive_.active) return;
    drive_.cancel_reason = why;
    if (drive_.planning) {
      transit_client_->remove_pending_request(drive_.request_id);
      drive_.planning = false;
      EndDrive(false, why, FollowPath::Result::REASON_CANCELED, 0);
    } else if (drive_.handle) {
      follow_client_->async_cancel_goal(drive_.handle);
    } else {
      drive_.cancel_pending = true;
    }
  }

  // Ends the drive, the approach or the path of the job; index is the pose of the follower's path it got to. Called
  // with the mutex held.
  void EndDrive(bool arrived, const Reason& why, uint8_t reason, uint32_t index) {
    if (drive_.header) RespondDrive(drive_.header, false, why.message);
    const char* name = PurposeName(drive_.purpose);
    if (arrived) {
      RCLCPP_INFO(get_logger(), "%s: done", name);
    } else {
      RCLCPP_WARN(get_logger(), "%s ended: %s (%s)", name, why.code.c_str(), why.message.c_str());
    }
    const Drive ended = drive_;
    drive_ = Drive();
    if (ended.purpose == Purpose::kDriveTo) {
      core_.OnDriveEnded(arrived, why);
      return;
    }
    if (ended.purpose == Purpose::kUndock) {
      core_.OnUndockEnded(ToLegEnd(reason), why);
      return;
    }
    if (ended.purpose == Purpose::kDock) {
      core_.OnDockingLegEnded(ToLegEnd(reason), why);
      return;
    }
    // In a job the way to the start of a path failed, or the follower did not take a path of the plan.
    Reason leg = why;
    const bool way = why.code == "TRANSIT_FAILED" || why.code == "PLANNER_TIMEOUT" || why.code == "FOLLOWER_REFUSED";
    if (way && ended.purpose == Purpose::kApproach) leg.code = "APPROACH_FAILED";
    if (way && ended.purpose == Purpose::kMow) leg.code = "INVALID";
    core_.OnLegEnded(ToLegEnd(reason), ended.PathIndex(index), leg);
  }

  static const char* PurposeName(int purpose) {
    switch (purpose) {
      case 1: return "Approach";
      case 2: return "Mow path";
      case 3: return "Undocking";
      case 4: return "Docking";
      default: return "Drive";
    }
  }

  // The next step of the job, when nothing is on its way for it. Called with the mutex held.
  void PumpJob() {
    if (drive_.active || job_plan_.active || !core_.Mowing()) return;
    JobStep step = core_.NextJobStep(MonotonicSeconds());
    switch (step.kind) {
      case JobStep::Kind::kNone: return;
      case JobStep::Kind::kPlan: PlanJobArea(step.area_id); return;
      case JobStep::Kind::kApproach: Approach(step.goal); return;
      case JobStep::Kind::kFollow: FollowJobPath(step.path, step.start_index); return;
    }
  }

  void PlanJobArea(const std::string& area_id) {
    const PlanQuestion question = planner_.Prepare(json{{"area_id", area_id}});
    if (!question.ok) {
      core_.OnJobPlanFailed(question.message);
      return;
    }
    if (!plan_client_->service_is_ready()) {
      core_.OnJobPlanFailed("The planner is not running");
      return;
    }
    RCLCPP_INFO(get_logger(), "Mowing: planning area %s", area_id.c_str());
    const uint64_t seq = ++plan_seq_;
    job_plan_.active = true;
    job_plan_.seq = seq;
    job_plan_.sent = MonotonicSeconds();
    job_plan_.request_id =
        plan_client_
            ->async_send_request(ToServiceRequest(question.request),
                                 [this, seq](rclcpp::Client<mowbite_planner_interfaces::srv::PlanCoverage>::SharedFuture f) {
                                   OnJobPlan(seq, *f.get());
                                 })
            .request_id;
  }

  void OnJobPlan(uint64_t seq, const mowbite_planner_interfaces::srv::PlanCoverage::Response& response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!job_plan_.active || seq != job_plan_.seq) return;
      job_plan_ = JobPlan();
      const CoverageAnswer answer = ToAnswer(response);
      if (!answer.success) {
        core_.OnJobPlanFailed(answer.message);
      } else {
        std::vector<JobPath> paths = JobPaths(answer);
        RCLCPP_INFO(get_logger(), "Mowing: %zu paths from %zu poses", paths.size(), answer.poses.size());
        core_.OnJobPlanned(std::move(paths), answer.fingerprint);
      }
    }
    Flush();
  }

  // Drives to the start of a path over the drivable areas, the blade off. Called with the mutex held.
  void Approach(const JobPose& goal) {
    Reason why = PlanWay(Purpose::kApproach, goal.x, goal.y, goal.heading);
    if (why.code == "PLANNER_MISSING") why = Reason("APPROACH_FAILED", why.message);
    if (!why.empty()) {
      core_.OnLegEnded(LegEnd::kFailed, 0, why);
      return;
    }
    RCLCPP_INFO(get_logger(), "Mowing: approaching (%.2f, %.2f)", goal.x, goal.y);
  }

  // Plans the way to (x, y) facing heading over the drivable areas, which the follower then drives (OnTransit): the
  // approach to a path of the job or to the docking station. Empty, or why not. Called with the mutex held.
  Reason PlanWay(Purpose purpose, double x, double y, double heading) {
    std::vector<Ring> drivable;
    std::vector<Ring> obstacles;
    const Reason why = planner_.TransitAreas(&drivable, &obstacles);
    if (!why.empty()) return why;
    if (!transit_client_->service_is_ready()) return PlannerMissing();
    const RobotStateData state = core_.RobotState();
    auto plan = std::make_shared<mowbite_planner_interfaces::srv::PlanTransit::Request>();
    plan->start = PoseAt(state.pose_x, state.pose_y, state.pose_heading);
    plan->goal = PoseAt(x, y, heading);
    plan->use_goal_heading = true;
    for (const auto& ring : drivable) plan->drivable_areas.push_back(ToPolygon(ring));
    for (const auto& ring : obstacles) plan->obstacles.push_back(ToPolygon(ring));
    plan->max_outside = transit_max_outside_;
    drive_ = Drive();
    drive_.purpose = purpose;
    drive_.seq = ++drive_seq_;
    drive_.active = true;
    drive_.planning = true;
    drive_.sent = MonotonicSeconds();
    drive_.allowed = plan->drivable_areas;
    drive_.obstacles = plan->obstacles;
    const uint64_t seq = drive_.seq;
    drive_.request_id =
        transit_client_
            ->async_send_request(plan,
                                 [this, seq](rclcpp::Client<mowbite_planner_interfaces::srv::PlanTransit>::SharedFuture f) {
                                   OnTransit(seq, *f.get());
                                 })
            .request_id;
    return {};
  }

  // The next leg of docking, once nothing else is on its way. Called with the mutex held.
  void PumpDocking() {
    if (drive_.active || job_plan_.active) return;
    if (auto leg = core_.TakeDockingLeg()) SendDockingLeg(*leg);
  }

  // The way to the docking station over the drivable areas, or a line for the follower: besides the drivable areas
  // base_link may be in the corridor along it. Called with the mutex held.
  void SendDockingLeg(const DockingLeg& leg) {
    if (leg.kind == DockingLeg::Kind::kApproach) {
      const Reason why = PlanWay(Purpose::kDock, leg.goal.x, leg.goal.y, leg.goal.heading);
      if (!why.empty()) {
        core_.OnDockingLegEnded(LegEnd::kFailed, why);
        return;
      }
      RCLCPP_INFO(get_logger(), "Docking: the way to (%.2f, %.2f) in front of the docking station", leg.goal.x,
                  leg.goal.y);
      return;
    }
    std::vector<Ring> drivable;
    std::vector<Ring> obstacles;
    planner_.TransitAreas(&drivable, &obstacles);
    FollowPath::Goal goal;
    for (const auto& p : leg.poses) goal.poses.push_back(PoseAt(p.x, p.y, p.heading));
    goal.start_index = 0;
    for (const auto& ring : drivable) goal.allowed_areas.push_back(ToPolygon(ring));
    goal.allowed_areas.push_back(ToPolygon(leg.corridor));
    for (const auto& ring : obstacles) goal.obstacles.push_back(ToPolygon(ring));
    goal.max_speed = static_cast<float>(leg.speed);
    goal.stall_time = static_cast<float>(leg.stall_time);
    goal.straight = leg.straight;
    goal.reverse = leg.reverse;
    goal.end_tolerance = static_cast<float>(leg.end_tolerance);
    drive_ = Drive();
    drive_.purpose = Purpose::kDock;
    drive_.seq = ++drive_seq_;
    drive_.active = true;
    drive_.sent = MonotonicSeconds();
    const auto& from = leg.poses.front();
    const auto& to = leg.poses.back();
    char text[128];
    std::snprintf(text, sizeof(text), "%s, %.2f m %s to (%.2f, %.2f) at %.2f m/s", DockingLegName(leg.kind),
                  std::hypot(to.x - from.x, to.y - from.y), leg.reverse ? "backwards" : "forwards", to.x, to.y,
                  leg.speed);
    SendFollow(goal, text);
  }

  // Hands a path of the plan to the follower from start_index on. Poses on the one before are left out, the follower
  // refuses them; feedback and result are counted back in poses of the path. Called with the mutex held.
  void FollowJobPath(const JobPath& path, size_t start_index) {
    std::vector<Ring> drivable;
    std::vector<Ring> obstacles;
    const Reason why = planner_.TransitAreas(&drivable, &obstacles);
    if (!why.empty()) {
      core_.OnLegEnded(LegEnd::kFailed, start_index, why);
      return;
    }
    drive_ = Drive();
    drive_.purpose = Purpose::kMow;
    FollowPath::Goal goal;
    goal.start_index = 0;
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const JobPose& p = path.poses[i];
      if (!goal.poses.empty() && std::hypot(p.x - goal.poses.back().position.x, p.y - goal.poses.back().position.y) < 1e-6) {
        continue;
      }
      if (i <= start_index) goal.start_index = static_cast<uint32_t>(goal.poses.size());
      goal.poses.push_back(PoseAt(p.x, p.y, p.heading));
      goal.kinds.push_back(p.kind);
      drive_.path_index.push_back(i);
    }
    // Nothing left to drive on this path.
    if (goal.poses.size() < 3 || goal.start_index + 1 >= goal.poses.size()) {
      drive_ = Drive();
      core_.OnLegEnded(LegEnd::kDone, path.poses.empty() ? 0 : path.poses.size() - 1, Reason("DONE", ""));
      return;
    }
    for (const auto& ring : drivable) goal.allowed_areas.push_back(ToPolygon(ring));
    for (const auto& ring : obstacles) goal.obstacles.push_back(ToPolygon(ring));
    drive_.seq = ++drive_seq_;
    drive_.active = true;
    drive_.sent = MonotonicSeconds();
    char text[64];
    std::snprintf(text, sizeof(text), "%zu poses from %zu", path.poses.size(), start_index);
    SendFollow(goal, text);
  }

  // Every 100 ms: first the follower's sign of life (G22), then the lease of hw/blade, kept alive while the logic
  // wants the blade and switched off at once when not.
  void UpdateBlade() {
    bool stalled;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stalled = CheckFollowerLocked();
      UpdateBladeLocked();
    }
    if (!stalled) return;
    // One zero command: the follower sends none any more, the driver's watchdog would stop the wheels only later.
    SendStop();
    Flush();
  }

  void UpdateBladeLocked() {
    const bool wanted = core_.BladeWanted();
    const double t = MonotonicSeconds();
    if (!wanted) {
      if (!blade_on_requested_) return;
      blade_on_requested_ = false;
      SendBlade(false);
      return;
    }
    if (blade_on_requested_ && t - blade_sent_s_ < blade_renew_s_) return;
    if (!blade_client_->service_is_ready()) {
      core_.OnBladeRefused("hw/blade is not available");
      return;
    }
    blade_on_requested_ = true;
    blade_sent_s_ = t;
    SendBlade(true);
  }

  // Called with the mutex held.
  void SendBlade(bool on) {
    auto request = std::make_shared<mowbite_msgs::srv::SetBlade::Request>();
    request->on = on;
    double lease = blade_lease_s_;
    if (blade_lease_max_s_ > 0) lease = std::min(lease, static_cast<double>(blade_lease_max_s_));
    request->lease_s = static_cast<float>(lease);
    blade_client_->async_send_request(request, [this, on](rclcpp::Client<mowbite_msgs::srv::SetBlade>::SharedFuture f) {
      const auto response = f.get();
      if (response->success) return;
      RCLCPP_WARN(get_logger(), "hw/blade refused %s: %s (reason bits 0x%x)", on ? "on" : "off",
                  response->message.c_str(), response->emergency_reason);
      if (!on) return;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        blade_on_requested_ = false;
        core_.OnBladeRefused(response->message, static_cast<int>(response->emergency_reason));
      }
      Flush();
    });
  }

  // <data_dir>/checkpoint.json, written to a new file and renamed, so a crash never leaves half a checkpoint.
  std::string CheckpointPath() const {
    return data_dir_.empty() ? "" : data_dir_ + "/checkpoint.json";
  }

  void RestoreCheckpoint() {
    const std::string path = CheckpointPath();
    if (path.empty()) return;
    std::ifstream in(path);
    if (!in) return;
    std::stringstream text;
    text << in.rdbuf();
    const auto progress = ParseCheckpoint(text.str());
    if (!progress) {
      RCLCPP_WARN(get_logger(), "%s is not a checkpoint, starting over", path.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Checkpoint: job %s, area %d, path %d, index %d", progress->job_id.c_str(), progress->area,
                progress->path, progress->index);
    core_.RestoreJob(*progress);
  }

  void SaveCheckpoint(const std::string& text) {
    const std::string path = CheckpointPath();
    if (path.empty()) return;
    std::string error;
    if (!WriteFileSafely(path, text, &error)) RCLCPP_ERROR(get_logger(), "Checkpoint: %s", error.c_str());
  }

  void RespondDrive(const std::shared_ptr<rmw_request_id_t>& header, bool success, const std::string& message) {
    if (!header) return;
    mowbite_msgs::srv::DriveTo::Response response;
    response.success = success;
    response.message = message;
    drive_srv_->send_response(*header, response);
  }

  void OnStateTimer() {
    RobotStateData state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double t = MonotonicSeconds();
      if (!last_hw_ || t - *last_hw_ > hw_timeout_s_) core_.OnHardwareTimeout();
      if (!last_pose_ || t - *last_pose_ > pose_timeout_s_) core_.OnPoseTimeout();
      core_.OnTick(t);
      if (edit_sent_ && t - *edit_sent_ > map_edit_timeout_s_) {
        edit_sent_.reset();
        if (edit_request_id_) {
          if (edit_is_dock_) {
            set_dock_client_->remove_pending_request(*edit_request_id_);
          } else {
            add_area_client_->remove_pending_request(*edit_request_id_);
          }
          edit_request_id_.reset();
        }
        core_.OnMapEditResult(false, "", "NO_ANSWER", "No answer from mowbite_map; look at the map before saving again");
      }
      if (dock_check_sent_ && t - *dock_check_sent_ > dock_check_timeout_s_) {
        set_heading_client_->remove_pending_request(dock_check_request_id_);
        FinishDockCheck(false, 0, 0, 999.0f, "No answer from the localization (localization/set_heading)");
      }
      if (release_sent_ && t - *release_sent_ > kReleaseTimeoutS) {
        emergency_client_->remove_pending_request(release_request_id_);
        release_sent_.reset();
        RCLCPP_WARN(get_logger(), "Emergency stays: no answer from safety/release_emergency");
        core_.RefuseReset(Reason("NO_ANSWER", "No answer to the release from mowbite_safety"));
      }
      if (charge_sent_ && t - *charge_sent_ > charge_timeout_s_) {
        charge_client_->remove_pending_request(charge_request_id_);
        charge_sent_.reset();
        core_.OnChargeOff(false, "no answer from hw/charge");
      }
      ExpireHoldLocked(t);
      state = core_.RobotState();
    }
    Flush();
    MirrorHold();
    mowbite_msgs::msg::RobotState msg;
    msg.current_state = state.state;
    msg.current_sub_state = state.sub_state;
    msg.current_area = state.current_area;
    msg.current_path = state.current_path;
    msg.current_path_index = state.current_path_index;
    msg.battery_percentage = state.battery_percentage;
    msg.emergency = state.emergency;
    msg.emergency_reason = static_cast<uint32_t>(state.emergency_reason);
    msg.is_charging = state.is_charging;
    msg.rain_detected = state.rain_detected;
    msg.gps_percentage = state.gps_percentage;
    msg.pose_x = state.pose_x;
    msg.pose_y = state.pose_y;
    msg.pose_heading = state.pose_heading;
    msg.pos_accuracy = state.pos_accuracy;
    msg.heading_accuracy = state.heading_accuracy;
    msg.heading_valid = state.heading_valid;
    robot_state_pub_->publish(msg);
  }

  void OnRpcRequest(const mowbite_msgs::msg::RpcRequest& request) {
    // The bridge hands every registered method to every node, the others answer theirs.
    if (request.method == "params.set") {
      OnParamsSet(request);
      return;
    }
    if (request.method == "robot.geometry.get") {
      OnGeometryGet(request);
      return;
    }
    if (request.method == "robot.geometry.set") {
      OnGeometrySet(request);
      return;
    }
    if (request.method == "docking.start") {
      OnDockingStart(request);
      return;
    }
    if (request.method == "system.hold") {
      OnSystemHold(request);
      return;
    }
    if (request.method != "mowing.plan") return;
    json params;
    if (!request.params.empty()) {
      params = json::parse(request.params, nullptr, false);
      if (params.is_discarded()) {
        PublishRpcError(request.id, kPlanInvalidParams, "The parameters are not valid JSON");
        return;
      }
    }
    PlanQuestion question;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      question = planner_.Prepare(params);
    }
    if (!question.ok) {
      PublishRpcError(request.id, question.code, question.message);
      return;
    }
    if (!plan_client_->service_is_ready()) {
      PublishRpcError(request.id, kPlanInternal, "The planner is not running");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const uint64_t seq = ++plan_seq_;
    auto& pending = pending_plans_[seq];
    pending.rpc_id = request.id;
    pending.question = question;
    pending.sent = MonotonicSeconds();
    pending.request_id =
        plan_client_
            ->async_send_request(ToServiceRequest(question.request),
                                 [this, seq](rclcpp::Client<mowbite_planner_interfaces::srv::PlanCoverage>::SharedFuture f) {
                                   OnPlanAnswer(seq, *f.get());
                                 })
            .request_id;
  }

  // Only in IDLE and all or nothing; a change counts once it is saved.
  void OnParamsSet(const mowbite_msgs::msg::RpcRequest& request) {
    using mowbite_msgs::msg::RpcError;
    json params;
    if (!request.params.empty()) {
      params = json::parse(request.params, nullptr, false);
      if (params.is_discarded()) {
        PublishRpcError(request.id, RpcError::ERROR_INVALID_PARAMS, "The parameters are not valid JSON");
        return;
      }
    }
    ParamsResult result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (core_.HoldUntil() > 0) {
        PublishHoldRefusal(request.id, "Settings change after it");
        return;
      }
      if (!core_.Idle()) {
        PublishRpcError(request.id, RpcError::ERROR_INVALID_REQUEST,
                        "Settings change only in IDLE, not while recording, driving or mowing");
        return;
      }
      LogicParams changed = logic_params_;
      result = changed.Set(params);
      if (!result.ok) {
        PublishRpcError(request.id, RpcError::ERROR_INVALID_PARAMS, result.message);
        return;
      }
      std::string error;
      if (!SaveLogicParams(changed, error)) {
        RCLCPP_ERROR(get_logger(), "params.set: %s", error.c_str());
        PublishRpcError(request.id, RpcError::ERROR_INTERNAL, "Cannot save the settings: " + error);
        return;
      }
      logic_params_ = std::move(changed);
      ApplyLogicParams();
    }
    RCLCPP_INFO(get_logger(), "params.set: %s", result.applied.dump().c_str());
    PublishLogicParams();
    Flush();
    mowbite_msgs::msg::RpcResponse msg;
    msg.result = result.applied.dump();
    msg.id = request.id;
    rpc_response_pub_->publish(msg);
  }

  // "Home" from IDLE: {"state": "DOCKING"} once docking began, else -32600 with the reason, its message and fields as
  // data, and DRIVE_REFUSED. Parameters are not needed.
  void OnDockingStart(const mowbite_msgs::msg::RpcRequest& request) {
    Reason why;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      why = core_.DockingProblem();
      if (why.empty() && !transit_client_->service_is_ready()) why = PlannerMissing();
      if (why.empty() && !follow_client_->action_server_is_ready()) why = FollowerMissing();
      if (why.empty() && (drive_.active || job_plan_.active)) {
        why = Reason("STILL_STOPPING", "The last drive is still stopping, try again in a moment");
      }
      if (why.empty()) {
        why = core_.StartDocking();
      } else {
        core_.RefuseDrive(why);
      }
    }
    if (why.empty()) {
      RCLCPP_INFO(get_logger(), "docking.start: docking");
      mowbite_msgs::msg::RpcResponse msg;
      msg.result = json{{"state", "DOCKING"}}.dump();
      msg.id = request.id;
      rpc_response_pub_->publish(msg);
    } else {
      RCLCPP_WARN(get_logger(), "docking.start refused: %s (%s)", why.code.c_str(), why.message.c_str());
      json data = {{"reason", why.code}, {"message", why.message}};
      for (const auto& [key, value] : why.fields.items()) data[key] = value;
      PublishRpcError(request.id, mowbite_msgs::msg::RpcError::ERROR_INVALID_REQUEST, why.message,
                      data.dump(-1, ' ', false, json::error_handler_t::replace));
    }
    Flush();
  }

  // system.hold {"seconds": 0..1800} of the updater: granted only in IDLE with nothing on its way, else none; 0 lets go
  // and a new call replaces the old one. The answer holds what the gate of the updater looks at.
  void OnSystemHold(const mowbite_msgs::msg::RpcRequest& request) {
    using mowbite_msgs::msg::RpcError;
    const json params = request.params.empty() ? json() : json::parse(request.params, nullptr, false);
    const auto seconds = params.is_object() && params.contains("seconds") ? params["seconds"] : json();
    if (!seconds.is_number_integer() || seconds.get<int64_t>() < 0 ||
        seconds.get<int64_t>() > mowbite_hold::kMaxHoldSeconds) {
      PublishRpcError(request.id, RpcError::ERROR_INVALID_PARAMS,
                      "seconds: a whole number from 0 to " + std::to_string(mowbite_hold::kMaxHoldSeconds));
      return;
    }
    const int64_t wanted = seconds.get<int64_t>();
    std::string error;
    json answer;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double t = MonotonicSeconds();
      const int64_t wall = WallSeconds();
      if (wanted > 0 && core_.HoldAllowed()) {
        const mowbite_hold::HoldRecord record{wall + wanted, wall};
        if (hold_path_.empty() || mowbite_hold::WriteHoldFile(hold_path_, record, &error)) {
          hold_.Start(record, t, wall);
          core_.SetHold(record.until);
        }
      } else if (hold_path_.empty() || mowbite_hold::RemoveHoldFile(hold_path_, &error)) {
        hold_.Clear();
        core_.SetHold(0);
      }
      if (error.empty()) answer = HoldAnswerLocked();
    }
    if (!error.empty()) {
      RCLCPP_ERROR(get_logger(), "system.hold %lld: %s", static_cast<long long>(wanted), error.c_str());
      PublishRpcError(request.id, RpcError::ERROR_INTERNAL, "Cannot change the hold: " + error);
    } else {
      const int64_t until = answer["hold_until"].get<int64_t>();
      if (until > 0) {
        RCLCPP_INFO(get_logger(), "system.hold: held for %lld s until %lld (unix s), nothing starts",
                    static_cast<long long>(wanted), static_cast<long long>(until));
      } else if (wanted > 0) {
        RCLCPP_WARN(get_logger(), "system.hold %lld: not granted, the mower is %s %s", static_cast<long long>(wanted),
                    answer["state"].get<std::string>().c_str(), answer["sub_state"].get<std::string>().c_str());
      } else {
        RCLCPP_INFO(get_logger(), "system.hold 0: no hold");
      }
      mowbite_msgs::msg::RpcResponse msg;
      msg.result = answer.dump();
      msg.id = request.id;
      rpc_response_pub_->publish(msg);
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hold_mirror_due_ = true;
    }
    Flush();
    MirrorHold();
  }

  // Called with the mutex held.
  json HoldAnswerLocked() const {
    const RobotStateData state = core_.RobotState();
    return json{{"state", state.state},
                {"is_charging", state.is_charging},
                {"hold_until", HoldUntilLocked()},
                {"sub_state", state.sub_state},
                {"idle", core_.HoldAllowed()},
                {"emergency_reason", state.emergency_reason}};
  }

  // unix s while the logic holds, else 0. Called with the mutex held.
  int64_t HoldUntilLocked() const {
    return core_.HoldUntil();
  }

  // A change of data in the hold of the logic: a rollback of the update would drop it without a word. The driver's own
  // hold, still shown for up to 0.5 s after it ended, refuses none. Called with the mutex held.
  void PublishHoldRefusal(const std::string& id, const std::string& what) {
    const int64_t until = HoldUntilLocked();
    PublishRpcError(id, mowbite_msgs::msg::RpcError::ERROR_HOLD, "An update holds the mower. " + what,
                    json{{"reason", "HOLD"}, {"until", until}}.dump());
  }

  // A hold from before the restart stands for what is left of it, at most kMaxHoldSeconds from now; a file that cannot
  // be read holds as long and is removed after it, like the drivers hold until it is gone.
  void RestoreHold() {
    if (hold_path_.empty()) return;
    const mowbite_hold::HoldFile file = mowbite_hold::ReadHoldFile(hold_path_);
    if (file.state == mowbite_hold::HoldFile::State::kMissing) return;
    const double t = MonotonicSeconds();
    const int64_t wall = WallSeconds();
    if (file.state == mowbite_hold::HoldFile::State::kUnreadable) {
      RCLCPP_ERROR(get_logger(), "%s: held for %lld s, then it is removed", file.error.c_str(),
                   static_cast<long long>(mowbite_hold::kMaxHoldSeconds));
      hold_.Start({wall + mowbite_hold::kMaxHoldSeconds, wall}, t, wall);
    } else {
      hold_.Start(file.record, t, wall);
    }
    if (!hold_.Active(t)) return;
    core_.SetHold(hold_.record()->until);
    RCLCPP_INFO(get_logger(), "Held for an update until %lld (unix s): nothing starts",
                static_cast<long long>(hold_.record()->until));
  }

  // The hold ran out: S3 lets go and the file goes, again at the next tick if it cannot. Called with the mutex held.
  void ExpireHoldLocked(double t) {
    if (!hold_.record() || hold_.Active(t)) return;
    if (core_.HoldUntil() > 0) {
      core_.SetHold(0);
      hold_mirror_due_ = true;
      RCLCPP_INFO(get_logger(), "Hold ended: its time ran out");
    }
    std::string error;
    if (!hold_path_.empty() && !mowbite_hold::RemoveHoldFile(hold_path_, &error)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 60000, "Hold: %s", error.c_str());
      return;
    }
    hold_.Clear();
  }

  void PublishHoldIfChanged() {
    std_msgs::msg::Int64 msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg.data = HoldUntilLocked();
      if (published_hold_ == msg.data) return;
      published_hold_ = msg.data;
    }
    hold_pub_->publish(msg);
  }

  // mowbite/hold for scheduler.sh, the planner, App and HA, the OS its only writer on its host: {"until"} retained while
  // held, again every kHoldMirrorS, and empty once after it and after the start. It waits for the bridge.
  void MirrorHold() {
    if (publish_pub_->get_subscription_count() == 0) return;
    mowbite_msgs::msg::MqttPublish msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double t = MonotonicSeconds();
      const int64_t until = HoldUntilLocked();
      if (!hold_mirror_due_ && (until == 0 || t - hold_mirrored_s_ < kHoldMirrorS)) return;
      hold_mirror_due_ = false;
      hold_mirrored_s_ = t;
      msg.payload = until > 0 ? json{{"until", until}}.dump() : "";
    }
    msg.topic = "mowbite/hold";
    msg.retain = true;
    publish_pub_->publish(msg);
  }

  // <data_dir>/params/mower_logic.json, written like the checkpoint.
  std::string LogicParamsPath() const {
    return data_dir_.empty() ? "" : data_dir_ + "/params/mower_logic.json";
  }

  // tron.yaml, then what params.set saved. A file that cannot be read stays as it is, tron.yaml holds.
  void LoadLogicParams() {
    json base = json::object();
    base["max_position_accuracy"] = get_parameter("max_position_accuracy").as_double();
    base["gps_wait_time"] = get_parameter("gps_wait_time").as_double();
    base["gps_timeout"] = declare_parameter("gps_timeout", 10.0);
    base["undock_distance"] = get_parameter("undock_distance").as_double();
    base["docking_approach_distance"] = get_parameter("docking_approach_distance").as_double();
    base["docking_retry_count"] = get_parameter("docking_retry_count").as_int();
    logic_params_ = LogicParams(base);
    const std::string path = LogicParamsPath();
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "No data_dir: what params.set changes is gone after a restart");
    } else if (std::ifstream in(path); in) {
      std::stringstream text;
      text << in.rdbuf();
      if (logic_params_.Load(text.str())) {
        RCLCPP_INFO(get_logger(), "Settings from %s", path.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "%s is not valid, the settings of tron.yaml hold", path.c_str());
      }
    }
    ApplyLogicParams();
  }

  // What of the settings of params.set the core uses. Called with the mutex held, or before the node runs.
  void ApplyLogicParams() {
    core_.SetGpsSettings(logic_params_.Get("max_position_accuracy"), logic_params_.Get("gps_wait_time"),
                         logic_params_.Get("gps_timeout"));
    core_.SetUndockDistance(logic_params_.Get("undock_distance"));
    core_.SetDockingSettings(logic_params_.Get("docking_approach_distance"),
                             static_cast<int>(logic_params_.Get("docking_retry_count")));
  }

  bool SaveLogicParams(const LogicParams& params, std::string& error) {
    const std::string path = LogicParamsPath();
    return path.empty() || WriteFileSafely(path, params.Text(), &error);
  }

  void PublishLogicParams() {
    LogicParams params;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      params = logic_params_;
    }
    std_msgs::msg::String msg;
    msg.data = json{{"node", kLogicParamsNode}, {"params", params.Values()}}.dump();
    logic_params_pub_->publish(msg);
    mowbite_msgs::msg::GpsLimits limits;
    limits.max_position_accuracy = params.Get("max_position_accuracy");
    limits.gps_timeout = params.Get("gps_timeout");
    gps_limits_pub_->publish(limits);
  }

  // A number of the platform's file (parameters geometry.*); one that is missing stops the start.
  double GeometryNumber(const std::string& key) {
    const std::string name = "geometry." + key;
    if (!has_parameter(name)) declare_parameter(name, std::numeric_limits<double>::quiet_NaN());
    const double value = get_parameter(name).as_double();
    if (std::isnan(value)) throw std::invalid_argument(key + " is missing in the geometry file of the platform");
    return value;
  }

  // The dimensions of the platform (mowbite_bringup/config/geometry/<platform>.yaml); the entries of the user follow.
  Geometry FactoryGeometry() {
    json factory = json::object();
    for (const auto& key : GeometryKeys()) {
      if (key.type == GeometryType::kNumber) factory[key.name] = GeometryNumber(key.name);
    }
    // The outline of the body, if the platform has one: x0, y0, x1, y1, ...
    const std::vector<double> flat = declare_parameter("geometry.robot_outline", std::vector<double>());
    if (flat.size() % 2 != 0) throw std::invalid_argument("geometry.robot_outline needs pairs of x and y");
    if (!flat.empty()) {
      json points = json::array();
      for (size_t i = 0; i < flat.size(); i += 2) points.push_back(json::array({flat[i], flat[i + 1]}));
      factory["robot_outline"] = std::move(points);
    }
    return Geometry(declare_parameter("geometry.platform", std::string("tron")), factory);
  }

  std::string GeometryPath() const {
    return data_dir_.empty() ? "" : data_dir_ + "/geometry.json";
  }

  // What robot.geometry.set saved. A file that cannot be read stays as it is, the dimensions of the platform hold.
  void LoadGeometryEntries() {
    const std::string path = GeometryPath();
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "No data_dir: what robot.geometry.set changes is gone after a restart");
      return;
    }
    std::ifstream in(path);
    if (!in) return;
    std::stringstream text;
    text << in.rdbuf();
    std::string why;
    if (geometry_.Load(text.str(), &why)) {
      RCLCPP_INFO(get_logger(), "Dimensions of the mower from %s (version %d)", path.c_str(), geometry_.version());
    } else {
      RCLCPP_ERROR(get_logger(), "%s is not valid (%s), the dimensions of the platform hold", path.c_str(),
                   why.c_str());
    }
  }

  void PublishGeometry() {
    json params;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      params = geometry_.Params();
    }
    std_msgs::msg::String msg;
    msg.data = json{{"node", kGeometryNode}, {"params", params}}.dump();
    geometry_pub_->publish(msg);
  }

  void OnGeometryGet(const mowbite_msgs::msg::RpcRequest& request) {
    json answer;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      answer = geometry_.Get();
    }
    mowbite_msgs::msg::RpcResponse msg;
    msg.result = answer.dump();
    msg.id = request.id;
    rpc_response_pub_->publish(msg);
  }

  // Only in IDLE and all or nothing; a change counts once it is saved. The answer is the new robot.geometry.get.
  void OnGeometrySet(const mowbite_msgs::msg::RpcRequest& request) {
    using mowbite_msgs::msg::RpcError;
    json params;
    if (!request.params.empty()) {
      params = json::parse(request.params, nullptr, false);
      if (params.is_discarded()) {
        PublishRpcError(request.id, RpcError::ERROR_INVALID_PARAMS, "The parameters are not valid JSON");
        return;
      }
    }
    GeometryChange change;
    json answer;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (core_.HoldUntil() > 0) {
        PublishHoldRefusal(request.id, "The dimensions change after it");
        return;
      }
      if (!core_.Idle()) {
        PublishRpcError(request.id, RpcError::ERROR_INVALID_REQUEST,
                        "The dimensions change only in IDLE, not while recording, driving or mowing",
                        json{{"reason", "BUSY"}}.dump());
        return;
      }
      Geometry changed = geometry_;
      change = changed.Set(params);
      if (!change.ok) {
        json data = json::object();
        data["reason"] = change.problem.reason;
        data["message"] = change.problem.message;
        if (!change.problem.key.empty()) data["key"] = change.problem.key;
        for (const auto& item : change.problem.data.items()) data[item.key()] = item.value();
        PublishRpcError(request.id, RpcError::ERROR_INVALID_PARAMS, change.problem.message, data.dump());
        return;
      }
      if (change.stored) {
        std::string error;
        if (!SaveGeometry(changed, &error)) {
          RCLCPP_ERROR(get_logger(), "robot.geometry.set: %s", error.c_str());
          PublishRpcError(request.id, RpcError::ERROR_INTERNAL, "Cannot save the dimensions: " + error);
          return;
        }
        geometry_ = std::move(changed);
        // Only a value that is another now is news; a key entered with the platform's own value is not.
        if (!change.changed.empty()) core_.GeometryChanged(change.changed, change.below_default, geometry_.version());
      }
      answer = geometry_.Get();
    }
    if (change.stored) {
      RCLCPP_INFO(get_logger(), "robot.geometry.set: version %d, changed %zu key(s)", answer["version"].get<int>(),
                  change.changed.size());
      PublishGeometry();
      Flush();
    }
    mowbite_msgs::msg::RpcResponse msg;
    msg.result = answer.dump();
    msg.id = request.id;
    rpc_response_pub_->publish(msg);
  }

  bool SaveGeometry(const Geometry& geometry, std::string* error) {
    const std::string path = GeometryPath();
    return path.empty() || WriteFileSafely(path, geometry.Text(), error);
  }

  void OnPlanAnswer(uint64_t seq, const mowbite_planner_interfaces::srv::PlanCoverage::Response& response) {
    PendingPlan pending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = pending_plans_.find(seq);
      // Late: the request has timed out and was answered with an error.
      if (it == pending_plans_.end()) return;
      pending = std::move(it->second);
      pending_plans_.erase(it);
    }
    const CoverageAnswer answer = ToAnswer(response);
    if (!answer.success) {
      RCLCPP_WARN(get_logger(), "mowing.plan: the planner failed: %s", answer.message.c_str());
      PublishRpcError(pending.rpc_id, kPlanInternal, "coverage planning failed: " + answer.message);
      return;
    }
    const json result = planner_.Result(pending.question, answer);
    RCLCPP_INFO(get_logger(), "mowing.plan %s: %zu paths from %zu poses in %.1f s", pending.question.area_id.c_str(),
                result["paths"].size(), answer.poses.size(), MonotonicSeconds() - pending.sent);
    mowbite_msgs::msg::RpcResponse msg;
    msg.result = result.dump();
    msg.id = pending.rpc_id;
    rpc_response_pub_->publish(msg);
  }

  // A plan the planner does not answer in time is dropped, with an error for the app.
  void ExpirePlans() {
    std::vector<std::string> expired;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double t = MonotonicSeconds();
      if (drive_.planning && t - drive_.sent > plan_timeout_s_) {
        CancelDrive(Reason("PLANNER_TIMEOUT",
                           "No answer from the planner in " + std::to_string(static_cast<int>(plan_timeout_s_)) + " s"));
      }
      if (job_plan_.active && t - job_plan_.sent > plan_timeout_s_) {
        plan_client_->remove_pending_request(job_plan_.request_id);
        job_plan_ = JobPlan();
        core_.OnJobPlanFailed("No answer from the planner in " + std::to_string(static_cast<int>(plan_timeout_s_)) + " s");
      }
      for (auto it = pending_plans_.begin(); it != pending_plans_.end();) {
        if (t - it->second.sent > plan_timeout_s_) {
          plan_client_->remove_pending_request(it->second.request_id);
          expired.push_back(it->second.rpc_id);
          it = pending_plans_.erase(it);
        } else {
          ++it;
        }
      }
    }
    for (const auto& id : expired) {
      RCLCPP_WARN(get_logger(), "mowing.plan: no answer from the planner in %.0f s", plan_timeout_s_);
      PublishRpcError(id, kPlanInternal, "No answer from the planner in " + std::to_string(static_cast<int>(plan_timeout_s_)) + " s");
    }
  }

  void PublishRpcError(const std::string& id, int code, const std::string& message, const std::string& data = "") {
    mowbite_msgs::msg::RpcError msg;
    msg.code = static_cast<int16_t>(code);
    msg.message = message;
    msg.id = id;
    msg.data = data;
    rpc_error_pub_->publish(msg);
  }

  static std::shared_ptr<mowbite_planner_interfaces::srv::PlanCoverage::Request> ToServiceRequest(const CoverageRequest& r) {
    auto request = std::make_shared<mowbite_planner_interfaces::srv::PlanCoverage::Request>();
    request->boundary = ToPolygon(r.boundary);
    for (const auto& ring : r.obstacles) request->obstacles.push_back(ToPolygon(ring));
    for (const auto& ring : r.drivable_areas) request->drivable_areas.push_back(ToPolygon(ring));
    request->perimeter_passes = r.perimeter_passes;
    request->use_angle = r.use_angle;
    request->angle = r.angle;
    if (r.lane_overlap_passes) request->overrides.push_back(IntegerParameter("lane_overlap_passes", *r.lane_overlap_passes));
    if (r.perimeter_offset) request->overrides.push_back(DoubleParameter("perimeter_offset", *r.perimeter_offset));
    return request;
  }

  static CoverageAnswer ToAnswer(const mowbite_planner_interfaces::srv::PlanCoverage::Response& r) {
    CoverageAnswer answer;
    answer.success = r.success;
    answer.message = r.message;
    answer.angle = r.angle;
    answer.fingerprint = r.fingerprint;
    answer.poses.reserve(r.path.poses.size());
    for (const auto& pose : r.path.poses) {
      answer.poses.push_back({pose.pose.position.x, pose.pose.position.y});
      answer.headings.push_back(Yaw(pose.pose.orientation));
    }
    answer.section_start = r.section_start;
    answer.segment_start = r.segment_start;
    answer.segment_end = r.segment_end;
    answer.segment_type = r.segment_type;
    return answer;
  }

  void PublishReadings() {
    std::vector<SensorReading> readings;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      readings = core_.Readings();
    }
    for (const auto& r : readings) {
      mowbite_msgs::msg::SensorValue msg;
      msg.sensor_id = r.id;
      msg.value = r.value;
      msg.text = r.text;
      sensor_value_pub_->publish(msg);
    }
  }

  void PublishOverlay() {
    // The first, empty one replaces the lines of a recording before a restart, so it waits for the bridge.
    if (publish_pub_->get_subscription_count() == 0) return;
    mowbite_msgs::msg::MqttPublish msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!core_.TakeOverlayChanged()) return;
      msg.payload = core_.MapOverlay().dump();
    }
    msg.topic = "map_overlay/json";
    msg.retain = true;
    publish_pub_->publish(msg);
  }

  // Sends a change of the map the recording asks for; its answer goes back to the core. Called with the mutex held.
  void SendMapEdit(const MapEdit& edit) {
    const uint64_t seq = ++edit_seq_;
    edit_is_dock_ = edit.kind == MapEdit::Kind::kDock;
    const auto answer = [this, seq](bool ok, const std::string& id, const std::string& code,
                                    const std::string& message) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // A late answer after the timeout is not the current one any more.
        if (seq != edit_seq_ || !edit_sent_) return;
        edit_sent_.reset();
        edit_request_id_.reset();
        core_.OnMapEditResult(ok, id, code, message);
      }
      Flush();
    };
    if (edit_is_dock_) {
      if (!set_dock_client_->service_is_ready()) {
        core_.OnMapEditResult(false, "", "NO_ANSWER", "mowbite_map does not offer map/set_dock");
        return;
      }
      auto request = std::make_shared<mowbite_msgs::srv::SetDock::Request>();
      request->x = edit.x;
      request->y = edit.y;
      request->heading = edit.heading;
      edit_sent_ = MonotonicSeconds();
      edit_request_id_ = set_dock_client_
                             ->async_send_request(request,
                                                  [answer](rclcpp::Client<mowbite_msgs::srv::SetDock>::SharedFuture f) {
                                                    const auto r = f.get();
                                                    answer(r->success, r->id, r->code, r->message);
                                                  })
                             .request_id;
      return;
    }
    if (!add_area_client_->service_is_ready()) {
      core_.OnMapEditResult(false, "", "NO_ANSWER", "mowbite_map does not offer map/add_area");
      return;
    }
    auto request = std::make_shared<mowbite_msgs::srv::AddArea::Request>();
    request->type = edit.type;
    for (const auto& p : edit.outline) request->outline.push_back(ToPoint(p));
    for (const auto& ring : edit.obstacles) {
      mowbite_msgs::msg::Outline outline;
      for (const auto& p : ring) outline.points.push_back(ToPoint(p));
      request->obstacles.push_back(std::move(outline));
    }
    edit_sent_ = MonotonicSeconds();
    edit_request_id_ = add_area_client_
                           ->async_send_request(request,
                                                [answer](rclcpp::Client<mowbite_msgs::srv::AddArea>::SharedFuture f) {
                                                  const auto r = f.get();
                                                  answer(r->success, r->id, r->code, r->message);
                                                })
                           .request_id;
  }

  // Sets the heading of the mower on the docking station in the localization, whose answer decides the reference check.
  // Called with the mutex held.
  void SendDockCheck(double heading) {
    const uint64_t seq = ++dock_check_seq_;
    if (!set_heading_client_->service_is_ready()) {
      FinishDockCheck(false, 0, 0, 999.0f, "The localization does not offer localization/set_heading");
      return;
    }
    auto request = std::make_shared<mowbite_msgs::srv::SetHeading::Request>();
    request->heading = heading;
    dock_check_sent_ = MonotonicSeconds();
    dock_check_request_id_ =
        set_heading_client_
            ->async_send_request(request,
                                 [this, seq](rclcpp::Client<mowbite_msgs::srv::SetHeading>::SharedFuture f) {
                                   {
                                     std::lock_guard<std::mutex> lock(mutex_);
                                     // A late answer after the timeout is not the current one any more.
                                     if (seq != dock_check_seq_ || !dock_check_sent_) return;
                                     const auto r = f.get();
                                     FinishDockCheck(r->success, r->x, r->y, r->position_accuracy, r->message);
                                   }
                                   Flush();
                                 })
            .request_id;
    RCLCPP_INFO(get_logger(), "Reference check on the docking station: heading %.3f rad", heading);
  }

  // Called with the mutex held.
  void FinishDockCheck(bool ok, double x, double y, float accuracy, const std::string& message) {
    dock_check_sent_.reset();
    const Reason why = core_.OnDockCheck(ok, x, y, accuracy, message);
    const bool drive = dock_drive_.has_value();
    if (why.empty()) {
      RCLCPP_INFO(get_logger(), "%s from the docking station: reference check passed, undocking",
                  drive ? "Drive" : "Mowing");
      return;
    }
    if (why.code == "ON_DOCK") {
      RCLCPP_INFO(get_logger(), "%s from the docking station refused: %s (%s)", drive ? "Drive" : "Mowing",
                  why.code.c_str(), why.message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s from the docking station refused: %s (%s)", drive ? "Drive" : "Mowing",
                  why.code.c_str(), why.message.c_str());
    }
    if (drive) {
      RespondDrive(dock_drive_->header, false, why.message);
      dock_drive_.reset();
    }
  }

  // hw/charge: off before undocking, the answer goes to the core; on again after an undocking that ended on the
  // station. Called with the mutex held.
  void SendCharge(bool on) {
    const uint64_t seq = ++charge_seq_;
    charge_sent_.reset();
    if (!charge_client_->service_is_ready()) {
      if (on) {
        RCLCPP_WARN(get_logger(), "hw/charge is not available, charging stays as the driver has it");
      } else {
        core_.OnChargeOff(false, "the driver does not offer hw/charge");
      }
      return;
    }
    auto request = std::make_shared<mowbite_msgs::srv::SetCharge::Request>();
    request->on = on;
    if (!on) charge_sent_ = MonotonicSeconds();
    charge_request_id_ =
        charge_client_
            ->async_send_request(request,
                                 [this, seq, on](rclcpp::Client<mowbite_msgs::srv::SetCharge>::SharedFuture f) {
                                   const auto r = f.get();
                                   if (on) {
                                     if (!r->success) RCLCPP_WARN(get_logger(), "hw/charge on: %s", r->message.c_str());
                                     return;
                                   }
                                   {
                                     std::lock_guard<std::mutex> lock(mutex_);
                                     // A late answer after the timeout is not the current one any more.
                                     if (seq != charge_seq_ || !charge_sent_) return;
                                     charge_sent_.reset();
                                     core_.OnChargeOff(r->success, r->message);
                                   }
                                   Flush();
                                 })
            .request_id;
    RCLCPP_INFO(get_logger(), "Charging %s", on ? "allowed again" : "off before undocking");
  }

  // Hands the way out of the station to the follower: straight, without steering, at the speed of undocking. Besides
  // the drivable areas base_link may be in the corridor along it. Called with the mutex held.
  void SendUndock(const UndockPath& path) {
    std::vector<Ring> drivable;
    std::vector<Ring> obstacles;
    planner_.TransitAreas(&drivable, &obstacles);
    FollowPath::Goal goal;
    for (const auto& p : path.poses) goal.poses.push_back(PoseAt(p.x, p.y, p.heading));
    goal.start_index = 0;
    for (const auto& ring : drivable) goal.allowed_areas.push_back(ToPolygon(ring));
    goal.allowed_areas.push_back(ToPolygon(path.corridor));
    for (const auto& ring : obstacles) goal.obstacles.push_back(ToPolygon(ring));
    goal.max_speed = static_cast<float>(path.speed);
    goal.stall_time = static_cast<float>(path.stall_time);
    goal.straight = true;
    drive_ = Drive();
    drive_.purpose = Purpose::kUndock;
    drive_.seq = ++drive_seq_;
    drive_.active = true;
    drive_.sent = MonotonicSeconds();
    const auto& end = path.poses.back();
    char text[96];
    std::snprintf(text, sizeof(text), "%.2f m straight ahead to (%.2f, %.2f) at %.2f m/s",
                  std::hypot(end.x - path.poses.front().x, end.y - path.poses.front().y), end.x, end.y, path.speed);
    SendFollow(goal, text);
  }

  // After undocking for a drive: the drive goes on, or its request gets why not. Called with the mutex held.
  void OnUndockOutcome(const UndockOutcome& outcome) {
    if (!outcome.for_drive || !dock_drive_) return;
    const DockDrive pending = *dock_drive_;
    dock_drive_.reset();
    if (outcome.undocked && outcome.why.empty() && core_.Driving()) {
      PlanDrive(pending.request, pending.header);
    } else {
      RespondDrive(pending.header, false, outcome.why.message);
    }
  }

  static geometry_msgs::msg::Point ToPoint(const Point2& p) {
    geometry_msgs::msg::Point point;
    point.x = p.x;
    point.y = p.y;
    return point;
  }

  // Sends stop requests, events, changed actions and map edits collected by the core.
  void Flush() {
    // An edit refused before it went out brings its events with it, so they are collected after it.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (auto edit = core_.TakeMapEdit()) SendMapEdit(*edit);
      if (auto heading = core_.TakeDockCheck()) SendDockCheck(*heading);
      // Undocking: charging off, then the way out; after it the drive that waited, or charging on again.
      if (core_.TakeChargeOff()) SendCharge(false);
      if (auto path = core_.TakeUndockPath()) SendUndock(*path);
      if (core_.TakeChargeOn()) SendCharge(true);
      if (auto outcome = core_.TakeUndockOutcome()) OnUndockOutcome(*outcome);
    }
    std::vector<Event> events;
    bool actions_changed;
    bool stop;
    std::vector<ReasonChange> reasons;
    std::vector<std::string> notices;
    std::optional<std::string> checkpoint;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (auto why = core_.TakeDriveCancel()) CancelDrive(*why);
      // The blade goes off at once when the logic no longer wants it, not at the next tick.
      UpdateBladeLocked();
      PumpJob();
      if (auto why = core_.TakeDriveCancel()) CancelDrive(*why);
      PumpDocking();
      UpdateBladeLocked();
      events = core_.TakeEvents();
      actions_changed = core_.TakeActionsChanged();
      stop = core_.TakeStopRequest();
      reasons = core_.TakeReasonChanges();
      notices = core_.TakeNotices();
      checkpoint = core_.TakeCheckpoint();
    }
    if (checkpoint) SaveCheckpoint(*checkpoint);
    for (const auto& line : notices) RCLCPP_INFO(get_logger(), "%s", line.c_str());
    if (stop) SendStop();
    // The app shows the reasons of an active emergency from this line, like with mower_comms_v2 of OpenMower.
    for (const auto& change : reasons) {
      if (change.to == 0) {
        RCLCPP_INFO(get_logger(), "Emergency cleared");
      } else {
        RCLCPP_WARN(get_logger(), "Emergency reason changed from: '%s' to: '%s'",
                    EmergencyReasonNames(change.from).c_str(), EmergencyReasonNames(change.to).c_str());
      }
    }
    for (auto& event : events) {
      if (event.type == "RECORDING_REFUSED") {
        RCLCPP_WARN(get_logger(), "Recording: %s refused: %s", event.details.value("action", "").c_str(),
                    event.details.value("reason", "").c_str());
      }
      if (event.type == "UNDOCKING_FAILED") {
        RCLCPP_WARN(get_logger(), "UNDOCKING_FAILED %s: %s", event.details.value("reason", "").c_str(),
                    event.details.value("message", "").c_str());
      }
      mowbite_msgs::msg::MqttPublish msg;
      msg.topic = "events/json";
      msg.payload = mowbite_bridge::BuildEvent(event.type, event.details, now().seconds()).dump();
      publish_pub_->publish(msg);
    }
    if (actions_changed) PublishActions();
    PublishIdleIfChanged();
    PublishHoldIfChanged();
  }

  void PublishIdleIfChanged() {
    std_msgs::msg::Bool msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg.data = core_.Idle();
      if (published_idle_ == msg.data) return;
      published_idle_ = msg.data;
    }
    idle_pub_->publish(msg);
  }

  void PublishActionsIfChanged() {
    bool changed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      changed = core_.TakeActionsChanged();
    }
    if (changed) PublishActions();
  }

  void PublishActions() {
    std::vector<ActionGroup> groups;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      groups = core_.Actions();
    }
    for (const auto& group : groups) {
      mowbite_msgs::msg::ActionList msg;
      msg.node_prefix = group.prefix;
      for (const auto& action : group.actions) {
        mowbite_msgs::msg::ActionEntry entry;
        entry.id = action.id;
        entry.name = action.name;
        entry.enabled = action.enabled;
        msg.actions.push_back(entry);
      }
      actions_pub_->publish(msg);
    }
  }

  // The drive to a pose: its number tells late answers apart, the request waits for its answer until the follower
  // has taken the path.
  struct Drive {
    Purpose purpose = Purpose::kDriveTo;
    bool active = false;
    // kMow: the pose of the job's path for each pose of the follower's path.
    std::vector<size_t> path_index;
    size_t PathIndex(uint32_t index) const {
      if (path_index.empty()) return index;
      return path_index[std::min<size_t>(index, path_index.size() - 1)];
    }
    uint64_t seq = 0;
    std::shared_ptr<rmw_request_id_t> header;
    bool planning = false;
    int64_t request_id = 0;
    // MonotonicSeconds() when the request went out, like every time the node measures against a limit.
    double sent = 0;
    std::vector<geometry_msgs::msg::Polygon> allowed;
    std::vector<geometry_msgs::msg::Polygon> obstacles;
    FollowHandle::SharedPtr handle;
    bool cancel_pending = false;
    std::optional<Reason> cancel_reason;
    // MonotonicSeconds() of the follower's last sign for this drive (goal sent or taken, feedback; 0: no goal out
    // yet) and the pose of its path it got to.
    double follower_sign = 0;
    uint32_t follower_index = 0;
  };

  // The plan of the job's area on its way to the planner.
  struct JobPlan {
    bool active = false;
    uint64_t seq = 0;
    double sent = 0;
    int64_t request_id = 0;
  };

  // A mowing.plan request on its way to the planner.
  struct PendingPlan {
    std::string rpc_id;
    PlanQuestion question;
    double sent = 0;
    int64_t request_id = 0;
  };

  std::mutex mutex_;
  LogicCore core_;
  // system.hold: <data_dir>/hold.json (empty without data_dir), its time on the monotonic clock, what logic/hold said
  // last and when mowbite/hold went out.
  std::string hold_path_;
  mowbite_hold::HoldTimer hold_;
  std::optional<int64_t> published_hold_;
  bool hold_mirror_due_ = true;
  double hold_mirrored_s_ = 0;
  Geometry geometry_;
  Drive drive_;
  JobPlan job_plan_;
  std::string data_dir_;
  LogicParams logic_params_;
  double blade_lease_s_ = 1.0;
  double blade_renew_s_ = 0.5;
  float blade_lease_max_s_ = 0;
  bool blade_on_requested_ = false;
  double blade_sent_s_ = 0;
  uint64_t drive_seq_ = 0;
  double transit_max_outside_ = 0.3;
  MowingPlanner planner_;
  std::map<uint64_t, PendingPlan> pending_plans_;
  uint64_t plan_seq_ = 0;
  double plan_timeout_s_ = 30.0;
  // MonotonicSeconds() of the last hw/status and localization/pose: a step of the system clock moves no limit.
  std::optional<double> last_hw_;
  std::optional<double> last_pose_;
  double hw_timeout_s_;
  double pose_timeout_s_;
  double follower_timeout_s_ = 2.0;
  double map_edit_timeout_s_;
  // The map edit on its way: when it went out, its request, and a number that tells a late answer apart.
  std::optional<double> edit_sent_;
  std::optional<int64_t> edit_request_id_;
  // The release asked of S2, under mutex_.
  std::optional<double> release_sent_;
  int64_t release_request_id_ = 0;
  bool edit_is_dock_ = false;
  uint64_t edit_seq_ = 0;
  // The reference check on its way to the localization, and the drive request that waits for it and for undocking.
  std::optional<double> dock_check_sent_;
  int64_t dock_check_request_id_ = 0;
  uint64_t dock_check_seq_ = 0;
  double dock_check_timeout_s_ = 2.0;
  struct DockDrive {
    std::shared_ptr<rmw_request_id_t> header;
    mowbite_msgs::srv::DriveTo::Request request;
  };
  std::optional<DockDrive> dock_drive_;
  // Charging off on its way to the driver before undocking.
  std::optional<double> charge_sent_;
  int64_t charge_request_id_ = 0;
  uint64_t charge_seq_ = 0;
  double charge_timeout_s_ = 2.0;

  rclcpp::Publisher<mowbite_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::ActionList>::SharedPtr actions_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorSpec>::SharedPtr sensor_info_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logic_params_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr geometry_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::GpsLimits>::SharedPtr gps_limits_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr idle_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr hold_pub_;
  std::optional<bool> published_idle_;
  rclcpp::Publisher<mowbite_msgs::msg::DriveCommand>::SharedPtr drive_pub_;
  rclcpp::Client<mowbite_msgs::srv::SetEmergency>::SharedPtr emergency_client_;
  rclcpp::Client<mowbite_msgs::srv::SetBlade>::SharedPtr blade_client_;
  rclcpp::Client<mowbite_msgs::srv::AddArea>::SharedPtr add_area_client_;
  rclcpp::Client<mowbite_msgs::srv::SetDock>::SharedPtr set_dock_client_;
  rclcpp::Client<mowbite_msgs::srv::SetHeading>::SharedPtr set_heading_client_;
  rclcpp::Client<mowbite_msgs::srv::SetCharge>::SharedPtr charge_client_;
  rclcpp::Client<mowbite_planner_interfaces::srv::PlanCoverage>::SharedPtr plan_client_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr rpc_response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr rpc_error_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr rpc_methods_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr rpc_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::RobotPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::GnssFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::PositionJump>::SharedPtr jump_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MapStatus>::SharedPtr map_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr follower_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr caps_sub_;
  rclcpp::Client<mowbite_planner_interfaces::srv::PlanTransit>::SharedPtr transit_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp::Service<mowbite_msgs::srv::DriveTo>::SharedPtr drive_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_drive_srv_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::TimerBase::SharedPtr overlay_timer_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::TimerBase::SharedPtr blade_timer_;
};

}  // namespace mowbite_logic

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_logic::LogicNode)
