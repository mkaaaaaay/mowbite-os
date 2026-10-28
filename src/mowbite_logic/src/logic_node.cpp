#include <chrono>
#include <cmath>
#include <limits>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mowbite_msgs/msg/absolute_pose.hpp>
#include <mowbite_msgs/msg/action_list.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/map_status.hpp>
#include <mowbite_msgs/msg/mqtt_publish.hpp>
#include <mowbite_msgs/msg/outline.hpp>
#include <mowbite_msgs/msg/position_jump.hpp>
#include <mowbite_msgs/msg/robot_state.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mowbite_msgs/msg/sensor_info.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mowbite_msgs/srv/add_area.hpp>
#include <mowbite_msgs/srv/set_dock.hpp>
#include <mowbite_msgs/srv/set_emergency.hpp>
#include <map>
#include <mowbite_planner_interfaces/srv/plan_coverage.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include "mowbite_bridge/monitor.h"
#include "mowbite_logic/logic_core.h"
#include "mowbite_logic/mowing_plan.h"

namespace mowbite_logic {

namespace {

rclcpp::QoS LatchedQos(size_t depth) {
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
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
 public:
  explicit LogicNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mowbite_logic", options), core_(LoadConfig()), planner_(LoadPlanSettings()) {
    hw_timeout_s_ = declare_parameter("hw_status_timeout", 2.0);
    pose_timeout_s_ = declare_parameter("pose_timeout", 2.0);
    // mowbite_map answers at once; a recording waits this long (s) before it counts as not saved.
    map_edit_timeout_s_ = declare_parameter("map_edit_timeout", 5.0);

    robot_state_pub_ = create_publisher<mowbite_msgs::msg::RobotState>("bridge/robot_state", 10);
    actions_pub_ = create_publisher<mowbite_msgs::msg::ActionList>("bridge/actions", LatchedQos(50));
    sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorInfo>("bridge/sensor_info", LatchedQos(100));
    sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("bridge/sensor_value", 100);
    publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);
    // The only source of motion so far; the safety filter goes in between once there are more.
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("hw/cmd_vel", 10);

    // The app container rates the battery by /ll/board; any value except YardForce makes it show battery_percentage.
    params_pub_ = create_publisher<std_msgs::msg::String>(
        "bridge/params", rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local());
    std_msgs::msg::String board_msg;
    board_msg.data = nlohmann::json{{"node", "/ll"}, {"params", {{"board", declare_parameter("board", "Tron")}}}}.dump();
    params_pub_->publish(board_msg);
    emergency_client_ = create_client<mowbite_msgs::srv::SetEmergency>("hw/emergency");
    add_area_client_ = create_client<mowbite_msgs::srv::AddArea>("map/add_area");
    set_dock_client_ = create_client<mowbite_msgs::srv::SetDock>("map/set_dock");

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
          std::lock_guard<std::mutex> lock(mutex_);
          planner_.SetMap(msg->data);
        });
    mowbite_msgs::msg::RpcMethods methods;
    methods.node_id = "mowbite_logic";
    methods.methods = {"mowing.plan"};
    rpc_methods_pub_->publish(methods);

    hw_sub_ = create_subscription<mowbite_msgs::msg::HardwareStatus>(
        "hw/status", 10, [this](mowbite_msgs::msg::HardwareStatus::ConstSharedPtr msg) { OnHardwareStatus(*msg); });
    pose_sub_ = create_subscription<mowbite_msgs::msg::AbsolutePose>(
        "localization/pose", 10, [this](mowbite_msgs::msg::AbsolutePose::ConstSharedPtr msg) { OnPose(*msg); });
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
            planner_.SetMapStatus(msg->mowing_allowed, msg->reason);
          }
          Flush();
        });
    action_sub_ = create_subscription<std_msgs::msg::String>(
        "bridge/action", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) { OnAction(msg->data); });
    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "bridge/teleop", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { OnTeleop(*msg); });

    for (const auto& sensor : core_.Sensors()) {
      mowbite_msgs::msg::SensorInfo info;
      info.sensor_id = sensor.id;
      info.sensor_name = sensor.name;
      info.value_type = sensor.value_type;
      info.value_description = sensor.value_description;
      info.unit = sensor.unit;
      info.has_min_max = sensor.has_min_max;
      info.min_value = sensor.min_value;
      info.max_value = sensor.max_value;
      info.has_critical_low = sensor.has_critical_low;
      info.lower_critical_value = sensor.lower_critical_value;
      info.has_critical_high = sensor.has_critical_high;
      info.upper_critical_value = sensor.upper_critical_value;
      sensor_info_pub_->publish(info);
    }

    state_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { OnStateTimer(); });
    sensor_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { PublishReadings(); });
    // At most twice a second: a long outline is a large message. Retained, so a late app sees the lines too.
    overlay_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { PublishOverlay(); });
    plan_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { ExpirePlans(); });
    PublishActionsIfChanged();
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
    return config;
  }

  // The parameters of /mower_logic in OpenMower that shape a plan, same names.
  PlanSettings LoadPlanSettings() {
    PlanSettings settings;
    settings.tool_width = declare_parameter("tool_width", settings.tool_width);
    settings.mower_width = declare_parameter("mower_width", settings.mower_width);
    settings.outline_count = static_cast<int>(declare_parameter("outline_count", static_cast<int64_t>(settings.outline_count)));
    settings.outline_overlap_count =
        static_cast<int>(declare_parameter("outline_overlap_count", static_cast<int64_t>(settings.outline_overlap_count)));
    const double offset = declare_parameter("outline_offset", std::numeric_limits<double>::quiet_NaN());
    if (std::isfinite(offset)) settings.outline_offset = offset;
    settings.mow_angle_offset = declare_parameter("mow_angle_offset", settings.mow_angle_offset);
    settings.mow_angle_offset_is_absolute = declare_parameter("mow_angle_offset_is_absolute", false);
    return settings;
  }

  void OnHardwareStatus(const mowbite_msgs::msg::HardwareStatus& msg) {
    HardwareSnapshot hw;
    hw.telemetry_ok = msg.telemetry_ok;
    hw.emergency = msg.emergency;
    hw.emergency_reason = static_cast<int>(msg.emergency_reason);
    hw.rain = msg.rain;
    hw.dock_contact = msg.dock_contact;
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
      last_hw_ = now();
    }
    Flush();
  }

  void OnPose(const mowbite_msgs::msg::AbsolutePose& msg) {
    PoseSnapshot pose;
    pose.x = msg.pose.pose.position.x;
    pose.y = msg.pose.pose.position.y;
    pose.heading = msg.vehicle_heading;
    pose.position_accuracy = msg.position_accuracy;
    pose.orientation_accuracy = msg.orientation_accuracy;
    pose.orientation_valid = msg.orientation_valid;
    std::lock_guard<std::mutex> lock(mutex_);
    core_.OnPose(pose);
    last_pose_ = now();
  }

  void OnAction(const std::string& action_id) {
    ActionResult result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result = core_.OnAction(action_id);
    }
    switch (result) {
      case ActionResult::kReleaseEmergency: ReleaseEmergency(); break;
      case ActionResult::kRejected: RCLCPP_WARN(get_logger(), "Action %s not possible now", action_id.c_str()); break;
      case ActionResult::kUnknown: RCLCPP_DEBUG(get_logger(), "Ignoring action %s", action_id.c_str()); break;
      case ActionResult::kDone: RCLCPP_INFO(get_logger(), "Action %s", action_id.c_str()); break;
    }
    Flush();
  }

  void ReleaseEmergency() {
    if (!emergency_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "hw/emergency is not available, emergency stays");
      return;
    }
    auto request = std::make_shared<mowbite_msgs::srv::SetEmergency::Request>();
    request->latch = false;
    request->reason = "reset_emergency from the app";
    emergency_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "Emergency release requested");
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
    cmd_vel_pub_->publish(out);
  }

  void OnStateTimer() {
    RobotStateData state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!last_hw_ || (now() - *last_hw_).seconds() > hw_timeout_s_) core_.OnHardwareTimeout();
      if (!last_pose_ || (now() - *last_pose_).seconds() > pose_timeout_s_) core_.OnPoseTimeout();
      if (edit_sent_ && (now() - *edit_sent_).seconds() > map_edit_timeout_s_) {
        edit_sent_.reset();
        if (edit_request_id_) {
          if (edit_is_dock_) {
            set_dock_client_->remove_pending_request(*edit_request_id_);
          } else {
            add_area_client_->remove_pending_request(*edit_request_id_);
          }
          edit_request_id_.reset();
        }
        core_.OnMapEditResult(false, "", "No answer from mowbite_map; look at the map before saving again");
      }
      state = core_.RobotState();
    }
    Flush();
    mowbite_msgs::msg::RobotState msg;
    msg.current_state = state.state;
    msg.current_sub_state = state.sub_state;
    msg.current_area = -1;
    msg.current_path = -1;
    msg.current_path_index = -1;
    msg.battery_percentage = state.battery_percentage;
    msg.emergency = state.emergency;
    msg.emergency_reason = static_cast<uint16_t>(state.emergency_reason);
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
    pending.sent = now();
    pending.request_id =
        plan_client_
            ->async_send_request(ToServiceRequest(question.request),
                                 [this, seq](rclcpp::Client<mowbite_planner_interfaces::srv::PlanCoverage>::SharedFuture f) {
                                   OnPlanAnswer(seq, *f.get());
                                 })
            .request_id;
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
                result["paths"].size(), answer.poses.size(), (now() - pending.sent).seconds());
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
      for (auto it = pending_plans_.begin(); it != pending_plans_.end();) {
        if ((now() - it->second.sent).seconds() > plan_timeout_s_) {
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

  void PublishRpcError(const std::string& id, int code, const std::string& message) {
    mowbite_msgs::msg::RpcError msg;
    msg.code = static_cast<int16_t>(code);
    msg.message = message;
    msg.id = id;
    rpc_error_pub_->publish(msg);
  }

  static std::shared_ptr<mowbite_planner_interfaces::srv::PlanCoverage::Request> ToServiceRequest(const CoverageRequest& r) {
    auto request = std::make_shared<mowbite_planner_interfaces::srv::PlanCoverage::Request>();
    request->boundary = ToPolygon(r.boundary);
    for (const auto& ring : r.obstacles) request->obstacles.push_back(ToPolygon(ring));
    for (const auto& ring : r.drivable_areas) request->drivable_areas.push_back(ToPolygon(ring));
    request->mower_width = r.mower_width;
    request->overlap = r.overlap;
    request->perimeter_passes = r.perimeter_passes;
    request->use_angle = r.use_angle;
    request->angle = r.angle;
    request->overrides.push_back(IntegerParameter("lane_overlap_passes", r.lane_overlap_passes));
    if (r.perimeter_offset) request->overrides.push_back(DoubleParameter("perimeter_offset", *r.perimeter_offset));
    if (r.angle_offset_deg) request->overrides.push_back(DoubleParameter("angle_offset_deg", *r.angle_offset_deg));
    return request;
  }

  static CoverageAnswer ToAnswer(const mowbite_planner_interfaces::srv::PlanCoverage::Response& r) {
    CoverageAnswer answer;
    answer.success = r.success;
    answer.message = r.message;
    answer.angle = r.angle;
    answer.fingerprint = r.fingerprint;
    answer.poses.reserve(r.path.poses.size());
    for (const auto& pose : r.path.poses) answer.poses.push_back({pose.pose.position.x, pose.pose.position.y});
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
    const auto answer = [this, seq](bool ok, const std::string& id, const std::string& message) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // A late answer after the timeout is not the current one any more.
        if (seq != edit_seq_ || !edit_sent_) return;
        edit_sent_.reset();
        edit_request_id_.reset();
        core_.OnMapEditResult(ok, id, message);
      }
      Flush();
    };
    if (edit_is_dock_) {
      if (!set_dock_client_->service_is_ready()) {
        core_.OnMapEditResult(false, "", "mowbite_map does not offer map/set_dock");
        return;
      }
      auto request = std::make_shared<mowbite_msgs::srv::SetDock::Request>();
      request->x = edit.x;
      request->y = edit.y;
      request->heading = edit.heading;
      edit_sent_ = now();
      edit_request_id_ = set_dock_client_
                             ->async_send_request(request,
                                                  [answer](rclcpp::Client<mowbite_msgs::srv::SetDock>::SharedFuture f) {
                                                    const auto r = f.get();
                                                    answer(r->success, r->id, r->message);
                                                  })
                             .request_id;
      return;
    }
    if (!add_area_client_->service_is_ready()) {
      core_.OnMapEditResult(false, "", "mowbite_map does not offer map/add_area");
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
    edit_sent_ = now();
    edit_request_id_ = add_area_client_
                           ->async_send_request(request,
                                                [answer](rclcpp::Client<mowbite_msgs::srv::AddArea>::SharedFuture f) {
                                                  const auto r = f.get();
                                                  answer(r->success, r->id, r->message);
                                                })
                           .request_id;
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
    }
    std::vector<Event> events;
    bool actions_changed;
    bool stop;
    std::vector<ReasonChange> reasons;
    std::vector<std::string> notices;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      events = core_.TakeEvents();
      actions_changed = core_.TakeActionsChanged();
      stop = core_.TakeStopRequest();
      reasons = core_.TakeReasonChanges();
      notices = core_.TakeNotices();
    }
    for (const auto& line : notices) RCLCPP_INFO(get_logger(), "%s", line.c_str());
    if (stop) cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
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
      mowbite_msgs::msg::MqttPublish msg;
      msg.topic = "events/json";
      msg.payload = mowbite_bridge::EventPayload(event.type, event.details, now().seconds()).dump();
      publish_pub_->publish(msg);
    }
    if (actions_changed) PublishActions();
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
        mowbite_msgs::msg::ActionInfo info;
        info.action_id = action.id;
        info.action_name = action.name;
        info.enabled = action.enabled;
        msg.actions.push_back(info);
      }
      actions_pub_->publish(msg);
    }
  }

  // A mowing.plan request on its way to the planner.
  struct PendingPlan {
    std::string rpc_id;
    PlanQuestion question;
    rclcpp::Time sent{0, 0, RCL_ROS_TIME};
    int64_t request_id = 0;
  };

  std::mutex mutex_;
  LogicCore core_;
  MowingPlanner planner_;
  std::map<uint64_t, PendingPlan> pending_plans_;
  uint64_t plan_seq_ = 0;
  double plan_timeout_s_ = 30.0;
  std::optional<rclcpp::Time> last_hw_;
  std::optional<rclcpp::Time> last_pose_;
  double hw_timeout_s_;
  double pose_timeout_s_;
  double map_edit_timeout_s_;
  // The map edit on its way: when it went out, its request, and a number that tells a late answer apart.
  std::optional<rclcpp::Time> edit_sent_;
  std::optional<int64_t> edit_request_id_;
  bool edit_is_dock_ = false;
  uint64_t edit_seq_ = 0;

  rclcpp::Publisher<mowbite_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::ActionList>::SharedPtr actions_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorInfo>::SharedPtr sensor_info_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::MqttPublish>::SharedPtr publish_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<mowbite_msgs::srv::SetEmergency>::SharedPtr emergency_client_;
  rclcpp::Client<mowbite_msgs::srv::AddArea>::SharedPtr add_area_client_;
  rclcpp::Client<mowbite_msgs::srv::SetDock>::SharedPtr set_dock_client_;
  rclcpp::Client<mowbite_planner_interfaces::srv::PlanCoverage>::SharedPtr plan_client_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcResponse>::SharedPtr rpc_response_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcError>::SharedPtr rpc_error_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::RpcMethods>::SharedPtr rpc_methods_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::RpcRequest>::SharedPtr rpc_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr hw_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::AbsolutePose>::SharedPtr pose_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::PositionJump>::SharedPtr jump_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MapStatus>::SharedPtr map_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::TimerBase::SharedPtr overlay_timer_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
};

}  // namespace mowbite_logic

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_logic::LogicNode)
