#include "logic_node.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <rclcpp_components/register_node_macro.hpp>
#include "mowbite_bridge/event_builder.h"

namespace mowbite_logic {

namespace {

// S2 answers a release within 2 s (its own limit for the driver); later counts as no answer.
constexpr double kReleaseTimeoutS = 3.0;
// mowbite/hold goes out again this often while nothing changes.
constexpr double kHoldMirrorS = 30.0;
// Rounds of effects in one step: the callback, the next step of the job or docking, what their answers bring up.
constexpr int kMaxRounds = 4;
// A write of os-session.json or checkpoint.json longer than this (ms) gets a line with its time.
constexpr double kSlowWriteMs = 50.0;
// One write longer than this (s): an error line every 5 s until it ends.
constexpr double kWriterStuckS = 5.0;

// s a calibration program has to confirm a stop before its goal no longer blocks the next start.
constexpr double kCalibrationStopWaitS = 10.0;

using Wait = mowbite_hold::FileWriter::Wait;

using mowbite_hold::WriteFileSafely;

// An override for the planner, typed by its value (int an integer, double a double).
template <typename T>
rcl_interfaces::msg::Parameter Override(const std::string& name, T value) {
  return rclcpp::Parameter(name, value).to_parameter_msg();
}

Reason PlannerMissing() {
  return Reason("PLANNER_MISSING", "The planner is not running");
}

Reason FollowerMissing() {
  return Reason("FOLLOWER_MISSING", "The follower is not running");
}

}  // namespace

LogicNode::LogicNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mowbite_logic", options),
      journal_(mowbite_journal::OpenFromParameters(*this)),
      core_(journal_, LoadLogicConfig(*this), {{"api", mowbite_api::kApi}}),
      journal_watch_(journal_),
      geometry_(FactoryGeometry()) {
  journal_timer_ = create_wall_timer(std::chrono::seconds(10), [this] { journal_watch_.Check(get_logger()); });
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
  std::string problem;
  recorded_by_ = LoadRecordedBy(data_dir_, &problem);
  if (data_dir_.empty()) {
    RCLCPP_WARN(get_logger(), "No data_dir: recorded areas follow robot.geometry.set only until a restart");
  } else if (!problem.empty()) {
    RCLCPP_ERROR(get_logger(), "%s: recorded areas follow robot.geometry.set only until a restart", problem.c_str());
  }
  core_.Call<&LogicCore::SetTurningRadius>(geometry_.TurningRadius());
  // The blade: a lease of blade_lease (s), renewed every blade_renew (s) while the logic wants it to turn.
  blade_lease_s_ = declare_parameter("blade_lease", 1.0);
  blade_renew_s_ = declare_parameter("blade_renew", 0.5);
  // G22: no sign of the follower (goal taken, feedback every 0.5 s) for this long (s) while it has a path: the path
  // ends here, without waiting for the follower, and the mower and the blade stop.
  follower_timeout_s_ = declare_parameter("follower_timeout", 2.0);
  blade_client_ = create_client<mowbite_msgs::srv::SetBlade>("hw/blade");
  RestoreCheckpoint();
  // The kernel record of the platform ("<unix s> <reason>"), empty where it has none.
  JudgeRestart(declare_parameter("crash_record", std::string()));
  StartWriter();

  robot_state_pub_ = create_publisher<mowbite_msgs::msg::RobotState>("bridge/robot_state", 10);
  actions_pub_ = create_publisher<mowbite_msgs::msg::ActionList>("bridge/actions", LatchedQos(50));
  sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorSpec>("bridge/sensor_info", LatchedQos(100));
  sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("bridge/sensor_value", 100);
  publish_pub_ = create_publisher<mowbite_msgs::msg::MqttPublish>("bridge/publish", 100);
  // Every drive command goes to mowbite_safety (S2), the only writer of hw/cmd_vel, with its source: teleop while
  // recording, the follower while driving (LogicCore decides), stops of the logic.
  drive_pub_ = create_publisher<mowbite_msgs::msg::DriveCommand>("safety/drive", 10);

  // The app container rates the battery by /ll/board; any value except YardForce makes it show battery_percentage,
  // which the logic sends on every platform.
  params_pub_ = create_publisher<std_msgs::msg::String>("bridge/params",
                                                        rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local());
  std_msgs::msg::String board_msg;
  board_msg.data = nlohmann::json{{"node", "/ll"}, {"params", {{"board", "MowBite"}}}}.dump();
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
  recorded_body_client_ = create_client<mowbite_msgs::srv::SetRecordedBody>("map/set_recorded_body");
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
          core_.Call<&LogicCore::OnMowMap>(planner_.MowAreas(), planner_.MapProblem());
          core_.Call<&LogicCore::OnMapDock>(planner_.Dock());
          UpdateDrivableAreas();
        }
        Step();
      });
  // The calibration drive: the status of the program for calibration.check and .start, its commands while
  // calibrating.
  calibration_client_ = rclcpp_action::create_client<Calibrate>(this, "calibration/run");
  calibration_status_sub_ = create_subscription<mowbite_msgs::msg::CalibrationStatus>(
      "calibration/status", 10,
      [this](mowbite_msgs::msg::CalibrationStatus::ConstSharedPtr msg) { OnCalibrationStatus(*msg); });
  calibration_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "calibration/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        bool active;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          active = core_.View().CalibrationActive();
        }
        if (active) SendDrive(mowbite_msgs::msg::DriveCommand::SOURCE_CALIBRATION, *msg);
      });
  PublishRpcMethods();

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
          undocking = core_.View().Undocking();
          core_.Call<&LogicCore::OnGnssFix>(msg->x, msg->y);
        }
        if (undocking) Step();
      });
  // The sunset needs a safe wall clock and the datum in use.
  clock_sub_ = create_subscription<mowbite_msgs::msg::ClockStatus>(
      "time/clock", LatchedQos(1), [this](mowbite_msgs::msg::ClockStatus::ConstSharedPtr msg) {
        const double wall = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
        std::lock_guard<std::mutex> lock(mutex_);
        core_.Call<&LogicCore::OnClock>(msg->ok, msg->reason, wall - MonotonicSeconds());
      });
  datum_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps/datum", LatchedQos(1), [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        core_.Call<&LogicCore::OnDatum>(msg->latitude, msg->longitude);
      });
  jump_sub_ = create_subscription<mowbite_msgs::msg::PositionJump>(
      "localization/jump", 10, [this](mowbite_msgs::msg::PositionJump::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          core_.Call<&LogicCore::OnPositionJump>(msg->after.x, msg->after.y, msg->distance,
                                                 msg->reason == mowbite_msgs::msg::PositionJump::REASON_OUTLIERS);
        }
        Step();
      });
  // Only while recording areas, so that the journal does not take every measurement.
  wheel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "hw/wheel_odom", 10, [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!core_.View().AreaRecording()) return;
        core_.Call<&LogicCore::OnWheelOdom>(msg->twist.linear.x, MonotonicSeconds());
      });
  map_status_sub_ = create_subscription<mowbite_msgs::msg::MapStatus>(
      "map/status", LatchedQos(1), [this](mowbite_msgs::msg::MapStatus::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          core_.Call<&LogicCore::OnMapStatus>(ToDatum(msg->datum_state), msg->reason);
          planner_.SetMapStatus(msg->mowing_allowed,
                                msg->mowing_allowed ? Reason() : MapReason(ToDatum(msg->datum_state), msg->reason));
          core_.Call<&LogicCore::OnMowMap>(planner_.MowAreas(), planner_.MapProblem());
          UpdateDrivableAreas();
        }
        Step();
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
          active = core_.View().FollowerActive();
        }
        if (active) SendDrive(mowbite_msgs::msg::DriveCommand::SOURCE_FOLLOWER, *msg);
      });
  // Volatile and never latched: a restart replays no old press.
  button_sub_ = create_subscription<mowbite_msgs::msg::ButtonEvent>(
      "hw/button", rclcpp::QoS(10).reliable(), [this](const mowbite_msgs::msg::ButtonEvent& msg) { OnButton(msg); });
  caps_sub_ = create_subscription<mowbite_msgs::msg::HardwareCapabilities>(
      "hw/capabilities", LatchedQos(1), [this](mowbite_msgs::msg::HardwareCapabilities::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          core_.Call<&LogicCore::OnCapabilities>(!msg->safety_measurement_date.empty(), msg->charge_stop);
          blade_lease_max_s_ = msg->blade_lease_max_s;
          rain_sensor_ = msg->rain_sensor;
          TellRainWithoutSensor();
        }
        Step();
      });

  // Driving to a pose: the planner plans the way, the follower drives it. A ROS service only, not for the app yet.
  transit_max_outside_ = declare_parameter("transit_max_outside", 0.3);
  transit_client_ = create_client<PlanTransit>("mowbite_planner/plan_transit");
  follow_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
  drive_srv_ = create_service<mowbite_msgs::srv::DriveTo>(
      "logic/drive_to",
      [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<mowbite_msgs::srv::DriveTo::Request> request) {
        OnDriveTo(header, *request);
      });
  stop_drive_srv_ = create_service<std_srvs::srv::Trigger>(
      "logic/stop_drive", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          // Leaving the station for a drive is part of the drive.
          const bool undocking = core_.View().UndockingForDrive();
          response->success = undocking || (core_.View().Driving() && drive_.purpose == Purpose::kDriveTo);
          response->message = response->success ? "stopping" : "not driving";
          if (undocking) {
            core_.Call<&LogicCore::AbortUndocking>(Reason("STOPPED", "Stopped by logic/stop_drive"));
          } else if (response->success) {
            CancelDrive(Reason("STOPPED", "Stopped by logic/stop_drive"));
          }
        }
        Step();
      });

  for (const auto& sensor : core_.View().Sensors()) {
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
    Step();
  });
  blade_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { UpdateBlade(); });
  SetUpRuns();
  // What the core has for the node waits for the first callback; the actions go out at once.
  PublishActions();
  PublishIdleIfChanged();
  PublishHoldIfChanged();
  watch_ = mowbite_watch::Attach(*this);
}

// Its own stop: what is queued first, then a clean session, so the next start finds it clean.
LogicNode::~LogicNode() {
  writer_.reset();
  WriteSession(true);
}

void LogicNode::OnHardwareStatus(const mowbite_msgs::msg::HardwareStatus& msg) {
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
  hw.time_s = MonotonicSeconds();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    core_.Call<&LogicCore::OnHardwareStatus>(hw);
    last_hw_ = hw.time_s;
  }
  Step();
}

void LogicNode::OnPose(const mowbite_msgs::msg::RobotPose& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  core_.Call<&LogicCore::OnPose>(ToPose(msg));
  last_pose_ = MonotonicSeconds();
}

// What only the node knows against a start, for the core to weigh after its own reasons. Called with the mutex held.
Reason LogicNode::NodeProblemLocked(const std::string& command) const {
  Reason why;
  if (command == "mowing.start") {
    if (!plan_client_->service_is_ready() || !transit_client_->service_is_ready()) why = PlannerMissing();
    if (!follow_client_->action_server_is_ready()) why = FollowerMissing();
    // The node still ends a leg of the last job.
    if (drive_.active || job_plan_.active) {
      why = Reason("STILL_STOPPING", "The last job is still stopping, try again in a moment");
    }
  }
  if (command == "docking.start") {
    if (!transit_client_->service_is_ready()) why = PlannerMissing();
    if (why.empty() && !follow_client_->action_server_is_ready()) why = FollowerMissing();
    if (why.empty() && (drive_.active || job_plan_.active)) {
      why = Reason("STILL_STOPPING", "The last drive is still stopping, try again in a moment");
    }
  }
  return why;
}

// One way for both: an action runs the command it stands for, like its RPC without client and without an answer.
void LogicNode::OnAction(const std::string& action_id) {
  json params;
  const mowbite_api::Command* command = mowbite_api::CommandOfAction(action_id, &params);
  if (!command) {
    ActionResult result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result = core_.Call<&LogicCore::OnAction>(action_id);
    }
    if (result == ActionResult::kRejected) RCLCPP_WARN(get_logger(), "Action %s not possible", action_id.c_str());
    if (result == ActionResult::kUnknown) RCLCPP_DEBUG(get_logger(), "Ignoring action %s", action_id.c_str());
    Step();
    return;
  }
  CommandOutcome outcome;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    outcome = core_.Call<&LogicCore::Command>(command->name, params, NodeProblemLocked(command->name));
  }
  if (outcome.release) {
    RCLCPP_INFO(get_logger(), "Action %s", action_id.c_str());
    ReleaseEmergency();
  } else if (outcome.accepted) {
    RCLCPP_INFO(get_logger(), "Action %s", action_id.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Action %s refused: %s (%s)", action_id.c_str(), outcome.why.code.c_str(),
                outcome.why.message.c_str());
  }
  Step();
}

// A command of the catalog over RPC: -32602 for params that do not fit, the answer, or -32600 with the refusal;
// emergency.reset answers once the safety node did.
void LogicNode::OnCommandRpc(const mowbite_msgs::msg::RpcRequest& request, const mowbite_api::Command& command) {
  using mowbite_msgs::msg::RpcError;
  const json params = request.params.empty() ? json::object() : json::parse(request.params, nullptr, false);
  if (params.is_discarded()) {
    PublishRpcError(request.id, RpcError::ERROR_INVALID_PARAMS, "The parameters are not valid JSON");
    return;
  }
  CommandOutcome outcome;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    outcome = core_.Call<&LogicCore::Command>(command.name, params, NodeProblemLocked(command.name));
  }
  const std::string client = outcome.invalid.empty() && params.is_object() ? params.value("client", "") : "";
  const std::string by = client.empty() ? "" : " (client " + client + ")";
  if (!outcome.invalid.empty()) {
    RCLCPP_WARN(get_logger(), "%s: %s", command.name.c_str(), outcome.invalid.c_str());
    PublishRpcError(request.id, RpcError::ERROR_INVALID_PARAMS, outcome.invalid,
                    json{{"field", outcome.invalid_field}}.dump());
  } else if (outcome.release) {
    RCLCPP_INFO(get_logger(), "%s%s", command.name.c_str(), by.c_str());
    ReleaseEmergency(request.id);
  } else if (outcome.accepted) {
    RCLCPP_INFO(get_logger(), "%s%s: %s", command.name.c_str(), by.c_str(), outcome.result.dump().c_str());
    mowbite_msgs::msg::RpcResponse msg;
    msg.result = outcome.result.dump(-1, ' ', false, json::error_handler_t::replace);
    msg.id = request.id;
    rpc_response_pub_->publish(msg);
  } else {
    RCLCPP_WARN(get_logger(), "%s%s refused: %s (%s)", command.name.c_str(), by.c_str(), outcome.why.code.c_str(),
                outcome.why.message.c_str());
    PublishRpcError(request.id, RpcError::ERROR_INVALID_REQUEST, outcome.why.message,
                    outcome.data.dump(-1, ' ', false, json::error_handler_t::replace));
  }
  Step();
}

// Answers every emergency.reset waiting for the safety node: released, or the refusal and the id of its event.
// Called with the mutex held.
void LogicNode::AnswerReleaseLocked(bool released, const Reason& why, const std::string& event_id) {
  for (const auto& id : release_rpc_ids_) {
    if (released) {
      mowbite_msgs::msg::RpcResponse msg;
      msg.result = core_.View().CommandAnswer(json{{"released", true}}).dump();
      msg.id = id;
      rpc_response_pub_->publish(msg);
    } else {
      PublishRpcError(id, mowbite_msgs::msg::RpcError::ERROR_INVALID_REQUEST, why.message,
                      core_.View()
                          .RefusalData("EMERGENCY_RESET_REFUSED", why, event_id)
                          .dump(-1, ' ', false, json::error_handler_t::replace));
    }
  }
  release_rpc_ids_.clear();
}

// Called with the mutex held: refused here, the event and the answers to the RPCs waiting for it.
void LogicNode::RefuseReleaseLocked(const Reason& why) {
  const std::string event_id = core_.Call<&LogicCore::RefuseReset>(why);
  AnswerReleaseLocked(false, why, event_id);
}

// STOP let go after the gesture of the driver releases like emergency.reset, with its latch_count; held and canceled
// only show on the lights.
void LogicNode::OnButton(const mowbite_msgs::msg::ButtonEvent& msg) {
  using mowbite_msgs::msg::ButtonEvent;
  if (msg.button != ButtonEvent::BUTTON_STOP || msg.press != ButtonEvent::PRESS_LONG) return;
  bool emergency;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    emergency = core_.Call<&LogicCore::OnStopRelease>();
  }
  if (!emergency) {
    RCLCPP_INFO(get_logger(), "STOP let go after holding it, but there is no emergency to release");
    return;
  }
  RCLCPP_INFO(get_logger(), "STOP at the mower: releasing the emergency");
  ReleaseEmergency("", msg.latch_count, "STOP at the mower");
  Step();
}

// The answer comes from S2 once the driver answered; a refusal goes to the app as EMERGENCY_RESET_REFUSED and to the
// RPCs waiting for it (rpc_id: the one of emergency.reset, empty for the action and STOP). latch_count of a STOP
// gesture lets the driver refuse when another reason latched since, 0 releases without that check.
void LogicNode::ReleaseEmergency(const std::string& rpc_id, uint32_t latch_count, const std::string& reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!rpc_id.empty()) release_rpc_ids_.push_back(rpc_id);
  // The user looked at the mower: the restart hold goes first, the file before the flag, so S2 does not latch it
  // again; the release of S2 and the driver follows as for any emergency.
  if (core_.View().RestartHeld()) {
    std::string error;
    // A write of the hold still on its way lands before the removal.
    hold_text_.clear();
    while (writer_ && writer_->WaitFor(restart_hold_path_, std::chrono::seconds(1)) == Wait::kTimeout) CheckWriter();
    if (!mowbite_hold::RemoveRecordFile(restart_hold_path_, &error)) {
      RCLCPP_ERROR(get_logger(), "Restart hold stays: %s", error.c_str());
      RefuseReleaseLocked(Reason("RESTART_HOLD_STAYS", "The restart hold stays: " + error));
      return;
    }
    core_.Call<&LogicCore::SetRestartHold>(false);
    RCLCPP_INFO(get_logger(), "Restart hold released by emergency.reset");
  }
  if (!emergency_client_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "safety/release_emergency is not available, emergency stays");
    RefuseReleaseLocked(Reason("SAFETY_MISSING", "safety/release_emergency is not available"));
    return;
  }
  auto request = std::make_shared<mowbite_msgs::srv::SetEmergency::Request>();
  request->latch = false;
  request->reason = reason;
  request->latch_count = latch_count;
  // A second reset while one is pending replaces it; only the newest answer counts, for all that wait.
  if (release_sent_) emergency_client_->remove_pending_request(release_request_id_);
  release_sent_ = MonotonicSeconds();
  release_request_id_ =
      emergency_client_
          ->async_send_request(request,
                               [this](rclcpp::Client<mowbite_msgs::srv::SetEmergency>::SharedFuture future) {
                                 const auto response = future.get();
                                 {
                                   std::lock_guard<std::mutex> lock(mutex_);
                                   release_sent_.reset();
                                   if (response->success) {
                                     core_.Call<&LogicCore::OnReleaseAnswered>(MonotonicSeconds());
                                     AnswerReleaseLocked(true);
                                   } else {
                                     RCLCPP_WARN(get_logger(), "Emergency stays (%s): %s", response->code.c_str(),
                                                 response->message.c_str());
                                     RefuseReleaseLocked(ResetRefusal(response->code, response->message,
                                                                      response->value, response->limit));
                                   }
                                 }
                                 Step();
                               })
          .request_id;
  RCLCPP_INFO(get_logger(), "Emergency release requested");
}

void LogicNode::SendDrive(uint8_t source, const geometry_msgs::msg::Twist& twist) {
  mowbite_msgs::msg::DriveCommand msg;
  msg.source = source;
  msg.twist = twist;
  drive_pub_->publish(msg);
}

// A zero command at once instead of waiting for the cmd_vel watchdog of the driver.
void LogicNode::SendStop() {
  SendDrive(mowbite_msgs::msg::DriveCommand::SOURCE_STOP, geometry_msgs::msg::Twist());
}

void LogicNode::OnTeleop(const geometry_msgs::msg::Twist& msg) {
  std::optional<std::pair<double, double>> cmd;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd = core_.Call<&LogicCore::OnTeleop>(msg.linear.x, msg.angular.z, MonotonicSeconds());
  }
  if (!cmd) return;
  geometry_msgs::msg::Twist out;
  out.linear.x = cmd->first;
  out.angular.z = cmd->second;
  SendDrive(mowbite_msgs::msg::DriveCommand::SOURCE_TELEOP, out);
}

void LogicNode::OnDriveTo(const std::shared_ptr<rmw_request_id_t>& header,
                          const mowbite_msgs::srv::DriveTo::Request& request) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Ring> drivable;
    std::vector<Ring> obstacles;
    // What the core checks first (busy, emergency, position), then what only the node knows.
    Reason why = core_.View().DriveProblem();
    if (why.empty()) why = planner_.TransitAreas(&drivable, &obstacles);
    if (why.empty() && !transit_client_->service_is_ready()) why = PlannerMissing();
    if (why.empty() && !follow_client_->action_server_is_ready()) why = FollowerMissing();
    // The follower may still be letting go of the last path (right after docking): a new one would be refused.
    if (why.empty() && (drive_.active || job_plan_.active)) {
      why = Reason("STILL_STOPPING", "The last drive is still stopping, try again in a moment");
    }
    const bool from_dock = why.empty() && core_.View().DockContact();
    if (!why.empty()) {
      core_.Call<&LogicCore::RefuseDrive>(why);
    } else if (from_dock) {
      // From the docking station the reference check and undocking come first; the request is answered once the
      // drive is on its way after them, or with why not.
      core_.Call<&LogicCore::CheckDockBeforeDrive>(request.x, request.y);
      dock_drive_ = DockDrive{header, request};
      RCLCPP_INFO(get_logger(), "Drive to (%.2f, %.2f) from the docking station: checking the position first",
                  request.x, request.y);
    } else {
      why = core_.Call<&LogicCore::StartDrive>(request.x, request.y);
    }
    if (!why.empty()) {
      RCLCPP_WARN(get_logger(), "Drive refused: %s (%s)", why.code.c_str(), why.message.c_str());
      RespondDrive(header, false, why.message);
    } else if (!from_dock) {
      PlanDrive(request, header);
    }
  }
  Step();
}

// Plans the way of the drive the core has started (state DRIVING). Called with the mutex held.
void LogicNode::PlanDrive(const mowbite_msgs::srv::DriveTo::Request& request,
                          const std::shared_ptr<rmw_request_id_t>& header) {
  std::vector<Ring> drivable;
  std::vector<Ring> obstacles;
  Reason why = planner_.TransitAreas(&drivable, &obstacles);
  if (why.empty() && !transit_client_->service_is_ready()) why = PlannerMissing();
  if (!why.empty()) {
    RCLCPP_WARN(get_logger(), "Drive ended: %s (%s)", why.code.c_str(), why.message.c_str());
    RespondDrive(header, false, why.message);
    core_.Call<&LogicCore::OnDriveEnded>(false, why);
    return;
  }
  const RobotStateData state = core_.View().RobotState();
  auto plan = std::make_shared<PlanTransit::Request>();
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
                               [this, seq](rclcpp::Client<PlanTransit>::SharedFuture f) { OnTransit(seq, *f.get()); })
          .request_id;
  RCLCPP_INFO(get_logger(), "Drive to (%.2f, %.2f): planning the way", request.x, request.y);
}

// The drivable areas of the map for the core, which lets undocking end and docking begin only in one of them. Called
// with the mutex held.
void LogicNode::UpdateDrivableAreas() {
  std::vector<Ring> drivable;
  std::vector<Ring> obstacles;
  const Reason problem = planner_.TransitAreas(&drivable, &obstacles);
  if (!problem.empty()) {
    drivable.clear();
    obstacles.clear();
  }
  core_.Call<&LogicCore::OnDrivableAreas>(std::move(drivable), std::move(obstacles), problem);
}

void LogicNode::OnTransit(uint64_t seq, const PlanTransit::Response& response) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (seq != drive_.seq || !drive_.planning) return;
    drive_.planning = false;
    if (!response.success) {
      drive_.no_way = response.code == "NO_WAY" || response.code == "START_OUTSIDE";
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
  Step();
}

// Hands a path to the follower for the current drive. Called with the mutex held.
void LogicNode::SendFollow(const FollowPath::Goal& goal, const std::string& what) {
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
    Step();
  };
  options.feedback_callback = [this, seq](FollowHandle::SharedPtr,
                                          const std::shared_ptr<const FollowPath::Feedback> feedback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (seq != drive_.seq) return;
    drive_.follower_sign = MonotonicSeconds();
    drive_.follower_index = feedback->index;
    if (drive_.purpose != Purpose::kMow) return;
    core_.Call<&LogicCore::OnJobProgress>(drive_.PathIndex(feedback->index));
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
      // Stopped from here: the cause, not the cancel of the follower nor its own stop for the same emergency.
      Reason why = FollowerEndReason(FollowerCode(reason), message, drive_.cancel_reason);
      if (reason == FollowPath::Result::REASON_CANCELED && !drive_.cancel_reason) {
        // A cancel the logic did not ask for (G22): the path stops for the user, as when the follower does not answer.
        why = Reason("FOLLOWER_MISSING", "The follower dropped the path (" + message + ")");
        reason = FollowPath::Result::REASON_TIMEOUT;
      }
      EndDrive(reason == FollowPath::Result::REASON_DONE, why, reason, index);
    }
    Step();
  };
  drive_.follower_sign = MonotonicSeconds();
  drive_.follower_index = goal.start_index;
  follow_client_->async_send_goal(goal, options);
}

// G22: a follower that stops answering ends its path here, without waiting for its result, so that the logic does
// not keep the blade turning on the spot. True when it ended one. Called with the mutex held.
bool LogicNode::CheckFollowerLocked() {
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
void LogicNode::CancelDrive(const Reason& why) {
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
void LogicNode::EndDrive(bool arrived, const Reason& why, uint8_t reason, uint32_t index) {
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
    core_.Call<&LogicCore::OnDriveEnded>(arrived, why);
    return;
  }
  if (ended.purpose == Purpose::kUndock) {
    core_.Call<&LogicCore::OnUndockEnded>(ToLegEnd(reason), why);
    return;
  }
  if (ended.purpose == Purpose::kDock) {
    core_.Call<&LogicCore::OnDockingLegEnded>(ToLegEnd(reason), why);
    return;
  }
  // In a job the way to the start of a path failed, or the follower did not take a path of the plan; a planner
  // that gave no answer goes to the core as it is.
  Reason leg = why;
  const bool way = why.code == "TRANSIT_FAILED" || why.code == "FOLLOWER_REFUSED";
  if (way) leg.code = ended.purpose == Purpose::kApproach ? "APPROACH_FAILED" : "INVALID";
  // Only an approach asks the planner: its "no way from here" lets the core drive on along a skipped path.
  core_.Call<&LogicCore::OnLegEnded>(ended.no_way ? LegEnd::kNoWay : ToLegEnd(reason), ended.PathIndex(index), leg);
}

const char* LogicNode::PurposeName(int purpose) {
  switch (purpose) {
    case 1: return "Approach";
    case 2: return "Mow path";
    case 3: return "Undocking";
    case 4: return "Docking";
    default: return "Drive";
  }
}

// The next step of the job, when nothing is on its way for it. A plan asked for by a job that ended is dropped:
// nothing waits for its answer, not the way home either. Called with the mutex held.
void LogicNode::PumpJob() {
  if (job_plan_.active && !core_.View().Mowing()) {
    plan_client_->remove_pending_request(job_plan_.request_id);
    job_plan_ = JobPlan();
  }
  if (drive_.active || job_plan_.active || !core_.View().Mowing()) return;
  JobStep step = core_.Call<&LogicCore::NextJobStep>(MonotonicSeconds());
  switch (step.kind) {
    case JobStep::Kind::kNone: return;
    case JobStep::Kind::kPlan: PlanJobArea(step.area_id); return;
    case JobStep::Kind::kApproach: Approach(step.goal); return;
    case JobStep::Kind::kSkippedPath:
    case JobStep::Kind::kFollow: FollowJobPath(step.path, step.start_index); return;
  }
}

void LogicNode::PlanJobArea(const std::string& area_id) {
  PlanQuestion question = planner_.Prepare(json{{"area_id", area_id}});
  question.request.angle_offset_deg = core_.View().PlanAngle();
  if (!question.ok) {
    core_.Call<&LogicCore::OnJobPlanFailed>(Reason("PLANNING_FAILED", question.message));
    return;
  }
  if (!plan_client_->service_is_ready()) {
    core_.Call<&LogicCore::OnJobPlanFailed>(Reason("PLANNING_FAILED", "The planner is not running"));
    return;
  }
  RCLCPP_INFO(get_logger(), "Mowing: planning area %s", area_id.c_str());
  const uint64_t seq = ++plan_seq_;
  job_plan_.active = true;
  job_plan_.seq = seq;
  job_plan_.sent = MonotonicSeconds();
  job_plan_.request_id =
      plan_client_
          ->async_send_request(
              ToServiceRequest(question.request),
              [this, seq](rclcpp::Client<mowbite_planner_interfaces::srv::PlanCoverage>::SharedFuture f) {
                OnJobPlan(seq, *f.get());
              })
          .request_id;
}

void LogicNode::OnJobPlan(uint64_t seq, const mowbite_planner_interfaces::srv::PlanCoverage::Response& response) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!job_plan_.active || seq != job_plan_.seq) return;
    job_plan_ = JobPlan();
    const CoverageAnswer answer = ToAnswer(response);
    if (!answer.success) {
      core_.Call<&LogicCore::OnJobPlanFailed>(Reason("PLANNING_FAILED", answer.message));
    } else {
      std::vector<JobPath> paths = JobPaths(answer);
      RCLCPP_INFO(get_logger(), "Mowing: %zu paths from %zu poses", paths.size(), answer.poses.size());
      core_.Call<&LogicCore::OnJobPlanned>(std::move(paths), answer.fingerprint);
    }
  }
  Step();
}

// Drives to the start of a path over the drivable areas, the blade off. Called with the mutex held.
void LogicNode::Approach(const JobPose& goal) {
  Reason why = PlanWay(Purpose::kApproach, goal.x, goal.y, goal.heading);
  if (why.code == "PLANNER_MISSING") why = Reason("APPROACH_FAILED", why.message);
  if (!why.empty()) {
    core_.Call<&LogicCore::OnLegEnded>(LegEnd::kFailed, 0, why);
    return;
  }
  RCLCPP_INFO(get_logger(), "Mowing: approaching (%.2f, %.2f)", goal.x, goal.y);
}

// Plans the way to (x, y) facing heading over the drivable areas, which the follower then drives (OnTransit): the
// approach to a path of the job or to the docking station. Empty, or why not. Called with the mutex held.
Reason LogicNode::PlanWay(Purpose purpose, double x, double y, double heading) {
  std::vector<Ring> drivable;
  std::vector<Ring> obstacles;
  const Reason why = planner_.TransitAreas(&drivable, &obstacles);
  if (!why.empty()) return why;
  if (!transit_client_->service_is_ready()) return PlannerMissing();
  const RobotStateData state = core_.View().RobotState();
  auto plan = std::make_shared<PlanTransit::Request>();
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
                               [this, seq](rclcpp::Client<PlanTransit>::SharedFuture f) { OnTransit(seq, *f.get()); })
          .request_id;
  return {};
}

// The next leg of docking, once nothing else is on its way. Called with the mutex held.
void LogicNode::PumpDocking() {
  if (drive_.active || job_plan_.active) return;
  if (auto leg = core_.Call<&LogicCore::TakeDockingLeg>()) SendDockingLeg(*leg);
}

// The way to the docking station over the drivable areas, or a line for the follower: besides the drivable areas
// base_link may be in the corridor along it, if it has one. Called with the mutex held.
void LogicNode::SendDockingLeg(const DockingLeg& leg) {
  if (leg.kind == DockingLeg::Kind::kApproach) {
    const Reason why = PlanWay(Purpose::kDock, leg.goal.x, leg.goal.y, leg.goal.heading);
    if (!why.empty()) {
      core_.Call<&LogicCore::OnDockingLegEnded>(LegEnd::kFailed, why);
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
  if (!leg.corridor.empty()) goal.allowed_areas.push_back(ToPolygon(leg.corridor));
  for (const auto& ring : obstacles) goal.obstacles.push_back(ToPolygon(ring));
  goal.max_speed = static_cast<float>(leg.speed);
  goal.stall_time = static_cast<float>(leg.stall_time);
  goal.straight = leg.straight;
  goal.reverse = leg.reverse;
  goal.end_tolerance = static_cast<float>(leg.end_tolerance);
  goal.max_cross_track = static_cast<float>(leg.max_cross_track);
  drive_ = Drive();
  drive_.purpose = Purpose::kDock;
  drive_.seq = ++drive_seq_;
  drive_.active = true;
  drive_.sent = MonotonicSeconds();
  const auto& from = leg.poses.front();
  const auto& to = leg.poses.back();
  // A leg without a speed of its own drives at the follower's.
  char speed[24] = "";
  if (leg.speed > 0) std::snprintf(speed, sizeof(speed), " at %.2f m/s", leg.speed);
  char text[128];
  std::snprintf(text, sizeof(text), "%s, %.2f m %s to (%.2f, %.2f)%s", DockingLegName(leg.kind),
                std::hypot(to.x - from.x, to.y - from.y), leg.reverse ? "backwards" : "forwards", to.x, to.y, speed);
  SendFollow(goal, text);
}

// Hands a path of the plan to the follower from start_index on. Poses on the one before are left out, the follower
// refuses them; feedback and result are counted back in poses of the path. Called with the mutex held.
void LogicNode::FollowJobPath(const JobPath& path, size_t start_index) {
  std::vector<Ring> drivable;
  std::vector<Ring> obstacles;
  const Reason why = planner_.TransitAreas(&drivable, &obstacles);
  if (!why.empty()) {
    core_.Call<&LogicCore::OnLegEnded>(LegEnd::kFailed, start_index, why);
    return;
  }
  drive_ = Drive();
  drive_.purpose = Purpose::kMow;
  FollowPath::Goal goal;
  goal.start_index = 0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const JobPose& p = path.poses[i];
    if (!goal.poses.empty() &&
        std::hypot(p.x - goal.poses.back().position.x, p.y - goal.poses.back().position.y) < 1e-6) {
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
    core_.Call<&LogicCore::OnLegEnded>(LegEnd::kDone, path.poses.empty() ? 0 : path.poses.size() - 1,
                                       Reason("DONE", ""));
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
void LogicNode::UpdateBlade() {
  bool stalled;
  bool calibration_ended;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stalled = CheckFollowerLocked();
    // The calibration program's sign of life and the battery while it drives.
    calibration_ended = core_.Call<&LogicCore::TickCalibration>(MonotonicSeconds());
    UpdateBladeLocked();
  }
  if (!stalled && !calibration_ended) return;
  // One zero command: the follower sends none any more, the driver's watchdog would stop the wheels only later.
  if (stalled) SendStop();
  Step();
}

void LogicNode::UpdateBladeLocked() {
  const bool wanted = core_.View().BladeWanted();
  const double t = MonotonicSeconds();
  if (!wanted) {
    if (!blade_on_requested_) return;
    blade_on_requested_ = false;
    SendBlade(false);
    return;
  }
  if (blade_on_requested_ && t - blade_sent_s_ < blade_renew_s_) return;
  if (!blade_client_->service_is_ready()) {
    core_.Call<&LogicCore::OnBladeRefused>("hw/blade is not available", 0);
    return;
  }
  blade_on_requested_ = true;
  blade_sent_s_ = t;
  SendBlade(true);
}

// Called with the mutex held.
void LogicNode::SendBlade(bool on) {
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
      core_.Call<&LogicCore::OnBladeRefused>(response->message, static_cast<int>(response->emergency_reason));
    }
    Step();
  });
}

// The restart hold (G35) from os-session.json of the end before and the kernel record: it holds from here on, the
// event waits for the bridge. A hold file from before stands, whatever this start finds.
void LogicNode::JudgeRestart(const std::string& crash_record) {
  if (data_dir_.empty()) return;
  session_path_ = data_dir_ + "/" + mowbite_hold::kSessionFile;
  restart_hold_path_ = data_dir_ + "/" + mowbite_hold::kRestartHoldFile;
  boot_id_ = mowbite_hold::ReadBootId();
  if (boot_id_.empty()) RCLCPP_ERROR(get_logger(), "No boot id: every end before counts as one of another boot");
  const mowbite_hold::SessionFile session = mowbite_hold::ReadSessionFile(session_path_);
  const mowbite_hold::Verdict verdict =
      mowbite_hold::JudgeStart(session, boot_id_, mowbite_hold::ReadKernelRecord(crash_record),
                               mowbite_hold::ReadStall(data_dir_ + "/" + mowbite_hold::kExitNoteFile));
  bool held = mowbite_hold::RestartHoldPresent(restart_hold_path_);
  if (verdict.hold && !held) {
    std::string error;
    const std::string text = mowbite_hold::RestartHoldText(verdict, WallSeconds());
    if (!mowbite_hold::WriteRecordFile(restart_hold_path_, text, &error)) {
      RCLCPP_ERROR(get_logger(), "Restart hold in memory until it can be written: %s", error.c_str());
      hold_text_ = text;
      hold_failed_ = true;
    }
  }
  held = held || verdict.hold;
  core_.Call<&LogicCore::SetRestartHold>(held);
  if (verdict.cause != mowbite_hold::Cause::kNone) {
    const auto& stall = verdict.stalled;
    restart_report_ = RestartReport{
        mowbite_hold::CauseName(verdict.cause),
        verdict.detail,
        verdict.crashed_at,
        verdict.state_before,
        verdict.job_id_before,
        held,
        stall ? json{{"node", stall->node}, {"seconds", stall->seconds}, {"limit", stall->limit}} : json()};
    const std::string line = mowbite_hold::RestartLine(verdict, held);
    if (held) {
      RCLCPP_ERROR(get_logger(), "%s", line.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", line.c_str());
    }
  } else if (held) {
    RCLCPP_ERROR(get_logger(), "Held after an unexpected restart before: %s", restart_hold_path_.c_str());
  }
  WriteSession(false);
}

// os-session.json with state, job, emergency and charging now, when they changed, clean is new or the last write
// failed. Without the writer (start and stop) here; else through it, waited for only when this record begins a
// restart hold, so it is on disk before what it guards acts (G35). False when that wait failed. Without the mutex.
bool LogicNode::WriteSession(bool clean) {
  if (session_path_.empty()) return true;
  mowbite_hold::SessionRecord record;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const RobotStateData state = core_.View().RobotState();
    record = {mowbite_hold::SessionState(state.state, state.sub_state),
              core_.View().JobId(),
              boot_id_,
              WallSeconds(),
              clean,
              state.emergency,
              state.is_charging};
  }
  if (session_failed_) session_.reset();
  if (session_ && session_->state == record.state && session_->job_id == record.job_id && session_->clean == clean &&
      session_->emergency == record.emergency && session_->charging == record.charging) {
    return true;
  }
  const std::string text = mowbite_hold::SessionText(record);
  session_ = record;
  if (!mowbite_hold::HoldsOnRestart(record)) session_holds_ = false;
  if (!writer_) {
    std::string error;
    session_failed_ = !mowbite_hold::WriteRecordFile(session_path_, text, &error);
    if (session_failed_) RCLCPP_ERROR(get_logger(), "Session: %s", error.c_str());
    return true;
  }
  const bool wait = mowbite_hold::MustWaitForSession(session_holds_, record);
  writer_->Submit(session_path_, text);
  if (!wait) return true;
  Wait result;
  while ((result = writer_->WaitFor(session_path_, std::chrono::seconds(1))) == Wait::kTimeout) CheckWriter();
  session_holds_ = result == Wait::kWritten;
  return session_holds_;
}

// The writer of os-session.json and checkpoint.json, after the start has written its session.
void LogicNode::StartWriter() {
  if (data_dir_.empty()) return;
  writer_ = std::make_unique<mowbite_hold::FileWriter>(
      [this](const std::string& path, const std::string& text, std::string* error) {
        return path == session_path_ || path == restart_hold_path_ ? mowbite_hold::WriteRecordFile(path, text, error)
                                                                   : WriteFileSafely(path, text, error);
      },
      [this](const mowbite_hold::FileWriter::Done& done) { OnWritten(done); });
}

// In the writer thread: no core, no mutex. What of the restart hold failed is written again at the next step.
void LogicNode::OnWritten(const mowbite_hold::FileWriter::Done& done) {
  const double ms = std::chrono::duration<double, std::milli>(done.took).count();
  if (done.path == session_path_) session_failed_ = !done.ok;
  if (done.path == restart_hold_path_) hold_failed_ = !done.ok;
  if (!done.ok && (done.path == session_path_ || done.path == restart_hold_path_)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 60000, "Restart hold: %s", done.error.c_str());
  } else if (!done.ok) {
    RCLCPP_ERROR(get_logger(), "Checkpoint: %s", done.error.c_str());
  }
  if (ms > kSlowWriteMs) RCLCPP_INFO(get_logger(), "Wrote %s in %.0f ms", done.path.c_str(), ms);
}

// An error line while one write has run longer than kWriterStuckS.
void LogicNode::CheckWriter() {
  if (!writer_) return;
  const double busy = std::chrono::duration<double>(writer_->Busy()).count();
  if (busy > kWriterStuckS) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Writing in %s has hung for %.0f s", data_dir_.c_str(),
                          busy);
  }
}

// A restart hold not yet on the disk again; the file of the restart hold that cannot be written for the core, once
// the bridge listens, so its event reaches the history. Called with the mutex held.
void LogicNode::TellUnwritableFileLocked() {
  if (!hold_failed_) hold_text_.clear();
  if (!hold_text_.empty() && writer_) writer_->Submit(restart_hold_path_, hold_text_);
  std::string file = session_failed_ ? mowbite_hold::kSessionFile : "";
  if (file.empty() && !hold_text_.empty()) file = mowbite_hold::kRestartHoldFile;
  if (file == unwritable_told_ || (!file.empty() && !BridgeListens())) return;
  unwritable_told_ = file;
  core_.Call<&LogicCore::SetUnwritableFile>(file);
}

// Whether the bridge itself takes bridge/publish, not only a tool or test that listens there; it stays so, the
// bridge lives in this process.
bool LogicNode::BridgeListens() {
  if (bridge_listens_ || publish_pub_->get_subscription_count() == 0) return bridge_listens_;
  for (const auto& info : get_subscriptions_info_by_topic(publish_pub_->get_topic_name())) {
    if (info.node_name() == "mowbite_bridge") bridge_listens_ = true;
  }
  return bridge_listens_;
}

// UNEXPECTED_RESTART once the bridge listens, so the event reaches events/json and its history.
void LogicNode::ReportRestartWhenHeard() {
  if (!restart_report_ || !BridgeListens()) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    core_.Call<&LogicCore::ReportRestart>(*restart_report_);
  }
  restart_report_.reset();
  Step();
}

// <data_dir>/checkpoint.json, written to a new file and renamed, so a crash never leaves half a checkpoint.
std::string LogicNode::CheckpointPath() const {
  return data_dir_.empty() ? "" : data_dir_ + "/checkpoint.json";
}

void LogicNode::RestoreCheckpoint() {
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
  core_.Call<&LogicCore::RestoreJob>(*progress);
}

void LogicNode::SaveCheckpoint(const std::string& text) {
  const std::string path = CheckpointPath();
  if (path.empty() || !writer_) return;
  writer_->Submit(path, text);
}

void LogicNode::RespondDrive(const std::shared_ptr<rmw_request_id_t>& header, bool success,
                             const std::string& message) {
  if (!header) return;
  mowbite_msgs::srv::DriveTo::Response response;
  response.success = success;
  response.message = message;
  drive_srv_->send_response(*header, response);
}

void LogicNode::OnStateTimer() {
  RobotStateData state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double t = MonotonicSeconds();
    if (!last_hw_ || t - *last_hw_ > hw_timeout_s_) core_.Call<&LogicCore::OnHardwareTimeout>();
    if (!last_pose_ || t - *last_pose_ > pose_timeout_s_) core_.Call<&LogicCore::OnPoseTimeout>();
    core_.Call<&LogicCore::OnTick>(t);
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
      core_.Call<&LogicCore::OnMapEditResult>(false, "", "NO_ANSWER",
                                              "No answer from mowbite_map; look at the map before saving again");
    }
    if (body_sent_ && t - *body_sent_ > map_edit_timeout_s_) {
      body_sent_.reset();
      if (body_request_id_) recorded_body_client_->remove_pending_request(*body_request_id_);
      body_request_id_.reset();
      core_.Call<&LogicCore::RecordedBodyKept>(-1, "NO_ANSWER", "No answer from mowbite_map");
    }
    if (dock_check_sent_ && t - *dock_check_sent_ > dock_check_timeout_s_) {
      set_heading_client_->remove_pending_request(dock_check_request_id_);
      FinishDockCheck(false, 0, 0, 999.0f, "No answer from the localization (localization/set_heading)");
    }
    if (release_sent_ && t - *release_sent_ > kReleaseTimeoutS) {
      emergency_client_->remove_pending_request(release_request_id_);
      release_sent_.reset();
      RCLCPP_WARN(get_logger(), "Emergency stays: no answer from safety/release_emergency");
      RefuseReleaseLocked(Reason("NO_ANSWER", "No answer to the release from mowbite_safety"));
    }
    if (charge_sent_ && t - *charge_sent_ > charge_timeout_s_) {
      charge_client_->remove_pending_request(charge_request_id_);
      charge_sent_.reset();
      core_.Call<&LogicCore::OnChargeOff>(false, "no answer from hw/charge");
    }
    ExpireHoldLocked(t);
    // A program that never confirmed a stop does not block the next start for long.
    if (calibration_.active && calibration_.cancel_s > 0 && t - calibration_.cancel_s > kCalibrationStopWaitS) {
      calibration_ = Calibration();
    }
    state = core_.View().RobotState();
  }
  if (calibration_client_->action_server_is_ready() != calibration_offered_) PublishRpcMethods();
  Step();
  ReportRestartWhenHeard();
  MirrorHold();
  CheckWriter();
  robot_state_pub_->publish(ToMessage(state));
}

void LogicNode::OnRpcRequest(const mowbite_msgs::msg::RpcRequest& request) {
  // The bridge hands every registered method to every node, the others answer theirs.
  if (const mowbite_api::Command* command = mowbite_api::FindCommand(request.method); command && Ours(*command)) {
    OnCommandRpc(request, *command);
    return;
  }
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
  if (request.method == "system.hold") {
    OnSystemHold(request);
    return;
  }
  if (request.method == "calibration.check") {
    OnCalibrationCheck(request);
    return;
  }
  if (request.method == "calibration.start") {
    OnCalibrationStart(request);
    return;
  }
  if (request.method == "calibration.abort") {
    OnCalibrationAbort(request);
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
          ->async_send_request(
              ToServiceRequest(question.request),
              [this, seq](rclcpp::Client<mowbite_planner_interfaces::srv::PlanCoverage>::SharedFuture f) {
                OnPlanAnswer(seq, *f.get());
              })
          .request_id;
}

// Only in IDLE and all or nothing; a change counts once it is saved.
void LogicNode::OnParamsSet(const mowbite_msgs::msg::RpcRequest& request) {
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
    if (core_.View().HoldUntil() > 0) {
      PublishHoldRefusal(request.id, "Settings change after it");
      return;
    }
    if (!core_.View().Idle()) {
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
  Step();
  mowbite_msgs::msg::RpcResponse msg;
  msg.result = result.applied.dump();
  msg.id = request.id;
  rpc_response_pub_->publish(msg);
}

// system.hold {"seconds": 0..1800} of the updater: granted only in IDLE with nothing on its way, else none; 0 lets go
// and a new call replaces the old one. The answer holds what the gate of the updater looks at.
void LogicNode::OnSystemHold(const mowbite_msgs::msg::RpcRequest& request) {
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
  bool schedule_blocks = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double t = MonotonicSeconds();
    const int64_t wall = WallSeconds();
    schedule_blocks = ScheduleBlocksHoldLocked(wall);
    if (wanted > 0 && core_.View().HoldAllowed() && !schedule_blocks) {
      const mowbite_hold::HoldRecord record{wall + wanted, wall};
      if (hold_path_.empty() || mowbite_hold::WriteHoldFile(hold_path_, record, &error)) {
        hold_.Start(record, t, wall);
        core_.Call<&LogicCore::SetHold>(record.until);
      }
    } else if (hold_path_.empty() || mowbite_hold::RemoveHoldFile(hold_path_, &error)) {
      hold_.Clear();
      core_.Call<&LogicCore::SetHold>(0);
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
    } else if (wanted > 0 && schedule_blocks) {
      RCLCPP_WARN(get_logger(), "system.hold %lld: not granted, the schedule starts at %s (unix s)",
                  static_cast<long long>(wanted), answer["next_start"].dump().c_str());
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
  Step();
  MirrorHold();
}

// Called with the mutex held.
json LogicNode::HoldAnswerLocked() const {
  const RobotStateData state = core_.View().RobotState();
  return json{{"state", state.state},
              {"is_charging", state.is_charging},
              {"hold_until", HoldUntilLocked()},
              {"sub_state", state.sub_state},
              {"idle", core_.View().HoldAllowed() && !ScheduleBlocksHoldLocked(WallSeconds())},
              {"emergency_reason", state.emergency_reason},
              {"next_start", next_start_ > 0 ? json(next_start_) : json()}};
}

// unix s while the logic holds, else 0. Called with the mutex held.
int64_t LogicNode::HoldUntilLocked() const {
  return core_.View().HoldUntil();
}

// A change of data in the hold of the logic: a rollback of the update would drop it without a word. The driver's own
// hold, still shown for up to 0.5 s after it ended, refuses none. Called with the mutex held.
void LogicNode::PublishHoldRefusal(const std::string& id, const std::string& what) {
  const int64_t until = HoldUntilLocked();
  PublishRpcError(id, mowbite_msgs::msg::RpcError::ERROR_HOLD, "An update holds the mower. " + what,
                  json{{"reason", "HOLD"}, {"until", until}}.dump());
}

// A hold from before the restart stands for what is left of it, at most kMaxHoldSeconds from now; a file that cannot
// be read holds as long and is removed after it, like the drivers hold until it is gone.
void LogicNode::RestoreHold() {
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
  core_.Call<&LogicCore::SetHold>(hold_.record()->until);
  RCLCPP_INFO(get_logger(), "Held for an update until %lld (unix s): nothing starts",
              static_cast<long long>(hold_.record()->until));
}

// The hold ran out: S3 lets go and the file goes, again at the next tick if it cannot. Called with the mutex held.
void LogicNode::ExpireHoldLocked(double t) {
  if (!hold_.record() || hold_.Active(t)) return;
  if (core_.View().HoldUntil() > 0) {
    core_.Call<&LogicCore::SetHold>(0);
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

void LogicNode::PublishHoldIfChanged() {
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
void LogicNode::MirrorHold() {
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

// The commands of the catalog are the logic's: they take client.
bool LogicNode::Ours(const mowbite_api::Command& command) {
  return std::any_of(command.params.begin(), command.params.end(),
                     [](const mowbite_api::Field& param) { return param.name == "client"; });
}

// The methods of the logic; calibration.* only while calibration/run is there.
void LogicNode::PublishRpcMethods() {
  mowbite_msgs::msg::RpcMethods methods;
  methods.node_id = "mowbite_logic";
  for (const auto& method : mowbite_api::Methods()) {
    if (method.node == "mowbite_logic" && method.offered.empty()) methods.methods.push_back(method.name);
  }
  for (const auto& command : mowbite_api::Commands()) {
    // "Home" from IDLE, only where the OS docks (contacts at the back).
    if (!Ours(command) || (command.name == "docking.start" && !core_.View().CanDock())) continue;
    methods.methods.push_back(command.name);
  }
  calibration_offered_ = calibration_client_->action_server_is_ready();
  if (calibration_offered_) {
    for (const auto& method : mowbite_api::Methods()) {
      if (method.node == "mowbite_logic" && method.name.rfind("calibration.", 0) == 0) {
        methods.methods.push_back(method.name);
      }
    }
  }
  rpc_methods_pub_->publish(methods);
}

void LogicNode::OnCalibrationStatus(const mowbite_msgs::msg::CalibrationStatus& msg) {
  CalibrationStatus status;
  for (const auto& problem : msg.problems) {
    Reason why(problem.reason, problem.message);
    const json fields = json::parse(problem.fields.empty() ? "{}" : problem.fields, nullptr, false);
    if (fields.is_object()) why.fields = fields;
    status.problems.push_back(why);
  }
  status.heading = msg.heading;
  status.lane_length = msg.lane_length;
  status.strip_length = msg.strip_length;
  status.strip_width = msg.strip_width;
  status.running = msg.running;
  std::lock_guard<std::mutex> lock(mutex_);
  core_.Call<&LogicCore::OnCalibrationStatus>(status, MonotonicSeconds());
}

// What the core finds against a calibration drive, then what only the node knows. Called with the mutex held.
std::vector<Reason> LogicNode::CalibrationProblemsLocked(double t) const {
  std::vector<Reason> problems = core_.View().CalibrationProblems(t);
  const auto has = [&problems](const std::string& code) {
    for (const auto& why : problems) {
      if (why.code == code) return true;
    }
    return false;
  };
  if (calibration_.active && !core_.View().Calibrating() && !has("STILL_STOPPING")) {
    problems.push_back(Reason("STILL_STOPPING", "The last calibration drive is still stopping, try again in a moment"));
  }
  if (!calibration_client_->action_server_is_ready() && !has("CALIBRATION_MISSING")) {
    problems.push_back(Reason("CALIBRATION_MISSING", "The calibration program is not running"));
  }
  return problems;
}

json LogicNode::ReasonJson(const Reason& why) {
  json out = {{"reason", why.code}, {"message", why.message}};
  for (const auto& [key, value] : why.fields.items()) out[key] = value;
  return out;
}

// {ok, problems: [{reason, message, ...}], space: {length, width, lane_length, heading} or null}, without a drive.
void LogicNode::OnCalibrationCheck(const mowbite_msgs::msg::RpcRequest& request) {
  std::vector<Reason> problems;
  std::optional<CalibrationStatus> status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double t = MonotonicSeconds();
    problems = CalibrationProblemsLocked(t);
    status = core_.View().CalibrationStatusNow(t);
  }
  json result = {{"ok", problems.empty()}, {"problems", json::array()}, {"space", nullptr}};
  for (const auto& why : problems) result["problems"].push_back(ReasonJson(why));
  if (status && status->lane_length > 0) {
    result["space"] = {{"length", status->strip_length},
                       {"width", status->strip_width},
                       {"lane_length", status->lane_length},
                       {"heading", status->heading}};
  }
  mowbite_msgs::msg::RpcResponse msg;
  msg.result = result.dump(-1, ' ', false, json::error_handler_t::replace);
  msg.id = request.id;
  rpc_response_pub_->publish(msg);
}

// {"state": "CALIBRATING"} once the drive began, else -32600 with the reason, its message and fields as data, and
// CALIBRATION_REFUSED. Parameters are not needed.
void LogicNode::OnCalibrationStart(const mowbite_msgs::msg::RpcRequest& request) {
  Reason why;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double t = MonotonicSeconds();
    const std::vector<Reason> problems = CalibrationProblemsLocked(t);
    if (!problems.empty()) {
      why = problems.front();
      core_.Call<&LogicCore::RefuseCalibration>(why);
    } else {
      why = core_.Call<&LogicCore::StartCalibration>(t);
    }
  }
  if (why.empty()) {
    RCLCPP_INFO(get_logger(), "calibration.start: calibrating");
    mowbite_msgs::msg::RpcResponse msg;
    msg.result = json{{"state", "CALIBRATING"}}.dump();
    msg.id = request.id;
    rpc_response_pub_->publish(msg);
  } else {
    RCLCPP_WARN(get_logger(), "calibration.start refused: %s (%s)", why.code.c_str(), why.message.c_str());
    PublishRpcError(request.id, mowbite_msgs::msg::RpcError::ERROR_INVALID_REQUEST, why.message,
                    ReasonJson(why).dump(-1, ' ', false, json::error_handler_t::replace));
  }
  Step();
}

// The app and HA (pause, dock): {"ok": true} when a drive was running; the mower stops where it is.
void LogicNode::OnCalibrationAbort(const mowbite_msgs::msg::RpcRequest& request) {
  bool ok;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ok = core_.Call<&LogicCore::AbortCalibration>(Reason("STOPPED", "Stopped by calibration.abort"));
  }
  if (ok) RCLCPP_INFO(get_logger(), "calibration.abort: stopped");
  mowbite_msgs::msg::RpcResponse msg;
  msg.result = json{{"ok", ok}}.dump();
  msg.id = request.id;
  rpc_response_pub_->publish(msg);
  Step();
}

// Hands the lane to the program. Called with the mutex held.
void LogicNode::SendCalibration(const CalibrationGoal& goal) {
  if (!calibration_client_->action_server_is_ready()) {
    core_.Call<&LogicCore::OnCalibrationEnded>(
        CalibrationOutcome{"CALIBRATION_MISSING", "The calibration program is not running", "", ""});
    return;
  }
  calibration_ = Calibration();
  calibration_.active = true;
  calibration_.seq = ++calibration_seq_;
  const uint64_t seq = calibration_.seq;
  Calibrate::Goal msg;
  msg.heading = goal.heading;
  msg.lane_length = goal.lane_length;
  rclcpp_action::Client<Calibrate>::SendGoalOptions options;
  options.goal_response_callback = [this, seq](const CalibrateHandle::SharedPtr& handle) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (seq != calibration_.seq) return;
      if (!handle) {
        calibration_ = Calibration();
        core_.Call<&LogicCore::OnCalibrationEnded>(
            CalibrationOutcome{"CALIBRATION_MISSING", "The calibration program did not take the drive", "", ""});
      } else {
        calibration_.handle = handle;
        core_.Call<&LogicCore::OnCalibrationSign>(MonotonicSeconds());
        // Stopped here while the goal was on its way.
        if (calibration_.cancel_pending) calibration_client_->async_cancel_goal(handle);
      }
    }
    Step();
  };
  options.feedback_callback = [this, seq](CalibrateHandle::SharedPtr,
                                          const std::shared_ptr<const Calibrate::Feedback> feedback) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (seq != calibration_.seq) return;
      core_.Call<&LogicCore::OnCalibrationFeedback>(feedback->phase, feedback->index, MonotonicSeconds());
    }
    Step();
  };
  options.result_callback = [this, seq](const CalibrateHandle::WrappedResult& wrapped) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (seq != calibration_.seq) return;
      calibration_ = Calibration();
      CalibrationOutcome outcome;
      if (wrapped.result) {
        outcome = {wrapped.result->reason, wrapped.result->message, wrapped.result->status, wrapped.result->summary};
      }
      // Dropped without an end of its own (gone, or another goal took its place): never done.
      if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED && outcome.reason == "DONE") outcome.reason.clear();
      if (outcome.reason.empty()) outcome.message = "The calibration program dropped the drive";
      core_.Call<&LogicCore::OnCalibrationEnded>(outcome);
    }
    Step();
  };
  calibration_client_->async_send_goal(msg, options);
}

// Stops the program after the logic ended its drive. Called with the mutex held.
void LogicNode::CancelCalibration() {
  if (!calibration_.active) return;
  calibration_.cancel_s = MonotonicSeconds();
  if (!calibration_.handle) {
    calibration_.cancel_pending = true;
    return;
  }
  try {
    calibration_client_->async_cancel_goal(calibration_.handle);
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError&) {
    // The client has dropped the goal already; there is nothing left to cancel.
  }
}

// <data_dir>/params/mower_logic.json, written like the checkpoint.
std::string LogicNode::LogicParamsPath() const {
  return data_dir_.empty() ? "" : data_dir_ + "/params/mower_logic.json";
}

// tron.yaml, then what params.set saved. A file that cannot be read stays as it is, tron.yaml holds.
void LogicNode::LoadLogicParams() {
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
void LogicNode::ApplyLogicParams() {
  core_.Call<&LogicCore::SetGpsSettings>(logic_params_.Get("max_position_accuracy"), logic_params_.Get("gps_wait_time"),
                                         logic_params_.Get("gps_timeout"));
  core_.Call<&LogicCore::SetUndockDistance>(logic_params_.Get("undock_distance"));
  core_.Call<&LogicCore::SetDockingSettings>(logic_params_.Get("docking_approach_distance"),
                                             static_cast<int>(logic_params_.Get("docking_retry_count")));
  core_.Call<&LogicCore::SetHomeSettings>(
      static_cast<int>(logic_params_.Get("rain_mode")), logic_params_.Get("rain_check_seconds"),
      logic_params_.Get("motor_hot_temperature"), logic_params_.Get("motor_cold_temperature"));
  core_.Call<&LogicCore::SetSunsetSettings>(logic_params_.Get("dock_at_sunset") != 0);
  TellRainWithoutSensor();
}

// rain_mode other than 0 does nothing on a platform without a rain sensor: told once, again after it was 0.
void LogicNode::TellRainWithoutSensor() {
  const int mode = static_cast<int>(logic_params_.Get("rain_mode"));
  if (mode == 0 || !rain_sensor_ || *rain_sensor_) {
    rain_told_ = false;
    return;
  }
  if (rain_told_) return;
  rain_told_ = true;
  RCLCPP_WARN(get_logger(), "rain_mode %d has no effect: this platform has no rain sensor (hw/capabilities)", mode);
}

bool LogicNode::SaveLogicParams(const LogicParams& params, std::string& error) {
  const std::string path = LogicParamsPath();
  return path.empty() || WriteFileSafely(path, params.Text(), &error);
}

void LogicNode::PublishLogicParams() {
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

// The dimensions of the platform (mowbite_bringup/config/geometry/<platform>.yaml); the entries of the user follow.
Geometry LogicNode::FactoryGeometry() {
  json factory = json::object();
  for (const auto& key : GeometryKeys()) {
    if (key.type == GeometryType::kNumber) factory[key.name] = GeometryNumber(*this, key.name);
  }
  // The outline of the body, if the platform has one: x0, y0, x1, y1, ...
  const std::vector<double> flat = declare_parameter("geometry.robot_outline", std::vector<double>());
  if (flat.size() % 2 != 0) throw std::invalid_argument("geometry.robot_outline needs pairs of x and y");
  if (!flat.empty()) {
    json points = json::array();
    for (size_t i = 0; i < flat.size(); i += 2) points.push_back(json::array({flat[i], flat[i + 1]}));
    factory["robot_outline"] = std::move(points);
  }
  const std::string platform = declare_parameter("geometry.platform", std::string());
  if (platform.empty()) throw std::invalid_argument("platform is missing in the geometry file of the platform");
  return Geometry(platform, factory);
}

std::string LogicNode::GeometryPath() const {
  return data_dir_.empty() ? "" : data_dir_ + "/geometry.json";
}

// properties.recorded_body of an area recorded now. Called with the mutex held.
std::string LogicNode::RecordedBodyText() const {
  json body = geometry_.RecordedBody();
  body["by"] = recorded_by_;
  return body.dump();
}

// The areas recorded here take the body robot.geometry.set made; a failure is RECORDED_BODY_KEPT. Called with the
// mutex held.
void LogicNode::SendRecordedBody() {
  const uint64_t seq = ++body_seq_;
  if (!recorded_body_client_->service_is_ready()) {
    body_sent_.reset();
    core_.Call<&LogicCore::RecordedBodyKept>(-1, "NO_ANSWER", "mowbite_map does not offer map/set_recorded_body");
    return;
  }
  auto request = std::make_shared<mowbite_msgs::srv::SetRecordedBody::Request>();
  request->by = recorded_by_;
  request->recorded_body = RecordedBodyText();
  body_sent_ = MonotonicSeconds();
  body_request_id_ =
      recorded_body_client_
          ->async_send_request(request,
                               [this, seq](rclcpp::Client<mowbite_msgs::srv::SetRecordedBody>::SharedFuture f) {
                                 const auto r = f.get();
                                 {
                                   std::lock_guard<std::mutex> lock(mutex_);
                                   // A late answer after the timeout, or one to an older change.
                                   if (seq != body_seq_ || !body_sent_) return;
                                   body_sent_.reset();
                                   if (r->success) {
                                     RCLCPP_INFO(get_logger(), "recorded_body of %d area(s) follows the new dimensions",
                                                 r->areas);
                                     return;
                                   }
                                   core_.Call<&LogicCore::RecordedBodyKept>(r->areas, r->code, r->message);
                                 }
                                 Step();
                               })
          .request_id;
}

// What robot.geometry.set saved. A file that cannot be read stays as it is, the dimensions of the platform hold.
void LogicNode::LoadGeometryEntries() {
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
    RCLCPP_ERROR(get_logger(), "%s is not valid (%s), the dimensions of the platform hold", path.c_str(), why.c_str());
  }
}

void LogicNode::PublishGeometry() {
  json params;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    params = geometry_.Params();
  }
  std_msgs::msg::String msg;
  msg.data = json{{"node", kGeometryNode}, {"params", params}}.dump();
  geometry_pub_->publish(msg);
}

void LogicNode::OnGeometryGet(const mowbite_msgs::msg::RpcRequest& request) {
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
void LogicNode::OnGeometrySet(const mowbite_msgs::msg::RpcRequest& request) {
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
    if (core_.View().HoldUntil() > 0) {
      PublishHoldRefusal(request.id, "The dimensions change after it");
      return;
    }
    if (!core_.View().Idle()) {
      PublishRpcError(request.id, RpcError::ERROR_INVALID_REQUEST,
                      "The dimensions change only in IDLE, not while recording, driving or mowing",
                      json{{"reason", "BUSY"}}.dump());
      return;
    }
    const json body_before = geometry_.RecordedBody();
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
      core_.Call<&LogicCore::SetTurningRadius>(geometry_.TurningRadius());
      if (geometry_.RecordedBody() != body_before) SendRecordedBody();
      // Only a value that is another now is news; a key entered with the platform's own value is not.
      if (!change.changed.empty()) {
        core_.Call<&LogicCore::GeometryChanged>(change.changed, change.below_default, geometry_.version());
      }
    }
    answer = geometry_.Get();
  }
  if (change.stored) {
    RCLCPP_INFO(get_logger(), "robot.geometry.set: version %d, changed %zu key(s)", answer["version"].get<int>(),
                change.changed.size());
    PublishGeometry();
    Step();
  }
  mowbite_msgs::msg::RpcResponse msg;
  msg.result = answer.dump();
  msg.id = request.id;
  rpc_response_pub_->publish(msg);
}

bool LogicNode::SaveGeometry(const Geometry& geometry, std::string* error) {
  const std::string path = GeometryPath();
  return path.empty() || WriteFileSafely(path, geometry.Text(), error);
}

void LogicNode::OnPlanAnswer(uint64_t seq, const mowbite_planner_interfaces::srv::PlanCoverage::Response& response) {
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
void LogicNode::ExpirePlans() {
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
      core_.Call<&LogicCore::OnJobPlanFailed>(
          Reason("PLANNER_TIMEOUT",
                 "No answer from the planner in " + std::to_string(static_cast<int>(plan_timeout_s_)) + " s"));
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
    PublishRpcError(id, kPlanInternal,
                    "No answer from the planner in " + std::to_string(static_cast<int>(plan_timeout_s_)) + " s");
  }
}

void LogicNode::PublishRpcError(const std::string& id, int code, const std::string& message, const std::string& data) {
  mowbite_msgs::msg::RpcError msg;
  msg.code = static_cast<int16_t>(code);
  msg.message = message;
  msg.id = id;
  msg.data = data;
  rpc_error_pub_->publish(msg);
}

std::shared_ptr<mowbite_planner_interfaces::srv::PlanCoverage::Request> LogicNode::ToServiceRequest(
    const CoverageRequest& r) {
  auto request = std::make_shared<mowbite_planner_interfaces::srv::PlanCoverage::Request>();
  request->boundary = ToPolygon(r.boundary);
  for (const auto& ring : r.obstacles) request->obstacles.push_back(ToPolygon(ring));
  for (const auto& ring : r.drivable_areas) request->drivable_areas.push_back(ToPolygon(ring));
  request->perimeter_passes = r.perimeter_passes;
  request->use_angle = r.use_angle;
  request->angle = r.angle;
  if (r.lane_overlap_passes) request->overrides.push_back(Override("lane_overlap_passes", *r.lane_overlap_passes));
  if (r.perimeter_offset) request->overrides.push_back(Override("perimeter_offset", *r.perimeter_offset));
  if (r.angle_offset_deg) request->overrides.push_back(Override("angle_offset_deg", *r.angle_offset_deg));
  return request;
}

CoverageAnswer LogicNode::ToAnswer(const mowbite_planner_interfaces::srv::PlanCoverage::Response& r) {
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

void LogicNode::PublishReadings() {
  std::vector<SensorReading> readings;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    readings = core_.View().Readings();
  }
  for (const auto& r : readings) {
    mowbite_msgs::msg::SensorValue msg;
    msg.sensor_id = r.id;
    msg.value = r.value;
    msg.text = r.text;
    sensor_value_pub_->publish(msg);
  }
}

void LogicNode::PublishOverlay() {
  // The first, empty one replaces the lines of a recording before a restart, so it waits for the bridge.
  if (publish_pub_->get_subscription_count() == 0) return;
  std::unique_lock<std::mutex> lock(mutex_);
  if (!core_.Call<&LogicCore::TakeOverlayChanged>()) return;
  const json overlay = core_.View().MapOverlay();
  lock.unlock();
  mowbite_msgs::msg::MqttPublish msg;
  msg.payload = overlay.dump();
  msg.topic = "map_overlay/json";
  msg.retain = true;
  publish_pub_->publish(msg);
}

// Sends a change of the map the recording asks for; its answer goes back to the core. Called with the mutex held.
void LogicNode::SendMapEdit(const MapEdit& edit) {
  const uint64_t seq = ++edit_seq_;
  edit_is_dock_ = edit.kind == MapEdit::Kind::kDock;
  const auto answer = [this, seq](bool ok, const std::string& id, const std::string& code, const std::string& message) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // A late answer after the timeout is not the current one any more.
      if (seq != edit_seq_ || !edit_sent_) return;
      edit_sent_.reset();
      edit_request_id_.reset();
      core_.Call<&LogicCore::OnMapEditResult>(ok, id, code, message);
    }
    Step();
  };
  if (edit_is_dock_) {
    if (!set_dock_client_->service_is_ready()) {
      core_.Call<&LogicCore::OnMapEditResult>(false, "", "NO_ANSWER", "mowbite_map does not offer map/set_dock");
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
    core_.Call<&LogicCore::OnMapEditResult>(false, "", "NO_ANSWER", "mowbite_map does not offer map/add_area");
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
  // Only rings driven forward without a break carry the body they were recorded with.
  const std::string body = RecordedBodyText();
  if (edit.outline_driven_reason.empty()) request->recorded_body = body;
  for (const auto& reason : edit.obstacles_driven_reason) {
    request->obstacles_recorded_body.push_back(reason.empty() ? body : "");
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
void LogicNode::SendDockCheck(double heading) {
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
                                 Step();
                               })
          .request_id;
  RCLCPP_INFO(get_logger(), "Reference check on the docking station: heading %.3f rad", heading);
}

// Called with the mutex held.
void LogicNode::FinishDockCheck(bool ok, double x, double y, float accuracy, const std::string& message) {
  dock_check_sent_.reset();
  const Reason why = core_.Call<&LogicCore::OnDockCheck>(ok, x, y, accuracy, message);
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

// hw/charge: off before the mower leaves the contacts (undocking, a docking that failed there), the answer goes to
// the core; on again after it. Called with the mutex held.
void LogicNode::SendCharge(bool on) {
  const uint64_t seq = ++charge_seq_;
  charge_sent_.reset();
  if (!charge_client_->service_is_ready()) {
    if (on) {
      RCLCPP_WARN(get_logger(), "hw/charge is not available, charging stays as the driver has it");
    } else {
      core_.Call<&LogicCore::OnChargeOff>(false, "the driver does not offer hw/charge");
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
                                   core_.Call<&LogicCore::OnChargeOff>(r->success, r->message);
                                 }
                                 Step();
                               })
          .request_id;
  RCLCPP_INFO(get_logger(), "Charging %s", on ? "allowed again" : "off before leaving the docking station");
}

// Hands the way out of the station to the follower: straight, without steering, at the speed of undocking. Besides
// the drivable areas base_link may be in the corridor along it. Called with the mutex held.
void LogicNode::SendUndock(const UndockPath& path) {
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
void LogicNode::OnUndockOutcome(const UndockOutcome& outcome) {
  if (!outcome.for_drive || !dock_drive_) return;
  const DockDrive pending = *dock_drive_;
  dock_drive_.reset();
  if (outcome.undocked && outcome.why.empty() && core_.View().Driving()) {
    PlanDrive(pending.request, pending.header);
  } else {
    RespondDrive(pending.header, false, outcome.why.message);
  }
}

geometry_msgs::msg::Point LogicNode::ToPoint(const Point2& p) {
  geometry_msgs::msg::Point point;
  point.x = p.x;
  point.y = p.y;
  return point;
}

// Carries out what the core asks for after a callback, in a fixed order: what halts (drive canceled, zero command,
// calibration canceled, blade off), then os-session.json (waited for only when it begins the restart hold, G35),
// then the rest in the order it came up, among it what starts a motion, then the next step of the job and of
// docking. What that brings up goes the same way, at most kMaxRounds rounds; publications come last. The files go
// through the writer, so no fsync delays a halt.
void LogicNode::Step() {
  std::vector<Effect> record;
  for (int round = 0;; ++round) {
    std::vector<Effect> effects;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (round == 0) TellUnwritableFileLocked();
      effects = core_.Call<&LogicCore::TakeEffects>();
      if (round > 0 && effects.empty()) break;
      for (const auto& effect : effects) {
        if (PhaseOf(effect) == EffectPhase::kHalt) Halt(effect);
      }
      if (!core_.View().BladeWanted()) UpdateBladeLocked();
      if (round == kMaxRounds) {
        // Never in a sound core: stopped, what would act dropped.
        SendStop();
        RCLCPP_ERROR(get_logger(), "The logic still asked for %zu effects after %d rounds; stopped, the rest dropped",
                     effects.size(), kMaxRounds);
      }
    }
    for (auto& effect : effects) {
      if (PhaseOf(effect) == EffectPhase::kRecord) record.push_back(std::move(effect));
    }
    if (round == kMaxRounds) break;
    const bool written = WriteSession(false);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!written) {
      // Nothing of the start acts: the core takes it back, the next round carries out how it ends.
      unwritable_told_ = mowbite_hold::kSessionFile;
      core_.Call<&LogicCore::AbandonStart>(unwritable_told_);
      continue;
    }
    for (const auto& effect : effects) {
      if (PhaseOf(effect) == EffectPhase::kAct) Act(effect);
    }
    if (round == 0) {
      PumpJob();
      PumpDocking();
    }
    UpdateBladeLocked();
  }
  Record(record);
}

// Called with the mutex held.
void LogicNode::Halt(const Effect& effect) {
  if (const auto* cancel = std::get_if<DriveCancel>(&effect)) {
    CancelDrive(cancel->why);
  } else if (std::holds_alternative<StopRequest>(effect)) {
    SendStop();
  } else if (std::holds_alternative<CalibrationCancel>(effect)) {
    CancelCalibration();
  }
}

// Undocking (and docking that leaves the contacts): charging off, then the way out; after it the drive that waited,
// or charging on again. Called with the mutex held.
void LogicNode::Act(const Effect& effect) {
  if (std::holds_alternative<ChargeOff>(effect)) {
    SendCharge(false);
  } else if (std::holds_alternative<ChargeOn>(effect)) {
    SendCharge(true);
  } else if (const auto* check = std::get_if<ReferenceCheck>(&effect)) {
    SendDockCheck(check->heading);
  } else if (const auto* path = std::get_if<UndockPath>(&effect)) {
    SendUndock(*path);
  } else if (const auto* outcome = std::get_if<UndockOutcome>(&effect)) {
    OnUndockOutcome(*outcome);
  } else if (const auto* goal = std::get_if<CalibrationGoal>(&effect)) {
    SendCalibration(*goal);
  } else if (const auto* edit = std::get_if<MapEdit>(&effect)) {
    SendMapEdit(*edit);
  }
}

// The checkpoint (the latest of the step), log lines, events and actions. Called without the mutex.
void LogicNode::Record(const std::vector<Effect>& effects) {
  const Checkpoint* checkpoint = nullptr;
  bool actions_changed = false;
  for (const auto& effect : effects) {
    if (const auto* c = std::get_if<Checkpoint>(&effect)) checkpoint = c;
    actions_changed = actions_changed || std::holds_alternative<ActionsChanged>(effect);
  }
  if (checkpoint) SaveCheckpoint(checkpoint->text);
  for (const auto& effect : effects) {
    if (const auto* notice = std::get_if<Notice>(&effect)) {
      RCLCPP_INFO(get_logger(), "%s", notice->line.c_str());
    } else if (const auto* change = std::get_if<ReasonChange>(&effect)) {
      // The app shows the reasons of an active emergency from this line, like with mower_comms_v2 of OpenMower.
      if (change->to == 0) {
        RCLCPP_INFO(get_logger(), "Emergency cleared");
      } else {
        RCLCPP_WARN(get_logger(), "Emergency reason changed from: '%s' to: '%s'",
                    EmergencyReasonNames(change->from).c_str(), EmergencyReasonNames(change->to).c_str());
      }
    }
  }
  for (const auto& effect : effects) {
    const auto* event = std::get_if<Event>(&effect);
    if (!event) continue;
    if (event->type == "RECORDING_REFUSED") {
      RCLCPP_WARN(get_logger(), "Recording: %s refused: %s", event->details.value("action", "").c_str(),
                  event->details.value("reason", "").c_str());
    }
    if (event->type == "UNDOCKING_FAILED") {
      RCLCPP_WARN(get_logger(), "UNDOCKING_FAILED %s: %s", event->details.value("reason", "").c_str(),
                  event->details.value("message", "").c_str());
    }
    mowbite_msgs::msg::MqttPublish msg;
    msg.topic = "events/json";
    msg.payload = mowbite_bridge::BuildEvent(event->type, event->details, now().seconds(), event->id).dump();
    publish_pub_->publish(msg);
  }
  PublishRuns(effects);
  if (actions_changed) PublishActions();
  PublishIdleIfChanged();
  PublishHoldIfChanged();
}

void LogicNode::PublishIdleIfChanged() {
  std_msgs::msg::Bool msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.data = core_.View().Idle();
    if (published_idle_ == msg.data) return;
    published_idle_ = msg.data;
  }
  idle_pub_->publish(msg);
}

void LogicNode::PublishActions() {
  std::vector<ActionGroup> groups;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    groups = core_.View().Actions();
  }
  for (const auto& group : groups) {
    mowbite_msgs::msg::ActionList msg;
    msg.node_prefix = group.prefix;
    for (const auto& action : group.actions) {
      mowbite_msgs::msg::ActionEntry entry;
      entry.id = action.id;
      entry.name = action.name;
      entry.enabled = action.enabled;
      entry.reason = action.reason;
      msg.actions.push_back(entry);
    }
    actions_pub_->publish(msg);
  }
}

}  // namespace mowbite_logic

RCLCPP_COMPONENTS_REGISTER_NODE(mowbite_logic::LogicNode)
