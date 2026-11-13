// The node of the logic, declared here so that its parts can live in more than one file.
#ifndef MOWBITE_LOGIC_LOGIC_NODE_H
#define MOWBITE_LOGIC_LOGIC_NODE_H

#include <algorithm>
#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mowbite_journal/node_options.h>
#include <mowbite_msgs/action/calibrate.hpp>
#include <mowbite_msgs/action/follow_path.hpp>
#include <mowbite_msgs/msg/action_list.hpp>
#include <mowbite_msgs/msg/button_event.hpp>
#include <mowbite_msgs/msg/calibration_status.hpp>
#include <mowbite_msgs/msg/clock_status.hpp>
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
#include <mowbite_msgs/msg/run_status.hpp>
#include <mowbite_msgs/msg/rpc_error.hpp>
#include <mowbite_msgs/msg/rpc_methods.hpp>
#include <mowbite_msgs/msg/rpc_request.hpp>
#include <mowbite_msgs/msg/rpc_response.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mowbite_msgs/srv/add_area.hpp>
#include <mowbite_msgs/srv/drive_to.hpp>
#include <mowbite_msgs/srv/drop_tasks.hpp>
#include <mowbite_msgs/srv/set_blade.hpp>
#include <mowbite_msgs/srv/set_charge.hpp>
#include <mowbite_msgs/srv/set_dock.hpp>
#include <mowbite_msgs/srv/set_emergency.hpp>
#include <mowbite_msgs/srv/set_heading.hpp>
#include <mowbite_msgs/srv/set_recorded_body.hpp>
#include <mowbite_msgs/srv/start_run.hpp>
#include <mowbite_hold/file_io.h>
#include <mowbite_hold/file_writer.h>
#include <mowbite_hold/hold.h>
#include <mowbite_hold/restart.h>
#include <map>
#include <memory>
#include <mowbite_planner_interfaces/srv/plan_coverage.hpp>
#include <mowbite_planner_os_interfaces/srv/plan_transit.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "mowbite_logic/geometry.h"
#include "mowbite_logic/logic_config.h"
#include "mowbite_logic/logic_core.h"
#include "mowbite_logic/logic_journal.h"
#include "mowbite_logic/logic_messages.h"
#include "mowbite_logic/logic_params.h"
#include "mowbite_logic/mowing_plan.h"
#include "mowbite_watch/attach.h"

namespace mowbite_logic {

class LogicNode : public rclcpp::Node {
  using PlanTransit = mowbite_planner_os_interfaces::srv::PlanTransit;
  using FollowPath = mowbite_msgs::action::FollowPath;
  using FollowHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using Calibrate = mowbite_msgs::action::Calibrate;
  using CalibrateHandle = rclcpp_action::ClientGoalHandle<Calibrate>;
  // What the follower drives for: logic/drive_to, the approach to a path of the job, that path, out of the station, or
  // a leg of docking.
  enum Purpose { kDriveTo = 0, kApproach = 1, kMow = 2, kUndock = 3, kDock = 4 };

 public:
  explicit LogicNode(const rclcpp::NodeOptions& options);
  ~LogicNode() override;

 private:
  // The monotonic clock in s: every limit of the node is measured on it.
  static double MonotonicSeconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
  }

  static int64_t WallSeconds() {
    using namespace std::chrono;
    return duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
  }

  static rclcpp::QoS LatchedQos(size_t depth) {
    return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
  }

  void OnHardwareStatus(const mowbite_msgs::msg::HardwareStatus& msg);
  void OnPose(const mowbite_msgs::msg::RobotPose& msg);
  Reason NodeProblemLocked(const std::string& command) const;
  void OnAction(const std::string& action_id);
  void OnCommandRpc(const mowbite_msgs::msg::RpcRequest& request, const mowbite_api::Command& command);
  void AnswerReleaseLocked(bool released, const Reason& why = Reason(), const std::string& event_id = "");
  void RefuseReleaseLocked(const Reason& why);
  void OnButton(const mowbite_msgs::msg::ButtonEvent& msg);
  void ReleaseEmergency(const std::string& rpc_id = "", uint32_t latch_count = 0,
                        const std::string& reason = "reset_emergency from the app");
  void SendDrive(uint8_t source, const geometry_msgs::msg::Twist& twist);
  void SendStop();
  void OnTeleop(const geometry_msgs::msg::Twist& msg);
  void OnDriveTo(const std::shared_ptr<rmw_request_id_t>& header, const mowbite_msgs::srv::DriveTo::Request& request);
  void PlanDrive(const mowbite_msgs::srv::DriveTo::Request& request, const std::shared_ptr<rmw_request_id_t>& header);
  void UpdateDrivableAreas();
  void OnTransit(uint64_t seq, const PlanTransit::Response& response);
  void SendFollow(const FollowPath::Goal& goal, const std::string& what);
  bool CheckFollowerLocked();
  void CancelDrive(const Reason& why);
  void EndDrive(bool arrived, const Reason& why, uint8_t reason, uint32_t index);
  static const char* PurposeName(int purpose);
  void PumpJob();
  void PlanJobArea(const std::string& area_id);
  void OnJobPlan(uint64_t seq, const mowbite_planner_interfaces::srv::PlanCoverage::Response& response);
  void Approach(const JobPose& goal);
  Reason PlanWay(Purpose purpose, double x, double y, double heading);
  void PumpDocking();
  void SendDockingLeg(const DockingLeg& leg);
  void FollowJobPath(const JobPath& path, size_t start_index);
  void UpdateBlade();
  void UpdateBladeLocked();
  void SendBlade(bool on);
  void JudgeRestart(const std::string& crash_record);
  bool WriteSession(bool clean);
  void StartWriter();
  void OnWritten(const mowbite_hold::FileWriter::Done& done);
  void CheckWriter();
  void TellUnwritableFileLocked();
  bool BridgeListens();
  void ReportRestartWhenHeard();
  std::string CheckpointPath() const;
  void RestoreCheckpoint();
  void SaveCheckpoint(const std::string& text);
  void RespondDrive(const std::shared_ptr<rmw_request_id_t>& header, bool success, const std::string& message);
  void OnStateTimer();
  void OnRpcRequest(const mowbite_msgs::msg::RpcRequest& request);
  void OnParamsSet(const mowbite_msgs::msg::RpcRequest& request);
  void OnSystemHold(const mowbite_msgs::msg::RpcRequest& request);
  json HoldAnswerLocked() const;
  int64_t HoldUntilLocked() const;
  void PublishHoldRefusal(const std::string& id, const std::string& what);
  void RestoreHold();
  void ExpireHoldLocked(double t);
  void PublishHoldIfChanged();
  void MirrorHold();
  static bool Ours(const mowbite_api::Command& command);
  void PublishRpcMethods();
  void OnCalibrationStatus(const mowbite_msgs::msg::CalibrationStatus& msg);
  std::vector<Reason> CalibrationProblemsLocked(double t) const;
  static json ReasonJson(const Reason& why);
  void OnCalibrationCheck(const mowbite_msgs::msg::RpcRequest& request);
  void OnCalibrationStart(const mowbite_msgs::msg::RpcRequest& request);
  void OnCalibrationAbort(const mowbite_msgs::msg::RpcRequest& request);
  void SendCalibration(const CalibrationGoal& goal);
  void CancelCalibration();
  std::string LogicParamsPath() const;
  void LoadLogicParams();
  void ApplyLogicParams();
  void TellRainWithoutSensor();
  bool SaveLogicParams(const LogicParams& params, std::string& error);
  void PublishLogicParams();
  Geometry FactoryGeometry();
  std::string GeometryPath() const;
  std::string RecordedBodyText() const;
  void SendRecordedBody();
  void LoadGeometryEntries();
  void PublishGeometry();
  void OnGeometryGet(const mowbite_msgs::msg::RpcRequest& request);
  void OnGeometrySet(const mowbite_msgs::msg::RpcRequest& request);
  bool SaveGeometry(const Geometry& geometry, std::string* error);
  void OnPlanAnswer(uint64_t seq, const mowbite_planner_interfaces::srv::PlanCoverage::Response& response);
  void ExpirePlans();
  void PublishRpcError(const std::string& id, int code, const std::string& message, const std::string& data = "");
  static std::shared_ptr<mowbite_planner_interfaces::srv::PlanCoverage::Request> ToServiceRequest(
      const CoverageRequest& r);
  static CoverageAnswer ToAnswer(const mowbite_planner_interfaces::srv::PlanCoverage::Response& r);
  void PublishReadings();
  void PublishOverlay();
  void SendMapEdit(const MapEdit& edit);
  void SendDockCheck(double heading);
  void FinishDockCheck(bool ok, double x, double y, float accuracy, const std::string& message);
  void SendCharge(bool on);
  void SendUndock(const UndockPath& path);
  void OnUndockOutcome(const UndockOutcome& outcome);
  static geometry_msgs::msg::Point ToPoint(const Point2& p);
  void Step();
  void Halt(const Effect& effect);
  void Act(const Effect& effect);
  void Record(const std::vector<Effect>& effects);
  void PublishIdleIfChanged();
  void PublishActions();

  // The runs of the schedule (logic_node_run.cpp).
  void SetUpRuns();
  void OnStartRun(const mowbite_msgs::srv::StartRun::Request& request, mowbite_msgs::srv::StartRun::Response& response);
  void OnDropTasks(const mowbite_msgs::srv::DropTasks::Request& request,
                   mowbite_msgs::srv::DropTasks::Response& response);
  void PublishRuns(const std::vector<Effect>& effects);
  bool ScheduleBlocksHoldLocked(int64_t wall) const;

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
    bool no_way = false;  // the planner refused the way with NO_WAY or START_OUTSIDE
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

  // The goal of calibration/run on its way or running; its number tells late answers apart.
  struct Calibration {
    bool active = false;
    uint64_t seq = 0;
    CalibrateHandle::SharedPtr handle;
    bool cancel_pending = false;
    // MonotonicSeconds() when the logic stopped it; the program has kCalibrationStopWaitS to confirm.
    double cancel_s = 0;
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
  // Null without journal_dir.
  mowbite_journal::Journal* journal_;
  mowbite_journal::Journaled<LogicCore> core_;
  mowbite_journal::JournalWatch journal_watch_;
  rclcpp::TimerBase::SharedPtr journal_timer_;
  // system.hold: <data_dir>/hold.json (empty without data_dir), its time on the monotonic clock, what logic/hold said
  // last and when mowbite/hold went out.
  std::string hold_path_;
  // The restart hold: os-session.json, its record last written or queued, whether its last write failed, whether a
  // record that holds is surely on the disk; restart-hold.json (all empty without data_dir) and its text until written;
  // what the core was told cannot be written; the boot of this start and the event that waits for the bridge.
  std::string session_path_;
  std::optional<mowbite_hold::SessionRecord> session_;
  std::atomic<bool> session_failed_{false};
  bool session_holds_ = false;
  std::string restart_hold_path_;
  std::string hold_text_;
  std::atomic<bool> hold_failed_{false};
  std::string unwritable_told_;
  bool bridge_listens_ = false;
  std::string boot_id_;
  std::optional<RestartReport> restart_report_;
  // os-session.json and checkpoint.json after the start; null without data_dir and during the stop.
  std::unique_ptr<mowbite_hold::FileWriter> writer_;
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
  // rain_sensor of hw/capabilities, unknown until they came; whether the useless rain_mode was told.
  std::optional<bool> rain_sensor_;
  bool rain_told_ = false;
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
  // by of recorded_body, and the change of it on its way to the map like the edit above.
  std::string recorded_by_;
  std::optional<double> body_sent_;
  std::optional<int64_t> body_request_id_;
  uint64_t body_seq_ = 0;
  // The release asked of S2, under mutex_.
  std::optional<double> release_sent_;
  // ids of the emergency.reset calls that wait for the safety node.
  std::vector<std::string> release_rpc_ids_;
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
  rclcpp::Client<mowbite_msgs::srv::SetRecordedBody>::SharedPtr recorded_body_client_;
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
  rclcpp::Subscription<mowbite_msgs::msg::ClockStatus>::SharedPtr clock_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr datum_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::MapStatus>::SharedPtr map_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr follower_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr caps_sub_;
  rclcpp::Subscription<mowbite_msgs::msg::ButtonEvent>::SharedPtr button_sub_;
  rclcpp::Client<PlanTransit>::SharedPtr transit_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<Calibrate>::SharedPtr calibration_client_;
  rclcpp::Subscription<mowbite_msgs::msg::CalibrationStatus>::SharedPtr calibration_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr calibration_cmd_sub_;
  // The goal of calibration/run, whether calibration.* is in rpc.methods, and the numbers of the goals.
  Calibration calibration_;
  bool calibration_offered_ = false;
  uint64_t calibration_seq_ = 0;
  rclcpp::Service<mowbite_msgs::srv::DriveTo>::SharedPtr drive_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_drive_srv_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  rclcpp::TimerBase::SharedPtr overlay_timer_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::TimerBase::SharedPtr blade_timer_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
  // The next start of the schedule (schedule/next_start, unix s, 0 none), under mutex_.
  int64_t next_start_ = 0;
  rclcpp::Publisher<mowbite_msgs::msg::RunStatus>::SharedPtr run_pub_;
  rclcpp::Service<mowbite_msgs::srv::StartRun>::SharedPtr start_run_srv_;
  rclcpp::Service<mowbite_msgs::srv::DropTasks>::SharedPtr drop_tasks_srv_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr next_start_sub_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_NODE_H
