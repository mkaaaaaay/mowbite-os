// Docking at the node: the legs for the follower, and docking by the dock marker (hw/dock_marker,
// localization/dock_pose, follower/dock_cmd_vel).
#include <cmath>
#include <cstdio>
#include <vector>

#include <mowbite_msgs/msg/dock_marker.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>

#include "logic_node.h"

namespace mowbite_logic {

using DockMarkerMsg = mowbite_msgs::msg::DockMarker;

void LogicNode::SetUpDocking() {
  if (!core_.View().MarkerSource()) return;
  const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  marker_request_pub_ = create_publisher<std_msgs::msg::Empty>("hw/dock_marker_request", volatile_qos);
  heading_hint_pub_ = create_publisher<std_msgs::msg::Float64>("logic/dock_heading_hint", volatile_qos);
  // Measurements and the pose in the station frame reach the core only while it wants them, so the journal stays small.
  marker_sub_ = create_subscription<DockMarkerMsg>(
      "hw/dock_marker", volatile_qos, [this](const DockMarkerMsg::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!core_.View().MarkerWanted()) return;
        DockMarkerSample sample;
        sample.time = MonotonicSeconds();
        sample.state = msg->state;
        sample.valid = msg->valid;
        sample.yaw_valid = msg->yaw_valid;
        sample.reason = msg->reason;
        core_.Call<&LogicCore::OnDockMarker>(sample);
      });
  dock_pose_sub_ = create_subscription<mowbite_msgs::msg::RobotPose>(
      "localization/dock_pose", 10, [this](const mowbite_msgs::msg::RobotPose::ConstSharedPtr& msg) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!core_.View().MarkerWanted()) return;
          DockPoseSample pose;
          pose.time = MonotonicSeconds();
          pose.x = msg->x;
          pose.y = msg->y;
          pose.heading = msg->heading;
          pose.heading_valid = msg->heading_valid;
          pose.position_accuracy = msg->position_accuracy;
          pose.heading_accuracy = msg->heading_accuracy;
          pose.speed = msg->speed;
          core_.Call<&LogicCore::OnDockPose>(pose);
        }
        Step();
      });
  // The follower in the station frame reaches safety/drive only while the core drives a leg there.
  dock_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "follower/dock_cmd_vel", 10, [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
        bool active;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          active = core_.View().DockFollowerActive();
        }
        if (active) SendDrive(mowbite_msgs::msg::DriveCommand::SOURCE_DOCK, *msg);
      });
  marker_request_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { RequestMarker(); });
}

// Asks for measurements every REQUEST_PERIOD_S while the core wants them, the first at once, each with the heading
// hint for the estimate where the core has one.
void LogicNode::RequestMarker() {
  bool wanted;
  std::optional<double> hint;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    wanted = core_.View().MarkerWanted();
    hint = core_.View().DockHeadingHint();
  }
  const double now = MonotonicSeconds();
  if (!wanted) {
    marker_requested_.reset();
    return;
  }
  if (marker_requested_ && now - *marker_requested_ < DockMarkerMsg::REQUEST_PERIOD_S) return;
  marker_requested_ = now;
  marker_request_pub_->publish(std_msgs::msg::Empty());
  if (!hint) return;
  std_msgs::msg::Float64 msg;
  msg.data = *hint;
  heading_hint_pub_->publish(msg);
}

// Whether rpc.methods lists a command: docking.start only with charging contacts, docking.test also with a marker.
bool LogicNode::Offered(const std::string& command) const {
  if (command == "docking.start") return core_.View().CanDock();
  if (command == "docking.test") return core_.View().CanDock() && core_.View().MarkerSource();
  return true;
}

// docking.test needs the follower, not the planner. Called with the mutex held.
Reason LogicNode::DockTestProblemLocked() const {
  if (!follow_client_->action_server_is_ready()) return Reason("FOLLOWER_MISSING", "The follower is not running");
  if (drive_.active || job_plan_.active || job_cut_.active) {
    return Reason("STILL_STOPPING", "The last drive is still stopping, try again in a moment");
  }
  return {};
}

// The next leg of docking, once nothing else is on its way. Called with the mutex held.
void LogicNode::PumpDocking() {
  if (drive_.active || job_plan_.active || job_cut_.active) return;
  if (auto leg = core_.Call<&LogicCore::TakeDockingLeg>()) SendDockingLeg(*leg);
}

// The way to the docking station over the drivable areas, or a line for the follower: besides the drivable areas
// base_link may be in the corridor along it, if it has one. In the station frame the corridor is all there is.
// Called with the mutex held.
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
  const bool station = leg.frame == LegFrame::kDock;
  if (!station) planner_.TransitAreas(&drivable, &obstacles);
  FollowPath::Goal goal;
  goal.frame = station ? FollowPath::Goal::FRAME_DOCK : FollowPath::Goal::FRAME_MAP;
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
  std::snprintf(text, sizeof(text), "%s, %.2f m %s to (%.2f, %.2f)%s%s", DockingLegName(leg.kind),
                std::hypot(to.x - from.x, to.y - from.y), leg.reverse ? "backwards" : "forwards", to.x, to.y, speed,
                station ? " in the station frame" : "");
  SendFollow(goal, text);
}

}  // namespace mowbite_logic
