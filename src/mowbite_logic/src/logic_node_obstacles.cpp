// The hold for obstacles at the node: obstacles/status into the core, logic/drive_gate for the follower and
// obstacles/action for the list of mowbite_obstacles; driving around objects.
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "logic_node.h"

namespace mowbite_logic {

namespace {

// The gate goes out this often; the follower stands once it is older than its gate_timeout.
constexpr auto kGatePeriod = std::chrono::milliseconds(100);

}  // namespace

void LogicNode::SetUpObstacles() {
  gate_pub_ = create_publisher<mowbite_msgs::msg::DriveGate>("logic/drive_gate", 10);
  obstacle_action_pub_ = create_publisher<mowbite_msgs::msg::ObstacleAction>("obstacles/action", 10);
  obstacle_sub_ = create_subscription<mowbite_msgs::msg::ObstacleStatus>(
      "obstacles/status", 10, [this](const mowbite_msgs::msg::ObstacleStatus& msg) { OnObstacleStatus(msg); });
  // From the timer of this node: while the logic hangs, no gate goes out and the follower stands.
  gate_timer_ = create_wall_timer(kGatePeriod, [this] {
    std::lock_guard<std::mutex> lock(mutex_);
    PublishGateLocked();
  });
}

void LogicNode::OnObstacleStatus(const mowbite_msgs::msg::ObstacleStatus& msg) {
  ObstacleSample sample;
  sample.sight = msg.sight;
  for (const auto& track : msg.tracks) {
    sample.tracks.push_back({track.id, track.kind, track.confirmed, track.in_view, static_cast<double>(track.ahead),
                             track.zone, track.x, track.y, static_cast<double>(track.radius),
                             static_cast<double>(track.sigma)});
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    core_.Call<&LogicCore::OnObstacleStatus>(sample, MonotonicSeconds());
    PublishGateLocked();
  }
  Step();
}

// Called with the mutex held.
void LogicNode::PublishGateLocked() {
  mowbite_msgs::msg::DriveGate msg;
  msg.header.stamp = now();
  msg.go = core_.View().GateGo();
  msg.slow = core_.View().GateSlow();
  gate_pub_->publish(msg);
}

// Called without the mutex.
void LogicNode::PublishObstacleNotes(const std::vector<Effect>& effects) {
  for (const auto& effect : effects) {
    const auto* note = std::get_if<ObstacleNote>(&effect);
    if (!note) continue;
    mowbite_msgs::msg::ObstacleAction msg;
    msg.track_id = note->track_id;
    msg.action = note->action;
    msg.keep_out_r = static_cast<float>(note->keep_out_r);
    msg.left_out = static_cast<float>(note->left_out);
    msg.area_id = note->area_id;
    msg.area_name = note->area_name;
    obstacle_action_pub_->publish(msg);
  }
}

// Called with the mutex held, or before the node runs.
void LogicNode::ApplyObstacleModeLocked() {
  core_.Call<&LogicCore::SetObstacleMode>(static_cast<int>(logic_params_.Get("obstacle_mode")), obstacle_mode_max_);
}

// Called with the mutex held.
LogicNode::FollowPath::Goal LogicNode::Gated(const FollowPath::Goal& goal) const {
  FollowPath::Goal gated = goal;
  gated.gate = core_.View().GateWanted();
  return gated;
}

std::vector<geometry_msgs::msg::Polygon> LogicNode::CirclePolygons() const {
  std::vector<geometry_msgs::msg::Polygon> rings;
  for (const auto& circle : core_.View().Circles()) rings.push_back(ToPolygon(KeepOutRing(circle)));
  return rings;
}

void LogicNode::Detour(const JobStep& step) {
  Reason why = PlanWay(Purpose::kApproach, step.goal.x, step.goal.y, step.goal.heading, step.area_id,
                       core_.View().AvoidSpeed());
  if (why.code == "PLANNER_MISSING") why = Reason("APPROACH_FAILED", why.message);
  if (!why.empty()) {
    core_.Call<&LogicCore::OnLegEnded>(LegEnd::kFailed, 0, why);
    return;
  }
  RCLCPP_INFO(get_logger(), "Mowing: around the obstacle to (%.2f, %.2f), the blade on", step.goal.x, step.goal.y);
}

void LogicNode::CutJobPlan(const JobStep& step) {
  std::vector<Ring> drivable;
  std::vector<Ring> obstacles;
  Reason why = planner_.TransitAreas(&drivable, &obstacles, step.area_id);
  if (why.empty() && !cut_client_->service_is_ready()) why = PlannerMissing();
  if (!why.empty()) {
    CutAnswer failed;
    failed.code = why.code;
    failed.message = why.message;
    core_.Call<&LogicCore::OnCutAnswer>(failed);
    return;
  }
  const RobotStateData state = core_.View().RobotState();
  auto request = std::make_shared<CutPlan::Request>();
  request->start = PoseAt(state.pose_x, state.pose_y, state.pose_heading);
  request->path.header.frame_id = "map";
  job_cut_ = JobCut();
  // Each path a section: no piece goes across the end of one.
  for (const auto& path : step.paths) {
    job_cut_.offsets.push_back(request->path.poses.size());
    request->section_start.push_back(static_cast<uint32_t>(request->path.poses.size()));
    for (const auto& p : path.poses) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose = PoseAt(p.x, p.y, p.heading);
      request->path.poses.push_back(pose);
    }
  }
  request->from_pose = static_cast<uint32_t>(step.start_index);
  for (const auto& ring : drivable) request->drivable_areas.push_back(ToPolygon(ring));
  for (const auto& ring : obstacles) request->obstacles.push_back(ToPolygon(ring));
  for (const auto& circle : step.circles) request->temporary_obstacles.push_back(ToPolygon(KeepOutRing(circle)));
  request->max_outside = transit_max_outside_;
  job_cut_.active = true;
  job_cut_.seq = ++drive_seq_;
  job_cut_.sent = MonotonicSeconds();
  const uint64_t seq = job_cut_.seq;
  job_cut_.request_id =
      cut_client_
          ->async_send_request(request,
                               [this, seq](rclcpp::Client<CutPlan>::SharedFuture f) { OnCutPlan(seq, *f.get()); })
          .request_id;
  RCLCPP_INFO(get_logger(), "Mowing: cutting the plan by %zu obstacles", step.circles.size());
}

void LogicNode::OnCutPlan(uint64_t seq, const CutPlan::Response& response) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!job_cut_.active || seq != job_cut_.seq) return;
    const std::vector<size_t> offsets = job_cut_.offsets;
    job_cut_ = JobCut();
    CutAnswer answer;
    answer.ok = response.success;
    answer.code = response.code;
    answer.message = response.message;
    // Back to the paths: the number from the first one of the request and the poses in it.
    for (size_t i = 0; i < response.piece_first.size() && i < response.piece_last.size(); ++i) {
      const size_t first = response.piece_first[i];
      const auto section = std::upper_bound(offsets.begin(), offsets.end(), first) - offsets.begin() - 1;
      if (section < 0) continue;
      const size_t at = offsets[static_cast<size_t>(section)];
      answer.pieces.push_back({static_cast<int>(section), first - at, response.piece_last[i] - at});
    }
    if (answer.ok) {
      RCLCPP_INFO(get_logger(), "Mowing: the plan cut: %s", response.message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Mowing: cutting the plan failed: %s (%s)", response.code.c_str(),
                  response.message.c_str());
    }
    core_.Call<&LogicCore::OnCutAnswer>(answer);
  }
  Step();
}

}  // namespace mowbite_logic
