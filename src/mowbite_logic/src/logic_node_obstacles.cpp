// The hold for obstacles at the node: obstacles/status into the core, logic/drive_gate for the follower and
// obstacles/action for the list of mowbite_obstacles.
#include <chrono>
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
    sample.tracks.push_back(
        {track.id, track.kind, track.confirmed, track.in_view, static_cast<double>(track.ahead), track.zone});
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

}  // namespace mowbite_logic
