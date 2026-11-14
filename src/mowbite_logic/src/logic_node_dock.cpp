// Docking at the node: the legs for the follower.
#include <cmath>
#include <cstdio>
#include <vector>

#include "logic_node.h"

namespace mowbite_logic {

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

}  // namespace mowbite_logic
