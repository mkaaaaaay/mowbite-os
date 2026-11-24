// Coverage plans at the node: the request to the planner, its answer, and those that wait too long.
#include <string>

#include "logic_node.h"

namespace mowbite_logic {

namespace {

// An override for the planner, typed by its value (int an integer, double a double).
template <typename T>
rcl_interfaces::msg::Parameter Override(const std::string& name, T value) {
  return rclcpp::Parameter(name, value).to_parameter_msg();
}

}  // namespace

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
    if (job_cut_.active && t - job_cut_.sent > plan_timeout_s_) {
      cut_client_->remove_pending_request(job_cut_.request_id);
      job_cut_ = JobCut();
      CutAnswer late;
      late.code = "PLANNER_TIMEOUT";
      late.message = "No answer from the planner in " + std::to_string(static_cast<int>(plan_timeout_s_)) + " s";
      core_.Call<&LogicCore::OnCutAnswer>(late);
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
  answer.code = r.code;
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

}  // namespace mowbite_logic
