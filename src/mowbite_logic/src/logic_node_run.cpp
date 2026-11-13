// The runs of the schedule at the node: logic/start_run, logic/drop_tasks, logic/run and schedule/next_start.
#include <algorithm>
#include <string>
#include <vector>

#include "logic_node.h"

namespace mowbite_logic {

void LogicNode::SetUpRuns() {
  run_pub_ = create_publisher<mowbite_msgs::msg::RunStatus>("logic/run", LatchedQos(1));
  next_start_sub_ = create_subscription<std_msgs::msg::Int64>("schedule/next_start", LatchedQos(1),
                                                              [this](const std_msgs::msg::Int64::ConstSharedPtr& msg) {
                                                                std::lock_guard<std::mutex> lock(mutex_);
                                                                next_start_ = msg->data;
                                                              });
  start_run_srv_ = create_service<mowbite_msgs::srv::StartRun>(
      "logic/start_run",
      [this](const std::shared_ptr<mowbite_msgs::srv::StartRun::Request>& request,
             const std::shared_ptr<mowbite_msgs::srv::StartRun::Response>& response) {
        OnStartRun(*request, *response);
      });
  drop_tasks_srv_ = create_service<mowbite_msgs::srv::DropTasks>(
      "logic/drop_tasks",
      [this](const std::shared_ptr<mowbite_msgs::srv::DropTasks::Request>& request,
             const std::shared_ptr<mowbite_msgs::srv::DropTasks::Response>& response) {
        OnDropTasks(*request, *response);
      });
}

void LogicNode::OnStartRun(const mowbite_msgs::srv::StartRun::Request& request,
                           mowbite_msgs::srv::StartRun::Response& response) {
  ScheduleRun run;
  run.run_id = request.run_id;
  run.area_ids = request.area_ids;
  run.pass_over = request.pass_over;
  run.restart = request.restart;
  run.has_angle = request.has_angle;
  run.angle_offset_deg = request.angle_offset_deg;
  run.dark_allowed = request.dark_allowed;
  run.until_sunset = request.until_sunset;
  run.end_in_s = request.end_in_s;
  Reason why;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    why = core_.Call<&LogicCore::StartRun>(run, NodeProblemLocked("mowing.start"), MonotonicSeconds());
  }
  response.accepted = why.empty();
  response.code = why.code;
  response.message = why.message;
  response.data = why.fields.dump();
  if (why.empty()) {
    RCLCPP_INFO(get_logger(), "logic/start_run %s: announced", run.run_id.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "logic/start_run %s refused: %s %s", run.run_id.c_str(), why.code.c_str(),
                why.message.c_str());
  }
  Step();
}

void LogicNode::OnDropTasks(const mowbite_msgs::srv::DropTasks::Request& request,
                            mowbite_msgs::srv::DropTasks::Response& response) {
  Reason why;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    why = core_.Call<&LogicCore::DropTasks>(request.run_id, request.area_ids);
  }
  response.accepted = why.empty();
  response.message = why.empty() ? "taken" : why.code + ": " + why.message;
  if (!why.empty()) {
    RCLCPP_WARN(get_logger(), "logic/drop_tasks %s refused: %s", request.run_id.c_str(), response.message.c_str());
  }
  Step();
}

// Per run only its latest phase of the step, in the order of those: a start taken back in the same step (AbandonStart)
// says ENDED without the MOWING before it. Called without the mutex.
void LogicNode::PublishRuns(const std::vector<Effect>& effects) {
  std::vector<const RunChanged*> latest;
  for (const auto& effect : effects) {
    const auto* change = std::get_if<RunChanged>(&effect);
    if (!change) continue;
    latest.erase(std::remove_if(latest.begin(), latest.end(),
                                [change](const RunChanged* seen) { return seen->run_id == change->run_id; }),
                 latest.end());
    latest.push_back(change);
  }
  for (const RunChanged* change : latest) {
    mowbite_msgs::msg::RunStatus msg;
    msg.run_id = change->run_id;
    msg.phase = static_cast<uint8_t>(change->phase);
    msg.end_code = change->end_code;
    msg.detail = change->detail;
    run_pub_->publish(msg);
  }
}

// L8: no hold within kHoldBeforeStartS before the next start of the schedule, nor while it waits or is announced.
// Called with the mutex held.
bool LogicNode::ScheduleBlocksHoldLocked(int64_t wall) const {
  return StartBlocksHold(next_start_, wall);
}

}  // namespace mowbite_logic
