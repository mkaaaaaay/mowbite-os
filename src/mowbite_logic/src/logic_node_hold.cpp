// The hold of the updater at the node: system.hold, hold.json across a restart, its end and mowbite/hold.
#include <string>

#include "logic_node.h"

namespace mowbite_logic {

namespace {

// mowbite/hold goes out again this often while nothing changes.
constexpr double kHoldMirrorS = 30.0;

}  // namespace

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
  const bool scheduled = ScheduleBlocksHoldLocked(WallSeconds());
  return json{{"state", state.state},
              {"is_charging", state.is_charging},
              {"hold_until", HoldUntilLocked()},
              {"sub_state", state.sub_state},
              {"idle", core_.View().HoldAllowed() && !scheduled},
              {"emergency_reason", state.emergency_reason},
              {"next_start", next_start_ > 0 ? json(next_start_) : json()},
              {"scheduled_start", scheduled}};
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

}  // namespace mowbite_logic
