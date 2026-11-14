// The blade at the node: the lease of hw/blade, renewed while the logic wants the blade, and the height of each start.
#include <algorithm>

#include "logic_node.h"

namespace mowbite_logic {

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
    SendBlade(false, 0);
    return;
  }
  if (blade_on_requested_ && t - blade_sent_s_ < blade_renew_s_) return;
  if (!blade_client_->service_is_ready()) {
    core_.Call<&LogicCore::OnBladeRefused>("hw/blade is not available", 0);
    return;
  }
  // The renewals keep the height of the start: a change of mow_height_mm waits for the next start.
  if (!blade_on_requested_) {
    blade_height_mm_ = BladeHeightMm(logic_params_.Get("mow_height_mm"), blade_height_control_, blade_height_min_mm_,
                                     blade_height_max_mm_);
  }
  blade_on_requested_ = true;
  blade_sent_s_ = t;
  SendBlade(true, blade_height_mm_);
}

// Called with the mutex held.
void LogicNode::SendBlade(bool on, uint16_t height_mm) {
  auto request = std::make_shared<mowbite_msgs::srv::SetBlade::Request>();
  request->on = on;
  request->height_mm = height_mm;
  double lease = blade_lease_s_;
  if (blade_lease_max_s_ > 0) lease = std::min(lease, static_cast<double>(blade_lease_max_s_));
  request->lease_s = static_cast<float>(lease);
  blade_client_->async_send_request(request, [this, on](rclcpp::Client<mowbite_msgs::srv::SetBlade>::SharedFuture f) {
    const auto& response = f.get();
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

}  // namespace mowbite_logic
