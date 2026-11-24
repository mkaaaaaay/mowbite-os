// The blade at the node: the lease of hw/blade, renewed while the logic wants the blade, and the height of each start.
#include <algorithm>

#include "logic_node.h"

namespace mowbite_logic {

// Every 100 ms: the timeouts of hw/status and the pose, the follower's sign of life (G22), then the lease of hw/blade,
// kept alive while the logic wants the blade and switched off at once when not.
void LogicNode::UpdateBlade() {
  bool stalled;
  bool ended;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double t = MonotonicSeconds();
    // Poses of the dock marker age between the ticks of the state timer too.
    if (core_.View().Docking()) core_.Call<&LogicCore::AgeDockPose>(t);
    // So does the obstacles/status: a stale one frees nothing from then on.
    if (core_.View().ObstacleGoesStale(t)) core_.Call<&LogicCore::AgeObstacles>(t);
    // Signs of life only when this node did not stand still just before (G34): they may wait in the queue.
    stalled = false;
    ended = false;
    if (fast_gap_.Judge(t)) {
      // Told once when the timeout passes; the state timer goes on telling it every second.
      const bool hw_late = !last_hw_ || t - *last_hw_ > hw_timeout_s_;
      const bool pose_late = !last_pose_ || t - *last_pose_ > pose_timeout_s_;
      if (hw_late && !hw_late_) core_.Call<&LogicCore::OnHardwareTimeout>();
      if (pose_late && !pose_late_) core_.Call<&LogicCore::OnPoseTimeout>();
      hw_late_ = hw_late;
      pose_late_ = pose_late;
      stalled = CheckFollowerLocked();
      // The calibration program's sign of life and the battery while it drives.
      ended = core_.Call<&LogicCore::TickCalibration>(t);
      // The setup program's sign of life, the station and a hold while a step moves the wheels.
      ended = core_.Call<&LogicCore::TickSetup>(t) || ended;
    }
    // A lock the setup program closed ends drives and stops the blade like a missing safety measurement.
    lock_watch_->Poll();
    ended = TellSafetyLocked() || ended;
    UpdateBladeLocked();
  }
  if (!stalled && !ended) return;
  // One zero command: the follower sends none any more, the driver's watchdog would stop the wheels only later.
  if (stalled) SendStop();
  Step();
}

bool LogicNode::TellSafetyLocked() {
  if (!safety_date_) return false;
  const bool measured = mowbite_hold::SafetyMeasured(*safety_date_, lock_watch_->last());
  if (safety_told_ == measured) return false;
  if (safety_told_ && !measured) {
    RCLCPP_WARN(get_logger(), "No autonomous drive: %s", lock_watch_->last().text.c_str());
  }
  safety_told_ = measured;
  core_.Call<&LogicCore::OnCapabilities>(measured, charge_stop_);
  return true;
}

void LogicNode::UpdateBladeLocked() {
  const auto& view = core_.View();
  const bool wanted = view.BladeWanted();
  const bool setup = view.InSetup();
  const bool once = view.SetupBladeOnce();
  const double t = MonotonicSeconds();
  if (!wanted) {
    blade_once_sent_ = false;
    if (!blade_on_requested_) return;
    blade_on_requested_ = false;
    SendBlade(false, 0, 0, blade_lease_s_);
    return;
  }
  // Mowing: the height of the area or of mow_height_mm, a change sent at once with the blade running (T10: the board
  // takes it and the blade turns on). A setup step keeps the height of its start.
  uint16_t height = blade_height_mm_;
  if (!setup || !blade_on_requested_) {
    const double setting = setup ? setup_blade_height_mm_
                                 : planner_.MowHeight(view.MowingAreaId()).value_or(logic_params_.Get("mow_height_mm"));
    height = BladeHeightMm(setting, blade_height_control_, blade_height_min_mm_, blade_height_max_mm_);
  }
  const bool new_height = blade_on_requested_ && height != blade_height_mm_;
  // A setup step: its lease, renewed at the latest after half of it, or once and never again (BLADE_ONCE).
  const double lease = setup && setup_blade_lease_s_ > 0 ? setup_blade_lease_s_ : blade_lease_s_;
  if (once) {
    if (blade_once_sent_) return;
  } else {
    blade_once_sent_ = false;
    const double renew = setup ? std::min(blade_renew_s_, lease / 2) : blade_renew_s_;
    if (blade_on_requested_ && !new_height && t - blade_sent_s_ < renew) return;
  }
  if (!blade_client_->service_is_ready()) {
    core_.Call<&LogicCore::OnBladeRefused>("hw/blade is not available", 0);
    return;
  }
  if (new_height) RCLCPP_INFO(get_logger(), "Cutting height %u mm, was %u mm", height, blade_height_mm_);
  blade_height_mm_ = height;
  blade_on_requested_ = true;
  blade_once_sent_ = once;
  blade_sent_s_ = t;
  SendBlade(true, blade_height_mm_, setup ? setup_blade_percent_ : 0, lease);
}

// Called with the mutex held.
void LogicNode::SendBlade(bool on, uint16_t height_mm, uint8_t speed_percent, double lease_s) {
  auto request = std::make_shared<mowbite_msgs::srv::SetBlade::Request>();
  request->on = on;
  request->height_mm = height_mm;
  request->speed_percent = speed_percent;
  double lease = lease_s;
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
