#include "xcore_driver/odometry.h"

namespace xcore {

std::optional<WheelTwist> TickOdometry::OnTicks(uint32_t left, uint32_t right, uint64_t board_time_us) {
  Sample now{left, right, static_cast<uint32_t>(board_time_us)};
  std::optional<Sample> last = last_;
  last_ = now;
  if (!last) return std::nullopt;
  // Unsigned differences stay right across the wrap of the counters and of the 32-bit clock.
  uint32_t elapsed_us = now.time_us - last->time_us;
  double dt = elapsed_us * 1e-6;
  if (elapsed_us == 0 || dt > config_.max_gap_s) return std::nullopt;
  auto delta = [](uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b); };
  WheelTwist twist;
  twist.dt = dt;
  twist.left = config_.left_sign * delta(now.left, last->left) / config_.ticks_per_meter / dt;
  twist.right = config_.right_sign * delta(now.right, last->right) / config_.ticks_per_meter / dt;
  twist.linear = (twist.left + twist.right) / 2;
  twist.angular = (twist.right - twist.left) / config_.wheel_distance;
  return twist;
}

}  // namespace xcore
