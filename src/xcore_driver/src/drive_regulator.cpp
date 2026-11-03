#include "xcore_driver/drive_regulator.h"

#include <algorithm>
#include <cmath>

namespace xcore {

namespace {

// Measured speeds older than this do not correct.
constexpr double kMeasuredMaxAge = 0.2;

}  // namespace

void DriveRegulator::OnMeasured(double left, double right, double now) {
  left_.measured = left;
  right_.measured = right;
  last_measured_ = now;
}

void DriveRegulator::Reset() {
  left_ = Wheel{};
  right_ = Wheel{};
  last_update_ = -1;
}

double DriveRegulator::Duty(Wheel& wheel, double target, double dt, bool fresh, double now) {
  if (target == 0) {
    wheel.integral = 0;
    wheel.against_since = -1;
    return 0;
  }
  double duty = target / config_.full_duty_speed;
  if (fresh) {
    bool against = wheel.measured * target < 0 && std::fabs(wheel.measured) > config_.reverse_speed;
    if (!against) {
      wheel.against_since = -1;
    } else if (wheel.against_since < 0) {
      wheel.against_since = now;
    } else if (now - wheel.against_since >= config_.reverse_time_s) {
      fault_ = true;
    }
  }
  if (config_.feedback && fresh) {
    double error = target - wheel.measured;
    wheel.integral = std::clamp(wheel.integral + error * dt, -config_.integral_limit, config_.integral_limit);
    duty += config_.kp * error + config_.ki * wheel.integral;
  }
  return std::clamp(duty, -config_.max_duty, config_.max_duty);
}

DutyTwist DriveRegulator::Update(double linear, double angular, double now) {
  DutyTwist out;
  if (fault_) {
    Reset();
    return out;
  }
  double dt = last_update_ < 0 ? 0 : std::clamp(now - last_update_, 0.0, 0.2);
  last_update_ = now;
  bool fresh = last_measured_ >= 0 && now - last_measured_ <= kMeasuredMaxAge;
  double half = config_.wheel_distance / 2;
  out.left_duty = Duty(left_, linear - angular * half, dt, fresh, now);
  out.right_duty = Duty(right_, linear + angular * half, dt, fresh, now);
  if (fault_) {
    Reset();
    return DutyTwist{};
  }
  out.linear = (out.left_duty + out.right_duty) / 2;
  out.angular = (out.right_duty - out.left_duty) / config_.wheel_distance;
  return out;
}

}  // namespace xcore
