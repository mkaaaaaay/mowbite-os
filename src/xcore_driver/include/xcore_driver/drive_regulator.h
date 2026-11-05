#ifndef XCORE_DRIVER_DRIVE_REGULATOR_H
#define XCORE_DRIVER_DRIVE_REGULATOR_H

#include <optional>
#include <string>

namespace xcore {

struct RegulatorConfig {
  // The distance the board was configured with; it splits the twist into the wheels the same way.
  double wheel_distance = 0.3556;
  // A wheel turns at duty_slope x (duty - duty_deadband) m/s above the deadband and stands below it; the duty for a
  // wheel speed v is v / duty_slope + duty_deadband. The firmware treats 1 m/s as full duty.
  double duty_slope = 1.0;
  double duty_deadband = 0.0;
  // Largest duty either wheel gets; a larger one shortens both duties by the same factor.
  double max_duty = 0.3;
  // Correction on the measured wheel speed; off until the ticks are checked on the device.
  bool feedback = false;
  double kp = 0.3;
  double ki = 0.6;
  double integral_limit = 0.2;
  // A wheel measured against its setpoint for this long, at more than reverse_speed, stops the drive.
  double reverse_time_s = 0.5;
  double reverse_speed = 0.05;
};

struct DutyTwist {
  // The twist for the board: v and ω that give the wanted duty on each wheel.
  double linear = 0;
  double angular = 0;
  double left_duty = 0;
  double right_duty = 0;
};

// Why the speed model cannot drive, or nothing: a slope of 0 or less, a deadband below 0 or not below max_duty (other
// than with max_duty 0, which locks the wheels), or a value that is no number.
std::optional<std::string> CheckSpeedModel(const RegulatorConfig& config);
// The fastest wheel speed in m/s the model reaches within max_duty, 0 if it reaches none.
double ReachableWheelSpeed(const RegulatorConfig& config);

// Turns a wanted twist in m/s and rad/s into the duty twist the board applies to its motors, with feed forward and,
// once enabled, a PI correction per wheel on the speeds from the ticks.
class DriveRegulator {
 public:
  explicit DriveRegulator(const RegulatorConfig& config) : config_(config) {
  }

  // Measured wheel speeds in m/s, forward positive.
  void OnMeasured(double left, double right, double now);
  DutyTwist Update(double linear, double angular, double now);
  void Reset();
  // A wheel turned against its setpoint: with feedback a wrong sign of the ticks would run away.
  bool fault() const {
    return fault_;
  }
  void ClearFault() {
    fault_ = false;
  }

 private:
  struct Wheel {
    double integral = 0;
    double measured = 0;
    double against_since = -1;
  };

  double Duty(Wheel& wheel, double target, double dt, bool fresh, double now);

  RegulatorConfig config_;
  Wheel left_;
  Wheel right_;
  double last_update_ = -1;
  double last_measured_ = -1;
  bool fault_ = false;
};

}  // namespace xcore

#endif  // XCORE_DRIVER_DRIVE_REGULATOR_H
