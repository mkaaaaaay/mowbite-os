#ifndef XCORE_DRIVER_XCORE_CONTROL_H
#define XCORE_DRIVER_XCORE_CONTROL_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "xcore_driver/odometry.h"
#include "xcore_driver/wire.h"

namespace xcore {

struct ControlConfig {
  // The emergency input goes out this often; the board latches TIMEOUT_HIGH_LEVEL after 1 s without one.
  double emergency_period_s = 0.25;
  // Drive commands go out this often while the wheels are commanded; the board stops them after 1 s without one.
  double drive_period_s = 0.1;
  // Silence on cmd_vel after which the wheels get a stop.
  double cmd_vel_timeout_s = 0.5;
  int repeat_count = 3;
  double repeat_interval_s = 0.1;
  double max_linear_speed = 0.3;
  double max_angular_speed = 0.3;
  // Without an output of the emergency service for this long, motion and blade are blocked.
  double telemetry_timeout_s = 1.0;

  bool allow_blade = false;
  uint16_t blade_speed_percent = 100;
  double blade_lease_max_s = 2.0;
  // The blade setpoint is sent again this often while on and while off; the board stops the blade after 10 s.
  double blade_period_s = 1.0;
  // A blade the board still reports at blade_still_rpm or more this long after it went off latches MOWER_NOT_STOPPED.
  double blade_stop_time_s = 5.0;
  double blade_still_rpm = 100;

  // The station delivers this charge voltage or more while the mower stands on its contacts.
  double dock_voltage = 20.0;
  int dock_debounce_samples = 3;
  // Charging allowed goes out this often.
  double charge_period_s = 1.0;
  // Without a battery report for this long the battery fields read as unknown.
  double battery_timeout_s = 5.0;

  OdometryConfig odometry;
};

// Bits of the emergency reason in hw/status. 0 to 12 come from the board with its numbers; the others are the driver's.
constexpr uint32_t kReasonLatch = 1u << 0;
constexpr uint32_t kReasonTimeoutInputs = 1u << 1;
constexpr uint32_t kReasonStop = 1u << 2;
constexpr uint32_t kReasonLift = 1u << 3;
constexpr uint32_t kReasonCollision = 1u << 5;
constexpr uint32_t kReasonHighLevel = 1u << 7;
constexpr uint32_t kReasonFirmwareIncompatible = 1u << 12;
constexpr uint32_t kBoardReasons = (1u << 13) - 1;
constexpr uint32_t kReasonTilt = 1u << 13;
constexpr uint32_t kReasonFence = 1u << 14;
constexpr uint32_t kReasonStuck = 1u << 15;
constexpr uint32_t kReasonMowerNotStopped = 1u << 17;
// The bits a node above the driver may latch with over hw/emergency.
constexpr uint32_t kHostLatchReasons = kReasonHighLevel | kReasonTilt | kReasonFence | kReasonStuck;

enum class BladeOffReason : uint8_t {
  kNone = 0,
  kRequest = 1,
  kLeaseExpired = 2,
  kBlocked = 3,
  kDock = 4,
  kReconnect = 5,
};

struct BladeRequest {
  bool on = false;
  // 0 takes blade_speed_percent of the config, otherwise 1-100.
  uint16_t speed_percent = 0;
  // The xCore has no height motor: only 0 is accepted.
  uint16_t height_mm = 0;
  double lease_s = 0;
};

enum class BladeResult { kOk, kNotAllowed, kInvalidLease, kInvalidSpeed, kInvalidHeight, kBlocked, kDocked };

const char* BladeResultText(BladeResult result);

// A value for an input of a service.
struct InputValue {
  uint16_t service_id = 0;
  uint16_t channel_id = 0;
  Bytes value;
};

struct Battery {
  bool known = false;
  double voltage = 0;
  double charge_voltage = 0;
  double charge_current = 0;
  // 0-1 as the board reports it.
  double fraction = 0;
  std::string charging_status;
};

struct MotorReport {
  bool known = false;
  uint8_t status = 0;
  float temperature = 0;
  float current = 0;
  float rpm = 0;
};

struct ControlStatus {
  bool blocked = true;
  uint32_t emergency_reason = 0;
  bool latched = false;
  bool telemetry_ok = false;
  // Live flags from the inputs, false while unknown.
  bool lift = false;
  bool stop = false;
  bool bumper_left = false;
  bool bumper_right = false;
  bool rain = false;
  bool dock_contact = false;
  bool dock_contact_raw = false;
  bool moving = false;
  bool blade_on = false;
  BladeOffReason blade_off_reason = BladeOffReason::kNone;
  Battery battery;
  MotorReport blade_motor;
  MotorReport left_motor;
  MotorReport right_motor;
};

// Which index of the input service's mask means what; -1 where the mower has no such input.
struct InputMap {
  std::vector<int> stop;
  std::vector<int> lift;
  std::vector<int> bumper_left;
  std::vector<int> bumper_right;
};

// Decides what goes to the services of the xCore and when. Time is passed in (monotonic seconds); nothing here
// touches the network. The link layer sends the values only to services it has claimed and configured.
class XcoreControl {
 public:
  XcoreControl(const ControlConfig& config, const InputMap& inputs);

  void OnCmdVel(double linear, double angular, double now);
  BladeResult SetBlade(const BladeRequest& request, double now);
  // Bits must be within kHostLatchReasons; 0 means HIGH_LEVEL. Returns false for other bits.
  bool LatchEmergency(uint32_t bits);
  // Clears the host's own latches and asks the board to clear its own; the board keeps a cause that is still there.
  void ReleaseEmergency(double now);
  void SetCharging(bool allowed);
  // All services the driver needs are claimed and configured, and their firmware fits; false blocks.
  void SetServicesReady(bool ready, bool firmware_compatible);

  // An output of a service, with the board's time stamp. Returns a wheel twist when ticks gave one.
  std::optional<WheelTwist> OnOutput(uint16_t service_id, uint16_t channel_id, const Bytes& value,
                                     uint64_t board_time_us, double now);
  // The board restarted or a service was claimed again: its state is unknown.
  void OnServiceRestart(uint16_t service_id, double now);

  std::vector<InputValue> Tick(double now);
  ControlStatus Status(double now) const;

 private:
  struct Repeat {
    int remaining = 0;
    double next_s = 0;
  };

  bool Blocked(double now) const;
  bool TelemetryStale(double now) const;
  uint32_t Reason(double now) const;
  bool InputActive(const std::vector<int>& indices) const;
  bool DockContact() const;
  void DropBlade(BladeOffReason reason);
  bool Due(std::optional<double>& next, double period, double now);
  InputValue Twist(double linear, double angular) const;
  InputValue MowerSpeed(float speed) const;

  ControlConfig config_;
  InputMap inputs_;

  bool services_ready_ = false;
  bool firmware_compatible_ = true;

  std::optional<uint16_t> board_reason_;
  std::optional<double> last_emergency_s_;
  uint32_t host_latch_ = 0;
  // A release asks the board to clear this many more times.
  int clear_remaining_ = 0;
  std::optional<double> next_emergency_s_;

  std::optional<uint64_t> active_inputs_;
  bool rain_ = false;

  struct Command {
    double linear;
    double angular;
  };
  std::optional<Command> command_;
  double last_cmd_s_ = 0;
  bool moving_ = false;
  Repeat stop_repeat_;
  std::optional<double> next_drive_s_;

  struct BladeLease {
    float speed;
    double until_s;
  };
  std::optional<BladeLease> blade_;
  BladeOffReason blade_off_reason_ = BladeOffReason::kNone;
  std::optional<double> blade_off_since_s_;
  bool blade_turning_ = false;
  bool mower_not_stopped_ = false;
  Repeat blade_off_repeat_;
  std::optional<double> next_blade_s_;

  bool charging_allowed_ = true;
  std::optional<double> next_charge_s_;
  int dock_samples_ = 0;
  bool dock_raw_ = false;

  Battery battery_;
  std::optional<double> last_battery_s_;
  MotorReport blade_motor_;
  MotorReport left_motor_;
  MotorReport right_motor_;

  TickOdometry odometry_;
};

}  // namespace xcore

#endif  // XCORE_DRIVER_XCORE_CONTROL_H
