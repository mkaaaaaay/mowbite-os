#ifndef XCORE_DRIVER_XCORE_CONTROL_H
#define XCORE_DRIVER_XCORE_CONTROL_H

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_hold/restart.h"
#include "xcore_driver/drive_regulator.h"
#include "xcore_driver/odometry.h"
#include "xcore_driver/wire.h"

namespace xcore {

// Caps of the speed limits in every build: driving at most 0.3 m/s, turning at most 0.3 rad/s. Only the measurement
// build (CMake option XCORE_MEASUREMENT_BUILD, the image of the safety measurement) takes measurement_speed_limit as
// the cap for driving instead, up to kMeasurementSpeedCapMax.
constexpr double kLinearSpeedCap = 0.3;
constexpr double kAngularSpeedCap = 0.3;
constexpr double kMeasurementSpeedCapMax = 0.6;

// measurement_speed_limit as the cap for driving; throws std::invalid_argument unless it is a number from 0 to
// kMeasurementSpeedCapMax.
double MeasurementSpeedCap(double measurement_speed_limit);
// A limit of the parameters under its cap. Throws std::invalid_argument for a negative value or one that is no number,
// as either would undo the clamping it is meant for.
double CappedLimit(const char* name, double value, double cap);

struct ControlConfig {
  // Started with inputs.learn: INPUTS_LEARNING stands and holds the board. A release ends what else is latched and
  // is answered with LEARNING.
  bool learn_inputs = false;
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
  // Without an output of the emergency service for this long, motion and blade are blocked; it reports every second.
  double telemetry_timeout_s = 2.5;

  bool allow_blade = false;
  uint16_t blade_speed_percent = 100;
  double blade_lease_max_s = 2.0;
  // The blade setpoint is sent again this often while on and while off; the board stops the blade after 10 s.
  double blade_period_s = 1.0;
  // A blade the board still reports at blade_still_rpm or more this long after it went off latches MOWER_NOT_STOPPED.
  double blade_stop_time_s = 5.0;
  double blade_still_rpm = 100;

  // The station delivers this charge voltage or more while the mower stands on its contacts; reported every second.
  double dock_voltage = 5.0;
  int dock_debounce_samples = 2;
  // Without a battery report for this long the battery fields read as unknown.
  double battery_timeout_s = 5.0;

  // Tilt from gravity in the board's IMU after the remap: above tilt_limit_deg for tilt_time_s latches TILT; a release
  // needs a sample younger than imu_fresh_s below tilt_release_deg.
  double tilt_limit_deg = 40.0;
  double tilt_time_s = 0.5;
  double tilt_release_deg = 26.0;
  double imu_fresh_s = 0.5;
  // Without a plausible IMU sample for this long, motion and blade are blocked (IMU_TIMEOUT). Driving without a wheel
  // twist from the ticks for ticks_timeout_s latches WHEEL_RUNAWAY: the wheel checks need them. 0 switches either
  // off, which only tests of other checks do: CheckControlConfig refuses it.
  double imu_timeout_s = 0.5;
  double ticks_timeout_s = 1.0;
  // Wheels against the command, checked until uncommanded_window_s after the last drive setpoint other than 0: a wheel
  // above uncommanded_speed for uncommanded_time_s, uncommanded_after_s or more after that setpoint, latches; so does a
  // wheel above overspeed_factor times the fastest commandable wheel speed for overspeed_time_s. Later, pushing the
  // mower by hand latches nothing.
  double uncommanded_speed = 0.1;
  double uncommanded_after_s = 1.0;
  double uncommanded_window_s = 5.0;
  double uncommanded_time_s = 0.2;
  double overspeed_factor = 1.3;
  double overspeed_time_s = 0.5;
  // A gap this long between two passes of the I/O thread counts as a lost link: the board may have stopped and would
  // otherwise take the next setpoints.
  double io_stall_s = 0.8;
  // The longest hw/silence it takes (s), 0 for none.
  double silence_max_s = 0;

  OdometryConfig odometry;
  RegulatorConfig regulator;
};

// Throws std::invalid_argument naming the first value of the driver's own checks that is no finite number in its
// range: a NaN or a huge timeout, lease, tilt or wheel limit would switch its check off.
void CheckControlConfig(const ControlConfig& config);

// Lowers max_linear_speed and max_angular_speed to what the speed model reaches within max_duty: a wheel at most
// ReachableWheelSpeed, driving straight or turning on the spot. The capabilities and the wheel checks use the result.
void ApplyReachableSpeeds(ControlConfig& config);

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
constexpr uint32_t kReasonImuTimeout = 1u << 16;
constexpr uint32_t kReasonMowerNotStopped = 1u << 17;
// The driver's own checks: a wheel against the drive command, another host at the board, a stall of the I/O thread.
constexpr uint32_t kReasonWheelRunaway = 1u << 18;
constexpr uint32_t kReasonForeignHost = 1u << 19;
constexpr uint32_t kReasonDriverStall = 1u << 20;
// The driver started with inputs.learn: the board watches the candidate lines and none of them stops anything.
constexpr uint32_t kReasonInputsLearning = 1u << 22;
// Causes the board latches; the driver keeps them in its reason until the release, also across a restart of the board.
constexpr uint32_t kLatchedCauses = (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5) | (1u << 9) | (1u << 10) | (1u << 11);
// The bits a node above the driver may latch with over hw/emergency.
using mowbite_hold::kHostLatchReasons;

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

enum class BladeResult {
  kOk,
  kNotAllowed,
  kInvalidLease,
  kInvalidSpeed,
  kInvalidHeight,
  kBlocked,
  kDocked,
  kHeld,
  kSilent
};

// The shortest hw/silence (s).
constexpr double kSilenceMinS = 0.2;
// hw/silence needs an active setup/motion this recent (SilenceDriver.srv).
constexpr double kSetupMotionFreshS = 1.0;

const char* BladeResultText(BladeResult result);

// Why a release over hw/emergency is refused; nothing is cleared then, but with LEARNING all except INPUTS_LEARNING.
struct ReleaseRefusal {
  // STILL_TILTED, TILT_UNKNOWN, FOREIGN_HOST, LATCH_CHANGED, LEARNING or SILENT.
  std::string code;
  std::string message;
  // STILL_TILTED: the angle and the release limit in degrees; TILT_UNKNOWN: seconds since the last IMU sample. NaN
  // where there is none.
  double value = std::numeric_limits<double>::quiet_NaN();
  double limit = std::numeric_limits<double>::quiet_NaN();
};

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
  // 0-1 as the board reports it, linear between the empty and the full voltage.
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

// Status of a motor controller as the board reports it.
constexpr uint8_t kEscDisconnected = 99;
constexpr uint8_t kEscFault = 100;
constexpr uint8_t kEscBlocked = 150;
constexpr uint8_t kEscOk = 200;
constexpr uint8_t kEscRunning = 201;

// The code of hw/status for a controller status: fault -1, disconnected -2, other unknown values -3, idle 0,
// running 1, blocked 2.
int8_t MotorStatusCode(uint8_t esc_status);

struct ControlStatus {
  bool blocked = true;
  // An update holds the mower: motion and blade blocked without an emergency.
  bool held = false;
  uint32_t emergency_reason = 0;
  bool latched = false;
  // Goes up with every reason that latches, never 0 once one did.
  uint32_t latch_count = 0;
  bool telemetry_ok = false;
  // Live flags from the inputs, false while unknown.
  bool lift = false;
  bool stop = false;
  bool bumper_left = false;
  bool bumper_right = false;
  // The mask of the input service, bit i for line i of the table; nothing while unknown.
  std::optional<uint64_t> inputs;
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
  // A refusal because of an emergency sets emergency_reason to its bits as hw/status shows them, any other answer
  // to 0.
  BladeResult SetBlade(const BladeRequest& request, double now, uint32_t* emergency_reason = nullptr);
  // Bits must be within kHostLatchReasons; 0 means HIGH_LEVEL. Returns false for other bits.
  bool LatchEmergency(uint32_t bits);
  // Clears the host's own latches and asks the board to clear its own; the board sets a cause again that is still there.
  // Returns why a release is refused (the mower still tilted, no fresh IMU after a tilt, a latch_count that is not 0
  // and not the current one, the driver learning its inputs); nothing is cleared then.
  std::optional<ReleaseRefusal> ReleaseEmergency(double now, uint32_t latch_count = 0);
  // Latches FOREIGN_HOST for another host the node found at the board; the text goes to TakeErrors.
  void LatchForeignHost(const std::string& why, double now);
  // Acceleration in m/s² in the body frame from the board's IMU.
  void OnImu(double ax, double ay, double az, double now);
  // The I/O thread is alive without a link to tick.
  void Alive(double now);
  // Texts of the faults latched since the last call, for the log.
  std::vector<std::string> TakeErrors();
  // All services the driver needs are claimed and configured, and their firmware fits; false blocks.
  void SetServicesReady(bool ready, bool firmware_compatible);
  // The hold of an update (hold.json): motion and blade blocked without an emergency and without a latch on the board;
  // moving wheels get their stops, the blade goes off. The board charges on its own as before.
  void SetHold(bool held);

  // An output of a service, with the board's time stamp. Returns a wheel twist when ticks gave one.
  std::optional<WheelTwist> OnOutput(uint16_t service_id, uint16_t channel_id, const Bytes& value,
                                     uint64_t board_time_us, double now);
  // The board restarted or a service was claimed again: its state is unknown.
  void OnServiceRestart(uint16_t service_id, double now);
  // A service runs (again): wheels or blade not commanded get their stops, so it never starts with an old setpoint.
  void OnServiceRunning(uint16_t service_id, double now);

  // setup/motion of the setup program: active while one of its steps runs.
  void OnSetupMotion(bool active, double now);
  // hw/silence: from now on nothing goes to the board for seconds, drive commands are dropped, blade requests and
  // releases refused; then HIGH_LEVEL latches, the blade gets its offs and the wheels three stops. With blade it begins
  // also while the blade is on or turning: the board has to stop it. Empty when it began, else why not: BUSY,
  // OUT_OF_RANGE, NO_SETUP (no active setup/motion within kSetupMotionFreshS), CHARGING or BLADE.
  std::string Silence(double seconds, double now, bool blade = false);
  bool Silent(double now) const;
  // Why hw/end is refused now, empty if it may end: CHARGING or BLADE.
  std::string EndRefusal() const;

  std::vector<InputValue> Tick(double now);
  // Wheels and blade to 0, for a driver that stops; the board's own timeouts follow.
  std::vector<InputValue> StopInputs() const;
  int repeat_count() const { return config_.repeat_count; }
  ControlStatus Status(double now) const;

 private:
  struct Repeat {
    int remaining = 0;
    double next_s = 0;
  };

  bool Blocked(double now) const;
  // The driver sends HIGH_LEVEL|LATCH to the board: a latch from above or one of its own causes.
  bool HoldsBoard() const;
  // The reasons the driver keeps until a release.
  uint32_t Kept() const;
  // The count with the reasons kept now: one more than counted when one of them was not kept at the last count.
  // A status between two ticks so never shows a latch with the count of before it.
  uint32_t LatchCount() const;
  // Counts a reason that is kept now and was not at the last call.
  void CountLatches();
  void Fault(uint32_t bit, const std::string& why, double now);
  void CheckWheels(const WheelTwist& twist, double now);
  void CheckStall(double now);
  bool TelemetryStale(double now) const;
  bool ImuStale(double now) const;
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
  bool held_ = false;

  std::optional<uint16_t> board_reason_;
  std::optional<double> last_emergency_s_;
  uint32_t host_latch_ = 0;
  // Causes the board reported with its latch, held until a release.
  uint32_t latched_causes_ = 0;
  // Every reason that latched since the start, and the reasons counted in it that still stand.
  uint32_t latch_count_ = 0;
  uint32_t counted_ = 0;
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

  // The board reported no TIMEOUT_HIGH_LEVEL since the emergency service came up; a later one means lost messages.
  bool board_heard_us_ = false;
  bool link_lost_ = false;
  // The link counts as lost because the I/O thread stalled, not because the board missed messages.
  bool stalled_ = false;

  int dock_samples_ = 0;
  bool dock_raw_ = false;

  Battery battery_;
  std::optional<double> last_battery_s_;
  MotorReport blade_motor_;
  MotorReport left_motor_;
  MotorReport right_motor_;

  TickOdometry odometry_;
  DriveRegulator regulator_;
  bool drive_fault_ = false;

  // Bits of the driver's own checks (wheels, another host) that latched, held until a release.
  uint32_t fault_bits_ = 0;
  std::vector<std::string> errors_;

  bool tilt_latched_ = false;
  double tilt_deg_ = 0;
  std::optional<double> last_imu_s_;
  std::optional<double> tilted_since_s_;

  // When a drive setpoint other than 0 went out last; since when the wheels are commanded; the last twist of the ticks.
  std::optional<double> last_drive_s_;
  std::optional<double> driving_since_s_;
  std::optional<double> last_twist_s_;
  std::optional<double> uncommanded_since_s_;
  std::optional<double> overspeed_since_s_;

  std::optional<double> last_alive_s_;
  std::optional<double> silent_until_s_;
  // The last active setup/motion.
  std::optional<double> setup_motion_s_;
};

}  // namespace xcore

#endif  // XCORE_DRIVER_XCORE_CONTROL_H
