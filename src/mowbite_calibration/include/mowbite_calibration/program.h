#ifndef MOWBITE_CALIBRATION_PROGRAM_H
#define MOWBITE_CALIBRATION_PROGRAM_H

#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_calibration/lane.h"

namespace mowbite_calibration {

// Phases of the drive, named as in the recording (mowbite-calib/1) and the CALIBRATION events.
enum class Phase { kStart, kAlign, kStand, kLine, kSpinLeft, kSpinRight, kEnd };
const char* PhaseName(Phase phase);

struct ProgramConfig {
  // Rounds of line, stand, turn on the spot, stand.
  int rounds = 4;
  // Lines go forward only: up to line_speed (m/s), faster and slower by line_accel (m/s²), and while the end is ahead
  // never slower than line_min_speed, below which the wheels of the Tron stand still.
  double line_speed = 0.2;
  double line_accel = 0.1;
  double line_min_speed = 0.08;
  // A line keeps to the line through its start, by the pose of the localization.
  double steer_heading_gain = 1.0;
  double steer_cross_gain = 1.0;
  double steer_max = 0.05;
  // Turns on the spot: spin_angle (rad) by the gyro at spin_rate (rad/s), reached at spin_accel (rad/s²); the first
  // left at the far end of the lane, the next right at its start. ALIGN turns to the lane by the pose, only when more
  // than align_skip off, and stops within align_tolerance or once past it.
  double spin_angle = 3.0 * M_PI;
  double spin_rate = 0.3;
  double spin_accel = 0.3;
  double align_skip = 5.0 * M_PI / 180.0;
  double align_tolerance = 3.0 * M_PI / 180.0;
  // Stands (s): the first, and the one after every line and turn.
  double first_stand = 10.0;
  double stand = 5.0;

  // Checks while it runs; a check that holds for hold_time (s) in a row ends the drive.
  double hold_time = 0.5;
  // NO_RTK: an RTK fixed position of rtk_accuracy (m) or better at least every rtk_timeout (s).
  double rtk_accuracy = 0.03;
  double rtk_timeout = 1.0;
  // POSE_LOST: a pose with a heading at least every pose_timeout (s); SENSOR_MISSING: IMU and wheels every
  // sensor_timeout (s). All of them have to be there within start_timeout (s) of the start.
  double pose_timeout = 0.5;
  double sensor_timeout = 0.5;
  double start_timeout = 3.0;
  // LEFT_LANE: the antenna farther than lane_limit (m) from its line, or the pivot of a turn moved that far.
  double lane_limit = 0.3;
  // NOT_TURNING: turn_check_after (s) into a turn, the gyro below turn_min_ratio of the commanded rate.
  double turn_check_after = 2.0;
  double turn_min_ratio = 0.7;
  // NOT_MOVING: move_check_after (s) into a line, the antenna slower than move_min_ratio of the commanded speed.
  double move_check_after = 3.0;
  double move_min_ratio = 0.5;
  // OVERSPEED: faster than overspeed_ratio times the highest command of the last overspeed_window (s) plus
  // overspeed_floor (m/s or rad/s), or a line faster than max_speed (m/s).
  double overspeed_ratio = 1.3;
  double overspeed_floor = 0.05;
  double overspeed_window = 1.5;
  double max_speed = 0.3;
  // GYRO_SIGN: wheels and gyro both turn faster than gyro_sign_min (rad/s), in opposite directions.
  double gyro_sign_min = 0.05;
  // IMU_GRAVITY: gravity in a stand more than gravity_tolerance (share) off gravity (m/s²), or more than
  // gravity_max_tilt (rad) off +z. SLOPE: the ground slopes more than max_slope (rad). The slope counts from the IMU's
  // mounting as stored (imu_roll, imu_pitch: what it reads on level ground, robot.geometry), with mount_allowance
  // (rad) more allowed while that mounting is not measured. Once a stand has a partner, a stand where the mower stood
  // within pair_distance (m) facing the other way (within pair_angle, rad), the slope is half the angle between
  // their gravities and the mounting does not count.
  double gravity = 9.80665;
  double gravity_tolerance = 0.05;
  double gravity_max_tilt = 30.0 * M_PI / 180.0;
  double max_slope = 5.0 * M_PI / 180.0;
  double imu_roll = 0.0;
  double imu_pitch = 0.0;
  double mount_allowance = 0.0;
  double pair_distance = 0.5;
  double pair_angle = 20.0 * M_PI / 180.0;
  // TIMEOUT: a phase longer than timeout_factor times its expected time plus timeout_extra (s).
  double timeout_factor = 2.0;
  double timeout_extra = 5.0;
  // Before a start: IMU samples and RTK fixed positions per s at least.
  double min_imu_rate = 20.0;
  double min_gnss_rate = 2.0;
};

// An IMU sample in base_link: rad/s and m/s² (gravity up, +z when level).
struct ImuSample {
  double gx = 0;
  double gy = 0;
  double gz = 0;
  double ax = 0;
  double ay = 0;
  double az = 0;
};

// A GNSS solution: the antenna in the map frame.
struct GnssSample {
  double x = 0;
  double y = 0;
  double accuracy = 0;
  bool fixed = false;
};

// The pose of base_link from the localization.
struct PoseSample {
  double x = 0;
  double y = 0;
  double heading = 0;
  bool heading_valid = false;
  double accuracy = 0;
};

struct Twist2 {
  double linear = 0;
  double angular = 0;
};

// Why a drive cannot start or why it ended, as in CALIBRATION_REFUSED and CALIBRATION_FAILED: a fixed name, an
// English message and its numbers. A drive that ran to its end has the reason DONE.
struct Problem {
  std::string reason;
  std::string message;
  std::vector<std::pair<std::string, double>> numbers;
};

// A phase began or ended, for the recording; round as in Program::round.
struct PhaseEdge {
  Phase phase = Phase::kStart;
  int round = 0;
  bool start = true;
  double t = 0;
};

// The calibration drive (Kalibrierfahrt) without ROS: ALIGN to the lane, a stand, then rounds of line, stand, turn
// and stand; it ends where it began, facing the lane. Commands go out every Tick (50 ms) while it moves and not at all
// while it stands; every motion ends with one zero. It checks itself all the time and stops with one zero and a reason
// at the first problem. Time is passed in (monotonic s).
class Program {
 public:
  explicit Program(const ProgramConfig& config);

  // The mounting of the IMU as stored, and the allowance while it is not measured (rad); for the next checks.
  void SetMounting(double roll, double pitch, double allowance);
  // The limits of the driver (m/s, rad/s; 0 or less: none): lines and turns of the next drive stay within them.
  void SetSpeedLimits(double max_linear, double max_angular);
  const ProgramConfig& config() const { return config_; }

  void OnImu(const ImuSample& sample, double now);
  void OnWheels(double linear, double angular, double now);
  void OnGnss(const GnssSample& sample, double now);
  void OnPose(const PoseSample& sample, double now);
  void OnEmergency(bool emergency, double now);

  // What keeps a drive from starting now from the sensors: SENSOR_MISSING, NO_RTK, GNSS_RATE, IMU_GRAVITY, SLOPE.
  std::vector<Problem> Readiness(double now) const;

  // Starts a drive on the lane from where base_link stands; false while one runs.
  bool Start(const Lane& lane, double now);
  // The command for now, none for silence.
  std::optional<Twist2> Tick(double now);
  // Ends a running drive with STOPPED: one zero.
  std::optional<Twist2> Stop(const std::string& message, double now);

  bool Running() const;
  Phase phase() const;
  // 0 before the first line (start, align, the first stand), then the round of the line, turn and stands, 1 to rounds.
  int round() const;
  // The share of the drive done, by the expected time of its phases (0 to 1).
  double Progress(double now) const;
  // How the last drive ended, none while it runs and before the first.
  const std::optional<Problem>& outcome() const;
  std::vector<PhaseEdge> TakeEdges();

 private:
  struct Step {
    Phase phase;
    int round;
    // s it should take, and for a stand how long it is.
    double expected;
  };
  // The mean gravity of a stand, where and which way the mower stood.
  struct StandGravity {
    double x;
    double y;
    double heading;
    std::array<double, 3> g;
  };
  struct Fix {
    double t;
    double x;
    double y;
    bool good;
  };
  // A check that has to fail for hold_time in a row.
  struct Hold {
    std::optional<double> since;
    bool Failing(bool now_failing, double now, double hold_time);
  };

  std::optional<Problem> Common(double now);
  std::optional<Problem> StandChecks(double now) const;
  std::optional<Problem> TurnChecks(double rate_cmd, double now);
  std::optional<Problem> LineChecks(double now);
  // Mean of the accelerations between since and now, none with too few samples.
  std::optional<std::array<double, 3>> MeanGravity(double since, double now) const;
  // IMU_GRAVITY, else SLOPE against the stored mounting, or against a stand facing the other way with paired.
  std::optional<Problem> Gravity(double since, double now, bool paired) const;
  std::optional<double> AntennaSpeed(double now) const;
  void Begin(size_t index, double now);
  std::optional<Twist2> Next(double now, bool moved);
  std::optional<Twist2> Abort(Problem problem, double now);
  std::optional<Twist2> TickAlign(double now);
  std::optional<Twist2> TickLine(double now);
  std::optional<Twist2> TickSpin(double now);
  double LineHeading() const;
  double CommandPeak(bool angular, double now) const;
  std::optional<Twist2> Record(const Twist2& twist, double now);
  void Edge(bool start, double now);

  ProgramConfig config_;
  // Samples, the latest and what the checks look back on.
  std::optional<double> imu_s_;
  ImuSample imu_;
  std::deque<double> imu_times_;
  std::deque<std::pair<double, ImuSample>> accel_;
  std::optional<double> wheels_s_;
  double wheel_angular_ = 0;
  std::deque<Fix> fixes_;
  std::optional<double> good_fix_s_;
  std::optional<double> pose_s_;
  PoseSample pose_;
  bool emergency_ = false;
  // The gyro's offset from the last stand, and the angle turned since the turn began.
  double gyro_bias_ = 0;
  double gyro_sum_ = 0;
  size_t gyro_count_ = 0;
  // Gravity of the stand that runs, from its settling on, and of the stands before.
  std::array<double, 3> stand_sum_{};
  size_t stand_count_ = 0;
  std::vector<StandGravity> stands_;
  double turned_ = 0;

  bool running_ = false;
  Lane lane_;
  std::vector<Step> steps_;
  size_t index_ = 0;
  double step_s_ = 0;
  double last_tick_s_ = 0;
  // Command of the step and the commands of the last overspeed_window.
  double cmd_ = 0;
  std::deque<std::pair<double, Twist2>> commands_;
  // A line: base_link at its start, the antenna's line through the first fix of it.
  double line_x_ = 0;
  double line_y_ = 0;
  std::optional<std::pair<double, double>> antenna_origin_;
  // A turn: antenna positions by angle turned, and the pivot from the first pair half a turn apart.
  std::vector<std::pair<double, std::pair<double, double>>> turn_fixes_;
  std::optional<std::pair<double, double>> pivot_;
  double pivot_drift_ = 0;
  double align_sign_ = 0;
  Hold sign_hold_;
  Hold slow_hold_;
  Hold fast_hold_;
  Hold lane_hold_;
  std::optional<Problem> outcome_;
  std::vector<PhaseEdge> edges_;
};

}  // namespace mowbite_calibration

#endif  // MOWBITE_CALIBRATION_PROGRAM_H
