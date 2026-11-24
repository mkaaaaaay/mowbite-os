#ifndef MOWBITE_FOLLOWER_PATH_FOLLOWER_H
#define MOWBITE_FOLLOWER_PATH_FOLLOWER_H

#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_follower/path.h"
#include "mowbite_follower/speed_plan.h"
#include "mowbite_follower/steering.h"

namespace mowbite_follower {

struct FollowerConfig {
  SpeedLimits speed;
  SteeringConfig steering;
  // rad/s at most for every command.
  double max_angular = 0.3;
  // rad/s at least when turning on the spot, above the dead band of the wheels, and within how many rad of the
  // target such a turn ends.
  double min_turn_rate = 0.28;
  double turn_tolerance = 2.0 * M_PI / 180.0;
  // rad/s at most when turning on the spot, 0 for max_angular.
  double spot_max_angular = 0;
  // rad/s² a turn on the spot slows down with before its target, from sqrt(2 * turn_deceleration * error); 0 for
  // heading_gain * error.
  double turn_deceleration = 0;
  // A turn on the spot that the heading follows slower than commanded (slipping wheels) gets turn_feedback times the
  // missing rate on top, at most turn_feedback_max rad/s and never above the limit of such turns; 0: none.
  double turn_feedback = 0;
  double turn_feedback_max = 0;
  // A turn on the spot holds the place it began at, since wheels that slip unevenly move base_link along its
  // heading: the mower drives turn_hold_gain (1/s) times what base_link is off that place along its heading beyond
  // turn_hold_deadband m, plus the sum of that offset over the turn at turn_hold_gain squared / 4, as far as the
  // slower wheel keeps wheel_min_speed m/s on a track of wheel_distance m (hw/capabilities). Gain or track 0: no
  // such drive.
  double turn_hold_gain = 0;
  double turn_hold_deadband = 0.02;
  double wheel_min_speed = 0;
  double wheel_distance = 0;
  // m from the start pose at most when the path begins.
  double max_start_distance = 1.0;
  // m from the last pose at most when the mower got there.
  double goal_tolerance = 0.1;
  // rad the direction of travel turns by at a corner, which the mower takes by turning on the spot.
  double corner_angle = 15.0 * M_PI / 180.0;
  // m: arcs tighter than this are taken as corners too, since the inner wheel would stand in its dead band.
  double min_arc_radius = 0.3;
  // rad off the course at most to drive; further off it turns on the spot first.
  double max_drive_angle = 45.0 * M_PI / 180.0;
  // s a turn on the spot may take.
  double phase_timeout = 20.0;
  // s without 5 cm of progress along the path while it drives, when stall_time is 0.
  double progress_timeout = 10.0;

  // Per path, from the goal (FollowPath.action).
  double stall_time = 0;
  bool straight = false;
  bool reverse = false;
  double end_tolerance = 0;
};

// The speeds the follower drives at most under the driver's limits (hw/capabilities): what the planner plans with.
struct EffectiveSpeeds {
  double speed_fast = 0;
  double speed_slow = 0;
  double max_angular = 0;
};

// The follower's own values, each under max_linear or max_angular and under the driver's limit; a limit of the driver
// that is no number above 0 counts as none.
EffectiveSpeeds Effective(const FollowerConfig& config, double driver_max_linear, double driver_max_angular);
// The message for bridge/params: {"node":"<node>","params":{"speed_fast":…,"speed_slow":…,"max_angular":…}}.
std::string ParamsJson(const std::string& node, const EffectiveSpeeds& speeds);

enum class Phase {
  kIdle,
  kStartTurn,   // on the spot towards the path before driving
  kFollowing,
  kCornerTurn,  // on the spot at a corner, to the direction after it
  kRealign,     // on the spot after the heading got too far off the course
  kGoalTurn,    // on the spot at the end, to the heading of the last pose
  kDone,
  kFailed,
};

enum class Failure {
  kNone,
  kStart,    // the path cannot begin where the mower is
  kTimeout,  // stuck, no progress or a turn took too long
  kTurnRoom,  // a turn on the spot would take the body out of its room
};

// What a turn on the spot did, by the commands, the wheels and the pose.
struct TurnRecord {
  Phase phase = Phase::kIdle;
  double seconds = 0;
  // rad commanded, reported by the wheels (NaN without their odometry) and turned by the heading.
  double commanded = 0;
  double wheels = 0;
  double turned = 0;
  // m base_link got from the place the turn began at, at most and at its end, and m driven to hold that place.
  double slide = 0;
  double end_slide = 0;
  double hold = 0;
};

// Drives a mower along a path of base_link poses: turns on the spot at the start, at corners and at the end, steers
// along the path in between at the speed of its speed plan. Without ROS; the follower node calls Step with every
// pose and sends what it returns.
class PathFollower {
 public:
  struct Command {
    double linear = 0;
    double angular = 0;
  };

  // Empty if the body may turn on the spot at the place of at from its heading by delta rad, else why not.
  using TurnRoom = std::function<std::string(const Pose2& at, double delta)>;

  explicit PathFollower(const FollowerConfig& config) : config_(config) {}

  // Asked once at the start of every turn on the spot but those at corners of the path; without it no such check.
  void SetTurnRoom(TurnRoom turn_room) { turn_room_ = std::move(turn_room); }

  // Empty if the path can be followed from start_index, else why not.
  std::string Start(const std::vector<Pose2>& poses, const std::vector<Kind>& kinds, size_t start_index);
  // The command for the pose of base_link, dt s after the last step; wheel_turn is the rad the wheels turned the
  // base by since then, by their odometry, NaN without. Without a fresh fix behind the pose the mower does not turn
  // on the spot: wheels that slip there move it unseen. It stands until the fix is back, at most phase_timeout, which
  // counts neither for the turn nor as a lack of progress.
  Command Step(const Pose2& pose, double dt, double wheel_turn = std::numeric_limits<double>::quiet_NaN(),
               bool fresh_fix = true);
  void Cancel();
  // The mower stood for the gate of the logic: it starts again from the floor of the speed plan, with its ramp.
  void Halted() { speed_ = 0; }

  Phase phase() const { return phase_; }
  Failure failure_kind() const { return failure_kind_; }
  const std::string& failure() const { return failure_; }
  // The pose the mower got to: a path started from here goes on where this one stopped.
  size_t index() const;
  // The mower came onto the path (a straight one from its start) and has not left it by finishing or failing.
  bool OnPath() const;
  // m beside the path, left positive.
  double CrossTrack(const Pose2& pose) const;
  // m to the nearest point of the whole path, its ends included.
  double OffPath(const Pose2& pose) const;
  // m the pose is from the place the turn on the spot going on began at, 0 outside of such a turn.
  double TurnSlide(const Pose2& pose) const;
  // The turns on the spot ended since the path began, and the last of them.
  size_t turns() const { return turns_; }
  const TurnRecord& last_turn() const { return last_turn_; }

 private:
  Projection Foot(const Pose2& pose) const;
  Pose2 Travel(const Pose2& pose) const;
  Command Fail(Failure kind, const std::string& message);
  // Turns on the spot towards target (rad, for the heading of travel); empty once within turn_tolerance.
  std::optional<Command> TurnTowards(double heading, double target);
  // Takes what the mower did since the last step of the turn on the spot going on.
  void TrackTurn(double heading);
  // Ends the record of the turn going on.
  void EndTurn();
  // rad/s to add to a turn towards sign: what the heading stayed behind the commands so far.
  double TurnBoost(double sign) const;
  // m/s to hold the place of the turn while it turns at rate.
  double HoldSpeed(double rate);
  void Enter(Phase phase);
  Command Follow(const Pose2& pose, const Pose2& travel, double dt);
  Command FromTravel(double speed, double angular) const;
  // Fails the path if the mower stands while driven or makes no progress along it.
  bool CheckMoving(const Pose2& pose, const Projection& foot);

  FollowerConfig config_;
  Path path_;
  SpeedPlan plan_;
  Phase phase_ = Phase::kIdle;
  Failure failure_kind_ = Failure::kNone;
  std::string failure_;
  bool begun_ = false;
  bool joined_ = false;
  size_t segment_ = 0;
  size_t stop_ = 0;
  // m along the path the foot point got to.
  double along_ = 0;
  double time_ = 0;
  double phase_start_ = 0;
  // s a turn on the spot has waited for a fresh fix.
  double fix_wait_ = 0;
  // The turn on the spot going on was asked about its room.
  bool room_checked_ = false;
  TurnRoom turn_room_;
  double speed_ = 0;
  double step_dt_ = 0;
  // Of this step: the pose, the turn of the wheels and a fresh fix behind the pose.
  Pose2 at_;
  double wheel_turn_ = 0;
  bool fresh_fix_ = true;
  // Of the turn on the spot going on: the place it began at, the heading of the last step, its command, the rad/s
  // the turn stayed behind the commands, smoothed, what holds its place, and its record.
  std::optional<Pose2> pivot_;
  std::optional<double> turn_heading_;
  Command turn_command_;
  double turn_missing_ = 0;
  double hold_sum_ = 0;
  TurnRecord turn_;
  TurnRecord last_turn_;
  size_t turns_ = 0;
  std::optional<Pose2> still_at_;
  double still_since_ = 0;
  double progress_along_ = 0;
  double progress_since_ = 0;
};

}  // namespace mowbite_follower

#endif  // MOWBITE_FOLLOWER_PATH_FOLLOWER_H
