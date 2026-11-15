#ifndef MOWBITE_FOLLOWER_PATH_FOLLOWER_H
#define MOWBITE_FOLLOWER_PATH_FOLLOWER_H

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
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

  explicit PathFollower(const FollowerConfig& config) : config_(config) {}

  // Empty if the path can be followed from start_index, else why not.
  std::string Start(const std::vector<Pose2>& poses, const std::vector<Kind>& kinds, size_t start_index);
  // The command for the pose of base_link, dt s after the last step.
  Command Step(const Pose2& pose, double dt);
  void Cancel();

  Phase phase() const { return phase_; }
  Failure failure_kind() const { return failure_kind_; }
  const std::string& failure() const { return failure_; }
  // The pose the mower got to: a path started from here goes on where this one stopped.
  size_t index() const;
  // The mower came onto the path and has not left it by finishing or failing.
  bool OnPath() const;
  // m beside the path, left positive.
  double CrossTrack(const Pose2& pose) const;
  // m to the nearest point of the whole path, its ends included.
  double OffPath(const Pose2& pose) const;

 private:
  Projection Foot(const Pose2& pose) const;
  Pose2 Travel(const Pose2& pose) const;
  Command Fail(Failure kind, const std::string& message);
  // Turns on the spot towards target (rad, for the heading of travel); empty once within turn_tolerance.
  std::optional<Command> TurnTowards(double heading, double target);
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
  double speed_ = 0;
  std::optional<Pose2> still_at_;
  double still_since_ = 0;
  double progress_along_ = 0;
  double progress_since_ = 0;
};

}  // namespace mowbite_follower

#endif  // MOWBITE_FOLLOWER_PATH_FOLLOWER_H
