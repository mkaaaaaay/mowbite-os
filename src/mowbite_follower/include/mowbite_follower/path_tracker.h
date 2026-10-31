// Based on ftc_local_planner (FTCPlanner) from open_mower_ros, BSD License.
#ifndef MOWBITE_FOLLOWER_PATH_TRACKER_H
#define MOWBITE_FOLLOWER_PATH_TRACKER_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mowbite_follower {

// base_link in the map frame; yaw in rad, counter-clockwise from +x.
struct Pose2 {
  double x = 0;
  double y = 0;
  double yaw = 0;
};

// The segment types of the planner (FollowPath.action).
enum class Kind : uint8_t { kPerimeter = 0, kLane = 1, kTurn = 2, kTransit = 3 };

// Angle in (-pi, pi].
double NormalizeAngle(double a);

struct TrackerConfig {
  // m/s on a stretch of at least fast_distance (m) whose poses face within fast_angle (rad) of each other, on
  // anything else, and on turns. The control point speeds up and slows down by acceleration (m/s²).
  double speed_fast = 0.3;
  double speed_slow = 0.15;
  double speed_turn = 0.1;
  double fast_distance = 1.0;
  double fast_angle = 20.0 * M_PI / 180.0;
  double acceleration = 0.1;
  // rad/s the control point turns at most: below max_angular, so that the mower has some left to correct with on the
  // tightest arcs.
  double turn_rate = 0.25;
  // Limits of the command, the driver's limits on the Tron.
  double max_linear = 0.3;
  double max_angular = 0.3;
  // PID on the errors of the control point seen from base_link: ahead (lon), left (lat) and its heading (ang).
  double kp_lon = 4.0;
  double ki_lon = 0.0;
  double kd_lon = 0.0;
  double kp_lat = 25.0;
  double ki_lat = 0.06;
  double kd_lat = 1.23;
  double kp_ang = 3.0;
  double ki_ang = 0.0;
  double kd_ang = 0.5;
  double ki_lon_max = 10.0;
  double ki_lat_max = 62.0;
  double ki_ang_max = 62.0;
  // Within this lateral error (m) the heading counts fully, further out down to lateral_priority_min_gain of it.
  double lateral_priority_distance = 0.1;
  double lateral_priority_min_gain = 0.1;
  // Turning before the start and after the goal ends within this heading error (rad).
  double max_goal_angle_error = 20.0 * M_PI / 180.0;
  // The goal counts as reached this close (m).
  double max_goal_distance_error = 0.1;
  // A pose's heading is the one the mower leaves it with. A pose whose heading differs from the one before by more than
  // this (rad) is a turn on the spot, the planner's rule (its arcs turn at most 10 deg per pose): the mower arrives
  // with the heading before, stops on it and turns the shorter way.
  double spin_angle = 15.0 * M_PI / 180.0;
  // Forwards only while the heading is within this (rad) of the control point's; beyond it the mower turns first.
  double max_drive_angle_error = 45.0 * M_PI / 180.0;
  // s for turning before the start, approaching the goal or a corner and turning there.
  double phase_timeout = 20.0;
  // m between base_link and the control point; further means the mower stays behind (stuck).
  double max_follow_distance = 1.0;
  // Standing still while driven: base_link got no further than stall_distance (m) from where it stood for stall_time
  // s of commands of at least stall_speed (m/s), e.g. wheels that cannot climb out of a hollow. 0: no such check.
  double stall_time = 0;
  double stall_distance = 0.05;
  double stall_speed = 0.03;
  // A straight line driven without steering: no turning on the spot, angular always 0, done once base_link got along
  // the line to within max_goal_distance_error of its end.
  bool straight = false;
};

enum class Phase {
  kIdle,
  kPreRotate,
  kFollowing,
  kCornerApproach,
  kCornerRotate,
  kGoalApproach,
  kPostRotate,
  kDone,
  kFailed
};

const char* PhaseName(Phase phase);

// Follow the carrot: a control point runs along the path at the speed the path allows, the mower follows it with a
// PID on its errors. Forwards only. Not thread safe.
class PathTracker {
 public:
  explicit PathTracker(const TrackerConfig& config) : config_(config) {}

  // Empty if the path can be followed from start_index, else why not. kinds: one per pose or none (all transit).
  std::string Start(const std::vector<Pose2>& poses, const std::vector<Kind>& kinds, size_t start_index);
  // Ends following at once; Step then returns zero.
  void Cancel();

  struct Command {
    double linear = 0;
    double angular = 0;
  };
  // One control step with the pose of base_link, dt s after the last one.
  Command Step(const Pose2& pose, double dt);

  Phase phase() const {
    return phase_;
  }
  // Why the phase is kFailed.
  const std::string& failure() const {
    return failure_;
  }
  // On the path: following it or turning on one of its corners.
  bool OnPath() const {
    return phase_ == Phase::kFollowing || phase_ == Phase::kCornerApproach || phase_ == Phase::kCornerRotate;
  }
  // The pose the control point has passed.
  size_t index() const {
    return index_;
  }
  // m from the path to base_link, left positive, over the segments near the control point.
  double CrossTrack(const Pose2& pose) const;
  Pose2 ControlPoint() const {
    return control_;
  }

 private:
  void AdvanceControlPoint(double dt);
  // Whether the mower turns on the spot at poses_[i].
  bool SpinAt(size_t i) const;
  // The heading the mower arrives at poses_[i] with.
  double ArrivalYaw(size_t i) const;
  // m the poses from the control point on stay straight, up to fast_distance.
  double StraightAhead() const;
  // Fails the path when the mower stands still while linear asks it to move (stall_time).
  void CheckStall(const Pose2& pose, double linear, double dt);
  void SetPhase(Phase phase);
  void Fail(const std::string& why);

  TrackerConfig config_;
  std::vector<Pose2> poses_;
  std::vector<Kind> kinds_;
  Phase phase_ = Phase::kIdle;
  std::string failure_;
  double phase_time_ = 0;
  size_t index_ = 0;
  // 0..1 from poses_[index_] to poses_[index_ + 1]
  double progress_ = 0;
  double speed_ = 0;
  Pose2 control_;
  double lon_error_ = 0;
  double lat_error_ = 0;
  double angle_error_ = 0;
  double last_lon_error_ = 0;
  double last_lat_error_ = 0;
  double last_angle_error_ = 0;
  double i_lon_ = 0;
  double i_lat_ = 0;
  double i_angle_ = 0;
  bool first_step_ = true;
  // The turn on the spot at this pose is done already; SIZE_MAX: none.
  size_t corner_done_ = SIZE_MAX;
  // Where the mower stood when the commands to move began, and for how long (s) it has not got away from there.
  bool stall_anchored_ = false;
  Pose2 stall_anchor_;
  double stall_s_ = 0;
};

}  // namespace mowbite_follower

#endif  // MOWBITE_FOLLOWER_PATH_TRACKER_H
