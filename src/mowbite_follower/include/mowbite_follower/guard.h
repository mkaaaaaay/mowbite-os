#ifndef MOWBITE_FOLLOWER_GUARD_H
#define MOWBITE_FOLLOWER_GUARD_H

#include <string>
#include <vector>

#include "mowbite_follower/path.h"

namespace mowbite_follower {

struct Point2 {
  double x = 0;
  double y = 0;
};

// Open or closed, either way.
using Ring = std::vector<Point2>;

bool InsideRing(const Ring& ring, double x, double y);
// m from the point to the edge of the ring.
double DistanceToRing(const Ring& ring, double x, double y);

// Where base_link may be: in one of the allowed areas or at most margin (m) outside of all of them, never in an
// obstacle.
class Geofence {
 public:
  Geofence() = default;
  Geofence(std::vector<Ring> allowed, std::vector<Ring> obstacles, double margin);

  // Empty if the point may be there, else why not.
  std::string Check(double x, double y) const;
  // Why the areas cannot be a geofence, empty if they can.
  std::string Problem() const;

 private:
  std::vector<Ring> allowed_;
  std::vector<Ring> obstacles_;
  double margin_ = 0;
};

// Why the follower stops before the path is done. The numbers are the reasons of FollowPath.action.
enum class StopReason {
  kNone = 0,
  kCanceled = 1,
  kInvalid = 2,
  kNotAllowed = 3,
  kEmergency = 4,
  kPoseLost = 5,
  kPositionAccuracy = 6,
  kHeading = 7,
  kCrossTrack = 8,
  kOutside = 9,
  kTimeout = 10,
  kTurnSlide = 11,
};

struct GuardConfig {
  // s without a pose of the localization until the follower stops.
  double pose_timeout = 0.5;
  // s without hw/status.
  double hw_timeout = 1.0;
  // m, the shared max_position_accuracy.
  double max_position_accuracy = 0.1;
  // m from the path.
  double max_cross_track = 0.5;
  // m base_link may get from the place a turn on the spot began at; 0: no such limit.
  double max_turn_slide = 0;
};

// What the follower knows when it checks, ages in s.
struct GuardInput {
  // hw/capabilities has a safety measurement of the platform.
  bool safety_measured = false;
  // The driver's limits on hw/capabilities let the wheels move (both above 0).
  bool driver_moves = false;
  double hw_age = 1e9;
  bool emergency = true;
  double pose_age = 1e9;
  Pose2 pose;
  double position_accuracy = 999;
  bool heading_valid = false;
  double cross_track = 0;
  // Per path, from the goal (FollowPath.action): m base_link is from the nearest point of the path, and how far it
  // may be; 0: no such limit.
  double off_path = 0;
  double max_off_path = 0;
  // m base_link is from the place the turn on the spot going on began at, 0 outside of such a turn.
  double turn_slide = 0;
};

struct GuardResult {
  StopReason reason = StopReason::kNone;
  std::string message;
};

// The checks before every command, in order of importance: the first that fails is the reason.
GuardResult CheckGuard(const GuardConfig& config, const GuardInput& in, const Geofence& fence);

// Whether a turn on the spot may start: the last fix is at most limit s old now, its age at the pose plus the age of
// the pose. A limit <= 0 is no such rule.
bool TurnFixFresh(double fix_age, double pose_age, double limit);

}  // namespace mowbite_follower

#endif  // MOWBITE_FOLLOWER_GUARD_H
