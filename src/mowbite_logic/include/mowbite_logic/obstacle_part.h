#ifndef MOWBITE_LOGIC_OBSTACLE_PART_H
#define MOWBITE_LOGIC_OBSTACLE_PART_H

#include <cstdint>
#include <optional>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/reasons.h"

namespace mowbite_logic {

// One track of obstacles/status as the logic weighs it.
struct ObstacleTrackSample {
  uint32_t id = 0;
  // PERSON, ANIMAL, OBJECT or UNKNOWN.
  std::string kind;
  bool confirmed = false;
  // Its place lies in the view of a sensor with sight now.
  bool in_view = false;
  // Its edge from the nose of the mower along the heading (m).
  double ahead = 0.0;
  // ZONE_* of ObstacleTrack.msg: 0 none, 1 slow, 2 stop.
  int zone = 0;
  // Its middle in the map frame, its radius and 1 sigma of its place (m).
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
  double sigma = 0.0;
};

// obstacles/status: whether every sensor sees, and the tracks.
struct ObstacleSample {
  bool sight = true;
  std::vector<ObstacleTrackSample> tracks;
};

struct ObstacleConfig {
  // s the stop zone has to be seen free before the mower goes on; near_wait when a thing in it came nearer to the nose
  // than near_ahead (m), where the sensor sees only what stands higher.
  double free_wait = 10.0;
  double near_wait = 30.0;
  double near_ahead = 0.5;
  // An obstacles/status older than this (s) counts as one without sight: it starts no hold and frees none.
  double stale = 1.0;
  // obstacle_mode avoid: objects alone that held the mower avoid_wait s and moved no more than avoid_still (m) since
  // become circles kept out of: their radius, avoid_sigma times their spread and avoid_clearance between them and the
  // body. A circle goes once its track ends in view (seen free), after avoid_keep s or with the job.
  double avoid_wait = 3.0;
  double avoid_still = 0.2;
  double avoid_sigma = 2.0;
  double avoid_clearance = 0.30;
  double avoid_keep = 1800.0;
  // m/s at most on the way around them, with the blade on.
  double avoid_speed = 0.3;
};

// A circle the mower keeps out of (obstacle_mode avoid), in the map frame.
struct AvoidCircle {
  // The track it was made for (its entry in the list), and the one that lies in it now: 0 once that ended out of view.
  uint32_t track_id = 0;
  uint32_t follows = 0;
  double x = 0.0;
  double y = 0.0;
  // m, the thing with its spread and the clearance.
  double r = 0.0;
  // s, when it was made.
  double since = 0.0;
};

// The polygon around a circle for the planner and the follower: corners outside it, its edges touching it.
Ring KeepOutRing(const AvoidCircle& circle);

// What the mower did about a track, for its entry in vision.obstacles.list.
struct ObstacleNote {
  uint32_t track_id = 0;
  // slowed, would_stop, stopped, avoided.
  std::string action;
  // avoided: the circle (0 once it is gone), the m of the job left unmowed for it, the area.
  double keep_out_r = 0.0;
  double left_out = 0.0;
  std::string area_id;
  std::string area_name;
};

// The hold for obstacles (FP30 stop): with obstacle_mode stop on a platform proven for it, the mower stands while a
// confirmed track lies in the stop zone, and goes on once that zone is seen free for free_wait (near_wait after a thing
// right at the nose). A place out of view or without sight is never free; a status that stops coming starts nothing.
// obstacle_mode show only notes where the mower would have stopped. The core applies the hold to MOWING and to the map
// legs of DOCKING (logic_obstacle.cpp). With obstacle_mode avoid, objects alone that hold the mower and stand become
// circles to drive around (AvoidDue, Avoid); a confirmed object inside a circle holds the mower no more, a person or
// animal there still does.
class ObstaclePart {
 public:
  explicit ObstaclePart(const ObstacleConfig& config = {}) : config_(config) {}

  // obstacle_mode and the highest mode of the platform; below avoid the circles go.
  void SetMode(int setting, int mode_max);
  int mode() const;
  bool Armed() const {
    return mode() >= 2;
  }
  // A status of mowbite_obstacles; driving: the mower drives by itself now (a leg of the job or of docking in the map),
  // for the notes.
  void OnStatus(const ObstacleSample& sample, double now_s, bool driving);
  // A status older than stale frees nothing.
  void Tick(double now_s);
  // The mower is to stand for an obstacle.
  bool Holding() const {
    return hold_;
  }
  // A person or animal lies ahead in the zone to go slow.
  bool Slow() const {
    return Armed() && fresh_ && slow_;
  }
  // OBSTACLE with kind (the most of what held it: PERSON, ANIMAL, then OBJECT, UNKNOWN).
  Reason Why() const;
  // mowing.continue in a hold: the things that held it and lie out of view now count no more until a sensor sees them
  // again, and the hold ends. False, and nothing changes, while one of them is in view.
  bool Waive();
  std::vector<ObstacleNote> TakeNotes();

  // In a hold of objects alone that stood avoid_wait s and was not refused: the circles to keep out of around them;
  // else none.
  std::vector<AvoidCircle> AvoidDue(double now_s) const;
  // The circles are driven around: the hold ends, they count until they go.
  void Avoid(const std::vector<AvoidCircle>& circles);
  // No way around them: this hold waits like one of obstacle_mode stop.
  void AvoidRefused() {
    avoid_refused_ = true;
  }
  const std::vector<AvoidCircle>& circles() const {
    return circles_;
  }
  // Whether circles went since the last call.
  bool TakeCirclesGone() {
    return std::exchange(circles_gone_, false);
  }
  // The job ended: the circles go.
  void ClearCircles();

 private:
  // The tracks that hold the mower: confirmed in the stop zone, unless waived and out of view.
  std::vector<const ObstacleTrackSample*> Holders(const ObstacleSample& sample) const;
  void Note(uint32_t id, const std::string& action);
  // A confirmed object inside a circle, with avoid_still to spare.
  bool Inside(const ObstacleTrackSample& track, const AvoidCircle& circle) const;
  const AvoidCircle* CircleOf(const ObstacleTrackSample& track) const;
  void DropCircle(size_t index);
  void EndHold();

  ObstacleConfig config_;
  int setting_ = 0;
  int mode_max_ = 0;
  ObstacleSample last_;
  std::optional<double> last_at_;
  bool fresh_ = false;
  bool slow_ = false;
  bool hold_ = false;
  // The worst kind that held it, and whether a thing came within near_ahead, in this hold.
  std::string kind_;
  bool near_ = false;
  // Since when the zone is seen free in this hold.
  std::optional<double> free_since_;
  std::set<uint32_t> waived_;
  // Where the things of this hold stand and since when, whether driving around them was refused.
  struct Standing {
    double x = 0.0;
    double y = 0.0;
    double since = 0.0;
  };
  std::map<uint32_t, Standing> standing_;
  bool avoid_refused_ = false;
  std::vector<AvoidCircle> circles_;
  bool circles_gone_ = false;
  // Tracks already noted, by action.
  std::set<std::pair<uint32_t, std::string>> noted_;
  std::vector<ObstacleNote> notes_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_OBSTACLE_PART_H
