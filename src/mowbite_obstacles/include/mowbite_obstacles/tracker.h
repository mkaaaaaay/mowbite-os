// The obstacles around the mower, without ROS: finds of the sensors put into the map with the pose of their frame,
// tracks over frames, confirmation, the view and sight of each sensor, and the zones ahead of the mower. Times are s
// of one clock (the stamps of the frames and of the poses); nothing here reads a clock.
#ifndef MOWBITE_OBSTACLES_TRACKER_H
#define MOWBITE_OBSTACLES_TRACKER_H

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace mowbite_obstacles {

enum class Kind { kPerson, kAnimal, kObject, kUnknown };
// PERSON, ANIMAL, OBJECT, UNKNOWN as in Obstacle.msg; false for another name.
bool ParseKind(const std::string& name, Kind* kind);
const char* KindName(Kind kind);
bool Living(Kind kind);

// One find of a frame, in base_link, as Obstacle.msg has it.
struct Find {
  Kind kind = Kind::kObject;
  std::string class_name;
  double score = 0.0;
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
  double height = 0.0;
  double cov_xx = 0.0;
  double cov_xy = 0.0;
  double cov_yy = 0.0;
  double box_x = 0.0;
  double box_y = 0.0;
  double box_w = 0.0;
  double box_h = 0.0;
  // The box touches the lower edge of the image: the footprint reaches nearer than x, y.
  bool cut_bottom = false;
};

// Where a sensor sees the ground, as Obstacles.msg has it.
struct ViewSector {
  double x = 0.0;
  double y = 0.0;
  double angle_left = 0.0;
  double angle_right = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
};
// Whether a disc in base_link lies wholly inside the sector.
bool Inside(const ViewSector& view, double x, double y, double radius);

struct Frame {
  std::string source;
  double stamp = 0.0;
  bool sight = false;
  std::string reason;
  ViewSector view;
  std::vector<Find> finds;
};

struct Pose {
  double stamp = 0.0;
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  // Expected error of x and y (m) and of the heading (rad).
  double accuracy = 0.0;
  double heading_accuracy = 0.0;
};

// The body of the mower around base_link (m): its nose ahead, its back behind, half its width.
struct Body {
  double front = 0.0;
  double rear = 0.0;
  double half_width = 0.0;
};

struct TrackerConfig {
  // A find counts from this score on.
  double min_score_person = 0.5;
  double min_score_animal = 0.5;
  double min_score_object = 0.5;
  // Confirmed with confirm_hits finds among the last confirm_frames frames that had its place in view.
  int confirm_hits = 2;
  int confirm_frames = 3;
  // A find belongs to a track within match_distance plus twice their spread, at most match_max (m).
  double match_distance = 0.4;
  double match_max = 1.5;
  // How fast a thing of this kind may move (m/s); its track widens by that between finds.
  double living_speed = 1.5;
  double object_speed = 0.05;
  // A confirmed track ends once its place is in view this long without a find (s), or once nothing found it this long
  // while it lies in no stop zone: a place out of view is never free.
  double free_seen = 1.0;
  double track_timeout = 30.0;
  // A miss counts only where the track lies inside the view with view_margin (m) and twice its spread to spare: a thing
  // the edge of the view cuts is not seen free.
  double view_margin = 0.05;
  // An unconfirmed track ends after this long without a find (s).
  double candidate_timeout = 2.0;
  // A frame needs a pose within this of its stamp (s); poses kept this long.
  double pose_gap = 0.25;
  double pose_keep = 3.0;
  // A sensor without a frame this long has no sight, nor any without a pose this long (s).
  double stale = 1.0;
  // Zones from the nose (m): stop for living things ahead within stop_living_ahead in a corridor of the body plus
  // stop_living_margin on each side, or anywhere within stop_living_around of the body; for objects within
  // stop_object_ahead in a corridor of the body plus stop_object_margin; slow for living things within slow_ahead in
  // their corridor. A track counts zone_sigma times its spread nearer.
  double stop_living_ahead = 1.0;
  double stop_living_margin = 0.5;
  double stop_living_around = 1.0;
  double stop_object_ahead = 0.8;
  double stop_object_margin = 0.15;
  double slow_ahead = 2.0;
  double zone_sigma = 1.0;
  // A track that stops the mower leaves the stop zone only this much (m) beyond its edges: a thing on the edge does not
  // let the mower go and stop again as the pose wanders by a few cm.
  double zone_release = 0.2;
  // Tracks of UNKNOWN (geometry only) stop as objects; off until false stops in the grass are measured.
  bool unknown_stops = false;
  // Finds with a box a track keeps for its photo.
  size_t shots = 10;
};

// A find with its box in the image of its frame.
struct Shot {
  std::string source;
  double stamp = 0.0;
  double box_x = 0.0;
  double box_y = 0.0;
  double box_w = 0.0;
  double box_h = 0.0;
};

struct Track {
  uint32_t id = 0;
  Kind kind = Kind::kUnknown;
  std::string class_name;
  double score = 0.0;
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
  double height = 0.0;
  double cov_xx = 0.0;
  double cov_xy = 0.0;
  double cov_yy = 0.0;
  bool confirmed = false;
  bool in_view = false;
  // The sensor whose last frame had its place in view; a frame of another one that does not see it changes nothing.
  std::string viewed_by;
  double first_seen = 0.0;
  double last_seen = 0.0;
  // Since when its place has been in view without a find; negative while not.
  double free_since = -1.0;
  // It lay in the stop zone at the last Tick.
  bool stopping = false;
  // Finds (true) and misses over the last frames that had its place in view, newest last.
  std::deque<bool> window;
  // The sensor of its last find, and its last finds with a box, newest last, for a photo.
  std::string source;
  std::deque<Shot> shots;
};

// What changed with a frame: tracks newly confirmed (their ids) and frames left out.
struct FrameResult {
  bool used = false;
  // No pose near the stamp of the frame.
  bool no_pose = false;
  std::vector<uint32_t> confirmed;
  std::vector<uint32_t> ended;
};

enum class Zone { kNone, kSlow, kStop };

struct TrackView {
  const Track* track = nullptr;
  // The zone of a confirmed track; kNone for a candidate.
  Zone zone = Zone::kNone;
  // Edge of the track from the nose along the heading (m, negative behind it), and from the middle line (left +).
  double ahead = 0.0;
  double aside = 0.0;
  // 1 sigma along its larger axis (m).
  double sigma = 0.0;
};

struct Status {
  bool sight = true;
  std::string reason;
  std::string source;
  bool stop = false;
  uint32_t stop_id = 0;
  Kind stop_kind = Kind::kUnknown;
  bool slow = false;
  std::vector<TrackView> tracks;
};

class Tracker {
 public:
  explicit Tracker(TrackerConfig config = {});

  // The sensors the platform reports; others are ignored, and a sensor of these without frames has no sight.
  void SetSources(const std::vector<std::string>& sources);
  const std::vector<std::string>& sources() const { return sources_; }
  // Until the body is known the zones have no edge to count from: the sensors count as not set up.
  void SetBody(const Body& body) {
    body_ = body;
    body_known_ = true;
  }
  // A pose with a value that is not finite is left out.
  void OnPose(const Pose& pose);
  // now: when the frame arrived, for the sight of its sensor.
  FrameResult OnFrame(const Frame& frame, double now);
  // Ends tracks that ran out of time and keeps which lie in the stop zone; returns the ids of those that ended.
  std::vector<uint32_t> Tick(double now);
  // The sight of the sensors, the tracks and the zones at the latest pose; a frame without a pose near its stamp
  // leaves its sensor without sight (NO_SIGNAL) until the next one.
  Status StatusAt(double now) const;
  const std::map<uint32_t, Track>& tracks() const { return tracks_; }
  const Track* TrackOf(uint32_t id) const;
  void Clear() { tracks_.clear(); }

 private:
  struct Sensor {
    bool heard = false;
    double last = 0.0;
    bool sight = false;
    std::string reason;
  };

  bool PoseAt(double stamp, Pose* pose) const;
  // The track as seen from the pose, with its zone.
  TrackView ViewFrom(const Track& track, const Pose& pose) const;
  double MinScore(Kind kind) const;
  void Update(Track& track, const Find& find, const Frame& frame, double fx, double fy, double cxx, double cxy,
              double cyy);
  void Shoot(Track& track, const Find& find, const Frame& frame) const;

  TrackerConfig config_;
  Body body_;
  bool body_known_ = false;
  std::vector<std::string> sources_;
  std::map<std::string, Sensor> sensors_;
  std::deque<Pose> poses_;
  std::map<uint32_t, Track> tracks_;
  uint32_t next_id_ = 1;
};

}  // namespace mowbite_obstacles

#endif  // MOWBITE_OBSTACLES_TRACKER_H
