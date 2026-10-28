#ifndef MOWBITE_LOGIC_AREA_RECORDER_H
#define MOWBITE_LOGIC_AREA_RECORDER_H

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_logic/reasons.h"

namespace mowbite_logic {

struct Event {
  std::string type;
  json details;
};

struct Point2 {
  double x = 0;
  double y = 0;
};

// Open: the last point does not repeat the first.
using Ring = std::vector<Point2>;

// Why the ring cannot become an outline, empty if it can: RING_FEW_POINTS, RING_CROSSES_ITSELF, RING_TOO_SMALL.
Reason RingProblem(const Ring& ring, double min_area_m2);
// Whether two segments that are not neighbours on the closed ring touch or cross.
bool SelfIntersects(const Ring& ring);
// m², of the closed ring.
double RingArea(const Ring& ring);

struct RecorderConfig {
  // A new point needs more than this distance (m) to the last one, NEW_POINT_MIN_DISTANCE of OpenMower.
  double min_point_distance = 0.1;
  // Points only from a position at least this accurate (m), max_position_accuracy of OpenMower.
  double max_position_accuracy = 0.1;
  // A ring must enclose at least this (m²), so that points on a line are no area.
  double min_area = 0.05;
  // All rings of one recording together; the map must fit into one MQTT message.
  size_t max_points = 5000;
  // The two points of the docking station must be this far apart (m) for a usable heading.
  double min_dock_distance = 0.3;
};

// A change of the map the recording asks mowbite_map for.
struct MapEdit {
  enum class Kind { kArea, kDock };
  Kind kind = Kind::kArea;
  // kArea: mow or nav
  std::string type;
  Ring outline;
  std::vector<Ring> obstacles;
  // kDock: on the contacts, map frame; heading in rad, the way the mower drives in
  double x = 0;
  double y = 0;
  double heading = 0;
};

// What AREA_RECORDING records: one outline, the obstacles inside it and the docking station, like OpenMower. Without
// ROS; the logic passes poses and actions, sends the edits to mowbite_map and returns their results.
class AreaRecorder {
 public:
  enum class Result { kDone, kRefused, kUnknown };

  explicit AreaRecorder(const RecorderConfig& config);

  // Drops everything recorded and not saved; an edit on its way is still reported when it is answered.
  void Reset();
  // An action of mower_logic:area_recording without the prefix, except exit_recording_mode.
  Result OnAction(const std::string& name);
  void OnPose(double x, double y, float position_accuracy);
  // The pose got too old.
  void OnPoseLost();
  // Whether mowbite_map takes changes now, and why not.
  void SetMapWritable(bool writable, const Reason& reason);

  // The edit to send, once.
  std::optional<MapEdit> TakeMapEdit();
  // The answer of mowbite_map to the edit sent last; code says why it was not saved (SAVE_FAILED).
  void OnMapEditResult(bool ok, const std::string& id, const std::string& code, const std::string& message);
  bool EditPending() const {
    return pending_.has_value();
  }

  // RECORD_OUTLINE, RECORD_OBSTACLE, RECORD_DOCKING_POSITION or empty, as OpenMower names them.
  std::string SubState() const;
  // Enabled flags in the order of ActionNames().
  std::vector<bool> Enabled() const;
  static const std::vector<std::string>& ActionNames();

  // Events since the last call, without a position.
  std::vector<Event> TakeEvents();
  // Lines for the log since the last call.
  std::vector<std::string> TakeNotices();
  bool TakeActionsChanged();

  // map_overlay/json like OpenMower: the outline green, obstacles red, the open ring blue. Each line also carries
  // polygon.points and closed, which MowBite up to 1.4.0-beta.16 reads instead of poly and is_closed.
  json Overlay() const;
  // True once after the lines changed.
  bool TakeOverlayChanged();

  const std::optional<Ring>& outline() const {
    return outline_;
  }
  const std::vector<Ring>& obstacles() const {
    return obstacles_;
  }
  // The ring being recorded, empty when none is.
  const Ring& current() const {
    return current_;
  }
  bool recording() const {
    return recording_;
  }

 private:
  Result Refuse(const std::string& action, const Reason& reason);
  // Ends the open ring: the outline if there is none yet, else an obstacle.
  bool CloseRing(const std::string& action);
  Result Finish(const std::string& action, const std::string& type);
  Result RecordDock();
  void AddPoint(double x, double y);
  size_t PointCount() const;

  RecorderConfig config_;
  bool recording_ = false;
  Ring current_;
  std::optional<Ring> outline_;
  std::vector<Ring> obstacles_;
  // Like OpenMower: points come by themselves at min_point_distance.
  bool auto_points_ = true;
  bool collect_requested_ = false;
  // The point in front of the docking station, the first of two.
  std::optional<Point2> dock_first_;
  struct Pose {
    double x;
    double y;
    float accuracy;
  };
  std::optional<Pose> pose_;
  bool accuracy_blocked_ = false;
  bool full_reported_ = false;
  bool map_writable_ = false;
  Reason map_reason_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");
  // The edit waiting for its answer, and the one not yet taken by the node.
  std::optional<MapEdit> pending_;
  bool edit_untaken_ = false;
  std::vector<Event> events_;
  std::vector<std::string> notices_;
  bool actions_changed_ = true;
  bool overlay_changed_ = true;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_AREA_RECORDER_H
