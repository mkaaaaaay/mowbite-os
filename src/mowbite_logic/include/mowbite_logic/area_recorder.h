#ifndef MOWBITE_LOGIC_AREA_RECORDER_H
#define MOWBITE_LOGIC_AREA_RECORDER_H

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_logic/reasons.h"

namespace mowbite_logic {

struct Event {
  std::string type;
  json details;
  // The id the event goes out with; the logic sets it.
  std::string id;
};

struct Point2 {
  double x = 0;
  double y = 0;
};

// The docking station as map.json has it: base_link of the mower on the contacts, and the heading it drives in with.
struct DockPose {
  double x = 0;
  double y = 0;
  double heading = 0;
};

// Open: the last point does not repeat the first.
using Ring = std::vector<Point2>;

// Why the ring cannot become an outline, empty if it can: RING_FEW_POINTS, RING_CROSSES_ITSELF, RING_TOO_SMALL.
Reason RingProblem(const Ring& ring, double min_area_m2);
// Whether two segments that are not neighbours on the closed ring touch or cross.
bool SelfIntersects(const Ring& ring);
// m², of the closed ring.
double RingArea(const Ring& ring);

inline double Distance(const Point2& a, const Point2& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

// Rounded to mm, as positions go out.
inline double Millimetres(double metres) {
  return std::round(metres * 1000.0) / 1000.0;
}

// A ring whose end ran past its start: the loop from the first crossing of the line with itself, and how many
// points of the ring are not in it.
struct CutRing {
  Ring ring;
  size_t dropped = 0;
};
// The loop, or nullopt when the line does not cross itself where the path cut off before and after the crossing is
// at most max_cut (m).
std::optional<CutRing> CutOverhang(const Ring& ring, double max_cut);

// recording.undo without meters: the path (m) taken back from the end of the open ring.
constexpr double kUndoDefaultM = 1.0;

struct RecorderConfig {
  // A new point needs more than this distance (m) to the last one, NEW_POINT_MIN_DISTANCE of OpenMower.
  double min_point_distance = 0.1;
  // Points only from a position at least this accurate (m), max_position_accuracy of OpenMower.
  double max_position_accuracy = 0.1;
  // A ring must enclose at least this (m²), so that points on a line are no area.
  double min_area = 0.05;
  // Closing a ring that crosses itself once cuts off the start before the crossing and the end past it, together up
  // to this path (m); a longer one is more than driving past the start.
  double max_overhang = 2.0;
  // All rings of one recording together; the map must fit into one MQTT message.
  size_t max_points = 5000;
  // The two points of the docking station must be this far apart (m) for a usable heading: 2 cm across between them
  // turn it by 2.3 degrees at 0.5 m, by 3.8 at 0.3 m.
  double min_dock_distance = 0.5;
  // The mower backs onto the contacts (dock_contact_x below 0), for the hint in the log.
  bool dock_backwards = false;
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

  // From the next point on.
  void SetMaxPositionAccuracy(double accuracy) {
    config_.max_position_accuracy = accuracy;
  }

  // Drops everything recorded and not saved; an edit on its way is still reported when it is answered.
  void Reset();
  // An action of mower_logic:area_recording without the prefix, except exit_recording_mode; why: the reason of a
  // refusal that went out as RECORDING_REFUSED, empty for one without an event (the action is not enabled).
  Result OnAction(const std::string& name, Reason* why = nullptr, double undo_meters = kUndoDefaultM);
  // Points the last action cut off where the ring ran past its start.
  size_t trimmed_points() const {
    return trimmed_points_;
  }
  // heading_valid: the localization has its heading, else base_link may be off by the antenna offset.
  void OnPose(double x, double y, float position_accuracy, bool heading_valid);
  // The pose got too old.
  void OnPoseLost();
  // Whether mowbite_map takes changes now, and why not.
  void SetMapWritable(bool writable, const Reason& reason);
  // The debounced contact with the docking station, which the point on the contacts needs.
  void SetDockContact(bool contact) {
    dock_contact_ = contact;
  }

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
  // The display name of each of them, in the same order.
  static const std::vector<std::string>& ActionLabels();

  // Events since the last call, without a position.
  std::vector<Event> TakeEvents();
  // Lines for the log since the last call.
  std::vector<std::string> TakeNotices();
  bool TakeActionsChanged();

  // map_overlay/json like OpenMower: the outline green, obstacles red, the open ring blue; points in mm.
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
  Result Act(const std::string& name, double undo_meters);
  // Ends the open ring: the outline if there is none yet, else an obstacle; a ring without points just ends.
  bool CloseRing(const std::string& action);
  Result Finish(const std::string& action, const std::string& type);
  void Undo(double meters);
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
    bool heading_valid;
  };
  std::optional<Pose> pose_;
  bool accuracy_blocked_ = false;
  bool full_reported_ = false;
  size_t trimmed_points_ = 0;
  bool map_writable_ = false;
  bool dock_contact_ = false;
  Reason map_reason_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");
  // The edit waiting for its answer, and the one not yet taken by the node.
  std::optional<MapEdit> pending_;
  bool edit_untaken_ = false;
  std::vector<Event> events_;
  // The reason of the last Refuse, for OnAction.
  Reason refusal_;
  std::vector<std::string> notices_;
  bool actions_changed_ = true;
  bool overlay_changed_ = true;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_AREA_RECORDER_H
