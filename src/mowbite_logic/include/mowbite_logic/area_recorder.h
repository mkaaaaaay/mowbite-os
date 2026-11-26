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
  // The index in the old ring of the first point kept; the crossing put in before it, if added.
  size_t first = 0;
  bool crossing_added = false;
};
// The loop, or nullopt when the line does not cross itself where the path cut off before and after the crossing is
// at most max_cut (m).
std::optional<CutRing> CutOverhang(const Ring& ring, double max_cut);

// What the recorder knows of a point besides where it is.
struct PointMark {
  // rad, of base_link when the point was taken
  double heading = 0;
  bool heading_valid = false;
  // false: taken by collect_point
  bool automatic = false;
  // No interruption since the point before: automatic points stayed on, no undo, no inaccurate or lost pose, no jump.
  bool joined = false;
  // The mower reversed since the point before, so the edge to it was not driven.
  bool reversed = false;
};

// Why the track a ring was closed from is not the forward driven track of the body, empty if it is: MANUAL_POINT,
// NO_HEADING, INTERRUPTED, REVERSED, CLOSING, GAP, DIRECTION or SHARP_TURN, the first of them in this order that
// applies. crossed: the ring was closed where its line crossed itself, and track runs from the point before the
// crossing to the first point past it, so that its first and last edges cross. Without a crossing track is the ring,
// whose closing edge was never driven. marks has one entry per point of track; joined and reversed of the first point
// belong to the edge before it and do not count. Only rings that pass get recorded_body, which lets the planner drive
// the line without a margin.
std::string DrivenProblem(const Ring& track, const std::vector<PointMark>& marks, bool crossed);

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

  // Fixed limits of the driven track, no settings. base_link moved this far (m) from where the direction was judged
  // last is judged again: backwards takes points back.
  static constexpr double kReverseStep = 0.05;
  // This many judged steps backwards in a row are reversing; a step that is not ends it.
  static constexpr int kReverseSteps = 2;
  // The wheels this far (m) backwards are reversing, so that the next point does not join the driven track: more than
  // they roll back when stopping, less than the 0.10 m the pose needs to show it.
  static constexpr double kReverseTravel = 0.02;
  // Travel backwards that grows slower than this (m/s) is noise of standing wheels (a reported speed that swings
  // around a small bias while they stand): it leaks away instead of adding up to kReverseTravel.
  static constexpr double kWheelLeak = 0.005;
  // Wheel measurements further apart than this (s), or none, are a break of the driven track.
  static constexpr double kWheelGap = 0.5;
  // An edge of a driven ring is at most this long (m).
  static constexpr double kMaxDrivenEdge = 0.3;
  // An edge of a driven ring points between the headings at its ends, widened by this (rad) on each side.
  static constexpr double kDirectionAllowance = M_PI / 3;
  // Two edges of a driven ring turn by at most this (rad).
  static constexpr double kMaxDrivenTurn = 2 * M_PI / 3;
  // A driven ring turns by at most this (rad) at its first point, where the end crossed the start.
  static constexpr double kMaxClosingTurn = M_PI / 6;
};

// A change of the map the recording asks mowbite_map for.
struct MapEdit {
  enum class Kind { kArea, kDock };
  Kind kind = Kind::kArea;
  // kArea: mow or nav
  std::string type;
  Ring outline;
  std::vector<Ring> obstacles;
  // DrivenProblem of the outline and of each obstacle, empty for a driven ring.
  std::string outline_driven_reason;
  std::vector<std::string> obstacles_driven_reason;
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
  // An action id of Actions(), except recording.exit; why: the reason of a refusal that went out as
  // RECORDING_REFUSED, empty for one without an event (the action is not enabled).
  Result OnAction(const std::string& id, Reason* why = nullptr, double undo_meters = kUndoDefaultM);
  // Points the last action cut off where the ring ran past its start.
  size_t trimmed_points() const {
    return trimmed_points_;
  }
  // heading (rad) counts only with heading_valid: the localization has its heading, else base_link may be off by the
  // antenna offset.
  void OnPose(double x, double y, double heading, float position_accuracy, bool heading_valid);
  // The pose got too old.
  void OnPoseLost();
  // The localization jumped back onto GPS.
  void OnPositionJump();
  // m the wheels moved along base_link in the seconds since the last call, negative backwards (hw/wheel_odom).
  void OnWheelTravel(double metres, double seconds);
  // No wheel travel for RecorderConfig::kWheelGap: reversing would go unseen, so neither the points set since the
  // last travel nor the next one join the driven track.
  void OnWheelsLost();
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
  // An action of the recorder: id of the catalog (empty for the two places of manual mowing, which no command
  // stands for), name of the step as RECORDING_REFUSED gives it in action, label to show.
  struct ActionEntry {
    std::string id;
    std::string name;
    std::string label;
  };
  // Enabled flags in the order of Actions().
  std::vector<bool> Enabled() const;
  static const std::vector<ActionEntry>& Actions();

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
  // The marks of the open ring, one per point.
  const std::vector<PointMark>& marks() const {
    return marks_;
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
  Result Act(const std::string& id, double undo_meters);
  // Ends the open ring: the outline if there is none yet, else an obstacle; a ring without points just ends.
  bool CloseRing(const std::string& action);
  Result Finish(const std::string& action, const std::string& type);
  void Undo(double meters);
  Result RecordDock();
  void AddPoint(double x, double y, const PointMark& mark);
  void DropLastPoint();
  // Backwards takes back the last points in front of base_link; true while the mower reverses.
  bool TakeBackReversing(double x, double y, double heading);
  void EndReversing();
  // REVERSING once for each stretch backwards.
  void HintReversing();
  void Interrupt();
  size_t PointCount() const;

  RecorderConfig config_;
  bool recording_ = false;
  Ring current_;
  std::vector<PointMark> marks_;
  // Since the last point of the open ring.
  bool interrupted_ = false;
  // Where the direction of base_link was judged last, the steps backwards in a row since, whether the mower
  // reverses and the points taken back since.
  std::optional<Point2> reverse_anchor_;
  int backward_steps_ = 0;
  bool reversing_ = false;
  size_t reverse_taken_ = 0;
  // Since the last point of the open ring.
  bool reversed_ = false;
  // m the wheels went back since their furthest point forward or since they last showed reversing, and whether this
  // stretch backwards had its hint.
  double wheel_back_ = 0;
  bool reverse_hinted_ = false;
  // Points of the open ring the wheels have told their travel up to.
  size_t wheel_points_ = 0;
  std::optional<Ring> outline_;
  std::vector<Ring> obstacles_;
  std::string outline_driven_reason_;
  std::vector<std::string> obstacles_driven_reason_;
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
