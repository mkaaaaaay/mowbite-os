#ifndef MOWBITE_LOGIC_MARKER_DOCK_H
#define MOWBITE_LOGIC_MARKER_DOCK_H

#include <cstdint>
#include <optional>
#include <string>

#include "mowbite_logic/docking.h"

namespace mowbite_logic {

// When docking uses the marker (dock_guide): kNone never; kPrefer by the marker, by the position where its source
// cannot measure; kRequire never without it. docking.test always requires it.
enum class DockGuide : uint8_t { kNone = 0, kPrefer = 1, kRequire = 2 };

// Docking by the dock marker of the station (hw/dock_marker, localization/dock_pose): every leg in the station frame,
// origin base_link of the docked mower, x the way in; the map pose never steers it (G42). Platform values.
struct MarkerConfig {
  // m up to which the camera finds the marker (marker_range); 0: the platform has no marker and docks only by its
  // position. The line onto the station begins at most this far in front of it.
  double range = 0;
  // m before the docked pose where the marker leaves the image (marker_blind_distance): the last part is blind there.
  double blind_distance = 0.07;
  // s the logic waits for a good pose from the marker before a try fails (marker_wait).
  double wait = 25.0;
  // m base_link may get off a path in the station frame (marker_cross_track).
  double cross_track = 0.10;
  // At the gate the way along the line by the marker may differ from the way of the wheels by at most this part of it
  // (marker_scale_limit): a marker of the wrong size measures every distance wrong.
  double scale_limit = 0.15;
  // Whether docking.start and going home dock by the marker (dock_guide).
  DockGuide guide = DockGuide::kNone;
  // m of the marker in base_link of the docked mower (dock_marker, the platform's).
  double marker_x = 0;
  double marker_y = 0;
};

// What one message of hw/dock_marker tells the logic; time: monotonic s on arrival.
struct DockMarkerSample {
  double time = 0;
  // As DockMarker.msg: OFF 0, STARTING 1, MEASURING 2, FAULT 3.
  uint8_t state = 0;
  bool valid = false;
  bool yaw_valid = false;
  std::string reason;
};

// localization/dock_pose: base_link in the station frame; time: monotonic s on arrival; speed: of the wheels, m/s.
struct DockPoseSample {
  double time = 0;
  double x = 0;
  double y = 0;
  double heading = 0;
  bool heading_valid = false;
  float position_accuracy = 999;
  // rad; 999 unknown.
  float heading_accuracy = 999;
  double speed = 0;
};

// s without a message of hw/dock_marker after which its source counts as unable to measure (it reports every 1 s).
constexpr double kMarkerSilentS = 3.0;
// s after which the last dock_pose no longer counts.
constexpr double kDockPoseMaxAgeS = 0.5;
// m of way by the wheels below which the gate does not compare the scale.
constexpr double kScaleMinWay = 0.1;
// m before the approach point from which docking asks for the marker, so that the camera has found its exposure there.
constexpr double kMarkerRequestDistance = 3.0;
// rad the heading of the station by the marker may differ from the one of the map at the gate.
constexpr double kMarkerMapHeading = 10.0 * M_PI / 180.0;

// The station by the marker in the map: base_link at (x, y) facing heading in the map, and at pose in the station
// frame. Its position errs little with the direction of the marker, as the marker sits close to the docked base_link.
DockPose StationByMarker(double x, double y, double heading, const DockPoseSample& pose);

// Why the station by the marker does not fit the station of the map, empty if it does: further than limit m from it,
// or with heading_limit above 0 a heading further off than that (rad).
std::string MarkerMapProblem(const DockPose& by_marker, const DockPose& map, double limit, double heading_limit);

// The legs of docking by the marker: the station at the origin of the station frame, the line at most range m long.
DockingSequence MarkerDocking(DockConfig config, const UndockConfig& out, bool contacts_back,
                              const MarkerConfig& marker);

// What the logic hears of the marker while it docks by it, without ROS: the messages of the source since a wait began
// and the way along the line by marker and by wheels.
class MarkerWatch {
 public:
  // While waiting: kWaiting, or why it ended. kUnavailable: the source cannot measure (FAULT, OFF with a reason, silent
  // for kMarkerSilentS); kNotSeen: images came, but no marker in time; kNoDirection: the marker came, but never with
  // a direction of its own; kLost: on the line, the marker did not come back in time.
  enum class Verdict { kWaiting, kUnavailable, kNotSeen, kNoDirection, kLost };

  explicit MarkerWatch(const MarkerConfig& config = MarkerConfig()) : config_(config) {}

  void OnMarker(const DockMarkerSample& sample);
  // on_line: the line onto the station is driven now; its way counts from the first such pose.
  void OnPose(const DockPoseSample& pose, bool on_line);
  // A wait for the marker begins at now: sight before a leg, or lost on the way.
  void BeginWait(double now, bool sight);
  Verdict Check(double now) const;
  // The reason of the source that tells most about the verdict, empty without one.
  std::string Detail() const;
  // Why the way along the line by the marker does not fit the way of the wheels, empty if it does or was too short.
  std::string ScaleProblem() const;
  // A good pose: recent, with a heading, accurate to limit m and the source not reporting that it cannot measure.
  bool PoseGood(double now, double limit) const;
  // The latest message of the source says it cannot measure: FAULT, or OFF with a reason.
  bool SourceFault() const;
  const std::optional<DockPoseSample>& pose() const {
    return pose_;
  }
  bool sight() const {
    return sight_;
  }
  // The marker came with its direction since the wait began.
  bool seen() const {
    return valid_ && yaw_;
  }
  // The marker came since the wait began, with a direction or without.
  bool found() const {
    return valid_;
  }
  // Drops the pose at its age and forgets everything at the end of docking.
  void Expire(double now);
  void Reset();

 private:
  MarkerConfig config_;
  std::optional<DockPoseSample> pose_;
  // The wait: since when, sight or lost, the last message and since when, the reason of the last image without a
  // marker, and what came meanwhile.
  double since_ = 0;
  bool sight_ = false;
  std::optional<DockMarkerSample> last_;
  double heard_ = 0;
  std::string image_reason_;
  bool measuring_ = false;
  bool valid_ = false;
  bool yaw_ = false;
  // The line: where it began by the marker and the way of the wheels since.
  bool on_line_ = false;
  double line_x_ = 0;
  double wheel_way_ = 0;
};

// m the marker lies beside the axis of base_link at (x, y) facing heading in the station frame, against where it lies
// on the docked mower, left positive: driving straight the mower passes it that far off. The marker yaw errs by
// degrees that stay at one place, and the station frame turns with it about the marker; this does not.
double SideOffset(const DockingSequence& docking, const MarkerConfig& marker, double x, double y, double heading);

// Why a docking test cannot begin from pose (station frame of docking), empty if it can: facing the way the mower faces
// on the station; at the approach point, or on the line with the marker at most lateral m beside its axis
// (SideOffset); and where the line begins (the approach point after the straight way out, or where it stands there)
// at most lateral m plus the spread of the heading times the way from there to the marker beside the line, never more
// than marker.cross_track: the heading of the marker counts only with its spread.
std::string TestStartProblem(const DockingSequence& docking, const MarkerConfig& marker, const DockPoseSample& pose,
                             double lateral);

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_MARKER_DOCK_H
