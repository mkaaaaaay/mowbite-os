#ifndef MOWBITE_LOGIC_DOCKING_H
#define MOWBITE_LOGIC_DOCKING_H

#include <cmath>
#include <string>
#include <vector>

#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/undocking.h"

namespace mowbite_logic {

// Docking onto the station of the map with the contacts at the back: over the drivable areas to the point
// approach_distance m in front of it, facing away from it, then backwards along its line to the gate, where the mower
// stops and is checked, and on at contact_speed until the contacts. RTK steers the whole way; a camera may take over
// the last part later (O8). Assumptions until measured on the Tron in phase 7.
struct DockConfig {
  // m in front of the station along its way in (docking_approach_distance of params.set).
  double approach_distance = 0.8;
  // m/s backwards along the line, and from the gate on.
  double speed = 0.1;
  double contact_speed = 0.06;
  // gate_distance m before the station base_link has to stand at most gate_lateral m beside the line, its heading
  // within gate_heading (rad) of the one on the station, with the position accuracy of the station; else the try fails.
  double gate_distance = 0.25;
  double gate_lateral = 0.02;
  double gate_heading = 8.0 * M_PI / 180.0;
  // m the line reaches past the station: that far the mower looks for the contacts.
  double overshoot = 0.05;
  // s the mower may stand at the station without a debounced contact, pushing or waiting; on the line before the gate
  // standing counts as stuck after the stall time of undocking.
  double contact_timeout = 10.0;
  // Failed tries that are tried again (docking_retry_count of params.set).
  int retry_count = 4;
};

// What the follower drives for docking.
struct DockingLeg {
  // kApproach: over the drivable areas to the point in front of the station, planned like a drive. kOut: forwards along
  // the mower's own heading out of the station to that point. kLine: backwards along the line to the gate. kContact:
  // from the gate onto the contacts.
  enum class Kind { kApproach, kOut, kLine, kContact };
  Kind kind = Kind::kApproach;
  // kApproach: the point in front of the station, facing the way the mower faces on it.
  DockPose goal;
  // The others: poses at most 0.1 m apart that face the way the mower faces, and where base_link may be besides the
  // drivable areas; the rest as in FollowPath.
  std::vector<DockPose> poses;
  Ring corridor;
  bool straight = false;
  bool reverse = false;
  double speed = 0;
  double stall_time = 0;
  double end_tolerance = 0;
};

const char* DockingLegName(DockingLeg::Kind kind);

// The geometry and the tries of one docking, without ROS. LogicCore decides which leg comes next.
class DockingSequence {
 public:
  DockingSequence() = default;
  // station as in the map (heading: the way in), facing: the heading of the mower on it; out: how the mower leaves the
  // station forwards, like undocking.
  DockingSequence(const DockConfig& config, const UndockConfig& out, const DockPose& station, double facing);

  // Where the line onto the station begins.
  Point2 ApproachPoint() const;
  // m from the approach point along the way in (towards the station), and across it (left positive).
  double Along(double x, double y) const;
  double Across(double x, double y) const;
  // Whether base_link stands on the line between the approach point and the station, where a try begins by driving
  // forwards out to the approach point.
  bool OnLine(double x, double y) const;

  // The first leg of a try from base_link at (x, y) facing heading: out to the approach point from the line, the line
  // from the approach point, else the approach over the drivable areas.
  DockingLeg FirstLeg(double x, double y, double heading) const;
  DockingLeg Approach() const;
  DockingLeg Out(double x, double y, double heading) const;
  DockingLeg Line() const;
  DockingLeg Contact() const;

  // Why the mower may not go on from where it stands at the gate, empty if it may: base_link beside the line, the
  // heading off, or the position accuracy worse than accuracy_limit.
  std::string CheckGate(double x, double y, double heading, float accuracy, double accuracy_limit) const;

  // Counts a failed try; true when another one follows.
  bool Failed();
  int failures() const {
    return failures_;
  }
  const DockConfig& config() const {
    return config_;
  }

 private:
  // Poses facing the mower's heading on the station from along m to to m (from the approach point), and the corridor
  // of the line.
  DockingLeg Backwards(DockingLeg::Kind kind, double from, double to) const;

  DockConfig config_;
  UndockConfig out_;
  DockPose station_;
  double facing_ = 0;
  int failures_ = 0;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_DOCKING_H
