#ifndef MOWBITE_LOGIC_UNDOCKING_H
#define MOWBITE_LOGIC_UNDOCKING_H

#include <cmath>
#include <optional>
#include <vector>

#include "mowbite_logic/area_recorder.h"

namespace mowbite_logic {

// Leaving the docking station before mowing or a drive starts on it: straight ahead the way the mower faces there, out
// of the hollows and guides for its wheels, without steering. Assumptions until measured on the Tron in phase 7.
struct UndockConfig {
  // m along the line (undock_distance of params.set).
  double distance = 0.8;
  // m/s at most, also while the wheels climb out of the hollows.
  double speed = 0.15;
  // s the mower may stand while driven, then undocking fails instead of pushing on.
  double stall_time = 2.0;
  // Once the GNSS antenna got course_distance (m) away from where it was at the start, the way it moved may turn at
  // most course_limit (rad) off the line of the station: more means the station is turned or its heading recorded
  // wrong.
  double course_limit = 10.0 * M_PI / 180.0;
  double course_distance = 0.3;
  // m to each side of the line where base_link may be, on top of the drivable areas and the margin of the follower.
  double corridor = 0.1;
};

// The way out, as the follower drives it.
struct UndockPath {
  // From where base_link stands to distance m ahead, at most 0.1 m apart, all facing the way out; at least 3.
  std::vector<DockPose> poses;
  // The line, corridor wide to each side and reaching that far behind the start.
  Ring corridor;
  double speed = 0;
  double stall_time = 0;
};

// The way out from base_link at (x, y) facing heading.
UndockPath PlanUndock(double x, double y, double heading, const UndockConfig& config);

// Whether (x, y) lies in the open ring.
bool InsideRing(const Ring& ring, double x, double y);

// Whether base_link may end there: in one of the drivable areas and in none of the obstacles.
bool Reachable(const std::vector<Ring>& drivable, const std::vector<Ring>& obstacles, double x, double y);

// rad the way from (from_x, from_y) to (x, y) turns off heading, in (-pi, pi]; none closer than min_distance m.
std::optional<double> CourseError(double from_x, double from_y, double heading, double x, double y,
                                  double min_distance);

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_UNDOCKING_H
