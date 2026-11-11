#ifndef MOWBITE_LOGIC_DIRECT_ROUTE_H
#define MOWBITE_LOGIC_DIRECT_ROUTE_H

#include <string>
#include <vector>

#include "mowbite_logic/area_recorder.h"

namespace mowbite_logic {

// The check of a straight way for a mower whose planner does not answer. It turns on the spot, drives the line and
// turns on the spot again, and the follower may turn it on the spot anywhere in between. So the body has to fit in
// every heading wherever base_link may get: base_link stays within margin m of the line (how far off the follower
// lets it get before it ends the leg, and how exact the position is it measures that by), the body within
// turning_radius m of base_link. Nothing may lie closer to the line than the sum of both, ends included. Without ROS.
struct DirectRouteCheck {
  bool free = false;
  // Not free: what is in the way and where, for the message of the event.
  std::string why;
};

// The line from `from` (base_link now) to `to`. drivable: the areas base_link may drive in, they may overlap;
// obstacles: where it must never be. Free only if `from` lies in a drivable area and in no obstacle, no edge of an
// obstacle is closer to the line than turning_radius + margin, and no edge of a drivable area is either, except
// where it runs at least 1 cm deep inside another drivable area. Areas that only touch count as apart. Edges are
// sampled every 1 cm and held 1 cm farther off than needed, so the answer is never too generous. Not free without a
// turning_radius and a margin above 0.
DirectRouteCheck CheckDirectRoute(const Point2& from, const Point2& to, double turning_radius, double margin,
                                  const std::vector<Ring>& drivable, const std::vector<Ring>& obstacles);

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_DIRECT_ROUTE_H
