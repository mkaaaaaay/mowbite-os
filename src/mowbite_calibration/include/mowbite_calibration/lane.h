#ifndef MOWBITE_CALIBRATION_LANE_H
#define MOWBITE_CALIBRATION_LANE_H

#include <cmath>
#include <optional>
#include <vector>

#include "mowbite_safety/region.h"

namespace mowbite_calibration {

struct LaneConfig {
  // Lines of this length, or down to min_length where there is less room (the result gets worse).
  double length = 3.0;
  double min_length = 2.0;
  // Room beyond the body on every side of the strip, and the least distance of the strip from the docking station.
  double margin = 0.3;
  double station_clearance = 0.5;
  // Directions are tried in these steps (rad), nearest to the heading of the mower first.
  double direction_step = 10.0 * M_PI / 180.0;
  // Spacing of the points checked in the strip (m); an obstacle closer than this to one of them counts as hit, so none
  // slips between them.
  double resolution = 0.05;
};

// The way of base_link: from (x, y) length m along heading, and back.
struct Lane {
  double x = 0;
  double y = 0;
  double heading = 0;
  double length = 0;
};

struct LaneChoice {
  // None without room for min_length.
  std::optional<Lane> lane;
  // The longest lane that fits in any direction tried (m), 0 if none.
  double free_length = 0;
  // The strip the lane needs (m): length + 2 (body radius + margin) long, 2 (body radius + margin) wide; without a lane
  // the one of min_length.
  double strip_length = 0;
  double strip_width = 0;
};

// A lane needs a strip around it: all of it in the allowed areas of the region, outside of its obstacles and at least
// station_clearance from its docking station. Directions in steps of direction_step from heading, nearest first: the
// first with a lane of the full length, else the longest down to min_length. body_radius: the farthest point of the
// body from base_link (m).
LaneChoice ChooseLane(const mowbite_safety::Region& region, double x, double y, double heading, double body_radius,
                      const LaneConfig& config);

// Whether exactly this lane fits; the program checks a goal again with it where the mower stands.
bool LaneFits(const mowbite_safety::Region& region, const Lane& lane, double body_radius, const LaneConfig& config);

// The farthest point of the body from base_link: the outline (x ahead, y left) if it has 3 points or more, else the
// rectangle front m ahead, rear m behind, width m wide.
double BodyRadius(double front, double rear, double width, const std::vector<mowbite_safety::Point2>& outline);

}  // namespace mowbite_calibration

#endif  // MOWBITE_CALIBRATION_LANE_H
