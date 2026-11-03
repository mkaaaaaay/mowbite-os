#ifndef MOWBITE_SAFETY_REGION_H
#define MOWBITE_SAFETY_REGION_H

#include <optional>
#include <string>
#include <vector>

namespace mowbite_safety {

struct Point2 {
  double x = 0;
  double y = 0;
};

// Open or closed, either way.
using Ring = std::vector<Point2>;

bool InsideRing(const Ring& ring, double x, double y);
// m from the point to the edge of the ring.
double DistanceToRing(const Ring& ring, double x, double y);

// The docking station as map.json has it: base_link on the contacts and the heading the mower backs in with.
struct Station {
  double x = 0;
  double y = 0;
  double heading = 0;
};

// Where autonomous drives may take base_link, read from map.json: the active mow and navigation areas outside of the
// active obstacles, and the line of the active docking station.
struct Region {
  std::vector<Ring> allowed;
  std::vector<Ring> obstacles;
  std::optional<Station> station;
};

// The region of map.json; nullopt and why in error when the text is no map or an entry is malformed, since a dropped
// obstacle would widen the region.
std::optional<Region> ParseRegion(const std::string& map_json, std::string* error);

// The rectangle docking and undocking drive base_link in: behind m past the station, ahead m in front of it (the side
// the mower leaves on), half_width m to each side of its line.
Ring StationZone(const Station& station, double behind, double ahead, double half_width);

}  // namespace mowbite_safety

#endif  // MOWBITE_SAFETY_REGION_H
