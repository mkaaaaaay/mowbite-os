// The datum: the point on the earth where the local frame of the map starts (x east, y north).
#pragma once

#include <cmath>

namespace mowbite_map {

struct Datum {
  // degrees
  double lat = 0.0;
  double lon = 0.0;
  // m, kept with the map but not compared: the map is flat
  double height = 0.0;
};

inline bool IsValid(const Datum& d) {
  return std::isfinite(d.lat) && std::isfinite(d.lon) && std::isfinite(d.height) && std::abs(d.lat) <= 90.0 &&
         std::abs(d.lon) <= 180.0;
}

// How far two datums lie apart on the ground in m, flat earth: good for the few metres that matter here.
inline double DistanceM(const Datum& a, const Datum& b) {
  constexpr double kMetresPerDegree = 111320.0;
  constexpr double kPi = 3.14159265358979323846;
  const double mean_lat = (a.lat + b.lat) / 2.0 * kPi / 180.0;
  const double north = (a.lat - b.lat) * kMetresPerDegree;
  const double east = (a.lon - b.lon) * kMetresPerDegree * std::cos(mean_lat);
  return std::hypot(north, east);
}

}  // namespace mowbite_map
