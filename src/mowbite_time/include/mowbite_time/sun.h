#ifndef MOWBITE_TIME_SUN_H
#define MOWBITE_TIME_SUN_H

#include <optional>

namespace mowbite_time {

// Dark below this height of the sun (deg): the zenith of 90.833 deg the app takes for sunrise and sunset.
constexpr double kSunsetElevation = -0.833;

// Height of the sun over the horizon (deg, without refraction) at latitude and longitude (deg, east positive) at unix
// time unix_s (UTC), by the published formulas of the NOAA solar calculator.
double SunElevation(double latitude, double longitude, double unix_s);

// Sunrise and sunset (unix s) of the solar day whose noon is nearest to unix_s, where the sun crosses kSunsetElevation.
// Neither on a day the sun stays above it (polar_day) or below it (polar_night).
struct SunDay {
  double noon = 0;
  std::optional<double> rise;
  std::optional<double> set;
  bool polar_day = false;
  bool polar_night = false;
};
SunDay SunTimes(double latitude, double longitude, double unix_s);

}  // namespace mowbite_time

#endif  // MOWBITE_TIME_SUN_H
