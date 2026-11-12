#include "mowbite_time/sun.h"

#include <cmath>

namespace mowbite_time {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDay = 86400.0;

double Rad(double deg) {
  return deg * kPi / 180.0;
}

double Deg(double rad) {
  return rad * 180.0 / kPi;
}

// Where the sun stands on its way at this time: declination (rad) and the equation of time (min).
struct SunPosition {
  double declination = 0;
  double equation_of_time = 0;
};

SunPosition Position(double unix_s) {
  // Julian centuries since J2000.0.
  const double century = (unix_s / kDay + 2440587.5 - 2451545.0) / 36525.0;
  const double mean_longitude = std::fmod(280.46646 + century * (36000.76983 + century * 0.0003032), 360.0);
  const double mean_anomaly = 357.52911 + century * (35999.05029 - 0.0001537 * century);
  const double eccentricity = 0.016708634 - century * (0.000042037 + 0.0000001267 * century);
  const double m = Rad(mean_anomaly);
  const double center = std::sin(m) * (1.914602 - century * (0.004817 + 0.000014 * century)) +
                        std::sin(2 * m) * (0.019993 - 0.000101 * century) + std::sin(3 * m) * 0.000289;
  const double omega = Rad(125.04 - 1934.136 * century);
  const double apparent_longitude = mean_longitude + center - 0.00569 - 0.00478 * std::sin(omega);
  const double mean_obliquity =
      23.0 + (26.0 + (21.448 - century * (46.815 + century * (0.00059 - century * 0.001813))) / 60.0) / 60.0;
  const double obliquity = Rad(mean_obliquity + 0.00256 * std::cos(omega));
  SunPosition out;
  out.declination = std::asin(std::sin(obliquity) * std::sin(Rad(apparent_longitude)));
  const double y = std::pow(std::tan(obliquity / 2), 2);
  const double l = Rad(mean_longitude);
  out.equation_of_time =
      4.0 * Deg(y * std::sin(2 * l) - 2 * eccentricity * std::sin(m) +
                4 * eccentricity * y * std::sin(m) * std::cos(2 * l) - 0.5 * y * y * std::sin(4 * l) -
                1.25 * eccentricity * eccentricity * std::sin(2 * m));
  return out;
}

// Where the sun crosses kSunsetElevation between from and to, given that it is on the other side at each end.
double Crossing(double latitude, double longitude, double from, double to) {
  const bool rising = SunElevation(latitude, longitude, from) < kSunsetElevation;
  for (int i = 0; i < 40 && to - from > 0.5; ++i) {
    const double mid = (from + to) / 2;
    if ((SunElevation(latitude, longitude, mid) < kSunsetElevation) == rising) {
      from = mid;
    } else {
      to = mid;
    }
  }
  return (from + to) / 2;
}

}  // namespace

double SunElevation(double latitude, double longitude, double unix_s) {
  const SunPosition sun = Position(unix_s);
  const double minutes = std::fmod(std::fmod(unix_s, kDay) + kDay, kDay) / 60.0;
  const double solar_time = std::fmod(minutes + sun.equation_of_time + 4.0 * longitude + 2880.0, 1440.0);
  const double hour_angle = Rad(solar_time / 4.0 - 180.0);
  const double lat = Rad(latitude);
  const double cos_zenith =
      std::sin(lat) * std::sin(sun.declination) + std::cos(lat) * std::cos(sun.declination) * std::cos(hour_angle);
  return 90.0 - Deg(std::acos(std::fmax(-1.0, std::fmin(1.0, cos_zenith))));
}

SunDay SunTimes(double latitude, double longitude, double unix_s) {
  // Solar noon of the day nearest to unix_s: 12:00 UTC shifted by the longitude and the equation of time.
  double noon = std::floor(unix_s / kDay) * kDay + kDay / 2 - longitude * 240.0;
  while (noon - unix_s > kDay / 2) noon -= kDay;
  while (unix_s - noon > kDay / 2) noon += kDay;
  for (int i = 0; i < 2; ++i) {
    noon = std::floor((noon + longitude * 240.0) / kDay) * kDay + kDay / 2 - longitude * 240.0 -
           Position(noon).equation_of_time * 60.0;
  }
  SunDay day;
  day.noon = noon;
  const double before = noon - kDay / 2;
  const double after = noon + kDay / 2;
  if (SunElevation(latitude, longitude, noon) < kSunsetElevation) {
    day.polar_night = true;
    return day;
  }
  const bool dark_before = SunElevation(latitude, longitude, before) < kSunsetElevation;
  const bool dark_after = SunElevation(latitude, longitude, after) < kSunsetElevation;
  if (dark_before) day.rise = Crossing(latitude, longitude, before, noon);
  if (dark_after) day.set = Crossing(latitude, longitude, noon, after);
  day.polar_day = !dark_before && !dark_after;
  return day;
}

}  // namespace mowbite_time
