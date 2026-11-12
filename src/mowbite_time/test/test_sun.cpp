#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_time/sun.h"

using namespace mowbite_time;

namespace {

// Unix s of a day of 2026 at hh:mm UTC.
double Utc(int month, int day, int hour, int minute) {
  static const int kDaysBefore[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  // 2026-01-01 00:00 UTC.
  const double year = 1767225600.0;
  return year + (kDaysBefore[month - 1] + day - 1) * 86400.0 + hour * 3600.0 + minute * 60.0;
}

// Within a minute of the table: it gives whole minutes.
void ExpectAt(const std::optional<double>& time, double expected, const char* what) {
  ASSERT_TRUE(time.has_value()) << what;
  EXPECT_NEAR(*time, expected, 60.0) << what;
}

struct Place {
  double latitude;
  double longitude;
};

// Sunrise and sunset of the NOAA solar calculator tables for 2026 (gml.noaa.gov/grad/solcalc/table.php), local times
// of the table here in UTC.
const Place kBerlin{52.52, 13.405};
const Place kSydney{-33.87, 151.21};
const Place kTromso{69.65, 18.96};

}  // namespace

TEST(Sun, RiseAndSetAsTheTablesOfTwoPlaces) {
  struct Case {
    Place place;
    double around;
    double rise;
    double set;
    const char* what;
  };
  const Case cases[] = {
      // Berlin, CEST in June (04:43, 21:33), CET in December and November (08:15, 15:54; 07:10, 16:29).
      {kBerlin, Utc(6, 21, 12, 0), Utc(6, 21, 2, 43), Utc(6, 21, 19, 33), "Berlin June 21"},
      {kBerlin, Utc(12, 21, 12, 0), Utc(12, 21, 7, 15), Utc(12, 21, 14, 54), "Berlin December 21"},
      {kBerlin, Utc(11, 5, 12, 0), Utc(11, 5, 6, 10), Utc(11, 5, 15, 29), "Berlin November 5"},
      // Sydney, AEST in June (07:00, 16:54), AEDT in December (05:41, 20:05): the local day begins the evening before.
      {kSydney, Utc(6, 21, 2, 0), Utc(6, 20, 21, 0), Utc(6, 21, 6, 54), "Sydney June 21"},
      {kSydney, Utc(12, 21, 2, 0), Utc(12, 20, 18, 41), Utc(12, 21, 9, 5), "Sydney December 21"},
      // Tromsø, CET (08:30, 14:24).
      {kTromso, Utc(11, 5, 12, 0), Utc(11, 5, 7, 30), Utc(11, 5, 13, 24), "Tromso November 5"},
  };
  for (const auto& c : cases) {
    const SunDay day = SunTimes(c.place.latitude, c.place.longitude, c.around);
    EXPECT_FALSE(day.polar_day) << c.what;
    EXPECT_FALSE(day.polar_night) << c.what;
    ExpectAt(day.rise, c.rise, c.what);
    ExpectAt(day.set, c.set, c.what);
    // At sunrise and sunset the sun stands at the limit, light at noon between them.
    EXPECT_NEAR(SunElevation(c.place.latitude, c.place.longitude, *day.rise), kSunsetElevation, 0.01) << c.what;
    EXPECT_NEAR(SunElevation(c.place.latitude, c.place.longitude, *day.set), kSunsetElevation, 0.01) << c.what;
    EXPECT_GT(day.noon, *day.rise) << c.what;
    EXPECT_LT(day.noon, *day.set) << c.what;
  }
}

TEST(Sun, PolarDayAndPolarNight) {
  const SunDay summer = SunTimes(kTromso.latitude, kTromso.longitude, Utc(6, 21, 12, 0));
  EXPECT_TRUE(summer.polar_day);
  EXPECT_FALSE(summer.polar_night);
  EXPECT_FALSE(summer.rise);
  EXPECT_FALSE(summer.set);
  // Light at midnight.
  EXPECT_GT(SunElevation(kTromso.latitude, kTromso.longitude, Utc(6, 21, 23, 0)), kSunsetElevation);

  const SunDay winter = SunTimes(kTromso.latitude, kTromso.longitude, Utc(12, 21, 12, 0));
  EXPECT_TRUE(winter.polar_night);
  EXPECT_FALSE(winter.polar_day);
  EXPECT_FALSE(winter.rise);
  EXPECT_FALSE(winter.set);
  // Dark at noon.
  EXPECT_LT(SunElevation(kTromso.latitude, kTromso.longitude, winter.noon), kSunsetElevation);
}

TEST(Sun, ElevationAtNoonAndMidnight) {
  // Near the equinox the sun at noon stands about 90 deg minus the latitude high, at midnight as low.
  const SunDay day = SunTimes(kBerlin.latitude, kBerlin.longitude, Utc(3, 20, 12, 0));
  EXPECT_NEAR(SunElevation(kBerlin.latitude, kBerlin.longitude, day.noon), 90.0 - 52.52, 0.5);
  EXPECT_NEAR(SunElevation(kBerlin.latitude, kBerlin.longitude, day.noon + 43200.0), -(90.0 - 52.52), 0.5);
  // Solar noon in Berlin is about 12:07 UTC minus 13.4 deg of longitude (54 min) on that day.
  EXPECT_NEAR(day.noon, Utc(3, 20, 11, 14), 120.0);
}

TEST(Sun, TheDayNearestToTheTime) {
  // Late in the evening and early in the morning of the same local day give the same sunset.
  const SunDay evening = SunTimes(kBerlin.latitude, kBerlin.longitude, Utc(6, 21, 21, 0));
  const SunDay morning = SunTimes(kBerlin.latitude, kBerlin.longitude, Utc(6, 21, 3, 0));
  ASSERT_TRUE(evening.set && morning.set);
  EXPECT_NEAR(*evening.set, *morning.set, 1.0);
}
