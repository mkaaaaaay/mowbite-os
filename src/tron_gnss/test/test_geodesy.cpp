#include <gtest/gtest.h>

#include <cmath>

#include "tron_gnss/geodesy.h"

namespace tron {
namespace {

constexpr double kDeg = M_PI / 180.0;

TEST(Geodesy, DatumIsOrigin) {
  LocalFrame frame(52.0, 9.0, 60.0);
  Enu p = frame.ToEnu(52.0, 9.0, 60.0);
  EXPECT_NEAR(p.east, 0.0, 1e-6);
  EXPECT_NEAR(p.north, 0.0, 1e-6);
  EXPECT_NEAR(p.up, 0.0, 1e-6);
}

TEST(Geodesy, NorthAndEastMatchTheEllipsoidRadii) {
  const double lat = 52.0, a = 6378137.0, e2 = 6.69437999014e-3;
  const double s = std::sin(lat * kDeg);
  const double w = std::sqrt(1.0 - e2 * s * s);
  const double meridian_radius = a * (1.0 - e2) / (w * w * w);
  const double prime_vertical = a / w;
  LocalFrame frame(lat, 9.0, 0.0);

  Enu north = frame.ToEnu(lat + 0.001, 9.0, 0.0);
  EXPECT_NEAR(north.north, meridian_radius * 0.001 * kDeg, 0.01);
  EXPECT_NEAR(north.east, 0.0, 0.01);

  Enu east = frame.ToEnu(lat, 9.001, 0.0);
  EXPECT_NEAR(east.east, prime_vertical * std::cos(lat * kDeg) * 0.001 * kDeg, 0.01);
  EXPECT_NEAR(east.north, 0.0, 0.01);
}

TEST(Geodesy, SignsAtAllHemispheres) {
  for (double lat : {-45.0, 0.0, 60.0}) {
    for (double lon : {-120.0, 0.0, 170.0}) {
      LocalFrame frame(lat, lon, 0.0);
      Enu p = frame.ToEnu(lat - 0.0005, lon - 0.0005, 0.0);
      EXPECT_LT(p.north, 0.0);
      EXPECT_LT(p.east, 0.0);
    }
  }
}

TEST(Geodesy, HeightIsUp) {
  LocalFrame frame(52.0, 9.0, 60.0);
  Enu p = frame.ToEnu(52.0, 9.0, 62.5);
  EXPECT_NEAR(p.up, 2.5, 1e-6);
  EXPECT_NEAR(p.east, 0.0, 1e-6);
  EXPECT_NEAR(p.north, 0.0, 1e-6);
}

TEST(Geodesy, DistanceIsPreservedOverAGarden) {
  LocalFrame frame(52.0, 9.0, 60.0);
  Enu p = frame.ToEnu(52.0003, 9.0004, 60.0);
  // Independent flat-earth estimate, within a few cm over 50 m.
  const double north = 0.0003 * 111283.0;
  const double east = 0.0004 * 111321.0 * std::cos(52.0 * kDeg);
  EXPECT_NEAR(p.north, north, 0.1);
  EXPECT_NEAR(p.east, east, 0.1);
}

}  // namespace
}  // namespace tron
