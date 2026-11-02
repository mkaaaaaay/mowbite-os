#include "mowbite_gnss/geodesy.h"

#include <cmath>

namespace mowbite_gnss {

namespace {

constexpr double kA = 6378137.0;          // WGS84 semi-major axis
constexpr double kE2 = 6.69437999014e-3;  // first eccentricity squared
constexpr double kDegToRad = M_PI / 180.0;

struct Ecef {
  double x, y, z;
};

Ecef ToEcef(double lat_deg, double lon_deg, double height_m) {
  double lat = lat_deg * kDegToRad, lon = lon_deg * kDegToRad;
  double s = std::sin(lat);
  double n = kA / std::sqrt(1.0 - kE2 * s * s);
  return {(n + height_m) * std::cos(lat) * std::cos(lon), (n + height_m) * std::cos(lat) * std::sin(lon),
          (n * (1.0 - kE2) + height_m) * s};
}

}  // namespace

LocalFrame::LocalFrame(double datum_lat_deg, double datum_lon_deg, double datum_height_m)
    : sin_lat_(std::sin(datum_lat_deg * kDegToRad)),
      cos_lat_(std::cos(datum_lat_deg * kDegToRad)),
      sin_lon_(std::sin(datum_lon_deg * kDegToRad)),
      cos_lon_(std::cos(datum_lon_deg * kDegToRad)) {
  Ecef origin = ToEcef(datum_lat_deg, datum_lon_deg, datum_height_m);
  x0_ = origin.x;
  y0_ = origin.y;
  z0_ = origin.z;
}

Enu LocalFrame::ToEnu(double lat_deg, double lon_deg, double height_m) const {
  Ecef p = ToEcef(lat_deg, lon_deg, height_m);
  double dx = p.x - x0_, dy = p.y - y0_, dz = p.z - z0_;
  Enu enu;
  enu.east = -sin_lon_ * dx + cos_lon_ * dy;
  enu.north = -sin_lat_ * cos_lon_ * dx - sin_lat_ * sin_lon_ * dy + cos_lat_ * dz;
  enu.up = cos_lat_ * cos_lon_ * dx + cos_lat_ * sin_lon_ * dy + sin_lat_ * dz;
  return enu;
}

}  // namespace mowbite_gnss
