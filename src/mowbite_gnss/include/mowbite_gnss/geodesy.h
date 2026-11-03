#ifndef MOWBITE_GNSS_GEODESY_H
#define MOWBITE_GNSS_GEODESY_H

namespace mowbite_gnss {

struct Enu {
  double east = 0;
  double north = 0;
  double up = 0;
};

// Local tangent plane (east, north, up) around a datum on the WGS84 ellipsoid.
class LocalFrame {
 public:
  LocalFrame(double datum_lat_deg, double datum_lon_deg, double datum_height_m);

  Enu ToEnu(double lat_deg, double lon_deg, double height_m) const;

 private:
  double sin_lat_, cos_lat_, sin_lon_, cos_lon_;
  double x0_, y0_, z0_;
};

}  // namespace mowbite_gnss

#endif  // MOWBITE_GNSS_GEODESY_H
