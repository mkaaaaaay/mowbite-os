#ifndef MOWBITE_GNSS_GGA_H
#define MOWBITE_GNSS_GGA_H

#include <cstdint>
#include <string>

namespace mowbite_gnss {

// GGA qualities a host can tell a caster.
enum class GgaQuality : int { kInvalid = 0, kSingle = 1, kRtkFixed = 4, kRtkFloat = 5 };

struct GgaFields {
  double utc_s = 0;  // seconds of the UTC day
  double lat_deg = 0;
  double lon_deg = 0;
  double altitude_m = 0;  // above mean sea level
  GgaQuality quality = GgaQuality::kInvalid;
  int satellites = 0;
  double hdop = 0;
};

// XOR of the bytes between '$' and '*'.
uint8_t NmeaChecksum(const std::string& body);

// A GGA sentence ("$GPGGA,...*hh", without line end) for a position the host knows from elsewhere, as an NTRIP caster
// wants it for a virtual reference station. Geoid separation, correction age and station stay empty.
std::string BuildGga(const GgaFields& fields);

}  // namespace mowbite_gnss

#endif  // MOWBITE_GNSS_GGA_H
