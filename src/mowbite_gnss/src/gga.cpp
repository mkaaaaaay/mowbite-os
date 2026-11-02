#include "mowbite_gnss/gga.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mowbite_gnss {

namespace {

constexpr long long kMinuteUnits = 10000000;  // the minutes carry 7 decimals, about 2 mm
constexpr long long kCentisecondsPerDay = 8640000;

// Degrees as d..dmm.mmmmmmm with the hemisphere letter; the rounding carries into the degrees.
std::string Angle(double deg, int degree_digits, char positive, char negative) {
  const long long units = std::llround(std::abs(deg) * 60.0 * kMinuteUnits);
  const long long degrees = units / (60 * kMinuteUnits);
  const long long minutes = units % (60 * kMinuteUnits);
  char text[40];
  std::snprintf(text, sizeof(text), "%0*lld%02lld.%07lld,%c", degree_digits, degrees, minutes / kMinuteUnits,
                minutes % kMinuteUnits, deg < 0 ? negative : positive);
  return text;
}

}  // namespace

uint8_t NmeaChecksum(const std::string& body) {
  uint8_t sum = 0;
  for (char c : body) sum ^= static_cast<uint8_t>(c);
  return sum;
}

std::string BuildGga(const GgaFields& f) {
  long long cs = std::llround(f.utc_s * 100.0) % kCentisecondsPerDay;
  if (cs < 0) cs += kCentisecondsPerDay;
  char time[32];
  std::snprintf(time, sizeof(time), "%02lld%02lld%02lld.%02lld", cs / 360000, cs / 6000 % 60, cs / 100 % 60, cs % 100);
  char rest[96];
  std::snprintf(rest, sizeof(rest), ",%d,%02d,%.1f,%.3f,M,,M,,", static_cast<int>(f.quality),
                std::clamp(f.satellites, 0, 99), std::clamp(f.hdop, 0.0, 99.9), f.altitude_m);
  const std::string body =
      "GPGGA," + std::string(time) + "," + Angle(f.lat_deg, 2, 'N', 'S') + "," + Angle(f.lon_deg, 3, 'E', 'W') + rest;
  char sum[4];
  std::snprintf(sum, sizeof(sum), "%02X", NmeaChecksum(body));
  return "$" + body + "*" + sum;
}

}  // namespace mowbite_gnss
