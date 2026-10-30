#ifndef TRON_GNSS_NMEA_H
#define TRON_GNSS_NMEA_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace tron {

// GGA fix quality field.
enum class FixQuality : int {
  kInvalid = 0,
  kSingle = 1,
  kDgps = 2,
  kRtkFixed = 4,
  kRtkFloat = 5,
  kDeadReckoning = 6,
};

struct GgaData {
  double utc_s = 0;  // seconds since midnight UTC
  std::optional<double> lat_deg;
  std::optional<double> lon_deg;
  int quality = 0;  // raw field, see FixQuality
  int satellites = 0;
  std::optional<double> hdop;
  std::optional<double> altitude_m;        // above mean sea level
  std::optional<double> correction_age_s;  // age of the differential corrections, empty without
};

struct RmcData {
  double utc_s = 0;
  bool valid = false;                // status A and a mode other than N
  std::optional<double> speed_mps;   // over ground
  std::optional<double> course_deg;  // true, clockwise from north
};

struct VtgData {
  bool valid = false;  // mode other than N
  std::optional<double> speed_mps;
  std::optional<double> course_deg;
};

// $GNRTK of the RTK board (AT32 in front of the UM960): whether it passes the host's corrections (network RTK, ON) or
// uses its LoRa radio (OFF).
struct BoardRtkData {
  bool on = false;
};

struct NmeaMessage {
  enum class Type { kGga, kRmc, kVtg, kBoardRtk } type = Type::kGga;
  GgaData gga;
  RmcData rmc;
  VtgData vtg;
  BoardRtkData board;
  // The sentence as received, without line end.
  std::string sentence;
};

// Splits a byte stream into lines, checks the checksum and decodes GGA, RMC and VTG of any talker and the board's RTK
// report. Other sentences are counted and dropped. Lines longer than kMaxLine are discarded up to their end.
class NmeaParser {
 public:
  static constexpr size_t kMaxLine = 256;

  // Appends the messages completed by these bytes to out.
  void Feed(const uint8_t* data, size_t size, std::vector<NmeaMessage>& out);

  uint64_t checksum_errors() const {
    return checksum_errors_;
  }
  uint64_t malformed() const {
    return malformed_;
  }
  uint64_t ignored() const {
    return ignored_;
  }
  uint64_t decoded() const {
    return decoded_;
  }

 private:
  void HandleLine(const std::string& line, std::vector<NmeaMessage>& out);

  std::string line_;
  bool overflow_ = false;
  uint64_t checksum_errors_ = 0;
  uint64_t malformed_ = 0;
  uint64_t ignored_ = 0;
  uint64_t decoded_ = 0;
};

// XOR of the characters between '$' and '*'.
uint8_t NmeaChecksum(const std::string& body);

}  // namespace tron

#endif  // TRON_GNSS_NMEA_H
