#include "tron_gnss/nmea.h"

#include <cmath>
#include <cstdlib>

namespace tron {

namespace {

constexpr double kKnotsToMps = 0.514444444;

std::vector<std::string> Split(const std::string& s) {
  std::vector<std::string> fields;
  size_t start = 0;
  while (true) {
    size_t comma = s.find(',', start);
    if (comma == std::string::npos) {
      fields.push_back(s.substr(start));
      return fields;
    }
    fields.push_back(s.substr(start, comma - start));
    start = comma + 1;
  }
}

// Empty or malformed fields give nullopt.
std::optional<double> Number(const std::string& s) {
  if (s.empty()) return std::nullopt;
  char* end = nullptr;
  double v = std::strtod(s.c_str(), &end);
  if (end != s.c_str() + s.size() || !std::isfinite(v)) return std::nullopt;
  return v;
}

// hhmmss.ss to seconds of the day.
std::optional<double> Time(const std::string& s) {
  if (s.size() < 6) return std::nullopt;
  auto v = Number(s);
  if (!v) return std::nullopt;
  int hms = static_cast<int>(*v);
  int h = hms / 10000, m = hms / 100 % 100, sec = hms % 100;
  if (h > 23 || m > 59 || sec > 60) return std::nullopt;
  return h * 3600.0 + m * 60.0 + sec + (*v - hms);
}

// ddmm.mmmm (or dddmm.mmmm) with hemisphere to signed degrees.
std::optional<double> Coordinate(const std::string& value, const std::string& hemisphere, int degree_digits) {
  if (value.size() <= static_cast<size_t>(degree_digits) + 1 || hemisphere.size() != 1) return std::nullopt;
  auto deg = Number(value.substr(0, degree_digits));
  auto min = Number(value.substr(degree_digits));
  if (!deg || !min || *min >= 60.0) return std::nullopt;
  double result = *deg + *min / 60.0;
  char h = hemisphere[0];
  if (h == 'S' || h == 'W')
    result = -result;
  else if (h != 'N' && h != 'E')
    return std::nullopt;
  return result;
}

bool ModeValid(const std::string& mode) {
  return !mode.empty() && mode[0] != 'N';
}

bool ParseGga(const std::vector<std::string>& f, GgaData& out) {
  if (f.size() < 10) return false;
  auto time = Time(f[1]);
  auto quality = Number(f[6]);
  if (!time || !quality) return false;
  out.utc_s = *time;
  out.lat_deg = Coordinate(f[2], f[3], 2);
  out.lon_deg = Coordinate(f[4], f[5], 3);
  if (out.lat_deg && std::abs(*out.lat_deg) > 90.0) out.lat_deg.reset();
  if (out.lon_deg && std::abs(*out.lon_deg) > 180.0) out.lon_deg.reset();
  out.quality = static_cast<int>(*quality);
  auto sats = Number(f[7]);
  out.satellites = sats ? static_cast<int>(*sats) : 0;
  out.hdop = Number(f[8]);
  out.altitude_m = Number(f[9]);
  if (f.size() > 13) {
    auto age = Number(f[13]);
    if (age && *age >= 0) out.correction_age_s = age;
  }
  return true;
}

bool ParseRmc(const std::vector<std::string>& f, RmcData& out) {
  if (f.size() < 9) return false;
  auto time = Time(f[1]);
  if (!time) return false;
  out.utc_s = *time;
  out.valid = f[2] == "A" && (f.size() < 13 || f[12].empty() || ModeValid(f[12]));
  auto knots = Number(f[7]);
  if (knots) out.speed_mps = *knots * kKnotsToMps;
  out.course_deg = Number(f[8]);
  return true;
}

bool ParseVtg(const std::vector<std::string>& f, VtgData& out) {
  if (f.size() < 8) return false;
  out.valid = f.size() < 10 || f[9].empty() || ModeValid(f[9]);
  out.course_deg = Number(f[1]);
  auto knots = Number(f[5]);
  if (knots) {
    out.speed_mps = *knots * kKnotsToMps;
  } else if (auto kmh = Number(f[7])) {
    out.speed_mps = *kmh / 3.6;
  }
  return true;
}

int HexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

}  // namespace

uint8_t NmeaChecksum(const std::string& body) {
  uint8_t sum = 0;
  for (char c : body) sum ^= static_cast<uint8_t>(c);
  return sum;
}

void NmeaParser::Feed(const uint8_t* data, size_t size, std::vector<NmeaMessage>& out) {
  for (size_t i = 0; i < size; i++) {
    char c = static_cast<char>(data[i]);
    if (c == '\n' || c == '\r') {
      if (!overflow_ && !line_.empty()) HandleLine(line_, out);
      line_.clear();
      overflow_ = false;
    } else if (c == '$') {
      // A new sentence starts, whatever came before was cut off.
      if (!line_.empty() && !overflow_) malformed_++;
      line_.assign(1, c);
      overflow_ = false;
    } else if (!overflow_) {
      if (line_.size() >= kMaxLine) {
        overflow_ = true;
        malformed_++;
        line_.clear();
      } else if (!line_.empty()) {
        // Bytes before the first '$' (binary frames of the receiver) are skipped.
        line_.push_back(c);
      }
    }
  }
}

void NmeaParser::HandleLine(const std::string& line, std::vector<NmeaMessage>& out) {
  size_t star = line.rfind('*');
  if (line[0] != '$' || star == std::string::npos || star + 3 != line.size()) {
    malformed_++;
    return;
  }
  int hi = HexValue(line[star + 1]), lo = HexValue(line[star + 2]);
  if (hi < 0 || lo < 0) {
    malformed_++;
    return;
  }
  std::string body = line.substr(1, star - 1);
  if (NmeaChecksum(body) != static_cast<uint8_t>(hi * 16 + lo)) {
    checksum_errors_++;
    return;
  }
  auto fields = Split(body);
  const std::string& address = fields[0];
  if (address.size() != 5) {
    ignored_++;
    return;
  }
  const std::string kind = address.substr(2);
  NmeaMessage message;
  bool ok;
  if (kind == "GGA") {
    message.type = NmeaMessage::Type::kGga;
    ok = ParseGga(fields, message.gga);
  } else if (kind == "RMC") {
    message.type = NmeaMessage::Type::kRmc;
    ok = ParseRmc(fields, message.rmc);
  } else if (kind == "VTG") {
    message.type = NmeaMessage::Type::kVtg;
    ok = ParseVtg(fields, message.vtg);
  } else if (kind == "RTK") {
    // Format assumed as the command the board takes ($GNRTK,ON / OFF), checked on the device in phase 5.
    message.type = NmeaMessage::Type::kBoardRtk;
    ok = fields.size() >= 2 && (fields[1] == "ON" || fields[1] == "OFF");
    message.board.on = ok && fields[1] == "ON";
  } else {
    ignored_++;
    return;
  }
  if (!ok) {
    malformed_++;
    return;
  }
  decoded_++;
  message.sentence = line;
  out.push_back(message);
}

}  // namespace tron
