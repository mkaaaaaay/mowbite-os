#include "mowbite_serial/recognize.h"

#include <algorithm>
#include <cstdlib>
#include <set>

namespace mowbite_serial {

namespace {

constexpr size_t kNmeaMax = 200;
constexpr size_t kWitSize = 11;
constexpr size_t kRtcmHead = 3;
constexpr size_t kRtcmCrc = 3;
// Frames of a kind before it counts: chance alone makes one now and then out of bytes at a wrong rate.
constexpr int kNmeaEnough = 2;
constexpr int kWitEnough = 5;
constexpr int kRtcmEnough = 2;

int Hex(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Length of a valid NMEA line at data[0] up to its line end, 0 for none.
size_t NmeaLine(const uint8_t* data, size_t size) {
  if (data[0] != '$') return 0;
  uint8_t sum = 0;
  size_t i = 1;
  for (; i < size && i < kNmeaMax && data[i] != '*'; i++) {
    if (data[i] < 0x20 || data[i] > 0x7e) return 0;
    sum ^= data[i];
  }
  if (i + 3 > size || data[i] != '*') return 0;
  const int high = Hex(data[i + 1]);
  const int low = Hex(data[i + 2]);
  if (high < 0 || low < 0 || ((high << 4) | low) != sum) return 0;
  i += 3;
  if (i < size && data[i] == '\r') i++;
  if (i >= size || data[i] != '\n') return 0;
  return i + 1;
}

size_t WitFrame(const uint8_t* data, size_t size) {
  if (size < kWitSize || data[0] != 0x55 || (data[1] & 0xf0) != 0x50) return 0;
  uint8_t sum = 0;
  for (size_t i = 0; i < kWitSize - 1; i++) sum = static_cast<uint8_t>(sum + data[i]);
  return sum == data[kWitSize - 1] ? kWitSize : 0;
}

uint32_t Crc24q(const uint8_t* data, size_t size) {
  uint32_t crc = 0;
  for (size_t i = 0; i < size; i++) {
    crc ^= static_cast<uint32_t>(data[i]) << 16;
    for (int bit = 0; bit < 8; bit++) {
      crc <<= 1;
      if (crc & 0x1000000) crc ^= 0x1864cfb;
    }
  }
  return crc & 0xffffff;
}

size_t RtcmFrame(const uint8_t* data, size_t size) {
  if (size < kRtcmHead || data[0] != 0xd3 || (data[1] & 0xfc) != 0) return 0;
  const size_t length = (static_cast<size_t>(data[1] & 0x03) << 8) | data[2];
  const size_t total = kRtcmHead + length + kRtcmCrc;
  if (length < 2 || size < total) return 0;
  const uint32_t crc = (static_cast<uint32_t>(data[total - 3]) << 16) | (static_cast<uint32_t>(data[total - 2]) << 8) |
                       data[total - 1];
  return Crc24q(data, kRtcmHead + length) == crc ? total : 0;
}

// The field of an NMEA line after `index` commas, without the checksum.
std::string Field(const std::string& line, int index) {
  size_t start = 0;
  for (int i = 0; i < index; i++) {
    start = line.find(',', start);
    if (start == std::string::npos) return "";
    start++;
  }
  return line.substr(start, line.find_first_of(",*", start) - start);
}

}  // namespace

const char* KindName(Kind kind) {
  switch (kind) {
    case Kind::kSilent: return "silent";
    case Kind::kUnknown: return "unknown";
    case Kind::kNmea: return "nmea";
    case Kind::kWit: return "wit";
    case Kind::kRtcm: return "rtcm";
  }
  return "unknown";
}

void Recognizer::Feed(const uint8_t* data, size_t size) {
  bytes_ += size;
  const size_t take = std::min(size, kKeep - kept_.size());
  kept_.insert(kept_.end(), data, data + take);
}

Heard Recognizer::Result(double seconds) const {
  Heard heard;
  heard.bytes = bytes_;
  heard.seconds = seconds;
  std::set<std::string> talkers;
  std::set<std::string> sentences;
  std::set<int> wit_types;
  std::set<int> rtcm_types;
  size_t nmea_bytes = 0;
  size_t wit_bytes = 0;
  size_t rtcm_bytes = 0;
  int gga = 0;
  const uint8_t* data = kept_.data();
  const size_t size = kept_.size();
  for (size_t at = 0; at < size;) {
    const size_t left = size - at;
    if (const size_t n = NmeaLine(data + at, left)) {
      const std::string line(reinterpret_cast<const char*>(data + at + 1), n - 1);
      const std::string address = line.substr(0, line.find_first_of(",*"));
      const bool proprietary = !address.empty() && address[0] == 'P';
      const size_t split = proprietary ? 1 : std::min<size_t>(2, address.size());
      talkers.insert(address.substr(0, split));
      sentences.insert(address.substr(split));
      if (!proprietary && address.size() == 5 && address.compare(2, 3, "GGA") == 0) {
        gga++;
        const std::string quality = Field(line, 6);
        heard.fix = quality.empty() ? 0 : std::atoi(quality.c_str());
      }
      heard.nmea_lines++;
      nmea_bytes += n;
      at += n;
    } else if (const size_t w = WitFrame(data + at, left)) {
      heard.wit_frames++;
      wit_types.insert(data[at + 1]);
      wit_bytes += w;
      at += w;
    } else if (const size_t r = RtcmFrame(data + at, left)) {
      heard.rtcm_frames++;
      rtcm_types.insert((data[at + 3] << 4) | (data[at + 4] >> 4));
      rtcm_bytes += r;
      at += r;
    } else {
      at++;
    }
  }
  heard.talkers.assign(talkers.begin(), talkers.end());
  heard.sentences.assign(sentences.begin(), sentences.end());
  heard.wit_types.assign(wit_types.begin(), wit_types.end());
  heard.rtcm_types.assign(rtcm_types.begin(), rtcm_types.end());
  if (seconds > 0) heard.gga_hz = gga / seconds;

  if (size == 0) return heard;
  heard.kind = Kind::kUnknown;
  struct Candidate {
    Kind kind;
    size_t bytes;
    bool enough;
  };
  const Candidate candidates[] = {{Kind::kNmea, nmea_bytes, heard.nmea_lines >= kNmeaEnough},
                                  {Kind::kWit, wit_bytes, heard.wit_frames >= kWitEnough},
                                  {Kind::kRtcm, rtcm_bytes, heard.rtcm_frames >= kRtcmEnough}};
  // A receiver may send NMEA and RTCM together: what was recognized counts as a whole, the largest part names it.
  if ((nmea_bytes + wit_bytes + rtcm_bytes) * 2 < size) return heard;
  size_t best = 0;
  for (const Candidate& candidate : candidates) {
    if (candidate.enough && candidate.bytes > best) {
      best = candidate.bytes;
      heard.kind = candidate.kind;
    }
  }
  return heard;
}

}  // namespace mowbite_serial
