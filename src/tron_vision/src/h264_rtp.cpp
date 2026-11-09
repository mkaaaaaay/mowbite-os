#include "tron_vision/h264_rtp.h"

#include <algorithm>
#include <cstdio>

namespace tron_vision {

namespace {

constexpr uint8_t kFuA = 28;
constexpr uint8_t kFuStart = 0x80;
constexpr uint8_t kFuEnd = 0x40;
constexpr uint8_t kMarker = 0x80;

}  // namespace

std::vector<NalUnit> SplitAnnexB(const uint8_t* data, size_t size) {
  std::vector<NalUnit> nals;
  // Position after the start code that opened the current unit; `size` while none is open.
  size_t begin = size;
  size_t i = 0;
  auto close = [&](size_t end) {
    // Zero bytes before a start code belong to it, not to the unit.
    while (end > begin && data[end - 1] == 0) end--;
    if (begin < end) nals.push_back({data + begin, end - begin});
  };
  while (i + 3 <= size) {
    if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
      if (begin < size) close(i);
      begin = i + 3;
      i += 3;
    } else {
      i++;
    }
  }
  if (begin < size) close(size);
  return nals;
}

bool FindParameterSets(const uint8_t* data, size_t size, ParameterSets& sets) {
  sets.sps.clear();
  sets.pps.clear();
  for (const NalUnit& nal : SplitAnnexB(data, size)) {
    if (nal.Type() == kNalSps && sets.sps.empty()) sets.sps.assign(nal.data, nal.data + nal.size);
    if (nal.Type() == kNalPps && sets.pps.empty()) sets.pps.assign(nal.data, nal.data + nal.size);
  }
  return !sets.sps.empty() && !sets.pps.empty();
}

std::string ProfileLevelId(const std::vector<uint8_t>& sps) {
  if (sps.size() < 4) return "";
  char text[8];
  std::snprintf(text, sizeof(text), "%02x%02x%02x", sps[1], sps[2], sps[3]);
  return text;
}

std::string Base64(const uint8_t* data, size_t size) {
  static const char kDigits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((size + 2) / 3 * 4);
  for (size_t i = 0; i < size; i += 3) {
    const size_t left = size - i;
    const uint32_t group = static_cast<uint32_t>(data[i]) << 16 | (left > 1 ? static_cast<uint32_t>(data[i + 1]) << 8 : 0U) |
                           (left > 2 ? static_cast<uint32_t>(data[i + 2]) : 0U);
    out += kDigits[group >> 18];
    out += kDigits[(group >> 12) & 0x3F];
    out += left > 1 ? kDigits[(group >> 6) & 0x3F] : '=';
    out += left > 2 ? kDigits[group & 0x3F] : '=';
  }
  return out;
}

uint32_t RtpClock::Next(int64_t timestamp_us) {
  if (first_) {
    first_ = false;
    origin_us_ = timestamp_us;
    last_ticks_ = 0;
    return start_;
  }
  const int64_t ticks = (timestamp_us - origin_us_) * 9 / 100;
  last_ticks_ = std::max(ticks, last_ticks_ + 1);
  return start_ + static_cast<uint32_t>(last_ticks_);
}

RtpPacketizer::RtpPacketizer(uint32_t ssrc, uint16_t first_sequence, size_t max_payload)
    : ssrc_(ssrc), sequence_(first_sequence), max_payload_(std::max<size_t>(max_payload, 3)) {}

std::vector<uint8_t>& RtpPacketizer::Begin(std::vector<std::vector<uint8_t>>& packets, size_t& used,
                                           uint32_t timestamp, size_t payload) {
  if (used == packets.size()) packets.emplace_back();
  std::vector<uint8_t>& p = packets[used++];
  p.clear();
  p.reserve(kHeaderBytes + payload);
  p.push_back(0x80);
  p.push_back(kPayloadType);
  p.push_back(static_cast<uint8_t>(sequence_ >> 8));
  p.push_back(static_cast<uint8_t>(sequence_));
  sequence_++;
  for (int shift = 24; shift >= 0; shift -= 8) p.push_back(static_cast<uint8_t>(timestamp >> shift));
  for (int shift = 24; shift >= 0; shift -= 8) p.push_back(static_cast<uint8_t>(ssrc_ >> shift));
  return p;
}

void RtpPacketizer::Packetize(const std::vector<NalUnit>& nals, uint32_t timestamp,
                              std::vector<std::vector<uint8_t>>& packets) {
  // Packets of earlier calls keep their memory.
  size_t used = 0;
  for (const NalUnit& nal : nals) {
    if (nal.size <= max_payload_) {
      std::vector<uint8_t>& p = Begin(packets, used, timestamp, nal.size);
      p.insert(p.end(), nal.data, nal.data + nal.size);
      continue;
    }
    const uint8_t indicator = static_cast<uint8_t>((nal.data[0] & 0xE0) | kFuA);
    const size_t chunk = max_payload_ - 2;
    for (size_t at = 1; at < nal.size; at += chunk) {
      const size_t n = std::min(chunk, nal.size - at);
      std::vector<uint8_t>& p = Begin(packets, used, timestamp, n + 2);
      p.push_back(indicator);
      p.push_back(static_cast<uint8_t>((at == 1 ? kFuStart : 0) | (at + n == nal.size ? kFuEnd : 0) | nal.Type()));
      p.insert(p.end(), nal.data + at, nal.data + at + n);
    }
  }
  packets.resize(used);
  if (used > 0) packets[used - 1][1] |= kMarker;
}

}  // namespace tron_vision
