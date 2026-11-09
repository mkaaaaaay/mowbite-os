#ifndef TRON_VISION_H264_RTP_H
#define TRON_VISION_H264_RTP_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace tron_vision {

constexpr uint8_t kNalIdr = 5;
constexpr uint8_t kNalSps = 7;
constexpr uint8_t kNalPps = 8;

// One NAL unit inside a buffer, without its start code.
struct NalUnit {
  const uint8_t* data = nullptr;
  size_t size = 0;
  uint8_t Type() const {
    return data[0] & 0x1F;
  }
};

// NAL units of an Annex B byte stream (start codes 00 00 01 and 00 00 00 01); empty ones are left out.
std::vector<NalUnit> SplitAnnexB(const uint8_t* data, size_t size);

struct ParameterSets {
  std::vector<uint8_t> sps;
  std::vector<uint8_t> pps;
};

// SPS and PPS out of an Annex B block; false unless both are there.
bool FindParameterSets(const uint8_t* data, size_t size, ParameterSets& sets);

// The three bytes after the SPS header as six hex digits ("42e01f"); empty for a shorter SPS.
std::string ProfileLevelId(const std::vector<uint8_t>& sps);

std::string Base64(const uint8_t* data, size_t size);

// RTP timestamps at 90 kHz from capture times in microseconds. Every call returns more than the one before, also
// when the capture time stands still or steps back.
class RtpClock {
 public:
  explicit RtpClock(uint32_t start) : start_(start) {}
  uint32_t Next(int64_t timestamp_us);

 private:
  uint32_t start_;
  bool first_ = true;
  int64_t origin_us_ = 0;
  int64_t last_ticks_ = 0;
};

// H.264 over RTP after RFC 6184 in packetization mode 1 without aggregation: a NAL unit up to the payload limit
// goes out alone, a larger one as FU-A fragments.
class RtpPacketizer {
 public:
  static constexpr uint8_t kPayloadType = 96;
  static constexpr size_t kHeaderBytes = 12;

  RtpPacketizer(uint32_t ssrc, uint16_t first_sequence, size_t max_payload = 1188);

  // The packets of one access unit: all with `timestamp`, the last with the marker bit.
  void Packetize(const std::vector<NalUnit>& nals, uint32_t timestamp, std::vector<std::vector<uint8_t>>& packets);

  uint32_t Ssrc() const {
    return ssrc_;
  }
  // Sequence number of the next packet.
  uint16_t Sequence() const {
    return sequence_;
  }

 private:
  std::vector<uint8_t>& Begin(std::vector<std::vector<uint8_t>>& packets, size_t& used, uint32_t timestamp,
                              size_t payload);

  uint32_t ssrc_;
  uint16_t sequence_;
  size_t max_payload_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_H264_RTP_H
