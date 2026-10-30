#ifndef MOWBITE_NTRIP_RTCM_H
#define MOWBITE_NTRIP_RTCM_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mowbite_ntrip {

// CRC-24Q of RTCM 3 (polynomial 0x1864CFB, start 0).
uint32_t Crc24q(const uint8_t* data, size_t size);

// Message number of a whole frame (first 12 bits after the header), 0 if the frame is too short.
uint16_t RtcmMessageType(const std::vector<uint8_t>& frame);

// Cuts a byte stream into RTCM 3 frames (0xD3, 6 zero bits, 10 bit length, payload, CRC-24Q). Only frames with a valid
// CRC come out; bytes outside of frames are skipped and counted, after a CRC error the search goes on one byte later.
class RtcmFramer {
 public:
  static constexpr size_t kMaxPayload = 1023;

  // Appends the frames completed by these bytes to frames.
  void Feed(const uint8_t* data, size_t size, std::vector<std::vector<uint8_t>>& frames);
  void Reset();

  uint64_t frames() const {
    return frames_;
  }
  uint64_t crc_errors() const {
    return crc_errors_;
  }
  uint64_t skipped() const {
    return skipped_;
  }

 private:
  std::vector<uint8_t> buffer_;
  uint64_t frames_ = 0;
  uint64_t crc_errors_ = 0;
  uint64_t skipped_ = 0;
};

}  // namespace mowbite_ntrip

#endif  // MOWBITE_NTRIP_RTCM_H
