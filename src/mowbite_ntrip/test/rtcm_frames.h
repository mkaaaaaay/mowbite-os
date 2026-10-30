#ifndef MOWBITE_NTRIP_TEST_RTCM_FRAMES_H
#define MOWBITE_NTRIP_TEST_RTCM_FRAMES_H

#include <cstdint>
#include <vector>

#include "mowbite_ntrip/rtcm.h"

namespace mowbite_ntrip {

// A frame of message number type with length payload bytes (type in the first 12 bits, then a counting pattern).
inline std::vector<uint8_t> MakeFrame(uint16_t type, size_t length) {
  std::vector<uint8_t> frame = {0xD3, static_cast<uint8_t>(length >> 8), static_cast<uint8_t>(length & 0xFF)};
  for (size_t i = 0; i < length; i++) frame.push_back(static_cast<uint8_t>(i * 7 + 3));
  if (length >= 2) {
    frame[3] = static_cast<uint8_t>(type >> 4);
    frame[4] = static_cast<uint8_t>(((type & 0x0F) << 4) | (frame[4] & 0x0F));
  }
  const uint32_t crc = Crc24q(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc >> 16));
  frame.push_back(static_cast<uint8_t>(crc >> 8));
  frame.push_back(static_cast<uint8_t>(crc));
  return frame;
}

}  // namespace mowbite_ntrip

#endif  // MOWBITE_NTRIP_TEST_RTCM_FRAMES_H
