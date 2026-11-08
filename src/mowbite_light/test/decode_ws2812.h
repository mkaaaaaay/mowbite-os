#ifndef MOWBITE_LIGHT_TEST_DECODE_WS2812_H
#define MOWBITE_LIGHT_TEST_DECODE_WS2812_H

#include <cstdint>
#include <vector>

#include "mowbite_light/ws2812.h"

namespace mowbite_light {

// The colours of a frame of EncodeWs2812; empty for bytes that are no frame.
inline std::vector<Rgb> DecodeWs2812(const std::vector<uint8_t>& bytes) {
  constexpr size_t kBytesPerLed = 24;
  auto take = [&bytes](size_t at, uint8_t* value) {
    *value = 0;
    for (size_t i = 0; i < 8; ++i) {
      const uint8_t b = bytes[at + i];
      if (b != 0xFC && b != 0xC0) return false;
      *value = static_cast<uint8_t>(*value << 1 | (b == 0xFC ? 1 : 0));
    }
    return true;
  };
  if (bytes.size() < 2 * kResetBytes || (bytes.size() - 2 * kResetBytes) % kBytesPerLed != 0) return {};
  for (size_t i = 0; i < kResetBytes; ++i) {
    if (bytes[i] != 0 || bytes[bytes.size() - 1 - i] != 0) return {};
  }
  std::vector<Rgb> leds;
  for (size_t at = kResetBytes; at + kResetBytes < bytes.size(); at += kBytesPerLed) {
    Rgb c;
    if (!take(at, &c.g) || !take(at + 8, &c.r) || !take(at + 16, &c.b)) return {};
    leds.push_back(c);
  }
  return leds;
}

}  // namespace mowbite_light

#endif  // MOWBITE_LIGHT_TEST_DECODE_WS2812_H
