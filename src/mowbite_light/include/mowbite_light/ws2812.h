#ifndef MOWBITE_LIGHT_WS2812_H
#define MOWBITE_LIGHT_WS2812_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mowbite_light {

struct Rgb {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  bool operator==(const Rgb& o) const { return r == o.r && g == o.g && b == o.b; }
  bool operator!=(const Rgb& o) const { return !(*this == o); }
};

// Zero bytes before and after the colours: the reset of the chain.
constexpr size_t kResetBytes = 100;

// The SPI bytes of a WS2812 chain clocked at 8 MHz: green, red and blue per LED, every colour bit one byte (0xFC for 1,
// 0xC0 for 0, most significant first), buffer_leds LEDs with those beyond the given ones dark, a reset before and after.
std::vector<uint8_t> EncodeWs2812(const std::vector<Rgb>& leds, size_t buffer_leds);

// Dims all colours alike so that the chain draws at most max_current_ma, at ma_per_channel for a channel at 255.
void LimitCurrent(std::vector<Rgb>* leds, double max_current_ma, double ma_per_channel);

}  // namespace mowbite_light

#endif  // MOWBITE_LIGHT_WS2812_H
