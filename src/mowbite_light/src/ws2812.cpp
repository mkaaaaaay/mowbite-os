#include "mowbite_light/ws2812.h"

#include <algorithm>
#include <cmath>

namespace mowbite_light {

namespace {

constexpr uint8_t kOne = 0xFC;
constexpr uint8_t kZero = 0xC0;
constexpr size_t kBytesPerLed = 24;

void Put(uint8_t value, std::vector<uint8_t>* out) {
  for (int bit = 7; bit >= 0; --bit) out->push_back((value >> bit) & 1 ? kOne : kZero);
}

}  // namespace

std::vector<uint8_t> EncodeWs2812(const std::vector<Rgb>& leds, size_t buffer_leds) {
  std::vector<uint8_t> out;
  out.reserve(2 * kResetBytes + buffer_leds * kBytesPerLed);
  out.assign(kResetBytes, 0);
  for (size_t i = 0; i < buffer_leds; ++i) {
    const Rgb c = i < leds.size() ? leds[i] : Rgb{};
    Put(c.g, &out);
    Put(c.r, &out);
    Put(c.b, &out);
  }
  out.insert(out.end(), kResetBytes, 0);
  return out;
}

void LimitCurrent(std::vector<Rgb>* leds, double max_current_ma, double ma_per_channel) {
  double sum = 0;
  for (const auto& c : *leds) sum += c.r + c.g + c.b;
  const double current = sum / 255.0 * ma_per_channel;
  if (current <= max_current_ma) return;
  const double scale = std::max(0.0, max_current_ma) / current;
  for (auto& c : *leds) {
    c.r = static_cast<uint8_t>(std::floor(c.r * scale));
    c.g = static_cast<uint8_t>(std::floor(c.g * scale));
    c.b = static_cast<uint8_t>(std::floor(c.b * scale));
  }
}

}  // namespace mowbite_light
