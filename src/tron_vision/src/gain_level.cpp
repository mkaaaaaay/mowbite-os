#include "tron_vision/gain_level.h"

#include <algorithm>
#include <cstddef>
#include <cstdio>

namespace tron_vision {
namespace {

constexpr const char* kNames[] = {"day", "dusk", "night"};

GainLevel Step(GainLevel level, int by) {
  return static_cast<GainLevel>(std::clamp(static_cast<int>(level) + by, 0, 2));
}

}  // namespace

const char* GainLevelName(GainLevel level) { return kNames[static_cast<int>(level)]; }

bool GainLevelFromName(const std::string& name, GainLevel& level) {
  for (int i = 0; i < 3; ++i) {
    if (name == kNames[i]) {
      level = static_cast<GainLevel>(i);
      return true;
    }
  }
  return false;
}

float MeanLumaLeft(const uint8_t* yuyv, uint32_t width, uint32_t height, uint32_t line_bytes, uint32_t step) {
  if (yuyv == nullptr || step == 0) return 0.0F;
  uint64_t sum = 0;
  uint64_t count = 0;
  for (uint32_t y = 0; y < height; y += step) {
    const uint8_t* line = yuyv + static_cast<size_t>(y) * line_bytes;
    for (uint32_t x = 0; x < width / 2; x += step) {
      sum += line[2 * x];
      ++count;
    }
  }
  return count == 0 ? 0.0F : static_cast<float>(static_cast<double>(sum) / static_cast<double>(count));
}

GainWishState NextGainWish(const GainWishState& state, double now_s, float luma, const GainWishLimits& limits) {
  GainWishState next = state;
  const bool resumed = !state.started || now_s - state.last_at > limits.gap_s || now_s < state.last_at;
  next.started = true;
  next.last_at = now_s;
  if (resumed) {
    // The time on the level before the gap counts; the light may be another one by now.
    next.dark_since = next.bright_since = -1.0;
    if (!state.started) next.changed_at = now_s - limits.settle_s;
  }
  auto change = [&](int by, bool trying) {
    next.level = Step(next.level, by);
    next.changed_at = now_s;
    next.dark_since = next.bright_since = -1.0;
    next.trying_down = trying;
  };
  if (now_s - next.changed_at < limits.settle_s) {
    next.dark_since = next.bright_since = -1.0;
    return next;
  }
  if (next.trying_down && now_s - next.changed_at >= limits.try_hold_s) {
    next.trying_down = false;
    next.try_down_wait = 0.0;
  }
  const double wait = next.try_down_wait > 0.0 ? next.try_down_wait : limits.try_down_s;
  if (luma < limits.dark_below) {
    next.bright_since = -1.0;
    if (next.dark_since < 0.0) next.dark_since = now_s;
    if (now_s - next.dark_since >= limits.dark_s && next.level != GainLevel::kNight) {
      if (next.trying_down) next.try_down_wait = std::min(2.0 * wait, limits.try_down_max_s);
      change(+1, false);
    }
    return next;
  }
  next.dark_since = -1.0;
  if (luma > limits.bright_above) {
    if (next.bright_since < 0.0) next.bright_since = now_s;
    if (now_s - next.bright_since >= limits.bright_s && next.level != GainLevel::kDay) {
      next.try_down_wait = 0.0;
      change(-1, false);
    }
    return next;
  }
  next.bright_since = -1.0;
  if (next.level != GainLevel::kDay && luma >= limits.reserve_above && now_s - next.changed_at >= wait) {
    change(-1, true);
  }
  return next;
}

std::string GainWishLine(GainLevel level, double monotonic_s, double unix_s) {
  char text[96];
  std::snprintf(text, sizeof(text), "%s %.3f %.3f\n", GainLevelName(level), monotonic_s, unix_s);
  return text;
}

bool GainLevelFromLine(const std::string& line, GainLevel& level) {
  return GainLevelFromName(line.substr(0, line.find(' ')), level);
}

}  // namespace tron_vision
