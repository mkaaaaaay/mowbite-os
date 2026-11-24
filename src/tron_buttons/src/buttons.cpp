#include "tron_buttons/buttons.h"

#include <sstream>

namespace tron {
namespace {

// A sample that lands on a deadline up to this many seconds early still counts as on time.
constexpr double kSlackS = 1e-6;

}  // namespace

std::optional<bool> GpioLow(const std::string& debugfs, int gpio) {
  // A line reads like " gpio-52  (                    |button1             ) in  lo IRQ ".
  const std::string name = "gpio-" + std::to_string(gpio);
  std::istringstream lines(debugfs);
  std::string line;
  while (std::getline(lines, line)) {
    const auto start = line.find_first_not_of(' ');
    if (start == std::string::npos || line.compare(start, name.size(), name) != 0) {
      continue;
    }
    const auto after = start + name.size();
    if (after < line.size() && line[after] != ' ' && line[after] != '\t') {
      continue;  // gpio-5 must not match gpio-52
    }
    const auto close = line.find(')', after);
    if (close == std::string::npos) {
      return std::nullopt;
    }
    std::istringstream rest(line.substr(close + 1));
    std::string direction, level;
    rest >> direction >> level;
    if (level == "lo") {
      return true;
    }
    if (level == "hi") {
      return false;
    }
    return std::nullopt;
  }
  return std::nullopt;
}

Keys::Keys(double hold_s) : hold_s_(hold_s) {}

void Keys::Lose(State& s) {
  s.valid = false;
  s.armed = false;
  if (s.down) {
    s.spoiled = true;
  }
}

std::vector<KeyEvent> Keys::Sample(const std::array<std::optional<bool>, kKeyCount>& low, double now) {
  const bool gap = last_s_ && now - *last_s_ > kKeyMaxGapS;
  last_s_ = now;
  std::vector<KeyEvent> events;

  for (std::size_t i = 0; i < kKeyCount; ++i) {
    State& s = keys_[i];
    if (gap || !low[i]) {
      Lose(s);
    }
    if (!low[i]) {
      continue;
    }
    const bool raw = *low[i];
    if (!s.valid || raw != s.raw) {
      s.raw = raw;
      s.raw_since_s = now;
      s.valid = true;
    }
    if (now - s.raw_since_s + kSlackS < kKeyDebounceS) {
      continue;
    }
    if (raw && !s.down) {
      s.down = true;
      s.began = true;
      s.held = false;
      s.spoiled = !s.armed;
      s.press_start_s = s.raw_since_s;
    } else if (!raw) {
      if (s.down && !s.spoiled && !s.held) {
        events.push_back({static_cast<Key>(i), KeyPress::kShort});
      }
      s.down = false;
      s.armed = true;
    }
  }

  // Two buttons down at once: neither press counts.
  std::size_t down = 0;
  for (const State& s : keys_) {
    down += s.down ? 1 : 0;
  }
  for (std::size_t i = 0; i < kKeyCount; ++i) {
    State& s = keys_[i];
    if (down > 1 && s.down) {
      s.spoiled = true;
    }
    if (s.down && !s.spoiled && !s.held && now - s.press_start_s + kSlackS >= hold_s_) {
      s.held = true;
      events.push_back({static_cast<Key>(i), KeyPress::kHeld});
    }
  }
  return events;
}

bool Keys::Began(Key key) {
  State& s = keys_[static_cast<std::size_t>(key)];
  const bool began = s.began;
  s.began = false;
  return began;
}

}  // namespace tron
