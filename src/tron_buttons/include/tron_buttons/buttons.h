#ifndef TRON_BUTTONS_BUTTONS_H
#define TRON_BUTTONS_BUTTONS_H

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace tron {

// The three buttons on top of the Tron beside STOP: start, home and on.
enum class Key : uint8_t { kStart = 0, kHome = 1, kPower = 2 };
constexpr std::size_t kKeyCount = 3;

// A level counts once it held for this long; at a sample every 20 ms that is two samples alike.
constexpr double kKeyDebounceS = 0.04;
// Samples further apart than this lose the press under way: its length is unknown.
constexpr double kKeyMaxGapS = 0.25;

enum class KeyPress : uint8_t { kNone = 0, kShort, kHeld };

struct KeyEvent {
  Key key;
  KeyPress press;
};

// Level of one GPIO line in the text of /sys/kernel/debug/gpio: true for lo (the buttons pull their line low), none
// when the line is missing or its level is unreadable.
std::optional<bool> GpioLow(const std::string& debugfs, int gpio);

// The buttons without ROS, time passed in (monotonic seconds). A press counts only after the button was seen up, so a
// button held or stuck at the start, or after a lost sample, gives nothing until it was let go. kHeld comes once the
// press lasted hold_s, while it is still down; kShort when it is let go before. A press during which another button
// was down gives nothing, nor does one with a gap in the samples or an unreadable line.
class Keys {
 public:
  explicit Keys(double hold_s);

  // One sample of all three lines (none: unreadable). Returns the events of this sample.
  std::vector<KeyEvent> Sample(const std::array<std::optional<bool>, kKeyCount>& low, double now);
  // True once per press when a press of the key began (debounced), also one that will give nothing.
  bool Began(Key key);

 private:
  struct State {
    // raw is known since raw_since_s; false after a lost sample.
    bool valid = false;
    bool raw = false;
    double raw_since_s = 0;
    bool down = false;
    bool armed = false;
    bool spoiled = false;
    bool held = false;
    bool began = false;
    double press_start_s = 0;
  };

  void Lose(State& s);

  double hold_s_;
  std::optional<double> last_s_;
  std::array<State, kKeyCount> keys_{};
};

}  // namespace tron

#endif  // TRON_BUTTONS_BUTTONS_H
