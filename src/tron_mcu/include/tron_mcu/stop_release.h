#ifndef TRON_MCU_STOP_RELEASE_H
#define TRON_MCU_STOP_RELEASE_H

#include <cstdint>
#include <optional>

namespace tron {

// A tick that lands on a deadline up to this many seconds early still counts as on time.
constexpr double kDeadlineSlackS = 1e-6;

// Timing of the release with the STOP button (G37). STOP is free for kStopReleaseFreeS before the press, held for
// kStopReleaseHoldS and let go before kStopReleaseMaxHoldS. Free means stop 0 for at least kStopDebounceS, so a bounce
// does not split a press.
constexpr double kStopReleaseFreeS = 2.0;
constexpr double kStopReleaseHoldS = 3.0;
constexpr double kStopReleaseMaxHoldS = 10.0;
constexpr double kStopDebounceS = 0.1;

// Numbered like ButtonEvent of mowbite_msgs.
enum class StopPress : uint8_t { kNone = 0, kHeld = 1, kLong = 2, kCanceled = 3 };

struct StopReleaseInput {
  // The emergency was latched before the sample of this tick.
  bool latched = false;
  // Fresh telemetry, and a fresh IMU where the driver forwards one.
  bool usable = false;
  bool lift = false;
  // Another reason latched since the last update, not counting the start of this press.
  bool new_reason = false;
};

// The STOP gesture without ROS: it only recognises the press, the release is up to the logic. Time is passed in
// (monotonic seconds). A press counts only while the emergency was latched before it began (the press that latched it
// never releases), after STOP was free for kStopReleaseFreeS with usable telemetry and an unchanged lift. Telemetry or
// IMU going stale, lift changing, a new reason or a reopened port end it and restart the free time.
class StopRelease {
 public:
  // Debounces the STOP sample of a tick; true when it begins a press.
  bool Sample(bool stop, double now);
  // After Sample of the same tick: kHeld once a counting press was held for kStopReleaseHoldS, kLong kStopDebounceS
  // after a held press was let go before kStopReleaseMaxHoldS and still latched, kCanceled when it ends otherwise.
  StopPress Update(const StopReleaseInput& input, double now);
  // The port reopened: the next update ends a press and restarts the free time.
  void Interrupt();

 private:
  enum class Phase { kNone, kCounting, kHeld, kSpent };

  StopPress End();

  // Debounced button.
  bool pressed_ = false;
  bool stop_ = false;
  double press_start_s_ = 0;
  std::optional<double> zero_since_s_;
  // Set by Sample for the next Update: a press began, or one ended (the time of its first 0 sample).
  bool began_ = false;
  std::optional<double> released_s_;

  Phase phase_ = Phase::kNone;
  std::optional<double> free_since_s_;
  std::optional<bool> lift_;
  bool interrupted_ = false;
};

}  // namespace tron

#endif  // TRON_MCU_STOP_RELEASE_H
