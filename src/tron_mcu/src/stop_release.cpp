#include "tron_mcu/stop_release.h"

namespace tron {

bool StopRelease::Sample(bool stop, double now) {
  stop_ = stop;
  if (stop) {
    zero_since_s_.reset();
    if (pressed_) return false;
    pressed_ = true;
    press_start_s_ = now;
    began_ = true;
    return true;
  }
  if (!pressed_) return false;
  if (!zero_since_s_) zero_since_s_ = now;
  if (now - *zero_since_s_ + kDeadlineSlackS >= kStopDebounceS) {
    pressed_ = false;
    released_s_ = zero_since_s_;
    zero_since_s_.reset();
  }
  return false;
}

StopPress StopRelease::Update(const StopReleaseInput& input, double now) {
  const bool lift_rose = lift_ && !*lift_ && input.lift;
  lift_ = input.lift;
  const bool fault = !input.usable || lift_rose || input.new_reason || interrupted_;
  interrupted_ = false;
  const bool began = began_;
  began_ = false;
  const std::optional<double> released = released_s_;
  released_s_.reset();

  StopPress event = StopPress::kNone;
  if (began) {
    const bool free = free_since_s_ && press_start_s_ - *free_since_s_ + kDeadlineSlackS >= kStopReleaseFreeS;
    phase_ = input.latched && !fault && free ? Phase::kCounting : Phase::kNone;
  } else if (phase_ == Phase::kSpent) {
    if (released) phase_ = Phase::kNone;
  } else if (phase_ != Phase::kNone) {
    if (fault) {
      event = End();
    } else if (released) {
      // Let go before kStopReleaseMaxHoldS, also when the debounce ends after it.
      const bool in_time = *released - press_start_s_ + kDeadlineSlackS < kStopReleaseMaxHoldS;
      event = phase_ == Phase::kHeld && in_time && input.latched ? StopPress::kLong : End();
      phase_ = Phase::kNone;
    } else if (stop_ && now - press_start_s_ + kDeadlineSlackS >= kStopReleaseMaxHoldS) {
      // Held too long: letting go does nothing, a new try starts with the free time.
      event = End();
      phase_ = Phase::kSpent;
    } else if (phase_ == Phase::kCounting && stop_ && now - press_start_s_ + kDeadlineSlackS >= kStopReleaseHoldS) {
      phase_ = Phase::kHeld;
      event = StopPress::kHeld;
    }
  }

  if (fault || pressed_) {
    free_since_s_.reset();
  } else if (!free_since_s_) {
    free_since_s_ = released ? *released : now;
  }
  return event;
}

void StopRelease::Interrupt() {
  interrupted_ = true;
}

// Ends a counting press without a release; only one that was reported as held is reported as canceled.
StopPress StopRelease::End() {
  const bool held = phase_ == Phase::kHeld;
  phase_ = Phase::kNone;
  return held ? StopPress::kCanceled : StopPress::kNone;
}

}  // namespace tron
