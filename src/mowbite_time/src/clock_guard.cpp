#include "mowbite_time/clock_guard.h"

#include <sys/timex.h>

#include <algorithm>
#include <cmath>

namespace mowbite_time {

namespace {

constexpr double kDay = 86400.0;

// Distance of two times of day (s), across midnight.
double DayDistance(double a, double b) {
  const double d = std::fabs(std::fmod(a, kDay) - std::fmod(b, kDay));
  return std::min(d, kDay - d);
}

}  // namespace

ClockVerdict ClockGuard::Probe(const ClockProbe& probe) {
  if (probe.gnss_day) gnss_mismatch_ = DayDistance(probe.wall, *probe.gnss_day) > kClockTolerance;
  ClockVerdict verdict = Base(probe);
  if (verdict.ok && gnss_mismatch_) return {false, kGnssMismatch};
  return verdict;
}

ClockVerdict ClockGuard::Base(const ClockProbe& probe) {
  const double offset = probe.wall - probe.mono;
  if (probe.kernel_synced) {
    base_offset_ = offset;
    synced_wall_ = probe.wall;
    jumped_ = false;
    return {true, ""};
  }
  if (!base_offset_) base_offset_ = offset;
  if (std::fabs(offset - *base_offset_) > kClockTolerance) jumped_ = true;
  if (!synced_wall_ && !RtcHolds(probe.wall)) return {false, kNeverSynced};
  if (jumped_) return {false, kClockJump};
  return {true, ""};
}

bool ClockGuard::RtcHolds(double wall) const {
  return start_.rtc && start_.hctosys && start_.synced_at && wall >= *start_.synced_at &&
         wall <= *start_.synced_at + kRtcTrust;
}

bool ClockSynchronized(int state, int status) {
  return state != -1 && state != TIME_ERROR && (status & STA_UNSYNC) == 0;
}

bool SystemClockSynchronized() {
  struct timex tx {};
  tx.modes = 0;
  const int state = adjtimex(&tx);
  return ClockSynchronized(state, tx.status);
}

}  // namespace mowbite_time
