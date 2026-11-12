#ifndef MOWBITE_TIME_CLOCK_GUARD_H
#define MOWBITE_TIME_CLOCK_GUARD_H

#include <optional>
#include <string>

namespace mowbite_time {

// Change of the wall clock against the monotonic clock (s) that counts as a jump, and the largest difference to the
// time of day of the GNSS receiver.
constexpr double kClockTolerance = 60.0;
// How long after the last synchronization the system clock from the real-time clock is trusted alone (s).
constexpr double kRtcTrust = 30 * 86400.0;

// Reasons of an unsafe clock.
constexpr char kNeverSynced[] = "never_synced";
constexpr char kClockJump[] = "clock_jump";
constexpr char kGnssMismatch[] = "gnss_mismatch";

// Known at the start: whether the platform has a real-time clock that keeps time (platform value clock.rtc), whether
// the kernel set the system clock from it at boot (hctosys) and the wall time of the last synchronization (clock.json).
struct ClockStart {
  bool rtc = false;
  bool hctosys = false;
  std::optional<double> synced_at;
};

// One look at the clocks (s): monotonic, wall (unix), whether a time service set the clock lately (ClockSynchronized),
// and the time of day (UTC) of the GNSS receiver while its fix is fresh.
struct ClockProbe {
  double mono = 0;
  double wall = 0;
  bool kernel_synced = false;
  std::optional<double> gnss_day;
};

struct ClockVerdict {
  bool ok = false;
  std::string reason;  // empty while ok
  bool operator==(const ClockVerdict& other) const { return ok == other.ok && reason == other.reason; }
  bool operator!=(const ClockVerdict& other) const { return !(*this == other); }
};

// Whether the wall clock can be trusted for date, weekday and sun. Safe while a time service set the clock lately (a
// jump then is the correction itself); after a synchronization while the clock has not jumped since; never synchronized
// since the start only with clock.rtc, when the kernel took the clock from the real-time clock, it lies within
// kRtcTrust after synced_at and has not jumped. A GNSS time of day off by more than kClockTolerance makes each of them
// unsafe until the two agree again.
class ClockGuard {
 public:
  explicit ClockGuard(const ClockStart& start) : start_(start) {}

  ClockVerdict Probe(const ClockProbe& probe);
  // Wall time of the last synchronization since the start.
  std::optional<double> SyncedAt() const { return synced_wall_; }

 private:
  ClockVerdict Base(const ClockProbe& probe);
  bool RtcHolds(double wall) const;

  const ClockStart start_;
  std::optional<double> base_offset_;  // wall - mono at the last synchronization, before one at the first probe
  std::optional<double> synced_wall_;
  bool jumped_ = false;
  bool gnss_mismatch_ = false;
};

// Whether the system clock was set by a time service lately, from what adjtimex returns: no error (-1) and a maximum
// error (us) below the 16 s the kernel starts with, sets on every manual setting and grows to within about 9 h
// without a service. STA_UNSYNC and TIME_ERROR do not count: timesyncd sets them on purpose where the real-time clock
// keeps local time.
bool ClockSynchronized(int state, long maxerror_us);
// The same for this machine; adjtimex only reads (modes 0), which needs no rights.
bool SystemClockSynchronized();

}  // namespace mowbite_time

#endif  // MOWBITE_TIME_CLOCK_GUARD_H
