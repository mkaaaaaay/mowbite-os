#include <gtest/gtest.h>
#include <sys/timex.h>

#include <cmath>

#include "mowbite_time/clock_guard.h"

using namespace mowbite_time;

namespace {

constexpr double kDay = 86400.0;
// 2026-11-12 18:00 UTC.
constexpr double kNow = 1794506400.0;
const ClockVerdict kOk{true, ""};

ClockProbe At(double mono, double wall, bool synced, std::optional<double> gnss_day = std::nullopt) {
  ClockProbe probe;
  probe.mono = mono;
  probe.wall = wall;
  probe.kernel_synced = synced;
  probe.gnss_day = gnss_day;
  return probe;
}

ClockStart Rtc(bool rtc, bool hctosys, std::optional<double> synced_at) {
  ClockStart start;
  start.rtc = rtc;
  start.hctosys = hctosys;
  start.synced_at = synced_at;
  return start;
}

}  // namespace

TEST(ClockGuard, SafeWhileTheKernelIsSynchronizedAndAJumpThenIsTheCorrection) {
  ClockGuard guard({});
  EXPECT_EQ(guard.Probe(At(10, kNow, true)), kOk);
  EXPECT_EQ(guard.Probe(At(11, kNow + 3600, true)), kOk);
  EXPECT_EQ(*guard.SyncedAt(), kNow + 3600);
}

TEST(ClockGuard, NeverSynchronizedWithoutTheRealTimeClock) {
  ClockGuard guard({});
  EXPECT_EQ(guard.Probe(At(10, kNow, false)), (ClockVerdict{false, kNeverSynced}));
  EXPECT_FALSE(guard.SyncedAt());
  EXPECT_EQ(guard.Probe(At(20, kNow + 10, true)), kOk);
}

TEST(ClockGuard, AfterASynchronizationSafeUntilTheClockJumps) {
  ClockGuard guard({});
  EXPECT_EQ(guard.Probe(At(10, kNow, true)), kOk);
  // Unsynchronized hours later, a little drift.
  EXPECT_EQ(guard.Probe(At(10 + 5 * 3600, kNow + 5 * 3600 + 30, false)), kOk);
  EXPECT_EQ(guard.Probe(At(10 + 5 * 3600 + 1, kNow + 5 * 3600 + 1 + 90, false)), (ClockVerdict{false, kClockJump}));
  // Set back by hand it stays unsafe, until the kernel is synchronized again.
  EXPECT_EQ(guard.Probe(At(10 + 5 * 3600 + 2, kNow + 5 * 3600 + 2, false)), (ClockVerdict{false, kClockJump}));
  EXPECT_EQ(guard.Probe(At(10 + 5 * 3600 + 3, kNow + 5 * 3600 + 3, true)), kOk);
}

TEST(ClockGuard, TheRealTimeClockOnlyWithThePlatformValueAndHctosys) {
  const double synced_at = kNow - 3 * kDay;
  EXPECT_EQ(ClockGuard(Rtc(false, true, synced_at)).Probe(At(10, kNow, false)), (ClockVerdict{false, kNeverSynced}));
  EXPECT_EQ(ClockGuard(Rtc(true, false, synced_at)).Probe(At(10, kNow, false)), (ClockVerdict{false, kNeverSynced}));
  EXPECT_EQ(ClockGuard(Rtc(true, true, std::nullopt)).Probe(At(10, kNow, false)),
            (ClockVerdict{false, kNeverSynced}));
  EXPECT_EQ(ClockGuard(Rtc(true, true, synced_at)).Probe(At(10, kNow, false)), kOk);
}

TEST(ClockGuard, TheRealTimeClockNotBeforeTheLastSynchronizationNorLongAfter) {
  const ClockStart start = Rtc(true, true, kNow);
  EXPECT_EQ(ClockGuard(start).Probe(At(10, kNow - 1, false)), (ClockVerdict{false, kNeverSynced}));
  EXPECT_EQ(ClockGuard(start).Probe(At(10, kNow + kRtcTrust, false)), kOk);
  EXPECT_EQ(ClockGuard(start).Probe(At(10, kNow + kRtcTrust + 1, false)), (ClockVerdict{false, kNeverSynced}));
  // Running on past the limit without a synchronization.
  ClockGuard guard(start);
  EXPECT_EQ(guard.Probe(At(10, kNow + kRtcTrust - 5, false)), kOk);
  EXPECT_EQ(guard.Probe(At(20, kNow + kRtcTrust + 5, false)), (ClockVerdict{false, kNeverSynced}));
}

TEST(ClockGuard, TheRealTimeClockOnlyWithoutAJumpSinceTheStart) {
  ClockGuard guard(Rtc(true, true, kNow - kDay));
  EXPECT_EQ(guard.Probe(At(10, kNow, false)), kOk);
  EXPECT_EQ(guard.Probe(At(20, kNow + 10 + 61, false)), (ClockVerdict{false, kClockJump}));
  EXPECT_EQ(guard.Probe(At(30, kNow + 20, false)), (ClockVerdict{false, kClockJump}));
}

TEST(ClockGuard, TheTimeOfDayOfTheReceiverInEveryWay) {
  const double day = std::fmod(kNow, kDay);
  ClockGuard guard({});
  EXPECT_EQ(guard.Probe(At(10, kNow, true, day + 59)), kOk);
  EXPECT_EQ(guard.Probe(At(11, kNow + 1, true, day + 1 + 61)), (ClockVerdict{false, kGnssMismatch}));
  // Without a fresh fix the last comparison holds; it ends once the two agree.
  EXPECT_EQ(guard.Probe(At(12, kNow + 2, true)), (ClockVerdict{false, kGnssMismatch}));
  EXPECT_EQ(guard.Probe(At(13, kNow + 3, true, day + 3)), kOk);
  // The reason of the clock itself comes first.
  ClockGuard never({});
  EXPECT_EQ(never.Probe(At(10, kNow, false, day + 600)), (ClockVerdict{false, kNeverSynced}));
}

TEST(ClockGuard, TheTimeOfDayAcrossMidnight) {
  const double midnight = std::floor(kNow / kDay) * kDay + kDay;
  ClockGuard guard({});
  EXPECT_EQ(guard.Probe(At(10, midnight - 20, true, 30)), kOk);
  EXPECT_EQ(guard.Probe(At(11, midnight + 20, true, kDay - 30)), kOk);
  EXPECT_EQ(guard.Probe(At(12, midnight - 20, true, 50)), (ClockVerdict{false, kGnssMismatch}));
}

TEST(ClockGuard, SynchronizedByTheMaximumError) {
  EXPECT_TRUE(ClockSynchronized(TIME_OK, 710000));
  EXPECT_TRUE(ClockSynchronized(TIME_OK, 15999999));
  EXPECT_FALSE(ClockSynchronized(TIME_OK, 16000000));
  EXPECT_FALSE(ClockSynchronized(-1, 0));
  // STA_UNSYNC with a small error: timesyncd where the real-time clock keeps local time.
  EXPECT_TRUE(ClockSynchronized(TIME_ERROR, 880000));
  // Only reads the kernel: it runs anywhere, whatever it answers.
  (void)SystemClockSynchronized();
}
