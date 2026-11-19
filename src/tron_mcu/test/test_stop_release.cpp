#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>

#include "tron_mcu/mcu_control.h"

using namespace tron;

namespace {

const WallClock kWall{2026, 11, 9, 12, 0, 0, 0};

bool IsClear(const SubPacket& packet) {
  return packet.type == static_cast<uint8_t>(PacketType::kHostToMcu) &&
         packet.module == static_cast<uint8_t>(Module::kSensor) && packet.payload == std::vector<uint8_t>(8, 0);
}

bool IsZeroSpeed(const SubPacket& packet) {
  const SubPacket zero = ZeroSpeedPacket();
  return packet.type == zero.type && packet.module == zero.module && packet.payload == zero.payload;
}

McuControlConfig GestureConfig(bool imu = false) {
  McuControlConfig config;
  config.stop_release = true;
  if (imu) {
    config.imu_timeout_s = 0.5;
    config.imu_start_timeout_s = 2.0;
  }
  return config;
}

// Runs the driver core in 10 ms steps with a sensor sample each, and records the STOP gestures it reports. With
// board_keeps_ the sample shows stop and lift as the Tron's MCU does: kept after the input went, until a clear frame.
class GestureTest : public ::testing::Test {
 protected:
  struct Event {
    double t;
    StopPress press;
    uint32_t latch_count;
  };

  explicit GestureTest(McuControlConfig config = GestureConfig()) : control_(config) {
  }

  void Run(double until) {
    while (now_ < until - 1e-9) {
      now_ = ++step_ * 0.01;
      SensorInfo sample = sensor_;
      if (board_keeps_) {
        kept_stop_ = kept_stop_ || sensor_.stop;
        kept_lift_ = kept_lift_ || sensor_.lift;
        sample.stop = kept_stop_;
        sample.lift = kept_lift_;
      }
      if (feed_sensor_) control_.OnSensorInfo(sample, now_);
      if (feed_imu_) control_.OnImu(ImuCounts{}, now_);
      const std::vector<SubPacket> packets = control_.Tick(now_, kWall);
      for (const auto& packet : packets) {
        if (!IsClear(packet)) continue;
        clears_.push_back(now_);
        clear_ticks_.push_back(packets);
        kept_stop_ = sensor_.stop;
        kept_lift_ = sensor_.lift;
      }
      const StopEvent event = control_.Status(now_).stop_event;
      if (event.seq != seq_) {
        EXPECT_EQ(event.seq, seq_ + 1) << "one gesture per tick at " << now_;
        seq_ = event.seq;
        events_.push_back({now_, event.press, event.latch_count});
      }
    }
  }

  // Holds STOP for held s, then lets go and runs on for after s.
  void Press(double held, double after = 0.5) {
    sensor_.stop = 1;
    Run(now_ + held);
    sensor_.stop = 0;
    Run(now_ + after);
  }

  // Latches the emergency with a short press and lets STOP be free for free s.
  void Trigger(double free = 2.5) {
    Press(0.3, free);
    ASSERT_TRUE(control_.Status(now_).latched);
  }

  std::vector<StopPress> Presses() const {
    std::vector<StopPress> presses;
    for (const auto& e : events_) presses.push_back(e.press);
    return presses;
  }

  McuControl control_;
  SensorInfo sensor_;
  bool feed_sensor_ = true;
  bool feed_imu_ = false;
  bool board_keeps_ = false;
  bool kept_stop_ = false;
  bool kept_lift_ = false;
  int step_ = 0;
  double now_ = 0;
  uint32_t seq_ = 0;
  std::vector<Event> events_;
  std::vector<double> clears_;
  // The frames of each tick that sent a clear frame.
  std::vector<std::vector<SubPacket>> clear_ticks_;
};

using PressList = std::vector<StopPress>;
const StopPress kHeld = StopPress::kHeld;
const StopPress kLong = StopPress::kLong;
const StopPress kCanceled = StopPress::kCanceled;

TEST_F(GestureTest, ThePressThatLatchesNeverReleasesHoweverLong) {
  Run(3.0);
  Press(6.0, 1.0);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_TRUE(events_.empty());
}

TEST_F(GestureTest, HeldAfterATenthLongATenthAfterLettingGo) {
  Run(1.0);
  Trigger(2.0);
  const double start = now_;
  sensor_.stop = 1;
  Run(start + 0.05);
  EXPECT_TRUE(events_.empty());
  Run(start + 0.15);
  ASSERT_EQ(Presses(), PressList{kHeld});
  EXPECT_NEAR(events_[0].t, start + 0.11, 0.015);
  Run(start + 1.0);
  sensor_.stop = 0;
  const double let_go = now_;
  Run(let_go + 0.5);
  ASSERT_EQ(Presses(), (PressList{kHeld, kLong}));
  EXPECT_NEAR(events_[1].t, let_go + 0.11, 0.015);
  // The press counted, so the release with its count goes through.
  EXPECT_EQ(events_[1].latch_count, control_.Status(now_).latch_count);
  EXPECT_NE(events_[1].latch_count, 0u);
  EXPECT_EQ(events_[0].latch_count, 0u);
  // The gesture only reports: the emergency stays until a release.
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_TRUE(control_.ReleaseEmergency(now_, events_[1].latch_count));
  Run(now_ + 0.05);
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(GestureTest, FreeTimeOfTwoSecondsCountsFromLettingGo) {
  Run(1.0);
  Trigger(1.9);
  Press(1.0, 2.0);
  EXPECT_TRUE(events_.empty());
  // Two seconds free after it: the next press counts.
  Press(1.0);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
}

TEST_F(GestureTest, APressUnderATenthDoesNothing) {
  Run(1.0);
  Trigger();
  // The last sample with STOP at 0.09 s: not yet held.
  Press(0.1, 2.5);
  EXPECT_TRUE(events_.empty());
  Press(0.12);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
}

TEST_F(GestureTest, TwoSecondsCancelAndLettingGoAfterDoesNothing) {
  Run(1.0);
  Trigger();
  const double start = now_;
  Press(3.0, 0.5);
  ASSERT_EQ(Presses(), (PressList{kHeld, kCanceled}));
  EXPECT_NEAR(events_[1].t, start + 2.01, 0.015);
  // The next try needs the free time again.
  Run(now_ + 1.6);
  Press(1.0);
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled, kHeld, kLong}));
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(GestureTest, LettingGoJustBeforeTwoSecondsReleases) {
  Run(1.0);
  Trigger();
  Press(1.95);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
}

TEST_F(GestureTest, BounceUnderATenthNeitherSplitsNorCounts) {
  Run(1.0);
  Trigger();
  const uint32_t count = control_.Status(now_).latch_count;
  // Bounce at the start and in the middle of the press.
  sensor_.stop = 1;
  Run(now_ + 0.03);
  sensor_.stop = 0;
  Run(now_ + 0.05);
  sensor_.stop = 1;
  Run(now_ + 0.5);
  sensor_.stop = 0;
  Run(now_ + 0.08);
  sensor_.stop = 1;
  Run(now_ + 0.5);
  sensor_.stop = 0;
  Run(now_ + 0.5);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
  // Only the start of the press counted.
  EXPECT_EQ(control_.Status(now_).latch_count, count + 1);
}

TEST_F(GestureTest, ATenthOfFreeTimeSplitsThePress) {
  Run(1.0);
  Trigger();
  sensor_.stop = 1;
  Run(now_ + 2.5);
  sensor_.stop = 0;
  Run(now_ + 0.12);
  sensor_.stop = 1;
  Run(now_ + 1.0);
  sensor_.stop = 0;
  Run(now_ + 0.5);
  // A press too long, then one without free time before it.
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled}));
}

TEST_F(GestureTest, QuickDoublePressDoesNotCount) {
  Run(1.0);
  Trigger(0.5);
  Press(1.0);
  EXPECT_TRUE(events_.empty());
}

TEST_F(GestureTest, TelemetryGapEndsTheGesture) {
  Run(1.0);
  Trigger();
  sensor_.stop = 1;
  Run(now_ + 0.5);
  feed_sensor_ = false;
  Run(now_ + 1.2);
  feed_sensor_ = true;
  Run(now_ + 1.0);
  sensor_.stop = 0;
  Run(now_ + 0.5);
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled}));
}

TEST_F(GestureTest, ATelemetryGapInTheFreeTimeRestartsIt) {
  Run(1.0);
  Trigger(1.0);
  feed_sensor_ = false;
  Run(now_ + 1.1);
  feed_sensor_ = true;
  Run(now_ + 1.5);
  Press(1.0, 2.5);
  EXPECT_TRUE(events_.empty());
  Press(1.0);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
}

TEST_F(GestureTest, LiftRisingEndsTheGestureAndRestartsTheFreeTime) {
  Run(1.0);
  Trigger();
  sensor_.stop = 1;
  Run(now_ + 0.5);
  sensor_.lift = 1;
  Run(now_ + 0.3);
  sensor_.lift = 0;
  Run(now_ + 0.5);
  sensor_.stop = 0;
  Run(now_ + 1.0);
  ASSERT_EQ(Presses(), (PressList{kHeld, kCanceled}));
  // Lifted again: the free time counts from lift rising, a press 1.9 s after it does not count either.
  sensor_.lift = 1;
  Run(now_ + 0.2);
  sensor_.lift = 0;
  Run(now_ + 1.7);
  Press(1.0);
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled}));
}

TEST_F(GestureTest, LiftFallingKeepsTheGesture) {
  Run(1.0);
  sensor_.lift = 1;
  Run(now_ + 0.5);
  ASSERT_TRUE(control_.Status(now_).latched);
  Run(now_ + 2.5);
  sensor_.stop = 1;
  Run(now_ + 0.3);
  // The clear frame drops the lift the board kept.
  sensor_.lift = 0;
  Run(now_ + 0.3);
  sensor_.stop = 0;
  Run(now_ + 0.5);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
}

TEST_F(GestureTest, LiftKeptByTheBoardStillReleasesWithTheClearFrame) {
  Run(1.0);
  sensor_.lift = 1;
  Run(now_ + 0.5);
  ASSERT_TRUE(control_.Status(now_).latched);
  Run(now_ + 2.5);
  Press(1.0, 0.15);
  ASSERT_EQ(Presses(), (PressList{kHeld, kLong}));
  ASSERT_TRUE(control_.ReleaseEmergency(now_, events_.back().latch_count));
  Run(now_ + 0.05);
  ASSERT_EQ(clears_.size(), 1u);
  // Still lifted: the release waits for the window.
  EXPECT_TRUE(control_.Status(now_).latched);
  sensor_.lift = 0;
  Run(now_ + 0.05);
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(GestureTest, ABumperWhileHoldingEndsTheGesture) {
  Run(1.0);
  Trigger();
  sensor_.stop = 1;
  Run(now_ + 0.5);
  sensor_.bumper_left = 1;
  Run(now_ + 0.05);
  sensor_.bumper_left = 0;
  Run(now_ + 0.5);
  sensor_.stop = 0;
  Run(now_ + 0.5);
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled}));
}

TEST_F(GestureTest, AHostLatchWhileHoldingEndsTheGesture) {
  Run(1.0);
  Trigger();
  sensor_.stop = 1;
  Run(now_ + 0.05);
  control_.LatchEmergency(kReasonTilt);
  Run(now_ + 3.0);
  sensor_.stop = 0;
  Run(now_ + 0.5);
  // Ended before it was held: nothing to report.
  EXPECT_TRUE(events_.empty());
}

TEST_F(GestureTest, AReopenedPortEndsTheGesture) {
  Run(1.0);
  Trigger();
  sensor_.stop = 1;
  Run(now_ + 0.5);
  control_.OnConnected(now_);
  Run(now_ + 0.5);
  sensor_.stop = 0;
  Run(now_ + 0.5);
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled}));
}

TEST_F(GestureTest, ReleasedByTheAppWhileHeldEndsCanceled) {
  Run(1.0);
  Trigger();
  sensor_.stop = 1;
  Run(now_ + 0.5);
  // The app asks while STOP is held; letting go within the window releases.
  EXPECT_TRUE(control_.ReleaseEmergency(now_, 0));
  Run(now_ + 0.5);
  EXPECT_TRUE(control_.Status(now_).latched);
  sensor_.stop = 0;
  Run(now_ + 0.5);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled}));
}

TEST_F(GestureTest, AnUpdateHoldAloneIsNoEmergency) {
  control_.SetHold(true);
  Run(3.0);
  Press(4.0);
  // The press latched (STOP), it was the first one.
  EXPECT_TRUE(events_.empty());
}

class GestureOffTest : public GestureTest {
 protected:
  GestureOffTest() : GestureTest(McuControlConfig{}) {
  }
};

TEST_F(GestureOffTest, NothingWithoutTheSwitch) {
  Run(1.0);
  Trigger();
  Press(4.0);
  EXPECT_TRUE(events_.empty());
  EXPECT_TRUE(control_.Status(now_).latched);
}

class GestureImuTest : public GestureTest {
 protected:
  GestureImuTest() : GestureTest(GestureConfig(true)) {
    feed_imu_ = true;
  }
};

TEST_F(GestureImuTest, AnImuGapEndsTheGesture) {
  Run(1.0);
  Trigger();
  sensor_.stop = 1;
  Run(now_ + 0.5);
  feed_imu_ = false;
  Run(now_ + 0.6);
  feed_imu_ = true;
  Run(now_ + 0.5);
  sensor_.stop = 0;
  Run(now_ + 0.5);
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled}));
}

TEST_F(GestureImuTest, WithAFreshImuItReleases) {
  Run(1.0);
  Trigger();
  Press(1.0);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
}

// The Tron's MCU keeps STOP: without clear frames during the emergency no press ever shows free.
class BoardKeepsStopTest : public GestureTest {
 protected:
  BoardKeepsStopTest() {
    board_keeps_ = true;
  }
};

TEST_F(BoardKeepsStopTest, WithoutClearFramesNothingReleases) {
  Run(1.0);
  Trigger();
  Run(now_ + 3.0);
  Press(1.0, 3.0);
  Press(1.0);
  EXPECT_TRUE(events_.empty());
  EXPECT_TRUE(clears_.empty());
  EXPECT_TRUE(control_.Status(now_).stop);
}

McuControlConfig StopClearConfig() {
  McuControlConfig config = GestureConfig();
  config.stop_clear_period_s = 0.25;
  return config;
}

// STOP cleared during the emergency (G37), the mark written.
class StopClearTest : public GestureTest {
 protected:
  StopClearTest() : GestureTest(StopClearConfig()) {
    board_keeps_ = true;
    control_.SetStopClearMark(true);
  }
};

TEST_F(StopClearTest, AShortPressAfterTwoSecondsFreeReleases) {
  Run(1.0);
  Press(0.3, 1.0);
  const double latched = 1.01;
  ASSERT_TRUE(control_.Status(now_).latched);
  EXPECT_TRUE(control_.Status(now_).stop);
  EXPECT_TRUE(clears_.empty());
  Run(now_ + 3.5);
  // One clear stop_clear_delay_s after the latch drops the let go STOP; none while stop is not shown.
  ASSERT_EQ(clears_.size(), 1u);
  EXPECT_NEAR(clears_[0], latched + 1.5, 0.015);
  EXPECT_FALSE(control_.Status(now_).stop);
  Press(0.5, 0.6);
  ASSERT_EQ(Presses(), (PressList{kHeld, kLong}));
  // Held, the MCU kept stop through the clear frames; let go, the next one dropped it.
  EXPECT_GE(clears_.size(), 3u);
  ASSERT_TRUE(control_.ReleaseEmergency(now_, events_.back().latch_count));
  Run(now_ + 0.05);
  EXPECT_FALSE(control_.Status(now_).latched);
  const size_t after_release = clears_.size();
  Press(0.3, 3.0);
  // A new press latches again, the clear frames start after the delay once more.
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_EQ(clears_.size(), after_release + 1);
}

TEST_F(StopClearTest, AHeldTriggerStaysShownAndNeverReleases) {
  Run(1.0);
  sensor_.stop = 1;
  Run(now_ + 5.0);
  ASSERT_GE(clears_.size(), 10u);
  for (size_t i = 1; i < clears_.size(); i++) EXPECT_GE(clears_[i] - clears_[i - 1], 0.25 - 1e-6);
  EXPECT_TRUE(control_.Status(now_).stop);
  sensor_.stop = 0;
  Run(now_ + 1.0);
  EXPECT_FALSE(control_.Status(now_).stop);
  EXPECT_TRUE(events_.empty());
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(StopClearTest, APressOverTwoSecondsCancels) {
  Run(1.0);
  Press(0.3, 4.5);
  Press(2.5, 0.6);
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled}));
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(StopClearTest, TwoPressesWithinTwoSecondsReleaseNothing) {
  Run(1.0);
  Press(0.3, 1.6);
  Press(0.3, 1.0);
  Press(0.3, 1.0);
  EXPECT_TRUE(events_.empty());
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(StopClearTest, NoClearFrameBeforeTheMarkIsWritten) {
  control_.SetStopClearMark(false);
  Run(1.0);
  Press(0.3, 2.0);
  EXPECT_TRUE(clears_.empty());
  EXPECT_TRUE(control_.Status(now_).stop_clear_mark_wanted);
  control_.SetStopClearMark(true);
  Run(now_ + 0.05);
  EXPECT_EQ(clears_.size(), 1u);
  EXPECT_FALSE(control_.Status(now_).stop_clear_mark_wanted);
}

TEST_F(StopClearTest, NoClearFrameWhileTheBladeTurns) {
  Run(5.5);
  // A blade still turning after it went off latches and holds the clear frames back.
  control_.OnBladeRpm(3000, now_);
  Press(0.3, 3.0);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_TRUE(clears_.empty());
  EXPECT_FALSE(control_.Status(now_).stop_clear_mark_wanted);
  control_.OnBladeRpm(0, now_);
  Run(now_ + 0.05);
  EXPECT_EQ(clears_.size(), 1u);
}

TEST_F(StopClearTest, NoClearFrameWithStaleTelemetry) {
  Run(1.0);
  Press(0.3, 0.0);
  feed_sensor_ = false;
  Run(now_ + 3.0);
  EXPECT_TRUE(clears_.empty());
}

TEST_F(StopClearTest, ZeroSpeedGoesBeforeTheClearAfterMotion) {
  Run(1.0);
  control_.OnCmdVel(0.1f, 0.0f, now_);
  Run(now_ + 0.2);
  ASSERT_TRUE(control_.Status(now_).moving);
  Press(0.3, 2.0);
  ASSERT_FALSE(clear_ticks_.empty());
  const auto& tick = clear_ticks_.front();
  const auto clear = std::find_if(tick.begin(), tick.end(), IsClear);
  ASSERT_NE(clear, tick.begin());
  EXPECT_TRUE(IsZeroSpeed(*(clear - 1)));
}

TEST_F(StopClearTest, ZeroSpeedGoesBeforeTheClearAlsoWithoutMotion) {
  Run(1.0);
  Press(0.3, 2.0);
  ASSERT_FALSE(clear_ticks_.empty());
  for (const auto& tick : clear_ticks_) {
    const auto clear = std::find_if(tick.begin(), tick.end(), IsClear);
    ASSERT_NE(clear, tick.begin());
    EXPECT_TRUE(IsZeroSpeed(*(clear - 1)));
  }
}

TEST_F(StopClearTest, LiftKeptByTheBoardDropsWithTheClearAndAPressReleases) {
  Run(1.0);
  sensor_.lift = 1;
  Run(now_ + 0.5);
  sensor_.lift = 0;
  Run(now_ + 3.0);
  // Stop is not shown: no clear frame, the board keeps lift.
  EXPECT_TRUE(clears_.empty());
  EXPECT_TRUE(control_.Status(now_).lift);
  Press(0.5, 0.6);
  ASSERT_EQ(Presses(), (PressList{kHeld, kLong}));
  EXPECT_FALSE(control_.Status(now_).lift);
  ASSERT_TRUE(control_.ReleaseEmergency(now_, events_.back().latch_count));
  Run(now_ + 0.05);
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(StopClearTest, NoClearFrameWithoutTheGesture) {
  McuControlConfig config = StopClearConfig();
  config.stop_release = false;
  McuControl control(config);
  control.SetStopClearMark(true);
  SensorInfo sensor;
  sensor.stop = 1;
  for (int i = 1; i <= 500; i++) {
    control.OnSensorInfo(sensor, i * 0.01);
    for (const auto& packet : control.Tick(i * 0.01, kWall)) EXPECT_FALSE(IsClear(packet)) << i;
  }
  EXPECT_TRUE(control.Status(5.0).latched);
}

// latch_count of the driver core.
class LatchCountTest : public GestureTest {};

TEST_F(LatchCountTest, EveryNewReasonCountsTheSameBitAgainDoesNot) {
  Run(1.0);
  EXPECT_EQ(control_.Status(now_).latch_count, 0u);
  control_.LatchEmergency(kReasonTilt);
  EXPECT_EQ(control_.Status(now_).latch_count, 1u);
  control_.LatchEmergency(kReasonTilt);
  EXPECT_EQ(control_.Status(now_).latch_count, 1u);
  control_.LatchEmergency(kReasonFence);
  EXPECT_EQ(control_.Status(now_).latch_count, 2u);
  // A press of STOP counts once while it lasts.
  Press(1.0);
  EXPECT_EQ(control_.Status(now_).latch_count, 3u);
  // Lift and bumper count when they rise, also with their bit latched.
  sensor_.lift = 1;
  Run(now_ + 0.1);
  sensor_.lift = 0;
  Run(now_ + 0.1);
  sensor_.lift = 1;
  Run(now_ + 0.1);
  sensor_.lift = 0;
  Run(now_ + 0.1);
  EXPECT_EQ(control_.Status(now_).latch_count, 5u);
  sensor_.bumper_right = 1;
  Run(now_ + 0.1);
  sensor_.bumper_right = 0;
  Run(now_ + 0.1);
  EXPECT_EQ(control_.Status(now_).latch_count, 6u);
}

TEST_F(LatchCountTest, AnImuGapCountsOnce) {
  McuControl control(GestureConfig(true));
  control.OnSensorInfo(SensorInfo{}, 0.01);
  control.OnImu(ImuCounts{}, 0.01);
  for (int i = 1; i <= 200; i++) {
    control.OnSensorInfo(SensorInfo{}, i * 0.01);
    control.Tick(i * 0.01, kWall);
  }
  EXPECT_TRUE(control.Status(2.0).latched);
  EXPECT_EQ(control.Status(2.0).latch_count, 1u);
}

TEST_F(LatchCountTest, AnOutdatedCountIsRefused) {
  Run(1.0);
  Trigger();
  Press(1.0, 0.15);
  ASSERT_EQ(Presses(), (PressList{kHeld, kLong}));
  const uint32_t count = events_.back().latch_count;
  // A short new press between letting go and the request.
  Press(0.05, 0.1);
  EXPECT_FALSE(control_.ReleaseEmergency(now_, count));
  Run(now_ + 1.5);
  EXPECT_TRUE(control_.Status(now_).latched);
  // Without the check (app, HA) it releases.
  EXPECT_TRUE(control_.ReleaseEmergency(now_, 0));
  Run(now_ + 0.05);
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(LatchCountTest, ANewReasonInTheWindowEndsTheRelease) {
  Run(1.0);
  sensor_.lift = 1;
  Run(now_ + 0.5);
  // The app asks while lifted, the board keeps lift after the clear frame: the window waits.
  EXPECT_TRUE(control_.ReleaseEmergency(now_, 0));
  Run(now_ + 0.3);
  // A press of STOP within the window ends it without releasing, even once lift and STOP are free.
  Press(0.05, 0.0);
  sensor_.lift = 0;
  Run(now_ + 0.3);
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(LatchCountTest, ANewReasonBetweenRequestAndTickSendsNoClearFrame) {
  Run(1.0);
  sensor_.lift = 1;
  Run(now_ + 0.5);
  EXPECT_TRUE(control_.ReleaseEmergency(now_, 0));
  control_.LatchEmergency(kReasonTilt);
  Run(now_ + 0.5);
  EXPECT_TRUE(clears_.empty());
}

// hw/status carries the count for a release with it: a latched driver never shows 0.
TEST_F(LatchCountTest, NeverZeroOnceLatched) {
  using Cause = void (*)(McuControl&, SensorInfo&);
  const Cause causes[] = {[](McuControl&, SensorInfo& sensor) { sensor.stop = true; },
                          [](McuControl&, SensorInfo& sensor) { sensor.lift = true; },
                          [](McuControl&, SensorInfo& sensor) { sensor.bumper_left = true; },
                          [](McuControl& control, SensorInfo&) { control.LatchEmergency(kReasonHighLevel); }};
  for (size_t i = 0; i < std::size(causes); i++) {
    McuControl control(GestureConfig());
    SensorInfo sensor;
    causes[i](control, sensor);
    // From the first sample on, as after a start with STOP kept by the board.
    for (int step = 1; step <= 3; step++) {
      control.OnSensorInfo(sensor, step * 0.01);
      control.Tick(step * 0.01, kWall);
      const McuControlStatus status = control.Status(step * 0.01);
      EXPECT_TRUE(status.latched) << i;
      EXPECT_NE(status.latch_count, 0u) << i;
    }
  }
}

// A release with the count of hw/status, as the setup program asks for after its own STOP. The board keeps STOP, so
// the try itself tells whether the button is free.
TEST_F(BoardKeepsStopTest, AReleaseWithTheCountOfTheStatusShowsWhetherStopIsFree) {
  Run(1.0);
  sensor_.stop = true;
  Run(now_ + 1.5);
  const uint32_t count = control_.Status(now_).latch_count;
  ASSERT_NE(count, 0u);
  // Held: taken, one clear frame, STOP and the latch stay, and the count with them.
  EXPECT_TRUE(control_.ReleaseEmergency(now_, count));
  Run(now_ + 1.5);
  EXPECT_EQ(clears_.size(), 1u);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_TRUE(control_.Status(now_).stop);
  EXPECT_EQ(control_.Status(now_).latch_count, count);
  // Let go: the board shows it until the next try, which releases.
  sensor_.stop = false;
  Run(now_ + 1.5);
  EXPECT_TRUE(control_.Status(now_).stop);
  EXPECT_TRUE(control_.ReleaseEmergency(now_, count));
  Run(now_ + 0.1);
  EXPECT_EQ(clears_.size(), 2u);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
}

TEST_F(BoardKeepsStopTest, AShortBumperSinceTheStatusRefusesTheReleaseAndStaysInTheReason) {
  Run(1.0);
  Press(0.3, 1.5);
  const uint32_t count = control_.Status(now_).latch_count;
  sensor_.bumper_right = true;
  Run(now_ + 0.02);
  sensor_.bumper_right = false;
  Run(now_ + 0.1);
  EXPECT_FALSE(control_.ReleaseEmergency(now_, count));
  Run(now_ + 1.5);
  EXPECT_TRUE(clears_.empty());
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonStop | kReasonCollision);
}

// The core of the gesture alone, with a sample per tick as the caller gives it.
TEST(StopRelease, CountsOnlyWhileLatchedAndFree) {
  StopRelease gesture;
  StopReleaseInput input;
  input.usable = true;
  double t = 0;
  auto tick = [&](bool stop, bool latched) {
    t += 0.01;
    gesture.Sample(stop, t);
    input.latched = latched;
    return gesture.Update(input, t);
  };
  for (int i = 0; i < 300; i++) EXPECT_EQ(tick(false, false), StopPress::kNone);
  // Not latched before the press: it latches, it never counts.
  for (int i = 0; i < 400; i++) EXPECT_EQ(tick(true, i > 0), StopPress::kNone);
  for (int i = 0; i < 210; i++) EXPECT_EQ(tick(false, true), StopPress::kNone);
  std::vector<StopPress> events;
  for (int i = 0; i < 100; i++) {
    const StopPress press = tick(true, true);
    if (press != StopPress::kNone) events.push_back(press);
  }
  for (int i = 0; i < 20; i++) {
    const StopPress press = tick(false, true);
    if (press != StopPress::kNone) events.push_back(press);
  }
  EXPECT_EQ(events, (std::vector<StopPress>{StopPress::kHeld, StopPress::kLong}));
}

}  // namespace
