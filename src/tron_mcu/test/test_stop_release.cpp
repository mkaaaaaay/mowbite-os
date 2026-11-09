#include <gtest/gtest.h>

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

McuControlConfig GestureConfig(bool imu = false) {
  McuControlConfig config;
  config.stop_release = true;
  if (imu) {
    config.imu_timeout_s = 0.5;
    config.imu_start_timeout_s = 2.0;
  }
  return config;
}

// Runs the driver core in 10 ms steps with a sensor sample each, and records the STOP gestures it reports.
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
      if (feed_sensor_) control_.OnSensorInfo(sensor_, now_);
      if (feed_imu_) control_.OnImu(ImuCounts{}, now_);
      for (auto& packet : control_.Tick(now_, kWall)) {
        if (IsClear(packet)) clears_.push_back(now_);
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
  int step_ = 0;
  double now_ = 0;
  uint32_t seq_ = 0;
  std::vector<Event> events_;
  std::vector<double> clears_;
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

TEST_F(GestureTest, HeldAfterThreeSecondsLongTenthAfterLettingGo) {
  Run(1.0);
  Trigger(2.0);
  const double start = now_;
  sensor_.stop = 1;
  Run(start + 2.95);
  EXPECT_TRUE(events_.empty());
  Run(start + 3.05);
  ASSERT_EQ(Presses(), PressList{kHeld});
  EXPECT_NEAR(events_[0].t, start + 3.01, 0.015);
  Run(start + 4.0);
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
  Press(4.0, 2.0);
  EXPECT_TRUE(events_.empty());
  // Two seconds free after it: the next press counts.
  Press(4.0);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
}

TEST_F(GestureTest, HoldOfJustUnderThreeSecondsDoesNothing) {
  Run(1.0);
  Trigger();
  Press(2.9, 2.5);
  EXPECT_TRUE(events_.empty());
  // The last sample with STOP at 2.99 s: not yet held.
  Press(3.0, 2.5);
  EXPECT_TRUE(events_.empty());
  Press(3.02);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
}

TEST_F(GestureTest, TenSecondsCancelAndLettingGoAfterDoesNothing) {
  Run(1.0);
  Trigger();
  const double start = now_;
  Press(12.0, 0.5);
  ASSERT_EQ(Presses(), (PressList{kHeld, kCanceled}));
  EXPECT_NEAR(events_[1].t, start + 10.01, 0.015);
  // The next try needs the free time again.
  Run(now_ + 1.6);
  Press(4.0);
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled, kHeld, kLong}));
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(GestureTest, LettingGoJustBeforeTenSecondsReleases) {
  Run(1.0);
  Trigger();
  Press(9.95);
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
  Run(now_ + 2.0);
  sensor_.stop = 0;
  Run(now_ + 0.08);
  sensor_.stop = 1;
  Run(now_ + 2.0);
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
  Run(now_ + 2.0);
  sensor_.stop = 0;
  Run(now_ + 0.12);
  sensor_.stop = 1;
  Run(now_ + 2.0);
  sensor_.stop = 0;
  Run(now_ + 0.5);
  // Two presses of 2 s, the second without free time before it.
  EXPECT_TRUE(events_.empty());
}

TEST_F(GestureTest, QuickDoublePressDoesNotCount) {
  Run(1.0);
  Trigger(0.5);
  Press(4.0);
  EXPECT_TRUE(events_.empty());
}

TEST_F(GestureTest, TelemetryGapEndsTheGesture) {
  Run(1.0);
  Trigger();
  sensor_.stop = 1;
  Run(now_ + 3.5);
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
  Press(4.0, 2.5);
  EXPECT_TRUE(events_.empty());
  Press(4.0);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
}

TEST_F(GestureTest, LiftChangingEndsTheGestureAndRestartsTheFreeTime) {
  Run(1.0);
  Trigger();
  sensor_.stop = 1;
  Run(now_ + 3.5);
  sensor_.lift = 1;
  Run(now_ + 0.3);
  sensor_.lift = 0;
  Run(now_ + 0.5);
  sensor_.stop = 0;
  Run(now_ + 1.0);
  ASSERT_EQ(Presses(), (PressList{kHeld, kCanceled}));
  // Lifted again within the free time: that try does not count either.
  sensor_.lift = 1;
  Run(now_ + 0.2);
  sensor_.lift = 0;
  Run(now_ + 1.9);
  Press(4.0);
  EXPECT_EQ(Presses(), (PressList{kHeld, kCanceled}));
}

TEST_F(GestureTest, LiftKeptByTheBoardStillReleasesWithTheClearFrame) {
  Run(1.0);
  sensor_.lift = 1;
  Run(now_ + 0.5);
  ASSERT_TRUE(control_.Status(now_).latched);
  Run(now_ + 2.5);
  Press(4.0, 0.15);
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
  Run(now_ + 3.5);
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
  Run(now_ + 1.0);
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
  Run(now_ + 3.5);
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
  Run(now_ + 3.5);
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
  Run(now_ + 3.5);
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
  Press(4.0);
  EXPECT_EQ(Presses(), (PressList{kHeld, kLong}));
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
  Press(4.0, 0.15);
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
  for (int i = 0; i < 400; i++) {
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
