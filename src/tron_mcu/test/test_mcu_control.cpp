#include <gtest/gtest.h>

#include <cmath>

#include "tron_mcu/mcu_control.h"

using namespace tron;

namespace {

const WallClock kWall{2026, 10, 28, 12, 0, 0, 0};

bool IsModule(const SubPacket& packet, Module module) {
  return packet.type == static_cast<uint8_t>(PacketType::kHostToMcu) && packet.module == static_cast<uint8_t>(module);
}

bool IsHeartbeat(const SubPacket& packet) {
  return packet.type == static_cast<uint8_t>(PacketType::kHeartbeat);
}

bool IsZeroSpeed(const SubPacket& packet) {
  return IsModule(packet, Module::kSpeed) && packet.payload == std::vector<uint8_t>(8, 0);
}

bool IsMovingSpeed(const SubPacket& packet) {
  return IsModule(packet, Module::kSpeed) && !IsZeroSpeed(packet);
}

bool IsClear(const SubPacket& packet) {
  return IsModule(packet, Module::kSensor) && packet.payload == std::vector<uint8_t>(8, 0);
}

bool IsBladeOn(const SubPacket& packet) {
  return IsModule(packet, Module::kCutter) && packet.payload[0] == 1;
}

bool IsBladeOff(const SubPacket& packet) {
  return IsModule(packet, Module::kCutter) && packet.payload[0] == 0;
}

uint16_t CutterHeight(const SubPacket& packet) {
  return packet.payload[10] | (packet.payload[11] << 8);
}

BladeRequest BladeOn(double lease_s = 2.0, uint16_t speed_percent = 0, uint16_t height_mm = 0) {
  BladeRequest request;
  request.on = true;
  request.lease_s = lease_s;
  request.speed_percent = speed_percent;
  request.height_mm = height_mm;
  return request;
}

const BladeRequest kBladeOff{};

// Runs the controller in 10 ms steps and records every packet with its time.
class ControlTest : public ::testing::Test {
 protected:
  struct Sent {
    double t;
    SubPacket packet;
  };

  explicit ControlTest(McuControlConfig config = {}) : control_(config) {
  }

  void Run(double until, bool feed_sensor = true) {
    while (now_ < until - 1e-9) {
      now_ = ++step_ * 0.01;
      if (feed_sensor) control_.OnSensorInfo(sensor_, now_);
      if (feed_imu_) control_.OnImu(ImuCounts{}, now_);
      for (auto& packet : control_.Tick(now_, kWall)) sent_.push_back({now_, packet});
    }
  }

  template <typename Pred>
  std::vector<double> Times(Pred pred, double from = -1, double to = 1e9) const {
    std::vector<double> times;
    for (const auto& s : sent_) {
      if (s.t >= from - 1e-9 && s.t <= to + 1e-9 && pred(s.packet)) times.push_back(s.t);
    }
    return times;
  }

  // Commands motion and blade, runs on and tells whether either reached the MCU.
  bool DrivesOrMows(double until) {
    const size_t before = sent_.size();
    control_.OnCmdVel(0.2f, 0.0f, now_);
    control_.SetBlade(BladeOn(), now_);
    Run(until);
    for (size_t i = before; i < sent_.size(); i++) {
      if (IsMovingSpeed(sent_[i].packet) || IsBladeOn(sent_[i].packet)) return true;
    }
    return false;
  }

  McuControl control_;
  SensorInfo sensor_;
  // An IMU sample with every step.
  bool feed_imu_ = false;
  int step_ = 0;
  double now_ = 0;
  std::vector<Sent> sent_;
};

class BladeControlTest : public ControlTest {
 protected:
  static McuControlConfig BladeConfig() {
    McuControlConfig config;
    config.allow_blade = true;
    config.mow_height_mm = 45;
    return config;
  }
  BladeControlTest() : ControlTest(BladeConfig()) {
  }
};

class FirmwareControlTest : public ControlTest {
 protected:
  static McuControlConfig FirmwareConfig() {
    McuControlConfig config;
    config.allow_blade = true;
    config.expected_cutter_version = "0.6.36";
    config.expected_chassis_version = "0.6.34";
    return config;
  }
  FirmwareControlTest() : ControlTest(FirmwareConfig()) {
  }
  static VersionInfo Versions(uint8_t cutter_patch, uint8_t chassis_patch) {
    VersionInfo v;
    v.cutter_board = {0, 6, cutter_patch};
    v.chassis_board = {0, 6, chassis_patch};
    return v;
  }
};

class ImuControlTest : public ControlTest {
 protected:
  static McuControlConfig ImuConfig() {
    McuControlConfig config;
    config.allow_blade = true;
    config.imu_timeout_s = 0.5;
    config.imu_start_timeout_s = 2.0;
    return config;
  }
  ImuControlTest() : ControlTest(ImuConfig()) {
  }
};

class ShortImuStartTest : public ControlTest {
 protected:
  static McuControlConfig ShortStartConfig() {
    McuControlConfig config;
    config.imu_timeout_s = 0.5;
    config.imu_start_timeout_s = 0.2;
    return config;
  }
  ShortImuStartTest() : ControlTest(ShortStartConfig()) {
  }
};

class BladeStopTest : public ControlTest {
 protected:
  static McuControlConfig StopConfig() {
    McuControlConfig config;
    config.allow_blade = true;
    config.blade_stop_time_s = 3.0;
    return config;
  }
  BladeStopTest() : ControlTest(StopConfig()) {
  }
};

}  // namespace

TEST_F(ControlTest, HeartbeatEvery100ms) {
  Run(1.0);
  auto times = Times(IsHeartbeat);
  ASSERT_EQ(times.size(), 10u);
  for (size_t i = 1; i < times.size(); i++) EXPECT_NEAR(times[i] - times[i - 1], 0.1, 1e-6);
}

TEST_F(ControlTest, BladeOffThreeTimesOnStart) {
  Run(1.0);
  auto times = Times(IsBladeOff);
  ASSERT_EQ(times.size(), 3u);
  for (const auto& s : sent_) {
    if (IsBladeOff(s.packet)) {
      EXPECT_EQ(CutterHeight(s.packet), 90);
    }
  }
}

TEST_F(ControlTest, ReconnectStopsWheelsAndBlade) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  control_.OnConnected(now_);
  Run(1.0);
  EXPECT_EQ(Times(IsMovingSpeed).size(), 1u);
  EXPECT_EQ(Times(IsZeroSpeed, 0.2).size(), 3u);
  // The reconnect restarts the blade-off burst.
  EXPECT_EQ(Times(IsBladeOff, 0.2).size(), 3u);
  EXPECT_FALSE(control_.Status(now_).moving);
}

TEST_F(ControlTest, NoMotionWithoutTelemetry) {
  control_.OnCmdVel(0.2f, 0.0f, 0.0);
  Run(0.5, false);
  EXPECT_TRUE(Times([](const SubPacket& p) { return IsModule(p, Module::kSpeed); }).empty());
  EXPECT_TRUE(control_.Status(now_).telemetry_stale);
}

TEST_F(ControlTest, OneFramePerCommandThenStopBurstThenSilence) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.1f, now_);
  Run(0.2);
  control_.OnCmdVel(0.0f, 0.0f, now_);
  Run(5.0);
  EXPECT_EQ(Times(IsMovingSpeed).size(), 1u);
  auto zeros = Times(IsZeroSpeed);
  ASSERT_EQ(zeros.size(), 3u);
  EXPECT_NEAR(zeros[1] - zeros[0], 0.1, 1e-6);
  EXPECT_NEAR(zeros[2] - zeros[1], 0.1, 1e-6);
  EXPECT_FALSE(control_.Status(now_).moving);
}

TEST_F(ControlTest, IdleZeroCommandsSendNothing) {
  for (int i = 0; i < 50; i++) {
    control_.OnCmdVel(0.0f, 0.0f, now_);
    Run(now_ + 0.1);
  }
  EXPECT_TRUE(Times([](const SubPacket& p) { return IsModule(p, Module::kSpeed); }).empty());
}

TEST_F(ControlTest, WatchdogStopsAfterSilence) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  double start = now_;
  Run(3.0);
  auto zeros = Times(IsZeroSpeed);
  ASSERT_EQ(zeros.size(), 3u);
  EXPECT_GT(zeros[0] - start, 0.5);
  EXPECT_LT(zeros[0] - start, 0.52);
}

TEST_F(ControlTest, CommandsKeepWatchdogQuiet) {
  Run(0.1);
  for (int i = 0; i < 30; i++) {
    control_.OnCmdVel(0.2f, 0.0f, now_);
    Run(now_ + 0.1);
  }
  EXPECT_EQ(Times(IsMovingSpeed).size(), 30u);
  EXPECT_TRUE(Times(IsZeroSpeed).empty());
}

TEST_F(ControlTest, NewCommandCancelsStopBurst) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  control_.OnCmdVel(0.0f, 0.0f, now_);
  Run(0.25);
  control_.OnCmdVel(0.1f, 0.0f, now_);
  Run(0.4);
  EXPECT_EQ(Times(IsZeroSpeed).size(), 1u);
  EXPECT_TRUE(control_.Status(now_).moving);
}

TEST_F(ControlTest, NonFiniteCommandStops) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  control_.OnCmdVel(NAN, 0.0f, now_);
  Run(1.0);
  EXPECT_EQ(Times(IsMovingSpeed).size(), 1u);
  EXPECT_EQ(Times(IsZeroSpeed).size(), 3u);
}

TEST_F(ControlTest, LiftStopsAndBlocksCommands) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  sensor_.lift = true;
  Run(0.3);
  for (int i = 0; i < 10; i++) {
    control_.OnCmdVel(0.2f, 0.0f, now_);
    Run(now_ + 0.05);
  }
  EXPECT_EQ(Times(IsMovingSpeed).size(), 1u);
  EXPECT_EQ(Times(IsZeroSpeed).size(), 3u);
  EXPECT_TRUE(control_.Status(now_).lift);
}

TEST_F(ControlTest, StaleTelemetryStops) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  // cmd_vel keeps coming, the MCU goes silent.
  for (int i = 0; i < 20; i++) {
    control_.OnCmdVel(0.2f, 0.0f, now_);
    Run(now_ + 0.1, false);
  }
  auto zeros = Times(IsZeroSpeed);
  ASSERT_EQ(zeros.size(), 3u);
  EXPECT_NEAR(zeros[0], 0.2 + 1.0, 0.03);
  EXPECT_TRUE(Times(IsMovingSpeed, zeros[0]).empty());
}

TEST_F(ControlTest, EmergencyLatchesUntilReleased) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  control_.LatchEmergency(kReasonHighLevel);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(1.0);
  EXPECT_EQ(Times(IsMovingSpeed).size(), 1u);
  EXPECT_EQ(Times(IsZeroSpeed).size(), 3u);
  control_.ReleaseEmergency(now_);
  Run(1.05);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(1.1);
  EXPECT_EQ(Times(IsMovingSpeed).size(), 2u);
  // Nothing to clear on the MCU.
  EXPECT_TRUE(Times(IsClear).empty());
}

TEST_F(ControlTest, RequestedBitsStayUntilReleased) {
  Run(0.1);
  control_.LatchEmergency(kReasonTilt);
  Run(0.2);
  EXPECT_TRUE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonTilt);
  control_.LatchEmergency(kReasonFence);
  Run(0.3);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonTilt | kReasonFence);
  EXPECT_FALSE(DrivesOrMows(0.5));
  control_.ReleaseEmergency(now_);
  Run(0.6);
  EXPECT_FALSE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  EXPECT_EQ(kHostLatchReasons & (kReasonLift | kReasonStop | kReasonCollision | kReasonImuTimeout), 0u);
}

TEST_F(ControlTest, LiftStaysLatchedAfterFlagDrops) {
  Run(0.1);
  sensor_.lift = true;
  Run(0.5);
  sensor_.lift = false;
  Run(1.0);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(1.5);
  EXPECT_TRUE(Times(IsMovingSpeed).empty());
  EXPECT_TRUE(control_.Status(now_).latched);
  control_.ReleaseEmergency(now_);
  Run(1.55);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(1.6);
  EXPECT_EQ(Times(IsMovingSpeed).size(), 1u);
}

TEST_F(ControlTest, ReleaseWithLiftSendsClearThenZeros) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  sensor_.lift = true;
  Run(1.0);
  control_.ReleaseEmergency(now_);
  Run(1.3);
  auto clears = Times(IsClear);
  ASSERT_EQ(clears.size(), 1u);
  // Three zeros when the lift stopped the wheels, three more after the clear.
  auto zeros = Times(IsZeroSpeed, clears[0]);
  ASSERT_EQ(zeros.size(), 3u);
  EXPECT_GE(zeros[0], clears[0]);
  // The MCU drops the flag within the release window.
  sensor_.lift = false;
  Run(1.4);
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(ControlTest, RepeatedReleaseWithStuckLiftIsQuiet) {
  Run(0.1);
  sensor_.lift = true;
  for (int i = 0; i < 20; i++) {
    control_.ReleaseEmergency(now_);
    Run(now_ + 0.5);
  }
  size_t clears = Times(IsClear).size();
  EXPECT_GE(clears, 9u);
  EXPECT_LE(clears, 11u);
  // No recent motion, so no speed frames at all, also not on the dock.
  EXPECT_TRUE(Times([](const SubPacket& p) { return IsModule(p, Module::kSpeed); }).empty());
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(ControlTest, ReleaseWindowExpires) {
  Run(0.1);
  sensor_.lift = true;
  Run(0.5);
  control_.ReleaseEmergency(now_);
  Run(2.0);
  sensor_.lift = false;
  Run(3.0);
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(ControlTest, BumperLatches) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  sensor_.bumper_left = true;
  Run(0.3);
  sensor_.bumper_left = false;
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(1.0);
  EXPECT_EQ(Times(IsMovingSpeed).size(), 1u);
  EXPECT_EQ(Times(IsZeroSpeed).size(), 3u);
  EXPECT_TRUE(control_.Status(now_).latched);
  // Bumper is not an MCU latch, no clear frame.
  control_.ReleaseEmergency(now_);
  Run(1.1);
  EXPECT_TRUE(Times(IsClear).empty());
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(ControlTest, LiftReasonStaysUntilRelease) {
  Run(0.5);
  EXPECT_FALSE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  sensor_.lift = true;
  Run(0.6);
  EXPECT_TRUE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonLift);
  sensor_.lift = false;
  Run(1.0);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonLift);
  control_.ReleaseEmergency(now_);
  Run(1.1);
  EXPECT_FALSE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
}

TEST_F(ControlTest, ReasonsAddUpUntilRelease) {
  Run(0.5);
  sensor_.bumper_right = true;
  Run(0.6);
  sensor_.bumper_right = false;
  sensor_.stop = true;
  Run(0.7);
  sensor_.stop = false;
  control_.LatchEmergency(kReasonHighLevel);
  Run(1.0);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonCollision | kReasonStop | kReasonHighLevel);
  control_.ReleaseEmergency(now_);
  Run(1.1);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  // A new cause starts from scratch.
  sensor_.lift = true;
  Run(1.2);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonLift);
}

TEST_F(ControlTest, StaleTelemetryBlocksWithoutLatch) {
  Run(0.5, false);
  EXPECT_TRUE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonTimeoutInputs);
  Run(0.6);
  EXPECT_FALSE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
}

TEST_F(ControlTest, NonLatchingBumperGivesNoReason) {
  McuControlConfig config;
  config.bumper_latches_emergency = false;
  McuControl control(config);
  SensorInfo sensor;
  sensor.bumper_left = true;
  control.OnSensorInfo(sensor, 0.01);
  control.Tick(0.01, kWall);
  EXPECT_TRUE(control.Status(0.01).bumper);
  EXPECT_FALSE(control.Status(0.01).blocked);
  EXPECT_EQ(control.Status(0.01).emergency_reason, 0u);
}

TEST_F(ControlTest, WithoutExpectedVersionsTheFirmwareIsNotChecked) {
  Run(0.1);
  EXPECT_FALSE(control_.Status(now_).firmware_incompatible);
  EXPECT_TRUE(control_.OnVersions(VersionInfo{}));
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  EXPECT_EQ(Times(IsMovingSpeed).size(), 1u);
}

TEST_F(FirmwareControlTest, BlockedUntilAVersionFrameMatches) {
  Run(0.1);
  EXPECT_TRUE(control_.Status(now_).firmware_incompatible);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_FALSE(DrivesOrMows(0.5));
  EXPECT_TRUE(control_.OnVersions(Versions(36, 34)));
  EXPECT_FALSE(control_.Status(now_).firmware_incompatible);
  EXPECT_TRUE(DrivesOrMows(0.7));
}

TEST_F(FirmwareControlTest, MismatchStaysBlockedAfterRelease) {
  Run(0.1);
  EXPECT_FALSE(control_.OnVersions(Versions(36, 35)));
  auto status = control_.Status(now_);
  EXPECT_TRUE(status.firmware_incompatible);
  EXPECT_TRUE(status.latched);
  control_.ReleaseEmergency(now_);
  Run(0.5);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_TRUE(control_.Status(now_).firmware_incompatible);
  EXPECT_FALSE(DrivesOrMows(1.0));
  // The next frame with the same wrong firmware does not latch again, the block stays as it is.
  EXPECT_FALSE(control_.OnVersions(Versions(36, 35)));
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_FALSE(DrivesOrMows(1.5));
}

TEST_F(FirmwareControlTest, MatchAfterMismatchStillNeedsRelease) {
  Run(0.1);
  control_.OnVersions(Versions(30, 34));
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonFirmwareIncompatible);
  control_.OnVersions(Versions(36, 34));
  EXPECT_FALSE(control_.Status(now_).firmware_incompatible);
  // The mismatch stays in the reason until the release.
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonFirmwareIncompatible);
  EXPECT_FALSE(DrivesOrMows(0.5));
  control_.ReleaseEmergency(now_);
  Run(0.6);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  EXPECT_TRUE(DrivesOrMows(0.8));
}

TEST_F(FirmwareControlTest, ReconnectForgetsTheVersion) {
  Run(0.1);
  control_.OnVersions(Versions(36, 34));
  EXPECT_TRUE(DrivesOrMows(0.3));
  control_.OnConnected(now_);
  EXPECT_TRUE(control_.Status(now_).firmware_incompatible);
  EXPECT_FALSE(DrivesOrMows(0.8));
}

TEST_F(ControlTest, BladeRefusedByDefault) {
  Run(0.1);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kNotAllowed);
  EXPECT_EQ(control_.SetBlade(kBladeOff, now_), BladeResult::kOk);
  Run(1.0);
  EXPECT_TRUE(Times(IsBladeOn).empty());
}

TEST_F(BladeControlTest, BladeOnOnceOffThreeTimes) {
  Run(0.5);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  Run(1.0);
  EXPECT_EQ(control_.SetBlade(kBladeOff, now_), BladeResult::kOk);
  Run(2.0);
  auto on = Times(IsBladeOn);
  ASSERT_EQ(on.size(), 1u);
  for (const auto& s : sent_) {
    if (IsBladeOn(s.packet)) {
      EXPECT_EQ(CutterHeight(s.packet), 45);
      EXPECT_EQ(s.packet.payload[2], 100);
    }
  }
  EXPECT_EQ(Times(IsBladeOff, on[0]).size(), 3u);
  EXPECT_EQ(control_.Status(now_).blade_off_reason, BladeOffReason::kRequest);
}

TEST_F(BladeControlTest, LeaseRunsOutWithoutRenewal) {
  Run(0.5);
  ASSERT_EQ(control_.SetBlade(BladeOn(0.5), now_), BladeResult::kOk);
  Run(0.99);
  EXPECT_TRUE(control_.Status(now_).blade_on);
  Run(2.0);
  auto on = Times(IsBladeOn);
  ASSERT_EQ(on.size(), 1u);
  auto off = Times(IsBladeOff, on[0]);
  ASSERT_EQ(off.size(), 3u);
  EXPECT_NEAR(off[0], 1.0, 0.011);
  EXPECT_NEAR(off[2] - off[0], 0.2, 1e-6);
  EXPECT_FALSE(control_.Status(now_).blade_on);
  EXPECT_EQ(control_.Status(now_).blade_off_reason, BladeOffReason::kLeaseExpired);
}

TEST_F(BladeControlTest, RenewedLeaseKeepsOneOnFrame) {
  Run(0.5);
  for (int i = 0; i < 10; i++) {
    ASSERT_EQ(control_.SetBlade(BladeOn(0.5), now_), BladeResult::kOk);
    Run(now_ + 0.3);
  }
  EXPECT_TRUE(control_.Status(now_).blade_on);
  EXPECT_EQ(Times(IsBladeOn).size(), 1u);
  EXPECT_TRUE(Times(IsBladeOff, 0.5).empty());
  // The last renewal at 3.2 s holds until 3.7 s.
  Run(4.0);
  auto off = Times(IsBladeOff, 0.5);
  ASSERT_EQ(off.size(), 3u);
  EXPECT_NEAR(off[0], 3.7, 0.011);
}

TEST_F(BladeControlTest, NewSettingSendsOneNewOnFrame) {
  Run(0.5);
  control_.SetBlade(BladeOn(1.0, 80, 40), now_);
  Run(0.6);
  control_.SetBlade(BladeOn(1.0, 80, 40), now_);
  Run(0.7);
  control_.SetBlade(BladeOn(1.0, 60, 70), now_);
  Run(0.8);
  std::vector<SubPacket> on;
  for (const auto& s : sent_) {
    if (IsBladeOn(s.packet)) on.push_back(s.packet);
  }
  ASSERT_EQ(on.size(), 2u);
  EXPECT_EQ(on[0].payload[2], 80);
  EXPECT_EQ(CutterHeight(on[0]), 40);
  EXPECT_EQ(on[1].payload[2], 60);
  EXPECT_EQ(CutterHeight(on[1]), 70);
  EXPECT_TRUE(Times(IsBladeOff, 0.5).empty());
}

TEST_F(BladeControlTest, OutOfRangeRequestsAreRefused) {
  Run(0.5);
  EXPECT_EQ(control_.SetBlade(BladeOn(0.0), now_), BladeResult::kInvalidLease);
  EXPECT_EQ(control_.SetBlade(BladeOn(-1.0), now_), BladeResult::kInvalidLease);
  EXPECT_EQ(control_.SetBlade(BladeOn(2.5), now_), BladeResult::kInvalidLease);
  EXPECT_EQ(control_.SetBlade(BladeOn(std::nan("")), now_), BladeResult::kInvalidLease);
  EXPECT_EQ(control_.SetBlade(BladeOn(INFINITY), now_), BladeResult::kInvalidLease);
  EXPECT_EQ(control_.SetBlade(BladeOn(1.0, 101), now_), BladeResult::kInvalidSpeed);
  EXPECT_EQ(control_.SetBlade(BladeOn(1.0, 0, 29), now_), BladeResult::kInvalidHeight);
  EXPECT_EQ(control_.SetBlade(BladeOn(1.0, 0, 91), now_), BladeResult::kInvalidHeight);
  Run(1.0);
  EXPECT_TRUE(Times(IsBladeOn).empty());
}

TEST_F(BladeControlTest, BladeOffOnLiftAndNotBackOn) {
  Run(0.5);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  sensor_.lift = true;
  Run(1.5);
  sensor_.lift = false;
  Run(3.0);
  auto on = Times(IsBladeOn);
  ASSERT_EQ(on.size(), 1u);
  EXPECT_EQ(Times(IsBladeOff, on[0]).size(), 3u);
  EXPECT_FALSE(control_.Status(now_).blade_on);
  EXPECT_EQ(control_.Status(now_).blade_off_reason, BladeOffReason::kBlocked);
}

TEST_F(BladeControlTest, RenewalWhileBlockedEndsTheLease) {
  Run(0.5);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  control_.LatchEmergency(kReasonHighLevel);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kBlocked);
  Run(1.5);
  EXPECT_FALSE(control_.Status(now_).blade_on);
  EXPECT_EQ(Times(IsBladeOff, 0.5).size(), 3u);
}

TEST_F(BladeControlTest, ARefusalForAnEmergencyCarriesItsReasonBits) {
  uint32_t reason = 1;
  // No telemetry yet: blocked by the input timeout, which is no latch.
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_, &reason), BladeResult::kBlocked);
  EXPECT_EQ(reason, kReasonTimeoutInputs);
  Run(0.5);
  ASSERT_EQ(control_.SetBlade(BladeOn(), now_, &reason), BladeResult::kOk);
  EXPECT_EQ(reason, 0u);
  Run(1.0);
  // The stop button latches with the next tick; the renewal right after it is refused with the bits of hw/status.
  sensor_.stop = true;
  Run(1.01);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_, &reason), BladeResult::kBlocked);
  EXPECT_EQ(reason, kReasonLatch | kReasonStop);
  EXPECT_EQ(reason, control_.Status(now_).emergency_reason);
  EXPECT_EQ(control_.SetBlade(kBladeOff, now_, &reason), BladeResult::kOk);
  EXPECT_EQ(reason, 0u);
  // Other refusals carry none.
  EXPECT_EQ(control_.SetBlade(BladeOn(2.5), now_, &reason), BladeResult::kInvalidLease);
  EXPECT_EQ(reason, 0u);
  sensor_.stop = false;
  control_.ReleaseEmergency(now_);
  Run(1.5);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  reason = 1;
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_, &reason), BladeResult::kDocked);
  EXPECT_EQ(reason, 0u);
}

TEST_F(BladeControlTest, BladeRefusedWithoutTelemetryOrOnDock) {
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kBlocked);
  Run(0.5, false);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kBlocked);
  Run(1.0);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kDocked);
  Run(1.5);
  EXPECT_TRUE(Times(IsBladeOn).empty());
}

TEST_F(BladeControlTest, ReconnectEndsTheLease) {
  Run(0.5);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  control_.OnConnected(now_);
  Run(2.0);
  EXPECT_EQ(Times(IsBladeOn).size(), 1u);
  EXPECT_EQ(Times(IsBladeOff, 1.0).size(), 3u);
  EXPECT_FALSE(control_.Status(now_).blade_on);
  EXPECT_EQ(control_.Status(now_).blade_off_reason, BladeOffReason::kReconnect);
}

TEST_F(ControlTest, ChargeOnlyOnDebouncedContact) {
  Run(0.1);
  control_.OnDockContactSample(true);
  control_.OnDockContactSample(true);
  Run(1.0);
  EXPECT_TRUE(Times([](const SubPacket& p) { return IsModule(p, Module::kCharge); }).empty());
  control_.OnDockContactSample(true);
  Run(65.0);
  auto charges = Times([](const SubPacket& p) { return IsModule(p, Module::kCharge) && p.payload[0] == 1; });
  ASSERT_EQ(charges.size(), 3u);
  EXPECT_NEAR(charges[1] - charges[0], 30.0, 0.02);
  control_.OnDockContactSample(false);
  Run(130.0);
  EXPECT_EQ(Times([](const SubPacket& p) { return IsModule(p, Module::kCharge); }).size(), 3u);
}

bool IsChargeOn(const SubPacket& packet) {
  return IsModule(packet, Module::kCharge) && packet.payload[0] == 1;
}

bool IsChargeOff(const SubPacket& packet) {
  return IsModule(packet, Module::kCharge) && packet.payload[0] == 0;
}

TEST_F(ControlTest, ChargingOffOnRequestUntilTheContactEnds) {
  Run(0.1);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  Run(1.0);
  ASSERT_EQ(Times(IsChargeOn).size(), 1u);
  control_.SetCharging(false, now_);
  Run(70.0);
  // Three off frames 100 ms apart, then no Charge 1 while this contact lasts, not even after charge_resend_s.
  const auto offs = Times(IsChargeOff);
  ASSERT_EQ(offs.size(), 3u);
  EXPECT_NEAR(offs[0], 1.01, 0.005);
  EXPECT_NEAR(offs[2] - offs[0], 0.2, 0.005);
  EXPECT_EQ(Times(IsChargeOn).size(), 1u);
  EXPECT_TRUE(control_.Status(now_).dock_contact);
  // The contact ends and a new one begins: charging as before.
  control_.OnDockContactSample(false);
  Run(71.0);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  Run(72.0);
  EXPECT_EQ(Times(IsChargeOn, 71.0).size(), 1u);
  EXPECT_EQ(Times(IsChargeOff).size(), 3u);
}

TEST_F(ControlTest, ChargingAllowedAgainOnTheContactsStartsAtOnce) {
  Run(0.1);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  Run(1.0);
  control_.SetCharging(false, now_);
  Run(1.05);
  control_.SetCharging(true, now_);
  Run(2.0);
  // The off frames still due are dropped, Charge 1 follows in the next tick.
  EXPECT_EQ(Times(IsChargeOff).size(), 1u);
  const auto ons = Times(IsChargeOn);
  ASSERT_EQ(ons.size(), 2u);
  EXPECT_NEAR(ons[1], 1.06, 0.005);
}

TEST_F(ControlTest, ChargingOffGoesOutBeforeTheFirstMotion) {
  Run(0.1);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  Run(1.0);
  control_.SetCharging(false, now_);
  control_.OnCmdVel(0.15f, 0.0f, now_);
  const size_t before = sent_.size();
  Run(now_ + 0.01);
  size_t off = sent_.size();
  size_t motion = sent_.size();
  for (size_t i = before; i < sent_.size(); i++) {
    if (IsChargeOff(sent_[i].packet) && off == sent_.size()) off = i;
    if (IsMovingSpeed(sent_[i].packet) && motion == sent_.size()) motion = i;
  }
  ASSERT_LT(off, sent_.size());
  ASSERT_LT(motion, sent_.size());
  EXPECT_LT(off, motion);
}

TEST_F(ControlTest, ChargingOffWithoutContactHoldsNothingBack) {
  Run(0.1);
  control_.SetCharging(false, now_);
  Run(1.0);
  EXPECT_EQ(Times(IsChargeOff).size(), 3u);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  Run(1.5);
  EXPECT_EQ(Times(IsChargeOn).size(), 1u);
}

TEST_F(ControlTest, RawContactFollowsEachSampleTheDebouncedOneThree) {
  Run(0.1);
  control_.OnDockContactSample(true);
  EXPECT_TRUE(control_.Status(now_).dock_contact_raw);
  EXPECT_FALSE(control_.Status(now_).dock_contact);
  control_.OnDockContactSample(true);
  control_.OnDockContactSample(true);
  EXPECT_TRUE(control_.Status(now_).dock_contact);
  control_.OnDockContactSample(false);
  EXPECT_FALSE(control_.Status(now_).dock_contact_raw);
  EXPECT_FALSE(control_.Status(now_).dock_contact);
}

TEST_F(ControlTest, ImuSamplesForwardedOnceAndRateLimited) {
  Run(0.1);
  // 100 Hz IMU into a 100 Hz loop, forwarded at most every 20 ms.
  for (int i = 0; i < 100; i++) {
    ImuCounts imu;
    imu.yaw = static_cast<int16_t>(i);
    control_.OnImu(imu, now_);
    Run(now_ + 0.01);
  }
  // The last sample goes out on the next free slot.
  Run(now_ + 0.02);
  size_t forwarded = Times([](const SubPacket& p) { return IsModule(p, Module::kImu); }).size();
  EXPECT_GE(forwarded, 45u);
  EXPECT_LE(forwarded, 51u);
  // IMU stops, nothing stale is repeated.
  Run(now_ + 1.0);
  EXPECT_EQ(Times([](const SubPacket& p) { return IsModule(p, Module::kImu); }).size(), forwarded);
}

TEST_F(ControlTest, WithoutImuTimeoutNoImuIsNeeded) {
  EXPECT_TRUE(DrivesOrMows(0.5));
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
}

TEST_F(ImuControlTest, BlockedWithoutLatchUntilTheFirstSample) {
  Run(2.5);
  auto status = control_.Status(now_);
  EXPECT_TRUE(status.blocked);
  EXPECT_FALSE(status.latched);
  EXPECT_EQ(status.emergency_reason, kReasonImuTimeout);
  EXPECT_FALSE(DrivesOrMows(2.7));
  feed_imu_ = true;
  Run(2.8);
  EXPECT_FALSE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  EXPECT_TRUE(DrivesOrMows(3.0));
}

TEST_F(ImuControlTest, ARefusalWhileTheImuIsStillDueCarriesNoReason) {
  Run(0.5);
  uint32_t reason = 1;
  // Held, but no emergency: the logic may not pause for one.
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_, &reason), BladeResult::kBlocked);
  EXPECT_EQ(reason, 0u);
  Run(2.5);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_, &reason), BladeResult::kBlocked);
  EXPECT_EQ(reason, kReasonImuTimeout);
}

TEST_F(ImuControlTest, StillDueAfterTheStartHoldsWithoutAnEmergency) {
  Run(1.9);
  auto status = control_.Status(now_);
  EXPECT_FALSE(status.blocked);
  EXPECT_FALSE(status.latched);
  EXPECT_EQ(status.emergency_reason, 0u);
  // Held all the same; the first tick was at 0.01 s.
  EXPECT_FALSE(DrivesOrMows(2.0));
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  Run(2.03);
  status = control_.Status(now_);
  EXPECT_TRUE(status.blocked);
  EXPECT_FALSE(status.latched);
  EXPECT_EQ(status.emergency_reason, kReasonImuTimeout);
}

TEST_F(ImuControlTest, FirstSampleWithinTheStartTimeIsNeverReported) {
  for (int i = 0; i < 190; i++) {
    Run(now_ + 0.01);
    EXPECT_EQ(control_.Status(now_).emergency_reason, 0u) << now_;
    EXPECT_FALSE(control_.Status(now_).blocked) << now_;
  }
  feed_imu_ = true;
  for (int i = 0; i < 100; i++) {
    Run(now_ + 0.01);
    EXPECT_EQ(control_.Status(now_).emergency_reason, 0u) << now_;
  }
  EXPECT_TRUE(DrivesOrMows(3.1));
}

TEST_F(ImuControlTest, ReconnectDoesNotRestartTheStartTime) {
  Run(2.2);
  control_.OnConnected(now_);
  Run(2.3);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonImuTimeout);
}

TEST_F(ShortImuStartTest, TheStartTimeIsAtLeastTheImuTimeout) {
  Run(0.5);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  Run(0.53);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonImuTimeout);
}

TEST_F(ImuControlTest, GapLatchesAndStopsWheelsAndBlade) {
  // Within the start time as well: once the first sample came, a gap latches after imu_timeout_s.
  feed_imu_ = true;
  Run(0.5);
  ASSERT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.6);
  feed_imu_ = false;
  for (int i = 0; i < 10; i++) {
    control_.OnCmdVel(0.2f, 0.0f, now_);
    Run(now_ + 0.1);
  }
  auto status = control_.Status(now_);
  EXPECT_TRUE(status.latched);
  EXPECT_FALSE(status.blade_on);
  EXPECT_EQ(status.emergency_reason, kReasonLatch | kReasonImuTimeout);
  auto zeros = Times(IsZeroSpeed);
  ASSERT_EQ(zeros.size(), 3u);
  // The last sample came at 0.6 s.
  EXPECT_NEAR(zeros[0], 1.1, 0.02);
  EXPECT_TRUE(Times(IsMovingSpeed, zeros[0]).empty());
  EXPECT_EQ(Times(IsBladeOff, 0.5).size(), 3u);
}

TEST_F(ImuControlTest, ReleaseOnlyOnceTheImuIsBack) {
  feed_imu_ = true;
  Run(0.5);
  feed_imu_ = false;
  Run(1.5);
  control_.ReleaseEmergency(now_);
  Run(3.0);
  EXPECT_TRUE(control_.Status(now_).latched);
  feed_imu_ = true;
  Run(3.5);
  // Back again, the gap stays latched until released.
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonImuTimeout);
  EXPECT_FALSE(DrivesOrMows(3.8));
  control_.ReleaseEmergency(now_);
  Run(4.0);
  EXPECT_FALSE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  EXPECT_TRUE(DrivesOrMows(4.3));
}

TEST_F(BladeStopTest, ABladeThatRunsDownInTimeIsFine) {
  Run(0.5);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  control_.OnBladeRpm(2800, now_);
  control_.SetBlade(kBladeOff, now_);
  Run(1.5);
  control_.OnBladeRpm(2000, now_);
  // Off went out at 1.01 s, so 3.9 s is still within the 3 s to run down.
  Run(3.9);
  control_.OnBladeRpm(300, now_);
  Run(4.5);
  control_.OnBladeRpm(0, now_);
  Run(5.0);
  EXPECT_FALSE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
}

TEST_F(BladeStopTest, ABladeStillTurningLatchesAndIsSentOffAgain) {
  Run(0.5);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  control_.SetBlade(kBladeOff, now_);
  Run(4.5);
  control_.OnBladeRpm(1500, now_);
  auto status = control_.Status(now_);
  EXPECT_TRUE(status.blocked);
  EXPECT_EQ(status.emergency_reason, kReasonLatch | kReasonMowerNotStopped);
  Run(5.0);
  EXPECT_EQ(Times(IsBladeOff, 4.5).size(), 3u);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kBlocked);
  // While it turns a release does not lift the block, and the next report sends off again.
  control_.ReleaseEmergency(now_);
  Run(5.5);
  control_.OnBladeRpm(1500, now_);
  Run(7.0);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_EQ(Times(IsBladeOff, 4.5).size(), 6u);
  control_.OnBladeRpm(0, now_);
  control_.ReleaseEmergency(now_);
  Run(7.2);
  EXPECT_FALSE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
}

TEST_F(BladeStopTest, ABladeStillTurningStaysLatchedAfterAReconnect) {
  Run(0.5);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  control_.SetBlade(kBladeOff, now_);
  Run(4.5);
  control_.OnBladeRpm(1500, now_);
  Run(5.0);
  // The port reopens: the off frames go out again, the blade keeps turning.
  control_.OnConnected(now_);
  Run(5.5);
  control_.OnBladeRpm(1500, now_);
  control_.ReleaseEmergency(now_);
  Run(6.0);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonMowerNotStopped);
  EXPECT_FALSE(DrivesOrMows(6.5));
}

TEST_F(BladeStopTest, ABladeTurningAtTheStartLatches) {
  // Never switched on from here; the start sent it off at 0.01 s.
  Run(3.5);
  control_.OnBladeRpm(2800, now_);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonMowerNotStopped);
}

TEST_F(BladeControlTest, NeverSendsPowerOff) {
  const SubPacket power_off = PowerOffPacket();
  for (int i = 0; i < 200; i++) {
    control_.OnCmdVel(i % 3 ? 0.3f : 0.0f, i % 5 ? -0.3f : 0.0f, now_);
    if (i % 7 == 0) control_.ReleaseEmergency(now_);
    if (i % 11 == 0) control_.SetBlade(i % 22 == 0 ? BladeOn(0.3) : kBladeOff, now_);
    if (i % 13 == 0) control_.LatchEmergency(kReasonHighLevel);
    control_.OnDockContactSample(i % 17 < 5);
    sensor_.lift = i % 19 == 0;
    sensor_.stop = i % 23 == 0;
    Run(now_ + 0.05);
  }
  for (const auto& s : sent_) {
    EXPECT_FALSE(s.packet.type == power_off.type && s.packet.module == power_off.module &&
                 s.packet.payload == power_off.payload);
  }
}

TEST_F(BladeControlTest, HoldBlocksMotionAndBladeWithoutEmergency) {
  Run(0.5);
  control_.SetHold(true);
  EXPECT_FALSE(DrivesOrMows(2.0));
  uint32_t reason = 99;
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_, &reason), BladeResult::kHeld);
  EXPECT_EQ(reason, 0u);
  const McuControlStatus status = control_.Status(now_);
  EXPECT_TRUE(status.held);
  EXPECT_FALSE(status.blocked);
  EXPECT_EQ(status.emergency_reason, 0u);
  EXPECT_FALSE(status.latched);
  // Ends without a release: the next command drives again.
  control_.SetHold(false);
  EXPECT_FALSE(control_.Status(now_).held);
  EXPECT_TRUE(DrivesOrMows(3.0));
}

TEST_F(BladeControlTest, HoldStopsAMovingMowerAndTheBlade) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  control_.SetBlade(BladeOn(), now_);
  Run(0.2);
  ASSERT_EQ(Times(IsMovingSpeed).size(), 1u);
  ASSERT_EQ(Times(IsBladeOn).size(), 1u);
  control_.SetHold(true);
  for (int i = 0; i < 10; i++) {
    control_.OnCmdVel(0.2f, 0.0f, now_);
    control_.SetBlade(BladeOn(), now_);
    Run(now_ + 0.1);
  }
  EXPECT_EQ(Times(IsMovingSpeed).size(), 1u);
  EXPECT_EQ(Times(IsZeroSpeed, 0.2).size(), 3u);
  EXPECT_EQ(Times(IsBladeOff, 0.2).size(), 3u);
  EXPECT_EQ(Times(IsBladeOn).size(), 1u);
  EXPECT_FALSE(control_.Status(now_).moving);
  EXPECT_EQ(control_.Status(now_).blade_off_reason, BladeOffReason::kBlocked);
}

TEST_F(BladeControlTest, AnEmergencyDuringAHoldIsReportedAsSuch) {
  Run(0.5);
  control_.SetHold(true);
  sensor_.lift = true;
  Run(0.6);
  uint32_t reason = 0;
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_, &reason), BladeResult::kBlocked);
  EXPECT_NE(reason & kReasonLift, 0u);
  EXPECT_TRUE(control_.Status(now_).blocked);
  EXPECT_TRUE(control_.Status(now_).held);
}

TEST_F(ControlTest, HoldLetsChargingGoOn) {
  control_.SetHold(true);
  Run(0.1);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  Run(31.0);
  EXPECT_EQ(Times(IsChargeOn).size(), 2u);
  EXPECT_TRUE(Times([](const SubPacket& p) { return IsModule(p, Module::kSpeed); }).empty());
}

TEST_F(ControlTest, HoldKeepsTheHeartbeat) {
  control_.SetHold(true);
  Run(1.0);
  EXPECT_EQ(Times(IsHeartbeat).size(), 10u);
}

TEST_F(ControlTest, HoldOnTheDockSendsNoSpeedData) {
  Run(0.1);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  control_.SetHold(true);
  for (int i = 0; i < 20; i++) {
    control_.OnCmdVel(0.0f, 0.0f, now_);
    Run(now_ + 0.1);
  }
  control_.SetHold(false);
  Run(now_ + 1.0);
  EXPECT_TRUE(Times([](const SubPacket& p) { return IsModule(p, Module::kSpeed); }).empty());
}
