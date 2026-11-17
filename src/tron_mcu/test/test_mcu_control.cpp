#include <gtest/gtest.h>

#include <cmath>
#include <iterator>

#include "tron_mcu/mcu_control.h"
#include "tron_mcu/mcu_journal.h"

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

// The setting of the Tron: the MCU drops a setpoint ~0.2 s after the last SpeedData.
class RepeatTest : public ControlTest {
 protected:
  static McuControlConfig RepeatConfig(double cmd_vel_timeout_s = 0.5) {
    McuControlConfig config;
    config.cmd_vel_timeout_s = cmd_vel_timeout_s;
    config.speed_repeat_window_s = 0.3;
    return config;
  }
  explicit RepeatTest(double cmd_vel_timeout_s = 0.5) : ControlTest(RepeatConfig(cmd_vel_timeout_s)) {
  }
};

class ShortTimeoutRepeatTest : public RepeatTest {
 protected:
  ShortTimeoutRepeatTest() : RepeatTest(0.2) {
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

TEST_F(ControlTest, CommandsAt20HzGoOutEvery100msWithTheNewestSetpoint) {
  Run(0.1);
  for (int i = 1; i <= 40; i++) {
    control_.OnCmdVel(0.01f * static_cast<float>(i), 0.0f, now_);
    Run(now_ + 0.05);
  }
  auto moving = Times(IsMovingSpeed);
  ASSERT_EQ(moving.size(), 20u);
  for (size_t i = 1; i < moving.size(); i++) EXPECT_NEAR(moving[i] - moving[i - 1], 0.1, 1e-6);
  // Each frame carries the command that came last before it, the one in between is dropped.
  std::vector<std::vector<uint8_t>> payloads;
  for (const auto& s : sent_) {
    if (IsMovingSpeed(s.packet)) payloads.push_back(s.packet.payload);
  }
  EXPECT_EQ(payloads.front(), SpeedPacket(0.01f, 0.0f)->payload);
  EXPECT_EQ(payloads[1], SpeedPacket(0.03f, 0.0f)->payload);
  EXPECT_EQ(payloads.back(), SpeedPacket(0.39f, 0.0f)->payload);
  EXPECT_TRUE(Times(IsZeroSpeed).empty());
}

TEST_F(ControlTest, AStopRightAfterAFrameGoesOutAtOnce) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.12);
  control_.OnCmdVel(0.0f, 0.0f, now_);
  Run(0.13);
  auto zeros = Times(IsZeroSpeed);
  ASSERT_EQ(zeros.size(), 1u);
  EXPECT_NEAR(zeros[0], 0.13, 1e-6);
}

TEST_F(ControlTest, MotionRightAfterAStopFrameWaits) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.5);
  control_.OnCmdVel(0.0f, 0.0f, now_);
  Run(0.51);
  control_.OnCmdVel(0.1f, 0.0f, now_);
  Run(1.0);
  auto zeros = Times(IsZeroSpeed);
  ASSERT_EQ(zeros.size(), 1u);
  auto moving = Times(IsMovingSpeed, 0.5);
  ASSERT_EQ(moving.size(), 1u);
  EXPECT_NEAR(moving[0] - zeros[0], 0.1, 1e-6);
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
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
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
    control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
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

// One bit per input of hw/inputs in its order: stop, lift, bumper_left, bumper_right.
unsigned ActiveInputs(const McuControl& control, double now) {
  unsigned active = 0;
  const auto inputs = control.Inputs(now);
  for (size_t i = 0; i < inputs.size(); i++) active |= static_cast<unsigned>(inputs[i].active) << i;
  return active;
}

TEST_F(ControlTest, InputsNameStopLiftAndEachBumper) {
  Run(0.1);
  const auto inputs = control_.Inputs(now_);
  EXPECT_STREQ(inputs[0].name, "stop");
  EXPECT_STREQ(inputs[0].reason, "stop");
  EXPECT_STREQ(inputs[1].name, "lift");
  EXPECT_STREQ(inputs[1].reason, "lift");
  EXPECT_STREQ(inputs[2].name, "bumper_left");
  EXPECT_STREQ(inputs[3].name, "bumper_right");
  for (size_t i = 0; i < inputs.size(); i++) {
    // The MCU keeps stop and lift until a clear frame, a bumper goes with its sensor.
    EXPECT_EQ(inputs[i].held, i < 2) << i;
    if (i >= 2) {
      EXPECT_STREQ(inputs[i].reason, "collision");
    }
  }
  EXPECT_EQ(ActiveInputs(control_, now_), 0u);

  bool SensorInfo::* const flags[] = {&SensorInfo::stop, &SensorInfo::lift, &SensorInfo::bumper_left,
                                      &SensorInfo::bumper_right};
  for (size_t i = 0; i < std::size(flags); i++) {
    sensor_ = SensorInfo{};
    sensor_.*flags[i] = true;
    Run(now_ + 0.01);
    EXPECT_EQ(ActiveInputs(control_, now_), 1u << i) << i;
  }
  // The sum byte of the bumpers alone names no side.
  sensor_ = SensorInfo{};
  sensor_.bumper = true;
  Run(now_ + 0.01);
  EXPECT_TRUE(control_.Status(now_).bumper);
  EXPECT_EQ(ActiveInputs(control_, now_), 0u);
}

TEST_F(ControlTest, NoInputIsActiveWithoutFreshTelemetry) {
  EXPECT_EQ(ActiveInputs(control_, 0.0), 0u);
  sensor_.stop = sensor_.lift = sensor_.bumper_left = sensor_.bumper_right = true;
  Run(0.1);
  EXPECT_EQ(ActiveInputs(control_, now_), 0xfu);
  Run(now_ + 1.5, false);
  ASSERT_TRUE(control_.Status(now_).telemetry_stale);
  EXPECT_EQ(ActiveInputs(control_, now_), 0u);
}

TEST_F(ControlTest, ANonLatchingBumperNamesNoEmergency) {
  McuControlConfig config;
  config.bumper_latches_emergency = false;
  const McuControl control(config);
  const auto inputs = control.Inputs(0.0);
  EXPECT_STREQ(inputs[2].reason, "");
  EXPECT_STREQ(inputs[3].reason, "");
  EXPECT_STREQ(inputs[0].reason, "stop");
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
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
  Run(3.0);
  EXPECT_TRUE(control_.Status(now_).latched);
  feed_imu_ = true;
  Run(3.5);
  // Back again, the gap stays latched until released.
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonImuTimeout);
  EXPECT_FALSE(DrivesOrMows(3.8));
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
  Run(5.5);
  control_.OnBladeRpm(1500, now_);
  Run(7.0);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_EQ(Times(IsBladeOff, 4.5).size(), 6u);
  control_.OnBladeRpm(0, now_);
  control_.ReleaseEmergency(now_, 0);
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
  control_.ReleaseEmergency(now_, 0);
  Run(6.0);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonMowerNotStopped);
  EXPECT_FALSE(DrivesOrMows(6.5));
}

TEST_F(BladeStopTest, AfterAReconnectOnlyAReportBelowTheStillRpmClears) {
  Run(0.5);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  control_.SetBlade(kBladeOff, now_);
  Run(4.5);
  control_.OnBladeRpm(1500, now_);
  Run(5.0);
  control_.OnConnected(now_);
  Run(5.5);
  // Within the new run-down: exactly the still rpm still turns, one less is still.
  control_.OnBladeRpm(100, now_);
  control_.ReleaseEmergency(now_, 0);
  Run(6.0);
  EXPECT_TRUE(control_.Status(now_).latched);
  control_.OnBladeRpm(99, now_);
  control_.ReleaseEmergency(now_, 0);
  Run(6.5);
  EXPECT_FALSE(control_.Status(now_).latched);
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
    if (i % 7 == 0) control_.ReleaseEmergency(now_, 0);
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

// Limits on the 10 ms grid: times where a limit falls exactly on a tick, or just off it by rounding.

TEST_F(ControlTest, ALatchBetweenReleaseAndTickStays) {
  Run(0.1);
  control_.LatchEmergency(kReasonHighLevel);
  Run(0.2);
  control_.ReleaseEmergency(now_, 0);
  control_.LatchEmergency(kReasonTilt);
  Run(0.5);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonHighLevel | kReasonTilt);
}

TEST_F(ControlTest, ALiftRightAfterAReleaseStaysLatched) {
  Run(0.1);
  control_.LatchEmergency(kReasonHighLevel);
  Run(0.2);
  control_.ReleaseEmergency(now_, 0);
  Run(0.3);
  ASSERT_FALSE(control_.Status(now_).latched);
  // A short lift within the second after the release.
  sensor_.lift = true;
  Run(0.32);
  sensor_.lift = false;
  Run(0.6);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonLift);
}

TEST_F(ControlTest, OneReleaseSendsOneClearWhileTheLiftStays) {
  Run(0.1);
  sensor_.lift = true;
  Run(0.5);
  control_.ReleaseEmergency(now_, 0);
  Run(5.0);
  EXPECT_EQ(Times(IsClear).size(), 1u);
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(ControlTest, ZerosFollowAClearOnlyAfterRecentMotion) {
  Run(6.0);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(6.2);
  control_.OnCmdVel(0.0f, 0.0f, now_);
  Run(7.0);
  sensor_.lift = true;
  Run(8.0);
  control_.ReleaseEmergency(now_, 0);
  Run(8.5);
  auto clears = Times(IsClear);
  ASSERT_EQ(clears.size(), 1u);
  // 2 s after the last motion: three zeros after the clear.
  EXPECT_EQ(Times(IsZeroSpeed, clears[0]).size(), 3u);
  sensor_.lift = false;
  Run(14.0);
  sensor_.lift = true;
  Run(15.0);
  control_.ReleaseEmergency(now_, 0);
  Run(15.5);
  clears = Times(IsClear);
  ASSERT_EQ(clears.size(), 2u);
  // 9 s after it: none.
  EXPECT_TRUE(Times(IsZeroSpeed, clears[1]).empty());
}

TEST_F(ControlTest, NoZerosAfterAClearExactlyFiveSecondsAfterTheMotion) {
  Run(0.02);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.1);
  control_.OnCmdVel(0.0f, 0.0f, now_);
  Run(1.0);
  sensor_.lift = true;
  Run(5.02);
  // The motion went out at 0.03 s, the clear goes out at 5.03 s.
  control_.ReleaseEmergency(now_, 0);
  Run(5.5);
  auto clears = Times(IsClear);
  ASSERT_EQ(clears.size(), 1u);
  EXPECT_NEAR(clears[0], 5.03, 1e-9);
  EXPECT_TRUE(Times(IsZeroSpeed, clears[0]).empty());
}

TEST_F(ControlTest, ASecondClearAfterTheMinimumInterval) {
  Run(0.1);
  sensor_.lift = true;
  Run(0.15);
  control_.ReleaseEmergency(now_, 0);
  // Clears at 0.16 s and 1.16 s, 0.9999999999999999 s apart in doubles.
  Run(1.15);
  control_.ReleaseEmergency(now_, 0);
  Run(1.5);
  auto clears = Times(IsClear);
  ASSERT_EQ(clears.size(), 2u);
  EXPECT_NEAR(clears[1], 1.16, 1e-9);
}

TEST_F(ControlTest, TheReleaseWindowLastsUpToItsEnd) {
  sensor_.lift = true;
  Run(0.01);
  control_.ReleaseEmergency(now_, 0);
  // The window ends at exactly 1.01 s, the lift drops right after.
  Run(1.01);
  sensor_.lift = false;
  Run(1.02);
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(ControlTest, ACommandBeforeAReconnectIsDropped) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  control_.OnConnected(now_);
  Run(1.0);
  EXPECT_TRUE(Times(IsMovingSpeed).empty());
}

TEST_F(ControlTest, AReconnectStopsAMovingMowerAtOnceAndAStandingOneGetsNoZeros) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  control_.OnConnected(now_);
  Run(0.25);
  auto zeros = Times(IsZeroSpeed);
  ASSERT_FALSE(zeros.empty());
  EXPECT_NEAR(zeros[0], 0.21, 1e-9);
  Run(5.0);
  control_.OnConnected(now_);
  Run(6.0);
  EXPECT_TRUE(Times(IsZeroSpeed, 5.0).empty());
}

TEST_F(ControlTest, AReconnectSendsAHeartbeatAtOnce) {
  Run(0.15);
  control_.OnConnected(now_);
  Run(0.16);
  EXPECT_EQ(Times(IsHeartbeat, 0.155, 0.16).size(), 1u);
}

TEST_F(ControlTest, HeartbeatKeepsTenHertzOnLateTicks) {
  // Ticks every 70 ms: a heartbeat is due every 100 ms of the schedule, not 100 ms after the last late one.
  for (int i = 1; i <= 100; i++) {
    control_.OnSensorInfo(sensor_, i * 0.07);
    for (auto& packet : control_.Tick(i * 0.07, kWall)) sent_.push_back({i * 0.07, packet});
  }
  const size_t beats = Times(IsHeartbeat).size();
  EXPECT_GE(beats, 69u);
  EXPECT_LE(beats, 71u);
}

TEST_F(ControlTest, HeartbeatEvery100msForLong) {
  Run(30.0);
  auto times = Times(IsHeartbeat);
  ASSERT_EQ(times.size(), 300u);
  for (size_t i = 1; i < times.size(); i++) EXPECT_NEAR(times[i] - times[i - 1], 0.1, 1e-6);
}

TEST_F(ControlTest, TheFirstTicksSendOnlyHeartbeatAndBladeOff) {
  Run(0.5);
  for (const auto& s : sent_) EXPECT_TRUE(IsHeartbeat(s.packet) || IsBladeOff(s.packet)) << s.t;
}

TEST_F(ControlTest, TelemetryExactlyAtTheTimeoutIsStillFresh) {
  Run(0.01);
  Run(1.01, false);
  EXPECT_FALSE(control_.Status(now_).telemetry_stale);
  Run(1.02, false);
  EXPECT_TRUE(control_.Status(now_).telemetry_stale);
}

TEST_F(ControlTest, StatusShowsTheFlagsAndFreshTelemetry) {
  Run(0.1);
  EXPECT_FALSE(control_.Status(now_).telemetry_stale);
  sensor_.stop = true;
  Run(0.2);
  EXPECT_TRUE(control_.Status(now_).stop);
}

TEST_F(ControlTest, BetweenSensorInfoAndTick) {
  Run(0.1);
  // Not blocked yet: no reason.
  SensorInfo lifted = sensor_;
  lifted.lift = true;
  control_.OnSensorInfo(lifted, now_);
  EXPECT_FALSE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  control_.OnSensorInfo(sensor_, now_);
  control_.LatchEmergency(kReasonHighLevel);
  Run(0.2);
  // Blocked already: the new flags show before the tick latches them.
  SensorInfo flags = sensor_;
  flags.stop = true;
  flags.lift = true;
  flags.bumper = true;
  control_.OnSensorInfo(flags, now_);
  EXPECT_EQ(control_.Status(now_).emergency_reason,
            kReasonLatch | kReasonHighLevel | kReasonStop | kReasonLift | kReasonCollision);
}

TEST_F(ControlTest, StaleTelemetryAddsItsBitToTheLatchedOnes) {
  Run(0.1);
  control_.LatchEmergency(kReasonTilt);
  Run(0.2);
  Run(1.5, false);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonTilt | kReasonTimeoutInputs);
}

TEST_F(ControlTest, ABumperAddsItsBitToTheLatchedOnes) {
  Run(0.1);
  control_.LatchEmergency(kReasonFence);
  Run(0.2);
  sensor_.bumper = true;
  Run(0.3);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonFence | kReasonCollision);
}

TEST_F(ControlTest, AfterAReleaseOnlyTheNewCauseShows) {
  Run(0.1);
  control_.LatchEmergency(kReasonHighLevel);
  Run(0.2);
  control_.ReleaseEmergency(now_, 0);
  Run(0.3);
  Run(1.5, false);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonTimeoutInputs);
}

TEST_F(ControlTest, TheDebounceStartsAgainAfterALostSample) {
  Run(0.1);
  control_.OnDockContactSample(true);
  control_.OnDockContactSample(true);
  control_.OnDockContactSample(false);
  control_.OnDockContactSample(true);
  control_.OnDockContactSample(true);
  Run(0.5);
  EXPECT_FALSE(control_.Status(now_).dock_contact);
  EXPECT_TRUE(Times(IsChargeOn).empty());
  control_.OnDockContactSample(true);
  Run(0.6);
  EXPECT_TRUE(control_.Status(now_).dock_contact);
  EXPECT_EQ(Times(IsChargeOn).size(), 1u);
}

TEST_F(ControlTest, AContactAgainSoonChargesAtOnce) {
  Run(0.1);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  Run(1.0);
  control_.OnDockContactSample(false);
  Run(2.0);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  Run(2.1);
  auto charges = Times(IsChargeOn);
  ASSERT_EQ(charges.size(), 2u);
  EXPECT_NEAR(charges[1], 2.01, 1e-9);
}

TEST_F(ControlTest, AllowingChargingAgainSendsItAtOnce) {
  Run(0.1);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  Run(1.0);
  control_.SetCharging(true, now_);
  Run(1.1);
  auto charges = Times(IsChargeOn);
  ASSERT_EQ(charges.size(), 2u);
  EXPECT_NEAR(charges[1], 1.01, 1e-9);
}

TEST_F(ControlTest, ChargeResentAfterThirtySecondsOnTheTick) {
  Run(2.0);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  // Charge 1 at 2.01 s and 32.01 s, 29.999999999999996 s apart in doubles.
  Run(32.05);
  auto charges = Times(IsChargeOn);
  ASSERT_EQ(charges.size(), 2u);
  EXPECT_NEAR(charges[1], 32.01, 1e-9);
}

TEST_F(ControlTest, ImuForwardedEveryTwentyMilliseconds) {
  for (int i = 1; i <= 10; i++) {
    control_.OnImu(ImuCounts{}, i * 0.01);
    Run(i * 0.01);
  }
  auto imu = Times([](const SubPacket& p) { return IsModule(p, Module::kImu); });
  ASSERT_EQ(imu.size(), 5u);
  for (size_t i = 0; i < imu.size(); i++) EXPECT_NEAR(imu[i], 0.01 + 0.02 * i, 1e-9);
}

TEST_F(BladeControlTest, ContactWhileTheBladeTurnsSwitchesItOff) {
  Run(0.5);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  Run(1.01);
  EXPECT_FALSE(control_.Status(now_).blade_on);
  EXPECT_EQ(control_.Status(now_).blade_off_reason, BladeOffReason::kDock);
  EXPECT_EQ(Times(IsBladeOff, 1.0).size(), 1u);
}

TEST_F(BladeControlTest, ARefusalEndsTheLeaseAlsoWhenItsCauseEndsBeforeTheTick) {
  Run(0.5);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  control_.SetHold(true);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kHeld);
  control_.SetHold(false);
  Run(1.1);
  EXPECT_FALSE(control_.Status(now_).blade_on);
  EXPECT_EQ(control_.Status(now_).blade_off_reason, BladeOffReason::kBlocked);
  // The same on the dock, with a contact that bounces before the tick.
  control_.SetBlade(BladeOn(), now_);
  Run(1.5);
  ASSERT_TRUE(control_.Status(now_).blade_on);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kDocked);
  control_.OnDockContactSample(false);
  Run(1.6);
  EXPECT_FALSE(control_.Status(now_).blade_on);
  EXPECT_EQ(control_.Status(now_).blade_off_reason, BladeOffReason::kDock);
}

TEST_F(BladeControlTest, TheBladeOnDuringTheStartBurstCancelsTheRestOfIt) {
  Run(0.05);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  auto on = Times(IsBladeOn);
  ASSERT_EQ(on.size(), 1u);
  EXPECT_TRUE(Times(IsBladeOff, on[0]).empty());
}

TEST_F(BladeControlTest, AnOffRightAfterABurstGoesOutAtOnce) {
  // The start burst sent off at 0.01, 0.11 and 0.21 s.
  Run(0.22);
  control_.SetBlade(BladeOn(), now_);
  Run(0.23);
  control_.SetBlade(kBladeOff, now_);
  Run(0.3);
  auto offs = Times(IsBladeOff, 0.225);
  ASSERT_FALSE(offs.empty());
  EXPECT_NEAR(offs[0], 0.24, 1e-9);
}

TEST_F(BladeControlTest, TheLeaseEndsOnTheTickOfItsEnd) {
  Run(0.41);
  control_.SetBlade(BladeOn(1.0), now_);
  // It ends at 1.4100000000000001 s; the tick at 1.41 s is within the tolerance.
  Run(1.45);
  auto offs = Times(IsBladeOff, 0.5);
  ASSERT_FALSE(offs.empty());
  EXPECT_NEAR(offs[0], 1.41, 1e-9);
}

TEST_F(BladeControlTest, RequestsAtTheLimitsAreAccepted) {
  Run(0.1);
  EXPECT_EQ(control_.SetBlade(BladeOn(2.0, 100, 30), now_), BladeResult::kOk);
  EXPECT_EQ(control_.SetBlade(BladeOn(2.0, 1, 90), now_), BladeResult::kOk);
}

class FloatLeaseTest : public ControlTest {
 protected:
  static McuControlConfig FloatLeaseConfig() {
    McuControlConfig config;
    config.allow_blade = true;
    config.blade_lease_max_s = 1.1;
    return config;
  }
  FloatLeaseTest() : ControlTest(FloatLeaseConfig()) {
  }
};

TEST_F(FloatLeaseTest, TheLongestLeaseAfterAFloatRoundTripIsAccepted) {
  Run(0.1);
  EXPECT_EQ(control_.SetBlade(BladeOn(static_cast<float>(1.1)), now_), BladeResult::kOk);
}

class OffHeightTest : public ControlTest {
 protected:
  static McuControlConfig OffHeightConfig() {
    McuControlConfig config;
    config.blade_off_height_mm = 70;
    return config;
  }
  OffHeightTest() : ControlTest(OffHeightConfig()) {
  }
};

TEST_F(OffHeightTest, OffFramesCarryTheConfiguredHeight) {
  Run(0.5);
  auto offs = Times(IsBladeOff);
  ASSERT_EQ(offs.size(), 3u);
  for (const auto& s : sent_) {
    if (IsBladeOff(s.packet)) {
      EXPECT_EQ(CutterHeight(s.packet), 70);
    }
  }
}

class OneVersionTest : public ControlTest {
 protected:
  static McuControlConfig OneVersionConfig() {
    McuControlConfig config;
    config.expected_chassis_version = "0.6.34";
    return config;
  }
  OneVersionTest() : ControlTest(OneVersionConfig()) {
  }
};

TEST_F(OneVersionTest, OneExpectedVersionIsCheckedAsWell) {
  Run(0.5);
  EXPECT_TRUE(control_.Status(now_).firmware_incompatible);
  EXPECT_FALSE(DrivesOrMows(1.0));
  VersionInfo versions;
  versions.chassis_board = {0, 6, 34};
  versions.cutter_board = {9, 9, 9};
  EXPECT_TRUE(control_.OnVersions(versions));
  EXPECT_FALSE(control_.Status(now_).firmware_incompatible);
}

TEST_F(FirmwareControlTest, BeforeTheFirstVersionFrameTheReasonSaysFirmware) {
  Run(0.5);
  EXPECT_TRUE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonFirmwareIncompatible);
}

TEST_F(ImuControlTest, AnImuThatNeverCameAddsItsBitToTheLatchedOnes) {
  Run(2.5);
  control_.LatchEmergency(kReasonFence);
  Run(2.6);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonFence | kReasonImuTimeout);
}

TEST_F(ImuControlTest, AnImuGapKeepsTheOtherLatchedReasons) {
  feed_imu_ = true;
  Run(0.5);
  control_.LatchEmergency(kReasonTilt);
  Run(0.6);
  feed_imu_ = false;
  Run(1.5);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonTilt | kReasonImuTimeout);
}

TEST_F(ImuControlTest, AnImuSampleExactlyTheTimeoutAgoIsNoGap) {
  feed_imu_ = true;
  Run(0.01);
  feed_imu_ = false;
  Run(0.51);
  EXPECT_FALSE(control_.Status(now_).latched);
  Run(0.52);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonImuTimeout);
}

TEST_F(ImuControlTest, TheStartTimeIncludesItsLastTick) {
  // The first tick at 0.01 s; 2.01 s is 2.0000000000000004 s later in doubles.
  Run(2.01);
  EXPECT_FALSE(control_.Status(now_).blocked);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  Run(2.02);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonImuTimeout);
}

TEST_F(BladeStopTest, AReportExactlyAtTheStopTimeCounts) {
  Run(0.01);
  control_.SetBlade(BladeOn(), now_);
  Run(0.03);
  control_.SetBlade(kBladeOff, now_);
  // Off went out at 0.04 s, the report comes exactly 3 s later.
  Run(3.04);
  control_.OnBladeRpm(1500, now_);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonMowerNotStopped);
}

TEST_F(BladeStopTest, ExactlyTheStillRpmCounts) {
  Run(3.5);
  control_.OnBladeRpm(100, now_);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonMowerNotStopped);
}

TEST_F(BladeStopTest, AReleaseDuringTheRunDownGoesThrough) {
  Run(0.5);
  control_.SetBlade(BladeOn(), now_);
  Run(1.0);
  control_.LatchEmergency(kReasonHighLevel);
  Run(1.5);
  control_.OnBladeRpm(2000, now_);
  Run(2.0);
  control_.OnBladeRpm(50, now_);
  control_.ReleaseEmergency(now_, 0);
  Run(2.1);
  EXPECT_FALSE(control_.Status(now_).latched);
}

class NoBladeStopCheckTest : public ControlTest {
 protected:
  static McuControlConfig NoCheckConfig() {
    McuControlConfig config;
    config.blade_stop_time_s = 0;
    return config;
  }
  NoBladeStopCheckTest() : ControlTest(NoCheckConfig()) {
  }
};

TEST_F(NoBladeStopCheckTest, AStopTimeOfZeroTurnsTheCheckOff) {
  Run(3.5);
  control_.OnBladeRpm(2800, now_);
  Run(4.0);
  EXPECT_FALSE(control_.Status(now_).blocked);
}

class ShortBladeStopTest : public ControlTest {
 protected:
  static McuControlConfig ShortStopConfig() {
    McuControlConfig config;
    config.blade_stop_time_s = 0.5;
    return config;
  }
  ShortBladeStopTest() : ControlTest(ShortStopConfig()) {
  }
};

TEST_F(ShortBladeStopTest, AShortStopTimeStillChecks) {
  Run(1.0);
  control_.OnBladeRpm(2800, now_);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonMowerNotStopped);
}

TEST_F(RepeatTest, OneCommandIsRepeatedEvery100msForTheWindowThenStopped) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.1f, now_);
  const double start = now_;
  Run(2.0);
  auto moving = Times(IsMovingSpeed);
  // The repeat at 0.3 s falls on the tick of the stop: either side of it is fine.
  ASSERT_GE(moving.size(), 3u);
  ASSERT_LE(moving.size(), 4u);
  for (size_t i = 1; i < moving.size(); i++) EXPECT_NEAR(moving[i] - moving[i - 1], 0.1, 1e-6);
  EXPECT_LE(moving.back() - start, 0.3 + 1e-6);
  // Every repeat carries the same setpoint.
  const auto expected = *SpeedPacket(0.2f, 0.1f);
  for (const auto& s : sent_) {
    if (IsMovingSpeed(s.packet)) {
      EXPECT_EQ(s.packet.payload, expected.payload);
    }
  }
  auto zeros = Times(IsZeroSpeed);
  ASSERT_EQ(zeros.size(), 3u);
  EXPECT_GE(zeros[0] - start, 0.3 - 1e-6);
  EXPECT_LT(zeros[0] - start, 0.32);
  EXPECT_FALSE(control_.Status(now_).moving);
}

TEST_F(RepeatTest, CommandsAt10HzAreNotRepeated) {
  Run(0.1);
  for (int i = 0; i < 30; i++) {
    control_.OnCmdVel(0.2f, 0.0f, now_);
    Run(now_ + 0.1);
  }
  auto moving = Times(IsMovingSpeed);
  EXPECT_EQ(moving.size(), 30u);
  for (size_t i = 1; i < moving.size(); i++) EXPECT_NEAR(moving[i] - moving[i - 1], 0.1, 1e-6);
  EXPECT_TRUE(Times(IsZeroSpeed).empty());
}

TEST_F(RepeatTest, ACommandGapIsFilledEvery100ms) {
  Run(0.1);
  for (int i = 0; i < 10; i++) {
    control_.OnCmdVel(0.2f, 0.0f, now_);
    Run(now_ + 0.25);
  }
  auto moving = Times(IsMovingSpeed);
  for (size_t i = 1; i < moving.size(); i++) EXPECT_GE(moving[i] - moving[i - 1], 0.1 - 1e-6);
  EXPECT_GT(moving.size(), 20u);
  EXPECT_TRUE(Times(IsZeroSpeed, 0, 2.5).empty());
}

TEST_F(RepeatTest, ANewSetpointReplacesTheRepeatedOne) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  control_.OnCmdVel(-0.1f, 0.0f, now_);
  const auto expected = *SpeedPacket(-0.1f, 0.0f);
  Run(0.4);
  for (const auto& s : sent_) {
    if (s.t > 0.2 + 1e-9 && IsMovingSpeed(s.packet)) {
      EXPECT_EQ(s.packet.payload, expected.payload);
    }
  }
}

TEST_F(RepeatTest, AZeroCommandEndsTheRepeat) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  control_.OnCmdVel(0.0f, 0.0f, now_);
  Run(1.0);
  EXPECT_TRUE(Times(IsMovingSpeed, 0.21).empty());
  EXPECT_EQ(Times(IsZeroSpeed).size(), 3u);
}

TEST_F(RepeatTest, ALiftEndsTheRepeat) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.15);
  sensor_.lift = true;
  Run(1.0);
  EXPECT_TRUE(Times(IsMovingSpeed, 0.16).empty());
  EXPECT_EQ(Times(IsZeroSpeed).size(), 3u);
}

TEST_F(RepeatTest, IdleAndOnTheDockNothingIsRepeated) {
  Run(0.1);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  for (int i = 0; i < 20; i++) {
    control_.OnCmdVel(0.0f, 0.0f, now_);
    Run(now_ + 0.1);
  }
  EXPECT_TRUE(Times([](const SubPacket& p) { return IsModule(p, Module::kSpeed); }).empty());
}

TEST_F(ShortTimeoutRepeatTest, AShorterCmdVelTimeoutStopsFirst) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  const double start = now_;
  Run(1.0);
  auto zeros = Times(IsZeroSpeed);
  ASSERT_EQ(zeros.size(), 3u);
  EXPECT_GE(zeros[0] - start, 0.2 - 1e-6);
  EXPECT_LT(zeros[0] - start, 0.22);
  EXPECT_LE(Times(IsMovingSpeed).back() - start, 0.2 + 1e-6);
}

// Journals from before the clear of STOP during an emergency replay with what they ran with: never cleared.
TEST(McuJournal, ConfigFromBeforeTheStopClearReplays) {
  tron::McuControlConfig config;
  config.stop_clear_period_s = 0.25;
  config.stop_clear_delay_s = 3.0;
  nlohmann::json j = config;
  EXPECT_EQ(j.get<tron::McuControlConfig>().stop_clear_delay_s, 3.0);
  j.erase("stop_clear_period_s");
  j.erase("stop_clear_delay_s");
  const auto old = j.get<tron::McuControlConfig>();
  EXPECT_EQ(old.stop_clear_period_s, 0.0);
  EXPECT_EQ(old.stop_clear_delay_s, tron::McuControlConfig{}.stop_clear_delay_s);
  // Every other field stays required.
  j.erase("release_window_s");
  EXPECT_THROW(j.get<tron::McuControlConfig>(), nlohmann::json::out_of_range);
}
