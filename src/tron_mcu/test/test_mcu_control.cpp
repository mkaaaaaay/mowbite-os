#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

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

  explicit ControlTest(McuControlConfig config = {}) : control_(config), start_stops_(config.repeat_count) {
  }

  void Run(double until, bool feed_sensor = true) {
    while (now_ < until - 1e-9) {
      now_ = ++step_ * 0.01;
      if (feed_sensor) {
        control_.OnFrame();
        control_.OnSensorInfo(sensor_, now_);
      }
      if (feed_imu_) control_.OnImu(ImuCounts{}, now_);
      if (feed_setup_) control_.OnSetupMotion(true, now_);
      if (feed_wheels_) control_.OnWheelSpeed(wheel_linear_, wheel_angular_, now_);
      for (auto& packet : control_.Tick(now_, kWall)) {
        if (!StartStop(packet)) sent_.push_back({now_, packet});
      }
    }
  }

  // The stop frames of the start, up to the first motion; StartStopTest keeps them, the others leave them out.
  bool StartStop(const SubPacket& packet) {
    if (IsMovingSpeed(packet)) start_stops_ = 0;
    if (keep_start_stops_ || start_stops_ <= 0 || !IsZeroSpeed(packet)) return false;
    --start_stops_;
    return true;
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
  // An active setup/motion with every step.
  bool feed_setup_ = false;
  // The speeds the MCU reports, with every step.
  bool feed_wheels_ = false;
  float wheel_linear_ = 0;
  float wheel_angular_ = 0;
  bool keep_start_stops_ = false;
  int start_stops_;
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
    config.cutter_firmware = {"0.6.36"};
    config.chassis_firmware = {"0.6.34"};
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

TEST_F(ControlTest, ReleaseWithLiftSendsZeroClearThenZeros) {
  Run(0.1);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  sensor_.lift = true;
  Run(1.0);
  control_.ReleaseEmergency(now_, 0);
  Run(1.3);
  auto clears = Times(IsClear);
  ASSERT_EQ(clears.size(), 1u);
  // A zero right before the clear in its frame order, three more after it.
  const auto clear = std::find_if(sent_.begin(), sent_.end(), [](const Sent& s) { return IsClear(s.packet); });
  ASSERT_NE(clear, sent_.begin());
  EXPECT_TRUE(IsZeroSpeed((clear - 1)->packet));
  EXPECT_NEAR((clear - 1)->t, clears[0], 1e-9);
  EXPECT_EQ(Times(IsZeroSpeed, clears[0]).size(), 4u);
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
  // Only zeros, a stop around each clear.
  EXPECT_TRUE(Times(IsMovingSpeed).empty());
  EXPECT_EQ(Times(IsZeroSpeed).size(), 4 * clears);
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
  // A frame without sensor info: heard, but no telemetry.
  control_.OnFrame();
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
  control.OnFrame();
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

TEST_F(ControlTest, WithoutFirmwareListsTheFirmwareIsNotChecked) {
  Run(0.1);
  EXPECT_FALSE(control_.Status(now_).firmware_incompatible);
  EXPECT_TRUE(control_.OnVersions(VersionInfo{}));
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.2);
  EXPECT_EQ(Times(IsMovingSpeed).size(), 1u);
}

TEST_F(FirmwareControlTest, BlockedUntilAVersionFrameMatches) {
  Run(0.1);
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
  EXPECT_FALSE(DrivesOrMows(0.8));
  // The start time counts from the reopen, not from the first tick.
  Run(3.2);
  EXPECT_FALSE(control_.Status(now_).firmware_incompatible);
  Run(3.4);
  EXPECT_TRUE(control_.Status(now_).firmware_incompatible);
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
  // No frame yet, the port never opened: blocked, which is no latch.
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_, &reason), BladeResult::kBlocked);
  EXPECT_EQ(reason, kReasonBoardNotHeard);
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

TEST_F(BladeControlTest, AMotorFaultSwitchesTheBladeOffAndRefusesItUntilItClears) {
  Run(0.5);
  ASSERT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  Run(1.0);
  control_.OnBladeMotorStatus(-6);
  Run(2.0);
  EXPECT_FALSE(control_.Status(now_).blade_on);
  EXPECT_EQ(control_.Status(now_).blade_off_reason, BladeOffReason::kMotorFault);
  EXPECT_EQ(Times(IsBladeOff, 1.0).size(), 3u);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kMotorFault);
  Run(2.5);
  EXPECT_EQ(Times(IsBladeOn).size(), 1u);
  control_.OnBladeMotorStatus(0);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  Run(3.0);
  EXPECT_EQ(Times(IsBladeOn).size(), 2u);
}

TEST_F(BladeControlTest, RunningAndLockingAreNoFault) {
  Run(0.5);
  ASSERT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  for (int status : {0, 1, 2}) control_.OnBladeMotorStatus(status);
  Run(1.0);
  EXPECT_TRUE(control_.Status(now_).blade_on);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
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

TEST_F(ControlTest, ZerosGoAroundEveryClearHoweverLongAgoTheMotionWas) {
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
  EXPECT_EQ(Times(IsZeroSpeed, clears[0]).size(), 4u);
  sensor_.lift = false;
  Run(30.0);
  sensor_.lift = true;
  Run(31.0);
  control_.ReleaseEmergency(now_, 0);
  Run(31.5);
  clears = Times(IsClear);
  ASSERT_EQ(clears.size(), 2u);
  EXPECT_EQ(Times(IsZeroSpeed, clears[1]).size(), 4u);
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

TEST_F(ControlTest, AReconnectStopsAMovingMowerAtOnceAndAStandingOneAsWell) {
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
  EXPECT_EQ(Times(IsZeroSpeed, 5.0).size(), 3u);
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
    control_.OnFrame();
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

TEST_F(ControlTest, TheFirstTicksSendOnlyHeartbeatBladeOffAndStop) {
  keep_start_stops_ = true;
  Run(0.5);
  for (const auto& s : sent_) EXPECT_TRUE(IsHeartbeat(s.packet) || IsBladeOff(s.packet) || IsZeroSpeed(s.packet)) << s.t;
}

TEST_F(ControlTest, TheFirstMotionEndsTheStopsOfTheStart) {
  keep_start_stops_ = true;
  Run(0.05);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.3);
  ASSERT_EQ(Times(IsZeroSpeed).size(), 1u);
  EXPECT_NEAR(Times(IsZeroSpeed)[0], 0.01, 1e-9);
  // The motion waits speed_period_s after the stop frame, as after any.
  ASSERT_FALSE(Times(IsMovingSpeed).empty());
  EXPECT_NEAR(Times(IsMovingSpeed)[0], 0.11, 1e-9);
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
    config.chassis_firmware = {"0.6.34"};
    return config;
  }
  OneVersionTest() : ControlTest(OneVersionConfig()) {
  }
};

TEST_F(OneVersionTest, OneListIsCheckedAsWell) {
  EXPECT_FALSE(DrivesOrMows(3.5));
  EXPECT_TRUE(control_.Status(now_).firmware_incompatible);
  VersionInfo versions;
  versions.chassis_board = {0, 6, 34};
  versions.cutter_board = {9, 9, 9};
  EXPECT_TRUE(control_.OnVersions(versions));
  EXPECT_FALSE(control_.Status(now_).firmware_incompatible);
}

class FirmwareListTest : public ControlTest {
 protected:
  static McuControlConfig ListConfig() {
    McuControlConfig config;
    config.cutter_firmware = {"0.6.34", "0.6.36"};
    config.chassis_firmware = {"0.6.34"};
    return config;
  }
  FirmwareListTest() : ControlTest(ListConfig()) {
  }
  static VersionInfo Versions(Version cutter, Version chassis) {
    VersionInfo v;
    v.cutter_board = cutter;
    v.chassis_board = chassis;
    return v;
  }
};

TEST_F(FirmwareListTest, EachMeasuredStandOfABoardPasses) {
  Run(0.1);
  EXPECT_TRUE(control_.OnVersions(Versions({0, 6, 34}, {0, 6, 34})));
  EXPECT_TRUE(DrivesOrMows(0.3));
  control_.OnConnected(now_);
  Run(0.6);
  EXPECT_TRUE(control_.OnVersions(Versions({0, 6, 36}, {0, 6, 34})));
  EXPECT_FALSE(control_.Status(now_).firmware_incompatible);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  EXPECT_TRUE(DrivesOrMows(0.9));
}

TEST_F(FirmwareListTest, AStandOutsideTheListBlocksMotionAndBlade) {
  Run(0.1);
  // A newer cutter firmware than any measured one, and a chassis firmware measured only for the cutter.
  EXPECT_FALSE(control_.OnVersions(Versions({0, 6, 37}, {0, 6, 34})));
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonFirmwareIncompatible);
  control_.ReleaseEmergency(now_, 0);
  Run(0.5);
  EXPECT_TRUE(control_.Status(now_).firmware_incompatible);
  EXPECT_FALSE(DrivesOrMows(1.0));
  control_.OnConnected(now_);
  Run(1.1);
  EXPECT_FALSE(control_.OnVersions(Versions({0, 6, 36}, {0, 6, 36})));
  EXPECT_TRUE(control_.Status(now_).firmware_incompatible);
  EXPECT_FALSE(DrivesOrMows(1.5));
}

TEST_F(FirmwareControlTest, AVersionFrameStillDueBlocksWithoutAnEmergency) {
  EXPECT_FALSE(DrivesOrMows(3.0));
  auto status = control_.Status(now_);
  EXPECT_FALSE(status.blocked);
  EXPECT_FALSE(status.firmware_incompatible);
  EXPECT_EQ(status.emergency_reason, 0u);
}

TEST_F(FirmwareControlTest, WithoutAVersionFrameInTheStartTimeTheReasonSaysFirmware) {
  Run(3.1);
  auto status = control_.Status(now_);
  EXPECT_TRUE(status.blocked);
  EXPECT_TRUE(status.firmware_incompatible);
  EXPECT_FALSE(status.latched);
  EXPECT_EQ(status.emergency_reason, kReasonFirmwareIncompatible);
  // A late matching frame lifts the block without a release.
  EXPECT_TRUE(control_.OnVersions(Versions(36, 34)));
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  EXPECT_TRUE(DrivesOrMows(3.3));
}

TEST_F(FirmwareControlTest, AMismatchInTheStartTimeIsReportedAtOnce) {
  Run(0.5);
  EXPECT_FALSE(control_.OnVersions(Versions(36, 35)));
  auto status = control_.Status(now_);
  EXPECT_TRUE(status.blocked);
  EXPECT_TRUE(status.firmware_incompatible);
  EXPECT_EQ(status.emergency_reason, kReasonLatch | kReasonFirmwareIncompatible);
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

class StartLatchedTest : public ControlTest {
 protected:
  static McuControlConfig StartLatchedConfig(double imu_timeout_s = 0) {
    McuControlConfig config;
    config.allow_blade = true;
    config.start_latched = true;
    config.imu_timeout_s = imu_timeout_s;
    config.imu_start_timeout_s = 2.0;
    return config;
  }
  explicit StartLatchedTest(double imu_timeout_s = 0) : ControlTest(StartLatchedConfig(imu_timeout_s)) {
  }
};

class StartLatchedImuTest : public StartLatchedTest {
 protected:
  StartLatchedImuTest() : StartLatchedTest(0.5) {
  }
};

TEST_F(StartLatchedTest, LatchedWithHighLevelBeforeTheFirstTick) {
  auto status = control_.Status(0.0);
  EXPECT_TRUE(status.latched);
  EXPECT_TRUE(status.blocked);
  EXPECT_EQ(status.latch_count, 1u);
  EXPECT_TRUE(status.emergency_reason & kReasonHighLevel);
  Run(0.01);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonHighLevel);
  EXPECT_EQ(control_.Status(now_).latch_count, 1u);
}

TEST_F(StartLatchedTest, TheFirstTickSendsHeartbeatStopAndBladeOff) {
  keep_start_stops_ = true;
  Run(0.01);
  EXPECT_EQ(Times(IsHeartbeat).size(), 1u);
  EXPECT_EQ(Times(IsZeroSpeed).size(), 1u);
  EXPECT_EQ(Times(IsBladeOff).size(), 1u);
  Run(1.0);
  auto zeros = Times(IsZeroSpeed);
  ASSERT_EQ(zeros.size(), 3u);
  EXPECT_NEAR(zeros[0], 0.01, 1e-9);
  EXPECT_NEAR(zeros[1], 0.11, 1e-9);
  EXPECT_NEAR(zeros[2], 0.21, 1e-9);
  EXPECT_EQ(Times(IsBladeOff).size(), 3u);
  for (const auto& s : sent_) EXPECT_TRUE(IsHeartbeat(s.packet) || IsZeroSpeed(s.packet) || IsBladeOff(s.packet)) << s.t;
}

TEST_F(StartLatchedTest, NoMotionOrBladeUntilTheRelease) {
  Run(0.1);
  // A follower that still runs keeps commanding.
  for (int i = 0; i < 40; i++) {
    control_.OnCmdVel(0.2f, 0.1f, now_);
    uint32_t reason = 0;
    EXPECT_EQ(control_.SetBlade(BladeOn(), now_, &reason), BladeResult::kBlocked);
    EXPECT_EQ(reason, kReasonLatch | kReasonHighLevel);
    Run(now_ + 0.05);
  }
  EXPECT_TRUE(Times(IsMovingSpeed).empty());
  EXPECT_TRUE(Times(IsBladeOn).empty());
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_TRUE(control_.ReleaseEmergency(now_, 1));
  Run(now_ + 0.05);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  // What was commanded before the release is gone.
  Run(now_ + 0.5);
  EXPECT_TRUE(Times(IsMovingSpeed).empty());
  EXPECT_TRUE(DrivesOrMows(now_ + 0.5));
}

TEST_F(StartLatchedTest, AReconnectSendsTheStopFramesAgain) {
  Run(1.0);
  control_.OnConnected(now_);
  Run(2.0);
  EXPECT_EQ(Times(IsZeroSpeed, 1.0).size(), 3u);
  EXPECT_EQ(Times(IsBladeOff, 1.0).size(), 3u);
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(StartLatchedTest, ALatchWithTheSameBitCountsOnce) {
  control_.LatchEmergency(kReasonHighLevel);
  Run(0.1);
  EXPECT_EQ(control_.Status(now_).latch_count, 1u);
}

TEST_F(StartLatchedTest, AReleaseWithoutTelemetryHasNoEffect) {
  control_.OnFrame();
  Run(0.5, false);
  EXPECT_TRUE(control_.ReleaseEmergency(now_, 1));
  Run(2.0, false);
  EXPECT_TRUE(control_.Status(now_).latched);
  Run(2.5);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_FALSE(DrivesOrMows(now_ + 0.5));
}

TEST_F(StartLatchedTest, AReleaseBeforeTheBoardWasHeardHasNoEffect) {
  control_.OnConnected(0.0);
  EXPECT_TRUE(control_.ReleaseEmergency(now_, 1));
  Run(0.5, false);
  Run(1.0);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_FALSE(DrivesOrMows(now_ + 0.5));
}

TEST_F(BladeControlTest, NotAByteBeforeTheFirstFrame) {
  control_.OnConnected(0.0);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  control_.SetBlade(BladeOn(), now_, nullptr);
  control_.ReleaseEmergency(now_, 0);
  control_.SetCharging(false, now_);
  Run(2.0, false);
  EXPECT_TRUE(sent_.empty());
  EXPECT_TRUE(control_.GuardFrames().empty());
  EXPECT_TRUE(control_.Farewell().empty());
  // The first frame starts the takeover: the stop and the blade off, then the heartbeat, nothing that moves.
  control_.OnFrame();
  Run(2.05, false);
  EXPECT_FALSE(Times(IsHeartbeat).empty());
  EXPECT_FALSE(Times(IsBladeOff).empty());
  for (const auto& s : sent_) {
    EXPECT_FALSE(IsMovingSpeed(s.packet));
    EXPECT_FALSE(IsBladeOn(s.packet));
  }
  EXPECT_FALSE(control_.GuardFrames().empty());
}

TEST_F(ControlTest, BoardNotHeardIsReportedAfterTheTelemetryTimeout) {
  control_.OnConnected(0.0);
  Run(1.0, false);
  auto status = control_.Status(now_);
  EXPECT_FALSE(status.blocked);
  EXPECT_EQ(status.emergency_reason, 0u);
  EXPECT_FALSE(DrivesOrMows(1.0));
  Run(1.05, false);
  status = control_.Status(now_);
  EXPECT_TRUE(status.blocked);
  EXPECT_FALSE(status.latched);
  EXPECT_TRUE(status.telemetry_stale);
  EXPECT_EQ(status.emergency_reason, kReasonBoardNotHeard);
  // The first frame ends it without a release; then all is as at a normal start.
  Run(1.2);
  status = control_.Status(now_);
  EXPECT_FALSE(status.blocked);
  EXPECT_EQ(status.emergency_reason, 0u);
  EXPECT_TRUE(DrivesOrMows(1.4));
  // Silence later is the input timeout, a reopen is unheard again.
  Run(2.6, false);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonTimeoutInputs);
  control_.OnConnected(now_);
  Run(3.0, false);
  EXPECT_EQ(control_.Status(now_).emergency_reason, 0u);
  Run(3.7, false);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonBoardNotHeard);
}

TEST_F(StartLatchedImuTest, TheImuStillDueIsNoReasonAndAReleaseBeforeItHasNoEffect) {
  Run(0.5);
  EXPECT_EQ(control_.Status(now_).emergency_reason, kReasonLatch | kReasonHighLevel);
  EXPECT_TRUE(control_.ReleaseEmergency(now_, 1));
  Run(1.8);
  EXPECT_TRUE(control_.Status(now_).latched);
  feed_imu_ = true;
  Run(1.9);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_TRUE(control_.ReleaseEmergency(now_, 1));
  Run(2.0);
  EXPECT_FALSE(control_.Status(now_).latched);
}

// The MCU might still run a setpoint of a driver that ended in any way, also with the whole stack.
TEST_F(ControlTest, WithoutStartLatchedTheStartIsFreeAndStillStops) {
  keep_start_stops_ = true;
  Run(0.5);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_EQ(control_.Status(now_).latch_count, 0u);
  const auto zeros = Times(IsZeroSpeed);
  ASSERT_EQ(zeros.size(), 3u);
  for (size_t i = 0; i < zeros.size(); i++) EXPECT_NEAR(zeros[i], 0.01 + 0.1 * static_cast<double>(i), 1e-9);
}

// Journals from before start_latched replay as they ran: started free.
TEST(McuJournal, ConfigFromBeforeStartLatchedReplays) {
  tron::McuControlConfig config;
  config.start_latched = true;
  nlohmann::json j = config;
  EXPECT_TRUE(j.get<tron::McuControlConfig>().start_latched);
  j.erase("start_latched");
  EXPECT_FALSE(j.get<tron::McuControlConfig>().start_latched);
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

// Journals from before the firmware lists replay with their one expected version per board.
TEST(McuJournal, ConfigFromBeforeTheFirmwareListsReplays) {
  tron::McuControlConfig config;
  config.cutter_firmware = {"0.6.34", "0.6.36"};
  config.chassis_firmware = {"0.6.34"};
  nlohmann::json j = config;
  EXPECT_EQ(j.get<tron::McuControlConfig>().cutter_firmware, config.cutter_firmware);
  j.erase("cutter_firmware");
  j.erase("chassis_firmware");
  j["expected_cutter_version"] = "0.6.36";
  j["expected_chassis_version"] = "";
  const auto old = j.get<tron::McuControlConfig>();
  EXPECT_EQ(old.cutter_firmware, std::vector<std::string>{"0.6.36"});
  EXPECT_TRUE(old.chassis_firmware.empty());
}

class SilenceTest : public ControlTest {
 protected:
  static McuControlConfig SilenceConfig() {
    McuControlConfig config;
    config.allow_blade = true;
    config.silence_max_s = 3.0;
    return config;
  }
  SilenceTest() : ControlTest(SilenceConfig()) {
    feed_setup_ = true;
  }
  // Drives until now_ = 0.5, then the silence begins.
  double DriveThenSilence(double seconds) {
    Run(0.3);
    control_.OnCmdVel(0.2f, 0.0f, now_);
    Run(0.5);
    EXPECT_EQ(control_.Silence(seconds, now_), HostTestRefusal::kNone);
    return now_;
  }
};

TEST_F(SilenceTest, NothingGoesOutUntilTheEndThenLatchBladeOffAndThreeStops) {
  const double start = DriveThenSilence(1.0);
  EXPECT_TRUE(control_.Status(now_).silent);
  const uint32_t count = control_.Status(now_).latch_count;
  Run(start + 0.99);
  EXPECT_TRUE(Times([](const SubPacket&) { return true; }, start + 0.001).empty());
  Run(start + 1.5);
  const double end = start + 1.0;
  EXPECT_EQ(Times(IsHeartbeat, end - 1e-6, end + 1e-6).size(), 1u);
  auto zeros = Times(IsZeroSpeed, end - 1e-6);
  ASSERT_EQ(zeros.size(), 3u);
  EXPECT_NEAR(zeros[0], end, 1e-9);
  EXPECT_NEAR(zeros[2], end + 0.2, 1e-9);
  EXPECT_EQ(Times(IsBladeOff, end - 1e-6).size(), 3u);
  EXPECT_TRUE(Times(IsMovingSpeed, start + 0.001).empty());
  const auto status = control_.Status(now_);
  EXPECT_FALSE(status.silent);
  EXPECT_TRUE(status.latched);
  EXPECT_EQ(status.emergency_reason, kReasonLatch | kReasonHighLevel);
  EXPECT_EQ(status.latch_count, count + 1);
}

TEST_F(SilenceTest, ItBeginsAtTheTickOfTheLastFrameBeforeIt) {
  EXPECT_FALSE(control_.Status(now_).silent_from_s);
  const double start = DriveThenSilence(1.0);
  const auto sent = Times([](const SubPacket&) { return true; }, 0.0, start + 1e-9);
  ASSERT_FALSE(sent.empty());
  const auto from = control_.Status(now_).silent_from_s;
  ASSERT_TRUE(from);
  EXPECT_DOUBLE_EQ(*from, sent.back());
  Run(start + 1.5);
  ASSERT_TRUE(control_.Status(now_).silent_from_s);
  EXPECT_DOUBLE_EQ(*control_.Status(now_).silent_from_s, sent.back());
}

TEST_F(SilenceTest, CommandsBladeAndReleasesMeanwhileAreDropped) {
  const double start = DriveThenSilence(0.5);
  control_.LatchEmergency(kReasonHighLevel);
  const uint32_t count = control_.Status(now_).latch_count;
  for (int i = 0; i < 8; i++) {
    control_.OnCmdVel(0.3f, 0.2f, now_);
    EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kBlocked);
    EXPECT_FALSE(control_.ReleaseEmergency(now_, count));
    EXPECT_FALSE(control_.ReleaseEmergency(now_, 0));
    Run(now_ + 0.05);
  }
  Run(start + 2.0);
  EXPECT_TRUE(Times(IsMovingSpeed, start + 0.001).empty());
  EXPECT_TRUE(Times(IsBladeOn).empty());
  EXPECT_TRUE(Times(IsClear).empty());
  // HIGH_LEVEL was latched already: the end counts anew all the same.
  EXPECT_EQ(control_.Status(now_).latch_count, count + 1);
  EXPECT_FALSE(control_.ReleaseEmergency(now_, count));
  EXPECT_TRUE(control_.ReleaseEmergency(now_, count + 1));
  Run(now_ + 0.05);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_TRUE(Times(IsMovingSpeed, start + 0.001).empty());
  EXPECT_TRUE(DrivesOrMows(now_ + 0.5));
}

TEST_F(SilenceTest, TheImuIsNotForwardedMeanwhile) {
  feed_imu_ = true;
  Run(0.5);
  const double start = now_;
  ASSERT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNone);
  Run(start + 2.0);
  auto imu = [](const SubPacket& p) { return IsModule(p, Module::kImu); };
  EXPECT_TRUE(Times(imu, start + 0.001, start + 0.999).empty());
  EXPECT_FALSE(Times(imu, start + 1.0).empty());
  // The samples kept coming, so no IMU gap.
  EXPECT_FALSE(control_.Status(now_).emergency_reason & kReasonImuTimeout);
}

TEST_F(SilenceTest, RefusedOutOfRange) {
  Run(0.5);
  EXPECT_EQ(control_.Silence(0.19, now_), HostTestRefusal::kOutOfRange);
  EXPECT_EQ(control_.Silence(3.01, now_), HostTestRefusal::kOutOfRange);
  EXPECT_EQ(control_.Silence(-1.0, now_), HostTestRefusal::kOutOfRange);
  EXPECT_EQ(control_.Silence(std::nan(""), now_), HostTestRefusal::kOutOfRange);
  EXPECT_EQ(control_.Silence(INFINITY, now_), HostTestRefusal::kOutOfRange);
  EXPECT_FALSE(control_.Status(now_).silent);
  EXPECT_EQ(control_.Silence(static_cast<double>(0.2f), now_), HostTestRefusal::kNone);
}

TEST_F(ControlTest, WithoutSilenceMaxEverySilenceIsRefused) {
  Run(0.5);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kOutOfRange);
  EXPECT_FALSE(control_.Status(now_).silent);
}

TEST_F(SilenceTest, RefusedWhileSilentAlready) {
  Run(0.5);
  ASSERT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNone);
  Run(1.0);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kBusy);
  Run(1.6);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNone);
}

TEST_F(SilenceTest, RefusedOnTheChargingContactsAlsoBeforeTheDebounce) {
  Run(0.5);
  control_.OnDockContactSample(true);
  EXPECT_FALSE(control_.Status(now_).dock_contact);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kCharging);
  EXPECT_EQ(control_.End(), HostTestRefusal::kCharging);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kCharging);
  control_.OnDockContactSample(false);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNone);
}

TEST_F(SilenceTest, RefusedWithTheBladeOnOrTurning) {
  Run(0.5);
  ASSERT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kBlade);
  Run(0.6);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kBlade);
  EXPECT_EQ(control_.End(), HostTestRefusal::kBlade);
  control_.SetBlade(kBladeOff, now_);
  Run(0.7);
  // Off, but still running down.
  control_.OnBladeRpm(2500, now_);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kBlade);
  EXPECT_EQ(control_.End(), HostTestRefusal::kBlade);
  control_.OnBladeRpm(0, now_);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNone);
}

TEST_F(SilenceTest, AfterTheEndNothingGoesOutAnyMore) {
  Run(0.3);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(0.5);
  ASSERT_EQ(control_.End(), HostTestRefusal::kNone);
  const double end = now_;
  for (int i = 0; i < 20; i++) {
    control_.OnCmdVel(0.2f, 0.0f, now_);
    EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kBlocked);
    EXPECT_FALSE(control_.ReleaseEmergency(now_, 0));
    control_.OnConnected(now_);
    Run(now_ + 0.1);
  }
  EXPECT_TRUE(Times([](const SubPacket&) { return true; }, end + 0.001).empty());
  EXPECT_TRUE(control_.Status(now_).ended);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kBusy);
}

TEST_F(SilenceTest, TheEndIsTakenDuringASilence) {
  Run(0.5);
  ASSERT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNone);
  const double start = now_;
  Run(0.8);
  EXPECT_EQ(control_.End(), HostTestRefusal::kNone);
  Run(3.0);
  EXPECT_TRUE(Times([](const SubPacket&) { return true; }, start + 0.001).empty());
}

// Journals from before hw/silence replay with what they ran with: none offered.
TEST(McuJournal, ConfigFromBeforeTheSilenceReplays) {
  tron::McuControlConfig config;
  config.silence_max_s = 3.0;
  nlohmann::json j = config;
  EXPECT_EQ(j.get<tron::McuControlConfig>().silence_max_s, 3.0);
  j.erase("silence_max_s");
  EXPECT_EQ(j.get<tron::McuControlConfig>().silence_max_s, 0.0);
}

TEST_F(SilenceTest, WithTheBladeNothingGoesOutAndTheBladeStaysOffAfterwards) {
  Run(0.5);
  ASSERT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  Run(1.0);
  control_.OnBladeRpm(3000, now_);
  ASSERT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kBlade);
  ASSERT_EQ(control_.SilenceWithBlade(1.0, now_), HostTestRefusal::kNone);
  const double start = now_;
  // A renewal meanwhile is refused.
  Run(start + 0.3);
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kBlocked);
  control_.OnBladeRpm(0, now_);
  Run(start + 2.5);
  EXPECT_TRUE(Times([](const SubPacket&) { return true; }, start + 0.001, start + 0.999).empty());
  const double end = start + 1.0;
  EXPECT_EQ(Times(IsBladeOff, end - 1e-6).size(), 3u);
  EXPECT_EQ(Times(IsZeroSpeed, end - 1e-6).size(), 3u);
  EXPECT_TRUE(Times(IsBladeOn, start + 0.001).empty());
  auto status = control_.Status(now_);
  EXPECT_FALSE(status.blade_on);
  EXPECT_TRUE(status.latched);
  EXPECT_EQ(status.emergency_reason, kReasonLatch | kReasonHighLevel);
  // The lease from before does not come back with the release.
  EXPECT_TRUE(control_.ReleaseEmergency(now_, status.latch_count));
  Run(now_ + 1.0);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_TRUE(Times(IsBladeOn, start + 0.001).empty());
  EXPECT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  Run(now_ + 0.05);
  EXPECT_FALSE(Times(IsBladeOn, start + 0.001).empty());
}

TEST_F(SilenceTest, WithTheBladeAStillTurningBladeLatchesAfterTheEnd) {
  Run(0.5);
  ASSERT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  Run(1.0);
  ASSERT_EQ(control_.SilenceWithBlade(1.0, now_), HostTestRefusal::kNone);
  // The MCU does not stop it; blade_stop_time_s is 5 s here.
  for (int i = 0; i < 80; i++) {
    control_.OnBladeRpm(3000, now_);
    Run(now_ + 0.1);
  }
  EXPECT_TRUE(control_.Status(now_).emergency_reason & kReasonMowerNotStopped);
}

TEST_F(SilenceTest, WithTheBladeTheOtherRefusalsStay) {
  Run(0.5);
  ASSERT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  EXPECT_EQ(control_.SilenceWithBlade(0.1, now_), HostTestRefusal::kOutOfRange);
  EXPECT_EQ(control_.SilenceWithBlade(3.5, now_), HostTestRefusal::kOutOfRange);
  control_.OnDockContactSample(true);
  EXPECT_EQ(control_.SilenceWithBlade(1.0, now_), HostTestRefusal::kCharging);
  control_.OnDockContactSample(false);
  ASSERT_EQ(control_.SilenceWithBlade(1.0, now_), HostTestRefusal::kNone);
  EXPECT_EQ(control_.SilenceWithBlade(1.0, now_), HostTestRefusal::kBusy);
  // hw/end keeps refusing a blade that was on.
  EXPECT_EQ(control_.End(), HostTestRefusal::kBlade);
}

// Measurement settings (H3): heartbeats tied to SpeedData, hw/silence after a chosen tick, delayed stops after it.
class HoldMeasureTest : public ControlTest {
 protected:
  static McuControlConfig Config(McuControlConfig::SpeedHeartbeat heartbeat, McuControlConfig::SilenceAfter after,
                                 double stop_delay = 0) {
    McuControlConfig config;
    config.allow_blade = true;
    config.silence_max_s = 10.0;
    config.speed_heartbeat = heartbeat;
    config.silence_after = after;
    config.silence_stop_delay_s = stop_delay;
    return config;
  }
  explicit HoldMeasureTest(McuControlConfig config) : ControlTest(config) {
    feed_setup_ = true;
  }
  // Keeps commanding motion every 0.1 s up to `until`, as the logic does.
  void Drive(double until) {
    while (now_ < until - 1e-9) {
      control_.OnCmdVel(0.05f, 0.0f, now_);
      Run(std::min(until, now_ + 0.1));
    }
  }
  // The packets of the tick at t.
  std::vector<SubPacket> TickAt(double t) const {
    std::vector<SubPacket> packets;
    for (const auto& s : sent_) {
      if (std::fabs(s.t - t) < 1e-6) packets.push_back(s.packet);
    }
    return packets;
  }
  // The first time from `from` on after which nothing went out for `quiet` s.
  double SilentFrom(double from, double quiet) const {
    double last = from;
    for (const auto& s : sent_) {
      if (s.t < from - 1e-9) continue;
      if (s.t - last > quiet) return last;
      last = s.t;
    }
    return last;
  }
};

class HoldDefaultTest : public HoldMeasureTest {
 protected:
  HoldDefaultTest()
      : HoldMeasureTest(Config(McuControlConfig::SpeedHeartbeat::kNone, McuControlConfig::SilenceAfter::kSpeed)) {
  }
};

class HoldAfterTest : public HoldMeasureTest {
 protected:
  HoldAfterTest()
      : HoldMeasureTest(Config(McuControlConfig::SpeedHeartbeat::kAfter, McuControlConfig::SilenceAfter::kSpeed, 1.0)) {
  }
};

class HoldLaterTest : public HoldMeasureTest {
 protected:
  HoldLaterTest()
      : HoldMeasureTest(Config(McuControlConfig::SpeedHeartbeat::kLater, McuControlConfig::SilenceAfter::kSpeed)) {
  }
};

class HoldCutterTest : public HoldMeasureTest {
 protected:
  HoldCutterTest()
      : HoldMeasureTest(Config(McuControlConfig::SpeedHeartbeat::kNone, McuControlConfig::SilenceAfter::kCutter)) {
  }
};

// The Tron: a heartbeat later after each SpeedData, hw/silence without a chosen tick.
class TronHoldTest : public HoldMeasureTest {
 protected:
  TronHoldTest()
      : HoldMeasureTest(Config(McuControlConfig::SpeedHeartbeat::kLater, McuControlConfig::SilenceAfter::kNone)) {
  }
};

TEST_F(TronHoldTest, ASilenceRightAfterASetpointBeginsAfterItsHeartbeat) {
  Run(0.3);
  Drive(0.6);
  control_.OnCmdVel(0.05f, 0.0f, now_);
  Run(now_ + 0.01);
  ASSERT_TRUE(IsMovingSpeed(TickAt(now_).back()));
  const double speed = now_;
  ASSERT_EQ(control_.Silence(2.0, now_), HostTestRefusal::kNone);
  EXPECT_FALSE(control_.Status(now_).silent);
  EXPECT_TRUE(control_.Status(now_).silence_armed);
  Run(speed + 0.1);
  const auto beats = Times(IsHeartbeat, speed + 0.001);
  ASSERT_EQ(beats.size(), 1u);
  EXPECT_NEAR(beats[0] - speed, 0.025, 0.006);
  EXPECT_TRUE(control_.Status(now_).silent);
  ASSERT_TRUE(control_.Status(now_).silent_from_s);
  EXPECT_DOUBLE_EQ(*control_.Status(now_).silent_from_s, beats[0]);
  EXPECT_TRUE(Times([](const SubPacket&) { return true; }, beats[0] + 0.001, speed + 0.1).empty());
}

TEST_F(TronHoldTest, WithoutAHeartbeatDueTheSilenceBeginsAtOnce) {
  Run(0.3);
  Drive(0.6);
  Run(2.0);
  ASSERT_EQ(control_.Silence(2.0, now_), HostTestRefusal::kNone);
  EXPECT_TRUE(control_.Status(now_).silent);
  const auto sent = Times([](const SubPacket&) { return true; }, 0.0, now_ + 1e-9);
  ASSERT_TRUE(control_.Status(now_).silent_from_s);
  EXPECT_DOUBLE_EQ(*control_.Status(now_).silent_from_s, sent.back());
}

TEST_F(HoldDefaultTest, WithoutTheSettingNoExtraHeartbeatAndTheSetpointEndsItsTick) {
  Run(0.3);
  Drive(1.3);
  const auto beats = Times(IsHeartbeat, 0.31, 1.3);
  EXPECT_EQ(beats.size(), 10u);
  for (double t : Times(IsMovingSpeed, 0.31, 1.3)) EXPECT_TRUE(IsMovingSpeed(TickAt(t).back()));
}

TEST_F(HoldDefaultTest, TheSilenceWaitsForTheNextSetpointAndStartsRightAfterIt) {
  Run(0.3);
  Drive(0.85);
  ASSERT_EQ(control_.Silence(3.0, now_), HostTestRefusal::kNone);
  EXPECT_FALSE(control_.Status(now_).silent);
  EXPECT_FALSE(control_.Status(now_).silent_from_s);
  Drive(1.2);
  const double start = SilentFrom(0.85, 0.5);
  EXPECT_TRUE(IsMovingSpeed(TickAt(start).back()));
  EXPECT_TRUE(control_.Status(now_).silent);
  ASSERT_TRUE(control_.Status(now_).silent_from_s);
  EXPECT_DOUBLE_EQ(*control_.Status(now_).silent_from_s, start);
  Run(start + 2.9);
  EXPECT_TRUE(Times([](const SubPacket&) { return true; }, start + 0.001, start + 2.9).empty());
}

TEST_F(HoldDefaultTest, WithoutSuchATickTheArmedSilenceIsDropped) {
  Run(0.3);
  ASSERT_EQ(control_.Silence(3.0, now_), HostTestRefusal::kNone);
  EXPECT_EQ(control_.Silence(3.0, now_), HostTestRefusal::kBusy);
  EXPECT_TRUE(control_.Status(now_).silence_armed);
  Run(0.3 + kSilenceArmS + 0.05);
  EXPECT_FALSE(control_.Status(now_).silent);
  EXPECT_FALSE(control_.Status(now_).silence_armed);
  EXPECT_FALSE(control_.Status(now_).silent_from_s);
  Drive(now_ + 0.5);
  EXPECT_FALSE(control_.Status(now_).silent);
  EXPECT_FALSE(Times(IsMovingSpeed, now_ - 0.5).empty());
  EXPECT_EQ(control_.Silence(3.0, now_), HostTestRefusal::kNone);
}

TEST_F(HoldAfterTest, EverySpeedDataIsFollowedByAHeartbeatInItsTick) {
  Run(0.3);
  Drive(1.3);
  Run(2.5);
  const auto speeds = Times([](const SubPacket& p) { return IsModule(p, Module::kSpeed); }, 0.31);
  ASSERT_GE(speeds.size(), 13u);
  for (double t : speeds) {
    const auto tick = TickAt(t);
    for (size_t i = 0; i < tick.size(); ++i) {
      if (IsModule(tick[i], Module::kSpeed)) {
        ASSERT_LT(i + 1, tick.size());
        EXPECT_TRUE(IsHeartbeat(tick[i + 1]));
      }
    }
    EXPECT_TRUE(IsHeartbeat(tick.back()));
  }
}

TEST_F(HoldAfterTest, TheSilenceStartsAfterThePairAndItsStopsWaitWhileHeartbeatsGo) {
  Run(0.3);
  Drive(0.85);
  ASSERT_EQ(control_.Silence(2.0, now_), HostTestRefusal::kNone);
  Drive(1.2);
  const double start = SilentFrom(0.85, 0.5);
  const auto tick = TickAt(start);
  ASSERT_GE(tick.size(), 2u);
  EXPECT_TRUE(IsMovingSpeed(tick[tick.size() - 2]));
  EXPECT_TRUE(IsHeartbeat(tick.back()));
  Run(start + 4.0);
  const double end = start + 2.0;
  const auto zeros = Times(IsZeroSpeed, end - 0.01);
  ASSERT_EQ(zeros.size(), 3u);
  EXPECT_NEAR(zeros[0], end + 1.0, 0.011);
  EXPECT_GE(Times(IsHeartbeat, end - 0.01, end + 0.99).size(), 9u);
  EXPECT_TRUE(Times(IsMovingSpeed, start + 0.001).empty());
}

TEST_F(HoldLaterTest, AHeartbeatFollowsTheLastSetpointLaterAndTheSilenceStartsAfterIt) {
  Run(0.3);
  Drive(0.85);
  for (double t : Times(IsMovingSpeed, 0.31, 0.85)) EXPECT_TRUE(IsMovingSpeed(TickAt(t).back()));
  const auto speeds = Times(IsMovingSpeed, 0.31, 0.75);
  ASSERT_FALSE(speeds.empty());
  EXPECT_EQ(Times(IsHeartbeat, speeds.back() + 0.025, speeds.back() + 0.035).size(), 1u);
  ASSERT_EQ(control_.Silence(2.0, now_), HostTestRefusal::kNone);
  control_.OnCmdVel(0.05f, 0.0f, now_);
  Run(1.2);
  const double start = SilentFrom(0.85, 0.5);
  EXPECT_TRUE(IsHeartbeat(TickAt(start).back()));
  const auto last = Times(IsMovingSpeed, 0.85, start);
  ASSERT_FALSE(last.empty());
  EXPECT_NEAR(start - last.back(), 0.03, 0.006);
  ASSERT_TRUE(control_.Status(now_).silent_from_s);
  EXPECT_DOUBLE_EQ(*control_.Status(now_).silent_from_s, start);
}

TEST_F(HoldCutterTest, TheSilenceStartsRightAfterTheBladeOnFrame) {
  Run(0.3);
  ASSERT_EQ(control_.SilenceWithBlade(3.0, now_), HostTestRefusal::kNone);
  ASSERT_EQ(control_.SetBlade(BladeOn(), now_), BladeResult::kOk);
  Run(0.6);
  const auto on = Times(IsBladeOn);
  ASSERT_EQ(on.size(), 1u);
  EXPECT_TRUE(IsBladeOn(TickAt(on[0]).back()));
  EXPECT_TRUE(Times([](const SubPacket&) { return true; }, on[0] + 0.001).empty());
  EXPECT_TRUE(control_.Status(now_).silent);
  ASSERT_TRUE(control_.Status(now_).silent_from_s);
  EXPECT_DOUBLE_EQ(*control_.Status(now_).silent_from_s, on[0]);
}

// F5: a stop is not confirmed by the MCU; the wheel speeds it reports are.
class WheelStopTest : public ControlTest {
 protected:
  WheelStopTest() {
    feed_wheels_ = true;
  }
  // Drives from 0.3 s, the command ends at 0.5 s: the stop frames begin at 0.5 s.
  void DriveThenStop() {
    Run(0.3);
    control_.OnCmdVel(0.2f, 0.0f, now_);
    wheel_linear_ = 0.2f;
    Run(0.5);
    control_.OnCmdVel(0.0f, 0.0f, now_);
  }
  bool NotStopped() const {
    return (control_.Status(now_).emergency_reason & kReasonMowerNotStopped) != 0;
  }
};

TEST_F(WheelStopTest, WheelsStillTurningAfterTheStopLatchAndTheStopsGoOutAgain) {
  DriveThenStop();
  Run(1.99);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_EQ(Times(IsZeroSpeed, 0.5).size(), 3u);
  Run(2.01);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_TRUE(NotStopped());
  Run(2.3);
  EXPECT_EQ(Times(IsZeroSpeed, 2.0).size(), 3u);
  // Still turning: once more after another wheel_stop_time_s, not with every report.
  Run(3.45);
  EXPECT_EQ(Times(IsZeroSpeed, 2.0).size(), 3u);
  Run(3.8);
  EXPECT_EQ(Times(IsZeroSpeed, 2.0).size(), 6u);
}

TEST_F(WheelStopTest, AReleaseWaitsForStandingWheels) {
  DriveThenStop();
  Run(2.2);
  ASSERT_TRUE(NotStopped());
  control_.ReleaseEmergency(now_, 0);
  Run(2.5);
  EXPECT_TRUE(control_.Status(now_).latched);
  wheel_linear_ = 0;
  Run(2.6);
  ASSERT_TRUE(control_.ReleaseEmergency(now_, 0));
  Run(2.7);
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(WheelStopTest, WheelsThatStoodEndTheCheckAndLaterTurnsLatchNothing) {
  DriveThenStop();
  Run(0.7);
  wheel_linear_ = 0;
  Run(2.1);
  // Pushed by hand afterwards.
  wheel_linear_ = 0.3f;
  Run(6.0);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_EQ(Times(IsZeroSpeed, 0.5).size(), 3u);
}

TEST_F(WheelStopTest, AShortSpikeIsNoTurningWheel) {
  DriveThenStop();
  Run(0.7);
  wheel_linear_ = 0;
  Run(1.9);
  wheel_linear_ = 0.05f;
  Run(2.18);
  wheel_linear_ = 0;
  Run(2.3);
  wheel_linear_ = 0.05f;
  Run(2.5);
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(WheelStopTest, BelowTheStillSpeedTheWheelsStand) {
  DriveThenStop();
  // The standing Tron reports up to 0.011 m/s and 0.028 rad/s.
  wheel_linear_ = 0.011f;
  wheel_angular_ = 0.028f;
  Run(4.0);
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(WheelStopTest, ATurnOnTheSpotCountsByItsFasterWheel) {
  DriveThenStop();
  wheel_linear_ = 0;
  // 0.15 rad/s with 0.40 m between the wheels: 0.03 m/s each.
  wheel_angular_ = 0.15f;
  Run(2.3);
  EXPECT_TRUE(NotStopped());
}

TEST_F(WheelStopTest, TurningWhileCommandedIsNoFault) {
  Run(0.3);
  wheel_linear_ = 0.2f;
  for (int i = 0; i < 50; i++) {
    control_.OnCmdVel(0.2f, 0.0f, now_);
    Run(now_ + 0.1);
  }
  EXPECT_FALSE(control_.Status(now_).latched);
}

TEST_F(WheelStopTest, TheStopsOfTheStartAreCheckedAsWell) {
  wheel_linear_ = 0.1f;
  Run(1.0);
  EXPECT_FALSE(control_.Status(now_).latched);
  Run(1.8);
  EXPECT_TRUE(NotStopped());
}

TEST_F(WheelStopTest, AnotherCommandEndsTheCheck) {
  DriveThenStop();
  Run(1.0);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(1.2);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(1.4);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(1.6);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(1.8);
  EXPECT_FALSE(control_.Status(now_).latched);
}

class NoWheelStopCheckTest : public ControlTest {
 protected:
  static McuControlConfig NoCheckConfig() {
    McuControlConfig config;
    config.wheel_stop_time_s = 0;
    return config;
  }
  NoWheelStopCheckTest() : ControlTest(NoCheckConfig()) {
    feed_wheels_ = true;
  }
};

TEST_F(NoWheelStopCheckTest, ZeroTurnsTheCheckOff) {
  wheel_linear_ = 0.3f;
  Run(5.0);
  EXPECT_FALSE(control_.Status(now_).latched);
}

// Journals from before the check of the wheels replay as they ran: without it.
TEST(McuJournal, ConfigFromBeforeTheWheelCheckReplays) {
  tron::McuControlConfig config;
  config.wheel_stop_time_s = 2.5;
  config.wheel_still_mps = 0.04;
  nlohmann::json j = config;
  EXPECT_EQ(j.get<tron::McuControlConfig>().wheel_stop_time_s, 2.5);
  EXPECT_EQ(j.get<tron::McuControlConfig>().wheel_still_mps, 0.04);
  j.erase("wheel_stop_time_s");
  j.erase("wheel_still_mps");
  EXPECT_EQ(j.get<tron::McuControlConfig>().wheel_stop_time_s, 0.0);
}

// F8: the frames when the process ends.
TEST_F(ControlTest, TheFarewellStopsAlsoAStandingMowerAndSwitchesTheBladeOff) {
  Run(1.0);
  const auto farewell = control_.Farewell();
  ASSERT_EQ(farewell.size(), 2u);
  EXPECT_TRUE(IsZeroSpeed(farewell[0]));
  EXPECT_TRUE(IsBladeOff(farewell[1]));
  EXPECT_EQ(CutterHeight(farewell[1]), kMaxCutterHeightMm);
}

TEST_F(ControlTest, TheFarewellLeavesTheStopOutOnTheChargingContactsWithoutCommand) {
  Run(0.5);
  for (int i = 0; i < 3; i++) control_.OnDockContactSample(true);
  ASSERT_TRUE(control_.Status(now_).dock_contact);
  auto farewell = control_.Farewell();
  ASSERT_EQ(farewell.size(), 1u);
  EXPECT_TRUE(IsBladeOff(farewell[0]));
  // Driving onto the contacts: commanded, so the stop goes.
  control_.OnCmdVel(0.1f, 0.0f, now_);
  Run(0.6);
  ASSERT_TRUE(control_.Status(now_).moving);
  farewell = control_.Farewell();
  ASSERT_EQ(farewell.size(), 2u);
  EXPECT_TRUE(IsZeroSpeed(farewell[0]));
}

// R15-3/8 on the Tron: a time that would switch a check off refuses the start.
TEST(McuConfigTest, TheDefaultsAndTheTronValuesPass) {
  tron::McuControlConfig config;
  EXPECT_NO_THROW(tron::CheckConfig(config));
  config.imu_timeout_s = 0.5;
  config.imu_start_timeout_s = 2.0;
  EXPECT_NO_THROW(tron::CheckConfig(config));
}

TEST(McuConfigTest, NotFiniteZeroOrHugeTimesRefuseTheStart) {
  const double nan = std::nan("");
  constexpr double kHuge = std::numeric_limits<double>::infinity();
  using Field = double tron::McuControlConfig::*;
  for (Field field : {&tron::McuControlConfig::cmd_vel_timeout_s, &tron::McuControlConfig::telemetry_timeout_s,
                      &tron::McuControlConfig::blade_lease_max_s}) {
    for (double value : {nan, kHuge, -kHuge, 0.0, -0.1, 1e9}) {
      tron::McuControlConfig config;
      config.*field = value;
      EXPECT_THROW(tron::CheckConfig(config), std::invalid_argument) << value;
    }
  }
  for (Field field : {&tron::McuControlConfig::imu_timeout_s, &tron::McuControlConfig::imu_start_timeout_s,
                      &tron::McuControlConfig::speed_repeat_window_s, &tron::McuControlConfig::blade_stop_time_s,
                      &tron::McuControlConfig::wheel_stop_time_s}) {
    for (double value : {nan, kHuge, -0.1, 1e9}) {
      tron::McuControlConfig config;
      config.*field = value;
      EXPECT_THROW(tron::CheckConfig(config), std::invalid_argument) << value;
    }
  }
  tron::McuControlConfig config;
  config.cmd_vel_timeout_s = 1.5;
  EXPECT_THROW(tron::CheckConfig(config), std::invalid_argument);
  // T2, heartbeat alone: up to 10 s.
  config.cmd_vel_timeout_s = 10.0;
  EXPECT_NO_THROW(tron::CheckConfig(config, true));
  config.cmd_vel_timeout_s = 10.5;
  EXPECT_THROW(tron::CheckConfig(config, true), std::invalid_argument);
}

// F1: what the guard of the node writes while its I/O thread stalls.
TEST_F(ControlTest, TheGuardWritesTheFarewell) {
  Run(0.5);
  control_.OnCmdVel(0.2f, 0.0f, now_);
  Run(1.0);
  ASSERT_TRUE(control_.Status(now_).moving);
  const auto guard = control_.GuardFrames();
  ASSERT_EQ(guard.size(), 2u);
  EXPECT_TRUE(IsZeroSpeed(guard[0]));
  EXPECT_TRUE(IsBladeOff(guard[1]));
}

TEST_F(SilenceTest, TheGuardWritesNothingDuringASilenceOrAfterTheEnd) {
  const double start = DriveThenSilence(1.0);
  EXPECT_TRUE(control_.GuardFrames().empty());
  Run(start + 1.5);
  ASSERT_FALSE(control_.Status(now_).silent);
  EXPECT_FALSE(control_.GuardFrames().empty());
  ASSERT_EQ(control_.End(), HostTestRefusal::kNone);
  EXPECT_TRUE(control_.GuardFrames().empty());
}

TEST_F(ControlTest, TheFarewellStopsOnASingleContactSample) {
  Run(0.5);
  control_.OnDockContactSample(true);
  ASSERT_FALSE(control_.Status(now_).dock_contact);
  EXPECT_EQ(control_.Farewell().size(), 2u);
}

// F9: hw/silence only while a step of the setup program runs.
class SilenceSetupTest : public ControlTest {
 protected:
  static McuControlConfig Config() {
    McuControlConfig config;
    config.silence_max_s = 3.0;
    return config;
  }
  SilenceSetupTest() : ControlTest(Config()) {
  }
};

TEST_F(SilenceSetupTest, RefusedWithoutSetupMotion) {
  Run(0.5);
  control_.OnCmdVel(0.5f, 0.0f, now_);
  Run(0.6);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNoSetup);
  EXPECT_EQ(control_.SilenceWithBlade(1.0, now_), HostTestRefusal::kNoSetup);
  EXPECT_FALSE(control_.Status(now_).silent);
  EXPECT_STREQ(HostTestRefusalCode(HostTestRefusal::kNoSetup), "NO_SETUP");
}

TEST_F(SilenceSetupTest, TakenWithinASecondOfAnActiveSetupMotionOnly) {
  Run(0.5);
  control_.OnSetupMotion(true, now_);
  Run(1.5);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNone);
  Run(3.0);
  control_.OnSetupMotion(true, now_);
  Run(4.01);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNoSetup);
}

TEST_F(SilenceSetupTest, AnInactiveSetupMotionEndsIt) {
  Run(0.5);
  control_.OnSetupMotion(true, now_);
  control_.OnSetupMotion(false, now_);
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNoSetup);
}
