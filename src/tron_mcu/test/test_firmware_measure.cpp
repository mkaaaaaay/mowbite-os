#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <algorithm>
#include <stdexcept>

#include "tron_mcu/mcu_control.h"
#include "tron_mcu/mcu_journal.h"

using namespace tron;

namespace {

const WallClock kWall{2026, 11, 26, 12, 0, 0, 0};
const std::string kHash(64, 'a');
const std::string kOtherHash(64, 'b');

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

// The faster wheel of a SpeedData, as the MCU turns it into wheel speeds.
float FastestWheel(const SubPacket& packet) {
  float linear, angular;
  std::memcpy(&linear, packet.payload.data(), 4);
  std::memcpy(&angular, packet.payload.data() + 4, 4);
  return std::fabs(linear) + std::fabs(angular) * kHalfTrack;
}

VersionInfo Versions(const char* cutter, const char* chassis) {
  VersionInfo v;
  v.cutter_board = *ParseVersion(cutter);
  v.chassis_board = *ParseVersion(chassis);
  return v;
}

McuControlConfig MeasureConfig() {
  McuControlConfig config;
  config.allow_blade = true;
  config.cutter_firmware = {"0.6.36"};
  config.chassis_firmware = {"0.6.34"};
  config.device_hash = kHash;
  return config;
}

BladeRequest BladeOn() {
  BladeRequest request;
  request.on = true;
  request.lease_s = 1.0;
  return request;
}

// Runs the controller in 10 ms steps with fresh telemetry, the boards on the given firmware and, while measure_ is set,
// setup/firmware_measure at 4 Hz.
class MeasureTest : public ::testing::Test {
 protected:
  struct Sent {
    double t;
    SubPacket packet;
  };

  explicit MeasureTest(McuControlConfig config = MeasureConfig()) : control_(config) {
  }

  void Run(double until) {
    while (now_ < until - 1e-9) {
      now_ = ++step_ * 0.01;
      control_.OnFrame();
      control_.OnSensorInfo(sensor_, now_);
      control_.OnWheelSpeed(wheel_linear_, 0.0f, now_);
      if (feed_setup_) control_.OnSetupMotion(true, now_);
      if (feed_imu_) control_.OnImu(ImuCounts{}, now_);
      if (step_ % 60 == 1) control_.OnVersions(versions_);
      if (measure_ && step_ % 25 == 1) {
        control_.OnFirmwareMeasure(true, cutter_, chassis_, hash_, clear_seq_, now_);
        last_signal_s_ = now_;
      }
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

  std::vector<SubPacket> TickAt(double t) const {
    std::vector<SubPacket> packets;
    for (const auto& s : sent_) {
      if (std::fabs(s.t - t) < 1e-6) packets.push_back(s.packet);
    }
    return packets;
  }

  std::vector<SubPacket> Moving() const {
    std::vector<SubPacket> packets;
    for (const auto& s : sent_) {
      if (IsMovingSpeed(s.packet)) packets.push_back(s.packet);
    }
    return packets;
  }

  // Releases the latch of the firmware once the window is open, then commands motion and the blade.
  bool DrivesOrMows(double until, float linear = 0.3f, float angular = 0.0f) {
    control_.ReleaseEmergency(now_, 0);
    Run(now_ + 0.05);
    const double from = now_;
    control_.OnCmdVel(linear, angular, now_);
    control_.SetBlade(BladeOn(), now_);
    Run(until);
    return !Times(IsMovingSpeed, from).empty() || !Times(IsBladeOn, from).empty();
  }

  McuControl control_;
  SensorInfo sensor_;
  VersionInfo versions_ = Versions("0.6.38", "0.6.34");
  bool measure_ = true;
  std::string cutter_ = "0.6.38";
  std::string chassis_ = "0.6.34";
  std::string hash_ = kHash;
  uint32_t clear_seq_ = 0;
  bool feed_setup_ = false;
  // An IMU sample every step: its frame would follow a setpoint in the same tick.
  bool feed_imu_ = false;
  float wheel_linear_ = 0;
  double last_signal_s_ = 0;
  int step_ = 0;
  double now_ = 0;
  std::vector<Sent> sent_;
};

TEST(FirmwareVersion, ParsesThreeNumbersUpTo255) {
  const auto v = ParseVersion("0.6.36");
  ASSERT_TRUE(v);
  EXPECT_EQ(v->major, 0);
  EXPECT_EQ(v->minor, 6);
  EXPECT_EQ(v->patch, 36);
  EXPECT_TRUE(ParseVersion("255.255.255"));
  for (const char* bad : {"", "0.6", "0.6.36.1", "0.6.256", "0.6.x", "0..36", " 0.6.36", "0.6.36 ", "-1.6.36",
                          "0.6.0036"}) {
    EXPECT_FALSE(ParseVersion(bad)) << bad;
  }
}

TEST(FirmwareVersion, MeasuredEntriesHoldOnlyForTheirDevice) {
  std::vector<std::string> malformed;
  int others = -1;
  const auto pairs = MeasuredFirmware({"0.6.38/0.6.34/" + kHash, "0.6.39/0.6.35/" + kOtherHash, "0.6.38/0.6.34",
                                       "0.6.38/0.6.34/" + std::string(64, 'A'), "0.6.38/0.6.34/" + kHash + "/x",
                                       "0.6.x/0.6.34/" + kHash, "0.6.38/0.6.34/" + std::string(63, 'a')},
                                      kHash, &malformed, &others);
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0].cutter, "0.6.38");
  EXPECT_EQ(pairs[0].chassis, "0.6.34");
  EXPECT_EQ(others, 1);
  EXPECT_EQ(malformed.size(), 5u);
  EXPECT_TRUE(MeasuredFirmware({"0.6.38/0.6.34/" + kHash}, "").empty());
}

TEST(FirmwareVersion, TheLongestSilenceAfterASetpointIsBounded) {
  McuControlConfig config;
  config.silence_last_setpoint_max_s = kMaxSilenceLastSetpointS;
  EXPECT_NO_THROW(CheckConfig(config));
  config.silence_last_setpoint_max_s = 5.01;
  EXPECT_THROW(CheckConfig(config), std::invalid_argument);
  config.silence_last_setpoint_max_s = std::nan("");
  EXPECT_THROW(CheckConfig(config), std::invalid_argument);
}

TEST_F(MeasureTest, TheSourceFollowsListMeasurementAndAge) {
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kUnknown);
  control_.OnVersions(Versions("0.6.36", "0.6.34"));
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kListed);
  control_.OnVersions(Versions("0.6.38", "0.6.34"));
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kNone);
  control_.OnVersions(Versions("0.6.30", "0.6.40"));
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kOlder);
  control_.OnVersions(Versions("0.6.38", "0.5.99"));
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kOlder);
  control_.OnConnected(now_);
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kUnknown);
  EXPECT_STREQ(FirmwareSourceText(FirmwareSource::kUnknown), "");
  EXPECT_STREQ(FirmwareSourceText(FirmwareSource::kMeasured), "measured");
}

McuControlConfig MeasuredConfig(std::string hash = kHash) {
  McuControlConfig config = MeasureConfig();
  config.device_hash = std::move(hash);
  config.measured_firmware = {{"0.6.38", "0.6.34"}, {"0.6.30", "0.6.34"}};
  return config;
}

class MeasuredTest : public MeasureTest {
 protected:
  MeasuredTest() : MeasureTest(MeasuredConfig()) {
    measure_ = false;
  }
};

TEST_F(MeasuredTest, AMeasuredPairDrivesWithoutLimitAndWithoutTheSignal) {
  Run(0.1);
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kMeasured);
  EXPECT_FALSE(control_.Status(now_).latched);
  EXPECT_TRUE(DrivesOrMows(0.3));
  ASSERT_FALSE(Moving().empty());
  EXPECT_NEAR(FastestWheel(Moving().back()), 0.3, 1e-6);
}

TEST_F(MeasuredTest, OnlyTheWholePairCounts) {
  versions_ = Versions("0.6.38", "0.6.35");
  Run(0.1);
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kNone);
  EXPECT_FALSE(DrivesOrMows(0.5));
}

TEST_F(MeasuredTest, AnEntryForOlderFirmwareDoesNotCount) {
  versions_ = Versions("0.6.30", "0.6.34");
  Run(0.1);
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kOlder);
  EXPECT_FALSE(DrivesOrMows(0.5));
}

class MeasuredWithoutIdTest : public MeasureTest {
 protected:
  MeasuredWithoutIdTest() : MeasureTest(MeasuredConfig("")) {
    hash_ = "";
  }
};

TEST_F(MeasuredWithoutIdTest, WithoutADeviceIdNothingIsMeasuredAndNoWindowOpens) {
  Run(0.1);
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kNone);
  EXPECT_FALSE(control_.Status(now_).measuring);
  EXPECT_FALSE(DrivesOrMows(0.5));
}

TEST_F(MeasureTest, UnlistedFirmwareIsBlockedWithoutTheSignal) {
  measure_ = false;
  Run(0.1);
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kNone);
  EXPECT_TRUE(control_.Status(now_).firmware_incompatible);
  EXPECT_FALSE(DrivesOrMows(0.5));
}

TEST_F(MeasureTest, TheSignalOpensTheWindowAndCapsEachWheel) {
  Run(0.1);
  const auto status = control_.Status(now_);
  EXPECT_TRUE(status.measuring);
  EXPECT_FALSE(status.firmware_incompatible);
  // The latch of the mismatch stays until released.
  EXPECT_TRUE(status.latched);
  EXPECT_TRUE(DrivesOrMows(0.3, 0.3f, 0.5f));
  ASSERT_FALSE(Moving().empty());
  for (const auto& packet : Moving()) EXPECT_LE(FastestWheel(packet), kMeasureMaxWheelSpeed + 1e-6);
  EXPECT_NEAR(FastestWheel(Moving().front()), kMeasureMaxWheelSpeed, 1e-6);
  EXPECT_FALSE(Times(IsBladeOn).empty());
}

TEST_F(MeasureTest, AStaleSignalDoesNotOpen) {
  Run(0.1);
  measure_ = false;
  Run(0.1 + kFirmwareMeasureFreshS + 0.03);
  EXPECT_FALSE(control_.Status(now_).measuring);
  EXPECT_FALSE(DrivesOrMows(now_ + 0.5));
}

TEST_F(MeasureTest, AnInactiveSignalDoesNotOpen) {
  measure_ = false;
  Run(0.05);
  control_.OnFirmwareMeasure(false, cutter_, chassis_, hash_, 0, now_);
  Run(0.06);
  EXPECT_FALSE(control_.Status(now_).measuring);
  EXPECT_FALSE(DrivesOrMows(0.5));
}

TEST_F(MeasureTest, AnotherFirmwareInTheSignalDoesNotOpen) {
  chassis_ = "0.6.35";
  Run(0.1);
  EXPECT_FALSE(control_.Status(now_).measuring);
  EXPECT_FALSE(DrivesOrMows(0.5));
}

TEST_F(MeasureTest, AnotherDeviceInTheSignalDoesNotOpen) {
  hash_ = kOtherHash;
  Run(0.1);
  EXPECT_FALSE(control_.Status(now_).measuring);
  EXPECT_FALSE(DrivesOrMows(0.5));
}

TEST_F(MeasureTest, OlderFirmwareIsNeverMeasured) {
  versions_ = Versions("0.6.30", "0.6.34");
  cutter_ = "0.6.30";
  Run(0.1);
  EXPECT_EQ(control_.Status(now_).firmware_source, FirmwareSource::kOlder);
  EXPECT_FALSE(control_.Status(now_).measuring);
  EXPECT_FALSE(DrivesOrMows(0.5));
}

TEST_F(MeasureTest, ListedFirmwareIsNotCappedByTheSignal) {
  versions_ = Versions("0.6.36", "0.6.34");
  cutter_ = "0.6.36";
  Run(0.1);
  EXPECT_FALSE(control_.Status(now_).measuring);
  EXPECT_TRUE(DrivesOrMows(0.3));
  ASSERT_FALSE(Moving().empty());
  EXPECT_NEAR(FastestWheel(Moving().front()), 0.3, 1e-6);
}

TEST_F(MeasureTest, WhenTheSignalEndsStopBladeOffAndANewLatch) {
  Run(0.1);
  ASSERT_TRUE(DrivesOrMows(0.4));
  control_.OnCmdVel(0.05f, 0.0f, now_);
  control_.SetBlade(BladeOn(), now_);
  Run(0.5);
  const uint32_t count = control_.Status(now_).latch_count;
  EXPECT_FALSE(control_.Status(now_).latched);
  measure_ = false;
  const double last = last_signal_s_;
  Run(1.2);
  const auto status = control_.Status(now_);
  EXPECT_FALSE(status.measuring);
  EXPECT_TRUE(status.latched);
  EXPECT_EQ(status.latch_count, count + 1);
  EXPECT_TRUE(status.emergency_reason & kReasonFirmwareIncompatible);
  const auto zeros = Times(IsZeroSpeed, last);
  ASSERT_GE(zeros.size(), 3u);
  EXPECT_LE(zeros.front(), last + kFirmwareMeasureFreshS + 0.02);
  ASSERT_FALSE(Times(IsBladeOff, last).empty());
  EXPECT_LE(Times(IsBladeOff, last).front(), last + kFirmwareMeasureFreshS + 0.02);
  EXPECT_TRUE(Times(IsMovingSpeed, zeros.front()).empty());
  // The signal again: the latch stays until a new release.
  measure_ = true;
  control_.OnCmdVel(0.05f, 0.0f, now_);
  Run(1.5);
  EXPECT_TRUE(control_.Status(now_).measuring);
  EXPECT_TRUE(Times(IsMovingSpeed, zeros.front()).empty());
}

TEST_F(MeasureTest, AnotherVersionFrameClosesTheWindow) {
  Run(0.1);
  ASSERT_TRUE(DrivesOrMows(0.3));
  const uint32_t count = control_.Status(now_).latch_count;
  versions_ = Versions("0.6.39", "0.6.34");
  control_.OnVersions(versions_);
  Run(0.32);
  EXPECT_FALSE(control_.Status(now_).measuring);
  EXPECT_TRUE(control_.Status(now_).latched);
  EXPECT_GT(control_.Status(now_).latch_count, count);
  EXPECT_FALSE(Times(IsZeroSpeed, 0.3).empty());
}

TEST_F(MeasureTest, AReconnectClosesTheWindow) {
  Run(0.1);
  ASSERT_TRUE(DrivesOrMows(0.3));
  control_.OnConnected(now_);
  Run(0.4);
  EXPECT_FALSE(control_.Status(now_).measuring);
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(MeasureTest, AClearChangeSendsExactlyOneClearWhileStanding) {
  Run(0.3);
  EXPECT_TRUE(Times(IsClear).empty());
  ++clear_seq_;
  Run(0.6);
  const auto clears = Times(IsClear);
  ASSERT_EQ(clears.size(), 1u);
  const auto tick = TickAt(clears[0]);
  const auto clear = std::find_if(tick.begin(), tick.end(), IsClear);
  ASSERT_NE(clear, tick.begin());
  EXPECT_TRUE(IsZeroSpeed(*(clear - 1)));
  const auto status = control_.Status(now_);
  EXPECT_EQ(status.clear_seq, 1u);
  ASSERT_TRUE(status.clear_s);
  EXPECT_NEAR(*status.clear_s, clears[0], 1e-9);
  ++clear_seq_;
  Run(0.9);
  EXPECT_EQ(Times(IsClear).size(), 2u);
  EXPECT_EQ(control_.Status(now_).clear_seq, 2u);
}

TEST_F(MeasureTest, TheFirstSignalOnlySetsWhereTheCountStands) {
  measure_ = false;
  clear_seq_ = 7;
  Run(0.2);
  measure_ = true;
  Run(0.6);
  EXPECT_TRUE(Times(IsClear).empty());
  EXPECT_EQ(control_.Status(now_).clear_seq, 0u);
}

TEST_F(MeasureTest, AClearWaitsForStandingWheels) {
  Run(0.1);
  ASSERT_TRUE(DrivesOrMows(0.3));
  control_.SetBlade(BladeRequest{}, now_);
  wheel_linear_ = 0.1f;
  ++clear_seq_;
  // The command ran out at 0.65, the wheels still report turning.
  Run(0.8);
  EXPECT_TRUE(Times(IsClear).empty());
  wheel_linear_ = 0.0f;
  Run(0.9);
  EXPECT_EQ(Times(IsClear).size(), 1u);
}

TEST_F(MeasureTest, AClearWaitsForTheBladeOffAndStanding) {
  Run(0.1);
  ASSERT_TRUE(DrivesOrMows(0.3, 0.0f, 0.0f));
  ++clear_seq_;
  // The blade is on, its motor not reported yet.
  Run(0.6);
  EXPECT_TRUE(Times(IsClear).empty());
  control_.OnBladeRpm(3000, now_);
  control_.SetBlade(BladeRequest{}, now_);
  Run(0.7);
  EXPECT_TRUE(Times(IsClear).empty());
  control_.OnBladeRpm(0, now_);
  Run(0.8);
  EXPECT_EQ(Times(IsClear).size(), 1u);
}

TEST_F(MeasureTest, AClearIsDroppedWhenTheSignalEnds) {
  Run(0.1);
  ASSERT_TRUE(DrivesOrMows(0.3));
  wheel_linear_ = 0.1f;
  ++clear_seq_;
  Run(0.5);
  measure_ = false;
  wheel_linear_ = 0.0f;
  Run(1.5);
  EXPECT_TRUE(Times(IsClear).empty());
  EXPECT_EQ(control_.Status(now_).clear_seq, 0u);
}

TEST_F(MeasureTest, AClearWithStopShownWaitsForTheMark) {
  sensor_.stop = true;
  Run(0.3);
  ++clear_seq_;
  Run(0.6);
  EXPECT_TRUE(Times(IsClear).empty());
  EXPECT_TRUE(control_.Status(now_).stop_clear_mark_wanted);
  control_.SetStopClearMark(true);
  Run(0.7);
  EXPECT_EQ(Times(IsClear).size(), 1u);
  EXPECT_FALSE(control_.Status(now_).stop_clear_mark_wanted);
}

class MeasureStopClearTest : public MeasureTest {
 protected:
  static McuControlConfig Config() {
    McuControlConfig config = MeasureConfig();
    config.stop_release = true;
    config.stop_clear_period_s = 0.25;
    config.stop_clear_delay_s = 0.1;
    return config;
  }
  MeasureStopClearTest() : MeasureTest(Config()) {
    versions_ = Versions("0.6.36", "0.6.34");
    cutter_ = "0.6.36";
  }
};

TEST_F(MeasureStopClearTest, STOPIsNotClearedOnItsOwnWhileTheSignalIsFresh) {
  control_.SetStopClearMark(true);
  sensor_.stop = true;
  Run(1.5);
  EXPECT_TRUE(Times(IsClear).empty());
  measure_ = false;
  Run(3.0);
  EXPECT_FALSE(Times(IsClear, 1.5 + kFirmwareMeasureFreshS).empty());
}

TEST_F(MeasureStopClearTest, AClearAskedForGoesAlsoWithListedFirmware) {
  control_.SetStopClearMark(true);
  sensor_.stop = true;
  Run(0.5);
  ++clear_seq_;
  Run(0.8);
  EXPECT_EQ(Times(IsClear).size(), 1u);
}

class LastSetpointTest : public MeasureTest {
 protected:
  static McuControlConfig Config(double max) {
    McuControlConfig config = MeasureConfig();
    config.silence_max_s = 3.0;
    config.silence_last_setpoint_max_s = max;
    config.speed_heartbeat = McuControlConfig::SpeedHeartbeat::kLater;
    return config;
  }
  explicit LastSetpointTest(double max = 5.0) : MeasureTest(Config(max)) {
    versions_ = Versions("0.6.36", "0.6.34");
    measure_ = false;
    feed_setup_ = true;
    feed_imu_ = true;
  }
  void Drive(double until) {
    while (now_ < until - 1e-9) {
      control_.OnCmdVel(0.05f, 0.0f, now_);
      Run(std::min(until, now_ + 0.1));
    }
  }
};

TEST_F(LastSetpointTest, TheSetpointIsTheLastFrameAndTheStopsWait) {
  Run(0.3);
  Drive(0.85);
  ASSERT_EQ(control_.SilenceLastSetpoint(4.0, now_), HostTestRefusal::kNone);
  EXPECT_TRUE(control_.Status(now_).silence_armed);
  EXPECT_FALSE(control_.Status(now_).silent_from_s);
  Drive(1.2);
  ASSERT_TRUE(control_.Status(now_).silent);
  EXPECT_FALSE(control_.Status(now_).silence_armed);
  const auto moving = Times(IsMovingSpeed, 0.85);
  ASSERT_FALSE(moving.empty());
  const double start = moving.back();
  ASSERT_TRUE(control_.Status(now_).silent_from_s);
  EXPECT_DOUBLE_EQ(*control_.Status(now_).silent_from_s, start);
  const auto tick = TickAt(start);
  ASSERT_GE(tick.size(), 2u);
  EXPECT_TRUE(IsMovingSpeed(tick.back()));
  EXPECT_TRUE(std::any_of(tick.begin(), tick.end(), [](const SubPacket& p) { return IsModule(p, Module::kImu); }));
  Run(start + 4.0 + 1.0);
  const double end = start + 4.0;
  EXPECT_TRUE(Times([](const SubPacket&) { return true; }, start + 0.001, end - 0.011).empty());
  const auto zeros = Times(IsZeroSpeed, end - 0.01);
  ASSERT_EQ(zeros.size(), 3u);
  EXPECT_NEAR(zeros[0], end + kLastSetpointStopDelayS, 0.011);
  EXPECT_GE(Times(IsHeartbeat, end - 0.01, end + kLastSetpointStopDelayS - 0.01).size(), 4u);
  EXPECT_TRUE(Times(IsMovingSpeed, start + 0.001).empty());
  EXPECT_TRUE(control_.Status(now_).latched);
}

TEST_F(LastSetpointTest, ItsOwnLongestAndTheOtherRefusals) {
  Run(0.3);
  EXPECT_EQ(control_.SilenceLastSetpoint(5.01, now_), HostTestRefusal::kOutOfRange);
  EXPECT_EQ(control_.SilenceLastSetpoint(0.1, now_), HostTestRefusal::kOutOfRange);
  EXPECT_EQ(control_.Silence(4.0, now_), HostTestRefusal::kOutOfRange);
  feed_setup_ = false;
  Run(0.3 + kSetupMotionFreshS + 0.05);
  EXPECT_EQ(control_.SilenceLastSetpoint(4.0, now_), HostTestRefusal::kNoSetup);
  feed_setup_ = true;
  Run(now_ + 0.05);
  ASSERT_EQ(control_.SilenceLastSetpoint(4.0, now_), HostTestRefusal::kNone);
  EXPECT_EQ(control_.SilenceLastSetpoint(4.0, now_), HostTestRefusal::kBusy);
  // Without a setpoint within the arm time it is dropped.
  Run(now_ + kSilenceArmS + 0.05);
  EXPECT_FALSE(control_.Status(now_).silent);
  EXPECT_EQ(control_.SilenceLastSetpoint(4.0, now_), HostTestRefusal::kNone);
}

class NoLastSetpointTest : public LastSetpointTest {
 protected:
  NoLastSetpointTest() : LastSetpointTest(0.0) {
  }
};

TEST_F(NoLastSetpointTest, WithoutItsLongestItIsRefused) {
  Run(0.3);
  EXPECT_EQ(control_.SilenceLastSetpoint(1.0, now_), HostTestRefusal::kLastSetpoint);
  EXPECT_STREQ(HostTestRefusalCode(HostTestRefusal::kLastSetpoint), "LAST_SETPOINT");
  EXPECT_EQ(control_.Silence(1.0, now_), HostTestRefusal::kNone);
}

TEST(McuJournal, ConfigFromBeforeTheMeasurementReplays) {
  nlohmann::json j = McuControlConfig{};
  j.erase("measured_firmware");
  j.erase("device_hash");
  j.erase("silence_last_setpoint_max_s");
  const auto config = j.get<McuControlConfig>();
  EXPECT_TRUE(config.measured_firmware.empty());
  EXPECT_TRUE(config.device_hash.empty());
  EXPECT_EQ(config.silence_last_setpoint_max_s, 0.0);
  McuControlConfig measured = MeasureConfig();
  measured.measured_firmware = {{"0.6.38", "0.6.34"}};
  measured.silence_last_setpoint_max_s = 5.0;
  const auto back = nlohmann::json(measured).get<McuControlConfig>();
  EXPECT_EQ(back.measured_firmware, measured.measured_firmware);
  EXPECT_EQ(back.device_hash, kHash);
  EXPECT_EQ(back.silence_last_setpoint_max_s, 5.0);
}

}  // namespace
