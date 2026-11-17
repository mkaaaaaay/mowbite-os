#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "xcore_driver/services.h"
#include "xcore_driver/xcore_control.h"

using namespace xcore;

namespace {

Bytes U16(uint16_t v) {
  Writer w;
  w.U16(v);
  return w.Take();
}

Bytes F32(float v) {
  Writer w;
  w.F32(v);
  return w.Take();
}

Bytes U64(uint64_t v) {
  Writer w;
  w.U64(v);
  return w.Take();
}

std::vector<InputValue> Of(const std::vector<InputValue>& all, uint16_t service) {
  std::vector<InputValue> out;
  for (const auto& v : all) {
    if (v.service_id == service) out.push_back(v);
  }
  return out;
}

std::vector<double> TwistOf(const InputValue& v) {
  return *DecodeDoubles(v.value, 6);
}

float SpeedOf(const InputValue& v) {
  return Reader(v.value).F32();
}

std::pair<uint16_t, uint16_t> EmergencyOf(const InputValue& v) {
  Reader r(v.value);
  uint16_t set = r.U16();
  return {set, r.U16()};
}

InputMap Inputs() {
  InputMap m;
  m.stop = {0, 1};
  m.lift = {2, 3};
  m.bumper_left = {5};
  m.bumper_right = {4};
  return m;
}

// A control with all services up and a board that reports no emergency.
struct Fixture {
  explicit Fixture(ControlConfig config = Config()) : control(config, Inputs()) {
    control.SetServicesReady(true, true);
    Board(0, 0);
  }
  static ControlConfig Config() {
    ControlConfig c;
    c.allow_blade = true;
    return c;
  }
  void Board(uint16_t reason, double now) {
    control.OnOutput(kEmergencyService, emergency::kOutReason, U16(reason), 0, now);
  }
  XcoreControl control;
};

}  // namespace

TEST(XcoreControl, EmergencyHeartbeatGoesOutRegularly) {
  Fixture f;
  int count = 0;
  for (double t = 0; t < 1.0 - 1e-9; t += 0.01) {
    f.Board(0, t);
    for (const auto& v : Of(f.control.Tick(t), kEmergencyService)) {
      EXPECT_EQ(EmergencyOf(v), std::make_pair(uint16_t{0}, uint16_t{0}));
      count++;
    }
  }
  EXPECT_EQ(count, 4);
}

TEST(XcoreControl, BlockedUntilServicesAndTelemetry) {
  XcoreControl control(Fixture::Config(), Inputs());
  auto s = control.Status(0);
  EXPECT_TRUE(s.blocked);
  EXPECT_TRUE(s.emergency_reason & reason::kServiceNotReady);
  EXPECT_TRUE(s.emergency_reason & kReasonTimeoutInputs);
  control.SetServicesReady(true, true);
  control.OnOutput(kEmergencyService, emergency::kOutReason, U16(0), 0, 0);
  EXPECT_FALSE(control.Status(0.5).blocked);
  // The board falls silent; it reports every second.
  EXPECT_FALSE(control.Status(2.4).blocked);
  EXPECT_EQ(control.Status(2.6).emergency_reason, kReasonTimeoutInputs);
  control.SetServicesReady(true, false);
  EXPECT_TRUE(control.Status(0.5).emergency_reason & kReasonFirmwareIncompatible);
}

TEST(XcoreControl, BoardReasonsPassWithTheirBits) {
  Fixture f;
  f.Board(reason::kLatch | reason::kLift | reason::kLiftMultiple | reason::kTimeoutHighLevel, 0);
  auto s = f.control.Status(0);
  EXPECT_TRUE(s.blocked);
  EXPECT_TRUE(s.latched);
  EXPECT_EQ(s.emergency_reason, uint32_t{reason::kLatch | reason::kLift | reason::kLiftMultiple |
                                         reason::kTimeoutHighLevel});
}

TEST(XcoreControl, DriveIsLimitedAndStopsAfterSilence) {
  Fixture f;
  f.control.OnCmdVel(1.0, -2.0, 0);
  auto drive = Of(f.control.Tick(0), kDiffDriveService);
  ASSERT_EQ(drive.size(), 1u);
  auto twist = TwistOf(drive[0]);
  // 0.3 m/s and -0.3 rad/s, as duty: the outer wheel at the duty limit, the inner one shortened by the same factor.
  double half = ControlConfig{}.regulator.wheel_distance / 2;
  EXPECT_NEAR(twist[0] - twist[5] * half, 0.3, 1e-9);
  EXPECT_NEAR(twist[0] + twist[5] * half, (0.3 - 0.3 * half) * 0.3 / (0.3 + 0.3 * half), 1e-9);
  EXPECT_EQ(twist[1] + twist[2] + twist[3] + twist[4], 0);
  EXPECT_TRUE(f.control.Status(0).moving);
  // Resent every 0.1 s until cmd_vel falls silent for 0.5 s, then three stops 0.1 s apart, then nothing.
  std::vector<std::pair<double, std::vector<double>>> sent;
  for (int i = 1; i <= 150; i++) {
    double t = i * 0.01;
    f.Board(0, t);
    for (const auto& v : Of(f.control.Tick(t), kDiffDriveService)) sent.emplace_back(t, TwistOf(v));
  }
  int moving = 0, stops = 0;
  for (const auto& [t, v] : sent) {
    if (v[0] != 0) {
      moving++;
      EXPECT_LE(t, 0.5 + 1e-9);
    } else {
      stops++;
      EXPECT_GT(t, 0.5);
      EXPECT_EQ(v, std::vector<double>(6, 0.0));
    }
  }
  EXPECT_EQ(moving, 5);
  EXPECT_EQ(stops, 3);
  EXPECT_NEAR(sent.back().first, 0.71, 1e-9);
  EXPECT_FALSE(f.control.Status(1.5).moving);
}

TEST(XcoreControl, NonFiniteCommandStops) {
  Fixture f;
  f.control.OnCmdVel(0.2, 0, 0);
  f.control.Tick(0);
  f.control.OnCmdVel(std::nan(""), std::nan(""), 0.05);
  auto drive = Of(f.control.Tick(0.05), kDiffDriveService);
  ASSERT_EQ(drive.size(), 1u);
  EXPECT_EQ(TwistOf(drive[0])[0], 0);
}

TEST(XcoreControl, EmergencyStopsWheelsAndBlade) {
  Fixture f;
  f.control.OnCmdVel(0.2, 0, 0);
  ASSERT_EQ(f.control.SetBlade({true, 0, 0, 1.0}, 0), BladeResult::kOk);
  auto first = f.control.Tick(0);
  ASSERT_EQ(Of(first, kMowerService).size(), 1u);
  EXPECT_FLOAT_EQ(SpeedOf(Of(first, kMowerService)[0]), 1.0f);
  f.Board(reason::kLatch | reason::kStop, 0.05);
  auto out = f.control.Tick(0.05);
  ASSERT_EQ(Of(out, kDiffDriveService).size(), 1u);
  EXPECT_EQ(TwistOf(Of(out, kDiffDriveService)[0])[0], 0);
  ASSERT_EQ(Of(out, kMowerService).size(), 1u);
  EXPECT_EQ(SpeedOf(Of(out, kMowerService)[0]), 0);
  auto s = f.control.Status(0.05);
  EXPECT_FALSE(s.blade_on);
  EXPECT_EQ(s.blade_off_reason, BladeOffReason::kBlocked);
  // Commands while blocked do nothing.
  f.control.OnCmdVel(0.2, 0, 0.1);
  for (const auto& v : Of(f.control.Tick(0.1), kDiffDriveService)) EXPECT_EQ(TwistOf(v)[0], 0);
  uint32_t bits = 0;
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 1.0}, 0.1, &bits), BladeResult::kBlocked);
  // The refusal names the emergency as hw/status does.
  EXPECT_EQ(bits, f.control.Status(0.1).emergency_reason);
  EXPECT_EQ(bits & (reason::kLatch | reason::kStop), reason::kLatch | reason::kStop);
}

TEST(XcoreControl, BladeLease) {
  Fixture f;
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 0}, 0), BladeResult::kInvalidLease);
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 2.5}, 0), BladeResult::kInvalidLease);
  EXPECT_EQ(f.control.SetBlade({true, 101, 0, 1}, 0), BladeResult::kInvalidSpeed);
  uint32_t bits = 1;
  EXPECT_EQ(f.control.SetBlade({true, 0, 50, 1}, 0, &bits), BladeResult::kInvalidHeight);
  EXPECT_EQ(bits, 0u);
  bits = 1;
  ASSERT_EQ(f.control.SetBlade({true, 50, 0, 0.5}, 0, &bits), BladeResult::kOk);
  EXPECT_EQ(bits, 0u);
  std::vector<std::pair<double, float>> sent;
  for (int i = 0; i <= 120; i++) {
    double t = i * 0.01;
    f.Board(0, t);
    for (const auto& v : Of(f.control.Tick(t), kMowerService)) sent.emplace_back(t, SpeedOf(v));
  }
  // One on at once, the lease ends at 0.5 s without renewal: three offs 0.1 s apart, the next one a second later.
  ASSERT_EQ(sent.size(), 4u);
  EXPECT_EQ(sent[0], std::make_pair(0.0, 0.5f));
  EXPECT_NEAR(sent[1].first, 0.5, 1e-9);
  EXPECT_EQ(sent[1].second, 0.0f);
  EXPECT_NEAR(sent[2].first, 0.6, 1e-9);
  EXPECT_NEAR(sent[3].first, 0.7, 1e-9);
  EXPECT_EQ(f.control.Status(1.2).blade_off_reason, BladeOffReason::kLeaseExpired);
  XcoreControl closed(ControlConfig{}, Inputs());
  EXPECT_EQ(closed.SetBlade({true, 0, 0, 1}, 0), BladeResult::kNotAllowed);
}

TEST(XcoreControl, BladeOffIsSentRegularly) {
  Fixture f;
  int offs = 0;
  for (int i = 0; i < 300; i++) {
    double t = i * 0.01;
    f.Board(0, t);
    for (const auto& v : Of(f.control.Tick(t), kMowerService)) {
      EXPECT_EQ(SpeedOf(v), 0);
      offs++;
    }
  }
  EXPECT_EQ(offs, 3);
}

TEST(XcoreControl, BladeStillTurningLatches) {
  Fixture f;
  f.control.SetBlade({true, 0, 0, 1}, 0);
  f.control.Tick(0);
  f.control.SetBlade({false, 0, 0, 0}, 0.1);
  f.control.Tick(0.1);
  f.Board(0, 2.0);
  f.control.OnOutput(kMowerService, mower::kOutRpm, F32(3000), 0, 2.0);
  EXPECT_FALSE(f.control.Status(2.0).blocked);
  f.Board(0, 5.1);
  f.control.OnOutput(kMowerService, mower::kOutRpm, F32(500), 0, 5.2);
  auto s = f.control.Status(5.2);
  EXPECT_TRUE(s.blocked);
  EXPECT_TRUE(s.latched);
  EXPECT_TRUE(s.emergency_reason & kReasonMowerNotStopped);
  // The board gets a high level emergency, the blade more offs. The I/O thread ran in between.
  f.control.Alive(5.2);
  auto out = f.control.Tick(5.2);
  ASSERT_FALSE(Of(out, kEmergencyService).empty());
  EXPECT_EQ(EmergencyOf(Of(out, kEmergencyService)[0]).first, reason::kHighLevel | reason::kLatch);
  ASSERT_FALSE(Of(out, kMowerService).empty());
  // A release while it still turns keeps the latch, a slow report lets it go.
  f.Board(0, 5.3);
  f.control.ReleaseEmergency(5.3);
  EXPECT_TRUE(f.control.Status(5.3).emergency_reason & kReasonMowerNotStopped);
  f.control.OnOutput(kMowerService, mower::kOutRpm, F32(10), 0, 5.4);
  f.control.ReleaseEmergency(5.4);
  EXPECT_FALSE(f.control.Status(5.4).blocked);
}

TEST(XcoreControl, HostLatchAndRelease) {
  Fixture f;
  EXPECT_FALSE(f.control.LatchEmergency(kReasonLift));
  EXPECT_TRUE(f.control.LatchEmergency(kReasonTilt));
  auto s = f.control.Status(0);
  EXPECT_EQ(s.emergency_reason, kReasonTilt | kReasonLatch);
  auto out = Of(f.control.Tick(0), kEmergencyService);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(EmergencyOf(out[0]), std::make_pair(uint16_t(reason::kHighLevel | reason::kLatch), uint16_t{0}));
  EXPECT_TRUE(f.control.LatchEmergency(0));
  EXPECT_EQ(f.control.Status(0).emergency_reason, kReasonTilt | kReasonHighLevel | kReasonLatch);
  // Release: three clears 0.1 s apart, then plain heartbeats again.
  f.control.ReleaseEmergency(0.05);
  EXPECT_FALSE(f.control.Status(0.05).blocked);
  std::vector<std::pair<double, std::pair<uint16_t, uint16_t>>> sent;
  for (int i = 5; i <= 60; i++) {
    double t = i * 0.01;
    f.Board(0, t);
    for (const auto& v : Of(f.control.Tick(t), kEmergencyService)) sent.emplace_back(t, EmergencyOf(v));
  }
  ASSERT_GE(sent.size(), 4u);
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(sent[i].first, 0.05 + 0.1 * i, 1e-9);
    EXPECT_EQ(sent[i].second.first, 0);
    EXPECT_EQ(sent[i].second.second,
              reason::kLatch | reason::kHighLevel | reason::kMowerRpmTimeout | reason::kMowerRpmLimit);
  }
  EXPECT_EQ(sent[3].second, std::make_pair(uint16_t{0}, uint16_t{0}));
}

TEST(XcoreControl, InputsAndDock) {
  Fixture f;
  f.control.OnOutput(kInputService, input::kOutActive, U64((1u << 1) | (1u << 4)), 0, 0);
  auto s = f.control.Status(0);
  EXPECT_TRUE(s.stop);
  EXPECT_FALSE(s.lift);
  EXPECT_TRUE(s.bumper_right);
  EXPECT_FALSE(s.bumper_left);
  for (int i = 1; i < 2; i++) {
    f.Board(0, i);
    f.control.OnOutput(kPowerService, power::kOutChargeVoltage, F32(29.0f), 0, i);
  }
  f.Board(0, 2);
  s = f.control.Status(2);
  EXPECT_TRUE(s.dock_contact_raw);
  EXPECT_FALSE(s.dock_contact);
  f.control.OnOutput(kPowerService, power::kOutChargeVoltage, F32(29.0f), 0, 2);
  EXPECT_TRUE(f.control.Status(2).dock_contact);
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 1}, 2), BladeResult::kDocked);
  f.control.OnOutput(kPowerService, power::kOutChargeVoltage, F32(0.5f), 0, 3);
  s = f.control.Status(3);
  EXPECT_FALSE(s.dock_contact_raw);
  EXPECT_FALSE(s.dock_contact);
}

TEST(XcoreControl, ChargingIsLeftToTheBoard) {
  Fixture f;
  for (int i = 0; i < 30; i++) EXPECT_TRUE(Of(f.control.Tick(i * 0.1), kPowerService).empty());
}

TEST(XcoreControl, LostMessagesLatch) {
  // Before the first message arrives the board reports TIMEOUT_HIGH_LEVEL; that does not latch.
  XcoreControl control(Fixture::Config(), Inputs());
  control.SetServicesReady(true, true);
  control.OnOutput(kEmergencyService, emergency::kOutReason, U16(reason::kTimeoutHighLevel), 0, 0);
  control.OnOutput(kEmergencyService, emergency::kOutReason, U16(0), 0, 0.5);
  EXPECT_FALSE(control.Status(0.5).blocked);
  // Later it means the board missed the host for a second: the driver keeps it stopped until a release.
  control.OnOutput(kEmergencyService, emergency::kOutReason, U16(reason::kTimeoutHighLevel), 0, 1);
  control.OnOutput(kEmergencyService, emergency::kOutReason, U16(0), 0, 1.5);
  auto s = control.Status(1.5);
  EXPECT_TRUE(s.blocked);
  EXPECT_TRUE(s.latched);
  EXPECT_TRUE(s.emergency_reason & reason::kTimeoutHighLevel);
  auto out = Of(control.Tick(1.5), kEmergencyService);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(EmergencyOf(out[0]).first, reason::kHighLevel | reason::kLatch);
  control.ReleaseEmergency(1.6);
  EXPECT_FALSE(control.Status(1.6).blocked);
}

TEST(XcoreControl, WheelAgainstItsSetpointLatches) {
  Fixture f;
  f.control.OnCmdVel(0.2, 0, 0);
  Writer w0;
  w0.U32(1000);
  w0.U32(1000);
  f.control.OnOutput(kDiffDriveService, diff_drive::kOutTicks, w0.Take(), 0, 0);
  uint32_t left = 1000;
  for (int i = 1; i <= 20; i++) {
    double t = i * 0.04;
    // The left wheel runs backwards, the right one forward (its counter falls).
    left -= 20;
    Writer w;
    w.U32(left);
    w.U32(1000 - 16 * i);
    f.Board(0, t);
    f.control.OnCmdVel(0.2, 0, t);
    f.control.OnOutput(kDiffDriveService, diff_drive::kOutTicks, w.Take(), static_cast<uint64_t>(t * 1e6), t);
    f.control.Tick(t);
  }
  auto s = f.control.Status(0.8);
  EXPECT_TRUE(s.blocked);
  EXPECT_TRUE(s.latched);
  EXPECT_FALSE(s.moving);
}

TEST(XcoreControl, BatteryAndOdometry) {
  Fixture f;
  EXPECT_FALSE(f.control.Status(0).battery.known);
  f.control.OnOutput(kPowerService, power::kOutBatteryVoltage, F32(26.5f), 0, 0);
  f.control.OnOutput(kPowerService, power::kOutBatteryPercentage, F32(0.55f), 0, 0);
  auto s = f.control.Status(1);
  EXPECT_TRUE(s.battery.known);
  EXPECT_FLOAT_EQ(s.battery.voltage, 26.5f);
  EXPECT_NEAR(s.battery.fraction, 0.55, 1e-6);
  EXPECT_FALSE(f.control.Status(6).battery.known);
  Writer w;
  w.U32(0);
  w.U32(0);
  EXPECT_FALSE(f.control.OnOutput(kDiffDriveService, diff_drive::kOutTicks, w.Take(), 1000000, 0));
  Writer w2;
  w2.U32(81);
  w2.U32(static_cast<uint32_t>(-81));
  auto twist = f.control.OnOutput(kDiffDriveService, diff_drive::kOutTicks, w2.Take(), 1040000, 0.04);
  ASSERT_TRUE(twist);
  EXPECT_NEAR(twist->linear, 81 / 2025.0 / 0.04, 1e-9);
}

TEST(XcoreControl, RestartOfTheMowerServiceSendsOff) {
  Fixture f;
  f.control.SetBlade({true, 0, 0, 1}, 0);
  f.control.Tick(0);
  f.control.OnServiceRestart(kMowerService, 0.1);
  auto out = Of(f.control.Tick(0.1), kMowerService);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(SpeedOf(out[0]), 0);
  EXPECT_EQ(f.control.Status(0.1).blade_off_reason, BladeOffReason::kReconnect);
}

TEST(XcoreControl, RestartForgetsTheMotors) {
  Fixture f;
  Bytes status{kEscRunning};
  f.control.OnOutput(kDiffDriveService, diff_drive::kOutLeftStatus, status, 0, 0);
  f.control.OnOutput(kMowerService, mower::kOutStatus, status, 0, 0);
  EXPECT_TRUE(f.control.Status(0).left_motor.known);
  EXPECT_TRUE(f.control.Status(0).blade_motor.known);
  f.control.OnServiceRestart(kDiffDriveService, 0.1);
  f.control.OnServiceRestart(kMowerService, 0.1);
  EXPECT_FALSE(f.control.Status(0.1).left_motor.known);
  EXPECT_FALSE(f.control.Status(0.1).blade_motor.known);
}

TEST(XcoreControl, StopInputs) {
  Fixture f;
  auto out = f.control.StopInputs();
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].service_id, kDiffDriveService);
  EXPECT_EQ(TwistOf(out[0]), std::vector<double>(6, 0.0));
  EXPECT_EQ(out[1].service_id, kMowerService);
  EXPECT_EQ(SpeedOf(out[1]), 0);
}

TEST(XcoreControl, MotorStatusCodes) {
  EXPECT_EQ(MotorStatusCode(kEscOk), 0);
  EXPECT_EQ(MotorStatusCode(kEscRunning), 1);
  EXPECT_EQ(MotorStatusCode(kEscBlocked), 2);
  EXPECT_EQ(MotorStatusCode(kEscFault), -1);
  EXPECT_EQ(MotorStatusCode(kEscDisconnected), -2);
  EXPECT_EQ(MotorStatusCode(7), -3);
}

TEST(XcoreControl, LatchedCauseStaysUntilRelease) {
  Fixture f;
  f.Board(reason::kLift | reason::kLatch, 0);
  EXPECT_EQ(f.control.Status(0).emergency_reason, kReasonLift | kReasonLatch);
  // The lift is free again; the board keeps only its latch, the driver the cause as well.
  f.Board(reason::kLatch, 0.5);
  EXPECT_EQ(f.control.Status(0.5).emergency_reason, kReasonLift | kReasonLatch);
  // The board restarts and forgets its latch: the driver sets it again.
  f.control.OnServiceRestart(kEmergencyService, 1);
  f.Board(reason::kTimeoutHighLevel, 1.1);
  EXPECT_TRUE(f.control.Status(1.1).emergency_reason & kReasonLift);
  auto out = Of(f.control.Tick(1.1), kEmergencyService);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(EmergencyOf(out[0]).first, reason::kLatch);
  f.Board(reason::kLatch, 1.3);
  f.control.ReleaseEmergency(1.4);
  out = Of(f.control.Tick(1.4), kEmergencyService);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(EmergencyOf(out[0]).first, 0);
  EXPECT_TRUE(EmergencyOf(out[0]).second & reason::kLatch);
  f.Board(0, 1.5);
  EXPECT_FALSE(f.control.Status(1.5).blocked);
}

TEST(XcoreControl, CauseStillThereLatchesAgainAfterRelease) {
  Fixture f;
  f.Board(reason::kStop | reason::kLatch, 0);
  f.control.ReleaseEmergency(0.1);
  f.Board(reason::kStop | reason::kLatch, 0.2);
  EXPECT_EQ(f.control.Status(0.2).emergency_reason, kReasonStop | kReasonLatch);
  f.Board(reason::kLatch, 0.3);
  EXPECT_EQ(f.control.Status(0.3).emergency_reason, kReasonStop | kReasonLatch);
}

TEST(XcoreControl, EveryReasonThatLatchesCounts) {
  Fixture f;
  EXPECT_EQ(f.control.Status(0).latch_count, 0u);
  f.Board(reason::kStop | reason::kLatch, 0.1);
  f.control.Tick(0.1);
  EXPECT_EQ(f.control.Status(0.1).latch_count, 1u);
  // The same reason once more is none, another one is.
  f.Board(reason::kStop | reason::kLatch, 0.2);
  f.control.Tick(0.2);
  EXPECT_EQ(f.control.Status(0.2).latch_count, 1u);
  f.Board(reason::kCollision | reason::kLatch, 0.3);
  f.control.Tick(0.3);
  EXPECT_EQ(f.control.Status(0.3).latch_count, 2u);
  // After the release the reason counts again.
  f.Board(reason::kLatch, 0.4);
  EXPECT_FALSE(f.control.ReleaseEmergency(0.4).has_value());
  f.Board(0, 0.5);
  f.Board(reason::kStop | reason::kLatch, 0.6);
  f.control.Tick(0.6);
  EXPECT_EQ(f.control.Status(0.6).latch_count, 3u);
  // A status between the report of the board and the next tick has the count of its latch already.
  f.Board(reason::kStop | reason::kLift | reason::kLatch, 0.7);
  EXPECT_EQ(f.control.Status(0.7).emergency_reason, kReasonStop | kReasonLift | kReasonLatch);
  EXPECT_EQ(f.control.Status(0.7).latch_count, 4u);
  f.control.Tick(0.7);
  EXPECT_EQ(f.control.Status(0.7).latch_count, 4u);
}

TEST(XcoreControl, AReleaseWithTheCountOfBeforeAnotherReasonIsRefused) {
  Fixture f;
  f.Board(reason::kStop | reason::kLatch, 0.1);
  f.control.Tick(0.1);
  const uint32_t count = f.control.Status(0.1).latch_count;
  f.Board(reason::kLatch, 0.2);
  // The lift latches before the release arrives, without a tick between.
  f.Board(reason::kLift | reason::kLatch, 0.3);
  f.Board(reason::kLatch, 0.35);
  const auto refused = f.control.ReleaseEmergency(0.4, count);
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->code, "LATCH_CHANGED");
  EXPECT_EQ(f.control.Status(0.4).emergency_reason, kReasonStop | kReasonLift | kReasonLatch);
  EXPECT_EQ(f.control.Status(0.4).latch_count, count + 1);
  // With the count of now it releases, as without one.
  EXPECT_FALSE(f.control.ReleaseEmergency(0.5, count + 1).has_value());
  f.Board(0, 0.6);
  EXPECT_FALSE(f.control.Status(0.6).blocked);
}

TEST(XcoreControl, LearningHoldsTheBoardAndTakesNoRelease) {
  ControlConfig config = Fixture::Config();
  config.learn_inputs = true;
  Fixture f(config);
  EXPECT_EQ(f.control.Status(0).emergency_reason, kReasonInputsLearning | kReasonLatch);
  auto out = Of(f.control.Tick(0), kEmergencyService);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(EmergencyOf(out[0]).first, reason::kHighLevel | reason::kLatch);
  auto refused = f.control.ReleaseEmergency(0.1);
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->code, "LEARNING");
  // A latch from above goes with a release as always, the one of learning stays and holds the board.
  ASSERT_TRUE(f.control.LatchEmergency(kReasonHighLevel));
  EXPECT_EQ(f.control.Status(0.1).emergency_reason, kReasonInputsLearning | kReasonLatch | kReasonHighLevel);
  refused = f.control.ReleaseEmergency(0.15);
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->code, "LEARNING");
  EXPECT_EQ(f.control.Status(0.15).emergency_reason, kReasonInputsLearning | kReasonLatch);
  out = Of(f.control.Tick(0.15), kEmergencyService);
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(EmergencyOf(out.back()).first, reason::kHighLevel | reason::kLatch);
  f.control.OnCmdVel(0.2, 0, 0.2);
  EXPECT_TRUE(Of(f.control.Tick(0.2), kDiffDriveService).empty());
  EXPECT_TRUE(f.control.Status(0.2).blocked);
  // The lines show as the board reports them.
  f.control.OnOutput(kInputService, input::kOutActive, U64(1u << 3), 0, 0.3);
  ASSERT_TRUE(f.control.Status(0.3).inputs);
  EXPECT_EQ(*f.control.Status(0.3).inputs, 1u << 3);
}

namespace {

constexpr double kG = 9.80665;

// Feeds the IMU with gravity tilted by deg about the x axis.
void Tilt(XcoreControl& control, double deg, double now) {
  const double r = deg * M_PI / 180.0;
  control.OnImu(0, kG * std::sin(r), kG * std::cos(r), now);
}

// Counters of the board for wheel speeds in m/s, forward positive; the right one falls going forward.
struct Wheels {
  double left = 0;
  double right = 0;
  double board_s = 100;
  Bytes Step(double v_left, double v_right, double dt) {
    left += v_left * 2025.0 * dt;
    right -= v_right * 2025.0 * dt;
    board_s += dt;
    Writer w;
    w.U32(static_cast<uint32_t>(static_cast<int64_t>(std::llround(left))));
    w.U32(static_cast<uint32_t>(static_cast<int64_t>(std::llround(right))));
    return w.Take();
  }
  uint64_t us() const {
    return static_cast<uint64_t>(board_s * 1e6);
  }
};

// Runs the control up to t1 in steps of 10 ms with fresh board reports, ticks at 40 ms and the IMU level.
struct DriveRun {
  DriveRun() = default;
  explicit DriveRun(const ControlConfig& config) : f(config) {}
  Fixture f;
  Wheels wheels;
  std::vector<InputValue> sent;
  double t = 0;
  void Until(double t1, double v_left, double v_right, bool command = false, double v_command = 0.1,
             double w_command = 0) {
    for (; t < t1 - 1e-9; t += 0.01) {
      f.Board(0, t);
      Tilt(f.control, 0, t);
      if (command) f.control.OnCmdVel(v_command, w_command, t);
      if (std::lround(t * 100) % 4 == 0) {
        f.control.OnOutput(kDiffDriveService, diff_drive::kOutTicks, wheels.Step(v_left, v_right, 0.04),
                           wheels.us(), t);
      }
      auto out = f.control.Tick(t);
      sent.insert(sent.end(), out.begin(), out.end());
    }
  }
};

bool HighLevelLatch(const std::vector<InputValue>& out) {
  for (const auto& v : Of(out, kEmergencyService)) {
    if (EmergencyOf(v).first == (reason::kHighLevel | reason::kLatch)) return true;
  }
  return false;
}

}  // namespace

TEST(XcoreControl, TiltLatchesAfterHalfASecond) {
  Fixture f;
  ASSERT_EQ(f.control.SetBlade({true, 0, 0, 2.0}, 0), BladeResult::kOk);
  // 0.3 s over the limit and back: nothing.
  for (double t = 0; t < 0.3; t += 0.01) Tilt(f.control, 45, t);
  Tilt(f.control, 30, 0.3);
  EXPECT_FALSE(f.control.Status(0.3).blocked);
  for (double t = 0.4; t < 0.89; t += 0.01) {
    f.Board(0, t);
    Tilt(f.control, 45, t);
    f.control.Tick(t);
  }
  EXPECT_FALSE(f.control.Status(0.88).blocked);
  Tilt(f.control, 45, 0.9);
  auto s = f.control.Status(0.9);
  EXPECT_EQ(s.emergency_reason, kReasonTilt | kReasonLatch);
  auto out = f.control.Tick(0.9);
  EXPECT_TRUE(HighLevelLatch(out));
  ASSERT_EQ(Of(out, kMowerService).size(), 1u);
  EXPECT_EQ(SpeedOf(Of(out, kMowerService)[0]), 0);
  EXPECT_FALSE(f.control.Status(0.9).blade_on);
  auto errors = f.control.TakeErrors();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors[0].find("tilted 45 deg"), std::string::npos);
  EXPECT_TRUE(f.control.TakeErrors().empty());
}

TEST(XcoreControl, TiltReleaseOnlyLevelAndFresh) {
  Fixture f;
  for (double t = 0; t <= 0.6; t += 0.01) Tilt(f.control, 50, t);
  ASSERT_TRUE(f.control.Status(0.6).emergency_reason & kReasonTilt);
  Tilt(f.control, 30, 0.7);
  auto refused = f.control.ReleaseEmergency(0.7);
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->code, "STILL_TILTED");
  EXPECT_NE(refused->message.find("still tilted 30 deg"), std::string::npos);
  EXPECT_NEAR(refused->value, 30, 1e-6);
  EXPECT_EQ(refused->limit, 26);
  EXPECT_TRUE(f.control.Status(0.7).emergency_reason & kReasonTilt);
  // Level but the last sample is old.
  Tilt(f.control, 5, 0.8);
  refused = f.control.ReleaseEmergency(1.5);
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->code, "TILT_UNKNOWN");
  EXPECT_NE(refused->message.find("no fresh IMU"), std::string::npos);
  EXPECT_NEAR(refused->value, 0.7, 1e-6);
  EXPECT_TRUE(std::isnan(refused->limit));
  // A refused release leaves the other latches as they were.
  f.control.LatchEmergency(kReasonFence);
  Tilt(f.control, 30, 1.6);
  EXPECT_TRUE(f.control.ReleaseEmergency(1.6));
  EXPECT_TRUE(f.control.Status(1.6).emergency_reason & kReasonFence);
  Tilt(f.control, 5, 1.7);
  EXPECT_FALSE(f.control.ReleaseEmergency(1.7));
  f.Board(0, 1.8);
  EXPECT_FALSE(f.control.Status(1.8).blocked);
}

TEST(XcoreControl, UpsideDownIsTilted) {
  Fixture f;
  for (double t = 0; t <= 0.6; t += 0.01) f.control.OnImu(0, 0, -kG, t);
  EXPECT_TRUE(f.control.Status(0.6).emergency_reason & kReasonTilt);
  // Values that are no numbers change nothing: the last sample stays the one at 0.6.
  f.control.OnImu(std::nan(""), 0, kG, 0.65);
  EXPECT_TRUE(f.control.ReleaseEmergency(0.65));
  f.control.OnImu(0, std::nan(""), kG, 1.5);
  f.control.OnImu(0, 0, std::nan(""), 1.5);
  auto refused = f.control.ReleaseEmergency(1.5);
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->code, "TILT_UNKNOWN");
}

TEST(XcoreControl, WheelTurningAfterTheStopLatches) {
  DriveRun r;
  r.Until(1.0, 0.1, 0.1, true);
  r.f.control.OnCmdVel(0, 0, 1.0);
  // Coasting in the first second after the last setpoint is allowed.
  r.Until(1.9, 0.2, 0.2);
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  r.Until(2.0, 0, 0);
  r.sent.clear();
  // Turning again for less than 0.2 s.
  r.Until(2.15, 0.2, 0.2);
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  r.Until(2.5, 0.2, 0.2);
  auto s = r.f.control.Status(r.t);
  EXPECT_EQ(s.emergency_reason, kReasonWheelRunaway | kReasonLatch);
  EXPECT_TRUE(HighLevelLatch(r.sent));
  // The board echoes what the driver sends; the status keeps the driver's own reason.
  r.f.Board(reason::kHighLevel | reason::kLatch, r.t);
  EXPECT_EQ(r.f.control.Status(r.t).emergency_reason, kReasonWheelRunaway | kReasonLatch);
  // Stops go out although the driver sent none since its last setpoint.
  int stops = 0;
  for (const auto& v : Of(r.sent, kDiffDriveService)) stops += TwistOf(v)[0] == 0;
  EXPECT_EQ(stops, 3);
  auto errors = r.f.control.TakeErrors();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors[0].find("after the last drive setpoint"), std::string::npos);
  EXPECT_FALSE(r.f.control.ReleaseEmergency(r.t));
}

TEST(XcoreControl, PushingByHandLatchesNothing) {
  DriveRun r;
  r.Until(2.0, 0.3, 0.3);
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  r.Until(3.0, 0.1, 0.1, true);
  r.f.control.OnCmdVel(0, 0, 3.0);
  r.Until(8.1, 0, 0);
  // Later than 5 s after the last setpoint, also faster than the overspeed limit.
  r.Until(10.0, 0.8, 0.8);
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  EXPECT_TRUE(r.f.control.TakeErrors().empty());
}

TEST(XcoreControl, OverspeedStopsAndLatches) {
  DriveRun r;
  // 1.3 x (0.3 + 0.3 x 0.1778) = 0.459 m/s.
  r.Until(1.0, 0.45, 0.45, true, 0.3);
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  r.Until(1.45, 0.6, 0.6, true, 0.3);
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  r.sent.clear();
  r.Until(1.6, 0.6, 0.6, true, 0.3);
  EXPECT_EQ(r.f.control.Status(r.t).emergency_reason, kReasonWheelRunaway | kReasonLatch);
  EXPECT_TRUE(HighLevelLatch(r.sent));
  auto errors = r.f.control.TakeErrors();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors[0].find("above 0.46 m/s"), std::string::npos);
  // Commands stay without effect until a release.
  r.sent.clear();
  r.Until(2.0, 0, 0, true, 0.3);
  for (const auto& v : Of(r.sent, kDiffDriveService)) EXPECT_EQ(TwistOf(v)[0], 0);
}

TEST(XcoreControl, OverspeedFollowsTheRaisedLimit) {
  ControlConfig config = Fixture::Config();
  config.max_linear_speed = MeasurementSpeedCap(0.5);
  config.regulator.max_duty = 1.0;
  DriveRun r(config);
  // 1.3 x (0.5 + 0.3 x 0.1778) = 0.719 m/s; the command of 0.5 m/s goes out as such.
  r.Until(1.0, 0.7, 0.7, true, 0.5);
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  auto drive = Of(r.sent, kDiffDriveService);
  ASSERT_FALSE(drive.empty());
  EXPECT_NEAR(TwistOf(drive.back())[0], 0.5, 1e-9);
  r.Until(1.6, 0.75, 0.75, true, 0.6);
  EXPECT_EQ(r.f.control.Status(r.t).emergency_reason, kReasonWheelRunaway | kReasonLatch);
  auto errors = r.f.control.TakeErrors();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors[0].find("above 0.72 m/s"), std::string::npos);
}

TEST(SpeedCaps, ReachableSpeedsLowerTheLimits) {
  ControlConfig config;
  config.regulator.wheel_distance = 0.3556;
  config.regulator.duty_slope = 0.58;
  config.regulator.duty_deadband = 0.019;
  config.regulator.max_duty = 0.3;
  ApplyReachableSpeeds(config);
  // A wheel reaches 0.58 x 0.281 = 0.163 m/s; turning on the spot 2 x 0.163 / 0.3556 = 0.917 rad/s, above the cap.
  EXPECT_NEAR(config.max_linear_speed, 0.16298, 1e-9);
  EXPECT_EQ(config.max_angular_speed, 0.3);
  // The raised start of M8: 0.5 m/s stays, as 0.9 reaches 0.511 m/s.
  config.max_linear_speed = 0.5;
  config.regulator.max_duty = 0.9;
  ApplyReachableSpeeds(config);
  EXPECT_EQ(config.max_linear_speed, 0.5);
  // A small max_duty lowers the turning as well; max_duty 0 lets nothing move.
  config.regulator.max_duty = 0.05;
  ApplyReachableSpeeds(config);
  EXPECT_NEAR(config.max_linear_speed, 0.58 * 0.031, 1e-12);
  EXPECT_NEAR(config.max_angular_speed, 2 * 0.58 * 0.031 / 0.3556, 1e-12);
  config.regulator.max_duty = 0;
  ApplyReachableSpeeds(config);
  EXPECT_EQ(config.max_linear_speed, 0);
  EXPECT_EQ(config.max_angular_speed, 0);
}

TEST(SpeedCaps, MeasurementLimitFromZeroToSixTenths) {
  EXPECT_EQ(MeasurementSpeedCap(0.5), 0.5);
  EXPECT_EQ(MeasurementSpeedCap(0.6), 0.6);
  EXPECT_EQ(MeasurementSpeedCap(0.0), 0.0);
  EXPECT_THROW(MeasurementSpeedCap(0.61), std::invalid_argument);
  EXPECT_THROW(MeasurementSpeedCap(-0.1), std::invalid_argument);
  EXPECT_THROW(MeasurementSpeedCap(std::nan("")), std::invalid_argument);
  EXPECT_THROW(MeasurementSpeedCap(INFINITY), std::invalid_argument);
}

TEST(SpeedCaps, LimitsStayUnderTheirCap) {
  EXPECT_EQ(CappedLimit("max_linear_speed", 0.5, kLinearSpeedCap), 0.3);
  EXPECT_EQ(CappedLimit("max_linear_speed", 0.2, kLinearSpeedCap), 0.2);
  EXPECT_EQ(CappedLimit("max_angular_speed", 0.5, kAngularSpeedCap), 0.3);
  EXPECT_EQ(CappedLimit("max_linear_speed", 0.0, kLinearSpeedCap), 0.0);
  EXPECT_EQ(CappedLimit("max_duty", 1.5, 1.0), 1.0);
  EXPECT_EQ(CappedLimit("max_linear_speed", INFINITY, kLinearSpeedCap), 0.3);
  // NaN would pass every comparison of the clamps after it, a negative value inverts them.
  EXPECT_THROW(CappedLimit("max_linear_speed", std::nan(""), kLinearSpeedCap), std::invalid_argument);
  EXPECT_THROW(CappedLimit("max_angular_speed", -0.1, kAngularSpeedCap), std::invalid_argument);
  EXPECT_THROW(CappedLimit("max_duty", std::nan(""), 1.0), std::invalid_argument);
  try {
    CappedLimit("max_duty", -1.0, 1.0);
    FAIL();
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("max_duty"), std::string::npos);
  }
}

TEST(XcoreControl, StalledThreadLatchesBeforeTheNextFrame) {
  Fixture f;
  double t = 0;
  for (; t < 1.0; t += 0.01) {
    f.Board(0, t);
    f.control.OnCmdVel(0.2, 0, t);
    f.control.SetBlade({true, 0, 0, 2.0}, t);
    f.control.Tick(t);
  }
  // A gap of 0.7 s is no stall.
  f.Board(0, 1.7);
  f.control.OnCmdVel(0.2, 0, 1.7);
  f.control.Tick(1.7);
  EXPECT_FALSE(f.control.Status(1.7).blocked);
  // 1.2 s: what arrived meanwhile comes first, then the tick.
  f.control.OnCmdVel(0.2, 0, 2.9);
  ASSERT_EQ(f.control.SetBlade({true, 0, 0, 2.0}, 2.9), BladeResult::kOk);
  auto out = f.control.Tick(2.9);
  auto emergency = Of(out, kEmergencyService);
  ASSERT_EQ(emergency.size(), 1u);
  EXPECT_EQ(EmergencyOf(emergency[0]).first, reason::kHighLevel | reason::kLatch);
  auto drive = Of(out, kDiffDriveService);
  ASSERT_EQ(drive.size(), 1u);
  EXPECT_EQ(TwistOf(drive[0]), std::vector<double>(6, 0.0));
  for (const auto& v : Of(out, kMowerService)) EXPECT_EQ(SpeedOf(v), 0);
  auto s = f.control.Status(2.9);
  EXPECT_FALSE(s.moving);
  EXPECT_TRUE(s.emergency_reason & kReasonDriverStall);
  EXPECT_FALSE(s.emergency_reason & reason::kTimeoutHighLevel);
  EXPECT_TRUE(s.latched);
  EXPECT_FALSE(s.blade_on);
  EXPECT_EQ(f.control.TakeErrors().size(), 1u);
  // The board lets go once it hears the host again; the driver holds until a release and a new command.
  f.Board(0, 3.0);
  f.control.OnCmdVel(0.2, 0, 3.0);
  for (const auto& v : f.control.Tick(3.0)) {
    if (v.service_id == kDiffDriveService) {
      EXPECT_EQ(TwistOf(v)[0], 0);
    }
    if (v.service_id == kMowerService) {
      EXPECT_EQ(SpeedOf(v), 0);
    }
  }
  EXPECT_FALSE(f.control.ReleaseEmergency(3.1));
  f.Board(0, 3.2);
  EXPECT_FALSE(f.control.Status(3.2).blocked);
}

TEST(XcoreControl, AliveWithoutLinkIsNoStall) {
  Fixture f;
  f.control.Tick(0);
  for (double t = 0.05; t < 3.0; t += 0.05) f.control.Alive(t);
  f.Board(0, 3.0);
  f.control.Tick(3.0);
  EXPECT_FALSE(f.control.Status(3.0).blocked);
}

TEST(XcoreControl, HighLevelOfTheBoardShowsWithoutAnOwnCause) {
  Fixture f;
  // Left over at the board after a release, or set by another host: it blocks and shows.
  f.Board(reason::kHighLevel | reason::kLatch, 0);
  EXPECT_EQ(f.control.Status(0).emergency_reason, kReasonHighLevel | kReasonLatch);
  // A latch from above with HIGH_LEVEL shows it as well, one with TILT only TILT.
  EXPECT_TRUE(f.control.LatchEmergency(kReasonTilt));
  EXPECT_EQ(f.control.Status(0).emergency_reason, kReasonTilt | kReasonLatch);
  EXPECT_TRUE(f.control.LatchEmergency(0));
  EXPECT_EQ(f.control.Status(0).emergency_reason, kReasonTilt | kReasonHighLevel | kReasonLatch);
}

TEST(XcoreControl, FaultHoldsUntilRelease) {
  Fixture f;
  f.control.LatchForeignHost("another host", 0);
  f.control.LatchForeignHost("again", 0.1);
  EXPECT_EQ(f.control.TakeErrors(), std::vector<std::string>{"another host"});
  EXPECT_TRUE(HighLevelLatch(f.control.Tick(0.1)));
  f.Board(0, 0.2);
  EXPECT_EQ(f.control.Status(0.2).emergency_reason, kReasonForeignHost | kReasonLatch);
  EXPECT_FALSE(f.control.ReleaseEmergency(0.2));
  EXPECT_FALSE(f.control.Status(0.2).blocked);
}

TEST(XcoreControl, WheelAgainstTheCommandLatches) {
  ControlConfig config = Fixture::Config();
  config.regulator.feedback = true;
  DriveRun r(config);
  // Forward commanded, both wheels measured backwards for more than reverse_time.
  r.Until(1.0, -0.1, -0.1, true);
  auto s = r.f.control.Status(r.t);
  EXPECT_EQ(s.emergency_reason, kReasonWheelRunaway | kReasonLatch);
  EXPECT_TRUE(HighLevelLatch(r.sent));
  auto errors = r.f.control.TakeErrors();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors[0].find("against the drive command"), std::string::npos);
  r.Until(1.2, 0, 0);
  EXPECT_TRUE(r.f.control.TakeErrors().empty());
  EXPECT_FALSE(r.f.control.ReleaseEmergency(r.t));
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  // The regulator's fault is gone as well: the wheels turn with the command again.
  r.sent.clear();
  r.Until(2.0, 0.1, 0.1, true);
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  int forward = 0;
  for (const auto& v : Of(r.sent, kDiffDriveService)) forward += TwistOf(v)[0] > 0;
  EXPECT_GT(forward, 0);
}

TEST(XcoreControl, HoldStopsWheelsAndBladeWithoutEmergency) {
  Fixture f;
  f.control.OnCmdVel(0.2, 0, 0);
  ASSERT_EQ(f.control.SetBlade({true, 0, 0, 1.0}, 0), BladeResult::kOk);
  f.control.Tick(0);
  f.control.SetHold(true);
  f.Board(0, 0.05);
  auto out = f.control.Tick(0.05);
  ASSERT_EQ(Of(out, kDiffDriveService).size(), 1u);
  EXPECT_EQ(TwistOf(Of(out, kDiffDriveService)[0])[0], 0);
  ASSERT_EQ(Of(out, kMowerService).size(), 1u);
  EXPECT_EQ(SpeedOf(Of(out, kMowerService)[0]), 0);
  // No latch for the board: the emergency input stays clear.
  for (const auto& v : Of(out, kEmergencyService)) EXPECT_EQ(EmergencyOf(v).first, 0);
  auto s = f.control.Status(0.05);
  EXPECT_TRUE(s.held);
  EXPECT_FALSE(s.blocked);
  EXPECT_FALSE(s.latched);
  EXPECT_EQ(s.emergency_reason, 0u);
  EXPECT_EQ(s.blade_off_reason, BladeOffReason::kBlocked);
  // Commands and blade requests while held do nothing.
  for (int i = 6; i < 300; i++) {
    const double t = i * 0.01;
    f.Board(0, t);
    f.control.OnCmdVel(0.2, 0, t);
    uint32_t bits = 7;
    EXPECT_EQ(f.control.SetBlade({true, 0, 0, 1.0}, t, &bits), BladeResult::kHeld);
    EXPECT_EQ(bits, 0u);
    const auto values = f.control.Tick(t);
    for (const auto& v : Of(values, kDiffDriveService)) EXPECT_EQ(TwistOf(v)[0], 0);
    for (const auto& v : Of(values, kMowerService)) EXPECT_EQ(SpeedOf(v), 0);
    for (const auto& v : Of(values, kEmergencyService)) EXPECT_EQ(EmergencyOf(v).first, 0);
  }
  EXPECT_FALSE(f.control.Status(3.0).moving);
  // The hold ends without a release.
  f.control.SetHold(false);
  f.Board(0, 3.0);
  f.control.OnCmdVel(0.2, 0, 3.0);
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 1.0}, 3.0), BladeResult::kOk);
  const auto again = f.control.Tick(3.0);
  ASSERT_EQ(Of(again, kDiffDriveService).size(), 1u);
  EXPECT_GT(TwistOf(Of(again, kDiffDriveService)[0])[0], 0);
  ASSERT_EQ(Of(again, kMowerService).size(), 1u);
  EXPECT_GT(SpeedOf(Of(again, kMowerService)[0]), 0);
}

TEST(XcoreControl, AnEmergencyDuringAHoldIsReportedAsSuch) {
  Fixture f;
  f.control.SetHold(true);
  f.Board(reason::kLatch | reason::kStop, 0);
  uint32_t bits = 0;
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 1.0}, 0, &bits), BladeResult::kBlocked);
  EXPECT_EQ(bits & (reason::kLatch | reason::kStop), reason::kLatch | reason::kStop);
  EXPECT_TRUE(f.control.Status(0).blocked);
  EXPECT_TRUE(f.control.Status(0).held);
}

TEST(XcoreControl, HoldWithoutMotionSendsNoDrive) {
  Fixture f;
  f.control.SetHold(true);
  for (int i = 0; i < 200; i++) {
    const double t = i * 0.01;
    f.Board(0, t);
    EXPECT_TRUE(Of(f.control.Tick(t), kDiffDriveService).empty());
  }
}

TEST(XcoreControl, ACommandDuringAHoldIsDroppedNotKeptForLater) {
  Fixture f;
  f.control.SetHold(true);
  f.control.OnCmdVel(0.2, 0, 0);
  f.control.SetHold(false);
  for (int i = 0; i < 100; i++) {
    const double t = i * 0.01;
    f.Board(0, t);
    EXPECT_TRUE(Of(f.control.Tick(t), kDiffDriveService).empty());
  }
}

// A driver started again after its predecessor ended (start_latched): the board is held from the first frame on, and
// commands stay without effect until a release.
TEST(XcoreControl, LatchedBeforeTheFirstFrame) {
  XcoreControl control(Fixture::Config(), Inputs());
  ASSERT_TRUE(control.LatchEmergency(0));
  control.SetServicesReady(true, true);
  auto first = Of(control.Tick(0), kEmergencyService);
  ASSERT_FALSE(first.empty());
  EXPECT_EQ(EmergencyOf(first[0]).first, reason::kHighLevel | reason::kLatch);
  control.OnOutput(kEmergencyService, emergency::kOutReason, U16(reason::kHighLevel | reason::kLatch), 0, 0.01);
  EXPECT_TRUE(control.Status(0.01).blocked);
  control.OnCmdVel(0.1, 0, 0.02);
  for (double t = 0.05; t < 1.0; t += 0.05) {
    for (const auto& v : Of(control.Tick(t), kDiffDriveService)) EXPECT_EQ(TwistOf(v)[0], 0);
  }
  EXPECT_FALSE(control.ReleaseEmergency(1.0));
  control.OnOutput(kEmergencyService, emergency::kOutReason, U16(0), 0, 1.05);
  EXPECT_FALSE(control.Status(1.05).blocked);
}

namespace {

// One sample of the wheel counters at now, after 40 ms of board time.
void Ticks(Fixture& f, Wheels& wheels, double v_left, double v_right, double now) {
  f.control.OnOutput(kDiffDriveService, diff_drive::kOutTicks, wheels.Step(v_left, v_right, 0.04), wheels.us(), now);
}

// A drive setpoint goes out at t; the command goes on or ends with it.
void DriveAt(Fixture& f, double t, bool keep) {
  f.Board(0, t);
  f.control.OnCmdVel(0.1, 0, t);
  f.control.Tick(t);
  if (!keep) f.control.OnCmdVel(0, 0, t);
}

bool Runaway(const Fixture& f, double now) {
  return (f.control.Status(now).emergency_reason & kReasonWheelRunaway) != 0;
}

}  // namespace

TEST(XcoreControl, EveryHostReasonLatchesAndNoOther) {
  for (uint32_t bit : {kReasonHighLevel, kReasonTilt, kReasonFence, kReasonStuck}) {
    Fixture f;
    EXPECT_TRUE(f.control.LatchEmergency(bit));
    EXPECT_EQ(f.control.Status(0).emergency_reason, bit | kReasonLatch);
  }
  Fixture f;
  for (uint32_t bit : {kReasonStop, kReasonLift, kReasonMowerNotStopped, kReasonForeignHost, kReasonWheelRunaway}) {
    EXPECT_FALSE(f.control.LatchEmergency(bit));
  }
  EXPECT_FALSE(f.control.Status(0).blocked);
}

TEST(XcoreControl, TheBoardsLatchedCausesStayUntilRelease) {
  // TIMEOUT_HIGH_LEVEL (6) is left out: after the board heard the driver it latches as lost messages.
  for (int bit = 1; bit <= 12; bit++) {
    if (bit == 6) continue;
    Fixture f;
    const uint16_t cause = static_cast<uint16_t>(1u << bit);
    f.Board(cause | reason::kLatch, 0);
    f.Board(reason::kLatch, 0.1);
    const bool kept = (f.control.Status(0.1).emergency_reason & cause) != 0;
    EXPECT_EQ(kept, bit == 2 || bit == 3 || bit == 4 || bit == 5 || bit == 9 || bit == 10 || bit == 11) << bit;
  }
  // A cause without the latch goes with the next report.
  Fixture f;
  f.Board(reason::kLift, 0);
  EXPECT_TRUE(f.control.Status(0).blocked);
  f.Board(0, 0.1);
  EXPECT_FALSE(f.control.Status(0.1).blocked);
}

TEST(XcoreControl, LatchesGoOutWithTheNextTick) {
  Fixture f;
  ASSERT_EQ(Of(f.control.Tick(0), kEmergencyService).size(), 1u);
  // The heartbeat is due again at 0.25 s; a latch from above does not wait for it.
  EXPECT_TRUE(f.control.LatchEmergency(kReasonFence));
  EXPECT_TRUE(HighLevelLatch(f.control.Tick(0.05)));
  // Nor does a tilt, here at 0.51 s with heartbeats at 0.25 and 0.5 s.
  Fixture g;
  for (int i = 0; i <= 51; i++) {
    const double t = i * 0.01;
    g.Board(0, t);
    if (i >= 1) Tilt(g.control, 45, t);
    const auto out = g.control.Tick(t);
    if (i == 50) {
      EXPECT_FALSE(HighLevelLatch(out));
    }
    if (i == 51) {
      EXPECT_TRUE(HighLevelLatch(out));
    }
  }
}

TEST(XcoreControl, AReleaseClearsAtOnceAndTheHeartbeatFollowsTheLastClear) {
  Fixture f;
  EXPECT_TRUE(f.control.LatchEmergency(kReasonFence));
  std::vector<std::pair<double, std::pair<uint16_t, uint16_t>>> sent;
  for (int i = 0; i <= 100; i++) {
    const double t = i * 0.01;
    f.Board(0, t);
    if (i == 30) {
      EXPECT_FALSE(f.control.ReleaseEmergency(t));
    }
    for (const auto& v : Of(f.control.Tick(t), kEmergencyService)) sent.emplace_back(t, EmergencyOf(v));
  }
  // Latched at 0 and 0.25 s, released at 0.3 s between two heartbeats: clears at 0.3, 0.4 and 0.5 s, then every 0.25 s.
  const uint16_t latch = reason::kHighLevel | reason::kLatch;
  const uint16_t clear = reason::kLatch | reason::kHighLevel | reason::kMowerRpmTimeout | reason::kMowerRpmLimit;
  const std::vector<std::pair<double, std::pair<uint16_t, uint16_t>>> want{
      {0, {latch, 0}}, {0.25, {latch, 0}}, {0.3, {0, clear}}, {0.4, {0, clear}},
      {0.5, {0, clear}}, {0.75, {0, 0}},   {1.0, {0, 0}}};
  ASSERT_EQ(sent.size(), want.size());
  for (size_t i = 0; i < want.size(); i++) {
    EXPECT_NEAR(sent[i].first, want[i].first, 1e-9) << i;
    EXPECT_EQ(sent[i].second, want[i].second) << i;
  }
}

// Each cause of the driver adds its bits to a latch from above.
TEST(XcoreControl, ReasonsAddUp) {
  {
    Fixture f;
    f.control.SetBlade({true, 0, 0, 1}, 0);
    f.control.Tick(0);
    f.control.SetBlade({false, 0, 0, 0}, 0.1);
    f.control.Tick(0.1);
    ASSERT_TRUE(f.control.LatchEmergency(kReasonFence));
    f.Board(0, 5.2);
    f.control.OnOutput(kMowerService, mower::kOutRpm, F32(3000), 0, 5.2);
    EXPECT_EQ(f.control.Status(5.2).emergency_reason, kReasonFence | kReasonMowerNotStopped | kReasonLatch);
  }
  {
    Fixture f;
    f.control.Tick(0);
    ASSERT_TRUE(f.control.LatchEmergency(kReasonFence));
    f.Board(0, 1.0);
    f.control.Tick(1.0);
    EXPECT_EQ(f.control.Status(1.0).emergency_reason, kReasonFence | kReasonDriverStall | kReasonLatch);
  }
  {
    Fixture f;
    ASSERT_TRUE(f.control.LatchEmergency(kReasonFence));
    f.Board(reason::kTimeoutHighLevel, 0.5);
    EXPECT_EQ(f.control.Status(0.5).emergency_reason, kReasonFence | reason::kTimeoutHighLevel | kReasonLatch);
  }
  {
    ControlConfig config = Fixture::Config();
    config.regulator.feedback = true;
    DriveRun r(config);
    r.Until(1.0, -0.1, -0.1, true);
    ASSERT_TRUE(r.f.control.LatchEmergency(kReasonFence));
    EXPECT_EQ(r.f.control.Status(r.t).emergency_reason, kReasonFence | kReasonWheelRunaway | kReasonLatch);
  }
  {
    Fixture f;
    ASSERT_TRUE(f.control.LatchEmergency(kReasonFence));
    f.control.LatchForeignHost("another host", 0);
    EXPECT_EQ(f.control.Status(0).emergency_reason, kReasonFence | kReasonForeignHost | kReasonLatch);
    EXPECT_EQ(f.control.Status(3.0).emergency_reason,
              kReasonFence | kReasonForeignHost | kReasonLatch | kReasonTimeoutInputs);
    f.control.SetServicesReady(true, false);
    EXPECT_EQ(f.control.Status(0).emergency_reason,
              kReasonFence | kReasonForeignHost | kReasonLatch | kReasonFirmwareIncompatible);
  }
  {
    // Two checks of the driver keep both bits.
    DriveRun r;
    r.Until(1.0, 0.1, 0.1, true);
    r.f.control.OnCmdVel(0, 0, 1.0);
    r.f.control.LatchForeignHost("another host", 1.0);
    r.Until(2.5, 0.2, 0.2);
    EXPECT_EQ(r.f.control.Status(r.t).emergency_reason, kReasonForeignHost | kReasonWheelRunaway | kReasonLatch);
  }
}

TEST(XcoreControl, AReleaseStartsTheTimesOfTheChecksAgain) {
  {
    // Tilted for 0.3 s when another latch is released: the tilt counts from the next sample.
    Fixture f;
    ASSERT_TRUE(f.control.LatchEmergency(kReasonFence));
    for (int i = 0; i <= 30; i++) Tilt(f.control, 45, i * 0.01);
    EXPECT_FALSE(f.control.ReleaseEmergency(0.3));
    for (int i = 31; i <= 70; i++) Tilt(f.control, 45, i * 0.01);
    EXPECT_FALSE(f.control.Status(0.7).emergency_reason & kReasonTilt);
    for (int i = 71; i <= 85; i++) Tilt(f.control, 45, i * 0.01);
    EXPECT_TRUE(f.control.Status(0.85).emergency_reason & kReasonTilt);
  }
  {
    // A wheel turning on without a command after its latch was released.
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0, false);
    for (int i = 0; i <= 6; i++) Ticks(f, wheels, 0.2, 0.2, 1.0 + 0.04 * i);
    ASSERT_TRUE(Runaway(f, 1.24));
    EXPECT_FALSE(f.control.ReleaseEmergency(1.25));
    for (int i = 7; i <= 10; i++) Ticks(f, wheels, 0.2, 0.2, 1.0 + 0.04 * i);
    EXPECT_FALSE(Runaway(f, 1.4));
  }
  {
    // A wheel above the overspeed limit while commanded.
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0, true);
    for (int i = 0; i <= 14; i++) Ticks(f, wheels, 0.6, 0.6, 0.04 * i);
    ASSERT_TRUE(Runaway(f, 0.56));
    EXPECT_FALSE(f.control.ReleaseEmergency(0.57));
    for (int i = 15; i <= 25; i++) Ticks(f, wheels, 0.6, 0.6, 0.04 * i);
    EXPECT_FALSE(Runaway(f, 1.0));
  }
}

TEST(XcoreControl, UncommandedTurningCountsAtItsLimits) {
  {
    // 1.13 - 0.13 lies just under 1 s by rounding: the second after the setpoint is over.
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0.13, false);
    for (double t : {1.09, 1.13, 1.17, 1.21, 1.25, 1.29, 1.33}) Ticks(f, wheels, 0.2, 0.2, t);
    EXPECT_TRUE(Runaway(f, 1.33));
  }
  {
    // Exactly 1 s after the setpoint.
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0.5, false);
    for (double t : {1.46, 1.5, 1.54, 1.58, 1.62, 1.66, 1.7}) Ticks(f, wheels, 0.2, 0.2, t);
    EXPECT_TRUE(Runaway(f, 1.7));
  }
  {
    // 1.2 - 1.0 lies just under 0.2 s: enough.
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0, false);
    for (double t : {0.96, 1.0, 1.2}) Ticks(f, wheels, 0.2, 0.2, t);
    EXPECT_TRUE(Runaway(f, 1.2));
  }
  {
    // Exactly uncommanded_time, here 1 s.
    ControlConfig config = Fixture::Config();
    config.uncommanded_time_s = 1.0;
    Fixture f(config);
    Wheels wheels;
    DriveAt(f, 0, false);
    for (double t : {1.46, 1.5, 2.5}) Ticks(f, wheels, 0.2, 0.2, t);
    EXPECT_TRUE(Runaway(f, 2.5));
  }
  {
    // An interrupted run counts again.
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0, false);
    for (double t : {1.0, 1.04, 1.08, 1.12, 1.16}) Ticks(f, wheels, 0.2, 0.2, t);
    Ticks(f, wheels, 0, 0, 1.2);
    for (double t : {1.24, 1.28, 1.32, 1.36, 1.4}) Ticks(f, wheels, 0.2, 0.2, t);
    EXPECT_FALSE(Runaway(f, 1.4));
  }
  {
    // Exactly uncommanded_speed is not above it: 81 ticks in 0.4 s are 0.1 m/s.
    Fixture f;
    DriveAt(f, 0, false);
    for (uint32_t i = 0; i <= 5; i++) {
      Writer w;
      w.U32(81 * i);
      w.U32(-81 * i);
      f.control.OnOutput(kDiffDriveService, diff_drive::kOutTicks, w.Take(), 400000 * i, 1.0 + 0.4 * i);
    }
    EXPECT_FALSE(Runaway(f, 3.0));
  }
}

TEST(XcoreControl, OverspeedCountsAtItsLimits) {
  // Exactly 0.5 s, and 0.57 - 0.07 just under it by rounding.
  for (const auto& [since, latch] : std::vector<std::pair<double, double>>{{0.01, 0.51}, {0.07, 0.57}}) {
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0, true);
    for (double t : {since - 0.005, since, latch}) Ticks(f, wheels, 0.6, 0.6, t);
    EXPECT_TRUE(Runaway(f, latch)) << since;
  }
  {
    // One wheel alone.
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0, true);
    for (int i = 0; i <= 14; i++) Ticks(f, wheels, 0.6, 0, 0.04 * i);
    EXPECT_TRUE(Runaway(f, 0.56));
  }
  {
    // An interrupted run counts again.
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0, true);
    for (int i = 0; i <= 8; i++) Ticks(f, wheels, 0.6, 0.6, 0.04 * i);
    Ticks(f, wheels, 0.3, 0.3, 0.36);
    for (int i = 10; i <= 22; i++) Ticks(f, wheels, 0.6, 0.6, 0.04 * i);
    EXPECT_FALSE(Runaway(f, 0.88));
  }
}

TEST(XcoreControl, TheWatchEndsFiveSecondsAfterTheLastSetpoint) {
  // Exactly 5 s after the setpoint, and 8.05 - 3.05 just over 5 s by rounding: still watched.
  auto latches_at = [](double setpoint, double since, double last) {
    Fixture f;
    Wheels wheels;
    DriveAt(f, setpoint, false);
    for (double t : {since - 0.04, since, last}) Ticks(f, wheels, 0.2, 0.2, t);
    return Runaway(f, last);
  };
  EXPECT_TRUE(latches_at(0.01, 4.81, 5.01));
  EXPECT_TRUE(latches_at(3.05, 7.85, 8.05));
}

TEST(XcoreControl, ANewWatchCountsAfresh) {
  {
    // Above the overspeed limit since 4.8 s while commanded, on after the watch ended, then a new setpoint at 7 s:
    // the time counts from the first sample after it.
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0, true);
    for (int i = 0; i <= 56; i++) Ticks(f, wheels, 0.6, 0.6, 4.76 + 0.04 * i);
    f.control.Alive(6.99);
    DriveAt(f, 7.0, true);
    for (int i = 1; i <= 12; i++) Ticks(f, wheels, 0.6, 0.6, 7.0 + 0.04 * i);
    EXPECT_FALSE(Runaway(f, 7.48));
    Ticks(f, wheels, 0.6, 0.6, 7.52);
    EXPECT_FALSE(Runaway(f, 7.52));
  }
  {
    // A wheel turning without a command when the watch ends, a setpoint at 7 s and no samples for a second after it.
    Fixture f;
    Wheels wheels;
    DriveAt(f, 0, false);
    for (double t : {4.86, 4.9, 5.02}) Ticks(f, wheels, 0.2, 0.2, t);
    f.control.Alive(6.99);
    DriveAt(f, 7.0, false);
    for (double t : {8.0, 8.1}) Ticks(f, wheels, 0.2, 0.2, t);
    EXPECT_FALSE(Runaway(f, 8.1));
  }
}

TEST(XcoreControl, TurningOnTheSpotDrivesAndIsWatched) {
  DriveRun r;
  r.Until(1.0, -0.05, 0.05, true, 0, 0.3);
  auto drive = Of(r.sent, kDiffDriveService);
  ASSERT_FALSE(drive.empty());
  EXPECT_EQ(TwistOf(drive[0])[0], 0);
  EXPECT_GT(TwistOf(drive[0])[5], 0);
  r.f.control.OnCmdVel(0, 0, 1.0);
  r.Until(2.0, 0, 0);
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  r.Until(2.5, 0.2, 0.2);
  EXPECT_EQ(r.f.control.Status(r.t).emergency_reason, kReasonWheelRunaway | kReasonLatch);
}

TEST(XcoreControl, SetpointsOfZeroDutyStartNoWatch) {
  ControlConfig config = Fixture::Config();
  config.regulator.max_duty = 0;
  DriveRun r(config);
  r.Until(1.0, 0, 0, true);
  r.f.control.OnCmdVel(0, 0, 1.0);
  r.Until(3.0, 0.2, 0.2);
  EXPECT_FALSE(r.f.control.Status(r.t).blocked);
}

TEST(XcoreControl, ZeroCommandsGiveThreeStopsThenSilence) {
  Fixture f;
  f.control.OnCmdVel(0.2, 0, 0);
  f.control.Tick(0);
  int stops = 0, drives = 0;
  for (int i = 1; i <= 200; i++) {
    const double t = i * 0.01;
    f.Board(0, t);
    if (i % 10 == 0) f.control.OnCmdVel(0, 0, t);
    for (const auto& v : Of(f.control.Tick(t), kDiffDriveService)) (TwistOf(v)[0] == 0 ? stops : drives)++;
  }
  EXPECT_EQ(drives, 0);
  EXPECT_EQ(stops, 3);
  EXPECT_FALSE(f.control.Status(2.0).moving);
}

TEST(XcoreControl, AChangedSetpointGoesOutAtOnceTheSameOneEveryPeriod) {
  Fixture f;
  std::vector<double> at;
  for (const auto& [t, linear, angular] : std::vector<std::tuple<double, double, double>>{
           {0, 0.2, 0.1}, {0.03, 0.2, 0.1}, {0.05, 0.1, 0.1}, {0.07, 0.1, 0.2}, {0.09, 0.1, 0.2},
           {0.15, 0.1, 0.2}, {0.17, 0.1, 0.2}}) {
    f.control.OnCmdVel(linear, angular, t);
    if (!Of(f.control.Tick(t), kDiffDriveService).empty()) at.push_back(t);
  }
  EXPECT_EQ(at, (std::vector<double>{0, 0.05, 0.07, 0.17}));
}

TEST(XcoreControl, CommandsThatAreNoNumberStopOnce) {
  Fixture f;
  // An infinite speed would otherwise pass the clamp at the limit.
  f.control.OnCmdVel(INFINITY, 0, 0);
  EXPECT_TRUE(Of(f.control.Tick(0), kDiffDriveService).empty());
  f.control.OnCmdVel(0.2, 0, 0.01);
  f.control.Tick(0.01);
  int stops = 0;
  for (int i = 2; i <= 200; i++) {
    const double t = i * 0.01;
    f.Board(0, t);
    f.control.OnCmdVel(std::nan(""), std::nan(""), t);
    for (const auto& v : Of(f.control.Tick(t), kDiffDriveService)) {
      EXPECT_EQ(TwistOf(v), std::vector<double>(6, 0.0));
      stops++;
    }
  }
  EXPECT_EQ(stops, 3);
}

TEST(XcoreControl, CommandsDuringAnEmergencyKeepItsStops) {
  Fixture f;
  f.control.OnCmdVel(0.2, 0, 0);
  f.control.Tick(0);
  int stops = 0;
  for (int i = 1; i <= 100; i++) {
    const double t = i * 0.01;
    f.Board(reason::kLatch | reason::kStop, t);
    f.control.OnCmdVel(0.2, 0, t);
    for (const auto& v : Of(f.control.Tick(t), kDiffDriveService)) {
      EXPECT_EQ(TwistOf(v)[0], 0);
      stops++;
    }
  }
  EXPECT_EQ(stops, 3);
}

TEST(XcoreControl, ACommandWithdrawnBeforeItWentOutSendsNothing) {
  Fixture f;
  f.control.OnCmdVel(0.2, 0, 0);
  f.control.OnCmdVel(0, 0, 0.001);
  for (int i = 0; i < 100; i++) {
    f.Board(0, i * 0.01);
    EXPECT_TRUE(Of(f.control.Tick(i * 0.01), kDiffDriveService).empty());
  }
}

TEST(XcoreControl, AStopKeepsItsRepeatsWhenTheNextCommandEndsBeforeItWentOut) {
  for (const bool hold : {false, true}) {
    SCOPED_TRACE(hold ? "hold" : "zero");
    Fixture f;
    f.control.OnCmdVel(0.2, 0, 0);
    f.control.Tick(0);
    f.control.OnCmdVel(0, 0, 0.005);
    int stops = 0, drives = 0;
    for (int i = 1; i <= 100; i++) {
      const double t = i * 0.01;
      f.Board(0, t);
      for (const auto& v : Of(f.control.Tick(t), kDiffDriveService)) (TwistOf(v)[0] == 0 ? stops : drives)++;
      if (i == 1) {
        // A new command after the first stop that ends before the next tick.
        f.control.OnCmdVel(0.2, 0, t + 0.002);
        if (hold) {
          f.control.SetHold(true);
        } else {
          f.control.OnCmdVel(0, 0, t + 0.005);
        }
      }
    }
    EXPECT_EQ(drives, 0);
    EXPECT_EQ(stops, 3);
  }
}

TEST(XcoreControl, TimesOfTheDriveAtTheirLimits) {
  {
    // The command of 0.57 s is 0.5000000000000001 s old at 1.07 s: within the timeout.
    Fixture f;
    f.control.OnCmdVel(0.2, 0, 0.57);
    f.control.Tick(0.57);
    auto drive = Of(f.control.Tick(1.07), kDiffDriveService);
    ASSERT_EQ(drive.size(), 1u);
    EXPECT_GT(TwistOf(drive[0])[0], 0);
  }
  {
    // A stop at 0.2 s is due again at 0.2 + 0.1 = 0.30000000000000004 s: the tick at 0.3 s sends it.
    Fixture f;
    f.control.OnCmdVel(0.2, 0, 0.1);
    f.control.Tick(0.1);
    f.control.OnCmdVel(0, 0, 0.2);
    ASSERT_EQ(Of(f.control.Tick(0.2), kDiffDriveService).size(), 1u);
    EXPECT_EQ(Of(f.control.Tick(0.3), kDiffDriveService).size(), 1u);
  }
}

TEST(XcoreControl, RestartOfTheDriveServiceStopsAndForgets) {
  Fixture f;
  Wheels wheels;
  f.control.OnCmdVel(0.2, 0, 0);
  f.control.Tick(0);
  Ticks(f, wheels, 0.2, 0.2, 0);
  f.control.OnServiceRestart(kDiffDriveService, 0.05);
  EXPECT_FALSE(f.control.Status(0.05).moving);
  // Three stops and no setpoint until a new command; the counters start again.
  int stops = 0;
  for (int i = 5; i <= 60; i++) {
    const double t = i * 0.01;
    f.Board(0, t);
    for (const auto& v : Of(f.control.Tick(t), kDiffDriveService)) {
      EXPECT_EQ(TwistOf(v), std::vector<double>(6, 0.0));
      stops++;
    }
  }
  EXPECT_EQ(stops, 3);
  EXPECT_FALSE(f.control.OnOutput(kDiffDriveService, diff_drive::kOutTicks, wheels.Step(0.2, 0.2, 0.04), wheels.us(),
                                  0.61));
}

// The wheels measured against the command for 0.3 s, then the command goes on after a restart of the drive service or
// after a hold: the time against counts again.
TEST(XcoreControl, TheTimeAgainstTheCommandStartsAgainAfterARestartOrAHold) {
  {
    DriveRun r;
    r.Until(0.3, -0.1, -0.1, true);
    r.f.control.OnServiceRestart(kDiffDriveService, r.t);
    r.Until(0.7, -0.1, -0.1, true);
    EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  }
  {
    DriveRun r;
    r.Until(0.3, -0.1, -0.1, true);
    r.f.control.SetHold(true);
    r.Until(0.35, -0.1, -0.1, true);
    r.f.control.SetHold(false);
    r.Until(0.7, -0.1, -0.1, true);
    EXPECT_FALSE(r.f.control.Status(r.t).blocked);
  }
}

TEST(XcoreControl, TheTimeAgainstTheCommandGoesOnOverAStop) {
  // A command that flickers through zero does not start the time again.
  DriveRun r;
  r.Until(0.3, -0.1, -0.1, true);
  r.f.control.OnCmdVel(0, 0, r.t);
  r.Until(0.35, -0.1, -0.1);
  r.Until(0.7, -0.1, -0.1, true);
  EXPECT_TRUE(Runaway(r.f, r.t));
}

TEST(XcoreControl, TheRegulatorsFaultLatchesInTheTickThatFindsIt) {
  // A release after every tick never comes between the fault and its latch.
  DriveRun r;
  int latched = 0;
  while (r.t < 1.0 - 1e-9) {
    r.Until(r.t + 0.01, -0.1, -0.1, true);
    if (Runaway(r.f, r.t)) latched++;
    EXPECT_FALSE(r.f.control.ReleaseEmergency(r.t));
  }
  EXPECT_EQ(latched, 1);
  auto errors = r.f.control.TakeErrors();
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_NE(errors[0].find("against the drive command"), std::string::npos);
}

TEST(XcoreControl, StallAtItsLimits) {
  // A gap of exactly 0.8 s, and 1.37 - 0.57 just over it by rounding: no stall yet.
  for (const auto& [t0, t1] : std::vector<std::pair<double, double>>{{0.01, 0.81}, {0.57, 1.37}}) {
    Fixture f;
    f.control.Tick(t0);
    f.Board(0, t1);
    f.control.Tick(t1);
    EXPECT_FALSE(f.control.Status(t1).emergency_reason & kReasonDriverStall) << t0;
  }
  {
    // A setpoint that did not go out yet gets its stop as well.
    Fixture f;
    f.control.Tick(0);
    f.Board(0, 1.0);
    f.control.OnCmdVel(0.2, 0, 1.0);
    auto drive = Of(f.control.Tick(1.0), kDiffDriveService);
    ASSERT_EQ(drive.size(), 1u);
    EXPECT_EQ(TwistOf(drive[0]), std::vector<double>(6, 0.0));
  }
  {
    // The stall says why the blade went off, also when its lease ran out meanwhile.
    Fixture f;
    ASSERT_EQ(f.control.SetBlade({true, 0, 0, 0.5}, 0), BladeResult::kOk);
    f.control.Tick(0);
    f.Board(0, 1.0);
    f.control.Tick(1.0);
    EXPECT_EQ(f.control.Status(1.0).blade_off_reason, BladeOffReason::kBlocked);
  }
}

TEST(XcoreControl, TiltAtItsLimits) {
  {
    // 0.57 - 0.07 lies just under 0.5 s by rounding: enough.
    Fixture f;
    Tilt(f.control, 45, 0.07);
    Tilt(f.control, 45, 0.57);
    EXPECT_TRUE(f.control.Status(0.57).emergency_reason & kReasonTilt);
  }
  // A release takes a sample exactly 0.5 s old, and 1.07 - 0.57 just over it by rounding.
  for (const auto& [level, release] : std::vector<std::pair<double, double>>{{0.51, 1.01}, {0.57, 1.07}}) {
    Fixture f;
    Tilt(f.control, 50, 0);
    Tilt(f.control, 50, 0.5);
    ASSERT_TRUE(f.control.Status(0.5).emergency_reason & kReasonTilt);
    Tilt(f.control, 0, level);
    EXPECT_FALSE(f.control.ReleaseEmergency(release)) << level;
  }
  {
    // 90 deg exactly, on its side: not above a limit of 90 deg, and not below a release limit of 90 deg.
    ControlConfig config = Fixture::Config();
    config.tilt_limit_deg = 90;
    config.tilt_release_deg = 90;
    Fixture f(config);
    for (int i = 0; i <= 100; i++) f.control.OnImu(0, kG, 0, i * 0.01);
    EXPECT_FALSE(f.control.Status(1.0).emergency_reason & kReasonTilt);
    for (int i = 101; i <= 160; i++) f.control.OnImu(0, 0, -kG, i * 0.01);
    ASSERT_TRUE(f.control.Status(1.6).emergency_reason & kReasonTilt);
    f.control.OnImu(0, kG, 0, 1.61);
    auto refused = f.control.ReleaseEmergency(1.61);
    ASSERT_TRUE(refused);
    EXPECT_EQ(refused->code, "STILL_TILTED");
    f.control.OnImu(0, 0, kG, 1.62);
    EXPECT_FALSE(f.control.ReleaseEmergency(1.62));
  }
}

TEST(XcoreControl, BladeRequestsAtTheirLimits) {
  Fixture f;
  EXPECT_EQ(f.control.SetBlade({true, 100, 0, 1}, 0), BladeResult::kOk);
  // A lease a rounding error above blade_lease_max passes, a longer one does not.
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 2.0000005}, 0), BladeResult::kOk);
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 2.00001}, 0), BladeResult::kInvalidLease);
  // A speed above 100 % in the configuration runs the blade at full speed.
  ControlConfig config = Fixture::Config();
  config.blade_speed_percent = 150;
  Fixture g(config);
  ASSERT_EQ(g.control.SetBlade({true, 0, 0, 1}, 0), BladeResult::kOk);
  auto out = Of(g.control.Tick(0), kMowerService);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(SpeedOf(out[0]), 1.0f);
}

TEST(XcoreControl, ARenewalWaitsForThePeriodANewSpeedGoesOutAtOnce) {
  Fixture f;
  ASSERT_EQ(f.control.SetBlade({true, 50, 0, 1}, 0), BladeResult::kOk);
  ASSERT_EQ(Of(f.control.Tick(0), kMowerService).size(), 1u);
  ASSERT_EQ(f.control.SetBlade({true, 50, 0, 1}, 0.3), BladeResult::kOk);
  EXPECT_TRUE(Of(f.control.Tick(0.3), kMowerService).empty());
  ASSERT_EQ(f.control.SetBlade({true, 80, 0, 1}, 0.5), BladeResult::kOk);
  auto out = Of(f.control.Tick(0.5), kMowerService);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(SpeedOf(out[0]), 0.8f);
}

TEST(XcoreControl, ARefusalEndsTheLeaseAtOnce) {
  {
    // The stop is pressed and let go again before the next tick.
    Fixture f;
    ASSERT_EQ(f.control.SetBlade({true, 0, 0, 2.0}, 0), BladeResult::kOk);
    f.control.Tick(0);
    f.Board(reason::kStop, 0.1);
    EXPECT_EQ(f.control.SetBlade({true, 0, 0, 2.0}, 0.1), BladeResult::kBlocked);
    EXPECT_FALSE(f.control.Status(0.1).blade_on);
    f.Board(0, 0.15);
    auto out = Of(f.control.Tick(0.15), kMowerService);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(SpeedOf(out[0]), 0);
    EXPECT_EQ(f.control.Status(0.15).blade_off_reason, BladeOffReason::kBlocked);
  }
  {
    // On the dock; the contact is gone again before the next tick.
    Fixture f;
    ASSERT_EQ(f.control.SetBlade({true, 0, 0, 2.0}, 0), BladeResult::kOk);
    f.control.Tick(0);
    f.control.OnOutput(kPowerService, power::kOutChargeVoltage, F32(29.0f), 0, 0.05);
    f.control.OnOutput(kPowerService, power::kOutChargeVoltage, F32(29.0f), 0, 0.1);
    EXPECT_EQ(f.control.SetBlade({true, 0, 0, 2.0}, 0.1), BladeResult::kDocked);
    EXPECT_FALSE(f.control.Status(0.1).blade_on);
    f.control.OnOutput(kPowerService, power::kOutChargeVoltage, F32(0.5f), 0, 0.12);
    f.control.Tick(0.15);
    EXPECT_FALSE(f.control.Status(0.15).blade_on);
  }
  {
    // The contact while the blade runs: off with the next tick.
    Fixture f;
    ASSERT_EQ(f.control.SetBlade({true, 0, 0, 2.0}, 0), BladeResult::kOk);
    f.control.Tick(0);
    f.control.OnOutput(kPowerService, power::kOutChargeVoltage, F32(29.0f), 0, 0.05);
    f.control.OnOutput(kPowerService, power::kOutChargeVoltage, F32(29.0f), 0, 0.1);
    auto out = Of(f.control.Tick(0.1), kMowerService);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(SpeedOf(out[0]), 0);
    EXPECT_EQ(f.control.Status(0.1).blade_off_reason, BladeOffReason::kDock);
  }
}

TEST(XcoreControl, TimesOfTheBladeAtTheirLimits) {
  {
    // A lease of 0.1 s from 0.2 s ends at 0.30000000000000004 s: the tick at 0.3 s ends it.
    Fixture f;
    ASSERT_EQ(f.control.SetBlade({true, 0, 0, 0.1}, 0.2), BladeResult::kOk);
    f.control.Tick(0.2);
    f.control.Tick(0.3);
    EXPECT_FALSE(f.control.Status(0.3).blade_on);
  }
  {
    // An off at 0.2 s is due again at 0.30000000000000004 s: the tick at 0.3 s sends it.
    Fixture f;
    ASSERT_EQ(f.control.SetBlade({true, 0, 0, 1}, 0.1), BladeResult::kOk);
    f.control.Tick(0.1);
    f.control.SetBlade({false, 0, 0, 0}, 0.2);
    ASSERT_EQ(Of(f.control.Tick(0.2), kMowerService).size(), 1u);
    EXPECT_EQ(Of(f.control.Tick(0.3), kMowerService).size(), 1u);
  }
}

TEST(XcoreControl, ABladeTurningFromTheStartLatches) {
  Fixture f;
  for (int i = 0; i <= 500; i++) {
    const double t = i * 0.01;
    f.Board(0, t);
    f.control.Tick(t);
    if (i % 10 == 0) f.control.OnOutput(kMowerService, mower::kOutRpm, F32(1500), 0, t);
    if (i == 490) {
      EXPECT_FALSE(f.control.Status(t).blocked);
    }
  }
  // The first off went out at 0 s, five seconds before.
  EXPECT_TRUE(f.control.Status(5.0).emergency_reason & kReasonMowerNotStopped);
}

TEST(XcoreControl, TheStopTimeOfTheBladeAtItsLimits) {
  // The first off at 0.01 s and a report exactly 5 s later; at 3.04 s and one 4.999999999999999 s later by rounding.
  // blade_still_rpm itself counts as turning.
  for (const auto& [off, report] : std::vector<std::pair<double, double>>{{0.01, 5.01}, {3.04, 8.04}}) {
    Fixture f;
    f.Board(0, off);
    f.control.Tick(off);
    f.Board(0, report);
    f.control.OnOutput(kMowerService, mower::kOutRpm, F32(1500), 0, (off + report) / 2);
    EXPECT_FALSE(f.control.Status(report).emergency_reason & kReasonMowerNotStopped) << off;
    f.control.OnOutput(kMowerService, mower::kOutRpm, F32(100), 0, report);
    EXPECT_TRUE(f.control.Status(report).emergency_reason & kReasonMowerNotStopped) << off;
  }
}

TEST(XcoreControl, OffRequestsDoNotRestartTheStopTime) {
  Fixture f;
  ASSERT_EQ(f.control.SetBlade({true, 0, 0, 1}, 0), BladeResult::kOk);
  f.control.Tick(0);
  f.control.SetBlade({false, 0, 0, 0}, 0.1);
  for (int i = 10; i <= 520; i++) {
    const double t = i * 0.01;
    f.Board(0, t);
    if (i % 50 == 0) f.control.SetBlade({false, 0, 0, 0}, t);
    f.control.Tick(t);
    if (i % 10 == 0) f.control.OnOutput(kMowerService, mower::kOutRpm, F32(1500), 0, t);
  }
  EXPECT_TRUE(f.control.Status(5.2).emergency_reason & kReasonMowerNotStopped);
  EXPECT_EQ(f.control.Status(5.2).blade_off_reason, BladeOffReason::kRequest);
}

TEST(XcoreControl, ABladeStillTurningGetsItsOffsAtOnce) {
  Fixture f;
  ASSERT_EQ(f.control.SetBlade({true, 0, 0, 1}, 0), BladeResult::kOk);
  f.control.Tick(0);
  f.control.SetBlade({false, 0, 0, 0}, 0.1);
  std::vector<double> offs;
  for (int i = 10; i <= 540; i++) {
    const double t = i * 0.01;
    f.Board(0, t);
    if (i == 515) f.control.OnOutput(kMowerService, mower::kOutRpm, F32(1500), 0, t);
    if (!Of(f.control.Tick(t), kMowerService).empty()) offs.push_back(t);
  }
  // Three quick offs, then one every second, and three at once when the blade still turns 5 s after the first.
  const std::vector<double> want{0.1, 0.2, 0.3, 1.3, 2.3, 3.3, 4.3, 5.15, 5.25, 5.35};
  ASSERT_EQ(offs.size(), want.size());
  for (size_t i = 0; i < want.size(); i++) EXPECT_NEAR(offs[i], want[i], 1e-9) << i;
}

TEST(XcoreControl, RestartOfTheMowerServiceSendsOffsAlsoWithTheBladeOff) {
  Fixture f;
  ASSERT_EQ(Of(f.control.Tick(0), kMowerService).size(), 1u);
  f.control.OnServiceRestart(kMowerService, 0.5);
  size_t offs = 0;
  for (int i = 50; i <= 90; i++) offs += Of(f.control.Tick(i * 0.01), kMowerService).size();
  EXPECT_EQ(offs, 3u);
}

TEST(XcoreControl, StatusShowsBladeAndMotors) {
  Fixture f;
  ASSERT_EQ(f.control.SetBlade({true, 0, 0, 1}, 0), BladeResult::kOk);
  EXPECT_TRUE(f.control.Status(0).blade_on);
  f.control.OnOutput(kMowerService, mower::kOutRpm, F32(2500), 0, 0);
  f.control.OnOutput(kDiffDriveService, diff_drive::kOutLeftCurrent, F32(1.5f), 0, 0);
  f.control.OnOutput(kDiffDriveService, diff_drive::kOutRightCurrent, F32(2.5f), 0, 0);
  auto s = f.control.Status(0);
  EXPECT_FLOAT_EQ(s.blade_motor.rpm, 2500);
  EXPECT_FLOAT_EQ(s.left_motor.current, 1.5f);
  EXPECT_FLOAT_EQ(s.right_motor.current, 2.5f);
}

TEST(XcoreControl, InputsWhileUnknownAndAtTheirIndices) {
  Fixture f;
  auto none = [](const ControlStatus& s) { return !s.stop && !s.lift && !s.bumper_left && !s.bumper_right; };
  EXPECT_TRUE(none(f.control.Status(0)));
  f.control.OnOutput(kInputService, input::kOutActive, U64(1u << 0), 0, 0);
  EXPECT_TRUE(f.control.Status(0).stop);
  f.control.OnOutput(kInputService, input::kOutActive, U64((1u << 2) | (1u << 5)), 0, 0);
  auto s = f.control.Status(0);
  EXPECT_TRUE(s.lift);
  EXPECT_TRUE(s.bumper_left);
  EXPECT_FALSE(s.stop);
  EXPECT_FALSE(s.bumper_right);
  // Another channel of the service changes nothing; a restart makes all unknown again.
  f.control.OnOutput(kInputService, 7, U64(~uint64_t{0}), 0, 0);
  EXPECT_FALSE(f.control.Status(0).stop);
  f.control.OnOutput(kInputService, input::kOutActive, U64(~uint64_t{0}), 0, 0);
  f.control.OnServiceRestart(kInputService, 0.1);
  EXPECT_TRUE(none(f.control.Status(0.1)));
  // Indices outside the mask are no input.
  InputMap map = Inputs();
  map.stop = {-1, 64, 100};
  XcoreControl control(Fixture::Config(), map);
  control.OnOutput(kInputService, input::kOutActive, U64(~uint64_t{0}), 0, 0);
  EXPECT_FALSE(control.Status(0).stop);
}

TEST(XcoreControl, TelemetryOfTheEmergencyService) {
  Fixture f;
  EXPECT_TRUE(f.control.Status(0).telemetry_ok);
  // A report 2.5 s old is still fresh.
  EXPECT_FALSE(f.control.Status(2.5).blocked);
  EXPECT_FALSE(f.control.Status(2.6).telemetry_ok);
  // Another channel of the service is no reason.
  f.control.OnOutput(kEmergencyService, 1, U16(reason::kLatch | reason::kStop), 0, 0);
  EXPECT_FALSE(f.control.Status(0).blocked);
  // A restart: unknown at once, the heartbeat goes out with the next tick.
  ASSERT_EQ(Of(f.control.Tick(0), kEmergencyService).size(), 1u);
  f.control.OnServiceRestart(kEmergencyService, 0.1);
  EXPECT_TRUE(f.control.Status(0.1).emergency_reason & kReasonTimeoutInputs);
  EXPECT_EQ(Of(f.control.Tick(0.1), kEmergencyService).size(), 1u);
}

TEST(XcoreControl, DockContactAtItsLimits) {
  Fixture f;
  auto charge = [&f](float volts, double t) {
    f.control.OnOutput(kPowerService, power::kOutChargeVoltage, F32(volts), 0, t);
  };
  // Exactly dock_voltage counts, after two reports.
  charge(5.0f, 0);
  charge(5.0f, 1);
  EXPECT_TRUE(f.control.Status(1).dock_contact);
  // After reports without contact, one with it is not enough.
  for (int i = 2; i <= 4; i++) charge(0.5f, i);
  charge(29.0f, 5);
  EXPECT_TRUE(f.control.Status(5).dock_contact_raw);
  EXPECT_FALSE(f.control.Status(5).dock_contact);
  // A restart of the power service forgets the contact.
  charge(29.0f, 6);
  ASSERT_TRUE(f.control.Status(6).dock_contact);
  f.control.OnServiceRestart(kPowerService, 6.5);
  EXPECT_FALSE(f.control.Status(6.5).dock_contact_raw);
  EXPECT_FALSE(f.control.Status(6.5).dock_contact);
}

namespace {

ControlConfig SilenceConfig() {
  ControlConfig c = Fixture::Config();
  c.silence_max_s = 3.0;
  return c;
}

}  // namespace

TEST(XcoreControl, SilenceIsRefusedOutOfRangeOnTheDockWithTheBladeAndTwice) {
  Fixture none;
  EXPECT_EQ(none.control.Silence(1.0, 0), "OUT_OF_RANGE");
  Fixture f(SilenceConfig());
  EXPECT_EQ(f.control.Silence(0.1, 0), "OUT_OF_RANGE");
  EXPECT_EQ(f.control.Silence(3.5, 0), "OUT_OF_RANGE");
  EXPECT_EQ(f.control.Silence(std::nan(""), 0), "OUT_OF_RANGE");
  ASSERT_EQ(f.control.SetBlade({true, 0, 0, 1}, 0), BladeResult::kOk);
  EXPECT_EQ(f.control.Silence(1.0, 0), "BLADE");
  EXPECT_EQ(f.control.EndRefusal(), "BLADE");
  ASSERT_EQ(f.control.SetBlade({false, 0, 0, 0}, 0), BladeResult::kOk);
  EXPECT_EQ(f.control.EndRefusal(), "");
  EXPECT_EQ(f.control.Silence(1.0, 0), "");
  EXPECT_EQ(f.control.Silence(1.0, 0.5), "BUSY");
  f.control.OnOutput(kPowerService, power::kOutChargeVoltage, F32(29.0f), 0, 2);
  EXPECT_EQ(f.control.EndRefusal(), "CHARGING");
  EXPECT_EQ(f.control.Silence(1.0, 2), "CHARGING");
}

TEST(XcoreControl, SilenceSendsNothingThenLatchesAndStops) {
  Fixture f(SilenceConfig());
  f.control.OnCmdVel(0.1, 0, 0);
  ASSERT_EQ(Of(f.control.Tick(0), kDiffDriveService).size(), 1u);
  ASSERT_EQ(f.control.Silence(1.0, 0.05), "");
  EXPECT_TRUE(f.control.Silent(0.5));
  // Nothing goes out, a command is dropped, blade and release are refused.
  for (double t = 0.06; t < 1.04; t += 0.01) {
    f.Board(0, t);
    f.control.OnCmdVel(0.1, 0, t);
    EXPECT_TRUE(f.control.Tick(t).empty()) << t;
  }
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 1}, 0.5), BladeResult::kSilent);
  auto refusal = f.control.ReleaseEmergency(0.5);
  ASSERT_TRUE(refusal);
  EXPECT_EQ(refusal->code, "SILENT");
  // At the end: HIGH_LEVEL latched, the board held, three stops of the wheels, offs of the blade.
  std::vector<InputValue> all;
  for (double t = 1.05; t < 1.5; t += 0.01) {
    f.Board(0, t);
    for (auto& v : f.control.Tick(t)) all.push_back(v);
  }
  EXPECT_FALSE(f.control.Silent(1.1));
  const auto s = f.control.Status(1.5);
  EXPECT_TRUE(s.blocked);
  EXPECT_TRUE(s.emergency_reason & kReasonHighLevel);
  EXPECT_GE(s.latch_count, 1u);
  const auto emergency = Of(all, kEmergencyService);
  ASSERT_FALSE(emergency.empty());
  EXPECT_EQ(EmergencyOf(emergency[0]).first, reason::kHighLevel | reason::kLatch);
  const auto drive = Of(all, kDiffDriveService);
  ASSERT_EQ(drive.size(), 3u);
  for (const auto& v : drive) {
    for (double x : TwistOf(v)) EXPECT_EQ(x, 0);
  }
  EXPECT_GE(Of(all, kMowerService).size(), 3u);
  for (const auto& v : Of(all, kMowerService)) EXPECT_EQ(SpeedOf(v), 0);
  EXPECT_EQ(f.control.TakeErrors(), (std::vector<std::string>{"end of hw/silence: HIGH_LEVEL latched"}));
  // Nothing runs again until a release and a new command.
  f.control.OnCmdVel(0.1, 0, 1.5);
  EXPECT_TRUE(Of(f.control.Tick(1.51), kDiffDriveService).empty());
  EXPECT_FALSE(f.control.ReleaseEmergency(1.6, s.latch_count));
  for (double t = 1.6; t < 2.0; t += 0.01) {
    f.Board(0, t);
    f.control.Tick(t);
  }
  EXPECT_FALSE(f.control.Status(2.0).blocked);
  EXPECT_TRUE(Of(f.control.Tick(2.01), kDiffDriveService).empty());
  f.control.OnCmdVel(0.1, 0, 2.02);
  EXPECT_EQ(Of(f.control.Tick(2.02), kDiffDriveService).size(), 1u);
}
