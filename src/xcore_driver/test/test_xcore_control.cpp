#include <gtest/gtest.h>

#include <cmath>

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
  // 0.3 m/s and -0.3 rad/s, as duty: the outer wheel at the duty limit, the inner one slower.
  double half = ControlConfig{}.regulator.wheel_distance / 2;
  EXPECT_NEAR(twist[0] - twist[5] * half, 0.3, 1e-9);
  EXPECT_NEAR(twist[0] + twist[5] * half, 0.3 - 0.3 * half, 1e-9);
  EXPECT_EQ(twist[1] + twist[2] + twist[3] + twist[4], 0);
  EXPECT_TRUE(f.control.Status(0).moving);
  // Resent every 0.1 s until cmd_vel falls silent for 0.5 s, then three stops 0.1 s apart, then nothing.
  std::vector<std::pair<double, double>> sent;
  for (int i = 1; i <= 150; i++) {
    double t = i * 0.01;
    f.Board(0, t);
    for (const auto& v : Of(f.control.Tick(t), kDiffDriveService)) sent.emplace_back(t, TwistOf(v)[0]);
  }
  int moving = 0, stops = 0;
  for (const auto& [t, v] : sent) {
    if (v != 0) {
      moving++;
      EXPECT_LE(t, 0.5 + 1e-9);
    } else {
      stops++;
      EXPECT_GT(t, 0.5);
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
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 1.0}, 0.1), BladeResult::kBlocked);
}

TEST(XcoreControl, BladeLease) {
  Fixture f;
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 0}, 0), BladeResult::kInvalidLease);
  EXPECT_EQ(f.control.SetBlade({true, 0, 0, 2.5}, 0), BladeResult::kInvalidLease);
  EXPECT_EQ(f.control.SetBlade({true, 101, 0, 1}, 0), BladeResult::kInvalidSpeed);
  EXPECT_EQ(f.control.SetBlade({true, 0, 50, 1}, 0), BladeResult::kInvalidHeight);
  ASSERT_EQ(f.control.SetBlade({true, 50, 0, 0.5}, 0), BladeResult::kOk);
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
  EXPECT_TRUE(s.emergency_reason & kReasonMowerNotStopped);
  // The board gets a high level emergency, the blade more offs.
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
