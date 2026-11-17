#include <gtest/gtest.h>

#include <algorithm>

#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

HardwareSnapshot Healthy() {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_current = 1.2f;
  hw.battery_percentage = 80;
  hw.battery_temperature = 27;
  hw.blade.ok = true;
  hw.blade.rpm = 0;
  hw.blade.temperature = 30;
  hw.left.ok = true;
  hw.left.temperature = 31;
  hw.right.ok = true;
  hw.right.temperature = 32;
  return hw;
}

// Blocked by the driver with these reason bits.
HardwareSnapshot Stopped(int reason) {
  HardwareSnapshot hw = Healthy();
  hw.emergency = true;
  hw.emergency_reason = reason;
  return hw;
}

bool Enabled(const TestCore& core, const std::string& prefix, const std::string& id) {
  for (const auto& group : core.Actions()) {
    if (group.prefix != prefix) continue;
    for (const auto& action : group.actions) {
      if (action.id == id) return action.enabled;
    }
  }
  ADD_FAILURE() << "missing action " << prefix << "/" << id;
  return false;
}

const SensorReading* Find(const std::vector<SensorReading>& readings, const std::string& id) {
  auto it = std::find_if(readings.begin(), readings.end(), [&](const SensorReading& r) { return r.id == id; });
  return it == readings.end() ? nullptr : &*it;
}

}  // namespace

TEST(Logic, StartsInEmergencyWithoutEventOnFirstReport) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  EXPECT_TRUE(core.RobotState().emergency);
  EXPECT_TRUE(core.Emergency());
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.RobotState().emergency);
  EXPECT_FALSE(core.Emergency());
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_EQ(core.RobotState().state, "IDLE");
}

TEST(Logic, EmergencyEventsCarryReasonBits) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  auto lifted = Stopped(kEmergencyLatch | kEmergencyLift);
  core.OnHardwareStatus(lifted);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "EMERGENCY");
  EXPECT_EQ(events[0].details["emergency"], true);
  EXPECT_EQ(events[0].details["reason"], kEmergencyLatch | kEmergencyLift);
  EXPECT_EQ(events[0].details["severity"], "error");
  EXPECT_EQ(events[0].details["resume"], "user");
  core.OnHardwareStatus(Healthy());
  events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["emergency"], false);
  EXPECT_EQ(events[0].details["severity"], "info");
  EXPECT_EQ(events[0].details["resume"], "auto");
}

TEST(Logic, RefusedResetIsAnEvent) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Stopped(kEmergencyLatch | (1 << 13)));
  core.TakeEvents();
  core.RefuseReset(Reason("FOREIGN_HOST", "another host"));
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "EMERGENCY_RESET_REFUSED");
  EXPECT_EQ(events[0].details["reason"], "FOREIGN_HOST");
  EXPECT_EQ(events[0].details["message"], "another host");
  EXPECT_EQ(events[0].details["severity"], "error");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_TRUE(core.RobotState().emergency);
}

TEST(Logic, IncompatibleFirmwareIsEmergencyWithItsOwnBit) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  auto firmware = Stopped(kEmergencyFirmwareIncompatible);
  core.OnHardwareStatus(firmware);
  EXPECT_TRUE(core.RobotState().emergency);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], kEmergencyFirmwareIncompatible);
}

TEST(Logic, ReasonStaysInTheStateAndEveryChangeIsReported) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  // Before the driver reports, the mower stands for lack of telemetry.
  EXPECT_EQ(core.RobotState().emergency_reason, kEmergencyTimeoutInputs);
  core.OnHardwareStatus(Healthy());
  EXPECT_EQ(core.RobotState().emergency_reason, 0);
  EXPECT_TRUE(core.TakeReasonChanges().empty());

  auto lifted = Stopped(kEmergencyLatch | kEmergencyLift);
  core.OnHardwareStatus(lifted);
  core.OnHardwareStatus(lifted);
  auto requested = Stopped(kEmergencyLatch | kEmergencyLift | kEmergencyHighLevel);
  core.OnHardwareStatus(requested);
  EXPECT_EQ(core.RobotState().emergency_reason, kEmergencyLatch | kEmergencyLift | kEmergencyHighLevel);
  core.OnHardwareStatus(Healthy());
  EXPECT_EQ(core.RobotState().emergency_reason, 0);

  auto changes = core.TakeReasonChanges();
  ASSERT_EQ(changes.size(), 3u);
  EXPECT_EQ(changes[0].from, 0);
  EXPECT_EQ(changes[0].to, kEmergencyLatch | kEmergencyLift);
  EXPECT_EQ(changes[1].from, kEmergencyLatch | kEmergencyLift);
  EXPECT_EQ(changes[1].to, kEmergencyLatch | kEmergencyLift | kEmergencyHighLevel);
  EXPECT_EQ(changes[2].to, 0);
  // A further cause changes the reason but not the emergency: one event on, one off.
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].details["reason"], kEmergencyLatch | kEmergencyLift);
  EXPECT_EQ(events[1].details["reason"], 0);
}

TEST(Logic, LostHardwareStatusAddsTheTimeoutBit) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Stopped(kEmergencyLatch | kEmergencyLift));
  core.OnHardwareTimeout();
  EXPECT_EQ(core.RobotState().emergency_reason, kEmergencyLatch | kEmergencyLift | kEmergencyTimeoutInputs);
}

TEST(Logic, NoTelemetryFromTheStartIsReportedAsAChange) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareTimeout();
  auto changes = core.TakeReasonChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].from, 0);
  EXPECT_EQ(changes[0].to, kEmergencyTimeoutInputs);
}

TEST(Logic, ReasonWithoutEmergencyIsNotShown) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  auto bumper = Healthy();
  bumper.emergency_reason = kEmergencyCollision;
  core.OnHardwareStatus(bumper);
  EXPECT_FALSE(core.RobotState().emergency);
  EXPECT_EQ(core.RobotState().emergency_reason, 0);
}

TEST(Logic, ReasonNamesAsOpenMowerLogsThem) {
  EXPECT_EQ(EmergencyReasonNames(0), "");
  EXPECT_EQ(EmergencyReasonNames(kEmergencyLatch | kEmergencyLift), "LATCH, LIFT");
  EXPECT_EQ(EmergencyReasonNames(kEmergencyTimeoutInputs | kEmergencyStop | kEmergencyCollision),
            "TIMEOUT_INPUTS, STOP, COLLISION");
  EXPECT_EQ(EmergencyReasonNames(kEmergencyFirmwareIncompatible), "FIRMWARE_INCOMPATIBLE");
  EXPECT_EQ(EmergencyReasonNames(kEmergencyLatch | kEmergencyImuTimeout), "LATCH, IMU_TIMEOUT");
  EXPECT_EQ(EmergencyReasonNames(kEmergencyMowerNotStopped), "MOWER_NOT_STOPPED");
  EXPECT_EQ(EmergencyReasonNames((1 << 13) | (1 << 14) | (1 << 15)), "TILT, FENCE, STUCK");
  EXPECT_EQ(EmergencyReasonNames((1 << 18) | (1 << 19) | (1 << 20)), "WHEEL_RUNAWAY, FOREIGN_HOST, DRIVER_STALL");
  EXPECT_EQ(EmergencyReasonNames(kEmergencyLatch | kEmergencyUnexpectedRestart), "LATCH, UNEXPECTED_RESTART");
  EXPECT_EQ(EmergencyReasonNames(kEmergencyLatch | (1 << 22)), "LATCH, INPUTS_LEARNING");
  EXPECT_EQ(EmergencyReasonNames(kEmergencyLatch | (1 << 23) | (1 << 28)), "LATCH, UNKNOWN");
}

TEST(Logic, OwnReasonBitsReachTheRobotState) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Stopped(kEmergencyLatch | kEmergencyMowerNotStopped));
  EXPECT_EQ(core.RobotState().emergency_reason, kEmergencyLatch | kEmergencyMowerNotStopped);
  auto changes = core.TakeReasonChanges();
  ASSERT_FALSE(changes.empty());
  EXPECT_EQ(EmergencyReasonNames(changes.back().to), "LATCH, MOWER_NOT_STOPPED");
}

TEST(Logic, TelemetryTimeoutIsEmergency) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnHardwareTimeout();
  EXPECT_TRUE(core.RobotState().emergency);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], kEmergencyTimeoutInputs);
  EXPECT_TRUE(core.Readings().empty() || Find(core.Readings(), "om_v_battery") == nullptr);
}

TEST(Logic, TeleopOnlyWhileRecordingAndNoEmergency) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0, 0.0));
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  EXPECT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().state, "AREA_RECORDING");
  EXPECT_TRUE(Enabled(core, "mower_logic:area_recording", "exit_recording_mode"));
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  auto cmd = core.OnTeleop(0.2, -0.1, 2.0);
  ASSERT_TRUE(cmd);
  EXPECT_DOUBLE_EQ(cmd->first, 0.2);
  auto stopped = Stopped(kEmergencyLatch | kEmergencyStop);
  core.OnHardwareStatus(stopped);
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0, 4.0));
  core.OnHardwareStatus(Healthy());
  EXPECT_EQ(core.OnAction("mower_logic:area_recording/exit_recording_mode"), ActionResult::kDone);
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0, 6.0));
  auto events = core.TakeEvents();
  ASSERT_GE(events.size(), 2u);
  EXPECT_EQ(events.front().type, "STATE");
  EXPECT_EQ(events.front().details["state"], "AREA_RECORDING");
  EXPECT_EQ(events.back().details["state"], "IDLE");
}

namespace {

// Healthy mower in AREA_RECORDING.
TestCore RecordingCore() {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnAction("mower_logic:idle/start_area_recording");
  return core;
}

// A held joystick: teleop every 100 ms from from_s until before to_s. Returns how many reached the wheels.
int Passed(TestCore& core, double from_s, double to_s, double vx = 0.2) {
  int passed = 0;
  for (double t = from_s; t < to_s - 1e-9; t += 0.1) passed += core.OnTeleop(vx, 0.0, t) ? 1 : 0;
  return passed;
}

void Latch(TestCore& core) {
  auto lifted = Stopped(kEmergencyLatch | kEmergencyLift);
  core.OnHardwareStatus(lifted);
}

}  // namespace

TEST(Logic, HeldJoystickWaitsForZeroAfterRelease) {
  TestCore core = RecordingCore();
  EXPECT_EQ(Passed(core, 10.0, 11.0), 10);
  Latch(core);
  EXPECT_EQ(Passed(core, 11.0, 12.0), 0);
  core.OnHardwareStatus(Healthy());
  ASSERT_FALSE(core.RobotState().emergency);
  // Still held after the release: nothing moves.
  EXPECT_EQ(Passed(core, 12.0, 14.0), 0);
  // Let go: the zero goes through and the next press drives.
  auto zero = core.OnTeleop(0.0, 0.0, 14.0);
  ASSERT_TRUE(zero);
  EXPECT_DOUBLE_EQ(zero->first, 0.0);
  EXPECT_EQ(Passed(core, 14.1, 15.0), 9);
}

TEST(Logic, PauseOfOneSecondArmsTeleopAfterRelease) {
  TestCore core = RecordingCore();
  EXPECT_EQ(Passed(core, 10.0, 11.0), 10);
  Latch(core);
  core.OnHardwareStatus(Healthy());
  // The last teleop came at 10.9: a press 0.5 s later is held back, as is everything close to it.
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0, 11.4));
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0, 12.3));
  // A full second of silence arms it again.
  EXPECT_TRUE(core.OnTeleop(0.2, 0.0, 13.3));
  EXPECT_TRUE(core.OnTeleop(0.2, 0.0, 13.4));
}

// A new recording starts disarmed, also when the last one ended armed without a zero.
TEST(Logic, ANewRecordingWaitsForZeroOrQuiet) {
  TestCore core = RecordingCore();
  EXPECT_EQ(Passed(core, 10.0, 11.0), 10);
  ASSERT_EQ(core.OnAction("mower_logic:area_recording/exit_recording_mode"), ActionResult::kDone);
  ASSERT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kDone);
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0, 11.0).has_value());
}

TEST(Logic, ZeroDuringEmergencyDoesNotArm) {
  TestCore core = RecordingCore();
  EXPECT_EQ(Passed(core, 10.0, 11.0), 10);
  Latch(core);
  // Let go and pressed again while stopped: the zero came too early to count.
  EXPECT_FALSE(core.OnTeleop(0.0, 0.0, 11.0));
  EXPECT_EQ(Passed(core, 11.1, 12.0), 0);
  core.OnHardwareStatus(Healthy());
  EXPECT_EQ(Passed(core, 12.0, 13.0), 0);
}

TEST(Logic, HeldJoystickWaitsWhenRecordingStarts) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  EXPECT_EQ(Passed(core, 10.0, 11.0), 0);
  core.OnAction("mower_logic:idle/start_area_recording");
  EXPECT_EQ(Passed(core, 11.0, 12.0), 0);
  EXPECT_TRUE(core.OnTeleop(0.0, 0.0, 12.0));
  EXPECT_EQ(Passed(core, 12.1, 13.0), 9);
}

TEST(Logic, LeavingRecordingStopsTheWheels) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.TakeStopRequest());
  core.OnAction("mower_logic:idle/start_area_recording");
  EXPECT_FALSE(core.TakeStopRequest());
  EXPECT_EQ(core.OnAction("mower_logic:area_recording/exit_recording_mode"), ActionResult::kDone);
  EXPECT_TRUE(core.TakeStopRequest());
  EXPECT_FALSE(core.TakeStopRequest());
  // Rejected actions change nothing.
  EXPECT_EQ(core.OnAction("mower_logic:area_recording/exit_recording_mode"), ActionResult::kRejected);
  EXPECT_FALSE(core.TakeStopRequest());
}

TEST(Logic, NoRecordingDuringEmergency) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  auto latched = Stopped(kEmergencyLatch | kEmergencyHighLevel);
  core.OnHardwareStatus(latched);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  EXPECT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kRejected);
  EXPECT_EQ(core.OnAction("mower_logic/reset_emergency"), ActionResult::kReleaseEmergency);
  EXPECT_EQ(core.OnAction("mower_logic:idle/start_mowing"), ActionResult::kRejected);
  EXPECT_EQ(core.OnAction("something/else"), ActionResult::kUnknown);
}

TEST(Logic, ActionsChangeFlag) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  EXPECT_TRUE(core.TakeActionsChanged());
  EXPECT_FALSE(core.TakeActionsChanged());
  core.OnHardwareStatus(Healthy());
  EXPECT_TRUE(core.TakeActionsChanged());
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.TakeActionsChanged());
}

TEST(Logic, ActionsChangeWithHoldAndEmergencyAlsoWhenTheStartReasonStays) {
  // An update holds the logic: the reason against a start stays HOLD throughout.
  const auto held = [] {
    TestCore core{LogicConfig{}};
    core.OnMapStatus(MapDatum::kMatch, "");
    core.SetHold(1);
    core.TakeActionsChanged();
    return core;
  };
  TestCore core = held();
  core.OnHardwareStatus(Healthy());
  EXPECT_TRUE(core.TakeActionsChanged());
  core.OnHardwareStatus(Stopped(kEmergencyLatch));
  EXPECT_TRUE(core.TakeActionsChanged());
  // A first report that keeps the emergency, with the driver holding too.
  TestCore driver = held();
  HardwareSnapshot hw = Stopped(kEmergencyLatch);
  hw.hold = true;
  driver.OnHardwareStatus(hw);
  EXPECT_TRUE(driver.TakeActionsChanged());
}

TEST(Logic, BladesRunFromTheRunningRpmOn) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.TakeEvents();
  HardwareSnapshot hw = Healthy();
  hw.blade.rpm = LogicConfig{}.blade_running_rpm;
  core.OnHardwareStatus(hw);
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "BLADES");
  EXPECT_EQ(events[0].details["enabled"], true);
}

TEST(Logic, RobotStateAndSensors) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  auto docked = Healthy();
  docked.dock_contact = true;
  docked.battery_current = -3.0f;
  docked.rain = true;
  core.OnHardwareStatus(docked);
  auto state = core.RobotState();
  EXPECT_FLOAT_EQ(state.battery_percentage, 0.8f);
  EXPECT_TRUE(state.is_charging);
  EXPECT_TRUE(state.rain_detected);
  auto readings = core.Readings();
  ASSERT_TRUE(Find(readings, "om_v_charge"));
  EXPECT_DOUBLE_EQ(Find(readings, "om_v_charge")->value, 28.0);
  EXPECT_DOUBLE_EQ(Find(readings, "om_charge_current")->value, 3.0);
  EXPECT_EQ(Find(readings, "om_charge_state")->text, "Fast Charge");
  EXPECT_DOUBLE_EQ(Find(readings, "om_left_esc_temp")->value, 31.0);
  EXPECT_DOUBLE_EQ(Find(readings, "battery_temp")->value, 27.0);
  // One temperature per motor board: no mow ESC value that would only repeat the motor's.
  EXPECT_EQ(Find(readings, "om_mow_esc_temp"), nullptr);

  auto full = docked;
  full.battery_current = 0.0f;
  full.battery_percentage = 100;
  core.OnHardwareStatus(full);
  EXPECT_EQ(Find(core.Readings(), "om_charge_state")->text, "Done");
}

TEST(Logic, SensorDefinitionsMatchTheApp) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  auto sensors = core.Sensors();
  for (const char* id :
       {"om_v_battery", "om_v_charge", "om_charge_current", "om_charge_state", "om_mow_motor_rpm",
        "om_mow_motor_current", "om_mow_motor_temp", "om_left_esc_temp", "om_right_esc_temp", "battery_temp",
        "om_gps_accuracy"}) {
    EXPECT_TRUE(std::any_of(sensors.begin(), sensors.end(), [&](const SensorDefinition& s) { return s.id == id; }))
        << id;
  }
  EXPECT_FALSE(std::any_of(sensors.begin(), sensors.end(), [](const auto& s) { return s.id == "om_mow_esc_temp"; }));
  auto battery = std::find_if(sensors.begin(), sensors.end(), [](const auto& s) { return s.id == "om_v_battery"; });
  // The app derives the cell count from max_value: 25.2 V / 4.2 V = 6.
  EXPECT_DOUBLE_EQ(battery->max, 25.2);
}

namespace {

PoseSnapshot Pose(double x, double y, float accuracy, bool heading_valid = true) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.heading = 1.5;
  pose.position_accuracy = accuracy;
  pose.orientation_accuracy = 0.01f;
  pose.orientation_valid = heading_valid;
  return pose;
}

}  // namespace

TEST(Logic, WithoutPoseAccuracyIsNeverPerfect) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  auto state = core.RobotState();
  EXPECT_FLOAT_EQ(state.pos_accuracy, 999.0f);
  EXPECT_FLOAT_EQ(state.heading_accuracy, 999.0f);
  EXPECT_FLOAT_EQ(state.gps_percentage, 0.0f);
  EXPECT_FALSE(state.heading_valid);
  EXPECT_EQ(Find(core.Readings(), "om_gps_accuracy"), nullptr);
}

TEST(Logic, PoseFillsRobotStateAndGpsPercentage) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnPose(Pose(12.5, -3.25, 0.02f));
  auto state = core.RobotState();
  EXPECT_DOUBLE_EQ(state.pose_x, 12.5);
  EXPECT_DOUBLE_EQ(state.pose_y, -3.25);
  EXPECT_DOUBLE_EQ(state.pose_heading, 1.5);
  EXPECT_FLOAT_EQ(state.pos_accuracy, 0.02f);
  EXPECT_FLOAT_EQ(state.heading_accuracy, 0.01f);
  EXPECT_TRUE(state.heading_valid);
  // 1 - 0.02 / 0.1
  EXPECT_NEAR(state.gps_percentage, 0.8f, 1e-6);
  EXPECT_FLOAT_EQ(static_cast<float>(Find(core.Readings(), "om_gps_accuracy")->value), 0.02f);
}

TEST(Logic, GpsPercentageIsZeroWhenTooInaccurateOrWithoutHeading) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnPose(Pose(0, 0, 0.1f));
  EXPECT_FLOAT_EQ(core.RobotState().gps_percentage, 0.0f);
  core.OnPose(Pose(0, 0, 999.0f));
  EXPECT_FLOAT_EQ(core.RobotState().gps_percentage, 0.0f);
  EXPECT_FLOAT_EQ(core.RobotState().pos_accuracy, 999.0f);
  core.OnPose(Pose(0, 0, 0.01f, /*heading_valid=*/false));
  EXPECT_FLOAT_EQ(core.RobotState().gps_percentage, 0.0f);
  core.OnPose(Pose(0, 0, 0.0f));
  EXPECT_FLOAT_EQ(core.RobotState().gps_percentage, 1.0f);
}

TEST(Logic, PoseTimeoutDropsPoseButEventsKeepLastPosition) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnPose(Pose(4.0, 5.0, 0.02f));
  core.OnPoseTimeout();
  auto state = core.RobotState();
  EXPECT_FLOAT_EQ(state.pos_accuracy, 999.0f);
  EXPECT_FLOAT_EQ(state.gps_percentage, 0.0f);
  EXPECT_EQ(Find(core.Readings(), "om_gps_accuracy"), nullptr);
  core.OnAction("mower_logic:idle/start_area_recording");
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_DOUBLE_EQ(events[0].details["x"].get<double>(), 4.0);
  EXPECT_DOUBLE_EQ(events[0].details["y"].get<double>(), 5.0);
}

TEST(Logic, EventsCarryThePositionTheyHappenedAt) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnPose(Pose(1.0, 2.0, 0.02f));
  core.OnAction("mower_logic:idle/start_area_recording");
  core.OnPose(Pose(7.0, 8.0, 0.02f));
  auto lift = Stopped(kEmergencyLatch | kEmergencyLift);
  core.OnHardwareStatus(lift);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].type, "STATE");
  EXPECT_DOUBLE_EQ(events[0].details["x"].get<double>(), 1.0);
  EXPECT_EQ(events[1].type, "EMERGENCY");
  EXPECT_DOUBLE_EQ(events[1].details["x"].get<double>(), 7.0);
  EXPECT_DOUBLE_EQ(events[1].details["y"].get<double>(), 8.0);
}

TEST(Logic, PositionJumpIsAnEventAtTheNewPosition) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnPose(Pose(1.0, 2.0, 0.02f));
  core.OnPositionJump(7.0, 2.0, 6.0, true);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "POSITION_JUMP");
  EXPECT_DOUBLE_EQ(events[0].details["distance"].get<double>(), 6.0);
  EXPECT_EQ(events[0].details["reason"], "OUTLIERS");
  EXPECT_EQ(events[0].details["severity"], "warning");
  EXPECT_EQ(events[0].details["resume"], "auto");
  EXPECT_DOUBLE_EQ(events[0].details["x"].get<double>(), 7.0);
  core.OnPositionJump(7.5, 2.0, 0.5, false);
  EXPECT_EQ(core.TakeEvents()[0].details["reason"], "GAP");
}

TEST(Logic, EventsWithoutAnyPoseAreAtZero) {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnAction("mower_logic:idle/start_area_recording");
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_DOUBLE_EQ(events[0].details["x"].get<double>(), 0.0);
}

namespace {

PoseSnapshot GoodPose() {
  PoseSnapshot pose;
  pose.x = 1;
  pose.y = 2;
  pose.position_accuracy = 0.02f;
  pose.orientation_accuracy = 0.01f;
  pose.orientation_valid = true;
  return pose;
}

// Idle, healthy, with a good pose and a safety measurement: a drive may start.
TestCore ReadyToDrive() {
  TestCore core{LogicConfig{}};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(GoodPose());
  core.TakeEvents();
  return core;
}

bool HasEvent(const std::vector<Event>& events, const std::string& type) {
  return std::any_of(events.begin(), events.end(), [&](const Event& e) { return e.type == type; });
}

}  // namespace

TEST(LogicDrive, StartsFromIdleAndShowsDriving) {
  TestCore core = ReadyToDrive();
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_TRUE(core.StartDrive(5, 3).empty());
  EXPECT_TRUE(core.Driving());
  EXPECT_TRUE(core.FollowerActive());
  EXPECT_EQ(core.RobotState().state, "IDLE");
  EXPECT_EQ(core.RobotState().sub_state, "DRIVING");
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].type, "STATE");
  EXPECT_EQ(events[0].details["sub_state"], "DRIVING");
  EXPECT_EQ(events[1].type, "DRIVE");
  EXPECT_EQ(events[1].details["phase"], "start");
  EXPECT_EQ(events[1].details["goal_x"], 5.0);
  // No second drive and no recording meanwhile.
  const Reason busy = core.StartDrive(6, 3);
  EXPECT_EQ(busy.code, "BUSY");
  EXPECT_EQ(busy.fields["state"], "driving");
  EXPECT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kRejected);
}

TEST(LogicDrive, EndStopsTheWheelsAndGoesBackToIdle) {
  TestCore core = ReadyToDrive();
  ASSERT_TRUE(core.StartDrive(5, 3).empty());
  core.TakeEvents();
  core.TakeStopRequest();
  core.OnDriveEnded(false, Reason("POSITION_ACCURACY", "Position accuracy 0.30 m"));
  EXPECT_FALSE(core.Driving());
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_TRUE(core.TakeStopRequest());
  EXPECT_EQ(core.RobotState().sub_state, "");
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].type, "DRIVE");
  EXPECT_EQ(events[0].details["arrived"], false);
  EXPECT_EQ(events[0].details["reason"], "POSITION_ACCURACY");
  EXPECT_EQ(events[0].details["message"], "Position accuracy 0.30 m");
  EXPECT_EQ(events[0].details["severity"], "warning");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_EQ(events[1].type, "STATE");
  EXPECT_EQ(events[1].details["state"], "IDLE");
  // A late end changes nothing.
  core.OnDriveEnded(true, Reason("DONE", ""));
  EXPECT_TRUE(core.TakeEvents().empty());
}

TEST(LogicDrive, AFollowerThatStopsAnsweringEndsTheDrive) {
  // G22: the node ends the drive itself; the wheels get a stop and the event says why, with the seconds.
  TestCore core = ReadyToDrive();
  ASSERT_TRUE(core.StartDrive(5, 3).empty());
  core.TakeEvents();
  core.TakeStopRequest();
  core.OnDriveEnded(false, Reason("FOLLOWER_MISSING", "No answer from the follower for 2.0 s")
                               .With("seconds", 2.04)
                               .With("limit", 2.0));
  EXPECT_FALSE(core.Driving());
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_TRUE(core.TakeStopRequest());
  const auto events = core.TakeEvents();
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events[0].type, "DRIVE");
  EXPECT_EQ(events[0].details["phase"], "end");
  EXPECT_EQ(events[0].details["reason"], "FOLLOWER_MISSING");
  EXPECT_DOUBLE_EQ(events[0].details["seconds"].get<double>(), 2.04);
  EXPECT_EQ(events[0].details["severity"], "error");
  EXPECT_EQ(events[0].details["resume"], "user");
}

TEST(LogicDrive, RefusedWithAReason) {
  {
    TestCore core = ReadyToDrive();
    core.OnCapabilities(false);
    EXPECT_EQ(core.StartDrive(5, 3).code, "NO_SAFETY_MEASUREMENT");
    const auto events = core.TakeEvents();
    ASSERT_TRUE(HasEvent(events, "DRIVE_REFUSED"));
    EXPECT_EQ(events.back().details["reason"], "NO_SAFETY_MEASUREMENT");
    EXPECT_EQ(events.back().details["severity"], "error");
    EXPECT_EQ(events.back().details["resume"], "user");
    EXPECT_FALSE(events.back().details["message"].get<std::string>().empty());
  }
  {
    TestCore core = ReadyToDrive();
    core.OnHardwareStatus(Stopped(kEmergencyLatch | kEmergencyStop));
    EXPECT_EQ(core.StartDrive(5, 3).code, "EMERGENCY");
  }
  {
    TestCore core = ReadyToDrive();
    core.OnPoseTimeout();
    EXPECT_EQ(core.StartDrive(5, 3).code, "NO_POSITION");
  }
  {
    TestCore core = ReadyToDrive();
    PoseSnapshot pose = GoodPose();
    pose.position_accuracy = 0.3f;
    core.OnPose(pose);
    const Reason why = core.StartDrive(5, 3);
    EXPECT_EQ(why.code, "POSITION_ACCURACY");
    EXPECT_FLOAT_EQ(why.fields["accuracy"].get<float>(), 0.3f);
    EXPECT_DOUBLE_EQ(why.fields["limit"].get<double>(), 0.1);
    EXPECT_EQ(why.message, "Position accuracy 0.300 m is worse than 0.100 m");
  }
  {
    TestCore core = ReadyToDrive();
    PoseSnapshot pose = GoodPose();
    pose.orientation_valid = false;
    core.OnPose(pose);
    EXPECT_EQ(core.StartDrive(5, 3).code, "NO_HEADING");
  }
  {
    TestCore core = ReadyToDrive();
    ASSERT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kDone);
    const Reason why = core.StartDrive(5, 3);
    EXPECT_EQ(why.code, "BUSY");
    EXPECT_EQ(why.fields["state"], "AREA_RECORDING");
    EXPECT_FALSE(core.Driving());
  }
}

TEST(LogicDrive, EmergencyCutsTheFollowerOffAtOnceAndCancels) {
  TestCore core = ReadyToDrive();
  ASSERT_TRUE(core.StartDrive(5, 3).empty());
  EXPECT_FALSE(core.TakeDriveCancel());
  core.OnHardwareStatus(Stopped(kEmergencyLatch | kEmergencyCollision));
  EXPECT_FALSE(core.FollowerActive());
  const auto cancel = core.TakeDriveCancel();
  ASSERT_TRUE(cancel.has_value());
  EXPECT_EQ(cancel->code, "EMERGENCY");
  EXPECT_FALSE(core.TakeDriveCancel());
  // Driving until the follower has answered, then idle; the emergency stays.
  EXPECT_TRUE(core.Driving());
  core.OnDriveEnded(false, *cancel);
  EXPECT_FALSE(core.Driving());
  // After the release the follower stays off: a new drive needs a new request.
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.FollowerActive());
}

TEST(LogicDrive, LostTelemetryPoseOrSafetyMeasurementCancel) {
  {
    TestCore core = ReadyToDrive();
    ASSERT_TRUE(core.StartDrive(5, 3).empty());
    core.OnHardwareTimeout();
    EXPECT_FALSE(core.FollowerActive());
    EXPECT_EQ(core.TakeDriveCancel().value_or(Reason()).code, "EMERGENCY");
  }
  {
    TestCore core = ReadyToDrive();
    ASSERT_TRUE(core.StartDrive(5, 3).empty());
    core.OnPoseTimeout();
    EXPECT_EQ(core.TakeDriveCancel().value_or(Reason()).code, "POSE_LOST");
  }
  {
    TestCore core = ReadyToDrive();
    ASSERT_TRUE(core.StartDrive(5, 3).empty());
    core.OnCapabilities(false);
    EXPECT_EQ(core.TakeDriveCancel().value_or(Reason()).code, "NO_SAFETY_MEASUREMENT");
  }
}

TEST(LogicDrive, TeleopNeverReachesTheWheelsWhileDriving) {
  TestCore core = ReadyToDrive();
  ASSERT_TRUE(core.StartDrive(5, 3).empty());
  EXPECT_FALSE(core.OnTeleop(0.0, 0.0, 10.0).has_value());
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0, 12.0).has_value());
}

// A point asked for by hand is refused when the pose goes away, in the events of that same call.
TEST(Logic, PoseTimeoutRefusesTheRequestedPointAtOnce) {
  TestCore core = RecordingCore();
  core.OnPose(Pose(1.0, 2.0, 0.02f));
  ASSERT_EQ(core.OnAction("mower_logic:area_recording/start_recording"), ActionResult::kDone);
  ASSERT_EQ(core.OnAction("mower_logic:area_recording/auto_point_collecting_disable"), ActionResult::kDone);
  ASSERT_EQ(core.OnAction("mower_logic:area_recording/collect_point"), ActionResult::kDone);
  core.TakeEvents();
  core.OnPoseTimeout();
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "RECORDING_REFUSED");
  EXPECT_EQ(events[0].details["action"], "collect_point");
  EXPECT_EQ(events[0].details["reason"], "NO_POSITION");
}

// Leaving recording takes the actions change of the recorder along: a later answer of the map changes nothing.
TEST(Logic, AMapAnswerAfterRecordingChangesNoActions) {
  TestCore core = RecordingCore();
  ASSERT_EQ(core.OnAction("mower_logic:area_recording/exit_recording_mode"), ActionResult::kDone);
  core.TakeActionsChanged();
  core.OnMapEditResult(true, "lawn1", "", "");
  EXPECT_FALSE(core.TakeActionsChanged());
}
