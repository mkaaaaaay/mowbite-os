#include <gtest/gtest.h>

#include <algorithm>

#include "mowbite_logic/logic_core.h"

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

bool Enabled(const LogicCore& core, const std::string& prefix, const std::string& id) {
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
  LogicCore core{LogicConfig{}};
  EXPECT_TRUE(core.RobotState().emergency);
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.RobotState().emergency);
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_EQ(core.RobotState().state, "IDLE");
}

TEST(Logic, EmergencyEventsCarryReasonBits) {
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  auto lifted = Stopped(kEmergencyLatch | kEmergencyLift);
  core.OnHardwareStatus(lifted);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "EMERGENCY");
  EXPECT_EQ(events[0].details["emergency"], true);
  EXPECT_EQ(events[0].details["reason"], kEmergencyLatch | kEmergencyLift);
  core.OnHardwareStatus(Healthy());
  events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["emergency"], false);
}

TEST(Logic, IncompatibleFirmwareIsEmergencyWithItsOwnBit) {
  LogicCore core{LogicConfig{}};
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
  LogicCore core{LogicConfig{}};
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
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Stopped(kEmergencyLatch | kEmergencyLift));
  core.OnHardwareTimeout();
  EXPECT_EQ(core.RobotState().emergency_reason, kEmergencyLatch | kEmergencyLift | kEmergencyTimeoutInputs);
}

TEST(Logic, NoTelemetryFromTheStartIsReportedAsAChange) {
  LogicCore core{LogicConfig{}};
  core.OnHardwareTimeout();
  auto changes = core.TakeReasonChanges();
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].from, 0);
  EXPECT_EQ(changes[0].to, kEmergencyTimeoutInputs);
}

TEST(Logic, ReasonWithoutEmergencyIsNotShown) {
  LogicCore core{LogicConfig{}};
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
}

TEST(Logic, TelemetryTimeoutIsEmergency) {
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  core.OnHardwareTimeout();
  EXPECT_TRUE(core.RobotState().emergency);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], kEmergencyTimeoutInputs);
  EXPECT_TRUE(core.Readings().empty() || Find(core.Readings(), "om_v_battery") == nullptr);
}

TEST(Logic, TeleopOnlyWhileRecordingAndNoEmergency) {
  LogicCore core{LogicConfig{}};
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
LogicCore RecordingCore() {
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  core.OnAction("mower_logic:idle/start_area_recording");
  return core;
}

// A held joystick: teleop every 100 ms from from_s until before to_s. Returns how many reached the wheels.
int Passed(LogicCore& core, double from_s, double to_s, double vx = 0.2) {
  int passed = 0;
  for (double t = from_s; t < to_s - 1e-9; t += 0.1) passed += core.OnTeleop(vx, 0.0, t) ? 1 : 0;
  return passed;
}

void Latch(LogicCore& core) {
  auto lifted = Stopped(kEmergencyLatch | kEmergencyLift);
  core.OnHardwareStatus(lifted);
}

}  // namespace

TEST(Logic, HeldJoystickWaitsForZeroAfterRelease) {
  LogicCore core = RecordingCore();
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
  LogicCore core = RecordingCore();
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

TEST(Logic, ZeroDuringEmergencyDoesNotArm) {
  LogicCore core = RecordingCore();
  EXPECT_EQ(Passed(core, 10.0, 11.0), 10);
  Latch(core);
  // Let go and pressed again while stopped: the zero came too early to count.
  EXPECT_FALSE(core.OnTeleop(0.0, 0.0, 11.0));
  EXPECT_EQ(Passed(core, 11.1, 12.0), 0);
  core.OnHardwareStatus(Healthy());
  EXPECT_EQ(Passed(core, 12.0, 13.0), 0);
}

TEST(Logic, HeldJoystickWaitsWhenRecordingStarts) {
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  EXPECT_EQ(Passed(core, 10.0, 11.0), 0);
  core.OnAction("mower_logic:idle/start_area_recording");
  EXPECT_EQ(Passed(core, 11.0, 12.0), 0);
  EXPECT_TRUE(core.OnTeleop(0.0, 0.0, 12.0));
  EXPECT_EQ(Passed(core, 12.1, 13.0), 9);
}

TEST(Logic, LeavingRecordingStopsTheWheels) {
  LogicCore core{LogicConfig{}};
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
  LogicCore core{LogicConfig{}};
  auto latched = Stopped(kEmergencyLatch | kEmergencyHighLevel);
  core.OnHardwareStatus(latched);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  EXPECT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kRejected);
  EXPECT_EQ(core.OnAction("mower_logic/reset_emergency"), ActionResult::kReleaseEmergency);
  EXPECT_EQ(core.OnAction("mower_logic:idle/start_mowing"), ActionResult::kRejected);
  EXPECT_EQ(core.OnAction("something/else"), ActionResult::kUnknown);
}

TEST(Logic, ActionsChangeFlag) {
  LogicCore core{LogicConfig{}};
  EXPECT_TRUE(core.TakeActionsChanged());
  EXPECT_FALSE(core.TakeActionsChanged());
  core.OnHardwareStatus(Healthy());
  EXPECT_TRUE(core.TakeActionsChanged());
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.TakeActionsChanged());
}

TEST(Logic, RobotStateAndSensors) {
  LogicCore core{LogicConfig{}};
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
  LogicCore core{LogicConfig{}};
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
  EXPECT_DOUBLE_EQ(battery->max_value, 25.2);
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
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  auto state = core.RobotState();
  EXPECT_FLOAT_EQ(state.pos_accuracy, 999.0f);
  EXPECT_FLOAT_EQ(state.heading_accuracy, 999.0f);
  EXPECT_FLOAT_EQ(state.gps_percentage, 0.0f);
  EXPECT_FALSE(state.heading_valid);
  EXPECT_EQ(Find(core.Readings(), "om_gps_accuracy"), nullptr);
}

TEST(Logic, PoseFillsRobotStateAndGpsPercentage) {
  LogicCore core{LogicConfig{}};
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
  LogicCore core{LogicConfig{}};
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
  LogicCore core{LogicConfig{}};
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
  LogicCore core{LogicConfig{}};
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
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  core.OnPose(Pose(1.0, 2.0, 0.02f));
  core.OnPositionJump(7.0, 2.0, 6.0, true);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "POSITION_JUMP");
  EXPECT_DOUBLE_EQ(events[0].details["distance"].get<double>(), 6.0);
  EXPECT_EQ(events[0].details["reason"], "outliers");
  EXPECT_DOUBLE_EQ(events[0].details["x"].get<double>(), 7.0);
  core.OnPositionJump(7.5, 2.0, 0.5, false);
  EXPECT_EQ(core.TakeEvents()[0].details["reason"], "gap");
}

TEST(Logic, EventsWithoutAnyPoseAreAtZero) {
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  core.OnAction("mower_logic:idle/start_area_recording");
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_DOUBLE_EQ(events[0].details["x"].get<double>(), 0.0);
}
