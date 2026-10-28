#include <gtest/gtest.h>

#include <algorithm>

#include "mowbite_logic/logic_core.h"

using namespace mowbite_logic;

namespace {

McuSnapshot Healthy() {
  McuSnapshot mcu;
  mcu.telemetry_ok = true;
  mcu.battery_ok = true;
  mcu.battery_voltage = 24.5f;
  mcu.battery_current = 1.2f;
  mcu.battery_percentage = 80;
  mcu.battery_temperature = 27;
  mcu.cutter.ok = true;
  mcu.cutter.rpm = 0;
  mcu.cutter.temperature = 30;
  mcu.left.ok = true;
  mcu.left.temperature = 31;
  mcu.right.ok = true;
  mcu.right.temperature = 32;
  return mcu;
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
  core.OnMcuStatus(Healthy());
  EXPECT_FALSE(core.RobotState().emergency);
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_EQ(core.RobotState().state, "IDLE");
}

TEST(Logic, EmergencyEventsCarryReasonBits) {
  LogicCore core{LogicConfig{}};
  core.OnMcuStatus(Healthy());
  auto lifted = Healthy();
  lifted.lift = true;
  lifted.emergency_latched = true;
  core.OnMcuStatus(lifted);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "EMERGENCY");
  EXPECT_EQ(events[0].details["emergency"], true);
  EXPECT_EQ(events[0].details["reason"], kEmergencyLatch | kEmergencyLift);
  core.OnMcuStatus(Healthy());
  events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["emergency"], false);
}

TEST(Logic, TelemetryTimeoutIsEmergency) {
  LogicCore core{LogicConfig{}};
  core.OnMcuStatus(Healthy());
  core.OnMcuTimeout();
  EXPECT_TRUE(core.RobotState().emergency);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], kEmergencyTimeoutInputs);
  EXPECT_TRUE(core.Readings().empty() || Find(core.Readings(), "om_v_battery") == nullptr);
}

TEST(Logic, TeleopOnlyWhileRecordingAndNoEmergency) {
  LogicCore core{LogicConfig{}};
  core.OnMcuStatus(Healthy());
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0));
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  EXPECT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().state, "AREA_RECORDING");
  EXPECT_TRUE(Enabled(core, "mower_logic:area_recording", "exit_recording_mode"));
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  auto cmd = core.OnTeleop(0.2, -0.1);
  ASSERT_TRUE(cmd);
  EXPECT_DOUBLE_EQ(cmd->first, 0.2);
  auto stopped = Healthy();
  stopped.stop = true;
  stopped.emergency_latched = true;
  core.OnMcuStatus(stopped);
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0));
  core.OnMcuStatus(Healthy());
  EXPECT_EQ(core.OnAction("mower_logic:area_recording/exit_recording_mode"), ActionResult::kDone);
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0));
  auto events = core.TakeEvents();
  ASSERT_GE(events.size(), 2u);
  EXPECT_EQ(events.front().type, "STATE");
  EXPECT_EQ(events.front().details["state"], "AREA_RECORDING");
  EXPECT_EQ(events.back().details["state"], "IDLE");
}

TEST(Logic, NoRecordingDuringEmergency) {
  LogicCore core{LogicConfig{}};
  auto latched = Healthy();
  latched.emergency_latched = true;
  core.OnMcuStatus(latched);
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
  core.OnMcuStatus(Healthy());
  EXPECT_TRUE(core.TakeActionsChanged());
  core.OnMcuStatus(Healthy());
  EXPECT_FALSE(core.TakeActionsChanged());
}

TEST(Logic, RobotStateAndSensors) {
  LogicCore core{LogicConfig{}};
  auto docked = Healthy();
  docked.battery_dock = true;
  docked.battery_current = -3.0f;
  docked.rain = true;
  core.OnMcuStatus(docked);
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
  core.OnMcuStatus(full);
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
  core.OnMcuStatus(Healthy());
  auto state = core.RobotState();
  EXPECT_FLOAT_EQ(state.pos_accuracy, 999.0f);
  EXPECT_FLOAT_EQ(state.heading_accuracy, 999.0f);
  EXPECT_FLOAT_EQ(state.gps_percentage, 0.0f);
  EXPECT_FALSE(state.heading_valid);
  EXPECT_EQ(Find(core.Readings(), "om_gps_accuracy"), nullptr);
}

TEST(Logic, PoseFillsRobotStateAndGpsPercentage) {
  LogicCore core{LogicConfig{}};
  core.OnMcuStatus(Healthy());
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
  core.OnMcuStatus(Healthy());
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
  core.OnMcuStatus(Healthy());
  core.OnPose(Pose(1.0, 2.0, 0.02f));
  core.OnAction("mower_logic:idle/start_area_recording");
  core.OnPose(Pose(7.0, 8.0, 0.02f));
  auto lift = Healthy();
  lift.emergency_latched = true;
  lift.lift = true;
  core.OnMcuStatus(lift);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].type, "STATE");
  EXPECT_DOUBLE_EQ(events[0].details["x"].get<double>(), 1.0);
  EXPECT_EQ(events[1].type, "EMERGENCY");
  EXPECT_DOUBLE_EQ(events[1].details["x"].get<double>(), 7.0);
  EXPECT_DOUBLE_EQ(events[1].details["y"].get<double>(), 8.0);
}

TEST(Logic, EventsWithoutAnyPoseAreAtZero) {
  LogicCore core{LogicConfig{}};
  core.OnMcuStatus(Healthy());
  core.OnAction("mower_logic:idle/start_area_recording");
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_DOUBLE_EQ(events[0].details["x"].get<double>(), 0.0);
}
