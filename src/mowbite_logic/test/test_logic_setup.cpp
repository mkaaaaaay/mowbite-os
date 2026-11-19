#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "logic_test_core.h"
#include "mowbite_logic/logic_core.h"

using namespace mowbite_logic;

namespace {

HardwareSnapshot Healthy() {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  return hw;
}

// IDLE off the station, no safety measurement and no position: the setup needs neither.
TestCore Ready() {
  TestCore core{LogicConfig()};
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(false);
  core.TakeEvents();
  core.TakeNotices();
  core.TakeActionsChanged();
  core.TakeStopRequest();
  return core;
}

std::vector<std::string> Types(const std::vector<Event>& events) {
  std::vector<std::string> types;
  for (const auto& e : events) types.push_back(e.type);
  return types;
}

}  // namespace

TEST(LogicSetup, BeginsFromIdleAndEndsWithItsMotion) {
  TestCore core = Ready();
  ASSERT_TRUE(core.OnSetupMotion(true, "wheels", 10.0).empty());
  EXPECT_TRUE(core.InSetup());
  EXPECT_TRUE(core.SetupActive());
  EXPECT_FALSE(core.Idle());
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_FALSE(core.CalibrationActive());
  const RobotStateData state = core.RobotState();
  EXPECT_EQ(state.state, "IDLE");
  EXPECT_EQ(state.sub_state, "SETUP");
  auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE"}));
  EXPECT_EQ(events[0].details["state"], "IDLE");
  EXPECT_EQ(events[0].details["sub_state"], "SETUP");
  // Other starts are busy meanwhile.
  EXPECT_EQ(core.StartCalibration(10.1).code, "BUSY");
  core.TakeEvents();
  // The program goes on: nothing changes; its end leaves SETUP with one zero.
  EXPECT_TRUE(core.OnSetupMotion(true, "wheels", 10.25).empty());
  EXPECT_FALSE(core.TickSetup(11.0));
  EXPECT_TRUE(core.OnSetupMotion(false, "wheels", 11.1).empty());
  EXPECT_FALSE(core.InSetup());
  EXPECT_TRUE(core.Idle());
  EXPECT_TRUE(core.TakeStopRequest());
  events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE"}));
  EXPECT_FALSE(events[0].details.contains("sub_state"));
  EXPECT_TRUE(core.TakeNotices().empty());
}

TEST(LogicSetup, RefusedOnTheStationInAnEmergencyWithAHoldAndWhenBusy) {
  TestCore core = Ready();
  HardwareSnapshot hw = Healthy();
  hw.dock_contact_raw = true;
  core.OnHardwareStatus(hw);
  EXPECT_EQ(core.OnSetupMotion(true, "wheels", 10.0).code, "ON_DOCK");
  hw.dock_contact_raw = false;
  hw.dock_contact = true;
  core.OnHardwareStatus(hw);
  EXPECT_EQ(core.OnSetupMotion(true, "wheels", 10.0).code, "ON_DOCK");
  hw = Healthy();
  hw.emergency = true;
  hw.emergency_reason = 1u << 2;
  core.OnHardwareStatus(hw);
  EXPECT_EQ(core.OnSetupMotion(true, "wheels", 10.0).code, "EMERGENCY");
  core.OnHardwareStatus(Healthy());
  core.SetHold(2000000000);
  EXPECT_EQ(core.OnSetupMotion(true, "wheels", 10.0).code, "HOLD");
  core.SetHold(0);
  // An end without a beginning does nothing.
  EXPECT_TRUE(core.OnSetupMotion(false, "wheels", 10.0).empty());
  EXPECT_FALSE(core.TakeStopRequest());
}

TEST(LogicSetup, AnEmergencyKeepsSetupButStopsItsCommands) {
  TestCore core = Ready();
  ASSERT_TRUE(core.OnSetupMotion(true, "stop_moving", 10.0).empty());
  HardwareSnapshot hw = Healthy();
  hw.emergency = true;
  hw.emergency_reason = 1u << 2;
  core.OnHardwareStatus(hw);
  EXPECT_TRUE(core.InSetup());
  EXPECT_FALSE(core.SetupActive());
  core.OnHardwareStatus(Healthy());
  EXPECT_TRUE(core.SetupActive());
}

TEST(LogicSetup, EndsWithoutItsMotionOnTheStationAndWithAHold) {
  TestCore core = Ready();
  ASSERT_TRUE(core.OnSetupMotion(true, "wheels", 10.0).empty());
  EXPECT_FALSE(core.TickSetup(10.9));
  EXPECT_TRUE(core.TickSetup(11.05));
  EXPECT_FALSE(core.InSetup());
  EXPECT_TRUE(core.TakeStopRequest());
  const auto notices = core.TakeNotices();
  ASSERT_EQ(notices.size(), 1u);
  EXPECT_NE(notices[0].find("No sign of the setup program"), std::string::npos) << notices[0];

  ASSERT_TRUE(core.OnSetupMotion(true, "wheels", 20.0).empty());
  HardwareSnapshot hw = Healthy();
  hw.dock_contact_raw = true;
  core.OnHardwareStatus(hw);
  EXPECT_TRUE(core.TickSetup(20.1));
  EXPECT_FALSE(core.InSetup());
  core.OnHardwareStatus(Healthy());

  ASSERT_TRUE(core.OnSetupMotion(true, "wheels", 30.0).empty());
  core.SetHold(2000000000);
  EXPECT_TRUE(core.TickSetup(30.1));
  EXPECT_FALSE(core.InSetup());
}

TEST(LogicSetup, AnotherStepBeginsAnew) {
  TestCore core = Ready();
  ASSERT_TRUE(core.OnSetupMotion(true, "wheels", 10.0).empty());
  core.TakeEvents();
  ASSERT_TRUE(core.OnSetupMotion(true, "silence", 10.2).empty());
  EXPECT_TRUE(core.InSetup());
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"STATE", "STATE"}));
  EXPECT_TRUE(core.TakeStopRequest());
}

TEST(LogicSetup, TheBladeTurnsAsTheStepAsksWithoutASafetyMeasurement) {
  TestCore core = Ready();
  EXPECT_FALSE(core.BladeWanted());
  // Outside SETUP the program asks for nothing.
  core.OnSetupBlade("blade", 1);
  EXPECT_FALSE(core.BladeWanted());
  ASSERT_TRUE(core.OnSetupMotion(true, "blade", 10.0).empty());
  EXPECT_FALSE(core.BladeWanted());
  core.OnSetupBlade("other", 1);
  EXPECT_FALSE(core.BladeWanted());
  core.OnSetupBlade("blade", 1);
  EXPECT_TRUE(core.BladeWanted());
  EXPECT_FALSE(core.SetupBladeOnce());
  core.OnSetupBlade("blade", 2);
  EXPECT_TRUE(core.BladeWanted());
  EXPECT_TRUE(core.SetupBladeOnce());
  core.OnSetupBlade("blade", 7);
  EXPECT_FALSE(core.BladeWanted());
  core.OnSetupBlade("blade", 1);
  // An emergency or no telemetry switch it off until the step asks for the blade anew.
  for (int cut = 0; cut < 2; ++cut) {
    HardwareSnapshot hw = Healthy();
    hw.emergency = cut == 0;
    hw.emergency_reason = cut == 0 ? 1u << 2 : 0;
    hw.telemetry_ok = cut != 0;
    core.OnHardwareStatus(hw);
    EXPECT_FALSE(core.BladeWanted());
    core.OnHardwareStatus(Healthy());
    EXPECT_FALSE(core.BladeWanted()) << cut;
    core.OnSetupBlade("blade", 0);
    core.OnSetupBlade("blade", 1);
    EXPECT_TRUE(core.BladeWanted()) << cut;
  }
  // The station switches it off while it lasts.
  HardwareSnapshot hw = Healthy();
  hw.dock_contact = true;
  core.OnHardwareStatus(hw);
  EXPECT_FALSE(core.BladeWanted());
}

TEST(LogicSetup, TheBladeGoesOffWithTheEndOfSetup) {
  for (int end = 0; end < 3; ++end) {
    TestCore core = Ready();
    ASSERT_TRUE(core.OnSetupMotion(true, "blade", 10.0).empty());
    core.OnSetupBlade("blade", 1);
    ASSERT_TRUE(core.BladeWanted());
    if (end == 0) {
      core.OnSetupMotion(false, "blade", 10.1);
    } else if (end == 1) {
      EXPECT_TRUE(core.TickSetup(11.05));
    } else {
      // Another step begins anew, without the blade of the first.
      EXPECT_TRUE(core.OnSetupMotion(true, "ground", 10.1).empty());
    }
    EXPECT_FALSE(core.BladeWanted()) << end;
    EXPECT_FALSE(core.SetupBladeOnce()) << end;
    // A blade asked for later in IDLE turns nothing.
    core.OnSetupBlade("blade", 1);
    EXPECT_FALSE(core.BladeWanted()) << end;
  }
}

TEST(LogicSetup, ARefusedBladeStaysOffUntilTheStepAsksAnew) {
  TestCore core = Ready();
  ASSERT_TRUE(core.OnSetupMotion(true, "blade", 10.0).empty());
  core.OnSetupBlade("blade", 1);
  core.TakeNotices();
  core.OnBladeRefused("not allowed", 0);
  EXPECT_FALSE(core.BladeWanted());
  // No pause and no emergency of mowing: the step sees the blade stand.
  EXPECT_TRUE(core.InSetup());
  EXPECT_TRUE(core.SetupActive());
  const auto notices = core.TakeNotices();
  ASSERT_EQ(notices.size(), 1u);
  EXPECT_NE(notices[0].find("the blade was refused"), std::string::npos) << notices[0];
  core.OnSetupBlade("blade", 1);
  EXPECT_FALSE(core.BladeWanted());
  core.OnSetupBlade("blade", 0);
  core.OnSetupBlade("blade", 1);
  EXPECT_TRUE(core.BladeWanted());
}

// The disk failed while a setup step ran: SETUP ends, no refusal of another kind.
TEST(LogicSetup, AFailedRecordEndsTheStepAlone) {
  TestCore core = Ready();
  ASSERT_TRUE(core.OnSetupMotion(true, "wheels", 10.0).empty());
  core.TakeEvents();
  core.AbandonStart("os-session.json");
  EXPECT_FALSE(core.InSetup());
  EXPECT_TRUE(core.Idle());
  for (const auto& e : core.TakeEvents()) EXPECT_NE(e.type, "RECORDING_REFUSED");
}
