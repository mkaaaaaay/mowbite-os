#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

const DockPose kDock{-1.0, 0.0, M_PI};
const Ring kLawn{{-0.5, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.5, 2.0}};

HardwareSnapshot Healthy() {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  return hw;
}

PoseSnapshot Pose() {
  PoseSnapshot pose;
  pose.x = 2.0;
  pose.y = 0.5;
  pose.heading = M_PI;
  pose.position_accuracy = 0.02f;
  pose.orientation_accuracy = 0.01f;
  pose.orientation_valid = true;
  return pose;
}

// IDLE in the field with everything a start needs; held when the start found a restart hold.
TestCore Started(bool held) {
  LogicConfig config;
  config.dock_contact_x = -0.10;
  TestCore core{config};
  core.SetRestartHold(held);
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(Pose());
  core.OnMowMap({{"lawn", "Lawn", true}}, Reason());
  core.OnMapDock(kDock);
  core.OnDrivableAreas({kLawn}, {});
  core.TakeEvents();
  core.TakeActionsChanged();
  return core;
}

RestartReport Report(const std::string& cause, bool hold) {
  RestartReport report;
  report.cause = cause;
  report.detail = "what the start found";
  report.state_before = hold ? "MOWING" : "IDLE";
  report.hold = hold;
  return report;
}

}  // namespace

TEST(RestartHold, WithoutItTheMowerStarts) {
  TestCore core = Started(false);
  EXPECT_FALSE(core.Emergency());
  EXPECT_TRUE(core.StartProblem().empty()) << core.StartProblem().message;
}

TEST(RestartHold, RefusesAllThatDrivesOrMowsWhateverTheDriverSays) {
  TestCore core = Started(true);
  EXPECT_TRUE(core.Emergency());
  const RobotStateData state = core.RobotState();
  EXPECT_TRUE(state.emergency);
  EXPECT_EQ(state.emergency_reason, kEmergencyUnexpectedRestart);
  // The first report of the driver only set the state.
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_EQ(core.StartProblem().code, "EMERGENCY");
  EXPECT_EQ(core.Command("mowing.start", json::object()).why.code, "EMERGENCY");
  EXPECT_EQ(core.Command("docking.start", json::object()).why.code, "EMERGENCY");
  EXPECT_EQ(core.Command("recording.start", json::object()).why.code, "EMERGENCY");
  EXPECT_EQ(core.StartDrive(1.0, 0.0).code, "EMERGENCY");
  EXPECT_EQ(core.StartCalibration(0.0).code, "EMERGENCY");
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0, 10.0).has_value());
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_EQ(core.RobotState().state, "IDLE");
}

TEST(RestartHold, ShowsItsBitBesideTheDriversBits) {
  TestCore core = Started(true);
  HardwareSnapshot hw = Healthy();
  hw.emergency = true;
  hw.emergency_reason = kEmergencyLatch | kEmergencyUnexpectedRestart;
  core.OnHardwareStatus(hw);
  EXPECT_EQ(core.RobotState().emergency_reason, kEmergencyLatch | kEmergencyUnexpectedRestart);
  core.OnHardwareTimeout();
  EXPECT_EQ(core.RobotState().emergency_reason,
            kEmergencyLatch | kEmergencyTimeoutInputs | kEmergencyUnexpectedRestart);
}

TEST(RestartHold, AResetLetsGoAndTellsWhoReleased) {
  TestCore core = Started(true);
  const CommandOutcome reset = core.Command("emergency.reset", {{"client", "app"}});
  EXPECT_TRUE(reset.release);
  core.SetRestartHold(false);
  EXPECT_FALSE(core.Emergency());
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "EMERGENCY");
  EXPECT_EQ(events[0].details["emergency"], false);
  EXPECT_EQ(events[0].details["source"], "rpc");
  EXPECT_EQ(events[0].details["client"], "app");
  EXPECT_TRUE(core.StartProblem().empty()) << core.StartProblem().message;
}

TEST(RestartHold, TheDriverKeepsItsOwnLatchAfterTheRelease) {
  TestCore core = Started(true);
  HardwareSnapshot hw = Healthy();
  hw.emergency = true;
  hw.emergency_reason = kEmergencyLatch | kEmergencyUnexpectedRestart;
  core.OnHardwareStatus(hw);
  core.SetRestartHold(false);
  EXPECT_TRUE(core.Emergency());
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.Emergency());
}

TEST(RestartHold, TheEventIsRatedByHold) {
  TestCore core = Started(true);
  core.ReportRestart(Report("OS_CRASH", true));
  core.ReportRestart(Report("OS_CRASH", false));
  RestartReport kernel = Report("KERNEL_CRASH", true);
  kernel.crashed_at = 1800000100;
  kernel.job_id_before = "J1";
  core.ReportRestart(kernel);
  core.ReportRestart(Report("POWER_LOSS", false));
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 4u);
  const char* severity[] = {"error", "warning", "error", "warning"};
  const char* resume[] = {"user", "auto", "user", "auto"};
  for (size_t i = 0; i < events.size(); i++) {
    const json& d = events[i].details;
    EXPECT_EQ(events[i].type, "UNEXPECTED_RESTART");
    EXPECT_EQ(d["reason"], d["cause"]);
    EXPECT_EQ(d["severity"], severity[i]) << i;
    EXPECT_EQ(d["resume"], resume[i]) << i;
    EXPECT_TRUE(mowbite_api::CheckEvent("UNEXPECTED_RESTART", d).empty())
        << json(mowbite_api::CheckEvent("UNEXPECTED_RESTART", d)).dump();
  }
  EXPECT_TRUE(events[0].details["crashed_at"].is_null());
  EXPECT_TRUE(events[0].details["job_id_before"].is_null());
  EXPECT_EQ(events[2].details["crashed_at"], 1800000100);
  EXPECT_EQ(events[2].details["job_id_before"], "J1");
  EXPECT_EQ(events[2].details["state_before"], "MOWING");
}

TEST(RestartHold, AnEndByTheWatchNamesTheStalledNode) {
  TestCore core = Started(true);
  const json stalled = {{"node", "mowbite_bridge"}, {"seconds", 10.2}, {"limit", 10.0}};
  for (bool hold : {true, false}) {
    RestartReport report = Report("OS_CRASH", hold);
    report.stalled = stalled;
    core.ReportRestart(report);
  }
  core.ReportRestart(Report("OS_CRASH", true));
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 3u);
  const char* severity[] = {"error", "warning", "error"};
  const char* resume[] = {"user", "auto", "user"};
  for (size_t i = 0; i < events.size(); i++) {
    const json& d = events[i].details;
    // The hold judges by cause, which stays; the reason tells a stall apart.
    EXPECT_EQ(d["cause"], "OS_CRASH");
    EXPECT_EQ(d["reason"], i < 2 ? "NODE_STALLED" : "OS_CRASH");
    EXPECT_EQ(d["stalled"], i < 2 ? stalled : json()) << i;
    EXPECT_EQ(d["severity"], severity[i]) << i;
    EXPECT_EQ(d["resume"], resume[i]) << i;
    EXPECT_TRUE(mowbite_api::CheckEvent("UNEXPECTED_RESTART", d).empty())
        << json(mowbite_api::CheckEvent("UNEXPECTED_RESTART", d)).dump();
  }
  json odd = events[0].details;
  odd["stalled"] = {{"node", "mowbite_bridge"}};
  EXPECT_FALSE(mowbite_api::CheckEvent("UNEXPECTED_RESTART", odd).empty());
}
