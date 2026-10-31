#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mowbite_logic/logic_core.h"

using namespace mowbite_logic;

namespace {

const std::string kStart = "mower_logic:idle/start_mowing";
const std::string kRec = "mower_logic:area_recording/";

// The station of the map: the mower backs in going north, so on the contacts it faces south.
const DockPose kDock{-1.0, 2.0, M_PI / 2};

HardwareSnapshot Healthy(bool contact) {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  hw.dock_contact = contact;
  return hw;
}

PoseSnapshot PoseAt(double x, double y, float accuracy = 0.02f, bool heading = true) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.position_accuracy = accuracy;
  pose.orientation_accuracy = heading ? 0.01f : 999.0f;
  pose.orientation_valid = heading;
  return pose;
}

LogicConfig Tron() {
  LogicConfig config;
  config.dock_contact_x = -0.10;
  return config;
}

// On the contacts of the recorded station, the heading of the localization not aligned yet, as after a start there.
LogicCore Docked(LogicConfig config = Tron()) {
  LogicCore core{config};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy(true));
  core.OnCapabilities(true);
  core.OnPose(PoseAt(kDock.x, kDock.y, 0.02f, false));
  core.OnMowMap({{"front", "Front", true}}, Reason());
  core.OnMapDock(kDock);
  core.TakeEvents();
  core.TakeNotices();
  core.TakeActionsChanged();
  return core;
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

const Event* Find(const std::vector<Event>& events, const std::string& type) {
  auto it = std::find_if(events.begin(), events.end(), [&](const Event& e) { return e.type == type; });
  return it == events.end() ? nullptr : &*it;
}

// start_mowing on the station: the check is asked for with the heading of the mower there.
void StartChecking(LogicCore& core) {
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_TRUE(core.DockCheckPending());
}

}  // namespace

TEST(DockStart, OnTheContactsNoHeadingIsNeededButAStricterPosition) {
  LogicCore core = Docked();
  EXPECT_TRUE(core.StartProblem().empty()) << core.StartProblem().code;
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "start_mowing"));
  core.OnPose(PoseAt(kDock.x, kDock.y, 0.05f, false));
  const Reason why = core.StartProblem();
  EXPECT_EQ(why.code, "POSITION_ACCURACY");
  EXPECT_DOUBLE_EQ(why.fields["limit"].get<double>(), 0.03);
  EXPECT_NE(why.message.find("on the docking station"), std::string::npos);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_mowing"));
  // A limit of params.set below the one of the station holds there as well.
  core.SetGpsSettings(0.02, 10.0);
  core.OnPose(PoseAt(kDock.x, kDock.y, 0.025f, false));
  EXPECT_DOUBLE_EQ(core.StartProblem().fields["limit"].get<double>(), 0.02);
}

TEST(DockStart, WithoutTheStationInTheMapNoStartAndNoDrive) {
  LogicCore core = Docked();
  core.OnMapDock(std::nullopt);
  EXPECT_EQ(core.StartProblem().code, "NO_DOCK");
  EXPECT_EQ(core.DriveProblem().code, "NO_DOCK");
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kRejected);
  EXPECT_FALSE(core.DockCheckPending());
  const auto events = core.TakeEvents();
  const Event* refused = Find(events, "MOWING_REFUSED");
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->details["reason"], "NO_DOCK");
  EXPECT_EQ(refused->details["severity"], "warning");
  EXPECT_EQ(refused->details["resume"], "user");
}

TEST(DockStart, TheCheckComesFirstAndTheMowerStaysIdle) {
  LogicCore core = Docked();
  StartChecking(core);
  EXPECT_FALSE(core.Mowing());
  EXPECT_EQ(core.RobotState().state, "IDLE");
  EXPECT_FALSE(core.FollowerActive());
  // Backing in going north, the mower faces south on the contacts.
  const auto heading = core.TakeDockCheck();
  ASSERT_TRUE(heading);
  EXPECT_NEAR(*heading, -M_PI / 2, 1e-12);
  EXPECT_FALSE(core.TakeDockCheck());
  // A second start while it runs.
  EXPECT_EQ(core.StartProblem().code, "BUSY");
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_mowing"));
  core.TakeEvents();
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kRejected);
  const auto events = core.TakeEvents();
  const Event* refused = Find(events, "MOWING_REFUSED");
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->details["state"], "checking");
}

TEST(DockStart, FarFromTheRecordedStationIsAReferenceMismatch) {
  LogicCore core = Docked();
  StartChecking(core);
  core.TakeDockCheck();
  core.TakeEvents();
  const Reason why = core.OnDockCheck(true, kDock.x + 0.12, kDock.y + 0.16, 0.02f, "");
  EXPECT_EQ(why.code, "REFERENCE_MISMATCH");
  EXPECT_FALSE(core.DockCheckPending());
  EXPECT_FALSE(core.Mowing());
  EXPECT_FALSE(core.TakeStopRequest());
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "MOWING_REFUSED");
  EXPECT_EQ(events[0].details["reason"], "REFERENCE_MISMATCH");
  EXPECT_NEAR(events[0].details["distance"].get<double>(), 0.2, 1e-9);
  EXPECT_DOUBLE_EQ(events[0].details["limit"].get<double>(), 0.1);
  EXPECT_EQ(events[0].details["severity"], "warning");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_NE(events[0].details["message"].get<std::string>().find("0.20 m"), std::string::npos);
  // The next attempt checks again.
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "start_mowing"));
  StartChecking(core);
  EXPECT_TRUE(core.TakeDockCheck());
}

TEST(DockStart, ACheckThatPassesEndsOnTheStationUntilUndockingIsBuilt) {
  LogicCore core = Docked();
  StartChecking(core);
  core.TakeDockCheck();
  core.TakeNotices();
  const Reason why = core.OnDockCheck(true, kDock.x + 0.03, kDock.y, 0.022f, "");
  EXPECT_EQ(why.code, "ON_DOCK");
  EXPECT_FALSE(core.Mowing());
  const auto events = core.TakeEvents();
  const Event* refused = Find(events, "MOWING_REFUSED");
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->details["reason"], "ON_DOCK");
  EXPECT_NEAR(refused->details["distance"].get<double>(), 0.03, 1e-9);
  EXPECT_EQ(refused->details["severity"], "info");
  const auto notices = core.TakeNotices();
  EXPECT_NE(std::find(notices.begin(), notices.end(),
                      "Reference check on the docking station passed: 0.03 m from the recorded position"),
            notices.end());
}

TEST(DockStart, WithoutAFixOrWithAPoorOneTheStartIsRefused) {
  LogicCore core = Docked();
  StartChecking(core);
  core.TakeEvents();
  EXPECT_EQ(core.OnDockCheck(false, 0, 0, 999.0f, "No recent RTK fixed position").code, "NO_POSITION");
  const auto events = core.TakeEvents();
  const Event* refused = Find(events, "MOWING_REFUSED");
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->details["message"], "No recent RTK fixed position");
  StartChecking(core);
  const Reason why = core.OnDockCheck(true, kDock.x, kDock.y, 0.04f, "");
  EXPECT_EQ(why.code, "POSITION_ACCURACY");
  EXPECT_DOUBLE_EQ(why.fields["limit"].get<double>(), 0.03);
  EXPECT_FALSE(core.Mowing());
}

TEST(DockStart, WhatChangedDuringTheCheckComesFirst) {
  LogicCore core = Docked();
  StartChecking(core);
  HardwareSnapshot hw = Healthy(true);
  hw.emergency = true;
  hw.emergency_reason = kEmergencyLatch | kEmergencyStop;
  core.OnHardwareStatus(hw);
  EXPECT_EQ(core.OnDockCheck(true, kDock.x, kDock.y, 0.02f, "").code, "EMERGENCY");

  LogicCore lifted = Docked();
  StartChecking(lifted);
  lifted.OnHardwareStatus(Healthy(false));
  EXPECT_EQ(lifted.OnDockCheck(true, kDock.x, kDock.y, 0.02f, "").code, "NO_DOCK_CONTACT");
  EXPECT_FALSE(lifted.Mowing());
}

TEST(DockStart, ContactsInFrontFaceTheWayIn) {
  LogicConfig config;
  config.dock_contact_x = 0.3;
  LogicCore core = Docked(config);
  StartChecking(core);
  EXPECT_NEAR(core.TakeDockCheck().value_or(99), M_PI / 2, 1e-12);
}

TEST(DockStart, OffTheStationNoCheck) {
  LogicCore core = Docked();
  core.OnHardwareStatus(Healthy(false));
  core.OnPose(PoseAt(0, 0));
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_TRUE(core.Mowing());
  EXPECT_FALSE(core.TakeDockCheck());
  EXPECT_FALSE(core.DockCheckPending());
}

TEST(DockStart, AnAnswerWithoutACheckChangesNothing) {
  LogicCore core = Docked();
  EXPECT_EQ(core.OnDockCheck(true, kDock.x, kDock.y, 0.02f, "").code, "STOPPED");
  EXPECT_TRUE(core.TakeEvents().empty());
}

TEST(DockDrive, ADriveFromTheStationIsCheckedFirst) {
  LogicCore core = Docked();
  ASSERT_TRUE(core.DriveProblem().empty()) << core.DriveProblem().code;
  core.CheckDockBeforeDrive();
  EXPECT_FALSE(core.Driving());
  EXPECT_EQ(core.DriveProblem().code, "BUSY");
  EXPECT_NEAR(core.TakeDockCheck().value_or(99), -M_PI / 2, 1e-12);
  core.TakeEvents();
  EXPECT_EQ(core.OnDockCheck(true, kDock.x - 0.3, kDock.y, 0.02f, "").code, "REFERENCE_MISMATCH");
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "DRIVE_REFUSED");
  EXPECT_EQ(events[0].details["reason"], "REFERENCE_MISMATCH");
  EXPECT_EQ(events[0].details["severity"], "warning");
  EXPECT_FALSE(core.Driving());
  EXPECT_TRUE(core.DriveProblem().empty());
}

TEST(DockRecording, ContactAndHeadingComeFromTheCore) {
  LogicCore core = Docked();
  core.OnHardwareStatus(Healthy(false));
  core.OnMapStatus(MapDatum::kMatch, "");
  ASSERT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kDone);
  core.OnPose(PoseAt(-1.0, 1.0));
  ASSERT_EQ(core.OnAction(kRec + "record_dock"), ActionResult::kDone);
  core.TakeNotices();
  core.OnPose(PoseAt(-1.0, 2.0));
  EXPECT_EQ(core.OnAction(kRec + "record_dock"), ActionResult::kRejected);
  const auto events = core.TakeEvents();
  const Event* refused = Find(events, "RECORDING_REFUSED");
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->details["reason"], "NO_DOCK_CONTACT");
  EXPECT_NE(refused->details["message"].get<std::string>().find("backwards"), std::string::npos);
  // Without telemetry the contact counts as gone.
  core.OnHardwareStatus(Healthy(true));
  core.OnHardwareTimeout();
  EXPECT_EQ(core.OnAction(kRec + "record_dock"), ActionResult::kRejected);
  core.OnHardwareStatus(Healthy(true));
  // Without a heading the point would be off by the antenna offset.
  core.OnPose(PoseAt(-1.0, 2.0, 0.02f, false));
  EXPECT_EQ(core.OnAction(kRec + "record_dock"), ActionResult::kRejected);
  core.OnPose(PoseAt(-1.0, 2.0));
  ASSERT_EQ(core.OnAction(kRec + "record_dock"), ActionResult::kDone);
  const auto edit = core.TakeMapEdit();
  ASSERT_TRUE(edit);
  EXPECT_NEAR(edit->heading, M_PI / 2, 1e-9);
}
