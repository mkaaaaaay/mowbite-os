#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

const std::string kStart = "mower_logic:idle/start_mowing";
const std::string kRecord = "mower_logic:idle/start_area_recording";
const std::string kPause = "mower_logic:mowing/pause";
const std::string kContinue = "mower_logic:mowing/continue";
const std::string kAbort = "mower_logic:mowing/abort_mowing";
const std::string kReset = "mower_logic/reset_emergency";
constexpr int64_t kUntil = 1800000900;

const DockPose kDock{-1.0, 0.0, M_PI};
const Ring kLawn{{-0.5, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.5, 2.0}};

HardwareSnapshot Healthy(bool contact = false) {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  hw.dock_contact = contact;
  hw.dock_contact_raw = contact;
  return hw;
}

PoseSnapshot PoseAt(double x, double y, double heading = 0.0) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.heading = heading;
  pose.position_accuracy = 0.02f;
  pose.orientation_accuracy = 0.01f;
  pose.orientation_valid = true;
  return pose;
}

LogicConfig Tron() {
  LogicConfig config;
  config.dock_contact_x = -0.10;
  return config;
}

// IDLE in the field with a station and a lawn: everything may start.
TestCore Ready() {
  TestCore core{Tron()};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(PoseAt(2.0, 0.5, M_PI));
  core.OnMowMap({{"lawn", "Lawn", true}}, Reason());
  core.OnMapDock(kDock);
  core.OnDrivableAreas({kLawn}, {});
  core.TakeEvents();
  core.TakeNotices();
  core.TakeActionsChanged();
  return core;
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

const Event* Find(const std::vector<Event>& events, const std::string& type) {
  auto it = std::find_if(events.begin(), events.end(), [&](const Event& e) { return e.type == type; });
  return it == events.end() ? nullptr : &*it;
}

void ExpectHold(const Event* event, const std::string& type) {
  ASSERT_NE(event, nullptr) << type;
  EXPECT_EQ(event->details["reason"], "HOLD");
  EXPECT_EQ(event->details["until"], kUntil);
  EXPECT_EQ(event->details["severity"], "info");
  EXPECT_EQ(event->details["resume"], "user");
  EXPECT_FALSE(event->details.value("message", "").empty());
}

}  // namespace

TEST(LogicHold, AllowedOnlyInIdleWithNothingOnItsWay) {
  TestCore core = Ready();
  EXPECT_TRUE(core.HoldAllowed());
  ASSERT_TRUE(core.StartDrive(3.0, 0.5).empty());
  EXPECT_FALSE(core.HoldAllowed());
  core.OnDriveEnded(true, Reason("DONE", ""));
  EXPECT_TRUE(core.HoldAllowed());
  ASSERT_EQ(core.OnAction(kRecord), ActionResult::kDone);
  EXPECT_FALSE(core.HoldAllowed());
}

TEST(LogicHold, NotAllowedWhileMowing) {
  TestCore core = Ready();
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_FALSE(core.HoldAllowed());
  ASSERT_EQ(core.OnAction(kPause), ActionResult::kDone);
  EXPECT_FALSE(core.HoldAllowed());
}

TEST(LogicHold, NotAllowedDuringTheReferenceCheck) {
  TestCore core = Ready();
  core.OnHardwareStatus(Healthy(true));
  core.OnPose(PoseAt(kDock.x, kDock.y, 0.0));
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_TRUE(core.DockCheckPending());
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.HoldAllowed());
}

TEST(LogicHold, StartMowingIsRefused) {
  TestCore core = Ready();
  core.SetHold(kUntil);
  EXPECT_TRUE(core.Held());
  EXPECT_EQ(core.StartProblem().code, "HOLD");
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_mowing"));
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kRejected);
  EXPECT_EQ(core.RobotState().state, "IDLE");
  ExpectHold(Find(core.TakeEvents(), "MOWING_REFUSED"), "MOWING_REFUSED");
}

TEST(LogicHold, RecordingIsRefused) {
  TestCore core = Ready();
  core.SetHold(kUntil);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  EXPECT_EQ(core.OnAction(kRecord), ActionResult::kRejected);
  EXPECT_EQ(core.RobotState().state, "IDLE");
  const auto events = core.TakeEvents();
  const Event* refused = Find(events, "RECORDING_REFUSED");
  ExpectHold(refused, "RECORDING_REFUSED");
  EXPECT_EQ(refused->details["action"], "start_area_recording");
  // No remote control without recording.
  EXPECT_FALSE(core.OnTeleop(0.2, 0.0, 10.0));
}

TEST(LogicHold, DriveIsRefused) {
  TestCore core = Ready();
  core.SetHold(kUntil);
  EXPECT_EQ(core.DriveProblem().code, "HOLD");
  const Reason why = core.StartDrive(3.0, 0.5);
  EXPECT_EQ(why.code, "HOLD");
  EXPECT_EQ(why.fields["until"], kUntil);
  EXPECT_FALSE(core.Driving());
  ExpectHold(Find(core.TakeEvents(), "DRIVE_REFUSED"), "DRIVE_REFUSED");
}

TEST(LogicHold, DockingIsRefused) {
  TestCore core = Ready();
  ASSERT_TRUE(core.DockingProblem().empty()) << core.DockingProblem().code;
  core.SetHold(kUntil);
  EXPECT_EQ(core.DockingProblem().code, "HOLD");
  EXPECT_EQ(core.StartDocking().code, "HOLD");
  EXPECT_FALSE(core.Docking());
  ExpectHold(Find(core.TakeEvents(), "DRIVE_REFUSED"), "DRIVE_REFUSED");
}

TEST(LogicHold, ContinueIsRefused) {
  // Cannot happen through system.hold (only in IDLE); the core refuses it all the same.
  TestCore core = Ready();
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_EQ(core.OnAction(kPause), ActionResult::kDone);
  ASSERT_TRUE(Enabled(core, "mower_logic:mowing", "continue"));
  core.SetHold(kUntil);
  core.TakeEvents();
  EXPECT_FALSE(Enabled(core, "mower_logic:mowing", "continue"));
  EXPECT_EQ(core.OnAction(kContinue), ActionResult::kRejected);
  ExpectHold(Find(core.TakeEvents(), "MOWING_REFUSED"), "MOWING_REFUSED");
  // Stopping still works.
  EXPECT_EQ(core.OnAction(kAbort), ActionResult::kDone);
}

TEST(LogicHold, AReferenceCheckEndsInTheHold) {
  TestCore core = Ready();
  core.OnHardwareStatus(Healthy(true));
  core.OnPose(PoseAt(kDock.x, kDock.y, 0.0));
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_TRUE(core.TakeDockCheck());
  core.SetHold(kUntil);
  const Reason why = core.OnDockCheck(true, kDock.x + 0.02, kDock.y, 0.02f, "");
  EXPECT_EQ(why.code, "HOLD");
  EXPECT_FALSE(core.Undocking());
  EXPECT_FALSE(core.TakeUndockPath());
}

TEST(LogicHold, ResetOfTheEmergencyStays) {
  TestCore core = Ready();
  core.SetHold(kUntil);
  EXPECT_EQ(core.OnAction(kReset), ActionResult::kReleaseEmergency);
}

TEST(LogicHold, EndsAndEverythingStartsAgain) {
  TestCore core = Ready();
  core.SetHold(kUntil);
  EXPECT_TRUE(core.TakeActionsChanged());
  core.SetHold(0);
  EXPECT_TRUE(core.TakeActionsChanged());
  EXPECT_FALSE(core.Held());
  EXPECT_TRUE(core.HoldProblem().empty());
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "start_mowing"));
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
}

TEST(LogicHold, TheSameHoldChangesNoAction) {
  TestCore core = Ready();
  core.SetHold(kUntil);
  core.TakeActionsChanged();
  core.SetHold(kUntil);
  EXPECT_FALSE(core.TakeActionsChanged());
}

TEST(LogicHold, AHoldOfTheDriverAloneStopsStartsToo) {
  TestCore core = Ready();
  HardwareSnapshot hw = Healthy();
  hw.hold = true;
  core.OnHardwareStatus(hw);
  EXPECT_TRUE(core.TakeActionsChanged());
  EXPECT_TRUE(core.Held());
  EXPECT_EQ(core.HoldUntil(), 0);
  const Reason why = core.StartProblem();
  EXPECT_EQ(why.code, "HOLD");
  EXPECT_FALSE(why.fields.contains("until"));
  EXPECT_EQ(core.DriveProblem().code, "HOLD");
  EXPECT_EQ(core.DockingProblem().code, "HOLD");
  EXPECT_EQ(core.OnAction(kRecord), ActionResult::kRejected);
  // Without an emergency.
  EXPECT_FALSE(core.RobotState().emergency);
  core.OnHardwareStatus(Healthy());
  EXPECT_TRUE(core.TakeActionsChanged());
  EXPECT_FALSE(core.Held());
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
}

// A way back to an older backup whose map changed since the update: nothing mows by itself until the user confirmed
// it, drives and recording stay; without confirm only the event tells it.
TEST(LogicDataRollback, MowingWaitsForTheConfirmation) {
  TestCore core = Ready();
  core.SetDataRollback({{"from", 3}, {"to", 2}, {"backup", "backup-3"}, {"backup_of", 1767225600}, {"confirm", true}});
  auto events = core.TakeEvents();
  const Event* restored = Find(events, "DATA_ROLLBACK");
  ASSERT_NE(restored, nullptr);
  EXPECT_EQ(restored->details["what"], "restored");
  EXPECT_EQ(restored->details["backup_of"], 1767225600);
  EXPECT_EQ(restored->details["confirm"], true);
  EXPECT_TRUE(core.TakeActionsChanged());
  const Reason why = core.StartProblem();
  EXPECT_EQ(why.code, "DATA_ROLLBACK");
  EXPECT_EQ(why.fields["backup_of"], 1767225600);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_mowing"));
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kRejected);
  const Event* refused = Find(core.TakeEvents(), "MOWING_REFUSED");
  ASSERT_NE(refused, nullptr);
  EXPECT_EQ(refused->details["reason"], "DATA_ROLLBACK");
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  EXPECT_TRUE(core.DriveProblem().empty());
  core.SetDataRollback(nullptr);
  events = core.TakeEvents();
  const Event* confirmed = Find(events, "DATA_ROLLBACK");
  ASSERT_NE(confirmed, nullptr);
  EXPECT_EQ(confirmed->details["what"], "confirmed");
  EXPECT_TRUE(core.StartProblem().empty());
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "start_mowing"));
}

TEST(LogicDataRollback, ANoteWithoutConfirmLocksNothing) {
  TestCore core = Ready();
  core.SetDataRollback({{"from", 3}, {"to", 2}, {"backup", "backup-3"}, {"backup_of", 1767225600}, {"confirm", false}});
  ASSERT_NE(Find(core.TakeEvents(), "DATA_ROLLBACK"), nullptr);
  EXPECT_TRUE(core.StartProblem().empty());
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
}
