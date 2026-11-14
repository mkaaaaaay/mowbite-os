#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

const std::string kStart = "mower_logic:idle/start_mowing";
const std::string kAbort = "mower_logic:undocking/abort_undocking";

// The station of the simulation: backing in going west, the mower faces east on the contacts.
const DockPose kDock{-1.0, 0.0, M_PI};
// A lawn that reaches to 0.5 m in front of the station, and one that begins 1 m in front of it.
const Ring kLawn{{-0.5, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.5, 2.0}};
const Ring kFarLawn{{0.0, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {0.0, 2.0}};
// Where the reference check puts base_link: 2 cm in front of the recorded station.
const double kStartX = kDock.x + 0.02;

double Rad(double degrees) {
  return degrees * M_PI / 180.0;
}

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

// Contacts in front: on the same station the mower faces west, the way in.
LogicConfig Front() {
  LogicConfig config;
  config.dock_contact_x = 0.3;
  return config;
}

// On the contacts of the recorded station, as after a start there.
TestCore Docked(const Ring& lawn = kLawn, LogicConfig config = Tron()) {
  TestCore core{config};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy(true));
  core.OnCapabilities(true);
  core.OnPose(PoseAt(kDock.x, kDock.y, 0.02f, false));
  core.OnMowMap({{"lawn", "Lawn", true}}, Reason());
  core.OnMapDock(kDock);
  core.OnDrivableAreas({lawn}, {});
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

// start_mowing on the station and a reference check that passes.
void StartUndocking(TestCore& core) {
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_TRUE(core.TakeDockCheck());
  const Reason why = core.OnDockCheck(true, kStartX, kDock.y, 0.02f, "");
  ASSERT_TRUE(why.empty()) << why.code << ": " << why.message;
}

// Charging is off: the way out goes to the follower.
UndockPath ChargedOff(TestCore& core) {
  EXPECT_TRUE(core.TakeChargeOff());
  core.OnChargeOff(true, "charging off");
  const auto path = core.TakeUndockPath();
  EXPECT_TRUE(path);
  return path.value_or(UndockPath());
}

// The mower got out: off the contacts, 0.75 m along the way.
void Out(TestCore& core) {
  core.OnHardwareStatus(Healthy(false));
  core.OnPose(PoseAt(kStartX + 0.75, kDock.y));
}

}  // namespace

TEST(UndockPlan, StraightAheadInPosesAtMostATenthApart) {
  UndockConfig config;
  const UndockPath path = PlanUndock(1.0, 2.0, M_PI / 2, false, config);
  ASSERT_EQ(path.poses.size(), 9u);
  for (size_t i = 0; i < path.poses.size(); i++) {
    EXPECT_NEAR(path.poses[i].x, 1.0, 1e-9);
    EXPECT_NEAR(path.poses[i].y, 2.0 + 0.1 * i, 1e-9);
    EXPECT_DOUBLE_EQ(path.poses[i].heading, M_PI / 2);
  }
  config.distance = 0.15;
  EXPECT_EQ(PlanUndock(0, 0, 0, false, config).poses.size(), 3u);
  config.distance = 0.85;
  const UndockPath odd = PlanUndock(0, 0, 0, false, config);
  EXPECT_EQ(odd.poses.size(), 10u);
  EXPECT_NEAR(odd.poses.back().x, 0.85, 1e-9);
}

TEST(UndockPlan, TheCorridorReachesBehindTheStartAndAlongTheWay) {
  const UndockPath path = PlanUndock(1.0, 2.0, M_PI / 2, false, UndockConfig());
  EXPECT_TRUE(InsideRing(path.corridor, 1.0, 2.0));
  EXPECT_TRUE(InsideRing(path.corridor, 1.0, 1.95));
  EXPECT_TRUE(InsideRing(path.corridor, 1.09, 2.5));
  EXPECT_FALSE(InsideRing(path.corridor, 1.11, 2.5));
  EXPECT_TRUE(InsideRing(path.corridor, 0.95, 2.79));
  EXPECT_FALSE(InsideRing(path.corridor, 1.0, 2.81));
}

TEST(UndockPlan, StraightBackFacingTheWayTheMowerFaces) {
  const UndockPath path = PlanUndock(1.0, 2.0, M_PI / 2, true, UndockConfig());
  EXPECT_TRUE(path.reverse);
  ASSERT_EQ(path.poses.size(), 9u);
  for (size_t i = 0; i < path.poses.size(); i++) {
    EXPECT_NEAR(path.poses[i].x, 1.0, 1e-9);
    EXPECT_NEAR(path.poses[i].y, 2.0 - 0.1 * i, 1e-9);
    EXPECT_DOUBLE_EQ(path.poses[i].heading, M_PI / 2);
  }
  // The corridor reaches behind the start along the way out, which lies behind the mower.
  EXPECT_TRUE(InsideRing(path.corridor, 1.0, 2.05));
  EXPECT_TRUE(InsideRing(path.corridor, 1.0, 1.21));
  EXPECT_FALSE(InsideRing(path.corridor, 1.0, 1.19));
  EXPECT_FALSE(InsideRing(path.corridor, 1.0, 2.11));
  EXPECT_FALSE(PlanUndock(1.0, 2.0, M_PI / 2, false, UndockConfig()).reverse);
}

TEST(UndockPlan, ReachableInAnAreaOutsideOfTheObstacles) {
  const Ring obstacle{{1.0, -0.5}, {2.0, -0.5}, {2.0, 0.5}, {1.0, 0.5}};
  EXPECT_TRUE(Reachable({kLawn}, {}, 0.0, 0.0));
  EXPECT_FALSE(Reachable({kLawn}, {}, -0.6, 0.0));
  EXPECT_FALSE(Reachable({kLawn}, {obstacle}, 1.5, 0.0));
  EXPECT_FALSE(Reachable({}, {}, 0.0, 0.0));
  EXPECT_TRUE(Reachable({kFarLawn, kLawn}, {}, -0.2, 0.0));
}

TEST(UndockPlan, TheCourseCountsOnlyFarEnoughOut) {
  EXPECT_FALSE(CourseError(0, 0, 0, 0.2, 0.1, 0.3));
  EXPECT_NEAR(CourseError(0, 0, 0, 0.3, 0.3, 0.3).value_or(99), M_PI / 4, 1e-12);
  EXPECT_NEAR(CourseError(0, 0, 0, 0.3, -0.3, 0.3).value_or(99), -M_PI / 4, 1e-12);
  // Across the wrap of the angle.
  EXPECT_NEAR(CourseError(0, 0, M_PI, -0.4, -0.01, 0.3).value_or(99), 0.01 / 0.4, 1e-3);
}

TEST(Undock, ACheckThatPassesStartsWithChargingOff) {
  TestCore core = Docked();
  StartUndocking(core);
  EXPECT_TRUE(core.Undocking());
  EXPECT_FALSE(core.Idle());
  EXPECT_FALSE(core.Mowing());
  EXPECT_EQ(core.RobotState().state, "UNDOCKING");
  // Nothing moves before charging is off.
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_FALSE(core.TakeUndockPath());
  EXPECT_TRUE(core.TakeChargeOff());
  EXPECT_FALSE(core.TakeChargeOff());
  const auto events = core.TakeEvents();
  const Event* state = Find(events, "STATE");
  ASSERT_TRUE(state);
  EXPECT_EQ(state->details["state"], "UNDOCKING");
  EXPECT_FALSE(Find(events, "MOWING_REFUSED"));
  EXPECT_TRUE(Enabled(core, "mower_logic:undocking", "abort_undocking"));
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_mowing"));
  EXPECT_EQ(core.StartProblem().code, "BUSY");
  EXPECT_EQ(core.StartProblem().fields["state"], "UNDOCKING");
  EXPECT_FALSE(core.BladeWanted());
}

TEST(Undock, TheWayOutStartsWhereTheMowerStands) {
  TestCore core = Docked();
  StartUndocking(core);
  const UndockPath path = ChargedOff(core);
  ASSERT_EQ(path.poses.size(), 9u);
  EXPECT_NEAR(path.poses.front().x, kStartX, 1e-9);
  EXPECT_NEAR(path.poses.back().x, kStartX + 0.8, 1e-9);
  for (const auto& p : path.poses) {
    EXPECT_NEAR(p.y, 0.0, 1e-9);
    EXPECT_NEAR(std::remainder(p.heading, 2 * M_PI), 0.0, 1e-9);
  }
  EXPECT_DOUBLE_EQ(path.speed, 0.15);
  EXPECT_DOUBLE_EQ(path.stall_time, 2.0);
  EXPECT_TRUE(InsideRing(path.corridor, kStartX, 0.0));
  EXPECT_TRUE(core.FollowerActive());
  EXPECT_FALSE(core.TakeUndockPath());
  EXPECT_FALSE(core.BladeWanted());
}

TEST(Undock, WithTheContactsInFrontTheMowerLeavesBackwards) {
  TestCore core = Docked(kLawn, Front());
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_NEAR(core.TakeDockCheck().value_or(99), M_PI, 1e-12);
  core.TakeNotices();
  const Reason why = core.OnDockCheck(true, kStartX, kDock.y, 0.02f, "");
  ASSERT_TRUE(why.empty()) << why.code << ": " << why.message;
  EXPECT_TRUE(core.Undocking());
  const auto notices = core.TakeNotices();
  EXPECT_TRUE(std::any_of(notices.begin(), notices.end(), [](const std::string& n) {
    return n == "Undocking: charging off, then 0.80 m straight back";
  }));
  const UndockPath path = ChargedOff(core);
  EXPECT_TRUE(path.reverse);
  ASSERT_EQ(path.poses.size(), 9u);
  EXPECT_NEAR(path.poses.front().x, kStartX, 1e-9);
  EXPECT_NEAR(path.poses.back().x, kStartX + 0.8, 1e-9);
  for (const auto& p : path.poses) {
    EXPECT_NEAR(p.y, 0.0, 1e-9);
    EXPECT_NEAR(std::remainder(p.heading - M_PI, 2 * M_PI), 0.0, 1e-9);
  }
  // Backwards at most at the docking speed (G40).
  EXPECT_DOUBLE_EQ(path.speed, 0.1);
  EXPECT_TRUE(InsideRing(path.corridor, kStartX, 0.0));
  Out(core);
  core.OnUndockEnded(LegEnd::kDone, Reason("DONE", "done"));
  EXPECT_TRUE(core.Mowing());
}

TEST(Undock, WithTheContactsInFrontTheCourseProbeLooksAlongTheWayBack) {
  TestCore core = Docked(kLawn, Front());
  StartUndocking(core);
  ChargedOff(core);
  core.TakeEvents();
  // The GNSS antenna 0.1 m ahead of base_link, which faces west; the mower leaves east.
  const double ax = kStartX - 0.1;
  core.OnGnssFix(ax, 0.0);
  core.OnGnssFix(ax + 0.4 * std::cos(Rad(8)), 0.4 * std::sin(Rad(8)));
  EXPECT_TRUE(core.FollowerActive());
  EXPECT_FALSE(core.TakeDriveCancel());
  core.OnGnssFix(ax + 0.45 * std::cos(Rad(-15)), 0.45 * std::sin(Rad(-15)));
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_TRUE(core.TakeDriveCancel());
}

TEST(Undock, WithTheContactsInFrontTheEndOfTheWayBackCounts) {
  TestCore core = Docked(kFarLawn, Front());
  const Reason why = core.StartProblem();
  EXPECT_EQ(why.code, "DOCK_UNREACHABLE");
  EXPECT_NE(why.message.find("0.80 m straight back ends outside"), std::string::npos) << why.message;
  core.SetUndockDistance(1.2);
  EXPECT_TRUE(core.StartProblem().empty()) << core.StartProblem().code;
}

TEST(Undock, UndockedThenMowing) {
  TestCore core = Docked();
  StartUndocking(core);
  ChargedOff(core);
  Out(core);
  core.TakeEvents();
  core.OnUndockEnded(LegEnd::kDone, Reason("DONE", "done"));
  EXPECT_TRUE(core.Mowing());
  const auto events = core.TakeEvents();
  ASSERT_GE(events.size(), 2u);
  EXPECT_EQ(events[0].type, "UNDOCKED");
  EXPECT_FALSE(events[0].details.contains("reason"));
  EXPECT_EQ(events[1].type, "STATE");
  EXPECT_EQ(events[1].details["state"], "MOWING");
  const auto outcome = core.TakeUndockOutcome();
  ASSERT_TRUE(outcome);
  EXPECT_TRUE(outcome->undocked);
  EXPECT_FALSE(outcome->for_drive);
  EXPECT_TRUE(outcome->why.empty());
  EXPECT_FALSE(core.TakeChargeOn());
  EXPECT_FALSE(Enabled(core, "mower_logic:undocking", "abort_undocking"));
  // A late end of the follower changes nothing.
  core.OnUndockEnded(LegEnd::kFailed, Reason("TIMEOUT", "late"));
  EXPECT_TRUE(core.Mowing());
}

TEST(Undock, WhatChangedMeanwhileKeepsMowingFromStarting) {
  TestCore core = Docked();
  StartUndocking(core);
  ChargedOff(core);
  Out(core);
  HardwareSnapshot rain = Healthy(false);
  rain.rain = true;
  core.OnHardwareStatus(rain);
  core.TakeEvents();
  core.OnUndockEnded(LegEnd::kDone, Reason("DONE", "done"));
  EXPECT_FALSE(core.Mowing());
  EXPECT_TRUE(core.Idle());
  const auto events = core.TakeEvents();
  ASSERT_TRUE(Find(events, "UNDOCKED"));
  const Event* refused = Find(events, "MOWING_REFUSED");
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->details["reason"], "RAIN");
  const auto outcome = core.TakeUndockOutcome();
  ASSERT_TRUE(outcome);
  EXPECT_TRUE(outcome->undocked);
  EXPECT_EQ(outcome->why.code, "RAIN");
}

TEST(Undock, ADriveFromTheStationGoesOnAfterUndocking) {
  TestCore core = Docked();
  ASSERT_TRUE(core.DriveProblem().empty()) << core.DriveProblem().code;
  core.CheckDockBeforeDrive(2.0, 1.0);
  ASSERT_TRUE(core.TakeDockCheck());
  ASSERT_TRUE(core.OnDockCheck(true, kStartX, kDock.y, 0.02f, "").empty());
  EXPECT_TRUE(core.UndockingForDrive());
  ChargedOff(core);
  Out(core);
  core.TakeEvents();
  core.OnUndockEnded(LegEnd::kDone, Reason("DONE", "done"));
  EXPECT_TRUE(core.Driving());
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 3u);
  EXPECT_EQ(events[0].type, "UNDOCKED");
  EXPECT_EQ(events[1].details["sub_state"], "DRIVING");
  EXPECT_EQ(events[2].type, "DRIVE");
  EXPECT_EQ(events[2].details["phase"], "start");
  EXPECT_DOUBLE_EQ(events[2].details["goal_x"].get<double>(), 2.0);
  EXPECT_DOUBLE_EQ(events[2].details["goal_y"].get<double>(), 1.0);
  const auto outcome = core.TakeUndockOutcome();
  ASSERT_TRUE(outcome);
  EXPECT_TRUE(outcome->undocked);
  EXPECT_TRUE(outcome->for_drive);
  EXPECT_TRUE(outcome->why.empty());
}

TEST(Undock, AbortWhileChargingOffEndsAtOnce) {
  TestCore core = Docked();
  StartUndocking(core);
  core.TakeChargeOff();
  core.TakeEvents();
  ASSERT_EQ(core.OnAction(kAbort), ActionResult::kDone);
  EXPECT_TRUE(core.Idle());
  EXPECT_TRUE(core.TakeChargeOn());
  const auto events = core.TakeEvents();
  EXPECT_FALSE(Find(events, "UNDOCKING_FAILED"));
  ASSERT_TRUE(Find(events, "STATE"));
  EXPECT_EQ(Find(events, "STATE")->details["state"], "IDLE");
  const auto outcome = core.TakeUndockOutcome();
  ASSERT_TRUE(outcome);
  EXPECT_FALSE(outcome->undocked);
  EXPECT_EQ(outcome->why.message, "Aborted by the app");
  // A late answer of the driver changes nothing.
  core.OnChargeOff(true, "charging off");
  EXPECT_FALSE(core.TakeUndockPath());
  EXPECT_TRUE(core.Idle());
  EXPECT_EQ(core.OnAction(kAbort), ActionResult::kRejected);
}

TEST(Undock, AbortOnTheWayStopsTheFollowerFirst) {
  TestCore core = Docked();
  StartUndocking(core);
  ChargedOff(core);
  core.TakeStopRequest();
  core.TakeEvents();
  ASSERT_EQ(core.OnAction(kAbort), ActionResult::kDone);
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_TRUE(core.TakeStopRequest());
  const auto cancel = core.TakeDriveCancel();
  ASSERT_TRUE(cancel);
  EXPECT_EQ(cancel->code, "STOPPED");
  // Undocking until the follower stopped.
  EXPECT_TRUE(core.Undocking());
  core.OnUndockEnded(LegEnd::kCanceled, Reason("STOPPED", "Aborted by the app (canceled)"));
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(Find(core.TakeEvents(), "UNDOCKING_FAILED"));
  EXPECT_TRUE(core.TakeChargeOn());
}

TEST(Undock, TheCourseProbeStopsAMowerLeavingAskew) {
  TestCore core = Docked();
  // Fixes before undocking do not count.
  core.OnGnssFix(5.0, 5.0);
  StartUndocking(core);
  ChargedOff(core);
  core.TakeEvents();
  // The GNSS antenna 0.1 m ahead of base_link: its first fix is where the way is measured from.
  const double ax = kStartX + 0.1;
  core.OnGnssFix(ax, 0.0);
  // 20 deg off, but only 0.2 m out: not judged yet.
  core.OnGnssFix(ax + 0.2 * std::cos(Rad(20)), 0.2 * std::sin(Rad(20)));
  // 8 deg off at 0.4 m: within the limit.
  core.OnGnssFix(ax + 0.4 * std::cos(Rad(8)), 0.4 * std::sin(Rad(8)));
  // The fused pose does not count, it starts from the heading of the station.
  core.OnPose(PoseAt(kStartX + 0.35 * std::cos(Rad(-30)), 0.35 * std::sin(Rad(-30))));
  EXPECT_TRUE(core.FollowerActive());
  EXPECT_FALSE(core.TakeDriveCancel());
  // 15 deg to the other side at 0.35 m: stop.
  core.OnGnssFix(ax + 0.35 * std::cos(Rad(-15)), 0.35 * std::sin(Rad(-15)));
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_TRUE(core.TakeDriveCancel());
  EXPECT_TRUE(core.TakeStopRequest());
  core.OnUndockEnded(LegEnd::kCanceled, Reason("STOPPED", "canceled"));
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "UNDOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "path_failed");
  EXPECT_EQ(failed->details["severity"], "error");
  EXPECT_EQ(failed->details["resume"], "user");
  EXPECT_NE(failed->details["message"].get<std::string>().find("15 deg off the line of the docking station"),
            std::string::npos)
      << failed->details["message"];
  EXPECT_TRUE(core.Idle());
  EXPECT_TRUE(core.TakeChargeOn());
}

TEST(Undock, StandingInTheHollowFails) {
  TestCore core = Docked();
  StartUndocking(core);
  ChargedOff(core);
  core.TakeEvents();
  core.OnUndockEnded(LegEnd::kFailed,
                     Reason("TIMEOUT", "the mower does not move: 0.00 m in 2.0 s while driven at 0.15 m/s, stuck?"));
  EXPECT_TRUE(core.Idle());
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "UNDOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "path_failed");
  EXPECT_NE(failed->details["message"].get<std::string>().find("does not move"), std::string::npos);
  // On the contacts still: charging goes on.
  EXPECT_TRUE(core.TakeChargeOn());
  const auto outcome = core.TakeUndockOutcome();
  ASSERT_TRUE(outcome);
  EXPECT_FALSE(outcome->undocked);
  EXPECT_EQ(outcome->why.code, "path_failed");
  // The next start checks again.
  EXPECT_TRUE(core.StartProblem().empty()) << core.StartProblem().code;
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_TRUE(core.DockCheckPending());
}

TEST(Undock, ABadPositionIsNoGps) {
  TestCore core = Docked();
  StartUndocking(core);
  ChargedOff(core);
  core.TakeEvents();
  core.OnUndockEnded(LegEnd::kGps, Reason("POSITION_ACCURACY", "position accuracy 0.15 m worse than 0.10 m"));
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "UNDOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "no_gps");

  TestCore lost = Docked();
  StartUndocking(lost);
  ChargedOff(lost);
  lost.TakeEvents();
  lost.OnPoseTimeout();
  EXPECT_FALSE(lost.FollowerActive());
  ASSERT_TRUE(lost.TakeDriveCancel());
  lost.OnUndockEnded(LegEnd::kCanceled, Reason("POSE_LOST", "canceled"));
  const auto lost_events = lost.TakeEvents();
  ASSERT_TRUE(Find(lost_events, "UNDOCKING_FAILED"));
  EXPECT_EQ(Find(lost_events, "UNDOCKING_FAILED")->details["reason"], "no_gps");
}

TEST(Undock, AnEmergencyStopsWithoutUndockingFailed) {
  HardwareSnapshot stop = Healthy(true);
  stop.emergency = true;
  stop.emergency_reason = kEmergencyLatch | kEmergencyStop;

  TestCore core = Docked();
  StartUndocking(core);
  ChargedOff(core);
  core.TakeEvents();
  core.OnHardwareStatus(stop);
  EXPECT_FALSE(core.FollowerActive());
  const auto cancel = core.TakeDriveCancel();
  ASSERT_TRUE(cancel);
  EXPECT_EQ(cancel->code, "EMERGENCY");
  core.OnUndockEnded(LegEnd::kCanceled, Reason("EMERGENCY", "canceled"));
  EXPECT_TRUE(core.Idle());
  auto events = core.TakeEvents();
  EXPECT_TRUE(Find(events, "EMERGENCY"));
  EXPECT_FALSE(Find(events, "UNDOCKING_FAILED"));

  // While charging goes off nothing moves yet: it ends at once.
  TestCore early = Docked();
  StartUndocking(early);
  early.OnHardwareStatus(stop);
  EXPECT_TRUE(early.Idle());
  EXPECT_TRUE(early.TakeChargeOn());
  EXPECT_FALSE(Find(early.TakeEvents(), "UNDOCKING_FAILED"));
}

TEST(Undock, ChargingThatDoesNotGoOffKeepsTheMowerOnTheStation) {
  TestCore core = Docked();
  StartUndocking(core);
  ASSERT_TRUE(core.TakeChargeOff());
  core.TakeEvents();
  core.OnChargeOff(false, "no answer from hw/charge");
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.TakeUndockPath());
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "UNDOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "path_failed");
  EXPECT_NE(failed->details["message"].get<std::string>().find("charging could not be switched off"),
            std::string::npos);
  EXPECT_TRUE(core.TakeChargeOn());
}

TEST(Undock, AStationOutsideOfTheAreasIsUnreachable) {
  TestCore core = Docked(kFarLawn);
  const Reason why = core.StartProblem();
  EXPECT_EQ(why.code, "DOCK_UNREACHABLE");
  EXPECT_DOUBLE_EQ(why.fields["distance"].get<double>(), 0.8);
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_mowing"));
  EXPECT_EQ(core.DriveProblem().code, "DOCK_UNREACHABLE");
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kRejected);
  EXPECT_FALSE(core.DockCheckPending());
  const auto events = core.TakeEvents();
  const Event* refused = Find(events, "MOWING_REFUSED");
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->details["reason"], "DOCK_UNREACHABLE");
  EXPECT_EQ(refused->details["severity"], "warning");
  EXPECT_EQ(refused->details["resume"], "user");
  // A longer way out (params.set) reaches the lawn, an obstacle there does not.
  core.SetUndockDistance(1.2);
  EXPECT_TRUE(core.StartProblem().empty()) << core.StartProblem().code;
  const Ring obstacle{{0.1, -0.2}, {0.3, -0.2}, {0.3, 0.2}, {0.1, 0.2}};
  core.OnDrivableAreas({kFarLawn}, {obstacle});
  EXPECT_EQ(core.StartProblem().code, "DOCK_UNREACHABLE");
  // Off the station it does not matter.
  core.OnHardwareStatus(Healthy(false));
  core.OnPose(PoseAt(1.0, 0.0));
  EXPECT_TRUE(core.StartProblem().empty()) << core.StartProblem().code;
}

TEST(Undock, CheckedAgainFromWhereTheMowerStands) {
  // The recorded station reaches the lawn by 1 cm, the mower stands 5 cm behind it.
  const Ring edge{{-0.21, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.21, 2.0}};
  TestCore core = Docked(edge);
  ASSERT_TRUE(core.StartProblem().empty()) << core.StartProblem().code;
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_TRUE(core.TakeDockCheck());
  core.TakeEvents();
  EXPECT_EQ(core.OnDockCheck(true, kDock.x - 0.05, kDock.y, 0.02f, "").code, "DOCK_UNREACHABLE");
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.TakeChargeOff());
  const auto events = core.TakeEvents();
  const Event* refused = Find(events, "MOWING_REFUSED");
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->details["reason"], "DOCK_UNREACHABLE");
}

TEST(Undock, HardwareThatChargesByItselfLeavesWithoutChargingOff) {
  TestCore core = Docked();
  core.OnCapabilities(true, false);
  StartUndocking(core);
  EXPECT_TRUE(core.Undocking());
  EXPECT_FALSE(core.TakeChargeOff());
  const auto path = core.TakeUndockPath();
  ASSERT_TRUE(path);
  EXPECT_EQ(path->poses.size(), 9u);
  EXPECT_TRUE(core.FollowerActive());
  // A late answer of a driver changes nothing.
  core.OnChargeOff(false, "charging is controlled by the board");
  EXPECT_TRUE(core.Undocking());
  core.TakeStopRequest();
  ASSERT_EQ(core.OnAction(kAbort), ActionResult::kDone);
  core.OnUndockEnded(LegEnd::kCanceled, Reason("STOPPED", "Aborted by the app (canceled)"));
  EXPECT_TRUE(core.Idle());
  // Nothing to allow again either.
  EXPECT_FALSE(core.TakeChargeOn());
}

TEST(Undock, ADriverThatCanStopChargingIsWaitedFor) {
  TestCore core = Docked();
  core.OnCapabilities(true, false);
  core.OnCapabilities(true, true);
  StartUndocking(core);
  EXPECT_FALSE(core.TakeUndockPath());
  EXPECT_TRUE(core.TakeChargeOff());
  core.OnChargeOff(false, "no answer from hw/charge");
  EXPECT_TRUE(core.Idle());
  EXPECT_TRUE(core.TakeChargeOn());
}

// The platform loses its safety measurement while undocking: the way ends with UNDOCKING_FAILED NO_SAFETY_MEASUREMENT,
// not NOT_ALLOWED, which only the follower uses for outside of the areas.
TEST(Undock, WithoutTheSafetyMeasurementTheWayStops) {
  TestCore core = Docked();
  StartUndocking(core);
  ChargedOff(core);
  core.TakeEvents();
  core.OnCapabilities(false);
  EXPECT_EQ(core.TakeDriveCancel().value_or(Reason()).code, "NO_SAFETY_MEASUREMENT");
  EXPECT_TRUE(core.TakeStopRequest());
  core.OnUndockEnded(LegEnd::kCanceled, Reason("NO_SAFETY_MEASUREMENT", "canceled"));
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "UNDOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "NO_SAFETY_MEASUREMENT");
  EXPECT_EQ(failed->details["severity"], "error");
  EXPECT_EQ(failed->details["resume"], "user");
  EXPECT_TRUE(core.Idle());
}

TEST(Undock, AStartRefusedDuringTheCheckKeepsTheListOfTheFirst) {
  TestCore core = Docked();
  core.OnMowMap({{"lawn", "Lawn", true}, {"rose", "Rose", true}}, Reason());
  const json rose = {{"tasks", json::array({json::object({{"area_id", "rose"}})})}};
  ASSERT_TRUE(core.Command("mowing.start", rose).accepted);
  ASSERT_TRUE(core.TakeDockCheck());
  EXPECT_FALSE(core.Command("mowing.start", json::object()).accepted);
  const Reason why = core.OnDockCheck(true, kStartX, kDock.y, 0.02f, "");
  ASSERT_TRUE(why.empty()) << why.code << ": " << why.message;
  ChargedOff(core);
  Out(core);
  core.OnUndockEnded(LegEnd::kDone, Reason("DONE", "done"));
  ASSERT_TRUE(core.Mowing());
  const JobStep step = core.NextJobStep(0);
  ASSERT_EQ(step.kind, JobStep::Kind::kPlan);
  EXPECT_EQ(step.area_id, "rose");
}
