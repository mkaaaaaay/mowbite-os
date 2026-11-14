#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

const std::string kStart = "mower_logic:idle/start_mowing";
const std::string kAbortMowing = "mower_logic:mowing/abort_mowing";
const std::string kAbort = "mower_logic:docking/abort_docking";

// The station of the simulation: backing in going west, the mower faces east on the contacts. The line onto it begins
// 0.8 m east of it at (-0.2, 0), the gate is at (-0.75, 0).
const DockPose kDock{-1.0, 0.0, M_PI};
// A lawn that reaches to 0.5 m in front of the station, and one that begins 1 m in front of it.
const Ring kLawn{{-0.5, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.5, 2.0}};
const Ring kFarLawn{{0.0, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {0.0, 2.0}};

double Rad(double degrees) {
  return degrees * M_PI / 180.0;
}

double Angle(double radians) {
  return std::remainder(radians, 2 * M_PI);
}

HardwareSnapshot Healthy(bool contact = false, bool raw = false) {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  hw.dock_contact = contact;
  hw.dock_contact_raw = raw || contact;
  return hw;
}

// The report at t s (monotonic).
HardwareSnapshot At(HardwareSnapshot hw, double t) {
  hw.time_s = t;
  return hw;
}

// On the contacts at t s with amps A into the battery at percentage.
HardwareSnapshot Charging(double t, float amps = 3.0f, int percentage = 80) {
  HardwareSnapshot hw = At(Healthy(true), t);
  hw.battery_current = -amps;
  hw.battery_percentage = percentage;
  return hw;
}

PoseSnapshot PoseAt(double x, double y, double heading = 0.0, float accuracy = 0.02f, bool valid = true) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.heading = heading;
  pose.position_accuracy = accuracy;
  pose.orientation_accuracy = valid ? 0.01f : 999.0f;
  pose.orientation_valid = valid;
  return pose;
}

LogicConfig Tron() {
  LogicConfig config;
  config.dock_contact_x = -0.10;
  return config;
}

// IDLE in the field, 2 m east of the station, with the station and the lawn in the map.
TestCore Ready(LogicConfig config = Tron(), const Ring& lawn = kLawn) {
  TestCore core{config};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(PoseAt(2.0, 0.5, M_PI));
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

std::vector<std::string> Types(const std::vector<Event>& events) {
  std::vector<std::string> types;
  for (const auto& e : events) types.push_back(e.type);
  return types;
}

const Event* Find(const std::vector<Event>& events, const std::string& type) {
  auto it = std::find_if(events.begin(), events.end(), [&](const Event& e) { return e.type == type; });
  return it == events.end() ? nullptr : &*it;
}

// A job on a lawn without anything left to mow: JOB_COMPLETE at once. The events of the start are taken.
void CompleteJob(TestCore& core) {
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.TakeEvents();
  core.OnJobPlanned({}, "fp");
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kNone);
}

DockingLeg Take(TestCore& core, DockingLeg::Kind kind) {
  auto leg = core.TakeDockingLeg();
  EXPECT_TRUE(leg);
  if (!leg) return DockingLeg();
  EXPECT_EQ(leg->kind, kind) << DockingLegName(leg->kind);
  return *leg;
}

// Docking after the job, through the approach and the line, stopped at the gate within its limits.
TestCore AtTheGate(LogicConfig config = Tron()) {
  TestCore core = Ready(config);
  CompleteJob(core);
  Take(core, DockingLeg::Kind::kApproach);
  core.OnPose(PoseAt(-0.21, 0.01, Rad(3)));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  Take(core, DockingLeg::Kind::kLine);
  core.OnPose(PoseAt(-0.74, 0.01, Rad(-3)));
  return core;
}

// A lawn around the station and the point in front of it: from where the mower stands nothing lies within 1.8 m of
// the straight way there.
const Ring kOpenLawn{{-2.0, -3.0}, {6.0, -3.0}, {6.0, 3.0}, {-2.0, 3.0}};
// From base_link to the front corners of the Tron.
const double kTurningRadius = 0.644;
const Reason kNoAnswer("PLANNER_TIMEOUT", "No answer from the planner in 10 s");
const Reason kNotRunning("PLANNER_MISSING", "The planner is not running");

// Docking after the job, the planner asked for the way to the point in front of the station.
TestCore Approaching(LogicConfig config = Tron(), const Ring& lawn = kOpenLawn, const std::vector<Ring>& obstacles = {},
                     double turning_radius = kTurningRadius) {
  TestCore core = Ready(config, lawn);
  core.OnDrivableAreas({lawn}, obstacles);
  core.SetTurningRadius(turning_radius);
  CompleteJob(core);
  Take(core, DockingLeg::Kind::kApproach);
  core.TakeEvents();
  core.TakeNotices();
  return core;
}

// On the last leg onto the contacts.
TestCore OntoTheContacts(LogicConfig config = Tron()) {
  TestCore core = AtTheGate(config);
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  Take(core, DockingLeg::Kind::kContact);
  core.TakeEvents();
  core.TakeNotices();
  core.TakeStopRequest();
  core.TakeDriveCancel();
  return core;
}

}  // namespace

TEST(DockingSequence, TheLineOntoTheStation) {
  DockConfig config;
  const DockingSequence docking(config, UndockConfig(), kDock, true);
  const Point2 p = docking.ApproachPoint();
  EXPECT_NEAR(p.x, -0.2, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  EXPECT_NEAR(docking.Along(-1.0, 0.0), 0.8, 1e-9);
  // Left of the way in (west) is south.
  EXPECT_NEAR(docking.Across(-0.5, -0.1), 0.1, 1e-9);

  const DockingLeg line = docking.Line();
  EXPECT_EQ(line.kind, DockingLeg::Kind::kLine);
  EXPECT_TRUE(line.reverse);
  EXPECT_FALSE(line.straight);
  EXPECT_DOUBLE_EQ(line.speed, 0.1);
  EXPECT_DOUBLE_EQ(line.stall_time, 2.0);
  EXPECT_GT(line.end_tolerance, 0.0);
  ASSERT_GE(line.poses.size(), 3u);
  EXPECT_NEAR(line.poses.front().x, -0.2, 1e-9);
  EXPECT_NEAR(line.poses.back().x, -0.75, 1e-9);
  for (size_t i = 0; i < line.poses.size(); i++) {
    EXPECT_NEAR(line.poses[i].y, 0.0, 1e-9);
    EXPECT_NEAR(Angle(line.poses[i].heading), 0.0, 1e-9);
    if (i > 0) {
      EXPECT_LE(line.poses[i - 1].x - line.poses[i].x, 0.1 + 1e-9);
    }
  }
  const DockingLeg contact = docking.Contact();
  EXPECT_EQ(contact.kind, DockingLeg::Kind::kContact);
  EXPECT_TRUE(contact.reverse);
  EXPECT_DOUBLE_EQ(contact.speed, 0.06);
  EXPECT_DOUBLE_EQ(contact.stall_time, 10.0);
  EXPECT_LT(contact.end_tolerance, 0.01);
  EXPECT_NEAR(contact.poses.front().x, -0.75, 1e-9);
  // 0.05 m past the station: the mower looks for the contacts that far.
  EXPECT_NEAR(contact.poses.back().x, -1.05, 1e-9);
  // The corridor holds the line from the approach point to past the station, 0.1 m to each side.
  EXPECT_TRUE(InsideRing(line.corridor, -0.15, 0.09));
  EXPECT_TRUE(InsideRing(contact.corridor, -1.1, -0.09));
  EXPECT_FALSE(InsideRing(contact.corridor, -0.6, 0.11));
}

TEST(DockingSequence, WhereATryBegins) {
  const DockingSequence docking(DockConfig(), UndockConfig(), kDock, true);
  // Far from the line: the approach to the point in front of the station, facing away from it.
  DockingLeg leg = docking.FirstLeg(2.0, 1.0, M_PI);
  EXPECT_EQ(leg.kind, DockingLeg::Kind::kApproach);
  EXPECT_NEAR(leg.goal.x, -0.2, 1e-9);
  EXPECT_NEAR(Angle(leg.goal.heading), 0.0, 1e-9);
  // At the approach point facing about the right way: the line at once; turned more, the approach.
  EXPECT_EQ(docking.FirstLeg(-0.25, 0.05, Rad(15)).kind, DockingLeg::Kind::kLine);
  EXPECT_EQ(docking.FirstLeg(-0.25, 0.05, Rad(30)).kind, DockingLeg::Kind::kApproach);
  // On the line: forwards along its own heading out to the approach point.
  leg = docking.FirstLeg(-0.9, 0.1, Rad(5));
  EXPECT_EQ(leg.kind, DockingLeg::Kind::kOut);
  EXPECT_TRUE(leg.straight);
  EXPECT_FALSE(leg.reverse);
  EXPECT_DOUBLE_EQ(leg.speed, 0.15);
  EXPECT_DOUBLE_EQ(leg.stall_time, 2.0);
  EXPECT_NEAR(leg.poses.front().x, -0.9, 1e-9);
  EXPECT_NEAR(leg.poses.back().x, -0.9 + 0.7 * std::cos(Rad(5)), 1e-9);
  EXPECT_NEAR(Angle(leg.poses.back().heading), Rad(5), 1e-9);
  // Beside the line or past the station it is no try from the line.
  EXPECT_FALSE(docking.OnLine(-0.9, 0.4));
  EXPECT_FALSE(docking.OnLine(-1.3, 0.0));
}

TEST(DockingSequence, WithTheContactsInFrontForwardsOnAndBackwardsOut) {
  // The same station; on it the mower faces west, the way in.
  const DockingSequence docking(DockConfig(), UndockConfig(), kDock, false);
  DockingLeg leg = docking.FirstLeg(2.0, 1.0, 0.0);
  EXPECT_EQ(leg.kind, DockingLeg::Kind::kApproach);
  EXPECT_NEAR(leg.goal.x, -0.2, 1e-9);
  EXPECT_NEAR(Angle(leg.goal.heading - M_PI), 0.0, 1e-9);
  EXPECT_EQ(docking.FirstLeg(-0.25, 0.05, M_PI - Rad(15)).kind, DockingLeg::Kind::kLine);
  EXPECT_EQ(docking.FirstLeg(-0.25, 0.05, Rad(15)).kind, DockingLeg::Kind::kApproach);
  const DockingLeg line = docking.Line();
  EXPECT_FALSE(line.reverse);
  EXPECT_NEAR(line.poses.front().x, -0.2, 1e-9);
  EXPECT_NEAR(line.poses.back().x, -0.75, 1e-9);
  for (const auto& p : line.poses) EXPECT_NEAR(Angle(p.heading - M_PI), 0.0, 1e-9);
  const DockingLeg contact = docking.Contact();
  EXPECT_FALSE(contact.reverse);
  EXPECT_DOUBLE_EQ(contact.speed, 0.06);
  EXPECT_NEAR(contact.poses.back().x, -1.05, 1e-9);
  // On the line: straight back along its own heading out to the approach point, at most at the docking speed.
  leg = docking.FirstLeg(-0.9, 0.1, M_PI + Rad(5));
  EXPECT_EQ(leg.kind, DockingLeg::Kind::kOut);
  EXPECT_TRUE(leg.straight);
  EXPECT_TRUE(leg.reverse);
  EXPECT_DOUBLE_EQ(leg.speed, 0.1);
  EXPECT_NEAR(leg.poses.front().x, -0.9, 1e-9);
  EXPECT_NEAR(leg.poses.back().x, -0.9 + 0.7 * std::cos(Rad(5)), 1e-9);
  EXPECT_NEAR(leg.poses.back().y, 0.1 + 0.7 * std::sin(Rad(5)), 1e-9);
  EXPECT_NEAR(Angle(leg.poses.back().heading - M_PI - Rad(5)), 0.0, 1e-9);
  EXPECT_TRUE(InsideRing(leg.corridor, -0.9, 0.1));
}

TEST(DockingSequence, TheWayOffTheStation) {
  UndockConfig out;
  DockConfig dock;
  // Forwards at the speed of undocking.
  UndockPath path = PlanLeave(0.0, 0.0, 0.0, true, out, dock);
  EXPECT_FALSE(path.reverse);
  EXPECT_DOUBLE_EQ(path.speed, 0.15);
  EXPECT_NEAR(path.poses.back().x, 0.8, 1e-9);
  // Backwards at most at the docking speed, also where undocking would be slower.
  path = PlanLeave(0.0, 0.0, 0.0, false, out, dock);
  EXPECT_TRUE(path.reverse);
  EXPECT_DOUBLE_EQ(path.speed, 0.1);
  EXPECT_NEAR(path.poses.back().x, -0.8, 1e-9);
  out.speed = 0.05;
  EXPECT_DOUBLE_EQ(PlanLeave(0.0, 0.0, 0.0, false, out, dock).speed, 0.05);
}

TEST(DockingSequence, TheGateChecksSideHeadingAndAccuracy) {
  const DockingSequence docking(DockConfig(), UndockConfig(), kDock, true);
  EXPECT_EQ(docking.CheckGate(-0.75, 0.019, Rad(7.9), 0.02f, 0.03), "");
  EXPECT_NE(docking.CheckGate(-0.75, 0.025, 0.0, 0.02f, 0.03).find("0.025 m beside the line"), std::string::npos);
  EXPECT_NE(docking.CheckGate(-0.75, 0.0, Rad(-9), 0.02f, 0.03).find("faced 9.0 deg off"), std::string::npos);
  EXPECT_NE(docking.CheckGate(-0.75, 0.0, 0.0, 0.05f, 0.03).find("accuracy 0.050 m"), std::string::npos);
}

TEST(DockingSequence, TriesAgainRetryCountTimes) {
  DockConfig config;
  config.retry_count = 2;
  DockingSequence docking(config, UndockConfig(), kDock, true);
  EXPECT_TRUE(docking.Failed());
  EXPECT_TRUE(docking.Failed());
  EXPECT_FALSE(docking.Failed());
  EXPECT_EQ(docking.failures(), 3);
}

TEST(DockingSequence, TheStraightWayToTheApproachPoint) {
  DockConfig config;
  config.direct_margin = 0.2;
  const DockingSequence docking(config, UndockConfig(), kDock, true);
  // From 0.25 m north of the approach point: poses at most 0.1 m apart facing south, the last one facing east as on
  // the station.
  const auto leg = docking.Direct(-0.2, 0.25);
  ASSERT_TRUE(leg);
  EXPECT_EQ(leg->kind, DockingLeg::Kind::kDirect);
  ASSERT_EQ(leg->poses.size(), 4u);
  for (size_t i = 0; i < 3; i++) {
    EXPECT_NEAR(leg->poses[i].x, -0.2, 1e-9);
    EXPECT_NEAR(leg->poses[i].y, 0.25 - 0.25 * i / 3, 1e-9);
    EXPECT_NEAR(Angle(leg->poses[i].heading + M_PI / 2), 0.0, 1e-9);
  }
  EXPECT_NEAR(leg->poses[3].x, -0.2, 1e-9);
  EXPECT_NEAR(leg->poses[3].y, 0.0, 1e-9);
  EXPECT_NEAR(Angle(leg->poses[3].heading), 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(leg->max_cross_track, 0.2);
  EXPECT_TRUE(leg->corridor.empty());
  EXPECT_FALSE(leg->straight);
  EXPECT_FALSE(leg->reverse);
  EXPECT_DOUBLE_EQ(leg->speed, 0.0);
  EXPECT_DOUBLE_EQ(leg->stall_time, 0.0);
  EXPECT_DOUBLE_EQ(leg->end_tolerance, 0.0);
  // The follower needs 3 poses also on the shortest way.
  const auto shortest = docking.Direct(-0.2, 0.11);
  ASSERT_TRUE(shortest);
  ASSERT_EQ(shortest->poses.size(), 3u);
  EXPECT_NEAR(shortest->poses[1].y, 0.055, 1e-9);
  // Within 0.1 m of the approach point there is nothing to drive.
  EXPECT_FALSE(docking.Direct(-0.2, 0.099));
  EXPECT_FALSE(docking.Direct(-0.15, -0.05));
  // The other legs keep the limit of the follower.
  EXPECT_DOUBLE_EQ(docking.Line().max_cross_track, 0.0);
  EXPECT_DOUBLE_EQ(docking.Contact().max_cross_track, 0.0);
  EXPECT_DOUBLE_EQ(docking.Out(-0.6, 0.0, 0.0).max_cross_track, 0.0);
  EXPECT_STREQ(DockingLegName(DockingLeg::Kind::kDirect), "direct");
}

TEST(Docking, AfterTheJobTheMowerBacksOntoTheStation) {
  TestCore core = Ready();
  CompleteJob(core);
  EXPECT_TRUE(core.Docking());
  EXPECT_FALSE(core.Idle());
  EXPECT_EQ(core.RobotState().state, "DOCKING");
  auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"JOB_COMPLETE", "STATE"}));
  EXPECT_EQ(events[1].details["state"], "DOCKING");
  EXPECT_TRUE(Enabled(core, "mower_logic:docking", "abort_docking"));
  EXPECT_EQ(core.StartProblem().code, "BUSY");
  EXPECT_EQ(core.StartProblem().fields["state"], "DOCKING");
  // Nothing drives before the node took the leg; never the blade.
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_FALSE(core.BladeWanted());

  DockingLeg leg = Take(core, DockingLeg::Kind::kApproach);
  EXPECT_NEAR(leg.goal.x, -0.2, 1e-9);
  EXPECT_NEAR(leg.goal.y, 0.0, 1e-9);
  EXPECT_NEAR(Angle(leg.goal.heading), 0.0, 1e-9);
  EXPECT_TRUE(core.FollowerActive());
  EXPECT_FALSE(core.TakeDockingLeg());
  // A contact on the way there means nothing.
  core.OnHardwareStatus(Healthy(true));
  EXPECT_TRUE(core.Docking());
  core.OnHardwareStatus(Healthy());
  core.OnPose(PoseAt(-0.21, 0.01, Rad(3)));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));

  leg = Take(core, DockingLeg::Kind::kLine);
  EXPECT_TRUE(core.FollowerActive());
  core.OnPose(PoseAt(-0.74, 0.01, Rad(-3)));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  leg = Take(core, DockingLeg::Kind::kContact);
  EXPECT_TRUE(core.FollowerActive());

  // The first raw contact stops the mower at once: one zero command, then nothing.
  core.TakeStopRequest();
  core.OnHardwareStatus(Healthy(false, true));
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_TRUE(core.TakeStopRequest());
  EXPECT_TRUE(core.Docking());
  core.OnHardwareStatus(Healthy(false, true));
  EXPECT_FALSE(core.TakeStopRequest());

  // The debounced contact ends the leg; the charge current has to hold 5 s before DOCKED.
  core.TakeEvents();
  core.OnHardwareStatus(At(Healthy(true), 100.0));
  EXPECT_TRUE(core.Docking());
  EXPECT_TRUE(core.TakeDriveCancel());
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_FALSE(core.TakeDockingLeg());
  EXPECT_TRUE(Enabled(core, "mower_logic:docking", "abort_docking"));
  // The late end of the follower changes nothing.
  core.OnDockingLegEnded(LegEnd::kCanceled, Reason("STOPPED", "Contact (canceled)"));
  EXPECT_TRUE(core.Docking());
  core.OnHardwareStatus(Charging(101.0));
  core.OnTick(105.0);
  core.OnHardwareStatus(Charging(105.9));
  EXPECT_TRUE(core.Docking());
  EXPECT_TRUE(core.TakeEvents().empty());
  // Confirmed: DOCKED without fields, IDLE.
  core.OnHardwareStatus(Charging(106.0));
  EXPECT_TRUE(core.Idle());
  events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKED", "STATE"}));
  EXPECT_FALSE(events[0].details.contains("reason"));
  EXPECT_EQ(events[1].details["state"], "IDLE");
  EXPECT_FALSE(core.TakeDriveCancel());
  EXPECT_FALSE(core.TakeChargeOff());
  EXPECT_FALSE(core.TakeChargeOn());
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_FALSE(Enabled(core, "mower_logic:docking", "abort_docking"));
  EXPECT_TRUE(core.RobotState().is_charging);
  core.OnHardwareStatus(Charging(120.0));
  EXPECT_TRUE(core.TakeEvents().empty());
}

TEST(Docking, AContactThatGoesBeforeItIsConfirmedPushesOnSlowly) {
  TestCore core = OntoTheContacts();
  core.OnHardwareStatus(Healthy(false, true));
  EXPECT_FALSE(core.FollowerActive());
  core.OnHardwareStatus(Healthy(false, false));
  EXPECT_TRUE(core.FollowerActive());
  EXPECT_TRUE(core.Docking());
  EXPECT_FALSE(core.TakeDriveCancel());
}

TEST(Docking, AbortMowingDocksToo) {
  TestCore core = Ready();
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  core.TakeEvents();
  ASSERT_EQ(core.OnAction(kAbortMowing), ActionResult::kDone);
  EXPECT_TRUE(core.Docking());
  const auto events = core.TakeEvents();
  EXPECT_FALSE(Find(events, "JOB_COMPLETE"));
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING", "STATE"}));
  EXPECT_EQ(events[0].details["reason"], "MANUAL");
  EXPECT_EQ(events[0].details["severity"], "info");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_EQ(Find(events, "STATE")->details["state"], "DOCKING");
  Take(core, DockingLeg::Kind::kApproach);
}

TEST(Docking, WithoutAStationOrChargingContactsTheJobEndsInIdle) {
  TestCore core = Ready();
  core.OnMapDock(std::nullopt);
  CompleteJob(core);
  EXPECT_TRUE(core.Idle());
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"JOB_COMPLETE", "STATE"}));
  EXPECT_FALSE(core.TakeDockingLeg());

  LogicConfig bare;
  bare.dock_contact_x = 0.0;
  TestCore other = Ready(bare);
  EXPECT_FALSE(other.CanDock());
  CompleteJob(other);
  EXPECT_TRUE(other.Idle());
  EXPECT_FALSE(other.TakeDockingLeg());
  const Reason why = other.DockingProblem();
  EXPECT_EQ(why.code, "NO_CHARGE_CONTACTS");
  EXPECT_NE(why.message.find("no charging contacts"), std::string::npos);
  // Also from the contacts of a station: it is no dock to it.
  other.OnHardwareStatus(Healthy(true));
  EXPECT_EQ(other.DockingProblem().code, "NO_CHARGE_CONTACTS");
}

TEST(Docking, WithTheContactsInFrontTheMowerDrivesOnForwardsAndBacksOut) {
  LogicConfig front;
  front.dock_contact_x = 0.3;
  TestCore core = Ready(front);
  EXPECT_TRUE(core.CanDock());
  CompleteJob(core);
  const auto notices = core.TakeNotices();
  EXPECT_TRUE(std::any_of(notices.begin(), notices.end(), [](const std::string& n) {
    return n.find("in front of the docking station, then forwards onto it") != std::string::npos;
  }));
  // To the point in front of the station facing west, the way in.
  const DockingLeg approach = Take(core, DockingLeg::Kind::kApproach);
  EXPECT_NEAR(approach.goal.x, -0.2, 1e-9);
  EXPECT_NEAR(Angle(approach.goal.heading - M_PI), 0.0, 1e-9);
  core.OnPose(PoseAt(-0.21, 0.01, M_PI + Rad(3)));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  const DockingLeg line = Take(core, DockingLeg::Kind::kLine);
  EXPECT_FALSE(line.reverse);
  EXPECT_DOUBLE_EQ(line.speed, 0.1);
  EXPECT_NEAR(line.poses.back().x, -0.75, 1e-9);
  for (const auto& p : line.poses) EXPECT_NEAR(Angle(p.heading - M_PI), 0.0, 1e-9);
  // 3 cm beside the line at the gate: backwards out to the approach point, at most at the docking speed.
  core.OnPose(PoseAt(-0.74, 0.03, M_PI - Rad(2)));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  const DockingLeg out = Take(core, DockingLeg::Kind::kOut);
  EXPECT_TRUE(out.straight);
  EXPECT_TRUE(out.reverse);
  EXPECT_DOUBLE_EQ(out.speed, 0.1);
  EXPECT_NEAR(out.poses.front().x, -0.74, 1e-9);
  EXPECT_NEAR(out.poses.back().x, -0.74 + 0.54 * std::cos(Rad(2)), 1e-9);
  EXPECT_NEAR(Angle(out.poses.back().heading - M_PI + Rad(2)), 0.0, 1e-9);
  EXPECT_TRUE(InsideRing(out.corridor, -0.74, 0.03));
  core.OnPose(PoseAt(-0.21, 0.01, M_PI + Rad(1)));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  Take(core, DockingLeg::Kind::kLine);
  core.OnPose(PoseAt(-0.74, 0.01, M_PI - Rad(3)));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  const DockingLeg contact = Take(core, DockingLeg::Kind::kContact);
  EXPECT_FALSE(contact.reverse);
  EXPECT_NEAR(contact.poses.back().x, -1.05, 1e-9);
}

TEST(Docking, AnApproachPointOutsideOfTheAreasFailsAtOnce) {
  TestCore core = Ready(Tron(), kFarLawn);
  CompleteJob(core);
  EXPECT_TRUE(core.Idle());
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"JOB_COMPLETE", "DOCKING_FAILED", "STATE"}));
  const auto& failed = events[1].details;
  EXPECT_EQ(failed["reason"], "approach_failed");
  EXPECT_EQ(failed["attempts"], 1);
  EXPECT_EQ(failed["severity"], "error");
  EXPECT_EQ(failed["resume"], "user");
  EXPECT_NE(failed["message"].get<std::string>().find("0.80 m in front of it"), std::string::npos) << failed;
  EXPECT_FALSE(core.TakeDockingLeg());
}

TEST(Docking, OnTheContactsAtTheEndOfTheJobItIsDockedOnceTheChargeHolds) {
  TestCore core = Ready();
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnHardwareStatus(At(Healthy(true), 10.0));
  core.TakeEvents();
  core.OnJobPlanned({}, "fp");
  core.NextJobStep(0);
  EXPECT_TRUE(core.Idle());
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"JOB_COMPLETE", "STATE"}));
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnHardwareStatus(Charging(11.0));
  core.OnHardwareStatus(Charging(16.0));
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"DOCKED"}));
}

TEST(Docking, MissingTheGateDrivesOutAndTriesAgain) {
  TestCore core = AtTheGate();
  // 3 cm beside the line at the gate.
  core.OnPose(PoseAt(-0.74, 0.03, Rad(2)));
  core.TakeEvents();
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  // Out first, counted once there.
  const DockingLeg out = Take(core, DockingLeg::Kind::kOut);
  EXPECT_TRUE(out.straight);
  EXPECT_FALSE(out.reverse);
  EXPECT_NEAR(out.poses.front().x, -0.74, 1e-9);
  EXPECT_NEAR(out.poses.front().y, 0.03, 1e-9);
  EXPECT_NEAR(out.poses.back().x, -0.74 + 0.54 * std::cos(Rad(2)), 1e-9);
  EXPECT_TRUE(InsideRing(out.corridor, -0.74, 0.03));
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_TRUE(core.Docking());
  core.OnPose(PoseAt(-0.22, 0.04, Rad(2)));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_RETRY"}));
  const auto& retry = events[0].details;
  EXPECT_EQ(retry["reason"], "dock_failed");
  EXPECT_EQ(retry["attempts"], 1);
  EXPECT_EQ(retry["severity"], "warning");
  EXPECT_EQ(retry["resume"], "auto");
  EXPECT_NE(retry["message"].get<std::string>().find("0.030 m beside the line"), std::string::npos) << retry;
  // From the approach point straight onto the line again.
  Take(core, DockingLeg::Kind::kLine);
}

TEST(Docking, AfterTheLastRetryItFailsAtTheApproachPoint) {
  LogicConfig config = Tron();
  config.dock.retry_count = 1;
  TestCore core = OntoTheContacts(config);
  for (int attempt = 1; attempt <= 2; attempt++) {
    // Pushed at the station without a contact until it counts as stuck.
    core.OnPose(PoseAt(-0.99, 0.0, 0.0));
    const Reason stuck("TIMEOUT", "the mower does not move: 0.00 m in 10.0 s while driven at 0.06 m/s, stuck?");
    core.OnDockingLegEnded(LegEnd::kFailed, stuck);
    Take(core, DockingLeg::Kind::kOut);
    core.OnPose(PoseAt(-0.22, 0.0, 0.0));
    core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
    const auto events = core.TakeEvents();
    if (attempt == 1) {
      ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_RETRY"}));
      EXPECT_EQ(events[0].details["attempts"], 1);
      Take(core, DockingLeg::Kind::kLine);
      core.OnPose(PoseAt(-0.74, 0.0, 0.0));
      core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
      Take(core, DockingLeg::Kind::kContact);
    } else {
      ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_FAILED", "STATE"}));
      const auto& failed = events[0].details;
      EXPECT_EQ(failed["reason"], "dock_failed");
      EXPECT_EQ(failed["attempts"], 2);
      EXPECT_EQ(failed["severity"], "error");
      EXPECT_EQ(failed["resume"], "user");
      EXPECT_NE(failed["message"].get<std::string>().find("does not move"), std::string::npos) << failed;
      EXPECT_EQ(events[1].details["state"], "IDLE");
    }
  }
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.TakeDockingLeg());
}

TEST(Docking, TheEndOfTheLineWithoutAContactFailsTheTry) {
  LogicConfig config = Tron();
  config.dock.retry_count = 0;
  TestCore core = OntoTheContacts(config);
  // Past where the station should be: it is not there.
  core.OnPose(PoseAt(-1.045, 0.0, 0.0));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  Take(core, DockingLeg::Kind::kOut);
  core.OnPose(PoseAt(-0.22, 0.0, 0.0));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "DOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["attempts"], 1);
  EXPECT_NE(failed->details["message"].get<std::string>().find("no contact with the docking station up to 0.05 m"),
            std::string::npos)
      << failed->details;
  EXPECT_TRUE(core.Idle());
}

TEST(Docking, AWayThatFailsIsTriedAgain) {
  LogicConfig config = Tron();
  config.dock.retry_count = 1;
  TestCore core = Ready(config);
  CompleteJob(core);
  core.TakeEvents();
  Take(core, DockingLeg::Kind::kApproach);
  core.OnDockingLegEnded(LegEnd::kFailed, Reason("TRANSIT_FAILED", "Transit planning failed: no way"));
  auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_RETRY"}));
  EXPECT_EQ(events[0].details["reason"], "approach_failed");
  EXPECT_EQ(events[0].details["attempts"], 1);
  Take(core, DockingLeg::Kind::kApproach);
  core.OnDockingLegEnded(LegEnd::kFailed, Reason("TRANSIT_FAILED", "Transit planning failed: no way"));
  events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_FAILED", "STATE"}));
  EXPECT_EQ(events[0].details["reason"], "approach_failed");
  EXPECT_EQ(events[0].details["attempts"], 2);
  EXPECT_NE(events[0].details["message"].get<std::string>().find("no way"), std::string::npos);
}

TEST(DockingDirect, APlannerThatDoesNotAnswerSendsTheMowerStraight) {
  TestCore core = Approaching();
  core.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
  EXPECT_TRUE(core.Docking());
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_DIRECT"}));
  const json& direct = events[0].details;
  EXPECT_EQ(direct["cause"], "PLANNER_TIMEOUT");
  EXPECT_NEAR(direct["distance"].get<double>(), std::hypot(2.2, 0.5), 1e-9);
  EXPECT_EQ(direct["severity"], "warning");
  EXPECT_EQ(direct["resume"], "auto");
  EXPECT_EQ(direct["message"], "No way from the planner (No answer from the planner in 10 s): straight to the point "
                               "in front of the docking station, 2.26 m");

  const DockingLeg leg = Take(core, DockingLeg::Kind::kDirect);
  ASSERT_EQ(leg.poses.size(), 24u);
  EXPECT_NEAR(leg.poses.front().x, 2.0, 1e-9);
  EXPECT_NEAR(leg.poses.front().y, 0.5, 1e-9);
  EXPECT_NEAR(leg.poses.back().x, -0.2, 1e-9);
  EXPECT_NEAR(leg.poses.back().y, 0.0, 1e-9);
  // Along the way the poses face it, the last one faces as the mower does on the station.
  const double course = std::atan2(-0.5, -2.2);
  for (size_t i = 0; i + 1 < leg.poses.size(); i++) {
    EXPECT_NEAR(Angle(leg.poses[i].heading - course), 0.0, 1e-9) << i;
    const double step = std::hypot(leg.poses[i + 1].x - leg.poses[i].x, leg.poses[i + 1].y - leg.poses[i].y);
    EXPECT_NEAR(step, std::hypot(2.2, 0.5) / 23, 1e-9) << i;
  }
  EXPECT_NEAR(Angle(leg.poses.back().heading), 0.0, 1e-9);
  // base_link no farther off it than the margin of the check, and no corridor besides the areas.
  EXPECT_DOUBLE_EQ(leg.max_cross_track, 0.15);
  EXPECT_TRUE(leg.corridor.empty());
  EXPECT_TRUE(core.FollowerActive());
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_FALSE(core.TakeDockingLeg());

  // A contact on the way means nothing; from its end on as after the approach.
  core.OnHardwareStatus(Healthy(true));
  EXPECT_TRUE(core.FollowerActive());
  core.OnHardwareStatus(Healthy());
  core.OnPose(PoseAt(-0.21, 0.01, Rad(3)));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  Take(core, DockingLeg::Kind::kLine);
  EXPECT_TRUE(core.TakeEvents().empty());
}

TEST(DockingDirect, APlannerThatIsNotRunningCountsTheSame) {
  TestCore core = Approaching();
  core.OnDockingLegEnded(LegEnd::kFailed, kNotRunning);
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_DIRECT"}));
  EXPECT_EQ(events[0].details["cause"], "PLANNER_MISSING");
  Take(core, DockingLeg::Kind::kDirect);
}

TEST(DockingDirect, AWayThatIsNotFreeFailsTheTry) {
  struct Case {
    const char* what;
    Ring lawn;
    std::vector<Ring> obstacles;
    double turning_radius;
    double margin;
    const char* why;
  };
  const double margin = Tron().dock.direct_margin;
  const Ring stone{{0.9, 0.55}, {1.1, 0.55}, {1.1, 0.75}, {0.9, 0.75}};
  const std::vector<Case> cases = {
      {"the lawn ends 0.3 m behind the point in front of the station", kLawn, {}, kTurningRadius, margin,
       "the areas end 0.30 m from the way at (-0.50, 0.00), less than 0.90 m"},
      {"an obstacle 0.25 m beside the way", kOpenLawn, {stone}, kTurningRadius, margin,
       "an obstacle 0.25 m from the way at (1.10, 0.55), less than 0.90 m"},
      {"no dimensions of the body", kOpenLawn, {}, 0.0, margin, "the size of the body is not known"},
      {"no margin for the follower to hold", kOpenLawn, {}, kTurningRadius, 0.0,
       "the size of the body is not known"},
  };
  for (const Case& c : cases) {
    LogicConfig config = Tron();
    config.dock.direct_margin = c.margin;
    TestCore core = Approaching(config, c.lawn, c.obstacles, c.turning_radius);
    core.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
    const auto events = core.TakeEvents();
    ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_RETRY"})) << c.what;
    const json& retry = events[0].details;
    EXPECT_EQ(retry["reason"], "approach_failed") << c.what;
    EXPECT_EQ(retry["cause"], "PLANNER_TIMEOUT") << c.what;
    EXPECT_EQ(retry["direct_blocked"], true) << c.what;
    EXPECT_EQ(retry["attempts"], 1) << c.what;
    EXPECT_EQ(retry["severity"], "warning") << c.what;
    EXPECT_EQ(retry["resume"], "auto") << c.what;
    const std::string message = retry["message"];
    EXPECT_EQ(message.find("The way to the docking station failed: No answer from the planner in 10 s; the straight "
                           "way is not free: " + std::string(c.why)),
              0u)
        << message;
    // The next try asks the planner first again.
    Take(core, DockingLeg::Kind::kApproach);
  }
}

// The follower drives up to the accuracy limit of the position, so the way is free only with that to spare as well.
TEST(DockingDirect, TheAccuracyLimitOfThePositionIsKeptFreeToo) {
  // The way runs along y = 0, an obstacle begins 0.86 m beside it: more than the 0.644 m of the body, the margin of
  // 0.15 m and the 1 cm of the check together.
  const Ring stone{{0.8, 0.86}, {1.2, 0.86}, {1.2, 1.2}, {0.8, 1.2}};
  for (const double accuracy : {0.1, 0.03}) {
    LogicConfig config = Tron();
    config.max_position_accuracy = accuracy;
    TestCore core = Ready(config, kOpenLawn);
    core.OnDrivableAreas({kOpenLawn}, {stone});
    core.SetTurningRadius(kTurningRadius);
    core.OnPose(PoseAt(2.0, 0.0, M_PI));
    CompleteJob(core);
    Take(core, DockingLeg::Kind::kApproach);
    core.TakeEvents();
    core.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
    const auto events = core.TakeEvents();
    ASSERT_EQ(events.size(), 1u) << accuracy;
    if (accuracy > 0.05) {
      EXPECT_EQ(events[0].type, "DOCKING_RETRY");
      const std::string message = events[0].details["message"];
      EXPECT_NE(message.find("an obstacle 0.86 m from the way at ("), std::string::npos) << message;
      EXPECT_NE(message.find("), less than 0.90 m"), std::string::npos) << message;
    } else {
      EXPECT_EQ(events[0].type, "DOCKING_DIRECT") << events[0].details;
    }
  }
}

TEST(DockingDirect, NeverAfterAnAnswerOfThePlanner) {
  TestCore core = Approaching();
  core.OnDockingLegEnded(LegEnd::kFailed, Reason("TRANSIT_FAILED", "Transit planning failed: no way"));
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_RETRY"}));
  EXPECT_EQ(events[0].details["reason"], "approach_failed");
  EXPECT_FALSE(events[0].details.contains("cause"));
  EXPECT_FALSE(events[0].details.contains("direct_blocked"));
  Take(core, DockingLeg::Kind::kApproach);
}

TEST(DockingDirect, AStraightWayThatFailsCountsWithItsCause) {
  const std::vector<Reason> ends = {
      Reason("CROSS_TRACK", "0.16 m off the path (limit of this path 0.15 m)"),
      Reason("TIMEOUT", "no progress along the path for 10 s"),
      Reason("OUTSIDE", "the mower is 0.31 m outside of the allowed areas at (1.00, 0.20)"),
  };
  for (const Reason& end : ends) {
    TestCore core = Approaching();
    core.OnDockingLegEnded(LegEnd::kFailed, kNotRunning);
    Take(core, DockingLeg::Kind::kDirect);
    core.TakeEvents();
    core.OnPose(PoseAt(1.0, 0.4, M_PI));
    core.OnDockingLegEnded(LegEnd::kFailed, end);
    const auto events = core.TakeEvents();
    ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_RETRY"})) << end.code;
    const json& retry = events[0].details;
    EXPECT_EQ(retry["reason"], "approach_failed") << end.code;
    EXPECT_EQ(retry["cause"], "PLANNER_MISSING") << end.code;
    EXPECT_EQ(retry["direct_blocked"], false) << end.code;
    EXPECT_EQ(retry["attempts"], 1) << end.code;
    EXPECT_EQ(retry["message"], "The straight way to the docking station failed: " + end.message);
    // The planner comes first again, and a way it refuses fails without a cause.
    Take(core, DockingLeg::Kind::kApproach);
    core.OnDockingLegEnded(LegEnd::kFailed, Reason("TRANSIT_FAILED", "Transit planning failed: no way"));
    const auto next = core.TakeEvents();
    ASSERT_EQ(Types(next), (std::vector<std::string>{"DOCKING_RETRY"})) << end.code;
    EXPECT_FALSE(next[0].details.contains("cause")) << end.code;
    EXPECT_FALSE(next[0].details.contains("direct_blocked")) << end.code;
    EXPECT_EQ(next[0].details["attempts"], 2) << end.code;
  }
}

TEST(DockingDirect, WithTheSwitchOffTheTryFailsWithItsCauseOnly) {
  LogicConfig config = Tron();
  config.dock.direct = false;
  TestCore core = Approaching(config);
  core.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_RETRY"}));
  EXPECT_EQ(events[0].details["reason"], "approach_failed");
  EXPECT_EQ(events[0].details["cause"], "PLANNER_TIMEOUT");
  EXPECT_FALSE(events[0].details.contains("direct_blocked"));
  EXPECT_EQ(events[0].details["message"],
            "The way to the docking station failed: No answer from the planner in 10 s");
  Take(core, DockingLeg::Kind::kApproach);
}

TEST(DockingDirect, AfterTheLastTryDockingFailsWithTheCause) {
  LogicConfig config = Tron();
  config.dock.retry_count = 1;
  TestCore core = Approaching(config, kLawn);
  core.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
  ASSERT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"DOCKING_RETRY"}));
  Take(core, DockingLeg::Kind::kApproach);
  core.OnDockingLegEnded(LegEnd::kFailed, kNotRunning);
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_FAILED", "STATE"}));
  const json& failed = events[0].details;
  EXPECT_EQ(failed["reason"], "approach_failed");
  EXPECT_EQ(failed["cause"], "PLANNER_MISSING");
  EXPECT_EQ(failed["direct_blocked"], true);
  EXPECT_EQ(failed["attempts"], 2);
  EXPECT_EQ(failed["severity"], "error");
  EXPECT_EQ(failed["resume"], "user");
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.TakeDockingLeg());
}

TEST(DockingDirect, AnEmergencyStopsItLikeEveryLeg) {
  // The driver reports the stop: IDLE without DOCKING_FAILED, and nothing goes on after the reset.
  TestCore core = Approaching();
  core.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
  Take(core, DockingLeg::Kind::kDirect);
  core.TakeEvents();
  HardwareSnapshot stop = Healthy();
  stop.emergency = true;
  stop.emergency_reason = kEmergencyLatch | kEmergencyStop;
  core.OnHardwareStatus(stop);
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.FollowerActive());
  const auto cancel = core.TakeDriveCancel();
  ASSERT_TRUE(cancel);
  EXPECT_EQ(cancel->code, "EMERGENCY");
  EXPECT_TRUE(core.TakeStopRequest());
  auto events = core.TakeEvents();
  EXPECT_TRUE(Find(events, "EMERGENCY"));
  EXPECT_FALSE(Find(events, "DOCKING_FAILED"));
  EXPECT_FALSE(Find(events, "DOCKING_RETRY"));
  core.OnHardwareStatus(Healthy());
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.TakeDockingLeg());

  // The follower saw it first.
  TestCore other = Approaching();
  other.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
  Take(other, DockingLeg::Kind::kDirect);
  other.TakeEvents();
  other.OnDockingLegEnded(LegEnd::kEmergency, Reason("EMERGENCY", "Emergency stop"));
  EXPECT_TRUE(other.Idle());
  events = other.TakeEvents();
  EXPECT_FALSE(Find(events, "DOCKING_FAILED"));
  EXPECT_FALSE(Find(events, "DOCKING_RETRY"));
  EXPECT_FALSE(other.TakeDockingLeg());
}

TEST(DockingDirect, ABadPositionWaitsAndAsksThePlannerAgain) {
  // On the straight way: the follower stopped, the try is not counted.
  TestCore core = Approaching();
  core.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
  Take(core, DockingLeg::Kind::kDirect);
  core.TakeEvents();
  core.OnPose(PoseAt(1.0, 0.3, M_PI, 0.2f));
  core.OnDockingLegEnded(LegEnd::kGps, Reason("POSITION_ACCURACY", "position accuracy 0.20 m worse than 0.10 m"));
  EXPECT_TRUE(core.Docking());
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnTick(100.0);
  core.OnPose(PoseAt(1.0, 0.3, M_PI));
  core.OnTick(105.0);
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnTick(115.0);
  Take(core, DockingLeg::Kind::kApproach);
  EXPECT_TRUE(core.TakeEvents().empty());

  // Bad already when the planner gives no answer: the way is not checked from a place the mower may not stand at.
  TestCore early = Approaching();
  early.OnPose(PoseAt(2.0, 0.5, M_PI, 0.2f));
  early.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
  EXPECT_TRUE(early.Docking());
  EXPECT_FALSE(early.TakeDockingLeg());
  EXPECT_TRUE(early.TakeEvents().empty());
  early.OnTick(100.0);
  early.OnPose(PoseAt(2.0, 0.5, M_PI));
  early.OnTick(105.0);
  early.OnTick(115.0);
  Take(early, DockingLeg::Kind::kApproach);
  early.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
  EXPECT_EQ(Types(early.TakeEvents()), (std::vector<std::string>{"DOCKING_DIRECT"}));
  Take(early, DockingLeg::Kind::kDirect);
}

// At the point in front of the station already, but turned away from it: no way to drive, the line begins there.
TEST(DockingDirect, AtTheApproachPointTheLineFollowsAtOnce) {
  TestCore core = Ready(Tron(), kOpenLawn);
  core.SetTurningRadius(kTurningRadius);
  core.OnPose(PoseAt(-0.15, 0.05, M_PI / 2));
  CompleteJob(core);
  Take(core, DockingLeg::Kind::kApproach);
  core.TakeEvents();
  core.OnDockingLegEnded(LegEnd::kCanceled, kNoAnswer);
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_DIRECT"}));
  EXPECT_NEAR(events[0].details["distance"].get<double>(), std::hypot(0.05, 0.05), 1e-9);
  Take(core, DockingLeg::Kind::kLine);
}

TEST(Docking, AnEmergencyStopsWithoutDockingFailed) {
  TestCore core = OntoTheContacts();
  HardwareSnapshot stop = Healthy();
  stop.emergency = true;
  stop.emergency_reason = kEmergencyLatch | kEmergencyStop;
  core.OnHardwareStatus(stop);
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.FollowerActive());
  const auto cancel = core.TakeDriveCancel();
  ASSERT_TRUE(cancel);
  EXPECT_EQ(cancel->code, "EMERGENCY");
  EXPECT_TRUE(core.TakeStopRequest());
  const auto events = core.TakeEvents();
  EXPECT_TRUE(Find(events, "EMERGENCY"));
  EXPECT_FALSE(Find(events, "DOCKING_FAILED"));
  EXPECT_FALSE(Find(events, "DOCKING_RETRY"));
  // It does not go on by itself after the reset either.
  core.OnHardwareStatus(Healthy());
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.TakeDockingLeg());
}

TEST(Docking, AbortDockingStopsWithoutAnEvent) {
  TestCore core = OntoTheContacts();
  ASSERT_EQ(core.OnAction(kAbort), ActionResult::kDone);
  EXPECT_TRUE(core.Idle());
  const auto cancel = core.TakeDriveCancel();
  ASSERT_TRUE(cancel);
  EXPECT_EQ(cancel->code, "STOPPED");
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"STATE"}));
  EXPECT_EQ(core.OnAction(kAbort), ActionResult::kRejected);
}

TEST(Docking, ABadPositionWaitsAndGoesOn) {
  TestCore core = AtTheGate();
  // The position got bad on the line: the follower stopped.
  core.OnPose(PoseAt(-0.5, 0.0, 0.0, 0.2f));
  core.OnDockingLegEnded(LegEnd::kGps, Reason("POSITION_ACCURACY", "position accuracy 0.20 m worse than 0.10 m"));
  EXPECT_TRUE(core.Docking());
  EXPECT_FALSE(core.FollowerActive());
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnTick(100.0);
  core.OnPose(PoseAt(-0.5, 0.0, 0.0));
  core.OnTick(105.0);
  EXPECT_FALSE(core.TakeDockingLeg());
  // Good for gps_wait_time (10 s): on from where it stands, out of the line first.
  core.OnTick(115.0);
  Take(core, DockingLeg::Kind::kOut);
  EXPECT_FALSE(Find(core.TakeEvents(), "DOCKING_RETRY"));
}

TEST(Docking, APositionThatStaysBadEndsDocking) {
  TestCore core = AtTheGate();
  core.OnPoseTimeout();
  EXPECT_FALSE(core.FollowerActive());
  const auto cancel = core.TakeDriveCancel();
  ASSERT_TRUE(cancel);
  EXPECT_EQ(cancel->code, "POSE_LOST");
  // The late end of the canceled leg changes nothing.
  core.OnDockingLegEnded(LegEnd::kCanceled, Reason("POSE_LOST", "canceled"));
  EXPECT_TRUE(core.Docking());
  core.TakeEvents();
  core.OnTick(10.0);
  core.OnTick(19.0);
  EXPECT_TRUE(core.Docking());
  core.OnTick(20.0);
  EXPECT_TRUE(core.Idle());
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "DOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "NO_POSITION");
  EXPECT_EQ(failed->details["severity"], "error");
  EXPECT_EQ(failed->details["attempts"], 1);
  EXPECT_NE(failed->details["message"].get<std::string>().find("no good position for 10 s"), std::string::npos);
}

TEST(Docking, WithoutTheSafetyMeasurementDockingFails) {
  TestCore core = OntoTheContacts();
  core.OnCapabilities(false);
  EXPECT_TRUE(core.Idle());
  EXPECT_TRUE(core.TakeDriveCancel());
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "DOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "dock_failed");
}

TEST(DockingStart, RefusedWithAReason) {
  TestCore core = Ready();
  EXPECT_TRUE(core.DockingProblem().empty()) << core.DockingProblem().code;
  // On the contacts already.
  core.OnHardwareStatus(Healthy(true));
  EXPECT_EQ(core.DockingProblem().code, "ALREADY_THERE");
  const Reason there = core.StartDocking();
  EXPECT_EQ(there.code, "ALREADY_THERE");
  const auto events = core.TakeEvents();
  const Event* refused = Find(events, "DRIVE_REFUSED");
  ASSERT_TRUE(refused);
  EXPECT_EQ(refused->details["reason"], "ALREADY_THERE");
  EXPECT_EQ(refused->details["severity"], "info");
  EXPECT_TRUE(core.Idle());
  core.OnHardwareStatus(Healthy());

  HardwareSnapshot stop = Healthy();
  stop.emergency = true;
  core.OnHardwareStatus(stop);
  EXPECT_EQ(core.DockingProblem().code, "EMERGENCY");
  core.OnHardwareStatus(Healthy());

  core.OnPose(PoseAt(2.0, 0.5, M_PI, 0.02f, false));
  EXPECT_EQ(core.DockingProblem().code, "NO_HEADING");
  core.OnPose(PoseAt(2.0, 0.5, M_PI));

  core.OnDrivableAreas({}, {}, MapReason(MapDatum::kMismatch, "The map does not fit"));
  EXPECT_EQ(core.DockingProblem().code, "MAP_MISMATCH");
  core.OnDrivableAreas({kFarLawn}, {});
  const Reason far = core.DockingProblem();
  EXPECT_EQ(far.code, "DOCK_UNREACHABLE");
  EXPECT_DOUBLE_EQ(far.fields["distance"].get<double>(), 0.8);
  // A longer approach (params.set) reaches the lawn.
  core.SetDockingSettings(1.2, 4);
  EXPECT_TRUE(core.DockingProblem().empty()) << core.DockingProblem().code;

  core.OnMapDock(std::nullopt);
  EXPECT_EQ(core.DockingProblem().code, "NO_DOCK");
  core.OnMapDock(kDock);

  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_EQ(core.DockingProblem().code, "BUSY");
}

TEST(DockingStart, DocksFromIdle) {
  TestCore core = Ready();
  core.SetDockingSettings(1.2, 2);
  EXPECT_TRUE(core.StartDocking().empty());
  EXPECT_TRUE(core.Docking());
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING", "STATE"}));
  EXPECT_EQ(events[0].details["reason"], "MANUAL");
  EXPECT_EQ(events[1].details["state"], "DOCKING");
  // The approach point of the setting.
  const DockingLeg leg = Take(core, DockingLeg::Kind::kApproach);
  EXPECT_NEAR(leg.goal.x, 0.2, 1e-9);
  EXPECT_EQ(core.StartDocking().code, "BUSY");
}

TEST(DockingStart, WithABadPositionItWaitsForAGoodOne) {
  TestCore core = Ready();
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  core.OnPose(PoseAt(2.0, 0.5, M_PI, 0.3f));
  core.TakeEvents();
  ASSERT_EQ(core.OnAction(kAbortMowing), ActionResult::kDone);
  EXPECT_TRUE(core.Docking());
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnTick(0.0);
  core.OnPose(PoseAt(2.0, 0.5, M_PI));
  core.OnTick(5.0);
  core.OnTick(15.0);
  Take(core, DockingLeg::Kind::kApproach);
}

namespace {

ChargeSample Sample(double t, double amps, int percentage = 80, bool contact = true, bool battery = true) {
  ChargeSample sample;
  sample.time = t;
  sample.contact = contact;
  sample.battery_ok = battery;
  sample.charge_current = amps;
  sample.percentage = percentage;
  return sample;
}

// The last leg reached the station: raw contact at t - 2 s, debounced at t s, the charge is checked, no current yet.
TestCore Checking(LogicConfig config = Tron(), double t = 100.0) {
  TestCore core = OntoTheContacts(config);
  core.OnPose(PoseAt(-1.0, 0.0, 0.0));
  core.OnHardwareStatus(At(Healthy(false, true), t - 2.0));
  core.OnHardwareStatus(At(Healthy(true), t));
  EXPECT_TRUE(core.Docking());
  core.TakeEvents();
  core.TakeNotices();
  core.TakeStopRequest();
  core.TakeDriveCancel();
  return core;
}

// IDLE on the station, the charge confirmed there at t s.
TestCore OnTheStation(double t = 10.0) {
  TestCore core = Ready();
  core.OnPose(PoseAt(-1.0, 0.0, 0.0));
  core.OnHardwareStatus(Charging(t - 5.0));
  core.OnHardwareStatus(Charging(t));
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"DOCKED"}));
  core.TakeNotices();
  return core;
}

}  // namespace

TEST(ChargeCheck, AChargeCurrentThatHoldsConfirms) {
  ChargeCheck check(ChargeConfig(), 10.0);
  EXPECT_EQ(check.OnSample(Sample(10.0, 0.0)), ChargeCheck::Result::kWaiting);
  EXPECT_EQ(check.OnSample(Sample(11.0, 3.2)), ChargeCheck::Result::kWaiting);
  EXPECT_EQ(check.OnSample(Sample(15.9, 3.4)), ChargeCheck::Result::kWaiting);
  EXPECT_EQ(check.OnSample(Sample(16.0, 3.3)), ChargeCheck::Result::kCharging);
  EXPECT_DOUBLE_EQ(check.max_current(), 3.4);
  // The result stays.
  EXPECT_EQ(check.OnSample(Sample(17.0, 0.0, 80, false)), ChargeCheck::Result::kCharging);
}

TEST(ChargeCheck, AGapBeginsTheTimeAgain) {
  ChargeCheck check(ChargeConfig(), 0.0);
  check.OnSample(Sample(1.0, 3.0));
  check.OnSample(Sample(4.0, 3.0));
  EXPECT_EQ(check.OnSample(Sample(4.5, 0.4)), ChargeCheck::Result::kWaiting);
  check.OnSample(Sample(5.0, 3.0));
  EXPECT_EQ(check.OnSample(Sample(9.9, 3.0)), ChargeCheck::Result::kWaiting);
  EXPECT_EQ(check.OnSample(Sample(10.0, 3.0)), ChargeCheck::Result::kCharging);
}

TEST(ChargeCheck, NearFullTheContactIsEnough) {
  ChargeCheck full(ChargeConfig(), 0.0);
  full.OnSample(Sample(0.0, 0.0, 95));
  EXPECT_EQ(full.OnSample(Sample(4.9, 0.0, 95)), ChargeCheck::Result::kWaiting);
  EXPECT_EQ(full.OnSample(Sample(5.0, 0.0, 96)), ChargeCheck::Result::kFull);
  EXPECT_EQ(full.percentage(), 96);
  // Below it the current has to come.
  ChargeCheck low(ChargeConfig(), 0.0);
  low.OnSample(Sample(0.0, 0.2, 94));
  EXPECT_EQ(low.OnSample(Sample(19.9, 0.2, 94)), ChargeCheck::Result::kWaiting);
  EXPECT_EQ(low.OnSample(Sample(20.0, 0.2, 94)), ChargeCheck::Result::kNoCharge);
  EXPECT_DOUBLE_EQ(low.max_current(), 0.2);
}

TEST(ChargeCheck, TheContactHasToHold) {
  ChargeCheck check(ChargeConfig(), 0.0);
  check.OnSample(Sample(1.0, 3.0));
  EXPECT_EQ(check.OnSample(Sample(2.0, 3.0, 80, false)), ChargeCheck::Result::kContactLost);
  EXPECT_EQ(check.OnSample(Sample(8.0, 3.0)), ChargeCheck::Result::kContactLost);
}

TEST(ChargeCheck, TheDeadline) {
  ChargeCheck none(ChargeConfig(), 100.0);
  EXPECT_EQ(none.OnTick(119.9), ChargeCheck::Result::kWaiting);
  EXPECT_EQ(none.OnTick(120.0), ChargeCheck::Result::kNoCharge);
  // A current that came before the deadline may hold past it.
  ChargeCheck late(ChargeConfig(), 100.0);
  late.OnSample(Sample(118.0, 1.0));
  EXPECT_EQ(late.OnTick(121.0), ChargeCheck::Result::kWaiting);
  EXPECT_EQ(late.OnSample(Sample(123.0, 1.0)), ChargeCheck::Result::kCharging);
  // A battery without a report confirms nothing.
  ChargeCheck unknown(ChargeConfig(), 0.0);
  unknown.OnSample(Sample(1.0, 3.0, 99, true, false));
  EXPECT_EQ(unknown.OnSample(Sample(20.0, 3.0, 99, true, false)), ChargeCheck::Result::kNoCharge);
}

TEST(Docking, AContactWithoutAChargeCurrentFailsTheTry) {
  TestCore core = Checking();
  // On the contacts at 0.4 A: nothing before the deadline of 20 s.
  core.OnHardwareStatus(Charging(110.0, 0.4f));
  core.OnTick(119.0);
  EXPECT_TRUE(core.Docking());
  EXPECT_FALSE(core.TakeChargeOff());
  core.OnTick(120.0);
  // Charging off first, then out; the try counts at the approach point.
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_FALSE(core.TakeDockingLeg());
  EXPECT_TRUE(core.TakeChargeOff());
  EXPECT_FALSE(core.TakeChargeOff());
  // Its late answer is all the node waits for; the contact stays meanwhile.
  core.OnHardwareStatus(Charging(121.0, 0.4f));
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnChargeOff(true, "");
  const DockingLeg out = Take(core, DockingLeg::Kind::kOut);
  EXPECT_NEAR(out.poses.front().x, -1.0, 1e-9);
  EXPECT_FALSE(out.reverse);
  EXPECT_FALSE(core.TakeChargeOn());
  core.OnHardwareStatus(At(Healthy(), 125.0));
  core.OnPose(PoseAt(-0.21, 0.0, 0.0));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  // Allowed again once out.
  EXPECT_TRUE(core.TakeChargeOn());
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_RETRY"}));
  const auto& retry = events[0].details;
  EXPECT_EQ(retry["reason"], "NO_CHARGE_CURRENT");
  EXPECT_EQ(retry["attempts"], 1);
  EXPECT_EQ(retry["severity"], "warning");
  EXPECT_EQ(retry["resume"], "auto");
  EXPECT_NEAR(retry["current"].get<double>(), 0.4, 1e-6);
  EXPECT_DOUBLE_EQ(retry["limit"].get<double>(), 0.5);
  EXPECT_NE(retry["message"].get<std::string>().find("no charge current on the contacts within 20 s"),
            std::string::npos)
      << retry;
  Take(core, DockingLeg::Kind::kLine);
}

TEST(Docking, TheLastTryWithoutAChargeCurrentLeavesTheMowerOnTheContacts) {
  LogicConfig config = Tron();
  config.dock.retry_count = 0;
  TestCore core = Checking(config);
  core.OnTick(120.0);
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.TakeDockingLeg());
  EXPECT_FALSE(core.TakeChargeOff());
  EXPECT_FALSE(core.TakeChargeOn());
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_FAILED", "STATE"}));
  EXPECT_EQ(events[0].details["reason"], "NO_CHARGE_CURRENT");
  EXPECT_EQ(events[0].details["attempts"], 1);
  EXPECT_EQ(events[0].details["severity"], "error");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_EQ(events[1].details["state"], "IDLE");
  // Told already: no CHARGING_LOST for the same lack. A stable current later is DOCKED after all, once.
  core.OnHardwareStatus(Charging(121.0, 0.0f));
  core.OnHardwareStatus(Charging(160.0, 0.0f));
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnHardwareStatus(Charging(161.0));
  core.OnHardwareStatus(Charging(165.9));
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnHardwareStatus(Charging(166.0));
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"DOCKED"}));
  core.OnHardwareStatus(Charging(190.0));
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.TakeDockingLeg());
}

TEST(Docking, WhereTheHardwareChargesByItselfItDrivesOutAtOnce) {
  TestCore core = Checking();
  core.OnCapabilities(true, false);
  core.OnTick(120.0);
  EXPECT_FALSE(core.TakeChargeOff());
  Take(core, DockingLeg::Kind::kOut);
}

TEST(Docking, ChargingThatCannotBeSwitchedOffKeepsTheMowerOnTheContacts) {
  TestCore core = Checking();
  core.OnTick(120.0);
  ASSERT_TRUE(core.TakeChargeOff());
  core.OnChargeOff(false, "no answer from hw/charge");
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.TakeDockingLeg());
  // Allowed again in case it went off after all.
  EXPECT_TRUE(core.TakeChargeOn());
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "DOCKING_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "NO_CHARGE_CURRENT");
  EXPECT_EQ(failed->details["attempts"], 1);
  EXPECT_NE(failed->details["message"].get<std::string>().find("could not be switched off (no answer from hw/charge)"),
            std::string::npos)
      << failed->details;
}

TEST(Docking, AContactThatGoesWhileTheChargeIsCheckedFailsTheTry) {
  TestCore core = Checking();
  core.OnHardwareStatus(Charging(101.0));
  // Gone, the raw one as well: nothing to switch off.
  core.OnHardwareStatus(At(Healthy(), 102.0));
  EXPECT_FALSE(core.TakeChargeOff());
  Take(core, DockingLeg::Kind::kOut);
  core.OnPose(PoseAt(-0.21, 0.0, 0.0));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  EXPECT_FALSE(core.TakeChargeOn());
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"DOCKING_RETRY"}));
  EXPECT_EQ(events[0].details["reason"], "dock_failed");
  EXPECT_NE(events[0].details["message"].get<std::string>().find("went away while the charge was checked"),
            std::string::npos);
}

TEST(Docking, ANearlyFullBatteryNeedsTheContactOnly) {
  TestCore core = Checking();
  core.OnHardwareStatus(Charging(101.0, 0.0f, 97));
  EXPECT_TRUE(core.Docking());
  core.OnHardwareStatus(Charging(106.0, 0.0f, 97));
  EXPECT_TRUE(core.Idle());
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"DOCKED", "STATE"}));
}

TEST(Docking, AContactWhileWaitingForAPositionChecksTheCharge) {
  TestCore core = AtTheGate();
  core.OnDockingLegEnded(LegEnd::kGps, Reason("POSITION_ACCURACY", "position accuracy 0.20 m worse than 0.10 m"));
  core.OnTick(45.0);
  core.OnHardwareStatus(At(Healthy(true), 50.0));
  core.OnHardwareStatus(Charging(51.0));
  // The position does not matter on the contacts any more.
  core.OnTick(55.5);
  EXPECT_TRUE(core.Docking());
  core.OnHardwareStatus(Charging(56.0));
  EXPECT_TRUE(core.Idle());
  const auto events = core.TakeEvents();
  EXPECT_TRUE(Find(events, "DOCKED"));
  EXPECT_FALSE(Find(events, "DOCKING_FAILED"));
}

TEST(Docking, AContactBeforeTheWayOutAfterAFailedLegChecksTheCharge) {
  TestCore core = OntoTheContacts();
  // The end of the line with the first raw contact; debounced only after the leg ended.
  core.OnHardwareStatus(At(Healthy(false, true), 99.0));
  core.OnPose(PoseAt(-1.0, 0.0, 0.0));
  core.OnDockingLegEnded(LegEnd::kDone, Reason("DONE", "done"));
  EXPECT_FALSE(core.TakeDockingLeg());
  ASSERT_TRUE(core.TakeChargeOff());
  core.OnHardwareStatus(At(Healthy(true), 100.0));
  EXPECT_TRUE(core.Docking());
  // Charging allowed again; the late answer to "off" changes nothing.
  EXPECT_TRUE(core.TakeChargeOn());
  core.OnChargeOff(true, "");
  EXPECT_FALSE(core.TakeDockingLeg());
  core.OnHardwareStatus(Charging(101.0));
  core.OnHardwareStatus(Charging(106.0));
  EXPECT_TRUE(core.Idle());
  const auto events = core.TakeEvents();
  EXPECT_TRUE(Find(events, "DOCKED"));
  EXPECT_FALSE(Find(events, "DOCKING_RETRY"));
}

TEST(Docking, AnEmergencyOrAbortWhileTheChargeIsCheckedStops) {
  TestCore core = Checking();
  HardwareSnapshot stop = Charging(101.0);
  stop.emergency = true;
  stop.emergency_reason = kEmergencyLatch | kEmergencyStop;
  core.OnHardwareStatus(stop);
  EXPECT_TRUE(core.Idle());
  auto events = core.TakeEvents();
  EXPECT_FALSE(Find(events, "DOCKING_FAILED"));
  EXPECT_FALSE(Find(events, "DOCKED"));
  EXPECT_FALSE(core.TakeChargeOn());

  TestCore aborted = Checking();
  ASSERT_EQ(aborted.OnAction(kAbort), ActionResult::kDone);
  EXPECT_TRUE(aborted.Idle());
  EXPECT_EQ(Types(aborted.TakeEvents()), (std::vector<std::string>{"STATE"}));
  // On the contacts in IDLE the watch confirms the charge like for any stay.
  aborted.OnHardwareStatus(Charging(101.0));
  aborted.OnHardwareStatus(Charging(106.0));
  EXPECT_EQ(Types(aborted.TakeEvents()), (std::vector<std::string>{"DOCKED"}));
  EXPECT_FALSE(aborted.TakeDockingLeg());
}

TEST(ChargingLost, TheContactGoneForTenSecondsIsTold) {
  TestCore core = OnTheStation();
  core.OnHardwareStatus(At(Healthy(), 20.0));
  core.OnHardwareStatus(At(Healthy(), 29.9));
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnHardwareStatus(At(Healthy(), 30.0));
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"CHARGING_LOST"}));
  EXPECT_EQ(events[0].details["reason"], "NO_DOCK_CONTACT");
  EXPECT_EQ(events[0].details["severity"], "warning");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_EQ(events[0].details["battery_percentage"], 80);
  // Once, and it does not drive.
  core.OnHardwareStatus(At(Healthy(), 60.0));
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.TakeDockingLeg());
  // Put back by hand: docked again once the charge holds.
  core.OnHardwareStatus(Charging(70.0));
  core.OnHardwareStatus(Charging(75.0));
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"DOCKED"}));
}

TEST(ChargingLost, NotWhenTheContactComesBackOrTheMowerIsLifted) {
  TestCore core = OnTheStation();
  core.OnHardwareStatus(At(Healthy(), 20.0));
  core.OnHardwareStatus(Charging(25.0));
  core.OnHardwareStatus(Charging(31.0));
  EXPECT_TRUE(core.TakeEvents().empty());
  // Lifted: the emergency tells.
  core.OnHardwareStatus(At(Healthy(), 32.0));
  HardwareSnapshot lifted = At(Healthy(), 35.0);
  lifted.emergency = true;
  lifted.emergency_reason = kEmergencyLatch | kEmergencyLift;
  core.OnHardwareStatus(lifted);
  lifted.time_s = 60.0;
  core.OnHardwareStatus(lifted);
  EXPECT_FALSE(Find(core.TakeEvents(), "CHARGING_LOST"));
}

TEST(ChargingLost, AChargeCurrentThatStopsBelowTheLimitIsTold) {
  TestCore core = OnTheStation();
  core.OnHardwareStatus(Charging(20.0, 0.0f));
  core.OnHardwareStatus(Charging(29.9, 0.1f));
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnHardwareStatus(Charging(30.0, 0.1f));
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"CHARGING_LOST"}));
  EXPECT_EQ(events[0].details["reason"], "NO_CHARGE_CURRENT");
  EXPECT_NEAR(events[0].details["current"].get<double>(), 0.1, 1e-6);
  EXPECT_DOUBLE_EQ(events[0].details["limit"].get<double>(), 0.5);
  EXPECT_NE(events[0].details["message"].get<std::string>().find("stopped for 10 s"), std::string::npos);
  core.OnHardwareStatus(Charging(60.0, 0.0f));
  EXPECT_TRUE(core.TakeEvents().empty());
  // Charging again: DOCKED once it holds, and told again when it stops again.
  core.OnHardwareStatus(Charging(61.0));
  core.OnHardwareStatus(Charging(66.0));
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"DOCKED"}));
  core.OnHardwareStatus(Charging(67.0, 0.0f));
  core.OnHardwareStatus(Charging(77.0, 0.0f));
  EXPECT_EQ(Types(core.TakeEvents()), (std::vector<std::string>{"CHARGING_LOST"}));
}

TEST(ChargingLost, NotWhenChargingEndsNearFull) {
  TestCore core = OnTheStation();
  core.OnHardwareStatus(Charging(20.0, 0.0f, 96));
  core.OnHardwareStatus(Charging(40.0, 0.0f, 90));
  core.OnHardwareStatus(Charging(80.0, 0.0f, 90));
  EXPECT_TRUE(core.TakeEvents().empty());
}

TEST(ChargingLost, AStayThatNeverChargesIsToldOnceAfterTheDeadline) {
  // Put on the station by hand, or started there: unconfirmed until the charge holds.
  TestCore core = Ready();
  core.OnHardwareStatus(Charging(10.0, 0.0f));
  core.OnHardwareStatus(Charging(29.9, 0.0f));
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnHardwareStatus(Charging(30.0, 0.0f));
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"CHARGING_LOST"}));
  EXPECT_EQ(events[0].details["reason"], "NO_CHARGE_CURRENT");
  EXPECT_NE(events[0].details["message"].get<std::string>().find("no charge current for 20 s"), std::string::npos);
  core.OnHardwareStatus(Charging(90.0, 0.0f));
  EXPECT_TRUE(core.TakeEvents().empty());
  // Not while an emergency is on.
  TestCore stopped = Ready();
  HardwareSnapshot stop = Charging(10.0, 0.0f);
  stop.emergency = true;
  stop.emergency_reason = kEmergencyLatch | kEmergencyStop;
  stopped.OnHardwareStatus(stop);
  stop.time_s = 40.0;
  stopped.OnHardwareStatus(stop);
  EXPECT_FALSE(Find(stopped.TakeEvents(), "CHARGING_LOST"));
}

TEST(ChargingLost, LostTelemetryKeepsTheConfirmationAndStartsTheStayAnew) {
  // Confirmed before: no second DOCKED once telemetry is back.
  TestCore docked = OnTheStation();
  docked.OnHardwareTimeout();
  docked.OnHardwareStatus(Charging(20.0));
  docked.OnHardwareStatus(Charging(25.0));
  EXPECT_FALSE(Find(docked.TakeEvents(), "DOCKED"));
  // Never charging: the 20 s count from the first report after the gap.
  TestCore core = Ready();
  core.OnHardwareStatus(Charging(10.0, 0.0f));
  core.OnHardwareTimeout();
  core.OnHardwareStatus(Charging(26.0, 0.0f));
  core.OnHardwareStatus(Charging(30.0, 0.0f));
  EXPECT_FALSE(Find(core.TakeEvents(), "CHARGING_LOST"));
  core.OnHardwareStatus(Charging(46.0, 0.0f));
  EXPECT_TRUE(Find(core.TakeEvents(), "CHARGING_LOST"));
}

TEST(ChargingLost, NothingWhileTheMowerIsBusy) {
  TestCore core = OnTheStation();
  ASSERT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kDone);
  core.OnHardwareStatus(At(Healthy(), 20.0));
  core.OnHardwareStatus(At(Healthy(), 40.0));
  EXPECT_FALSE(Find(core.TakeEvents(), "CHARGING_LOST"));
}
