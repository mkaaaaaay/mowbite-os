#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "mowbite_logic/logic_core.h"

using namespace mowbite_logic;

namespace {

const std::string kStart = "mowing.start";
const std::string kAbortMowing = "mowing.abort";
const std::string kAbortUndocking = "undocking.abort";

// The station of the simulation: backing in going west, the mower faces east on the contacts.
const DockPose kDock{-1.0, 0.0, M_PI};
const Ring kLawn{{-0.5, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.5, 2.0}};

HardwareSnapshot Healthy(bool contact = false) {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  hw.dock_contact = contact;
  hw.blade.ok = true;
  hw.left.ok = true;
  hw.right.ok = true;
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

// IDLE at (x, y) with the station and the lawn in the map, the effects so far taken.
LogicCore Ready(double x, double y, bool contact = false) {
  LogicCore core{Tron()};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy(contact));
  core.OnCapabilities(true);
  core.OnPose(PoseAt(x, y, 0.02f, !contact));
  core.OnMowMap({{"lawn", "Lawn", true}}, Reason());
  core.OnMapDock(kDock);
  core.OnDrivableAreas({kLawn}, {});
  core.TakeEffects();
  return core;
}

// A path along y = 1 from x = 1: a transit pose, then lane.
JobPath Lane() {
  JobPath path;
  for (int i = 0; i < 5; ++i) {
    JobPose pose;
    pose.x = 1 + 0.5 * i;
    pose.y = 1;
    pose.kind = i == 0 ? kSegmentTransit : kSegmentLane;
    path.poses.push_back(pose);
  }
  return path;
}

// Mowing on the first path with the blade wanted, the effects so far taken.
LogicCore Mowing() {
  LogicConfig config = Tron();
  config.mow_with_blade = true;
  LogicCore core{config};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(PoseAt(1, 1));
  core.OnMowMap({{"lawn", "Lawn", true}}, Reason());
  core.OnMapDock(kDock);
  core.OnDrivableAreas({kLawn}, {});
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  core.OnJobPlanned({Lane(), Lane()}, "fp");
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kFollow);
  core.OnJobProgress(2);
  core.TakeEffects();
  return core;
}

std::string Name(const Effect& effect) {
  std::string name = EffectName(effect);
  if (const auto* event = std::get_if<Event>(&effect)) name += " " + event->type;
  return name;
}

// The effects as one step of the node carries them out: what halts, what acts, what is recorded, each in the order
// it came up; notices and actions left out.
std::vector<std::string> StepOrder(const std::vector<Effect>& effects) {
  std::vector<std::string> order;
  for (const EffectPhase phase : {EffectPhase::kHalt, EffectPhase::kAct, EffectPhase::kRecord}) {
    for (const auto& effect : effects) {
      if (PhaseOf(effect) != phase || std::holds_alternative<Notice>(effect)) continue;
      if (std::holds_alternative<ActionsChanged>(effect)) continue;
      order.push_back(Name(effect));
    }
  }
  return order;
}

size_t IndexOf(const std::vector<std::string>& order, const std::string& name) {
  const auto it = std::find(order.begin(), order.end(), name);
  EXPECT_NE(it, order.end()) << "no " << name;
  return static_cast<size_t>(it - order.begin());
}

}  // namespace

TEST(Effects, WhatHaltsWhatActsAndWhatIsRecorded) {
  for (const Effect& effect : {Effect(DriveCancel{}), Effect(StopRequest{}), Effect(CalibrationCancel{})}) {
    EXPECT_EQ(PhaseOf(effect), EffectPhase::kHalt) << Name(effect);
  }
  for (const Effect& effect : {Effect(ChargeOff{}), Effect(ChargeOn{}), Effect(ReferenceCheck{}), Effect(UndockPath{}),
                               Effect(UndockOutcome{}), Effect(CalibrationGoal{}), Effect(MapEdit{})}) {
    EXPECT_EQ(PhaseOf(effect), EffectPhase::kAct) << Name(effect);
  }
  for (const Effect& effect : {Effect(Event{}), Effect(ReasonChange{}), Effect(ActionsChanged{}), Effect(Notice{}),
                               Effect(Checkpoint{})}) {
    EXPECT_EQ(PhaseOf(effect), EffectPhase::kRecord) << Name(effect);
  }
}

TEST(Effects, AnEmergencyWhileMowingHaltsBeforeTheCheckpointAndTheEvents) {
  LogicCore core = Mowing();
  ASSERT_TRUE(core.BladeWanted());
  auto hw = Healthy();
  hw.emergency = true;
  hw.emergency_reason = kEmergencyStop;
  core.OnHardwareStatus(hw);
  // The blade goes off in the halt of the same step.
  EXPECT_FALSE(core.BladeWanted());
  const auto effects = core.TakeEffects();
  const auto order = StepOrder(effects);
  ASSERT_GE(order.size(), 4u);
  EXPECT_EQ(order[0], "DriveCancel");
  EXPECT_EQ(order[1], "StopRequest");
  EXPECT_LT(IndexOf(order, "StopRequest"), IndexOf(order, "Checkpoint"));
  EXPECT_LT(IndexOf(order, "StopRequest"), IndexOf(order, "Event EMERGENCY"));
  EXPECT_LT(IndexOf(order, "StopRequest"), IndexOf(order, "ReasonChange"));
  // Each flag once.
  EXPECT_EQ(std::count(order.begin(), order.end(), "StopRequest"), 1);
  EXPECT_EQ(std::count(order.begin(), order.end(), "DriveCancel"), 1);
  EXPECT_TRUE(core.TakeEffects().empty());
}

TEST(Effects, TheWayHomeIsAQueryAfterTheSessionIsWritten) {
  LogicCore core = Ready(2.0, 0.5);
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  core.TakeEffects();
  ASSERT_EQ(core.OnAction(kAbortMowing), ActionResult::kDone);
  ASSERT_TRUE(core.Docking());
  // Nothing in the step moves the mower: the leg comes from TakeDockingLeg once the node wrote the session.
  for (const auto& effect : core.TakeEffects()) EXPECT_NE(PhaseOf(effect), EffectPhase::kAct) << Name(effect);
  const auto leg = core.TakeDockingLeg();
  ASSERT_TRUE(leg);
  EXPECT_EQ(leg->kind, DockingLeg::Kind::kApproach);
}

TEST(Effects, UndockingSwitchesChargingOffThenDrivesThenTellsTheEnd) {
  LogicCore core = Ready(kDock.x, kDock.y, true);
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  auto order = StepOrder(core.TakeEffects());
  ASSERT_EQ(std::count(order.begin(), order.end(), "ReferenceCheck"), 1);
  ASSERT_TRUE(core.OnDockCheck(true, kDock.x + 0.02, kDock.y, 0.02f, "").empty());
  order = StepOrder(core.TakeEffects());
  EXPECT_EQ(order.front(), "ChargeOff");
  EXPECT_EQ(std::count(order.begin(), order.end(), "UndockPath"), 0);
  core.OnChargeOff(true, "charging off");
  order = StepOrder(core.TakeEffects());
  EXPECT_EQ(order, (std::vector<std::string>{"UndockPath"}));
  core.OnHardwareStatus(Healthy(false));
  core.OnPose(PoseAt(kDock.x + 0.8, kDock.y));
  core.TakeEffects();
  core.OnUndockEnded(LegEnd::kDone, Reason("DONE", "done"));
  order = StepOrder(core.TakeEffects());
  // Leaving UNDOCKING stops the wheels once; mowing begins with its own legs.
  ASSERT_GE(order.size(), 2u);
  EXPECT_EQ(order[0], "StopRequest");
  EXPECT_EQ(order[1], "UndockOutcome");
  EXPECT_EQ(std::count(order.begin(), order.end(), "ChargeOn"), 0);
  EXPECT_LT(IndexOf(order, "UndockOutcome"), IndexOf(order, "Event UNDOCKED"));
  EXPECT_TRUE(core.Mowing());
}

TEST(Effects, AnAbortBeforeChargingWentOffDropsItAndAllowsChargingAgain) {
  LogicCore core = Ready(kDock.x, kDock.y, true);
  ASSERT_EQ(core.OnAction(kStart), ActionResult::kDone);
  ASSERT_TRUE(core.OnDockCheck(true, kDock.x + 0.02, kDock.y, 0.02f, "").empty());
  ASSERT_EQ(core.OnAction(kAbortUndocking), ActionResult::kDone);
  const auto order = StepOrder(core.TakeEffects());
  EXPECT_EQ(std::count(order.begin(), order.end(), "ChargeOff"), 0);
  EXPECT_EQ(std::count(order.begin(), order.end(), "UndockPath"), 0);
  EXPECT_EQ(std::count(order.begin(), order.end(), "ChargeOn"), 1);
  EXPECT_EQ(std::count(order.begin(), order.end(), "UndockOutcome"), 1);
}

TEST(Effects, TheCheckpointAndTheReferenceCheckCarryTheirValuesWhenTaken) {
  LogicCore core = Mowing();
  core.OnJobProgress(3);
  ASSERT_EQ(core.OnAction("mowing.pause"), ActionResult::kDone);
  std::string text;
  for (const auto& effect : core.TakeEffects()) {
    if (const auto* checkpoint = std::get_if<Checkpoint>(&effect)) text = checkpoint->text;
  }
  ASSERT_FALSE(text.empty());
  const auto progress = ParseCheckpoint(text);
  ASSERT_TRUE(progress);
  EXPECT_EQ(progress->index, 3);

  LogicCore docked = Ready(kDock.x, kDock.y, true);
  ASSERT_EQ(docked.OnAction(kStart), ActionResult::kDone);
  std::optional<double> heading;
  for (const auto& effect : docked.TakeEffects()) {
    if (const auto* check = std::get_if<ReferenceCheck>(&effect)) heading = check->heading;
  }
  ASSERT_TRUE(heading);
  // The contacts are at the back: the mower faces away from the heading of the station.
  EXPECT_NEAR(std::cos(*heading), 1.0, 1e-9);
}

TEST(Effects, ComeInTheOrderTheyCameUp) {
  LogicCore core = Mowing();
  ASSERT_EQ(core.OnAction("mowing.pause"), ActionResult::kDone);
  ASSERT_EQ(core.OnAction(kAbortMowing), ActionResult::kDone);
  std::vector<std::string> events;
  for (const auto& effect : core.TakeEffects()) {
    if (const auto* event = std::get_if<Event>(&effect)) events.push_back(event->type);
  }
  ASSERT_GE(events.size(), 2u);
  // STATE PAUSED, then DOCKING and its STATE.
  EXPECT_LT(IndexOf(events, "STATE"), IndexOf(events, "DOCKING"));
}
