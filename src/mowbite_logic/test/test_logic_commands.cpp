#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

const std::string kStart = "mower_logic:idle/start_mowing";

// The station of the field: backing in going west, the lawn east of it.
const DockPose kDock{-1.0, 0.0, M_PI};
const Ring kLawn{{-0.5, -2.0}, {4.0, -2.0}, {4.0, 2.0}, {-0.5, 2.0}};
// The station of the start on the contacts: backing in going north, the lawn south of it.
const DockPose kHome{-1.0, 2.0, M_PI / 2};
const Ring kHomeLawn{{-3.0, -3.0}, {3.0, -3.0}, {3.0, 1.9}, {-3.0, 1.9}};

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

HardwareSnapshot InEmergency() {
  HardwareSnapshot hw = Healthy();
  hw.emergency = true;
  hw.emergency_reason = kEmergencyLatch | kEmergencyStop;
  return hw;
}

PoseSnapshot PoseAt(double x, double y, double heading = M_PI, bool valid = true) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.heading = heading;
  pose.position_accuracy = 0.02f;
  pose.orientation_accuracy = valid ? 0.01f : 999.0f;
  pose.orientation_valid = valid;
  return pose;
}

LogicConfig Tron() {
  LogicConfig config;
  config.dock_contact_x = -0.10;
  return config;
}

void Settle(TestCore& core) {
  core.TakeEvents();
  core.TakeNotices();
  core.TakeActionsChanged();
  core.TakeDriveCancel();
  core.TakeStopRequest();
  core.TakeCheckpoint();
  core.TakeMapEdit();
}

// IDLE in the field, 2 m east of the station, with the station and the lawn in the map.
TestCore Idle() {
  TestCore core{Tron()};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(PoseAt(2.0, 0.5));
  core.OnMowMap({{"lawn", "Lawn", true}}, Reason());
  core.OnMapDock(kDock);
  core.OnDrivableAreas({kLawn}, {});
  Settle(core);
  return core;
}

TestCore Mowing() {
  TestCore core = Idle();
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  Settle(core);
  return core;
}

// Mowing a lane in front of the mower: a skip waits for the leg to stop.
TestCore Following() {
  TestCore core = Mowing();
  EXPECT_EQ(core.NextJobStep(0).kind, JobStep::Kind::kPlan);
  JobPath path;
  for (int i = 0; i < 5; ++i) {
    JobPose pose;
    pose.x = 2.0 + 0.5 * i;
    pose.y = 0.5;
    pose.kind = i == 0 ? kSegmentTransit : kSegmentLane;
    path.poses.push_back(pose);
  }
  core.OnJobPlanned({path, path}, "fp");
  const JobStep step = core.NextJobStep(0);
  EXPECT_TRUE(step.kind == JobStep::Kind::kFollow || step.kind == JobStep::Kind::kApproach);
  Settle(core);
  return core;
}

TestCore Paused() {
  TestCore core = Mowing();
  EXPECT_EQ(core.OnAction("mower_logic:mowing/pause"), ActionResult::kDone);
  Settle(core);
  return core;
}

TestCore PausedInEmergency() {
  TestCore core = Mowing();
  core.OnHardwareStatus(InEmergency());
  Settle(core);
  return core;
}

TestCore Held(TestCore core) {
  core.SetHold(2000000000);
  Settle(core);
  return core;
}

// After abort_mowing on a planned path the mower goes home; the job stays unfinished.
TestCore Docking() {
  TestCore core = Following();
  EXPECT_EQ(core.OnAction("mower_logic:mowing/abort_mowing"), ActionResult::kDone);
  EXPECT_TRUE(core.Docking());
  Settle(core);
  return core;
}

TestCore IdleWithJob() {
  TestCore core = Docking();
  EXPECT_EQ(core.OnAction("mower_logic:docking/abort_docking"), ActionResult::kDone);
  Settle(core);
  return core;
}

TestCore Recording() {
  TestCore core = Idle();
  EXPECT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kDone);
  Settle(core);
  return core;
}

// Recording a ring of three points.
TestCore Ring3() {
  TestCore core = Recording();
  EXPECT_EQ(core.OnAction("mower_logic:area_recording/start_recording"), ActionResult::kDone);
  core.OnPose(PoseAt(2.0, 0.5));
  core.OnPose(PoseAt(3.0, 0.5));
  core.OnPose(PoseAt(3.0, 1.5));
  Settle(core);
  return core;
}

TestCore Unwritable() {
  TestCore core = Idle();
  core.OnMapStatus(MapDatum::kMismatch, "");
  Settle(core);
  return core;
}

// On the contacts of the recorded station, the heading not aligned yet, as after a start there.
TestCore OnDock() {
  TestCore core{Tron()};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy(true));
  core.OnCapabilities(true);
  core.OnPose(PoseAt(kHome.x, kHome.y, 0, false));
  core.OnMowMap({{"front", "Front", true}}, Reason());
  core.OnMapDock(kHome);
  core.OnDrivableAreas({kHomeLawn}, {});
  Settle(core);
  return core;
}

TestCore Checking() {
  TestCore core = OnDock();
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_TRUE(core.DockCheckPending());
  Settle(core);
  return core;
}

TestCore Undocking() {
  TestCore core = Checking();
  core.TakeDockCheck();
  EXPECT_TRUE(core.OnDockCheck(true, kHome.x + 0.03, kHome.y, 0.022f, "").empty());
  EXPECT_TRUE(core.Undocking());
  Settle(core);
  return core;
}

struct Scenario {
  const char* name;
  std::function<TestCore()> make;
};

const std::vector<Scenario>& Scenarios() {
  static const std::vector<Scenario> scenarios = {
      {"idle", Idle},
      {"idle_job", IdleWithJob},
      {"mowing", Mowing},
      {"following", Following},
      {"paused", Paused},
      {"paused_emergency", PausedInEmergency},
      {"held_idle", [] { return Held(Idle()); }},
      {"held_paused", [] { return Held(Paused()); }},
      {"emergency_idle",
       [] {
         TestCore core = Idle();
         core.OnHardwareStatus(InEmergency());
         Settle(core);
         return core;
       }},
      {"docking", Docking},
      {"recording", Recording},
      {"ring", Ring3},
      {"unwritable", Unwritable},
      {"on_dock", OnDock},
      {"checking", Checking},
      {"undocking", Undocking},
  };
  return scenarios;
}

bool Enabled(const TestCore& core, const std::string& action_id) {
  for (const auto& group : core.Actions()) {
    for (const auto& action : group.actions) {
      if (group.prefix + "/" + action.id == action_id) return action.enabled;
    }
  }
  ADD_FAILURE() << "missing action " << action_id;
  return false;
}

// The details of an event without what differs between two cores or two ways: random ids and the client.
json Comparable(json details) {
  for (const char* key : {"job_id", "session_id", "client"}) details.erase(key);
  return details;
}

const Event* Find(const std::vector<Event>& events, const std::string& type) {
  auto it = std::find_if(events.begin(), events.end(), [&](const Event& e) { return e.type == type; });
  return it == events.end() ? nullptr : &*it;
}

CommandOutcome Rpc(TestCore& core, const std::string& name, json params = json::object(),
                   const Reason& node_problem = Reason()) {
  params["client"] = "app";
  return core.Command(name, params, node_problem);
}

}  // namespace

// One way: every action and its RPC do the same in every state, events and all, but for client.
TEST(LogicCommands, ActionAndRpcAlikeInEveryState) {
  int compared = 0;
  for (const auto& scenario : Scenarios()) {
    for (const auto& command : mowbite_api::Commands()) {
      for (const auto& entry : command.actions) {
        const std::string where = std::string(scenario.name) + " " + entry.action;
        TestCore by_action = scenario.make();
        TestCore by_rpc = scenario.make();
        const bool enabled = Enabled(by_action, entry.action);
        const ActionResult result = by_action.OnAction(entry.action);
        const CommandOutcome outcome = Rpc(by_rpc, command.name, entry.params);
        ASSERT_TRUE(outcome.invalid.empty()) << where << ": " << outcome.invalid;
        const ActionResult same = outcome.release    ? ActionResult::kReleaseEmergency
                                  : outcome.accepted ? ActionResult::kDone
                                                     : ActionResult::kRejected;
        EXPECT_EQ(result, same) << where;
        // WRONG_STATE: the action would not have been enabled.
        if (outcome.why.code == "WRONG_STATE") {
          EXPECT_FALSE(enabled) << where;
        }
        const auto action_events = by_action.TakeEvents();
        const auto rpc_events = by_rpc.TakeEvents();
        ASSERT_EQ(action_events.size(), rpc_events.size()) << where;
        for (size_t i = 0; i < rpc_events.size(); i++) {
          EXPECT_EQ(action_events[i].type, rpc_events[i].type) << where;
          EXPECT_EQ(Comparable(action_events[i].details), Comparable(rpc_events[i].details)) << where;
          EXPECT_FALSE(action_events[i].details.contains("client")) << where;
          EXPECT_EQ(rpc_events[i].details.value("client", ""), "app") << where << " " << rpc_events[i].type;
        }
        EXPECT_EQ(by_action.RobotState().state, by_rpc.RobotState().state) << where;
        EXPECT_EQ(by_action.RobotState().sub_state, by_rpc.RobotState().sub_state) << where;
        for (const auto& group : by_rpc.Actions()) {
          for (const auto& action : group.actions) {
            EXPECT_EQ(Enabled(by_action, group.prefix + "/" + action.id), action.enabled)
                << where << ": " << action.id;
          }
        }
        EXPECT_EQ(by_action.TakeDriveCancel().has_value(), by_rpc.TakeDriveCancel().has_value()) << where;
        EXPECT_EQ(by_action.TakeStopRequest(), by_rpc.TakeStopRequest()) << where;
        EXPECT_EQ(by_action.TakeCheckpoint().has_value(), by_rpc.TakeCheckpoint().has_value()) << where;
        EXPECT_EQ(by_action.TakeMapEdit().has_value(), by_rpc.TakeMapEdit().has_value()) << where;
        EXPECT_EQ(by_action.DockCheckPending(), by_rpc.DockCheckPending()) << where;
        // The answer: state as robot_state has it now; a refusal names its group and the event it went out as.
        if (outcome.accepted) {
          EXPECT_EQ(outcome.result["state"], by_rpc.RobotState().state) << where;
          EXPECT_EQ(outcome.result["sub_state"], by_rpc.RobotState().sub_state) << where;
          // released comes from the node with the answer of the safety node.
          for (const auto& field : command.result) {
            if (field.required && !outcome.release) {
              EXPECT_TRUE(outcome.result.contains(field.name)) << where << ": " << field.name;
            }
          }
        } else {
          EXPECT_EQ(outcome.data["reason"], outcome.why.code) << where;
          const std::string group = outcome.data["event"];
          EXPECT_TRUE(group == command.refused || group == "COMMAND_REFUSED") << where << " " << group;
          EXPECT_NE(mowbite_api::FindReasonGroup(group), nullptr) << where;
          if (outcome.data.contains("event_id")) {
            const auto event = std::find_if(rpc_events.begin(), rpc_events.end(),
                                            [&](const Event& e) { return e.id == outcome.data["event_id"]; });
            ASSERT_NE(event, rpc_events.end()) << where;
            EXPECT_EQ(event->type, group) << where;
          }
        }
        compared++;
      }
    }
  }
  EXPECT_EQ(compared, 22 * static_cast<int>(Scenarios().size()));
}

TEST(LogicCommands, ARefusalNamesTheEventItWentOutAs) {
  TestCore core = Idle();
  core.OnHardwareStatus(InEmergency());
  Settle(core);
  const CommandOutcome outcome = Rpc(core, "mowing.start");
  EXPECT_FALSE(outcome.accepted);
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "MOWING_REFUSED");
  EXPECT_EQ(events[0].details["client"], "app");
  EXPECT_EQ(outcome.data["event"], "MOWING_REFUSED");
  EXPECT_EQ(outcome.data["reason"], "EMERGENCY");
  EXPECT_EQ(outcome.data["severity"], "error");
  EXPECT_EQ(outcome.data["resume"], "user");
  EXPECT_EQ(outcome.data["state"], "IDLE");
  EXPECT_EQ(outcome.data["sub_state"], "");
  EXPECT_EQ(outcome.data["event_id"], events[0].id);
  EXPECT_EQ(events[0].id.size(), 32u);
}

TEST(LogicCommands, WrongStateGoesOutOnlyAsTheAnswer) {
  TestCore core = Idle();
  for (const char* name : {"mowing.pause", "mowing.continue", "mowing.abort", "mowing.skip_area", "mowing.skip_path",
                           "undocking.abort", "docking.abort", "recording.exit", "recording.ring_start"}) {
    const CommandOutcome outcome = Rpc(core, name);
    EXPECT_FALSE(outcome.accepted) << name;
    EXPECT_EQ(outcome.why.code, "WRONG_STATE") << name;
    EXPECT_EQ(outcome.data,
              json::parse(R"({"event": "COMMAND_REFUSED", "reason": "WRONG_STATE", "state": "IDLE", "sub_state": "",
                              "message": "Not now: the mower is IDLE", "severity": "info", "resume": "user"})"))
        << name << " " << outcome.data.dump();
  }
  EXPECT_TRUE(core.TakeEvents().empty());
  // While recording, a step the recorder has off: WRONG_STATE with its sub state.
  TestCore recording = Recording();
  const CommandOutcome outcome = Rpc(recording, "recording.ring_stop");
  EXPECT_EQ(outcome.why.code, "WRONG_STATE");
  EXPECT_EQ(outcome.data["state"], "AREA_RECORDING");
  EXPECT_TRUE(recording.TakeEvents().empty());
}

TEST(LogicCommands, ResetJobOnlyWithAJob) {
  TestCore core = Idle();
  CommandOutcome outcome = Rpc(core, "mowing.reset_job");
  EXPECT_EQ(outcome.why.code, "NO_JOB");
  EXPECT_EQ(outcome.data["event"], "COMMAND_REFUSED");
  EXPECT_FALSE(outcome.data.contains("event_id"));
  core = IdleWithJob();
  outcome = Rpc(core, "mowing.reset_job");
  ASSERT_TRUE(outcome.accepted);
  const auto events = core.TakeEvents();
  const Event* reset = Find(events, "JOB_RESET");
  ASSERT_TRUE(reset);
  EXPECT_EQ(outcome.result["job_id"], reset->details["job_id"]);
  EXPECT_EQ(outcome.result["state"], "IDLE");
  EXPECT_EQ(reset->details["client"], "app");
  EXPECT_EQ(Rpc(core, "mowing.reset_job").why.code, "NO_JOB");
}

TEST(LogicCommands, SkipOnlyWhatTheClientSaw) {
  TestCore core = Following();
  ASSERT_EQ(core.RobotState().current_area, 0);
  CommandOutcome outcome = Rpc(core, "mowing.skip_area", {{"area", 1}});
  EXPECT_EQ(outcome.why.code, "CHANGED");
  EXPECT_EQ(outcome.data["area"], 0);
  EXPECT_EQ(outcome.data["event"], "COMMAND_REFUSED");
  EXPECT_FALSE(core.TakeDriveCancel().has_value());
  outcome = Rpc(core, "mowing.skip_path", {{"path", 3}});
  EXPECT_EQ(outcome.why.code, "CHANGED");
  EXPECT_EQ(outcome.data["path"], core.RobotState().current_path);
  EXPECT_TRUE(core.TakeEvents().empty());
  // The area it saw: skipped once the leg stopped, AREA_SKIPPED with the client of the skip.
  outcome = Rpc(core, "mowing.skip_area", {{"area", 0}});
  ASSERT_TRUE(outcome.accepted);
  EXPECT_EQ(outcome.result["area"], 0);
  EXPECT_TRUE(core.TakeDriveCancel().has_value());
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnLegEnded(LegEnd::kCanceled, 0, Reason("STOPPED", "Skipped by the app"));
  const auto events = core.TakeEvents();
  const Event* skipped = Find(events, "AREA_SKIPPED");
  ASSERT_TRUE(skipped);
  EXPECT_EQ(skipped->details["client"], "app");
}

TEST(LogicCommands, ParamsThatDoNotFitChangeNothing) {
  TestCore core = Idle();
  CommandOutcome outcome = core.Command("mowing.start", {{"areas", {"lawn"}}});
  EXPECT_EQ(outcome.invalid_field, "areas");
  EXPECT_FALSE(outcome.accepted);
  outcome = core.Command("mowing.start", {{"client", "App"}});
  EXPECT_EQ(outcome.invalid_field, "client");
  outcome = core.Command("recording.save", json::object());
  EXPECT_EQ(outcome.invalid_field, "type");
  outcome = core.Command("mowing.start", {{"speed", 1}});
  EXPECT_EQ(outcome.invalid_field, "speed");
  EXPECT_FALSE(core.Command("no.such", json::object()).invalid.empty());
  EXPECT_FALSE(core.Mowing());
  EXPECT_TRUE(core.TakeEvents().empty());
}

TEST(LogicCommands, StartAnswersWithTheState) {
  TestCore core = Idle();
  CommandOutcome outcome = Rpc(core, "mowing.start");
  ASSERT_TRUE(outcome.accepted);
  EXPECT_EQ(outcome.result, json::parse(R"({"state": "MOWING", "sub_state": "", "check": false})"));
  // On the station the reference check comes first.
  core = OnDock();
  outcome = Rpc(core, "mowing.start");
  ASSERT_TRUE(outcome.accepted);
  EXPECT_EQ(outcome.result, json::parse(R"({"state": "IDLE", "sub_state": "", "check": true})"));
  // What only the node knows comes after the logic's own reasons, as MOWING_REFUSED.
  core = Idle();
  outcome = Rpc(core, "mowing.start", json::object(), Reason("PLANNER_MISSING", "The planner is not running"));
  EXPECT_EQ(outcome.why.code, "PLANNER_MISSING");
  EXPECT_EQ(outcome.data["event"], "MOWING_REFUSED");
  EXPECT_TRUE(outcome.data.contains("event_id"));
  core.OnHardwareStatus(InEmergency());
  core.TakeEvents();
  outcome = Rpc(core, "mowing.start", json::object(), Reason("PLANNER_MISSING", "The planner is not running"));
  EXPECT_EQ(outcome.why.code, "EMERGENCY");
}

TEST(LogicCommands, DockingStart) {
  TestCore core = Idle();
  CommandOutcome outcome = Rpc(core, "docking.start");
  ASSERT_TRUE(outcome.accepted) << outcome.why.code;
  EXPECT_EQ(outcome.result, json::parse(R"({"state": "DOCKING", "sub_state": ""})"));
  core = Idle();
  outcome = Rpc(core, "docking.start", json::object(), Reason("STILL_STOPPING", "The last drive is still stopping"));
  EXPECT_EQ(outcome.data["event"], "DRIVE_REFUSED");
  EXPECT_EQ(outcome.data["reason"], "STILL_STOPPING");
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(outcome.data["event_id"], events[0].id);
}

TEST(LogicCommands, TheHoldRefusesEveryStartWithHold) {
  TestCore core = Held(Idle());
  for (const auto& [name, group] : std::vector<std::pair<std::string, std::string>>{
           {"mowing.start", "MOWING_REFUSED"},
           {"recording.start", "RECORDING_REFUSED"},
           {"docking.start", "DRIVE_REFUSED"}}) {
    const CommandOutcome outcome = Rpc(core, name);
    EXPECT_EQ(outcome.why.code, "HOLD") << name;
    EXPECT_EQ(outcome.data["event"], group) << name;
    EXPECT_EQ(outcome.data["until"], 2000000000) << name;
    EXPECT_TRUE(outcome.data.contains("event_id")) << name;
  }
  core = Held(Paused());
  const CommandOutcome outcome = Rpc(core, "mowing.continue");
  EXPECT_EQ(outcome.data["event"], "MOWING_REFUSED");
  EXPECT_EQ(outcome.why.code, "HOLD");
  EXPECT_EQ(core.RobotState().state, "PAUSED");
}

TEST(LogicCommands, InAnEmergencyNoSecondEvent) {
  TestCore core = PausedInEmergency();
  CommandOutcome outcome = Rpc(core, "mowing.continue");
  EXPECT_EQ(outcome.why.code, "EMERGENCY");
  EXPECT_EQ(outcome.data["event"], "MOWING_REFUSED");
  EXPECT_FALSE(outcome.data.contains("event_id"));
  core = Idle();
  core.OnHardwareStatus(InEmergency());
  core.TakeEvents();
  outcome = Rpc(core, "recording.start");
  EXPECT_EQ(outcome.why.code, "EMERGENCY");
  EXPECT_EQ(outcome.data["event"], "RECORDING_REFUSED");
  EXPECT_EQ(outcome.data["severity"], "error");
  EXPECT_TRUE(core.TakeEvents().empty());
}

TEST(LogicCommands, RecordingRefusalCarriesItsAction) {
  TestCore core = Recording();
  // Without an outline the recorder has saving off: WRONG_STATE, no event.
  CommandOutcome outcome = Rpc(core, "recording.save", {{"type", "mow"}});
  EXPECT_EQ(outcome.why.code, "WRONG_STATE");
  core = Ring3();
  core.OnMapStatus(MapDatum::kMismatch, "");
  core.TakeEvents();
  outcome = Rpc(core, "recording.save", {{"type", "nav"}});
  EXPECT_EQ(outcome.why.code, "MAP_MISMATCH");
  EXPECT_EQ(outcome.data["event"], "RECORDING_REFUSED");
  EXPECT_EQ(outcome.data["action"], "finish_navigation_area");
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(outcome.data["event_id"], events[0].id);
  EXPECT_EQ(events[0].details["client"], "app");
}

TEST(LogicCommands, UndoTakesMetresWithinItsRange) {
  TestCore core = Ring3();
  for (double meters : {0.05, 50.5, -1.0}) {
    const CommandOutcome outcome = Rpc(core, "recording.undo", {{"meters", meters}});
    EXPECT_FALSE(outcome.invalid.empty()) << meters;
    EXPECT_EQ(outcome.invalid_field, "meters");
  }
  EXPECT_EQ(core.recorder().current().size(), 3u);
  // From (3, 1.5) back 0.5 m: one point, the segment before it is 1 m.
  CommandOutcome outcome = Rpc(core, "recording.undo", {{"meters", 0.5}});
  ASSERT_TRUE(outcome.accepted);
  EXPECT_EQ(outcome.result["points"], 2);
  EXPECT_EQ(outcome.result["state"], "AREA_RECORDING");
  EXPECT_FALSE(Enabled(core, "mower_logic:area_recording/auto_point_collecting_disable"));
  outcome = Rpc(core, "recording.undo");
  EXPECT_EQ(outcome.result["points"], 1);
  outcome = Rpc(core, "recording.ring_stop");
  EXPECT_EQ(outcome.why.code, "RING_FEW_POINTS");
  EXPECT_EQ(Rpc(core, "recording.undo").result["points"], 0);
  outcome = Rpc(core, "recording.ring_stop");
  ASSERT_TRUE(outcome.accepted);
  EXPECT_EQ(outcome.result["trimmed_points"], 0);
}

// The end of the emergency after emergency.reset tells who released it (masterplan 3.1a).
TEST(LogicCommands, ReleaseCarriesSourceAndClient) {
  TestCore core = Idle();
  core.OnHardwareStatus(InEmergency());
  core.TakeEvents();
  CommandOutcome outcome = Rpc(core, "emergency.reset");
  EXPECT_TRUE(outcome.accepted);
  EXPECT_TRUE(outcome.release);
  core.OnReleaseAnswered(10.0);
  core.OnHardwareStatus(Healthy());
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["emergency"], false);
  EXPECT_EQ(events[0].details["source"], "rpc");
  EXPECT_EQ(events[0].details["client"], "app");
  // By the action: source rpc without a client.
  core.OnHardwareStatus(InEmergency());
  core.TakeEvents();
  EXPECT_EQ(core.OnAction("mower_logic/reset_emergency"), ActionResult::kReleaseEmergency);
  core.OnHardwareStatus(Healthy());
  events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["source"], "rpc");
  EXPECT_FALSE(events[0].details.contains("client"));
  // No end within kReleaseSourceS of the answer: refused with STILL_ACTIVE; an end after that does not say who.
  core.OnHardwareStatus(InEmergency());
  core.TakeEvents();
  Rpc(core, "emergency.reset");
  core.OnReleaseAnswered(20.0);
  core.OnTick(20.0 + kReleaseSourceS - 0.1);
  EXPECT_TRUE(core.TakeEvents().empty());
  core.OnTick(20.0 + kReleaseSourceS + 0.1);
  events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "EMERGENCY_RESET_REFUSED");
  EXPECT_EQ(events[0].details["reason"], "STILL_ACTIVE");
  EXPECT_EQ(events[0].details["source"], "rpc");
  EXPECT_EQ(events[0].details["client"], "app");
  EXPECT_EQ(events[0].details["severity"], "warning");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_EQ(events[0].details["bits"], json::array({"LATCH", "STOP"}));
  core.OnHardwareStatus(Healthy());
  events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_FALSE(events.back().details.contains("source"));
  // A refusal is the event with the client of the request, and its id for the answer.
  core.OnHardwareStatus(InEmergency());
  core.TakeEvents();
  Rpc(core, "emergency.reset");
  const std::string id = core.RefuseReset(Reason("STILL_TILTED", "Still tilted"));
  events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].id, id);
  EXPECT_EQ(events[0].details["client"], "app");
  const json data = core.RefusalData("EMERGENCY_RESET_REFUSED", Reason("STILL_TILTED", "Still tilted"), id);
  EXPECT_EQ(data["severity"], "warning");
  EXPECT_EQ(data["event_id"], id);
  // After the refusal the end of the emergency has no source.
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.TakeEvents().back().details.contains("source"));
}

// STOP let go after the gesture of the driver: released like emergency.reset, with source button.
TEST(LogicCommands, StopAtTheMowerReleasesWithSourceButton) {
  TestCore core = Idle();
  // Outside an emergency there is nothing to release.
  EXPECT_FALSE(core.OnStopRelease());
  core.OnHardwareStatus(InEmergency());
  core.TakeEvents();
  EXPECT_TRUE(core.OnStopRelease());
  core.OnReleaseAnswered(10.0);
  core.OnHardwareStatus(Healthy());
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["emergency"], false);
  EXPECT_EQ(events[0].details["source"], "button");
  EXPECT_FALSE(events[0].details.contains("client"));
  // A refusal of the driver (LATCH_CHANGED) or S2 carries the source.
  core.OnHardwareStatus(InEmergency());
  core.TakeEvents();
  EXPECT_TRUE(core.OnStopRelease());
  core.RefuseReset(Reason("LATCH_CHANGED", "another reason latched since the press"));
  events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], "LATCH_CHANGED");
  EXPECT_EQ(events[0].details["source"], "button");
  EXPECT_EQ(events[0].details["severity"], "warning");
  EXPECT_FALSE(events[0].details.contains("client"));
  // Both at once: the latest request names the end.
  EXPECT_TRUE(core.OnStopRelease());
  Rpc(core, "emergency.reset");
  core.OnHardwareStatus(Healthy());
  EXPECT_EQ(core.TakeEvents().back().details["source"], "rpc");
  core.OnHardwareStatus(InEmergency());
  Rpc(core, "emergency.reset");
  EXPECT_TRUE(core.OnStopRelease());
  core.OnReleaseAnswered(30.0);
  core.OnTick(30.0 + kReleaseSourceS + 0.1);
  events = core.TakeEvents();
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.back().details["reason"], "STILL_ACTIVE");
  EXPECT_EQ(events.back().details["source"], "button");
}

// The release by STOP starts nothing: mowing stays paused until continued (G11).
TEST(LogicCommands, StopAtTheMowerKeepsMowingPaused) {
  TestCore core = PausedInEmergency();
  ASSERT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_TRUE(core.OnStopRelease());
  core.OnReleaseAnswered(1.0);
  core.OnHardwareStatus(Healthy());
  Settle(core);
  EXPECT_EQ(core.RobotState().state, "PAUSED");
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_FALSE(core.FollowerActive());
}

TEST(LogicCommands, AnEmergencyEndsWithASourceOnlyAfterItsOwnRequest) {
  TestCore core = Idle();
  core.OnHardwareStatus(InEmergency());
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.TakeEvents().back().details.contains("source"));
  // A request that ended one emergency does not tell the end of the next.
  core.OnHardwareStatus(InEmergency());
  Rpc(core, "emergency.reset");
  core.OnHardwareStatus(Healthy());
  EXPECT_EQ(core.TakeEvents().back().details["source"], "rpc");
  core.OnHardwareStatus(InEmergency());
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.TakeEvents().back().details.contains("source"));
}

// I2: start_mowing carries a reason exactly while it is off, the one MOWING_REFUSED gets when it is pressed anyway.
TEST(LogicCommands, StartMowingShowsTheReasonOfItsRefusal) {
  const mowbite_api::EventEntry* refused = mowbite_api::FindEvent("MOWING_REFUSED");
  ASSERT_NE(refused, nullptr);
  std::set<std::string> reasons;
  for (const auto& scenario : Scenarios()) {
    TestCore core = scenario.make();
    const Action* start = nullptr;
    std::vector<ActionGroup> groups = core.Actions();
    for (const auto& group : groups) {
      for (const auto& action : group.actions) {
        EXPECT_TRUE(action.reason.empty() || action.id == "start_mowing") << scenario.name << " " << action.id;
        if (group.prefix == "mower_logic:idle" && action.id == "start_mowing") start = &action;
      }
    }
    ASSERT_NE(start, nullptr) << scenario.name;
    EXPECT_EQ(start->enabled, start->reason.empty()) << scenario.name << " " << start->reason;
    if (start->enabled) continue;
    reasons.insert(start->reason);
    EXPECT_TRUE(std::any_of(refused->reasons.begin(), refused->reasons.end(),
                            [&](const mowbite_api::ReasonEntry& r) { return r.code == start->reason; }))
        << scenario.name << " " << start->reason;
    core.TakeEvents();
    EXPECT_EQ(core.OnAction("mower_logic:idle/start_mowing"), ActionResult::kRejected) << scenario.name;
    const auto events = core.TakeEvents();
    const auto event = std::find_if(events.begin(), events.end(), [](const Event& e) { return e.type == "MOWING_REFUSED"; });
    ASSERT_NE(event, events.end()) << scenario.name;
    EXPECT_EQ(event->details.value("reason", ""), start->reason) << scenario.name;
  }
  // Busy, held and in an emergency at least.
  EXPECT_TRUE(reasons.count("BUSY"));
  EXPECT_TRUE(reasons.count("HOLD"));
  EXPECT_TRUE(reasons.count("EMERGENCY"));
}


namespace {

// The types of the events, in order.
std::vector<std::string> TypesOf(const std::vector<Event>& events) {
  std::vector<std::string> types;
  for (const auto& event : events) types.push_back(event.type);
  return types;
}

// The reason of the first event of this type, empty without one.
std::string ReasonOf(const std::vector<Event>& events, const std::string& type) {
  for (const auto& event : events) {
    if (event.type == type) return event.details.value("reason", "");
  }
  return "";
}

}  // namespace

// G35 with a disk that cannot be written: while os-session.json cannot, no start from IDLE, each refused with
// STORAGE_NOT_WRITABLE in the refusal of its kind; the event STORAGE tells it once.
TEST(LogicStorage, NothingStartsFromIdleWhileTheSessionCannotBeWritten) {
  TestCore core = Idle();
  core.SetUnwritableFile("os-session.json");
  auto events = core.TakeEvents();
  ASSERT_EQ(TypesOf(events), std::vector<std::string>{"STORAGE"});
  EXPECT_EQ(events[0].details["reason"], "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(events[0].details["file"], "os-session.json");
  EXPECT_EQ(events[0].details["severity"], "error");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_TRUE(core.TakeActionsChanged());
  EXPECT_EQ(core.StartProblem().code, "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(core.Command("mowing.start", json::object()).why.code, "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(ReasonOf(core.TakeEvents(), "MOWING_REFUSED"), "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(core.Command("docking.start", json::object()).why.code, "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(ReasonOf(core.TakeEvents(), "DRIVE_REFUSED"), "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(core.StartDrive(3.0, 0.5).code, "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(ReasonOf(core.TakeEvents(), "DRIVE_REFUSED"), "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(core.Command("recording.start", json::object()).why.code, "STORAGE_NOT_WRITABLE");
  events = core.TakeEvents();
  EXPECT_EQ(ReasonOf(events, "RECORDING_REFUSED"), "STORAGE_NOT_WRITABLE");
  EXPECT_EQ(events[0].details["file"], "os-session.json");
  EXPECT_TRUE(core.Idle());
  EXPECT_FALSE(core.FollowerActive());
  for (const auto& group : core.Actions()) {
    for (const auto& action : group.actions) {
      if (action.id == "start_mowing") {
        EXPECT_EQ(action.reason, "STORAGE_NOT_WRITABLE");
      }
      if (action.id == "start_area_recording") {
        EXPECT_FALSE(action.enabled);
      }
    }
  }
  // Another file while it lasts is the same incident; written again, starts go.
  core.SetUnwritableFile("restart-hold.json");
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_TRUE(core.TakeNotices().empty());
  EXPECT_EQ(core.StartProblem().fields.value("file", ""), "restart-hold.json");
  core.SetUnwritableFile("");
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_EQ(core.TakeNotices(), std::vector<std::string>{"data_dir can be written again: starts are allowed"});
  EXPECT_TRUE(core.TakeActionsChanged());
  // Told again what it knows: nothing new.
  core.SetUnwritableFile("");
  EXPECT_TRUE(core.TakeNotices().empty());
  EXPECT_TRUE(core.StartProblem().empty()) << core.StartProblem().message;
  EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
  EXPECT_TRUE(core.Mowing());
  // A new incident tells again.
  Settle(core);
  core.SetUnwritableFile("os-session.json");
  EXPECT_EQ(TypesOf(core.TakeEvents()), std::vector<std::string>{"STORAGE"});
}

// The disk turns read only during a job: the record of its start holds on the disk already, so the job goes on and
// stops as always; only a new start from IDLE waits.
TEST(LogicStorage, AJobGoesOnWhenTheDiskFailsMeanwhile) {
  TestCore core = Following();
  core.SetUnwritableFile("os-session.json");
  EXPECT_EQ(TypesOf(core.TakeEvents()), std::vector<std::string>{"STORAGE"});
  EXPECT_TRUE(core.Mowing());
  EXPECT_FALSE(core.TakeDriveCancel().has_value());
  EXPECT_FALSE(core.TakeStopRequest());
  EXPECT_EQ(core.NextJobStep(1).kind, JobStep::Kind::kNone);
  // A pause and its continue stay in the job.
  EXPECT_EQ(core.OnAction("mower_logic:mowing/pause"), ActionResult::kDone);
  Settle(core);
  EXPECT_EQ(core.OnAction("mower_logic:mowing/continue"), ActionResult::kDone);
  EXPECT_TRUE(core.Mowing());
  EXPECT_EQ(core.RobotState().state, "MOWING");
}

// The record of a start did not reach the disk: the start ends at once, with the refusal of its kind, and nothing it
// asked of the node is left; the mower is IDLE.
TEST(LogicStorage, AStartWhoseRecordFailedEndsAtOnce) {
  {
    TestCore core = Idle();
    EXPECT_EQ(core.OnAction(kStart), ActionResult::kDone);
    Settle(core);
    core.AbandonStart("os-session.json");
    const auto events = core.TakeEvents();
    EXPECT_EQ(TypesOf(events), (std::vector<std::string>{"STORAGE", "MOWING_REFUSED", "STATE"}));
    EXPECT_EQ(ReasonOf(events, "MOWING_REFUSED"), "STORAGE_NOT_WRITABLE");
    EXPECT_TRUE(core.Idle());
    EXPECT_EQ(core.NextJobStep(1).kind, JobStep::Kind::kNone);
    EXPECT_EQ(core.StartProblem().code, "STORAGE_NOT_WRITABLE");
  }
  {
    // What the start asked for the node took already, as in its step.
    TestCore core = Undocking();
    core.TakeChargeOff();
    core.TakeUndockPath();
    core.AbandonStart("os-session.json");
    const auto events = core.TakeEvents();
    EXPECT_EQ(TypesOf(events), (std::vector<std::string>{"STORAGE", "MOWING_REFUSED", "STATE"}));
    EXPECT_EQ(ReasonOf(events, "MOWING_REFUSED"), "STORAGE_NOT_WRITABLE");
    EXPECT_FALSE(core.TakeDriveCancel().has_value());
    EXPECT_TRUE(core.Idle());
    EXPECT_FALSE(core.Undocking());
    EXPECT_FALSE(core.TakeUndockPath().has_value());
    EXPECT_FALSE(core.TakeChargeOff());
    const auto outcome = core.TakeUndockOutcome();
    ASSERT_TRUE(outcome.has_value());
    EXPECT_FALSE(outcome->undocked);
    EXPECT_EQ(outcome->why.code, "STORAGE_NOT_WRITABLE");
  }
  {
    TestCore core = OnDock();
    core.CheckDockBeforeDrive(0.0, -1.0);
    core.TakeDockCheck();
    EXPECT_TRUE(core.OnDockCheck(true, kHome.x + 0.03, kHome.y, 0.022f, "").empty());
    ASSERT_TRUE(core.Undocking());
    Settle(core);
    core.TakeChargeOff();
    core.TakeUndockPath();
    core.AbandonStart("os-session.json");
    const auto events = core.TakeEvents();
    EXPECT_EQ(ReasonOf(events, "DRIVE_REFUSED"), "STORAGE_NOT_WRITABLE");
    EXPECT_EQ(ReasonOf(events, "MOWING_REFUSED"), "");
    const auto outcome = core.TakeUndockOutcome();
    ASSERT_TRUE(outcome.has_value());
    EXPECT_TRUE(outcome->for_drive);
    EXPECT_EQ(outcome->why.code, "STORAGE_NOT_WRITABLE");
    EXPECT_TRUE(core.Idle());
  }
  {
    TestCore core = Idle();
    ASSERT_TRUE(core.StartDrive(3.0, 0.5).empty());
    Settle(core);
    core.AbandonStart("os-session.json");
    const auto events = core.TakeEvents();
    EXPECT_EQ(TypesOf(events), (std::vector<std::string>{"STORAGE", "DRIVE_REFUSED", "DRIVE", "STATE"}));
    EXPECT_EQ(events[2].details["phase"], "end");
    EXPECT_EQ(events[2].details["arrived"], false);
    EXPECT_EQ(events[2].details["reason"], "STORAGE_NOT_WRITABLE");
    EXPECT_EQ(core.TakeDriveCancel().value_or(Reason()).code, "STORAGE_NOT_WRITABLE");
    EXPECT_TRUE(core.Idle());
    EXPECT_EQ(core.RobotState().sub_state, "");
    // The node ends the drive it was planning: the core is IDLE already and stays so.
    core.OnDriveEnded(false, Reason("STORAGE_NOT_WRITABLE", ""));
    EXPECT_TRUE(core.TakeEvents().empty());
  }
  {
    TestCore core = Idle();
    ASSERT_TRUE(core.Command("docking.start", json::object()).why.empty());
    ASSERT_TRUE(core.Docking());
    Settle(core);
    core.AbandonStart("os-session.json");
    const auto events = core.TakeEvents();
    EXPECT_EQ(TypesOf(events), (std::vector<std::string>{"STORAGE", "DRIVE_REFUSED", "STATE"}));
    EXPECT_EQ(ReasonOf(events, "DRIVE_REFUSED"), "STORAGE_NOT_WRITABLE");
    EXPECT_TRUE(core.Idle());
    EXPECT_FALSE(core.TakeDockingLeg().has_value());
    EXPECT_FALSE(core.TakeCalibrationCancel());
  }
  {
    TestCore core = Recording();
    core.AbandonStart("os-session.json");
    const auto events = core.TakeEvents();
    EXPECT_EQ(TypesOf(events), (std::vector<std::string>{"STORAGE", "RECORDING_REFUSED", "STATE"}));
    EXPECT_EQ(ReasonOf(events, "RECORDING_REFUSED"), "STORAGE_NOT_WRITABLE");
    EXPECT_TRUE(core.Idle());
    EXPECT_FALSE(core.OnTeleop(0.1, 0.0, 100.0).has_value());
  }
  {
    // In IDLE there is nothing to take back.
    TestCore core = Idle();
    core.AbandonStart("os-session.json");
    EXPECT_EQ(TypesOf(core.TakeEvents()), std::vector<std::string>{"STORAGE"});
    EXPECT_TRUE(core.Idle());
  }
}
