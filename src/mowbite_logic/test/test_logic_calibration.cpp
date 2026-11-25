#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

using namespace mowbite_logic;

namespace {

const std::string kStartMowing = "mower_logic:idle/start_mowing";
const std::string kStartRecording = "mower_logic:idle/start_area_recording";
const Ring kLawn{{0, 0}, {10, 0}, {10, 6}, {0, 6}};

HardwareSnapshot Healthy() {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  return hw;
}

PoseSnapshot PoseAt(double x, double y, float accuracy = 0.015f, bool heading = true) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.heading = 0.3;
  pose.position_accuracy = accuracy;
  pose.orientation_accuracy = heading ? 0.01f : 999.0f;
  pose.orientation_valid = heading;
  return pose;
}

CalibrationStatus Lane(double heading = 0.5, double length = 3.0) {
  CalibrationStatus status;
  status.heading = heading;
  status.lane_length = length;
  status.strip_length = length + 1.88;
  status.strip_width = 1.88;
  status.rounds = 4;
  status.expected_s = 245.4;
  return status;
}

// On the lawn, everything there for a calibration drive; the program reported at t = 10 s.
TestCore Ready(LogicConfig config = LogicConfig()) {
  config.dock_contact_x = -0.10;
  TestCore core{config};
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnHardwareStatus(Healthy());
  core.OnCapabilities(true);
  core.OnPose(PoseAt(2, 3));
  core.OnMowMap({{"lawn", "Lawn", true}}, Reason());
  core.OnDrivableAreas({kLawn}, {});
  core.OnCalibrationStatus(Lane(), 10.0);
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

const Event* Find(const std::vector<Event>& events, const std::string& type) {
  for (const auto& e : events) {
    if (e.type == type) return &e;
  }
  return nullptr;
}

// The details of the first event of this type, none if there is none.
json DetailsOf(const std::vector<Event>& events, const std::string& type) {
  const Event* event = Find(events, type);
  if (!event) {
    ADD_FAILURE() << "no event " << type;
    return json::object();
  }
  return event->details;
}

std::vector<std::string> Codes(const std::vector<Reason>& reasons) {
  std::vector<std::string> codes;
  for (const auto& why : reasons) codes.push_back(why.code);
  return codes;
}

// Started at t = 10 s with the goal taken by the node.
TestCore Calibrating(LogicConfig config = LogicConfig()) {
  TestCore core = Ready(config);
  EXPECT_TRUE(core.StartCalibration(10.0).empty());
  EXPECT_TRUE(core.TakeCalibrationGoal().has_value());
  core.OnCalibrationSign(10.1);
  core.TakeEvents();
  core.TakeStopRequest();
  core.TakeNotices();
  return core;
}

// The end of a drive the logic stopped: CALIBRATION_FAILED with reason, the end of the phases, IDLE, one zero and the
// program canceled.
void ExpectStopped(TestCore& core, const std::string& reason, const std::string& severity) {
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"CALIBRATION_FAILED", "CALIBRATION", "STATE"}));
  EXPECT_EQ(events[0].details["reason"], reason);
  EXPECT_EQ(events[0].details["severity"], severity);
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_EQ(events[1].details["phase"], "end");
  EXPECT_EQ(events[2].details["state"], "IDLE");
  EXPECT_FALSE(events[2].details.contains("sub_state"));
  EXPECT_FALSE(core.Calibrating());
  EXPECT_FALSE(core.CalibrationActive());
  EXPECT_TRUE(core.TakeStopRequest());
  EXPECT_TRUE(core.TakeCalibrationCancel());
  EXPECT_FALSE(core.TakeCalibrationCancel());
  // It stays where it stopped: no way home.
  EXPECT_FALSE(core.Docking());
}

}  // namespace

TEST(Calibration, StartsOnTheLawnAndEndsCalibrated) {
  LogicConfig config;
  config.mow_with_blade = true;
  TestCore core = Ready(config);
  EXPECT_TRUE(core.CalibrationProblems(10.5).empty());
  ASSERT_TRUE(core.StartCalibration(10.5).empty());
  EXPECT_TRUE(core.Calibrating());
  EXPECT_TRUE(core.CalibrationActive());
  EXPECT_FALSE(core.Idle());
  EXPECT_FALSE(core.FollowerActive());
  const RobotStateData state = core.RobotState();
  EXPECT_EQ(state.state, "IDLE");
  EXPECT_EQ(state.sub_state, "CALIBRATING");
  auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE", "CALIBRATION"}));
  EXPECT_EQ(events[0].details["state"], "IDLE");
  EXPECT_EQ(events[0].details["sub_state"], "CALIBRATING");
  EXPECT_EQ(events[1].details["phase"], "start");
  EXPECT_EQ(events[1].details["index"], 0);
  EXPECT_EQ(events[1].details["rounds"], 4);
  EXPECT_EQ(events[1].details["expected_s"], 245);
  // The lane the start was checked with, once.
  const auto goal = core.TakeCalibrationGoal();
  ASSERT_TRUE(goal);
  EXPECT_DOUBLE_EQ(goal->heading, 0.5);
  EXPECT_DOUBLE_EQ(goal->lane_length, 3.0);
  EXPECT_FALSE(core.TakeCalibrationGoal());
  // A phase is told once, when it changes.
  core.OnCalibrationFeedback("stand", 0, 11.0);
  core.OnCalibrationFeedback("stand", 0, 11.4);
  core.OnCalibrationFeedback("line", 1, 21.0);
  events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"CALIBRATION", "CALIBRATION"}));
  EXPECT_EQ(events[0].details["phase"], "stand");
  EXPECT_EQ(events[1].details["phase"], "line");
  EXPECT_EQ(events[1].details["index"], 1);
  // Never the blade, also with mow_with_blade.
  EXPECT_FALSE(core.BladeWanted());
  EXPECT_FALSE(core.TickCalibration(22.0));
  core.OnCalibrationEnded({"DONE", "The calibration drive ran to its end", "done", "4 of 4 values ok"});
  events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"CALIBRATED", "CALIBRATION", "STATE"}));
  EXPECT_EQ(events[0].details["status"], "done");
  EXPECT_EQ(events[0].details["summary"], "4 of 4 values ok");
  EXPECT_EQ(events[0].details["severity"], "info");
  EXPECT_EQ(events[1].details["phase"], "end");
  EXPECT_EQ(events[1].details["index"], 1);
  EXPECT_TRUE(core.Idle());
  EXPECT_TRUE(core.TakeStopRequest());
  // The program ended by itself, nothing to cancel.
  EXPECT_FALSE(core.TakeCalibrationCancel());
  EXPECT_FALSE(core.Docking());
}

// calibration.start: the answer of a command, the client in its events, a refusal with the data of a refused command.
TEST(Calibration, StartAnswersLikeACommand) {
  {
    TestCore core = Ready();
    const CommandOutcome out = core.CalibrationStart(core.CalibrationProblems(10.5), "app", 10.5);
    ASSERT_TRUE(out.accepted);
    EXPECT_EQ(out.result, (json{{"state", "CALIBRATING"}}));
    const auto events = core.TakeEvents();
    ASSERT_EQ(Types(events), (std::vector<std::string>{"STATE", "CALIBRATION"}));
    EXPECT_EQ(events[1].details["client"], "app");
    // Later events are the program's, without the client.
    core.OnCalibrationFeedback("stand", 0, 11.0);
    EXPECT_FALSE(DetailsOf(core.TakeEvents(), "CALIBRATION").contains("client"));
  }
  {
    TestCore core = Ready();
    HardwareSnapshot hw = Healthy();
    hw.emergency = true;
    hw.emergency_reason = kEmergencyStop;
    core.OnHardwareStatus(hw);
    core.TakeEvents();
    const CommandOutcome out = core.CalibrationStart(core.CalibrationProblems(11.0), "ha", 11.0);
    EXPECT_FALSE(out.accepted);
    EXPECT_EQ(out.why.code, "EMERGENCY");
    const auto events = core.TakeEvents();
    ASSERT_EQ(Types(events), std::vector<std::string>{"CALIBRATION_REFUSED"});
    EXPECT_EQ(events[0].details["client"], "ha");
    EXPECT_EQ(out.data["event"], "CALIBRATION_REFUSED");
    EXPECT_EQ(out.data["reason"], "EMERGENCY");
    EXPECT_EQ(out.data["severity"], "error");
    EXPECT_EQ(out.data["state"], "IDLE");
    EXPECT_EQ(out.data["sub_state"], "");
    EXPECT_EQ(out.data["event_id"], events[0].id);
    EXPECT_FALSE(core.Calibrating());
  }
  {
    // A reason only the node knows, without a client.
    TestCore core = Ready();
    const CommandOutcome out =
        core.CalibrationStart({Reason("CALIBRATION_MISSING", "The calibration program is not running")}, "", 10.5);
    EXPECT_EQ(out.data["reason"], "CALIBRATION_MISSING");
    const auto events = core.TakeEvents();
    ASSERT_EQ(Types(events), std::vector<std::string>{"CALIBRATION_REFUSED"});
    EXPECT_EQ(out.data["event_id"], events[0].id);
    EXPECT_FALSE(events[0].details.contains("client"));
    EXPECT_FALSE(core.Calibrating());
    EXPECT_FALSE(core.TakeCalibrationGoal());
  }
}

TEST(Calibration, NotEvaluatedIsAWarning) {
  TestCore core = Calibrating();
  core.OnCalibrationEnded({"DONE", "", "not_evaluated", "Recorded, not evaluated"});
  const auto events = core.TakeEvents();
  const Event* calibrated = Find(events, "CALIBRATED");
  ASSERT_TRUE(calibrated);
  EXPECT_EQ(calibrated->details["status"], "not_evaluated");
  EXPECT_EQ(calibrated->details["severity"], "warning");
}

TEST(Calibration, EveryReasonAgainstAStart) {
  const auto refused = [](TestCore& core, const std::string& code) {
    core.TakeEvents();
    const Reason why = core.StartCalibration(11.0);
    EXPECT_EQ(why.code, code);
    EXPECT_FALSE(core.Calibrating());
    EXPECT_FALSE(core.TakeCalibrationGoal());
    const auto events = core.TakeEvents();
    EXPECT_EQ(Types(events), std::vector<std::string>{"CALIBRATION_REFUSED"});
    if (!events.empty()) {
      EXPECT_EQ(events[0].details["reason"], code);
    }
    return events.empty() ? Event{} : events[0];
  };
  {
    TestCore core = Ready();
    ASSERT_EQ(core.OnAction(kStartRecording), ActionResult::kDone);
    const Event e = refused(core, "BUSY");
    EXPECT_EQ(e.details["severity"], "info");
  }
  {
    TestCore core = Ready();
    HardwareSnapshot hw = Healthy();
    hw.emergency = true;
    hw.emergency_reason = kEmergencyStop;
    core.OnHardwareStatus(hw);
    EXPECT_EQ(refused(core, "EMERGENCY").details["severity"], "error");
  }
  {
    TestCore core = Ready();
    core.OnCapabilities(false);
    refused(core, "NO_SAFETY_MEASUREMENT");
  }
  {
    TestCore core = Ready();
    HardwareSnapshot hw = Healthy();
    hw.dock_contact = true;
    core.OnHardwareStatus(hw);
    EXPECT_EQ(refused(core, "ON_DOCK").details["severity"], "info");
  }
  {
    TestCore core = Ready();
    core.OnPoseTimeout();
    refused(core, "NO_POSITION");
  }
  {
    // Stricter than for mowing: 3 cm.
    TestCore core = Ready();
    core.OnPose(PoseAt(2, 3, 0.05f));
    const Event e = refused(core, "POSITION_ACCURACY");
    EXPECT_DOUBLE_EQ(e.details["limit"].get<double>(), 0.03);
    EXPECT_EQ(e.details["severity"], "warning");
  }
  {
    TestCore core = Ready();
    core.OnPose(PoseAt(2, 3, 0.015f, false));
    refused(core, "NO_HEADING");
  }
  {
    TestCore core = Ready();
    HardwareSnapshot hw = Healthy();
    hw.battery_percentage = 25;
    core.OnHardwareStatus(hw);
    const Event e = refused(core, "BATTERY_LOW");
    EXPECT_EQ(e.details["battery_percentage"], 25);
    EXPECT_DOUBLE_EQ(e.details["battery_voltage"].get<double>(), 24.5);
    // In % here, as x-reasons of CALIBRATION_REFUSED says.
    EXPECT_EQ(e.details["limit"], 30);
    hw.battery_ok = false;
    core.OnHardwareStatus(hw);
    EXPECT_EQ(refused(core, "BATTERY_LOW").details["message"], "No battery level from the driver");
  }
  {
    TestCore core = Ready();
    core.OnDrivableAreas({}, {}, MapReason(MapDatum::kMismatch, ""));
    EXPECT_EQ(refused(core, "MAP_MISMATCH").details["severity"], "error");
  }
  {
    // The status of the program counts for 2 s.
    TestCore core = Ready();
    core.TakeEvents();
    EXPECT_EQ(core.StartCalibration(12.5).code, "CALIBRATION_MISSING");
    EXPECT_EQ(DetailsOf(core.TakeEvents(), "CALIBRATION_REFUSED")["severity"], "error");
    TestCore none = Ready();
    none.OnCalibrationStatus(Lane(), -100);
    refused(none, "CALIBRATION_MISSING");
  }
  {
    // Room and sensors come from the program, with their numbers.
    TestCore core = Ready();
    CalibrationStatus status = Lane(0, 0);
    status.problems.push_back(Reason("NO_ROOM", "No room").With("free_length", 1.4).With("length", 2.0));
    status.problems.push_back(Reason("SLOPE", "Tilted by 7 deg").With("slope", 7.0).With("limit", 5.0));
    core.OnCalibrationStatus(status, 10.5);
    const Event e = refused(core, "NO_ROOM");
    EXPECT_DOUBLE_EQ(e.details["free_length"].get<double>(), 1.4);
    EXPECT_EQ(e.details["severity"], "warning");
    EXPECT_EQ(Codes(core.CalibrationProblems(11.0)), (std::vector<std::string>{"NO_ROOM", "SLOPE"}));
    // Without a lane and without a reason from the program.
    core.OnCalibrationStatus(Lane(0, 0), 10.5);
    refused(core, "NO_ROOM");
  }
  {
    TestCore core = Ready();
    CalibrationStatus status = Lane();
    status.running = true;
    core.OnCalibrationStatus(status, 10.5);
    refused(core, "STILL_STOPPING");
  }
}

TEST(Calibration, CheckListsEveryReasonOnce) {
  TestCore core = Ready();
  core.OnCapabilities(false);
  core.OnPose(PoseAt(2, 3, 0.2f, false));
  HardwareSnapshot hw = Healthy();
  hw.battery_percentage = 10;
  core.OnHardwareStatus(hw);
  CalibrationStatus status = Lane();
  status.problems.push_back(Reason("NO_RTK", "No RTK"));
  status.problems.push_back(Reason("NO_HEADING", "the program says so too"));
  core.OnCalibrationStatus(status, 10.5);
  EXPECT_EQ(Codes(core.CalibrationProblems(11.0)),
            (std::vector<std::string>{"NO_SAFETY_MEASUREMENT", "POSITION_ACCURACY", "NO_HEADING", "BATTERY_LOW",
                                      "NO_RTK"}));
  // A check changes nothing.
  EXPECT_TRUE(core.TakeEvents().empty());
  EXPECT_FALSE(core.Calibrating());
}

TEST(Calibration, BusyForEverythingElse) {
  TestCore core = Calibrating();
  EXPECT_EQ(core.StartProblem().code, "BUSY");
  EXPECT_EQ(core.StartProblem().fields["state"], "calibrating");
  EXPECT_EQ(core.DriveProblem().code, "BUSY");
  EXPECT_EQ(core.DockingProblem().code, "BUSY");
  EXPECT_EQ(core.OnAction(kStartMowing), ActionResult::kRejected);
  EXPECT_EQ(core.OnAction(kStartRecording), ActionResult::kRejected);
  EXPECT_EQ(core.StartCalibration(11.0).code, "BUSY");
  EXPECT_FALSE(core.TakeCalibrationGoal());
  EXPECT_TRUE(core.Calibrating());
  EXPECT_FALSE(core.OnTeleop(0.3, 0.0, 11.0));
}

// The hold of an update refuses a start like the other starts, and none is granted while calibrating.
TEST(Calibration, TheHoldOfAnUpdateRefusesAStart) {
  TestCore core = Ready();
  const int64_t until = 1'790'000'600;
  core.SetHold(until);
  EXPECT_EQ(Codes(core.CalibrationProblems(10.0)).front(), "HOLD");
  EXPECT_EQ(core.StartCalibration(10.0).code, "HOLD");
  EXPECT_FALSE(core.TakeCalibrationGoal());
  EXPECT_FALSE(core.Calibrating());
  const json details = DetailsOf(core.TakeEvents(), "CALIBRATION_REFUSED");
  EXPECT_EQ(details["reason"], "HOLD");
  EXPECT_EQ(details["until"], until);
  EXPECT_EQ(details["severity"], "info");
  EXPECT_EQ(details["resume"], "user");
  // The hold the driver reports refuses it as well.
  core.SetHold(0);
  HardwareSnapshot hw = Healthy();
  hw.hold = true;
  core.OnHardwareStatus(hw);
  EXPECT_EQ(core.StartCalibration(10.0).code, "HOLD");
  hw.hold = false;
  core.OnHardwareStatus(hw);
  EXPECT_TRUE(core.StartCalibration(10.0).empty());
  EXPECT_TRUE(core.Calibrating());
  EXPECT_FALSE(core.HoldAllowed());
}

TEST(Calibration, EmergencyEndsItWithOneZero) {
  TestCore core = Calibrating();
  HardwareSnapshot hw = Healthy();
  hw.emergency = true;
  hw.emergency_reason = kEmergencyLift;
  core.OnHardwareStatus(hw);
  const auto events = core.TakeEvents();
  ASSERT_EQ(Types(events), (std::vector<std::string>{"CALIBRATION_FAILED", "CALIBRATION", "STATE", "EMERGENCY"}));
  EXPECT_EQ(events[0].details["reason"], "EMERGENCY");
  EXPECT_EQ(events[0].details["severity"], "error");
  EXPECT_FALSE(core.Calibrating());
  EXPECT_TRUE(core.TakeStopRequest());
  EXPECT_TRUE(core.TakeCalibrationCancel());
  // After the release nothing goes on by itself (G11).
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(core.Calibrating());
  EXPECT_FALSE(core.TakeCalibrationGoal());
}

TEST(Calibration, PoseLostMeasurementLostAndAbort) {
  {
    TestCore core = Calibrating();
    core.OnPoseTimeout();
    ExpectStopped(core, "POSE_LOST", "warning");
  }
  {
    TestCore core = Calibrating();
    core.OnCapabilities(false);
    ExpectStopped(core, "NO_SAFETY_MEASUREMENT", "error");
  }
  {
    // App and HA: the mower stays where it is.
    TestCore core = Calibrating();
    EXPECT_TRUE(core.AbortCalibration(Reason("STOPPED", "Stopped by calibration.abort")));
    ExpectStopped(core, "STOPPED", "info");
    EXPECT_FALSE(core.AbortCalibration(Reason("STOPPED", "again")));
    EXPECT_TRUE(core.TakeEvents().empty());
  }
}

TEST(Calibration, LowBatteryHeldLikeWhileMowing) {
  // Below empty (21.0 V) the highest voltage of 20 s counts; a voltage above it within the window keeps it going.
  TestCore core = Calibrating();
  HardwareSnapshot hw = Healthy();
  hw.battery_voltage = 20.5f;
  core.OnHardwareStatus(hw);
  for (double t = 10.5; t < 20.0; t += 0.5) {
    core.OnCalibrationFeedback("line", 1, t);
    ASSERT_FALSE(core.TickCalibration(t)) << t;
  }
  core.OnHardwareStatus(Healthy());
  core.OnCalibrationFeedback("line", 1, 20.0);
  EXPECT_FALSE(core.TickCalibration(20.0));
  core.OnHardwareStatus(hw);
  for (double t = 20.5; t < 40.0; t += 0.5) {
    core.OnCalibrationFeedback("line", 1, t);
    ASSERT_FALSE(core.TickCalibration(t)) << t;
  }
  core.OnCalibrationFeedback("line", 1, 40.0);
  EXPECT_TRUE(core.TickCalibration(40.5));
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "CALIBRATION_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "BATTERY_LOW");
  EXPECT_NEAR(failed->details["battery_voltage"].get<double>(), 20.5, 1e-6);
  EXPECT_DOUBLE_EQ(failed->details["limit"].get<double>(), 21.0);
  EXPECT_TRUE(core.TakeCalibrationCancel());
}

TEST(Calibration, CriticalBatteryAfterItsHoldTime) {
  TestCore core = Calibrating();
  HardwareSnapshot hw = Healthy();
  hw.battery_voltage = 19.5f;
  const auto tick = [&core](double t) {
    core.OnCalibrationFeedback("line", 1, t);
    return core.TickCalibration(t);
  };
  core.OnHardwareStatus(hw);
  EXPECT_FALSE(tick(10.5));
  // A dip under load does not count.
  core.OnHardwareStatus(Healthy());
  EXPECT_FALSE(tick(11.5));
  core.OnHardwareStatus(hw);
  EXPECT_FALSE(tick(12.0));
  EXPECT_FALSE(tick(13.5));
  EXPECT_TRUE(tick(14.0));
  const auto taken = core.TakeEvents();
  const Event* failed = Find(taken, "CALIBRATION_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "BATTERY_LOW");
  EXPECT_DOUBLE_EQ(failed->details["limit"].get<double>(), 19.7);
}

TEST(Calibration, WatchdogOfTheProgram) {
  TestCore core = Calibrating();
  core.OnCalibrationFeedback("align", 0, 11.0);
  EXPECT_FALSE(core.TickCalibration(12.9));
  core.OnCalibrationFeedback("align", 0, 12.95);
  EXPECT_FALSE(core.TickCalibration(14.9));
  core.TakeEvents();
  EXPECT_TRUE(core.TickCalibration(15.0));
  const auto events = core.TakeEvents();
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events[0].details["reason"], "CALIBRATION_MISSING");
  EXPECT_NEAR(events[0].details["seconds"].get<double>(), 2.05, 1e-9);
  EXPECT_DOUBLE_EQ(events[0].details["limit"].get<double>(), 2.0);
  EXPECT_TRUE(core.TakeCalibrationCancel());
  EXPECT_TRUE(core.TakeStopRequest());
  EXPECT_FALSE(core.TickCalibration(20.0));
}

TEST(Calibration, TheProgramStopsWithItsReason) {
  {
    TestCore core = Calibrating();
    core.OnCalibrationFeedback("line", 2, 50.0);
    core.TakeEvents();
    core.OnCalibrationEnded({"LEFT_LANE", "The antenna is 0.31 m off its line, at most 0.30 m", "", ""});
    const auto events = core.TakeEvents();
    ASSERT_EQ(Types(events), (std::vector<std::string>{"CALIBRATION_FAILED", "CALIBRATION", "STATE"}));
    EXPECT_EQ(events[0].details["reason"], "LEFT_LANE");
    EXPECT_EQ(events[0].details["message"], "The antenna is 0.31 m off its line, at most 0.30 m");
    EXPECT_EQ(events[0].details["severity"], "error");
    EXPECT_EQ(events[1].details["index"], 2);
    // The program ended it, nothing to cancel.
    EXPECT_FALSE(core.TakeCalibrationCancel());
    EXPECT_TRUE(core.TakeStopRequest());
  }
  {
    TestCore core = Calibrating();
    core.OnCalibrationEnded({"NO_RTK", "No RTK fixed position", "", ""});
    EXPECT_EQ(DetailsOf(core.TakeEvents(), "CALIBRATION_FAILED")["severity"], "warning");
  }
  {
    // Without a reason the program counts as gone.
    TestCore core = Calibrating();
    core.OnCalibrationEnded({"", "The calibration program dropped the drive", "", ""});
    EXPECT_EQ(DetailsOf(core.TakeEvents(), "CALIBRATION_FAILED")["reason"], "CALIBRATION_MISSING");
  }
  {
    // A late result after the logic stopped it changes nothing.
    TestCore core = Calibrating();
    core.AbortCalibration(Reason("STOPPED", "Stopped by calibration.abort"));
    core.TakeEvents();
    core.OnCalibrationEnded({"STOPPED", "canceled", "", ""});
    core.OnCalibrationFeedback("line", 1, 30.0);
    EXPECT_TRUE(core.TakeEvents().empty());
  }
}

TEST(Calibration, PositionJumpsDoNotEndIt) {
  TestCore core = Calibrating();
  core.OnPositionJump(2.3, 3.1, 0.3, true);
  EXPECT_TRUE(core.Calibrating());
  EXPECT_EQ(Types(core.TakeEvents()), std::vector<std::string>{"POSITION_JUMP"});
  EXPECT_FALSE(core.TakeStopRequest());
  EXPECT_FALSE(core.TakeCalibrationCancel());
}

// G35: no calibration drive while os-session.json cannot be written, and one whose record failed ends at once.
TEST(LogicCalibration, StorageThatCannotBeWrittenRefusesAndTakesBackTheStart) {
  {
    TestCore core = Ready();
    core.SetUnwritableFile("os-session.json");
    core.TakeEvents();
    EXPECT_EQ(core.StartCalibration(10.0).code, "STORAGE_NOT_WRITABLE");
    EXPECT_EQ(DetailsOf(core.TakeEvents(), "CALIBRATION_REFUSED")["reason"], "STORAGE_NOT_WRITABLE");
    EXPECT_FALSE(core.TakeCalibrationGoal().has_value());
  }
  TestCore core = Ready();
  ASSERT_TRUE(core.StartCalibration(10.0).empty());
  // The node took the goal already, as in its step.
  ASSERT_TRUE(core.TakeCalibrationGoal().has_value());
  core.TakeEvents();
  core.AbandonStart("os-session.json");
  const auto events = core.TakeEvents();
  EXPECT_EQ(Types(events), (std::vector<std::string>{"STORAGE", "CALIBRATION_REFUSED", "CALIBRATION", "STATE"}));
  EXPECT_EQ(DetailsOf(events, "CALIBRATION_REFUSED")["reason"], "STORAGE_NOT_WRITABLE");
  EXPECT_FALSE(core.Calibrating());
  EXPECT_FALSE(core.TakeCalibrationGoal().has_value());
  EXPECT_TRUE(core.TakeCalibrationCancel());
}

// A calibration whose record failed tells nothing of a setup step.
TEST(LogicCalibration, AFailedRecordTellsOnlyTheCalibration) {
  TestCore core = Ready();
  ASSERT_TRUE(core.StartCalibration(10.0).empty());
  core.TakeNotices();
  core.AbandonStart("os-session.json");
  for (const auto& line : core.TakeNotices()) EXPECT_EQ(line.rfind("Setup step", 0), std::string::npos) << line;
}
