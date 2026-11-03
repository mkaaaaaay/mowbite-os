#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "mowbite_logic/logic_core.h"

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
  return status;
}

// On the lawn, everything there for a calibration drive; the program reported at t = 10 s.
LogicCore Ready(LogicConfig config = LogicConfig()) {
  config.dock_contact_x = -0.10;
  LogicCore core{config};
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
LogicCore Calibrating(LogicConfig config = LogicConfig()) {
  LogicCore core = Ready(config);
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
void ExpectStopped(LogicCore& core, const std::string& reason, const std::string& severity) {
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
  LogicCore core = Ready(config);
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

TEST(Calibration, NotEvaluatedIsAWarning) {
  LogicCore core = Calibrating();
  core.OnCalibrationEnded({"DONE", "", "not_evaluated", "Recorded, not evaluated"});
  const auto events = core.TakeEvents();
  const Event* calibrated = Find(events, "CALIBRATED");
  ASSERT_TRUE(calibrated);
  EXPECT_EQ(calibrated->details["status"], "not_evaluated");
  EXPECT_EQ(calibrated->details["severity"], "warning");
}

TEST(Calibration, EveryReasonAgainstAStart) {
  const auto refused = [](LogicCore& core, const std::string& code) {
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
    LogicCore core = Ready();
    ASSERT_EQ(core.OnAction(kStartRecording), ActionResult::kDone);
    const Event e = refused(core, "BUSY");
    EXPECT_EQ(e.details["severity"], "info");
  }
  {
    LogicCore core = Ready();
    HardwareSnapshot hw = Healthy();
    hw.emergency = true;
    hw.emergency_reason = kEmergencyStop;
    core.OnHardwareStatus(hw);
    EXPECT_EQ(refused(core, "EMERGENCY").details["severity"], "error");
  }
  {
    LogicCore core = Ready();
    core.OnCapabilities(false);
    refused(core, "NO_SAFETY_MEASUREMENT");
  }
  {
    LogicCore core = Ready();
    HardwareSnapshot hw = Healthy();
    hw.dock_contact = true;
    core.OnHardwareStatus(hw);
    EXPECT_EQ(refused(core, "ON_DOCK").details["severity"], "info");
  }
  {
    LogicCore core = Ready();
    core.OnPoseTimeout();
    refused(core, "NO_POSITION");
  }
  {
    // Stricter than for mowing: 3 cm.
    LogicCore core = Ready();
    core.OnPose(PoseAt(2, 3, 0.05f));
    const Event e = refused(core, "POSITION_ACCURACY");
    EXPECT_DOUBLE_EQ(e.details["limit"].get<double>(), 0.03);
    EXPECT_EQ(e.details["severity"], "warning");
  }
  {
    LogicCore core = Ready();
    core.OnPose(PoseAt(2, 3, 0.015f, false));
    refused(core, "NO_HEADING");
  }
  {
    LogicCore core = Ready();
    HardwareSnapshot hw = Healthy();
    hw.battery_percentage = 25;
    core.OnHardwareStatus(hw);
    const Event e = refused(core, "BATTERY_LOW");
    EXPECT_EQ(e.details["battery_percentage"], 25);
    EXPECT_EQ(e.details["limit"], 30);
    hw.battery_ok = false;
    core.OnHardwareStatus(hw);
    EXPECT_EQ(refused(core, "BATTERY_LOW").details["message"], "No battery level from the driver");
  }
  {
    LogicCore core = Ready();
    core.OnDrivableAreas({}, {}, MapReason(MapDatum::kMismatch, ""));
    EXPECT_EQ(refused(core, "MAP_MISMATCH").details["severity"], "error");
  }
  {
    // The status of the program counts for 2 s.
    LogicCore core = Ready();
    core.TakeEvents();
    EXPECT_EQ(core.StartCalibration(12.5).code, "CALIBRATION_MISSING");
    EXPECT_EQ(DetailsOf(core.TakeEvents(), "CALIBRATION_REFUSED")["severity"], "error");
    LogicCore none = Ready();
    none.OnCalibrationStatus(Lane(), -100);
    refused(none, "CALIBRATION_MISSING");
  }
  {
    // Room and sensors come from the program, with their numbers.
    LogicCore core = Ready();
    CalibrationStatus status = Lane(0, 0);
    status.problems.push_back(Reason("NO_ROOM", "No room").With("length", 1.4).With("limit", 2.0));
    status.problems.push_back(Reason("SLOPE", "Tilted by 7 deg").With("slope", 7.0).With("limit", 5.0));
    core.OnCalibrationStatus(status, 10.5);
    const Event e = refused(core, "NO_ROOM");
    EXPECT_DOUBLE_EQ(e.details["length"].get<double>(), 1.4);
    EXPECT_EQ(e.details["severity"], "warning");
    EXPECT_EQ(Codes(core.CalibrationProblems(11.0)), (std::vector<std::string>{"NO_ROOM", "SLOPE"}));
    // Without a lane and without a reason from the program.
    core.OnCalibrationStatus(Lane(0, 0), 10.5);
    refused(core, "NO_ROOM");
  }
  {
    LogicCore core = Ready();
    CalibrationStatus status = Lane();
    status.running = true;
    core.OnCalibrationStatus(status, 10.5);
    refused(core, "STILL_STOPPING");
  }
}

TEST(Calibration, CheckListsEveryReasonOnce) {
  LogicCore core = Ready();
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
  LogicCore core = Calibrating();
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

TEST(Calibration, EmergencyEndsItWithOneZero) {
  LogicCore core = Calibrating();
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
    LogicCore core = Calibrating();
    core.OnPoseTimeout();
    ExpectStopped(core, "POSE_LOST", "warning");
  }
  {
    LogicCore core = Calibrating();
    core.OnCapabilities(false);
    ExpectStopped(core, "NO_SAFETY_MEASUREMENT", "error");
  }
  {
    // App and HA: the mower stays where it is.
    LogicCore core = Calibrating();
    EXPECT_TRUE(core.AbortCalibration(Reason("STOPPED", "Stopped by calibration.abort")));
    ExpectStopped(core, "STOPPED", "info");
    EXPECT_FALSE(core.AbortCalibration(Reason("STOPPED", "again")));
    EXPECT_TRUE(core.TakeEvents().empty());
  }
}

TEST(Calibration, LowBatteryHeldLikeWhileMowing) {
  LogicConfig config;
  config.battery_critical_time = 10.0;
  LogicCore core = Calibrating(config);
  HardwareSnapshot hw = Healthy();
  hw.battery_voltage = 20.5f;
  core.OnHardwareStatus(hw);
  for (double t = 10.5; t < 20.0; t += 0.5) {
    core.OnCalibrationFeedback("line", 1, t);
    ASSERT_FALSE(core.TickCalibration(t)) << t;
  }
  // A dip under load does not count.
  core.OnHardwareStatus(Healthy());
  core.OnCalibrationFeedback("line", 1, 20.4);
  EXPECT_FALSE(core.TickCalibration(20.5));
  core.OnHardwareStatus(hw);
  core.OnCalibrationFeedback("line", 1, 21.0);
  EXPECT_FALSE(core.TickCalibration(21.0));
  for (double t = 21.5; t < 30.9; t += 0.5) core.OnCalibrationFeedback("line", 1, t);
  EXPECT_FALSE(core.TickCalibration(30.9));
  EXPECT_TRUE(core.TickCalibration(31.0));
  const auto events = core.TakeEvents();
  const Event* failed = Find(events, "CALIBRATION_FAILED");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->details["reason"], "BATTERY_LOW");
  EXPECT_NEAR(failed->details["battery_voltage"].get<double>(), 20.5, 1e-6);
  EXPECT_DOUBLE_EQ(failed->details["limit"].get<double>(), 21.0);
  EXPECT_TRUE(core.TakeCalibrationCancel());
}

TEST(Calibration, WatchdogOfTheProgram) {
  LogicCore core = Calibrating();
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
    LogicCore core = Calibrating();
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
    LogicCore core = Calibrating();
    core.OnCalibrationEnded({"NO_RTK", "No RTK fixed position", "", ""});
    EXPECT_EQ(DetailsOf(core.TakeEvents(), "CALIBRATION_FAILED")["severity"], "warning");
  }
  {
    // Without a reason the program counts as gone.
    LogicCore core = Calibrating();
    core.OnCalibrationEnded({"", "The calibration program dropped the drive", "", ""});
    EXPECT_EQ(DetailsOf(core.TakeEvents(), "CALIBRATION_FAILED")["reason"], "CALIBRATION_MISSING");
  }
  {
    // A late result after the logic stopped it changes nothing.
    LogicCore core = Calibrating();
    core.AbortCalibration(Reason("STOPPED", "Stopped by calibration.abort"));
    core.TakeEvents();
    core.OnCalibrationEnded({"STOPPED", "canceled", "", ""});
    core.OnCalibrationFeedback("line", 1, 30.0);
    EXPECT_TRUE(core.TakeEvents().empty());
  }
}

TEST(Calibration, PositionJumpsDoNotEndIt) {
  LogicCore core = Calibrating();
  core.OnPositionJump(2.3, 3.1, 0.3, true);
  EXPECT_TRUE(core.Calibrating());
  EXPECT_EQ(Types(core.TakeEvents()), std::vector<std::string>{"POSITION_JUMP"});
  EXPECT_FALSE(core.TakeStopRequest());
  EXPECT_FALSE(core.TakeCalibrationCancel());
}
