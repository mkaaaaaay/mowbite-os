#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/logic_core.h"

using namespace mowbite_logic;

namespace {

const std::string kPrefix = "mower_logic:area_recording/";

AreaRecorder Writable(const RecorderConfig& config = RecorderConfig{}) {
  AreaRecorder recorder(config);
  recorder.SetMapWritable(true, Reason());
  recorder.TakeActionsChanged();
  return recorder;
}

bool On(const AreaRecorder& recorder, const std::string& name) {
  const auto& names = AreaRecorder::ActionNames();
  for (size_t i = 0; i < names.size(); i++) {
    if (names[i] == name) return recorder.Enabled()[i];
  }
  ADD_FAILURE() << "missing action " << name;
  return false;
}

// Drives the mower along the corners in steps of 0.05 m with a pose at every step.
void Drive(AreaRecorder& recorder, const Ring& corners, float accuracy = 0.02f) {
  for (size_t i = 0; i + 1 < corners.size(); i++) {
    const Point2& a = corners[i];
    const Point2& b = corners[i + 1];
    const int steps = static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.05));
    for (int s = 0; s < steps; s++) {
      const double t = static_cast<double>(s) / steps;
      recorder.OnPose(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, accuracy, true);
    }
  }
  recorder.OnPose(corners.back().x, corners.back().y, accuracy, true);
}

// Around a square of side metres from (x0, y0), ending close to the start.
Ring Square(double side, double x0 = 0, double y0 = 0) {
  return {{x0, y0}, {x0 + side, y0}, {x0 + side, y0 + side}, {x0, y0 + side}, {x0, y0 + 0.05}};
}

}  // namespace

TEST(Ring, ThreePointsMakeARing) {
  EXPECT_TRUE(RingProblem({{0, 0}, {1, 0}, {0, 1}}, 0.05).empty());
  EXPECT_DOUBLE_EQ(RingArea({{0, 0}, {1, 0}, {0, 1}}), 0.5);
}

TEST(Ring, TooFewPoints) {
  const Reason problem = RingProblem({{0, 0}, {1, 0}}, 0.05);
  EXPECT_EQ(problem.code, "RING_FEW_POINTS");
  EXPECT_EQ(problem.fields["points"], 2);
  EXPECT_EQ(problem.fields["limit"], 3);
}

TEST(Ring, PointsOnALineEncloseNothing) {
  const Reason problem = RingProblem({{0, 0}, {1, 0}, {2, 0}, {3, 0.01}}, 0.05);
  EXPECT_EQ(problem.code, "RING_TOO_SMALL");
  EXPECT_LT(problem.fields["area"].get<double>(), 0.05);
  EXPECT_DOUBLE_EQ(problem.fields["limit"].get<double>(), 0.05);
  EXPECT_NE(problem.message.find("encloses only"), std::string::npos) << problem.message;
}

TEST(Ring, AFigureEightCrossesItself) {
  EXPECT_TRUE(SelfIntersects({{0, 0}, {2, 2}, {2, 0}, {0, 2}}));
  EXPECT_EQ(RingProblem({{0, 0}, {2, 2}, {2, 0}, {0, 2}}, 0.05).code, "RING_CROSSES_ITSELF");
}

TEST(Ring, TheClosingSegmentCountsToo) {
  // A spiral that is fine as a line, but closing it runs through the segment from (0,1) to (2,1).
  const Ring spiral = {{0, 0}, {3, 0}, {3, 3}, {0, 3}, {0, 1}, {2, 1}, {2, 2}, {1, 2}};
  EXPECT_TRUE(SelfIntersects(spiral));
}

TEST(Ring, TouchingAnEarlierSegmentCounts) {
  EXPECT_TRUE(SelfIntersects({{0, 0}, {4, 0}, {4, 4}, {2, 0}, {0, 4}}));
}

TEST(Ring, AConcaveRingIsFine) {
  EXPECT_FALSE(SelfIntersects({{0, 0}, {4, 0}, {4, 4}, {2, 1}, {0, 4}}));
}

TEST(Recorder, StartsWithStartRecordingOnly) {
  AreaRecorder recorder = Writable();
  EXPECT_TRUE(On(recorder, "start_recording"));
  EXPECT_TRUE(On(recorder, "exit_recording_mode"));
  EXPECT_FALSE(On(recorder, "stop_recording"));
  EXPECT_FALSE(On(recorder, "finish_mowing_area"));
  EXPECT_FALSE(On(recorder, "start_manual_mowing"));
  EXPECT_FALSE(On(recorder, "stop_manual_mowing"));
  EXPECT_EQ(recorder.SubState(), "");
}

TEST(Recorder, PointsComeEveryTenthOfAMetre) {
  AreaRecorder recorder = Writable();
  recorder.OnPose(0, 0, 0.02f, true);
  EXPECT_TRUE(recorder.current().empty()) << "no points before start_recording";
  ASSERT_EQ(recorder.OnAction("start_recording"), AreaRecorder::Result::kDone);
  EXPECT_EQ(recorder.SubState(), "RECORD_OUTLINE");
  // Every 0.05 m: the first point at once, then whenever more than 0.1 m from the last.
  Drive(recorder, {{0, 0}, {1, 0}});
  ASSERT_GE(recorder.current().size(), 7u);
  ASSERT_LE(recorder.current().size(), 10u);
  for (size_t i = 1; i < recorder.current().size(); i++) {
    const double d = std::hypot(recorder.current()[i].x - recorder.current()[i - 1].x,
                                recorder.current()[i].y - recorder.current()[i - 1].y);
    EXPECT_GT(d, 0.1);
  }
}

TEST(Recorder, NoPointsFromAnInaccuratePositionAndOneEventForIt) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  recorder.OnPose(0, 0, 0.02f, true);
  Drive(recorder, {{0, 0}, {1, 0}}, 0.3f);
  EXPECT_EQ(recorder.current().size(), 1u);
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "RECORDING_REFUSED");
  EXPECT_EQ(events[0].details["action"], "point");
  EXPECT_EQ(events[0].details["reason"], "POSITION_ACCURACY");
  EXPECT_FLOAT_EQ(events[0].details["accuracy"].get<float>(), 0.3f);
  EXPECT_DOUBLE_EQ(events[0].details["limit"].get<double>(), 0.1);
  EXPECT_NE(events[0].details["message"].get<std::string>().find("0.30 m"), std::string::npos);
  EXPECT_EQ(events[0].details["resume"], "auto");
  // Accurate again: points come and a log line says so.
  recorder.TakeNotices();
  Drive(recorder, {{1, 0}, {2, 0}});
  EXPECT_GT(recorder.current().size(), 5u);
  EXPECT_EQ(recorder.TakeNotices(), std::vector<std::string>{"Position accurate again, points are recorded"});
  EXPECT_TRUE(recorder.TakeEvents().empty());
}

TEST(Recorder, NanAccuracyCountsAsInaccurate) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  recorder.OnPose(0, 0, std::nanf(""), true);
  EXPECT_TRUE(recorder.current().empty());
}

TEST(Recorder, ManualPointsOnlyOnRequestAndFarEnough) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  ASSERT_EQ(recorder.OnAction("auto_point_collecting_disable"), AreaRecorder::Result::kDone);
  EXPECT_TRUE(On(recorder, "collect_point"));
  EXPECT_TRUE(On(recorder, "auto_point_collecting_enable"));
  EXPECT_FALSE(On(recorder, "auto_point_collecting_disable"));
  // Without a position there is nothing to collect.
  EXPECT_EQ(recorder.OnAction("collect_point"), AreaRecorder::Result::kRefused);
  recorder.TakeEvents();
  Drive(recorder, {{0, 0}, {1, 0}});
  EXPECT_TRUE(recorder.current().empty());
  ASSERT_EQ(recorder.OnAction("collect_point"), AreaRecorder::Result::kDone);
  recorder.OnPose(1, 0, 0.02f, true);
  ASSERT_EQ(recorder.current().size(), 1u);
  // A second one at the same spot is refused with the distance.
  recorder.OnAction("collect_point");
  recorder.OnPose(1.05, 0, 0.02f, true);
  EXPECT_EQ(recorder.current().size(), 1u);
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["action"], "collect_point");
  EXPECT_EQ(events[0].details["reason"], "POINT_TOO_CLOSE");
  EXPECT_NEAR(events[0].details["distance"].get<double>(), 0.05, 1e-9);
  EXPECT_DOUBLE_EQ(events[0].details["limit"].get<double>(), 0.1);
  recorder.OnAction("collect_point");
  recorder.OnPose(1.5, 0, 0.02f, true);
  EXPECT_EQ(recorder.current().size(), 2u);
}

TEST(Recorder, ManualPointWithBadAccuracyOrLostPoseIsRefused) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  recorder.OnAction("auto_point_collecting_disable");
  recorder.OnPose(0, 0, 0.02f, true);
  recorder.OnAction("collect_point");
  recorder.OnPose(0, 0, 0.5f, true);
  recorder.OnAction("collect_point");
  recorder.OnPoseLost();
  EXPECT_TRUE(recorder.current().empty());
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].details["reason"], "POSITION_ACCURACY");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_EQ(events[1].details["reason"], "NO_POSITION");
}

TEST(Recorder, OutlineThenObstacleThenSave) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, Square(5));
  ASSERT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kDone);
  ASSERT_TRUE(recorder.outline());
  EXPECT_FALSE(recorder.recording());
  EXPECT_TRUE(On(recorder, "start_recording"));
  EXPECT_TRUE(On(recorder, "finish_mowing_area"));
  EXPECT_TRUE(On(recorder, "finish_navigation_area"));
  EXPECT_TRUE(On(recorder, "finish_discard"));
  EXPECT_FALSE(On(recorder, "stop_recording"));

  recorder.OnAction("start_recording");
  EXPECT_EQ(recorder.SubState(), "RECORD_OBSTACLE");
  Drive(recorder, Square(1, 2, 2));
  // Saving while recording closes the obstacle first.
  ASSERT_EQ(recorder.OnAction("finish_mowing_area"), AreaRecorder::Result::kDone);
  EXPECT_TRUE(recorder.EditPending());
  // Only exit while it is saved.
  for (const auto& name : AreaRecorder::ActionNames()) EXPECT_EQ(On(recorder, name), name == "exit_recording_mode");

  auto edit = recorder.TakeMapEdit();
  ASSERT_TRUE(edit);
  EXPECT_FALSE(recorder.TakeMapEdit());
  EXPECT_EQ(edit->type, "mow");
  EXPECT_GT(edit->outline.size(), 100u);
  ASSERT_EQ(edit->obstacles.size(), 1u);
  EXPECT_GT(edit->obstacles[0].size(), 20u);

  recorder.TakeEvents();
  recorder.OnMapEditResult(true, "lawn1", "", "");
  EXPECT_FALSE(recorder.EditPending());
  EXPECT_FALSE(recorder.outline());
  EXPECT_TRUE(recorder.obstacles().empty());
  EXPECT_TRUE(On(recorder, "start_recording"));
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "AREA_RECORDED");
  EXPECT_EQ(events[0].details["area_id"], "lawn1");
  EXPECT_EQ(events[0].details["area_type"], "mow");
  EXPECT_EQ(events[0].details["obstacles"], 1);
  EXPECT_NEAR(events[0].details["area_m2"].get<double>(), 25.0, 0.5);
}

TEST(Recorder, NavigationAreaGoesAsNav) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, Square(2));
  ASSERT_EQ(recorder.OnAction("finish_navigation_area"), AreaRecorder::Result::kDone);
  auto edit = recorder.TakeMapEdit();
  ASSERT_TRUE(edit);
  EXPECT_EQ(edit->type, "nav");
  EXPECT_TRUE(edit->obstacles.empty());
}

TEST(Recorder, ABadRingStaysOpen) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {0.15, 0}});
  EXPECT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kRefused);
  EXPECT_TRUE(recorder.recording());
  EXPECT_FALSE(recorder.outline());
  // Saving does not get around it either.
  EXPECT_EQ(recorder.OnAction("finish_mowing_area"), AreaRecorder::Result::kRefused);
  EXPECT_FALSE(recorder.TakeMapEdit());
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].details["action"], "stop_recording");
  EXPECT_EQ(events[0].details["reason"], "RING_FEW_POINTS");
  // Driving on makes it a ring.
  Drive(recorder, {{0.15, 0}, {2, 0}, {2, 2}, {0, 0.1}});
  EXPECT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kDone);
}

TEST(Recorder, AFigureEightIsRefused) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {2, 2}, {2, 0}, {0, 2}});
  EXPECT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kRefused);
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], "RING_CROSSES_ITSELF");
  EXPECT_EQ(events[0].details["message"], "Cannot close the ring: the line crosses itself");
}

TEST(Recorder, NoSavingWhileTheMapIsNotWritable) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, Square(2));
  recorder.OnAction("stop_recording");
  recorder.SetMapWritable(false, MapReason(MapDatum::kMismatch, "The map was recorded for another datum"));
  EXPECT_EQ(recorder.OnAction("finish_mowing_area"), AreaRecorder::Result::kRefused);
  EXPECT_FALSE(recorder.TakeMapEdit());
  EXPECT_TRUE(recorder.outline());
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], "MAP_MISMATCH");
  EXPECT_EQ(events[0].details["message"], "The map was recorded for another datum");
  recorder.SetMapWritable(true, Reason());
  EXPECT_EQ(recorder.OnAction("finish_mowing_area"), AreaRecorder::Result::kDone);
}

TEST(Recorder, ARefusedSaveKeepsTheArea) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, Square(2));
  recorder.OnAction("finish_mowing_area");
  ASSERT_TRUE(recorder.TakeMapEdit());
  recorder.OnMapEditResult(false, "", "STORAGE", "Cannot store the map: disk full");
  EXPECT_TRUE(recorder.outline());
  EXPECT_TRUE(On(recorder, "finish_mowing_area"));
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "RECORDING_REFUSED");
  EXPECT_EQ(events[0].details["action"], "finish_mowing_area");
  EXPECT_EQ(events[0].details["reason"], "SAVE_FAILED");
  EXPECT_EQ(events[0].details["code"], "STORAGE");
  EXPECT_EQ(events[0].details["message"], "Cannot store the map: disk full");
  EXPECT_EQ(events[0].details["severity"], "error");
  // A late second answer changes nothing.
  recorder.OnMapEditResult(true, "x", "", "");
  EXPECT_TRUE(recorder.outline());
  EXPECT_TRUE(recorder.TakeEvents().empty());
}

TEST(Recorder, DiscardDropsEverything) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, Square(2));
  recorder.OnAction("stop_recording");
  recorder.OnAction("start_recording");
  Drive(recorder, {{0.5, 0.5}, {1, 0.5}});
  ASSERT_EQ(recorder.OnAction("finish_discard"), AreaRecorder::Result::kDone);
  EXPECT_FALSE(recorder.outline());
  EXPECT_FALSE(recorder.recording());
  EXPECT_TRUE(recorder.current().empty());
  EXPECT_TRUE(On(recorder, "start_recording"));
  EXPECT_FALSE(On(recorder, "finish_discard"));
}

TEST(Recorder, DisabledActionsAreRefused) {
  AreaRecorder recorder = Writable();
  EXPECT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kRefused);
  EXPECT_EQ(recorder.OnAction("finish_mowing_area"), AreaRecorder::Result::kRefused);
  EXPECT_EQ(recorder.OnAction("start_manual_mowing"), AreaRecorder::Result::kRefused);
  EXPECT_EQ(recorder.OnAction("record_nothing"), AreaRecorder::Result::kUnknown);
}

TEST(Recorder, StopsCollectingAtTheLimit) {
  RecorderConfig config;
  config.max_points = 5;
  AreaRecorder recorder = Writable(config);
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {3, 0}});
  EXPECT_EQ(recorder.current().size(), 5u);
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], "TOO_MANY_POINTS");
  EXPECT_EQ(events[0].details["limit"], 5);
}

namespace {

HardwareSnapshot Healthy() {
  HardwareSnapshot hw;
  hw.telemetry_ok = true;
  hw.battery_ok = true;
  hw.battery_voltage = 24.5f;
  hw.battery_percentage = 80;
  return hw;
}

PoseSnapshot At(double x, double y, float accuracy = 0.02f) {
  PoseSnapshot pose;
  pose.x = x;
  pose.y = y;
  pose.position_accuracy = accuracy;
  pose.orientation_accuracy = 0.05f;
  pose.orientation_valid = true;
  return pose;
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

void DriveCore(LogicCore& core, const Ring& corners) {
  for (size_t i = 0; i + 1 < corners.size(); i++) {
    const Point2& a = corners[i];
    const Point2& b = corners[i + 1];
    const int steps = static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.05));
    for (int s = 0; s < steps; s++) {
      const double t = static_cast<double>(s) / steps;
      core.OnPose(At(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
    }
  }
}

}  // namespace

TEST(LogicRecording, NeedsAMapThatFitsTheDatum) {
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  // No map/status yet.
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  EXPECT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kRejected);
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "RECORDING_REFUSED");
  EXPECT_EQ(events[0].details["reason"], "MAP_UNKNOWN");
  EXPECT_EQ(events[0].details["message"], "The map status is not known yet");

  core.OnMapStatus(MapDatum::kMismatch, "The map was recorded for the datum 52, 9");
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  core.OnAction("mower_logic:idle/start_area_recording");
  const auto refused = core.TakeEvents();
  EXPECT_EQ(refused[0].details["reason"], "MAP_MISMATCH");
  EXPECT_EQ(refused[0].details["message"], "The map was recorded for the datum 52, 9");
  core.OnMapStatus(MapDatum::kUnbound, "");
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  core.OnMapStatus(MapDatum::kUnknown, "");
  EXPECT_FALSE(Enabled(core, "mower_logic:idle", "start_area_recording"));

  // An empty map takes the datum in use with its first area.
  core.TakeActionsChanged();
  core.OnMapStatus(MapDatum::kEmpty, "The map is empty");
  EXPECT_TRUE(core.TakeActionsChanged());
  EXPECT_TRUE(Enabled(core, "mower_logic:idle", "start_area_recording"));
  core.OnMapStatus(MapDatum::kMatch, "");
  EXPECT_EQ(core.OnAction("mower_logic:idle/start_area_recording"), ActionResult::kDone);
}

TEST(LogicRecording, RecordsAndSavesThroughTheCore) {
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnAction("mower_logic:idle/start_area_recording");
  // Outside of AREA_RECORDING all of them are off, now the recorder decides.
  EXPECT_TRUE(Enabled(core, "mower_logic:area_recording", "start_recording"));
  EXPECT_FALSE(Enabled(core, "mower_logic:area_recording", "stop_recording"));
  ASSERT_EQ(core.OnAction(kPrefix + "start_recording"), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().sub_state, "RECORD_OUTLINE");
  DriveCore(core, Square(3));
  ASSERT_EQ(core.OnAction(kPrefix + "finish_mowing_area"), ActionResult::kDone);
  EXPECT_EQ(core.RobotState().sub_state, "");
  auto edit = core.TakeMapEdit();
  ASSERT_TRUE(edit);
  core.TakeEvents();
  core.OnMapEditResult(true, "a1", "", "");
  auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "AREA_RECORDED");
  // At the position of the mower, like every event.
  EXPECT_NEAR(events[0].details["x"].get<double>(), 0.0, 0.1);
  EXPECT_NEAR(events[0].details["y"].get<double>(), 0.05, 0.1);
}

TEST(LogicRecording, ExitDropsWhatIsNotSaved) {
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnAction("mower_logic:idle/start_area_recording");
  core.OnAction(kPrefix + "start_recording");
  DriveCore(core, Square(3));
  core.OnAction(kPrefix + "stop_recording");
  ASSERT_EQ(core.OnAction(kPrefix + "exit_recording_mode"), ActionResult::kDone);
  EXPECT_FALSE(core.recorder().outline());
  for (const auto& name : AreaRecorder::ActionNames()) {
    EXPECT_FALSE(Enabled(core, "mower_logic:area_recording", name)) << name;
  }
  // In IDLE the recording actions are refused.
  EXPECT_EQ(core.OnAction(kPrefix + "start_recording"), ActionResult::kRejected);
  // Poses outside of AREA_RECORDING make no points.
  core.OnAction("mower_logic:idle/start_area_recording");
  EXPECT_TRUE(core.recorder().current().empty());
}

TEST(LogicRecording, ManualMowingStaysOff) {
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnAction("mower_logic:idle/start_area_recording");
  EXPECT_FALSE(Enabled(core, "mower_logic:area_recording", "start_manual_mowing"));
  EXPECT_EQ(core.OnAction(kPrefix + "start_manual_mowing"), ActionResult::kRejected);
}

TEST(RecorderDock, TwoPointsGiveTheHeading) {
  AreaRecorder recorder = Writable();
  EXPECT_TRUE(On(recorder, "record_dock"));
  recorder.OnPose(2, 1, 0.02f, true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  EXPECT_EQ(recorder.SubState(), "RECORD_DOCKING_POSITION");
  // The step the app calls dock: record_dock without start_recording.
  EXPECT_TRUE(On(recorder, "record_dock"));
  EXPECT_FALSE(On(recorder, "start_recording"));
  EXPECT_FALSE(On(recorder, "finish_mowing_area"));
  EXPECT_FALSE(recorder.TakeMapEdit());
  recorder.OnPose(2, 2, 0.02f, true);
  recorder.SetDockContact(true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  auto edit = recorder.TakeMapEdit();
  ASSERT_TRUE(edit);
  EXPECT_EQ(edit->kind, MapEdit::Kind::kDock);
  EXPECT_DOUBLE_EQ(edit->x, 2);
  EXPECT_DOUBLE_EQ(edit->y, 2);
  EXPECT_NEAR(edit->heading, M_PI / 2, 1e-9);
  recorder.TakeEvents();
  recorder.OnMapEditResult(true, "dock1", "", "");
  EXPECT_EQ(recorder.SubState(), "");
  EXPECT_TRUE(On(recorder, "start_recording"));
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "DOCK_RECORDED");
  EXPECT_EQ(events[0].details["dock_id"], "dock1");
  EXPECT_NEAR(events[0].details["heading"].get<double>(), M_PI / 2, 1e-9);
}

TEST(RecorderDock, TooCloseKeepsTheFirstPoint) {
  AreaRecorder recorder = Writable();
  recorder.OnPose(0, 0, 0.02f, true);
  recorder.OnAction("record_dock");
  recorder.SetDockContact(true);
  recorder.OnPose(0.45, 0, 0.02f, true);
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  EXPECT_FALSE(recorder.TakeMapEdit());
  EXPECT_EQ(recorder.SubState(), "RECORD_DOCKING_POSITION");
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], "DOCK_TOO_CLOSE");
  EXPECT_NEAR(events[0].details["distance"].get<double>(), 0.45, 1e-9);
  EXPECT_DOUBLE_EQ(events[0].details["limit"].get<double>(), 0.5);
  // Backwards onto the contacts: the heading points the way the mower moved.
  recorder.OnPose(-1, 0, 0.02f, true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  EXPECT_NEAR(std::fabs(recorder.TakeMapEdit()->heading), M_PI, 1e-9);
}

TEST(RecorderDock, NeedsAnAccuratePositionAndAWritableMap) {
  AreaRecorder recorder = Writable();
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  recorder.OnPose(0, 0, 0.4f, true);
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  recorder.OnPose(0, 0, 0.02f, true);
  recorder.SetMapWritable(false, MapReason(MapDatum::kMismatch, "The map was recorded for another datum"));
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 3u);
  EXPECT_EQ(events[0].details["reason"], "NO_POSITION");
  EXPECT_EQ(events[1].details["reason"], "POSITION_ACCURACY");
  EXPECT_FLOAT_EQ(events[1].details["accuracy"].get<float>(), 0.4f);
  EXPECT_EQ(events[2].details["reason"], "MAP_MISMATCH");
  EXPECT_EQ(recorder.SubState(), "");
}

TEST(RecorderDock, ARefusedSaveKeepsTheFirstPoint) {
  AreaRecorder recorder = Writable();
  recorder.OnPose(0, 0, 0.02f, true);
  recorder.OnAction("record_dock");
  recorder.SetDockContact(true);
  recorder.OnPose(1, 0, 0.02f, true);
  recorder.OnAction("record_dock");
  ASSERT_TRUE(recorder.TakeMapEdit());
  recorder.OnMapEditResult(false, "", "STORAGE", "Cannot store the map: disk full");
  EXPECT_EQ(recorder.SubState(), "RECORD_DOCKING_POSITION");
  EXPECT_EQ(recorder.TakeEvents().back().details["action"], "record_dock");
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  EXPECT_TRUE(recorder.TakeMapEdit());
}

TEST(RecorderDock, ThePointOnTheContactsNeedsTheContact) {
  RecorderConfig config;
  config.dock_backwards = true;
  AreaRecorder recorder = Writable(config);
  // The first point needs none.
  recorder.OnPose(0, 0, 0.02f, true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  EXPECT_EQ(recorder.TakeNotices().back(),
            "Recorded the point in front of the docking station, now drive backwards onto the contacts");
  recorder.OnPose(-1, 0, 0.02f, true);
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  EXPECT_FALSE(recorder.TakeMapEdit());
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "RECORDING_REFUSED");
  EXPECT_EQ(events[0].details["action"], "record_dock");
  EXPECT_EQ(events[0].details["reason"], "NO_DOCK_CONTACT");
  EXPECT_EQ(events[0].details["severity"], "warning");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_NE(events[0].details["message"].get<std::string>().find("backwards onto the contacts"), std::string::npos);
  // The first point stays.
  EXPECT_EQ(recorder.SubState(), "RECORD_DOCKING_POSITION");
  recorder.SetDockContact(true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  const auto edit = recorder.TakeMapEdit();
  ASSERT_TRUE(edit);
  EXPECT_DOUBLE_EQ(edit->x, -1);
  EXPECT_NEAR(std::fabs(edit->heading), M_PI, 1e-9);
}

TEST(RecorderDock, BothPointsNeedAValidHeading) {
  AreaRecorder recorder = Writable();
  recorder.OnPose(0, 0, 0.02f, false);
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  EXPECT_EQ(recorder.SubState(), "");
  recorder.OnPose(0, 0, 0.02f, true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  recorder.SetDockContact(true);
  recorder.OnPose(0, 1, 0.02f, false);
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].details["reason"], "NO_HEADING");
  EXPECT_EQ(events[1].details["reason"], "NO_HEADING");
  EXPECT_EQ(events[1].details["severity"], "warning");
  EXPECT_EQ(recorder.SubState(), "RECORD_DOCKING_POSITION");
  recorder.OnPose(0, 1, 0.02f, true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  EXPECT_NEAR(recorder.TakeMapEdit()->heading, M_PI / 2, 1e-9);
}

TEST(RecorderDock, DiscardDropsTheFirstPoint) {
  AreaRecorder recorder = Writable();
  recorder.OnPose(0, 0, 0.02f, true);
  recorder.OnAction("record_dock");
  EXPECT_TRUE(On(recorder, "finish_discard"));
  ASSERT_EQ(recorder.OnAction("finish_discard"), AreaRecorder::Result::kDone);
  EXPECT_EQ(recorder.SubState(), "");
  EXPECT_TRUE(On(recorder, "start_recording"));
}

TEST(RecorderDock, NotWhileAnAreaIsRecorded) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  EXPECT_FALSE(On(recorder, "record_dock"));
  Drive(recorder, Square(2));
  recorder.OnAction("stop_recording");
  EXPECT_FALSE(On(recorder, "record_dock"));
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
}

TEST(RecorderOverlay, EmptyAtFirst) {
  AreaRecorder recorder = Writable();
  EXPECT_TRUE(recorder.TakeOverlayChanged());
  EXPECT_EQ(recorder.Overlay().dump(), R"({"polygons":[]})");
  EXPECT_FALSE(recorder.TakeOverlayChanged());
}

TEST(RecorderOverlay, LinesInBothFormats) {
  AreaRecorder recorder = Writable();
  recorder.TakeOverlayChanged();
  recorder.OnAction("start_recording");
  recorder.OnPose(0, 0, 0.02f, true);
  // One point is no line yet.
  EXPECT_TRUE(recorder.Overlay()["polygons"].empty());
  recorder.OnPose(1, 0, 0.02f, true);
  EXPECT_TRUE(recorder.TakeOverlayChanged());
  json line = recorder.Overlay()["polygons"][0];
  EXPECT_EQ(line["is_closed"], 0);
  EXPECT_EQ(line["closed"], 0);
  EXPECT_EQ(line["color"], "blue");
  EXPECT_DOUBLE_EQ(line["line_width"].get<double>(), 0.1);
  ASSERT_EQ(line["poly"].size(), 2u);
  EXPECT_DOUBLE_EQ(line["poly"][1]["x"].get<double>(), 1.0);
  EXPECT_EQ(line["polygon"]["points"], line["poly"]);

  // A pose too close changes nothing.
  recorder.OnPose(1.05, 0, 0.02f, true);
  EXPECT_FALSE(recorder.TakeOverlayChanged());

  Drive(recorder, {{1, 0}, {1, 1}, {0, 1}, {0, 0.05}});
  recorder.OnAction("stop_recording");
  EXPECT_TRUE(recorder.TakeOverlayChanged());
  json polygons = recorder.Overlay()["polygons"];
  ASSERT_EQ(polygons.size(), 1u);
  EXPECT_EQ(polygons[0]["color"], "green");
  EXPECT_EQ(polygons[0]["is_closed"], 1);
  EXPECT_EQ(polygons[0]["closed"], 1);

  recorder.OnAction("start_recording");
  Drive(recorder, {{0.2, 0.2}, {0.6, 0.2}, {0.6, 0.6}, {0.2, 0.55}});
  recorder.OnAction("stop_recording");
  polygons = recorder.Overlay()["polygons"];
  ASSERT_EQ(polygons.size(), 2u);
  EXPECT_EQ(polygons[1]["color"], "red");

  recorder.OnAction("finish_mowing_area");
  recorder.TakeMapEdit();
  recorder.TakeOverlayChanged();
  recorder.OnMapEditResult(true, "a", "", "");
  EXPECT_TRUE(recorder.TakeOverlayChanged());
  EXPECT_TRUE(recorder.Overlay()["polygons"].empty());
}

TEST(RecorderOverlay, LeavingRecordingClearsIt) {
  LogicCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnAction("mower_logic:idle/start_area_recording");
  core.OnAction(kPrefix + "start_recording");
  DriveCore(core, {{0, 0}, {1, 0}});
  EXPECT_EQ(core.MapOverlay()["polygons"].size(), 1u);
  core.TakeOverlayChanged();
  core.OnAction(kPrefix + "exit_recording_mode");
  EXPECT_TRUE(core.TakeOverlayChanged());
  EXPECT_TRUE(core.MapOverlay()["polygons"].empty());
}
