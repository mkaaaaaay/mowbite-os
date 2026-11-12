#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/logic_core.h"
#include "logic_test_core.h"

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

// Drives the mower forward along the corners in steps of 0.05 m with a pose at every step, turning on the spot at
// each corner.
void Drive(AreaRecorder& recorder, const Ring& corners, float accuracy = 0.02f) {
  double heading = 0;
  for (size_t i = 0; i + 1 < corners.size(); i++) {
    const Point2& a = corners[i];
    const Point2& b = corners[i + 1];
    heading = std::atan2(b.y - a.y, b.x - a.x);
    const int steps = static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.05));
    for (int s = 0; s <= steps; s++) {
      const double t = static_cast<double>(s) / steps;
      recorder.OnPose(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, heading, accuracy, true);
    }
  }
}

// Around a rectangle of w by h metres from (x0, y0); at the end across the first side at 18 degrees, 0.45 m from the
// start, and on to 0.6 m past it.
Ring Lap(double w, double h, double x0 = 0, double y0 = 0) {
  return {{x0, y0}, {x0 + w, y0}, {x0 + w, y0 + h}, {x0, y0 + h}, {x0, y0 + 0.15}, {x0 + 0.6, y0 - 0.05}};
}

Ring Square(double side, double x0 = 0, double y0 = 0) {
  return Lap(side, side, x0, y0);
}

// Closes the open ring as the outline and saves it: why it was not driven, empty if it was.
std::string DrivenReason(AreaRecorder& recorder) {
  if (recorder.OnAction("finish_mowing_area") != AreaRecorder::Result::kDone) return "not saved";
  return recorder.TakeMapEdit()->outline_driven_reason;
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

TEST(Ring, CutOverhangKeepsTheLoopFromTheCrossing) {
  // The last segment runs through the first at (0.444, 0), past the start.
  const Ring ring = {{0, 0}, {4, 0}, {4, 4}, {0, 4}, {0.5, -0.5}};
  const auto cut = CutOverhang(ring, 2.0);
  ASSERT_TRUE(cut);
  ASSERT_EQ(cut->ring.size(), 4u);
  EXPECT_NEAR(cut->ring[0].x, 4.0 / 9, 1e-9);
  EXPECT_NEAR(cut->ring[0].y, 0, 1e-9);
  EXPECT_DOUBLE_EQ(cut->ring[3].x, 0);
  EXPECT_DOUBLE_EQ(cut->ring[3].y, 4);
  EXPECT_EQ(cut->dropped, 2u);
  EXPECT_TRUE(RingProblem(cut->ring, 0.05).empty());
  // Before the crossing 0.44 m, after it 0.5 m.
  EXPECT_FALSE(CutOverhang(ring, 0.9));
}

TEST(Ring, CutOverhangAtACornerKeepsTheCorner) {
  // The last segment runs through the point (1, 0).
  const auto cut = CutOverhang({{0, 0}, {1, 0}, {2, 0}, {2, 2}, {1, 2}, {1, -1}}, 2.0);
  ASSERT_TRUE(cut);
  ASSERT_EQ(cut->ring.size(), 4u);
  EXPECT_DOUBLE_EQ(cut->ring[0].x, 1);
  EXPECT_DOUBLE_EQ(cut->ring[0].y, 0);
  EXPECT_EQ(cut->dropped, 2u);
  EXPECT_TRUE(RingProblem(cut->ring, 0.05).empty());
}

TEST(Ring, CutOverhangNeedsACrossingNearTheEnds) {
  EXPECT_FALSE(CutOverhang({{0, 0}, {4, 0}, {4, 4}, {0, 4}}, 2.0));
  // A figure eight: half of it lies before and after the crossing.
  EXPECT_FALSE(CutOverhang({{0, 0}, {2, 2}, {2, 0}, {0, 2}}, 2.0));
}

TEST(Recorder, EveryActionHasALabelAndAFlag) {
  EXPECT_EQ(AreaRecorder::ActionLabels().size(), AreaRecorder::ActionNames().size());
  EXPECT_EQ(Writable().Enabled().size(), AreaRecorder::ActionNames().size());
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
  recorder.OnPose(0, 0, 0, 0.02f, true);
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
  recorder.OnPose(0, 0, 0, 0.02f, true);
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
  recorder.OnPose(0, 0, 0, std::nanf(""), true);
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
  recorder.OnPose(1, 0, 0, 0.02f, true);
  ASSERT_EQ(recorder.current().size(), 1u);
  // A second one at the same spot is refused with the distance.
  recorder.OnAction("collect_point");
  recorder.OnPose(1.05, 0, 0, 0.02f, true);
  EXPECT_EQ(recorder.current().size(), 1u);
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["action"], "collect_point");
  EXPECT_EQ(events[0].details["reason"], "POINT_TOO_CLOSE");
  EXPECT_NEAR(events[0].details["distance"].get<double>(), 0.05, 1e-9);
  EXPECT_DOUBLE_EQ(events[0].details["limit"].get<double>(), 0.1);
  recorder.OnAction("collect_point");
  recorder.OnPose(1.5, 0, 0, 0.02f, true);
  EXPECT_EQ(recorder.current().size(), 2u);
}

TEST(Recorder, ManualPointWithBadAccuracyOrLostPoseIsRefused) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  recorder.OnAction("auto_point_collecting_disable");
  recorder.OnPose(0, 0, 0, 0.02f, true);
  recorder.OnAction("collect_point");
  recorder.OnPose(0, 0, 0, 0.5f, true);
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

TEST(Recorder, ClosingCutsTheEndPastTheStart) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {4, 0}, {4, 4}, {0.37, 4}, {0.37, -0.6}});
  const size_t points = recorder.current().size();
  ASSERT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kDone);
  ASSERT_TRUE(recorder.outline());
  const size_t trimmed = recorder.trimmed_points();
  EXPECT_GE(trimmed, 4u);
  // The crossing comes in as a point of its own.
  EXPECT_EQ(recorder.outline()->size(), points - trimmed + 1);
  for (const auto& p : *recorder.outline()) {
    EXPECT_GE(p.y, -1e-9);
    EXPECT_GE(p.x, 0.37 - 1e-9);
  }
  EXPECT_TRUE(recorder.TakeEvents().empty());
  EXPECT_NEAR(RingArea(*recorder.outline()), 3.63 * 4, 0.3);
}

TEST(Recorder, AFarOverhangNeedsUndo) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {4, 0}, {4, 4}, {0.37, 4}, {0.37, -3}});
  EXPECT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kRefused);
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], "RING_CROSSES_ITSELF");
  ASSERT_EQ(recorder.OnAction("undo_points", nullptr, 3.2), AreaRecorder::Result::kDone);
  EXPECT_GT(recorder.current().back().y, 0);
  EXPECT_LT(recorder.current().back().y, 0.5);
  ASSERT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kDone);
  EXPECT_EQ(recorder.trimmed_points(), 0u);
}

TEST(Recorder, UndoTakesBackThePathAndStopsAutoPoints) {
  AreaRecorder recorder = Writable();
  EXPECT_FALSE(On(recorder, "undo_points"));
  EXPECT_EQ(recorder.OnAction("undo_points"), AreaRecorder::Result::kRefused);
  recorder.OnAction("start_recording");
  EXPECT_FALSE(On(recorder, "undo_points")) << "no point yet";
  recorder.TakeActionsChanged();
  Drive(recorder, {{0, 0}, {3, 0}});
  EXPECT_TRUE(recorder.TakeActionsChanged()) << "undo_points comes with the first point";
  EXPECT_TRUE(On(recorder, "undo_points"));
  recorder.TakeOverlayChanged();
  const size_t before = recorder.current().size();
  ASSERT_EQ(recorder.OnAction("undo_points"), AreaRecorder::Result::kDone);
  // 1 m by default: the end is at most 2 m from the start now, and not much less.
  EXPECT_LE(recorder.current().back().x, 2.0 + 1e-9);
  EXPECT_GT(recorder.current().back().x, 1.7);
  EXPECT_GE(before - recorder.current().size(), 7u);
  EXPECT_TRUE(recorder.TakeOverlayChanged());
  EXPECT_TRUE(recorder.TakeActionsChanged());
  EXPECT_TRUE(On(recorder, "auto_point_collecting_enable"));
  EXPECT_FALSE(On(recorder, "auto_point_collecting_disable"));
  // Without automatic points the mower may drive back to the new end.
  const size_t left = recorder.current().size();
  Drive(recorder, {{3, 0}, {2, 0}});
  EXPECT_EQ(recorder.current().size(), left);
  EXPECT_TRUE(recorder.TakeEvents().empty());

  ASSERT_EQ(recorder.OnAction("undo_points", nullptr, 50), AreaRecorder::Result::kDone);
  EXPECT_TRUE(recorder.current().empty());
  EXPECT_FALSE(On(recorder, "undo_points"));
  EXPECT_TRUE(recorder.recording());
}

TEST(Recorder, ARingWithoutPointsJustEnds) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  ASSERT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kDone);
  EXPECT_EQ(recorder.trimmed_points(), 0u);
  EXPECT_FALSE(recorder.recording());
  EXPECT_FALSE(recorder.outline());
  EXPECT_TRUE(On(recorder, "start_recording"));
  EXPECT_TRUE(recorder.TakeEvents().empty());

  // An obstacle begun without a point does not stop the save.
  recorder.OnAction("start_recording");
  Drive(recorder, Square(3));
  recorder.OnAction("stop_recording");
  recorder.OnAction("start_recording");
  ASSERT_EQ(recorder.OnAction("finish_mowing_area"), AreaRecorder::Result::kDone);
  const auto edit = recorder.TakeMapEdit();
  ASSERT_TRUE(edit);
  EXPECT_TRUE(edit->obstacles.empty());
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

TEST(Recorder, EveryRingAtTheLimitReportsIt) {
  RecorderConfig config;
  // Reached on the second or third side of the square, so the ring encloses an area.
  config.max_points = 20;
  AreaRecorder recorder = Writable(config);
  const auto full = [&recorder]() {
    const auto events = recorder.TakeEvents();
    return events.size() == 1 && events[0].details["reason"] == "TOO_MANY_POINTS";
  };
  // The second area of a session meets the limit like the first.
  for (const char* id : {"a", "b"}) {
    recorder.OnAction("start_recording");
    Drive(recorder, Square(1));
    EXPECT_TRUE(full()) << id;
    ASSERT_EQ(recorder.OnAction("finish_mowing_area"), AreaRecorder::Result::kDone) << id;
    ASSERT_TRUE(recorder.TakeMapEdit()) << id;
    recorder.OnMapEditResult(true, id, "", "");
    recorder.TakeEvents();
  }
  // An obstacle begun at the limit takes no point and says why.
  recorder.OnAction("start_recording");
  Drive(recorder, Square(1));
  EXPECT_TRUE(full());
  ASSERT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kDone);
  recorder.OnAction("start_recording");
  Drive(recorder, Square(0.5, 0.2, 0.2));
  EXPECT_TRUE(recorder.current().empty());
  EXPECT_TRUE(full());
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

void DriveCore(TestCore& core, const Ring& corners) {
  for (size_t i = 0; i + 1 < corners.size(); i++) {
    const Point2& a = corners[i];
    const Point2& b = corners[i + 1];
    const int steps = static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.05));
    for (int s = 0; s <= steps; s++) {
      const double t = static_cast<double>(s) / steps;
      PoseSnapshot pose = At(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
      pose.heading = std::atan2(b.y - a.y, b.x - a.x);
      core.OnPose(pose);
    }
  }
}

}  // namespace

TEST(LogicRecording, NeedsAMapThatFitsTheDatum) {
  TestCore core{LogicConfig{}};
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
  TestCore core{LogicConfig{}};
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
  EXPECT_NEAR(events[0].details["x"].get<double>(), 0.6, 0.1);
  EXPECT_NEAR(events[0].details["y"].get<double>(), -0.05, 0.1);
}

TEST(LogicRecording, ExitDropsWhatIsNotSaved) {
  TestCore core{LogicConfig{}};
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
  TestCore core{LogicConfig{}};
  core.OnHardwareStatus(Healthy());
  core.OnMapStatus(MapDatum::kMatch, "");
  core.OnAction("mower_logic:idle/start_area_recording");
  EXPECT_FALSE(Enabled(core, "mower_logic:area_recording", "start_manual_mowing"));
  EXPECT_EQ(core.OnAction(kPrefix + "start_manual_mowing"), ActionResult::kRejected);
}

TEST(RecorderDock, TwoPointsGiveTheHeading) {
  AreaRecorder recorder = Writable();
  EXPECT_TRUE(On(recorder, "record_dock"));
  recorder.OnPose(2, 1, 0, 0.02f, true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  EXPECT_EQ(recorder.SubState(), "RECORD_DOCKING_POSITION");
  // The step the app calls dock: record_dock without start_recording.
  EXPECT_TRUE(On(recorder, "record_dock"));
  EXPECT_FALSE(On(recorder, "start_recording"));
  EXPECT_FALSE(On(recorder, "finish_mowing_area"));
  EXPECT_FALSE(recorder.TakeMapEdit());
  recorder.OnPose(2, 2, 0, 0.02f, true);
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
  recorder.OnPose(0, 0, 0, 0.02f, true);
  recorder.OnAction("record_dock");
  recorder.SetDockContact(true);
  recorder.OnPose(0.45, 0, 0, 0.02f, true);
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  EXPECT_FALSE(recorder.TakeMapEdit());
  EXPECT_EQ(recorder.SubState(), "RECORD_DOCKING_POSITION");
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["reason"], "DOCK_TOO_CLOSE");
  EXPECT_NEAR(events[0].details["distance"].get<double>(), 0.45, 1e-9);
  EXPECT_DOUBLE_EQ(events[0].details["limit"].get<double>(), 0.5);
  // Backwards onto the contacts: the heading points the way the mower moved.
  recorder.OnPose(-1, 0, 0, 0.02f, true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  EXPECT_NEAR(std::fabs(recorder.TakeMapEdit()->heading), M_PI, 1e-9);
}

TEST(RecorderDock, NeedsAnAccuratePositionAndAWritableMap) {
  AreaRecorder recorder = Writable();
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  recorder.OnPose(0, 0, 0, 0.4f, true);
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  recorder.OnPose(0, 0, 0, 0.02f, true);
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
  recorder.OnPose(0, 0, 0, 0.02f, true);
  recorder.OnAction("record_dock");
  recorder.SetDockContact(true);
  recorder.OnPose(1, 0, 0, 0.02f, true);
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
  recorder.OnPose(0, 0, 0, 0.02f, true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  EXPECT_EQ(recorder.TakeNotices().back(),
            "Recorded the point in front of the docking station, now drive backwards onto the contacts");
  recorder.OnPose(-1, 0, 0, 0.02f, true);
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
  recorder.OnPose(0, 0, 0, 0.02f, false);
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  EXPECT_EQ(recorder.SubState(), "");
  recorder.OnPose(0, 0, 0, 0.02f, true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  recorder.SetDockContact(true);
  recorder.OnPose(0, 1, 0, 0.02f, false);
  EXPECT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kRefused);
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].details["reason"], "NO_HEADING");
  EXPECT_EQ(events[1].details["reason"], "NO_HEADING");
  EXPECT_EQ(events[1].details["severity"], "warning");
  EXPECT_EQ(recorder.SubState(), "RECORD_DOCKING_POSITION");
  recorder.OnPose(0, 1, 0, 0.02f, true);
  ASSERT_EQ(recorder.OnAction("record_dock"), AreaRecorder::Result::kDone);
  EXPECT_NEAR(recorder.TakeMapEdit()->heading, M_PI / 2, 1e-9);
}

TEST(RecorderDock, DiscardDropsTheFirstPoint) {
  AreaRecorder recorder = Writable();
  recorder.OnPose(0, 0, 0, 0.02f, true);
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

TEST(RecorderOverlay, LinesInMillimetres) {
  AreaRecorder recorder = Writable();
  recorder.TakeOverlayChanged();
  recorder.OnAction("start_recording");
  recorder.OnPose(0, 0, 0, 0.02f, true);
  // One point is no line yet.
  EXPECT_TRUE(recorder.Overlay()["polygons"].empty());
  recorder.OnPose(1, 0, 0, 0.02f, true);
  EXPECT_TRUE(recorder.TakeOverlayChanged());
  json line = recorder.Overlay()["polygons"][0];
  EXPECT_EQ(line["is_closed"], 0);
  EXPECT_EQ(line["color"], "blue");
  EXPECT_DOUBLE_EQ(line["line_width"].get<double>(), 0.1);
  ASSERT_EQ(line["poly"].size(), 2u);
  EXPECT_DOUBLE_EQ(line["poly"][1]["x"].get<double>(), 1.0);
  // Only the form of OpenMower.
  EXPECT_EQ(line.size(), 4u);
  recorder.OnPose(1.123456, 0.2, 0, 0.02f, true);
  EXPECT_EQ(recorder.Overlay()["polygons"][0]["poly"][2].dump(), R"({"x":1.123,"y":0.2})");
  recorder.OnAction("undo_points", nullptr, 0.1);
  EXPECT_TRUE(recorder.TakeOverlayChanged());
  EXPECT_EQ(recorder.Overlay()["polygons"][0]["poly"].size(), 2u);
  recorder.OnAction("auto_point_collecting_enable");

  // A pose too close changes nothing.
  recorder.OnPose(1.05, 0, 0, 0.02f, true);
  EXPECT_FALSE(recorder.TakeOverlayChanged());

  Drive(recorder, {{1, 0}, {1, 1}, {0, 1}, {0, 0.05}});
  recorder.OnAction("stop_recording");
  EXPECT_TRUE(recorder.TakeOverlayChanged());
  json polygons = recorder.Overlay()["polygons"];
  ASSERT_EQ(polygons.size(), 1u);
  EXPECT_EQ(polygons[0]["color"], "green");
  EXPECT_EQ(polygons[0]["is_closed"], 1);

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
  TestCore core{LogicConfig{}};
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

namespace {

// A ring as the recorder takes it along the corners: a point whenever the mower is more than 0.1 m from the last one,
// turning on the spot at each corner; the heading is that of the side it drives on. noise: sigma (m) of each point,
// and a tenth of it in rad for the heading.
struct Track {
  Ring ring;
  std::vector<PointMark> marks;
};

Track Recorded(const Ring& corners, double noise = 0, unsigned seed = 1) {
  Track track;
  for (size_t i = 0; i + 1 < corners.size(); i++) {
    const Point2& a = corners[i];
    const Point2& b = corners[i + 1];
    const double heading = std::atan2(b.y - a.y, b.x - a.x);
    const int steps = static_cast<int>(std::round(Distance(a, b) / 0.01));
    for (int s = 0; s <= steps; s++) {
      const Point2 at{a.x + (b.x - a.x) * s / steps, a.y + (b.y - a.y) * s / steps};
      if (!track.ring.empty() && Distance(track.ring.back(), at) <= 0.1) continue;
      track.ring.push_back(at);
      PointMark mark;
      mark.heading = heading;
      mark.heading_valid = mark.automatic = mark.joined = true;
      track.marks.push_back(mark);
    }
  }
  std::mt19937 source(seed);
  std::normal_distribution<double> error(0, noise > 0 ? noise : 1);
  if (noise > 0) {
    for (auto& p : track.ring) {
      p.x += error(source);
      p.y += error(source);
    }
    for (auto& mark : track.marks) mark.heading += error(source) / 10;
  }
  return track;
}

// A rectangle of w by h from the middle of its first side, ending 0.05 m short of the start on that side: a ring as
// closed over a crossing.
Ring Rectangle(double w, double h) {
  return {{w / 2, 0}, {w, 0}, {w, h}, {0, h}, {0, 0}, {w / 2 - 0.05, 0}};
}

// The same with its corners as arcs of radius r.
Ring Rounded(double w, double h, double r) {
  Ring corners;
  const double centres[4][3] = {{w - r, r, -M_PI / 2}, {w - r, h - r, 0}, {r, h - r, M_PI / 2}, {r, r, M_PI}};
  for (const auto& c : centres) {
    for (int k = 0; k <= 12; k++) {
      const double angle = c[2] + M_PI / 2 * k / 12;
      corners.push_back({c[0] + r * std::cos(angle), c[1] + r * std::sin(angle)});
    }
  }
  corners.push_back({corners.front().x - 0.05, corners.front().y});
  return corners;
}

}  // namespace

TEST(DrivenRing, StraightSidesAndCornersTurnedOnTheSpot) {
  const Track track = Recorded(Rectangle(3, 2));
  EXPECT_EQ(DrivenProblem(track.ring, track.marks, true), "");
}

TEST(DrivenRing, AnArc) {
  Track track;
  for (int k = 0; k < 125; k++) {
    const double angle = 2 * M_PI * k / 126;
    track.ring.push_back({2 * std::cos(angle), 2 * std::sin(angle)});
    PointMark mark;
    mark.heading = angle + M_PI / 2;
    mark.heading_valid = mark.automatic = mark.joined = true;
    track.marks.push_back(mark);
  }
  EXPECT_EQ(DrivenProblem(track.ring, track.marks, true), "");
}

TEST(DrivenRing, AQuarterTurnOnTheSpotFromTheRecorder) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, Lap(3, 2));
  EXPECT_EQ(DrivenReason(recorder), "");
}

TEST(DrivenRing, FiveHundredEdgesWithTheNoiseOfRtk) {
  // sigma 1.1 cm on every point. Over 1000 seeds 0.4 % of such rings fail by DIRECTION, and 5.5 % of rectangles with
  // corners turned on the spot by SHARP_TURN; with 5 mm none (the localization filters the pose).
  const Track track = Recorded(Rounded(17, 10.5, 1.0), 0.011, 7);
  ASSERT_GE(track.ring.size(), 500u);
  EXPECT_EQ(DrivenProblem(track.ring, track.marks, true), "");
  const Track rectangle = Recorded(Rectangle(17, 10.5), 0.005, 7);
  EXPECT_EQ(DrivenProblem(rectangle.ring, rectangle.marks, true), "");
}

TEST(DrivenRing, APointByHandOrWithoutHeadingOrAfterABreakOrReversing) {
  for (const char* code : {"MANUAL_POINT", "NO_HEADING", "INTERRUPTED", "REVERSED"}) {
    Track track = Recorded(Rectangle(3, 2));
    PointMark& mark = track.marks[10];
    if (std::string(code) == "MANUAL_POINT") mark.automatic = false;
    if (std::string(code) == "NO_HEADING") mark.heading_valid = false;
    if (std::string(code) == "INTERRUPTED") mark.joined = false;
    if (std::string(code) == "REVERSED") mark.reversed = true;
    EXPECT_EQ(DrivenProblem(track.ring, track.marks, true), code);
  }
  // The first point stands for the closing edge as well.
  Track track = Recorded(Rectangle(3, 2));
  track.marks[0].joined = false;
  EXPECT_EQ(DrivenProblem(track.ring, track.marks, true), "INTERRUPTED");
  track.marks[0].joined = true;
  track.marks[0].reversed = true;
  EXPECT_EQ(DrivenProblem(track.ring, track.marks, true), "REVERSED");
  // The first reason in the order of the list.
  track.marks[5].joined = false;
  EXPECT_EQ(DrivenProblem(track.ring, track.marks, true), "INTERRUPTED");
  track.marks[6].automatic = false;
  EXPECT_EQ(DrivenProblem(track.ring, track.marks, true), "MANUAL_POINT");
}

TEST(DrivenRing, AnEdgeLongerThanThirtyCentimetres) {
  Track track = Recorded(Rectangle(3, 2));
  // Points 0.11 m apart: one out of three gaps of 0.33 m.
  const Ring with_gap = [&] {
    Ring ring = track.ring;
    ring.erase(ring.begin() + 10, ring.begin() + 12);
    return ring;
  }();
  std::vector<PointMark> marks = track.marks;
  marks.erase(marks.begin() + 10, marks.begin() + 12);
  EXPECT_EQ(DrivenProblem(with_gap, marks, true), "GAP");
  // Up to 0.3 m: two of 0.11 m are 0.22.
  track.ring.erase(track.ring.begin() + 10);
  track.marks.erase(track.marks.begin() + 10);
  EXPECT_EQ(DrivenProblem(track.ring, track.marks, true), "");
}

TEST(DrivenRing, AnEdgeBackwardsOrSideways) {
  Track track = Recorded(Rectangle(3, 2));
  // Point 10 behind point 9 on the first side, heading still east.
  Track backwards = track;
  backwards.ring[10] = {backwards.ring[9].x - 0.05, 0};
  EXPECT_EQ(DrivenProblem(backwards.ring, backwards.marks, true), "DIRECTION");
  // A step of 0.2 m to the side with the same heading, then on along the new line.
  Track sideways = track;
  for (size_t k = 10; k < 20; k++) sideways.ring[k].y += 0.2;
  EXPECT_EQ(DrivenProblem(sideways.ring, sideways.marks, true), "DIRECTION");
}

TEST(DrivenRing, AHalfTurnOnTheSpot) {
  // Out along y = 0, a half turn on the spot, back the same way: which way it turned is not in the line.
  const Track track = Recorded({{0, 0}, {5, 0}, {4, 0}, {4, 2}, {-1, 2}, {-1, 0}, {-0.05, 0}});
  EXPECT_EQ(DrivenProblem(track.ring, track.marks, true), "SHARP_TURN");
  // Two quarter turns 0.1 m apart: each turn is clear, so the line is.
  const Track two = Recorded({{0, 0}, {5, 0}, {5, 0.1}, {4, 0.1}, {4, 2}, {-1, 2}, {-1, 0}, {-0.05, 0}});
  EXPECT_EQ(DrivenProblem(two.ring, two.marks, true), "");
}

TEST(DrivenRing, OnlyAFlatCrossingClosesTheRing) {
  // Without a crossing the closing edge was never driven, even 5 cm along the heading.
  const Track track = Recorded(Rectangle(3, 2));
  EXPECT_EQ(DrivenProblem(track.ring, track.marks, false), "CLOSING");
  // Before GAP: 0.5 m short of the start.
  const Track short_of = Recorded({{1.5, 0}, {3, 0}, {3, 2}, {0, 2}, {0, 0}, {1, 0}});
  EXPECT_EQ(DrivenProblem(short_of.ring, short_of.marks, false), "CLOSING");
  // Started at a corner and ended across the first side: a quarter turn at the first point.
  const Track corner = Recorded({{0, 0}, {3, 0}, {3, 2}, {0, 2}, {0, 0.05}});
  EXPECT_EQ(DrivenProblem(corner.ring, corner.marks, true), "CLOSING");
}

TEST(RecorderDriven, ReversingTakesPointsBackAndTheRingIsNotDriven) {
  AreaRecorder plain = Writable();
  plain.OnAction("start_recording");
  Drive(plain, Lap(3, 2));
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {2, 0}});
  const size_t before = recorder.current().size();
  // 1 m straight back, heading east, then forward again.
  for (int s = 1; s <= 20; s++) recorder.OnPose(2 - 0.05 * s, 0, 0, 0.02f, true);
  EXPECT_LT(recorder.current().size(), before - 7);
  EXPECT_LE(recorder.current().back().x, 1.0 + 1e-9);
  Drive(recorder, {{1, 0}, {3, 0}, {3, 2}, {0, 2}, {0, 0.15}, {0.6, -0.05}});
  EXPECT_NEAR(static_cast<double>(recorder.current().size()), static_cast<double>(plain.current().size()), 2);
  for (const auto& p : recorder.current()) EXPECT_TRUE(std::fabs(p.y) < 1e-9 || p.x > 2.9 || p.y > 0.1 || p.x < 0.7);
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].type, "RECORDING_REFUSED");
  EXPECT_EQ(events[0].details["action"], "point");
  EXPECT_EQ(events[0].details["reason"], "REVERSING");
  EXPECT_EQ(events[0].details["resume"], "auto");
  const auto notices = recorder.TakeNotices();
  EXPECT_NE(std::find_if(notices.begin(), notices.end(),
                         [](const std::string& line) { return line.find("while reversing") != std::string::npos; }),
            notices.end());
  // The edge after the stretch backwards was never driven; with an offset the body would swing beyond the track.
  EXPECT_EQ(DrivenReason(recorder), "REVERSED");
  EXPECT_EQ(DrivenReason(plain), "");
}

TEST(RecorderDriven, ReversingOnAnArcWithAnOffsetFails) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {2, 0}});
  // Back 1 m bending 0.3 m to the left, then forward along y = 0.3.
  for (int s = 1; s <= 20; s++) {
    const double t = 0.05 * s;
    recorder.OnPose(2 - t, 0.3 * t * t, std::atan2(-0.6 * t, 1), 0.02f, true);
  }
  Drive(recorder, {{1, 0.3}, {3, 0.3}, {3, 2}, {0, 2}, {0, 0.15}, {0.6, -0.05}});
  EXPECT_EQ(DrivenReason(recorder), "REVERSED");
}

TEST(RecorderDriven, LessThanTwoStepsBackwardsChangeNothing) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {2, 0}});
  const size_t before = recorder.current().size();
  // 0.09 m back is at most one judged step.
  for (int k = 1; k <= 9; k++) recorder.OnPose(2 - 0.01 * k, 0, 0, 0.02f, true);
  EXPECT_EQ(recorder.current().size(), before);
  Drive(recorder, {{1.91, 0}, {3, 0}, {3, 2}, {0, 2}, {0, 0.15}, {0.6, -0.05}});
  EXPECT_TRUE(recorder.TakeEvents().empty());
  EXPECT_EQ(DrivenReason(recorder), "");
}

TEST(RecorderDriven, AHalfTurnOnTheSpotTakesNothingBack) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {2, 0}});
  const size_t before = recorder.current().size();
  for (int k = 0; k <= 18; k++) recorder.OnPose(2, 0, M_PI * k / 18, 0.02f, true);
  EXPECT_EQ(recorder.current().size(), before);
  // Forward the other way: on, nothing taken back.
  for (int s = 1; s <= 10; s++) recorder.OnPose(2 - 0.05 * s, 0, M_PI, 0.02f, true);
  EXPECT_GT(recorder.current().size(), before);
  EXPECT_TRUE(recorder.TakeEvents().empty());
}

TEST(RecorderDriven, StandingWithNoiseTakesNothingBack) {
  // The pose of the localization, which averages GNSS while the wheels stand (5 mm).
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {2, 0}});
  const size_t before = recorder.current().size();
  std::mt19937 source(3);
  for (double sigma : {0.005, 0.011}) {
    // 60 s at 10 Hz; 1.1 cm is RTK without the filter, where two steps backwards in a row keep it.
    std::normal_distribution<double> error(0, sigma);
    for (int k = 0; k < 600; k++) recorder.OnPose(2 + error(source), error(source), error(source) / 10, 0.02f, true);
    EXPECT_EQ(recorder.current().size(), before) << sigma;
    EXPECT_TRUE(recorder.TakeEvents().empty()) << sigma;
  }
}

TEST(RecorderDriven, OneHintForEachStretchBackwards) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {3, 0}});
  for (int round = 0; round < 2; round++) {
    for (int s = 1; s <= 10; s++) recorder.OnPose(3 - 0.05 * s, 0, 0, 0.02f, true);
    for (int s = 1; s <= 10; s++) recorder.OnPose(2.5 + 0.05 * s, 0, 0, 0.02f, true);
  }
  int hints = 0;
  for (const auto& event : recorder.TakeEvents()) hints += event.details["reason"] == "REVERSING";
  EXPECT_EQ(hints, 2);
}

TEST(RecorderDriven, ReversingWithoutAutomaticPointsTakesNothingBack) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {2, 0}});
  recorder.OnAction("auto_point_collecting_disable");
  const size_t before = recorder.current().size();
  for (int s = 1; s <= 10; s++) recorder.OnPose(2 - 0.05 * s, 0, 0, 0.02f, true);
  EXPECT_EQ(recorder.current().size(), before);
}

TEST(RecorderDriven, AreaRecordedSaysWhichRingsWereDriven) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, Lap(5, 5));
  recorder.OnAction("stop_recording");
  // One obstacle driven, one with a break of the position.
  recorder.OnAction("start_recording");
  Drive(recorder, Square(1, 1, 1));
  recorder.OnAction("stop_recording");
  recorder.OnAction("start_recording");
  Drive(recorder, {{3, 3}, {4, 3}, {4, 3.5}});
  recorder.OnPoseLost();
  Drive(recorder, {{4, 3.5}, {4, 4}, {3, 4}, {3, 3.15}, {3.6, 2.95}});
  recorder.OnAction("stop_recording");
  ASSERT_EQ(recorder.OnAction("finish_mowing_area"), AreaRecorder::Result::kDone);
  const auto edit = recorder.TakeMapEdit();
  EXPECT_EQ(edit->outline_driven_reason, "");
  EXPECT_EQ(edit->obstacles_driven_reason, (std::vector<std::string>{"", "INTERRUPTED"}));
  recorder.TakeEvents();
  recorder.OnMapEditResult(true, "lawn1", "", "");
  auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["driven"], true);
  EXPECT_FALSE(events[0].details.contains("driven_reason"));
  EXPECT_EQ(events[0].details["obstacles_driven"], 1);
}

TEST(RecorderDriven, AnOutlineByHandIsNotDriven) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  recorder.OnAction("auto_point_collecting_disable");
  for (const Point2& p : Ring{{0, 0}, {2, 0}, {2, 2}, {0, 2}}) {
    recorder.OnAction("collect_point");
    recorder.OnPose(p.x, p.y, 0, 0.02f, true);
  }
  ASSERT_EQ(recorder.OnAction("finish_mowing_area"), AreaRecorder::Result::kDone);
  recorder.TakeMapEdit();
  recorder.TakeEvents();
  recorder.OnMapEditResult(true, "lawn1", "", "");
  const auto events = recorder.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].details["driven"], false);
  EXPECT_EQ(events[0].details["driven_reason"], "MANUAL_POINT");
  EXPECT_EQ(events[0].details["obstacles_driven"], 0);
}

TEST(RecorderDriven, AJumpOfThePositionIsABreak) {
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  Drive(recorder, {{0, 0}, {3, 0}, {3, 1}});
  recorder.OnPositionJump();
  Drive(recorder, {{3, 1}, {3, 2}, {0, 2}, {0, 0.15}, {0.6, -0.05}});
  EXPECT_EQ(DrivenReason(recorder), "INTERRUPTED");
}

namespace {

// Around 3 by 2 m from (0, 0) and down the last side to (0, d); then poses every 0.11 m at angle degrees below the
// first side, so that each makes a point, up to past. stop(k) comes before the pose k, after the corner.
template <typename Stop>
void LapCrossingAt(AreaRecorder& recorder, double degrees, double d, double past, Stop stop) {
  Drive(recorder, {{0, 0}, {3, 0}, {3, 2}, {0, 2}, {0, d}});
  const double a = degrees * M_PI / 180;
  recorder.OnPose(0, d, -a, 0.02f, true);
  const double crossing = d / std::sin(a);
  for (int k = 1; 0.11 * k <= crossing + past; k++) {
    stop(k);
    recorder.OnPose(0.11 * k * std::cos(a), d - 0.11 * k * std::sin(a), -a, 0.02f, true);
  }
}

}  // namespace

TEST(RecorderDriven, ClosingOverACrossingOfUpToThirtyDegrees) {
  // 29.9 for 30: the direction of the edges is computed, the limit not.
  for (const double degrees : {3.0, 15.0, 29.9, 45.0}) {
    AreaRecorder recorder = Writable();
    recorder.OnAction("start_recording");
    LapCrossingAt(recorder, degrees, degrees < 10 ? 0.03 : 0.15, 0.5, [](int) {});
    ASSERT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kDone) << degrees;
    EXPECT_GT(recorder.trimmed_points(), 0u) << degrees;
    EXPECT_EQ(DrivenReason(recorder), degrees < 30 ? "" : "CLOSING") << degrees;
  }
}

TEST(RecorderDriven, AStraightLineOfPointsDoesNotCrossItself) {
  // Points of one straight line, poses every 0.01 m: two pieces 1.2 m apart once seemed to cross by rounding, so
  // that the line from inside at 60 degrees kept the ring open.
  AreaRecorder recorder = Writable();
  recorder.OnAction("start_recording");
  // These exact poses; other steps round differently.
  const Ring way{{5, 0}, {10, 0}, {10, 6}, {0, 6}, {0, 0}, {5, 0.866}, {6, -0.866}};
  for (size_t k = 0; k + 1 < way.size(); k++) {
    const Point2& a = way[k];
    const Point2& b = way[k + 1];
    const double heading = std::atan2(b.y - a.y, b.x - a.x);
    const int steps = static_cast<int>(std::ceil(Distance(a, b) / 0.01 - 1e-9));
    for (int s = 0; s <= steps; s++) {
      recorder.OnPose(a.x + (b.x - a.x) * s / steps, a.y + (b.y - a.y) * s / steps, heading, 0.02f, true);
    }
  }
  ASSERT_EQ(recorder.OnAction("stop_recording"), AreaRecorder::Result::kDone);
  EXPECT_GT(recorder.trimmed_points(), 0u);
  EXPECT_EQ(DrivenReason(recorder), "CLOSING");
}

TEST(RecorderDriven, ABreakOrReversingOnTheEdgeIntoTheCrossingCounts) {
  // At 15 degrees from 0.15 m the track crosses the first side 0.58 m after the corner: the point at 0.55 m is the
  // last one kept, the one at 0.66 m the first cut off.
  AreaRecorder plain = Writable();
  plain.OnAction("start_recording");
  LapCrossingAt(plain, 15, 0.15, 0.5, [](int) {});
  EXPECT_EQ(DrivenReason(plain), "");

  AreaRecorder broken = Writable();
  broken.OnAction("start_recording");
  LapCrossingAt(broken, 15, 0.15, 0.5, [&](int k) {
    if (k != 6) return;
    ASSERT_NEAR(broken.current().back().y, 0.15 - 0.55 * std::sin(15 * M_PI / 180), 1e-9);
    broken.OnPositionJump();
  });
  EXPECT_EQ(DrivenReason(broken), "INTERRUPTED");

  // Back from 0.55 m to 0.43 m takes back the points at 0.44 and 0.55 m; the first point after it comes at 0.66 m.
  AreaRecorder reversed = Writable();
  reversed.OnAction("start_recording");
  const double a = 15 * M_PI / 180;
  LapCrossingAt(reversed, 15, 0.15, 0.5, [&](int k) {
    if (k != 6) return;
    for (const double s : {0.49, 0.43}) reversed.OnPose(s * std::cos(a), 0.15 - s * std::sin(a), -a, 0.02f, true);
  });
  EXPECT_EQ(DrivenReason(reversed), "REVERSED");
}

TEST(LogicRecording, AFailedFollowOfTheBodyIsAnEvent) {
  TestCore core{LogicConfig{}};
  core.TakeEvents();
  core.RecordedBodyKept(2, "STORAGE", "Cannot store the map: disk full");
  core.RecordedBodyKept(-1, "NO_ANSWER", "No answer from mowbite_map");
  const auto events = core.TakeEvents();
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].type, "RECORDED_BODY_KEPT");
  EXPECT_EQ(events[0].details["areas"], 2);
  EXPECT_EQ(events[0].details["reason"], "SAVE_FAILED");
  EXPECT_EQ(events[0].details["code"], "STORAGE");
  EXPECT_EQ(events[0].details["severity"], "warning");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_NE(events[0].details["message"].get<std::string>().find("keep the old body"), std::string::npos);
  EXPECT_FALSE(events[1].details.contains("areas"));
  EXPECT_EQ(events[1].details["code"], "NO_ANSWER");
}
