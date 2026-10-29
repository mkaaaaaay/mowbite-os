#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_logic/mowing_job.h"

using namespace mowbite_logic;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Poses along the x axis, 0.1 m apart, in one section.
CoverageAnswer Line(size_t n) {
  CoverageAnswer answer;
  for (size_t i = 0; i < n; ++i) answer.poses.push_back({0.1 * static_cast<double>(i), 0});
  answer.section_start = {0};
  return answer;
}

void Segment(CoverageAnswer& answer, uint8_t type, uint32_t first, uint32_t last) {
  answer.segment_start.push_back(first);
  answer.segment_end.push_back(last);
  answer.segment_type.push_back(type);
}

std::vector<JobArea> Garden() {
  return {{"front", "Front", true}, {"bed", "Bed", false}, {"back", "Back", true}};
}

// Two paths of 5 poses each.
std::vector<JobPath> TwoPaths() {
  std::vector<JobPath> paths(2);
  for (auto& path : paths) path.poses.resize(5);
  return paths;
}

}  // namespace

TEST(MowingJobPaths, CutLikeMowingPlanWithTheKindOfTheStretchAfterEachPose) {
  CoverageAnswer answer = Line(20);
  Segment(answer, kSegmentPerimeter, 0, 6);
  Segment(answer, kSegmentTransit, 6, 9);
  Segment(answer, kSegmentLane, 9, 15);
  Segment(answer, kSegmentTurn, 15, 17);
  Segment(answer, kSegmentLane, 17, 19);
  const auto paths = JobPaths(answer);
  const auto pieces = CutIntoPieces(answer);
  ASSERT_EQ(paths.size(), pieces.size());
  ASSERT_EQ(paths.size(), 2u);
  EXPECT_TRUE(paths[0].is_outline);
  EXPECT_FALSE(paths[1].is_outline);
  // The outline: 0..6; the rest starts with the transit that leads to the lanes.
  ASSERT_EQ(paths[0].poses.size(), 7u);
  ASSERT_EQ(paths[1].poses.size(), 14u);
  EXPECT_EQ(paths[0].poses[0].kind, kSegmentPerimeter);
  EXPECT_EQ(paths[0].poses[6].kind, kSegmentPerimeter);
  EXPECT_EQ(paths[1].poses[0].kind, kSegmentTransit);
  EXPECT_EQ(paths[1].poses[2].kind, kSegmentTransit);
  EXPECT_EQ(paths[1].poses[3].kind, kSegmentLane);
  EXPECT_EQ(paths[1].poses[9].kind, kSegmentTurn);
  EXPECT_EQ(paths[1].poses[11].kind, kSegmentLane);
  EXPECT_EQ(paths[1].poses[13].kind, kSegmentLane);
  EXPECT_DOUBLE_EQ(paths[1].poses[13].x, 1.9);
}

TEST(MowingJobPaths, HeadingsFromTheAnswerOrFacingTheNextPose) {
  CoverageAnswer answer;
  answer.poses = {{0, 0}, {0, 1}, {0, 2}, {1, 2}};
  answer.section_start = {0};
  Segment(answer, kSegmentLane, 0, 3);
  auto paths = JobPaths(answer);
  ASSERT_EQ(paths.size(), 1u);
  EXPECT_NEAR(paths[0].poses[0].heading, kPi / 2, 1e-9);
  EXPECT_NEAR(paths[0].poses[2].heading, 0, 1e-9);
  EXPECT_NEAR(paths[0].poses[3].heading, 0, 1e-9);
  answer.headings = {0.1, 0.2, 0.3, 0.4};
  paths = JobPaths(answer);
  EXPECT_DOUBLE_EQ(paths[0].poses[3].heading, 0.4);
}

TEST(MowingJobPaths, OnlyPerimeterLanesAndTurnsAreMowed) {
  EXPECT_TRUE(IsMowKind(kSegmentPerimeter));
  EXPECT_TRUE(IsMowKind(kSegmentLane));
  EXPECT_TRUE(IsMowKind(kSegmentTurn));
  EXPECT_FALSE(IsMowKind(kSegmentTransit));
  EXPECT_FALSE(IsMowKind(7));
}

TEST(MowingJobCheckpoint, RoundTrip) {
  JobProgress p;
  p.job_id = "abc";
  p.area = 2;
  p.area_id = "back";
  p.path = 3;
  p.index = 41;
  p.fingerprint = "f00";
  p.angle_increment_sum = 30;
  const auto back = ParseCheckpoint(CheckpointText(p));
  ASSERT_TRUE(back);
  EXPECT_EQ(back->job_id, "abc");
  EXPECT_EQ(back->area, 2);
  EXPECT_EQ(back->area_id, "back");
  EXPECT_EQ(back->path, 3);
  EXPECT_EQ(back->index, 41);
  EXPECT_EQ(back->fingerprint, "f00");
  EXPECT_DOUBLE_EQ(back->angle_increment_sum, 30);
}

TEST(MowingJobCheckpoint, RefusesWhatIsNotOne) {
  for (const char* text : {"", "{", "[]", "{\"version\": 2}", R"({"version": 1, "area": 0})",
                           R"({"version":1,"job_id":"","area":-1,"area_id":"","path":0,"index":0,"fingerprint":"","angle_increment_sum":0})",
                           R"({"version":1,"job_id":3,"area":0,"area_id":"","path":0,"index":0,"fingerprint":"","angle_increment_sum":0})"}) {
    EXPECT_FALSE(ParseCheckpoint(text)) << text;
  }
}

TEST(MowingJob, PassesOverAreasThatAreNotMowedAndEndsAfterTheLast) {
  MowingJob job;
  job.SetAreas(Garden());
  EXPECT_TRUE(job.HasMowableArea());
  EXPECT_FALSE(job.Unfinished());
  ASSERT_TRUE(job.SeekArea(nullptr));
  EXPECT_EQ(job.CurrentArea()->id, "front");
  job.SkipArea();
  ASSERT_TRUE(job.SeekArea(nullptr));
  EXPECT_EQ(job.progress().area, 2);
  EXPECT_EQ(job.progress().area_id, "back");
  EXPECT_TRUE(job.Unfinished());
  job.SkipArea();
  EXPECT_FALSE(job.SeekArea(nullptr));
  job.SetAreas({{"bed", "", false}});
  EXPECT_FALSE(job.HasMowableArea());
}

TEST(MowingJob, WalksThroughThePathsOfAPlan) {
  MowingJob job;
  job.SetAreas(Garden());
  ASSERT_TRUE(job.SeekArea(nullptr));
  EXPECT_FALSE(job.CurrentPath());
  job.SetPlan(TwoPaths(), "fp");
  ASSERT_TRUE(job.CurrentPath());
  job.OnIndex(3);
  EXPECT_EQ(job.progress().index, 3);
  // Never past the last pose.
  job.OnIndex(99);
  EXPECT_EQ(job.progress().index, 4);
  job.PathDone();
  EXPECT_EQ(job.progress().path, 1);
  EXPECT_EQ(job.progress().index, 0);
  job.PathDone();
  // The area is done: on to the next mowable one.
  EXPECT_FALSE(job.HasPlan());
  ASSERT_TRUE(job.SeekArea(nullptr));
  EXPECT_EQ(job.CurrentArea()->id, "back");
}

TEST(MowingJob, AnEmptyPlanFinishesTheArea) {
  MowingJob job;
  job.SetAreas(Garden());
  ASSERT_TRUE(job.SeekArea(nullptr));
  job.SetPlan({}, "fp");
  ASSERT_TRUE(job.SeekArea(nullptr));
  EXPECT_EQ(job.CurrentArea()->id, "back");
}

TEST(MowingJob, GoesOnFromTheCheckpointOnlyWithTheSamePlan) {
  JobProgress saved;
  saved.job_id = "j";
  saved.area = 2;
  saved.area_id = "back";
  saved.path = 1;
  saved.index = 3;
  saved.fingerprint = "fp";
  MowingJob job;
  job.SetAreas(Garden());
  job.Restore(saved);
  EXPECT_TRUE(job.Unfinished());
  ASSERT_TRUE(job.SeekArea(nullptr));
  job.SetPlan(TwoPaths(), "fp");
  EXPECT_EQ(job.progress().path, 1);
  EXPECT_EQ(job.progress().index, 3);
  // Another plan: the area starts over.
  job.Restore(saved);
  ASSERT_TRUE(job.SeekArea(nullptr));
  job.SetPlan(TwoPaths(), "other");
  EXPECT_EQ(job.progress().path, 0);
  EXPECT_EQ(job.progress().index, 0);
  EXPECT_EQ(job.progress().fingerprint, "other");
  // A path the plan does not have.
  saved.path = 5;
  job.Restore(saved);
  ASSERT_TRUE(job.SeekArea(nullptr));
  job.SetPlan(TwoPaths(), "fp");
  EXPECT_EQ(job.progress().path, 0);
}

TEST(MowingJob, StartsOverWhenTheAreaOfTheCheckpointMoved) {
  JobProgress saved;
  saved.area = 1;
  saved.area_id = "front";
  saved.path = 1;
  MowingJob job;
  job.SetAreas(Garden());
  job.Restore(saved);
  std::string notice;
  ASSERT_TRUE(job.SeekArea(&notice));
  EXPECT_EQ(job.CurrentArea()->id, "front");
  EXPECT_EQ(job.progress().path, 0);
  EXPECT_NE(notice.find("starting over"), std::string::npos);
}

TEST(MowingJob, SkipPathAndCompleteAndReset) {
  MowingJob job;
  job.SetAreas(Garden());
  job.SetJobId("j");
  ASSERT_TRUE(job.SeekArea(nullptr));
  job.SetPlan(TwoPaths(), "fp");
  job.OnIndex(2);
  job.SkipPath();
  EXPECT_EQ(job.progress().path, 1);
  EXPECT_EQ(job.progress().index, 0);
  job.Complete(100);
  job.Complete(300);
  EXPECT_DOUBLE_EQ(job.progress().angle_increment_sum, 40);
  EXPECT_EQ(job.progress().job_id, "");
  EXPECT_FALSE(job.Unfinished());
  job.SetJobId("k");
  ASSERT_TRUE(job.SeekArea(nullptr));
  job.SetPlan(TwoPaths(), "fp");
  job.Reset();
  EXPECT_FALSE(job.Unfinished());
  EXPECT_EQ(job.progress().job_id, "");
  EXPECT_DOUBLE_EQ(job.progress().angle_increment_sum, 40);
}

TEST(MowingJob, MowAreasOfTheMapInMapOrder) {
  MowingPlanner planner;
  EXPECT_TRUE(planner.MowAreas().empty());
  planner.SetMap(R"({"areas": [
    {"id": "a", "properties": {"type": "mow", "name": "A"}, "outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]},
    {"id": "n", "properties": {"type": "nav"}, "outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]},
    {"id": "b", "properties": {"type": "mow", "active": false}, "outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]},
    {"id": "c", "properties": {"type": "mow", "mowable": false}, "outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]},
    {"id": "d", "properties": {"type": "mow"}, "outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]}
  ]})");
  const auto areas = planner.MowAreas();
  ASSERT_EQ(areas.size(), 4u);
  EXPECT_EQ(areas[0].id, "a");
  EXPECT_EQ(areas[0].name, "A");
  EXPECT_TRUE(areas[0].mowable);
  EXPECT_FALSE(areas[1].mowable);
  EXPECT_FALSE(areas[2].mowable);
  EXPECT_EQ(areas[3].id, "d");
  EXPECT_TRUE(areas[3].mowable);
  EXPECT_FALSE(planner.MapProblem().empty());
  planner.SetMapStatus(true, Reason());
  EXPECT_TRUE(planner.MapProblem().empty());
}
