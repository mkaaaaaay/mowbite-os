#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_logic/mowing_plan.h"

using namespace mowbite_logic;

namespace {

// A garden: a lawn with a tree on it, a path (nav) beside it, a second lawn that is not mowed, a bed that is not active.
const char* kMap = R"({
  "areas": [
    {"id": "lawn", "properties": {"name": "Front", "type": "mow", "angle": 0.5, "outline_count": 2},
     "outline": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 6}, {"x": 0, "y": 6}, {"x": 0, "y": 0}]},
    {"id": "tree", "properties": {"type": "obstacle"},
     "outline": [{"x": 4, "y": 2}, {"x": 5, "y": 2}, {"x": 5, "y": 3}, {"x": 4, "y": 2}]},
    {"id": "old tree", "properties": {"type": "obstacle", "active": false},
     "outline": [{"x": 7, "y": 2}, {"x": 8, "y": 2}, {"x": 8, "y": 3}, {"x": 7, "y": 2}]},
    {"id": "path", "properties": {"type": "nav"},
     "outline": [{"x": 10, "y": 2}, {"x": 14, "y": 2}, {"x": 14, "y": 3}, {"x": 10, "y": 2}]},
    {"id": "back", "properties": {"type": "mow", "mowable": false},
     "outline": [{"x": 14, "y": 0}, {"x": 20, "y": 0}, {"x": 20, "y": 6}, {"x": 14, "y": 0}]},
    {"id": "side", "properties": {"type": "mow", "angle": 1.0, "outline_overlap_count": 1, "outline_offset": 0.05},
     "outline": [{"x": 0, "y": 10}, {"x": 5, "y": 10}, {"x": 5, "y": 15}, {"x": 0, "y": 10}]},
    {"id": "bed", "properties": {"type": "mow", "active": false},
     "outline": [{"x": 30, "y": 0}, {"x": 31, "y": 0}, {"x": 31, "y": 1}, {"x": 30, "y": 0}]},
    {"id": "wish", "properties": {"type": "draft"},
     "outline": [{"x": 40, "y": 0}, {"x": 41, "y": 0}, {"x": 41, "y": 1}, {"x": 40, "y": 0}]}
  ],
  "docking_stations": []
})";

MowingPlanner PlannerWithMap() {
  MowingPlanner planner;
  planner.SetMap(kMap);
  planner.SetMapStatus(true, Reason());
  return planner;
}

json Params(const char* text) {
  return json::parse(text);
}

}  // namespace

TEST(MowingPlanParams, WantAnObject) {
  MowingPlanner planner = PlannerWithMap();
  for (const char* text : {"null", "[]", "\"lawn\"", "3"}) {
    const PlanQuestion q = planner.Prepare(json::parse(text));
    EXPECT_FALSE(q.ok) << text;
    EXPECT_EQ(q.code, -32602) << text;
    EXPECT_EQ(q.message, "parameters must be an object") << text;
  }
}

TEST(MowingPlanParams, RefuseWhatIsNotValidWithTheNameOfTheParameter) {
  MowingPlanner planner = PlannerWithMap();
  const std::vector<std::pair<const char*, const char*>> cases = {
      {R"({"area_id": 3})", "non-string parameter: area_id"},
      {R"({"area_id": "tree"})", "no mowing area with id: tree"},
      {R"({"area_id": "nobody"})", "no mowing area with id: nobody"},
      {R"({"area_id": "back"})", "no mowing area with id: back"},
      {R"({"area_id": "bed"})", "no mowing area with id: bed"},
      {R"({"area_id": "wish"})", "no mowing area with id: wish"},
      {R"({"area_id": "lawn", "outline": 3})", "parameter is not a list of points: outline"},
      {R"({"outline": [{"x": 0}]})", "point without numeric x and y in parameter: outline"},
      {R"({"outline": [{"x": 0, "y": "1"}]})", "point without numeric x and y in parameter: outline"},
      {R"({"outline": [{"x": 0, "y": 1e300}]})", "point out of range in parameter: outline"},
      {R"({"area_id": "lawn", "obstacles": {}})", "parameter is not a list of polygons: obstacles"},
      {R"({"area_id": "lawn", "obstacles": [[{"x": 0}]]})", "point without numeric x and y in parameter: obstacles"},
      {R"({"area_id": "lawn", "angle": "0.5"})", "non-numeric parameter: angle"},
      {R"({"area_id": "lawn", "outline_offset": null})", "non-numeric parameter: outline_offset"},
      {R"({"area_id": "lawn", "outline_count": 1.5})", "parameter is not a whole number from 0 to 255: outline_count"},
      {R"({"area_id": "lawn", "outline_count": -1})", "parameter is not a whole number from 0 to 255: outline_count"},
      {R"({"area_id": "lawn", "outline_overlap_count": 256})",
       "parameter is not a whole number from 0 to 255: outline_overlap_count"},
      {R"({})", "the area needs at least 3 outline points (area_id or outline)"},
      {R"({"outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}]})", "the area needs at least 3 outline points (area_id or outline)"},
  };
  for (const auto& [text, message] : cases) {
    const PlanQuestion q = planner.Prepare(Params(text));
    EXPECT_FALSE(q.ok) << text;
    EXPECT_EQ(q.code, -32602) << text;
    EXPECT_EQ(q.message, message) << text;
  }
}

TEST(MowingPlanMap, TakesTheAreaItsObstaclesAndWhereTheMowerMayDrive) {
  MowingPlanner planner = PlannerWithMap();
  const PlanQuestion q = planner.Prepare(Params(R"({"area_id": "lawn"})"));
  ASSERT_TRUE(q.ok) << q.message;
  EXPECT_EQ(q.area_id, "lawn");
  // The ring is given once, not closed.
  ASSERT_EQ(q.request.boundary.size(), 4u);
  EXPECT_DOUBLE_EQ(q.request.boundary[2].x, 10);
  EXPECT_DOUBLE_EQ(q.request.boundary[2].y, 6);
  // Only the active obstacle; the planner leaves out those outside of the area.
  ASSERT_EQ(q.request.obstacles.size(), 1u);
  EXPECT_DOUBLE_EQ(q.request.obstacles[0][0].x, 4);
  // The nav area and the other mowing areas that are active, even the one that is not mowed; not the area itself.
  ASSERT_EQ(q.request.drivable_areas.size(), 3u);
  EXPECT_DOUBLE_EQ(q.request.drivable_areas[0][0].x, 10);
  EXPECT_DOUBLE_EQ(q.request.drivable_areas[1][0].x, 14);
  EXPECT_DOUBLE_EQ(q.request.drivable_areas[2][0].y, 10);
}

TEST(MowingPlanMap, NeedsAMapThatFitsTheDatum) {
  MowingPlanner planner;
  planner.SetMap(kMap);
  // No status yet
  PlanQuestion q = planner.Prepare(Params(R"({"area_id": "lawn"})"));
  EXPECT_FALSE(q.ok);
  EXPECT_EQ(q.code, -32000);
  EXPECT_EQ(q.message, "Cannot plan from the map: The map status is not known yet");

  planner.SetMapStatus(false, MapReason(MapDatum::kMismatch,
                                        "The map was recorded for the datum 52.0000000, 9.0000000, the datum in use is far away"));
  q = planner.Prepare(Params(R"({"area_id": "lawn"})"));
  EXPECT_EQ(q.code, -32000);
  EXPECT_EQ(q.message,
            "Cannot plan from the map: The map was recorded for the datum 52.0000000, 9.0000000, the datum in use is far away");

  planner.SetMapStatus(false, Reason());
  EXPECT_EQ(planner.Prepare(Params(R"({"area_id": "lawn"})")).message,
            "Cannot plan from the map: The map does not fit the datum in use");

  planner.SetMapStatus(true, Reason());
  EXPECT_TRUE(planner.Prepare(Params(R"({"area_id": "lawn"})")).ok);
}

TEST(MowingPlanMap, AnOutlineInTheRequestNeedsNoMap) {
  MowingPlanner planner;
  const PlanQuestion q = planner.Prepare(Params(R"({
    "outline": [{"x": 0, "y": 0}, {"x": 4, "y": 0}, {"x": 4, "y": 3}, {"x": 0, "y": 3}, {"x": 0, "y": 0}],
    "obstacles": [[{"x": 1, "y": 1}, {"x": 2, "y": 1}, {"x": 2, "y": 2}]]})"));
  ASSERT_TRUE(q.ok) << q.message;
  EXPECT_EQ(q.area_id, "");
  EXPECT_EQ(q.request.boundary.size(), 4u);
  ASSERT_EQ(q.request.obstacles.size(), 1u);
  EXPECT_EQ(q.request.obstacles[0].size(), 3u);
  EXPECT_TRUE(q.request.drivable_areas.empty());
}

TEST(MowingPlanMap, AnOutlineAndObstaclesInTheRequestReplaceThoseOfTheArea) {
  MowingPlanner planner = PlannerWithMap();
  const PlanQuestion q = planner.Prepare(Params(R"({"area_id": "lawn",
    "outline": [{"x": 0, "y": 0}, {"x": 2, "y": 0}, {"x": 2, "y": 2}],
    "obstacles": []})"));
  ASSERT_TRUE(q.ok) << q.message;
  EXPECT_EQ(q.area_id, "lawn");
  EXPECT_EQ(q.request.boundary.size(), 3u);
  EXPECT_TRUE(q.request.obstacles.empty());
  EXPECT_EQ(q.request.drivable_areas.size(), 3u);
}

TEST(MowingPlanMap, ANewMapReplacesTheOldOne) {
  MowingPlanner planner = PlannerWithMap();
  planner.SetMap(R"({"areas": [], "docking_stations": []})");
  EXPECT_EQ(planner.Prepare(Params(R"({"area_id": "lawn"})")).message, "no mowing area with id: lawn");
  planner.SetMap("{not json");
  const PlanQuestion q = planner.Prepare(Params(R"({"area_id": "lawn"})"));
  EXPECT_EQ(q.code, -32000);
  EXPECT_EQ(q.message, "Cannot plan from the map: The map is not readable");
}

TEST(MowingPlanMap, AMapWithTheWrongTypeOfAPropertyIsNotPlannedFrom) {
  MowingPlanner planner;
  planner.SetMapStatus(true, Reason());
  planner.SetMap(R"({"areas": [{"id": "lawn", "properties": {"type": "mow", "active": "yes"},
                                "outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]}]})");
  const PlanQuestion q = planner.Prepare(Params(R"({"area_id": "lawn"})"));
  EXPECT_EQ(q.code, -32000);
  EXPECT_EQ(q.message, "Cannot plan from the map: The map is not readable");
  // An outline in the request still works.
  EXPECT_TRUE(planner.Prepare(Params(R"({"outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]})")).ok);
}

TEST(MowingPlanRequest, CarriesOnlyWhatTheRequestSets) {
  MowingPlanner planner = PlannerWithMap();
  // The areas have an angle, counts and an offset in the map: the planner reads those itself.
  for (const char* area : {"lawn", "side"}) {
    const PlanQuestion q = planner.Prepare(json{{"area_id", area}});
    ASSERT_TRUE(q.ok) << q.message;
    EXPECT_EQ(q.request.perimeter_passes, -1) << area;
    EXPECT_FALSE(q.request.lane_overlap_passes.has_value()) << area;
    EXPECT_FALSE(q.request.perimeter_offset.has_value()) << area;
    EXPECT_FALSE(q.request.use_angle) << area;
  }
  const PlanQuestion outline = planner.Prepare(Params(R"({"outline": [{"x": 0, "y": 0}, {"x": 2, "y": 0}, {"x": 2, "y": 2}]})"));
  ASSERT_TRUE(outline.ok);
  EXPECT_EQ(outline.request.perimeter_passes, -1);
  EXPECT_FALSE(outline.request.lane_overlap_passes.has_value());
  EXPECT_FALSE(outline.request.perimeter_offset.has_value());
  EXPECT_FALSE(outline.request.use_angle);
}

TEST(MowingPlanRequest, PassesOnCountsOffsetAndAngleOfTheRequest) {
  MowingPlanner planner = PlannerWithMap();
  PlanQuestion q = planner.Prepare(
      Params(R"({"area_id": "side", "outline_count": 0, "outline_overlap_count": 0, "outline_offset": 0.2, "angle": -1.25})"));
  ASSERT_TRUE(q.ok) << q.message;
  EXPECT_EQ(q.request.perimeter_passes, 0);
  ASSERT_TRUE(q.request.lane_overlap_passes.has_value());
  EXPECT_EQ(*q.request.lane_overlap_passes, 0);
  ASSERT_TRUE(q.request.perimeter_offset.has_value());
  EXPECT_DOUBLE_EQ(*q.request.perimeter_offset, 0.2);
  EXPECT_TRUE(q.request.use_angle);
  EXPECT_DOUBLE_EQ(q.request.angle, -1.25);

  // each one alone
  q = planner.Prepare(Params(R"({"area_id": "lawn", "outline_count": 5})"));
  EXPECT_EQ(q.request.perimeter_passes, 5);
  EXPECT_FALSE(q.request.lane_overlap_passes.has_value());
  EXPECT_FALSE(q.request.perimeter_offset.has_value());
  EXPECT_FALSE(q.request.use_angle);
  q = planner.Prepare(Params(R"({"area_id": "lawn", "outline_overlap_count": 2})"));
  EXPECT_EQ(q.request.perimeter_passes, -1);
  EXPECT_EQ(*q.request.lane_overlap_passes, 2);
}

namespace {

// A route over `n` poses on a line, as the planner would number it.
CoverageAnswer Route(size_t n) {
  CoverageAnswer answer;
  answer.success = true;
  for (size_t i = 0; i < n; ++i) answer.poses.push_back({0.05 * static_cast<double>(i), 0.0});
  return answer;
}

void AddSegment(CoverageAnswer& answer, uint8_t type, uint32_t first, uint32_t last) {
  answer.segment_type.push_back(type);
  answer.segment_start.push_back(first);
  answer.segment_end.push_back(last);
}

}  // namespace

TEST(MowingPlanPieces, SplitLoopsFromLanesAndLetATransitGoWithWhatFollowsIt) {
  CoverageAnswer answer = Route(100);
  answer.section_start = {0};
  AddSegment(answer, kSegmentPerimeter, 0, 20);
  AddSegment(answer, kSegmentPerimeter, 20, 30);
  AddSegment(answer, kSegmentTransit, 30, 40);
  AddSegment(answer, kSegmentLane, 40, 60);
  AddSegment(answer, kSegmentTurn, 60, 65);
  AddSegment(answer, kSegmentLane, 65, 90);
  const auto pieces = CutIntoPieces(answer);
  ASSERT_EQ(pieces.size(), 2u);
  EXPECT_TRUE(pieces[0].is_outline);
  EXPECT_EQ(pieces[0].first, 0u);
  EXPECT_EQ(pieces[0].last, 30u);
  // the transit leads to the lanes, so it is the start of their path
  EXPECT_FALSE(pieces[1].is_outline);
  EXPECT_EQ(pieces[1].first, 30u);
  EXPECT_EQ(pieces[1].last, 90u);
}

TEST(MowingPlanPieces, ATransitBeforeLoopsBelongsToThem) {
  CoverageAnswer answer = Route(80);
  answer.section_start = {0};
  AddSegment(answer, kSegmentLane, 0, 20);
  AddSegment(answer, kSegmentTransit, 20, 30);
  AddSegment(answer, kSegmentPerimeter, 30, 70);
  const auto pieces = CutIntoPieces(answer);
  ASSERT_EQ(pieces.size(), 2u);
  EXPECT_FALSE(pieces[0].is_outline);
  EXPECT_EQ(pieces[0].last, 20u);
  EXPECT_TRUE(pieces[1].is_outline);
  EXPECT_EQ(pieces[1].first, 20u);
  EXPECT_EQ(pieces[1].last, 70u);
}

TEST(MowingPlanPieces, EverySectionStartsANewPath) {
  CoverageAnswer answer = Route(100);
  answer.section_start = {0, 50};
  AddSegment(answer, kSegmentLane, 0, 30);
  AddSegment(answer, kSegmentLane, 30, 49);
  AddSegment(answer, kSegmentLane, 50, 80);
  const auto pieces = CutIntoPieces(answer);
  ASSERT_EQ(pieces.size(), 2u);
  EXPECT_EQ(pieces[0].first, 0u);
  EXPECT_EQ(pieces[0].last, 49u);
  EXPECT_EQ(pieces[1].first, 50u);
  EXPECT_EQ(pieces[1].last, 80u);
}

TEST(MowingPlanPieces, LeaveOutRunsOfFewerThanThreePoses) {
  CoverageAnswer answer = Route(40);
  answer.section_start = {0, 10};
  AddSegment(answer, kSegmentLane, 0, 1);
  AddSegment(answer, kSegmentLane, 10, 30);
  const auto pieces = CutIntoPieces(answer);
  ASSERT_EQ(pieces.size(), 1u);
  EXPECT_EQ(pieces[0].first, 10u);
}

TEST(MowingPlanPieces, CutLongRunsIntoEqualPartsThatShareThePoseWhereOneEndsAndTheNextStarts) {
  CoverageAnswer answer = Route(30);
  answer.section_start = {0};
  AddSegment(answer, kSegmentLane, 0, 21);
  // at most 10 poses a path: 21 steps in parts of at most 9
  const auto pieces = CutIntoPieces(answer, 10);
  ASSERT_EQ(pieces.size(), 3u);
  EXPECT_EQ(pieces[0].first, 0u);
  EXPECT_EQ(pieces[0].last, pieces[1].first);
  EXPECT_EQ(pieces[1].last, pieces[2].first);
  EXPECT_EQ(pieces[2].last, 21u);
  for (const auto& piece : pieces) EXPECT_LE(piece.last - piece.first + 1, 10u);
}

TEST(MowingPlanPieces, AnEmptyRouteHasNoPieces) {
  CoverageAnswer answer;
  EXPECT_TRUE(CutIntoPieces(answer).empty());
}

TEST(MowingPlanSimplify, KeepsTheEndsAndTheCornersOfALine) {
  std::vector<Point2> points;
  for (int i = 0; i <= 10; ++i) points.push_back({0.1 * i, 0.0});
  for (int i = 1; i <= 10; ++i) points.push_back({1.0, 0.1 * i});
  const std::vector<size_t> expected = {0, 10, 20};
  EXPECT_EQ(SimplifiedIndices(points, 0.01), expected);
}

TEST(MowingPlanSimplify, DropsADeviationBelowTheToleranceAndKeepsOneAbove) {
  const std::vector<Point2> small = {{0, 0}, {0.5, 0.005}, {1, 0}};
  EXPECT_EQ(SimplifiedIndices(small, 0.01), (std::vector<size_t>{0, 2}));
  const std::vector<Point2> large = {{0, 0}, {0.5, 0.05}, {1, 0}};
  EXPECT_EQ(SimplifiedIndices(large, 0.01), (std::vector<size_t>{0, 1, 2}));
}

TEST(MowingPlanSimplify, KeepsAllOfAShortList) {
  EXPECT_TRUE(SimplifiedIndices({}, 0.01).empty());
  EXPECT_EQ(SimplifiedIndices({{0, 0}}, 0.01), (std::vector<size_t>{0}));
  EXPECT_EQ(SimplifiedIndices({{0, 0}, {0, 0}}, 0.01), (std::vector<size_t>{0, 1}));
}

TEST(MowingPlanSimplify, ALoopBackToTheStartIsKept) {
  const std::vector<Point2> square = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}};
  EXPECT_EQ(SimplifiedIndices(square, 0.01).size(), 5u);
}

TEST(MowingPlanResult, ListsThePathsLikeOpenMowerDoes) {
  MowingPlanner planner = PlannerWithMap();
  const PlanQuestion q = planner.Prepare(Params(R"({"area_id": "lawn"})"));
  CoverageAnswer answer;
  answer.success = true;
  answer.angle = 0.5;
  answer.fingerprint = "abc";
  answer.section_start = {0};
  // a loop of 4 corners with the poses between them, then 3 lanes
  for (int i = 0; i <= 10; ++i) answer.poses.push_back({0.1 * i, 0.0});
  for (int i = 1; i <= 10; ++i) answer.poses.push_back({1.0, 0.1 * i});
  for (int i = 0; i <= 4; ++i) answer.poses.push_back({1.0 - 0.25 * i, 1.0004});
  AddSegment(answer, kSegmentPerimeter, 0, 20);
  AddSegment(answer, kSegmentLane, 20, 25);

  const json result = planner.Result(q, answer);
  EXPECT_EQ(result["area_id"], "lawn");
  EXPECT_DOUBLE_EQ(result["angle"].get<double>(), 0.5);
  EXPECT_EQ(result["fingerprint"], "abc");
  ASSERT_EQ(result["paths"].size(), 2u);
  const json& loop = result["paths"][0];
  EXPECT_EQ(loop["is_outline"], true);
  EXPECT_EQ(loop["points"], json::parse("[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]"));
  EXPECT_EQ(loop["pose_index"], json::parse("[0, 10, 20]"));
  const json& lane = result["paths"][1];
  EXPECT_EQ(lane["is_outline"], false);
  // rounded to mm: 1.0004 becomes 1.0
  EXPECT_EQ(lane["points"], json::parse("[[1.0, 1.0], [0.0, 1.0]]"));
  EXPECT_EQ(lane["pose_index"], json::parse("[0, 5]"));
}

TEST(MowingPlanResult, AnOutlineGivenInTheRequestHasNoAreaId) {
  MowingPlanner planner;
  const PlanQuestion q = planner.Prepare(Params(R"({"outline": [{"x": 0, "y": 0}, {"x": 2, "y": 0}, {"x": 2, "y": 2}]})"));
  CoverageAnswer answer = Route(5);
  answer.section_start = {0};
  AddSegment(answer, kSegmentLane, 0, 4);
  const json result = planner.Result(q, answer);
  EXPECT_EQ(result["area_id"], "");
  ASSERT_EQ(result["paths"].size(), 1u);
  EXPECT_EQ(result["paths"][0]["points"].size(), 2u);
}

TEST(MowingPlanResult, SegmentsBeyondThePosesAreLeftOut) {
  MowingPlanner planner;
  const PlanQuestion q = planner.Prepare(Params(R"({"outline": [{"x": 0, "y": 0}, {"x": 2, "y": 0}, {"x": 2, "y": 2}]})"));
  CoverageAnswer answer = Route(5);
  answer.section_start = {0};
  AddSegment(answer, kSegmentLane, 0, 9);
  EXPECT_TRUE(planner.Result(q, answer)["paths"].empty());
}

TEST(MowingPlanTransit, DrivesOnTheActiveMowAndNavAreasAroundTheActiveObstacles) {
  MowingPlanner planner = PlannerWithMap();
  std::vector<Ring> drivable;
  std::vector<Ring> obstacles;
  ASSERT_TRUE(planner.TransitAreas(&drivable, &obstacles).empty());
  // lawn, path, back (not mowed, still drivable), side; not the inactive bed nor the draft
  ASSERT_EQ(drivable.size(), 4u);
  EXPECT_EQ(drivable[1].front().x, 10.0);
  EXPECT_EQ(drivable[2].front().x, 14.0);
  ASSERT_EQ(obstacles.size(), 1u);
  EXPECT_EQ(obstacles[0].front().x, 4.0);
  // Open rings like everywhere in the logic.
  EXPECT_EQ(drivable[0].size(), 4u);
}

TEST(MowingPlanTransit, NeedsAMapThatFitsAndAnArea) {
  MowingPlanner planner;
  std::vector<Ring> drivable;
  std::vector<Ring> obstacles;
  Reason why = planner.TransitAreas(&drivable, &obstacles);
  EXPECT_EQ(why.code, "MAP_UNKNOWN");
  EXPECT_EQ(why.message, "Cannot drive on the map: The map status is not known yet");
  planner.SetMapStatus(true, Reason());
  EXPECT_EQ(planner.TransitAreas(&drivable, &obstacles).code, "MAP_UNREADABLE");
  planner.SetMap(R"({"areas": [{"id": "t", "properties": {"type": "obstacle"},
                    "outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]}]})");
  EXPECT_EQ(planner.TransitAreas(&drivable, &obstacles).code, "NO_MOWING_AREA");
  planner.SetMap(kMap);
  planner.SetMapStatus(false, MapReason(MapDatum::kMismatch, "The map was recorded for another datum"));
  why = planner.TransitAreas(&drivable, &obstacles);
  EXPECT_EQ(why.code, "MAP_MISMATCH");
  EXPECT_EQ(why.message, "Cannot drive on the map: The map was recorded for another datum");
  EXPECT_TRUE(drivable.empty());
}
