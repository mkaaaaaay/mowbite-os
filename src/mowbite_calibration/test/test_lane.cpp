#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "mowbite_calibration/lane.h"

using mowbite_calibration::BodyRadius;
using mowbite_calibration::ChooseLane;
using mowbite_calibration::Lane;
using mowbite_calibration::LaneConfig;
using mowbite_calibration::LaneFits;
using mowbite_safety::ParseRegion;
using mowbite_safety::Region;

namespace {

// The Tron: 0.60 m ahead, 0.12 m behind base_link, 0.45 m wide.
constexpr double kTron = 0.6408;
constexpr double kDeg = M_PI / 180.0;

std::string Box(double x0, double y0, double x1, double y1, const std::string& type) {
  const auto n = [](double v) { return std::to_string(v); };
  return R"({"outline": [{"x": )" + n(x0) + R"(, "y": )" + n(y0) + R"(}, {"x": )" + n(x1) + R"(, "y": )" + n(y0) +
         R"(}, {"x": )" + n(x1) + R"(, "y": )" + n(y1) + R"(}, {"x": )" + n(x0) + R"(, "y": )" + n(y1) +
         R"(}], "properties": {"type": ")" + type + R"("}})";
}

Region Map(const std::string& areas, const std::string& stations = "[]") {
  std::string error;
  const auto region = ParseRegion(R"({"areas": [)" + areas + R"(], "docking_stations": )" + stations + "}", &error);
  EXPECT_TRUE(region.has_value()) << error;
  return region.value_or(Region());
}

// How far apart two headings are, 0 to pi.
double AngleDiff(double a, double b) {
  const double apart = std::fmod(std::abs(a - b), 2.0 * M_PI);
  return apart > M_PI ? 2.0 * M_PI - apart : apart;
}

// Distance of a point from the segment of the lane.
double FromLane(const Lane& lane, double px, double py) {
  const double ux = std::cos(lane.heading), uy = std::sin(lane.heading);
  const double t = std::clamp((px - lane.x) * ux + (py - lane.y) * uy, 0.0, lane.length);
  return std::hypot(lane.x + t * ux - px, lane.y + t * uy - py);
}

}  // namespace

TEST(Lane, BodyRadiusFromTheRectangleOrTheOutline) {
  EXPECT_NEAR(BodyRadius(0.60, 0.12, 0.45, {}), std::hypot(0.60, 0.225), 1e-12);
  // A body reaching further behind than ahead.
  EXPECT_NEAR(BodyRadius(0.2, 0.5, 0.4, {}), std::hypot(0.5, 0.2), 1e-12);
  EXPECT_NEAR(BodyRadius(0.60, 0.12, 0.45, {{0.7, 0.1}, {0.7, -0.1}, {-0.1, -0.3}, {-0.1, 0.3}}), std::hypot(0.7, 0.1),
              1e-12);
}

TEST(Lane, FullLaneStraightAheadOnAnOpenLawn) {
  const Region region = Map(Box(0, 0, 10, 6, "mow"));
  const auto choice = ChooseLane(region, 2, 3, 0.0, kTron, LaneConfig{});
  ASSERT_TRUE(choice.lane);
  EXPECT_DOUBLE_EQ(choice.lane->length, 3.0);
  EXPECT_NEAR(choice.lane->heading, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(choice.lane->x, 2);
  EXPECT_DOUBLE_EQ(choice.free_length, 3.0);
  // The strip of the Tron: 4.9 x 1.9 m.
  EXPECT_NEAR(choice.strip_length, 3.0 + 2 * (kTron + 0.3), 1e-9);
  EXPECT_NEAR(choice.strip_width, 2 * (kTron + 0.3), 1e-9);
  EXPECT_TRUE(LaneFits(region, *choice.lane, kTron, LaneConfig{}));
}

TEST(Lane, NearestDirectionThatFitsLeftFirst) {
  // A corridor 2.6 m wide: there is room only along it, at most 5 deg off; the mower faces its wall.
  const Region region = Map(Box(0, 0, 20, 2.6, "mow"));
  const auto choice = ChooseLane(region, 10, 1.3, M_PI / 2, kTron, LaneConfig{});
  ASSERT_TRUE(choice.lane);
  EXPECT_DOUBLE_EQ(choice.lane->length, 3.0);
  // 90 deg left and right are equally near; left comes first.
  EXPECT_NEAR(AngleDiff(choice.lane->heading, M_PI), 0.0, 1e-9);
  // In steps of 10 deg from the heading: 80 deg to the right is nearer than 90 deg to the left.
  const auto right = ChooseLane(region, 10, 1.3, M_PI / 2 - 5 * kDeg, kTron, LaneConfig{});
  ASSERT_TRUE(right.lane);
  EXPECT_NEAR(AngleDiff(right.lane->heading, 5 * kDeg), 0.0, 1e-9);
  EXPECT_FALSE(LaneFits(region, Lane{10, 1.3, M_PI / 2, 3.0}, kTron, LaneConfig{}));
  EXPECT_FALSE(LaneFits(region, Lane{10, 1.3, 15 * kDeg, 3.0}, kTron, LaneConfig{}));
}

TEST(Lane, ShorterWhereThereIsLessRoom) {
  // 6 x 3 m from its middle: 2.06 m fit along it, no more.
  const Region region = Map(Box(0, 0, 6, 3, "mow"));
  const auto choice = ChooseLane(region, 3, 1.5, 0.0, kTron, LaneConfig{});
  ASSERT_TRUE(choice.lane);
  EXPECT_GE(choice.lane->length, 2.0);
  EXPECT_LE(choice.lane->length, 6.0 - 3.0 - (kTron + 0.3) + 1e-9);
  EXPECT_GT(choice.lane->length, 1.95);
  EXPECT_NEAR(choice.strip_length, choice.lane->length + 2 * (kTron + 0.3), 1e-9);
  EXPECT_DOUBLE_EQ(choice.free_length, choice.lane->length);
  EXPECT_TRUE(LaneFits(region, *choice.lane, kTron, LaneConfig{}));
}

TEST(Lane, NoRoomBelowTheShortestLane) {
  const Region region = Map(Box(0, 0, 4, 3, "mow"));
  const auto choice = ChooseLane(region, 2, 1.5, 0.0, kTron, LaneConfig{});
  EXPECT_FALSE(choice.lane);
  EXPECT_GT(choice.free_length, 0.9);
  EXPECT_LT(choice.free_length, 2.0);
  // What it would need: the strip of the shortest lane.
  EXPECT_NEAR(choice.strip_length, 2.0 + 2 * (kTron + 0.3), 1e-9);
  // The NX60i, smaller, fits here.
  const auto small = ChooseLane(region, 1.0, 1.5, 0.0, 0.25, LaneConfig{});
  ASSERT_TRUE(small.lane);
}

TEST(Lane, ObstaclesAndThinPolesBlock) {
  const std::string lawn = Box(0, 0, 10, 6, "mow");
  // A tree straight ahead: the lane turns away from it.
  const Region tree = Map(lawn + "," + Box(3.9, 2.9, 4.1, 3.1, "obstacle"));
  const auto choice = ChooseLane(tree, 2, 3, 0.0, kTron, LaneConfig{});
  ASSERT_TRUE(choice.lane);
  EXPECT_GT(AngleDiff(choice.lane->heading, 0.0), 1e-6);
  EXPECT_GT(FromLane(*choice.lane, 4, 3), kTron + 0.3 + 0.1);
  // A pole of 2 cm between the points of the grid still blocks the lane over it.
  const Region pole = Map(lawn + "," + Box(4.512, 3.212, 4.532, 3.232, "obstacle"));
  EXPECT_FALSE(LaneFits(pole, Lane{2, 3, 0.0, 3.0}, kTron, LaneConfig{}));
  EXPECT_TRUE(LaneFits(Map(lawn), Lane{2, 3, 0.0, 3.0}, kTron, LaneConfig{}));
  // An inactive obstacle does not.
  const Region off = Map(lawn + R"(, {"outline": [{"x": 3.9, "y": 2.9}, {"x": 4.1, "y": 2.9}, {"x": 4.1, "y": 3.1}],
                                      "properties": {"type": "obstacle", "active": false}})");
  EXPECT_TRUE(LaneFits(off, Lane{2, 3, 0.0, 3.0}, kTron, LaneConfig{}));
}

TEST(Lane, AcrossAdjacentAreasButNotOutOfThem) {
  // A mowing area and a navigation area side by side: the strip may cross from one into the other.
  const Region region = Map(Box(0, 0, 5, 6, "mow") + "," + Box(5, 0, 12, 6, "nav"));
  EXPECT_TRUE(LaneFits(region, Lane{3, 3, 0.0, 3.0}, kTron, LaneConfig{}));
  // A gap of 0.3 m between them does not count as room.
  const Region gap = Map(Box(0, 0, 5, 6, "mow") + "," + Box(5.3, 0, 12, 6, "nav"));
  EXPECT_FALSE(LaneFits(gap, Lane{3, 3, 0.0, 3.0}, kTron, LaneConfig{}));
  // Decor and draft areas are no room.
  const Region decor = Map(Box(0, 0, 5, 6, "mow") + "," + Box(5, 0, 12, 6, "decor"));
  EXPECT_FALSE(LaneFits(decor, Lane{3, 3, 0.0, 3.0}, kTron, LaneConfig{}));
}

TEST(Lane, KeepsAwayFromTheStation) {
  const std::string lawn = Box(0, 0, 10, 6, "mow");
  // The station 0.36 m beyond the end of the strip straight ahead: the next direction with the full lane.
  const Region ahead = Map(lawn, R"([{"position": {"x": 6.3, "y": 3}, "heading": 0}])");
  EXPECT_FALSE(LaneFits(ahead, Lane{2, 3, 0.0, 3.0}, kTron, LaneConfig{}));
  const auto choice = ChooseLane(ahead, 2, 3, 0.0, kTron, LaneConfig{});
  ASSERT_TRUE(choice.lane);
  EXPECT_DOUBLE_EQ(choice.lane->length, 3.0);
  EXPECT_NEAR(choice.lane->heading, 20 * kDeg, 1e-9);
  EXPECT_GE(FromLane(*choice.lane, 6.3, 3) - (kTron + 0.3), 0.5 - 1e-9);
  // Right beside the mower: nowhere.
  const Region beside = Map(lawn, R"([{"position": {"x": 1.5, "y": 3}, "heading": 0}])");
  const auto none = ChooseLane(beside, 2, 3, 0.0, kTron, LaneConfig{});
  EXPECT_FALSE(none.lane);
  EXPECT_DOUBLE_EQ(none.free_length, 0.0);
  // An inactive station does not count.
  const Region off = Map(lawn, R"([{"position": {"x": 6.3, "y": 3}, "heading": 0, "properties": {"active": false}}])");
  EXPECT_TRUE(LaneFits(off, Lane{2, 3, 0.0, 3.0}, kTron, LaneConfig{}));
}

TEST(Lane, NothingWithoutAnArea) {
  const Region region = Map("");
  const auto choice = ChooseLane(region, 0, 0, 0.0, kTron, LaneConfig{});
  EXPECT_FALSE(choice.lane);
  EXPECT_DOUBLE_EQ(choice.free_length, 0.0);
}
