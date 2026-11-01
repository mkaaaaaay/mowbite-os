#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_safety/region.h"

using mowbite_safety::DistanceToRing;
using mowbite_safety::InsideRing;
using mowbite_safety::ParseRegion;
using mowbite_safety::Ring;
using mowbite_safety::Station;
using mowbite_safety::StationZone;

namespace {

const char* kMap = R"({
  "areas": [
    {"id": "a", "outline": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 6}, {"x": 0, "y": 6}],
     "properties": {"type": "mow", "name": "Lawn"}},
    {"id": "b", "outline": [{"x": 10, "y": 2}, {"x": 14, "y": 2}, {"x": 14, "y": 3}, {"x": 10, "y": 3}],
     "properties": {"type": "nav", "mowable": false}},
    {"id": "c", "outline": [{"x": 4, "y": 2}, {"x": 5, "y": 2}, {"x": 5, "y": 3}, {"x": 4, "y": 3}],
     "properties": {"type": "obstacle"}},
    {"id": "d", "outline": [{"x": 20, "y": 0}, {"x": 30, "y": 0}, {"x": 30, "y": 6}],
     "properties": {"type": "mow", "active": false}},
    {"id": "e", "outline": [{"x": 40, "y": 0}, {"x": 41, "y": 0}, {"x": 41, "y": 1}],
     "properties": {"type": "decor"}}
  ],
  "docking_stations": [
    {"position": {"x": 50, "y": 50}, "heading": 0, "properties": {"active": false}},
    {"position": {"x": -1, "y": 3}, "heading": 3.141592653589793}
  ]
})";

}  // namespace

TEST(Region, InsideOutsideAndDistance) {
  const Ring lawn = {{0, 0}, {10, 0}, {10, 6}, {0, 6}};
  EXPECT_TRUE(InsideRing(lawn, 5, 3));
  EXPECT_FALSE(InsideRing(lawn, -0.1, 3));
  Ring closed = lawn;
  closed.push_back(closed.front());
  EXPECT_TRUE(InsideRing(closed, 9.9, 5.9));
  EXPECT_NEAR(DistanceToRing(lawn, -0.3, 3), 0.3, 1e-9);
  EXPECT_NEAR(DistanceToRing(closed, 5, 3), 3.0, 1e-9);
}

TEST(Region, ActiveMowNavObstacleAndTheFirstActiveStation) {
  std::string error = "old";
  const auto region = ParseRegion(kMap, &error);
  ASSERT_TRUE(region.has_value()) << error;
  EXPECT_EQ(error, "");
  // Inactive and decor areas count neither way.
  ASSERT_EQ(region->allowed.size(), 2u);
  EXPECT_TRUE(InsideRing(region->allowed[1], 12, 2.5));
  ASSERT_EQ(region->obstacles.size(), 1u);
  ASSERT_TRUE(region->station.has_value());
  EXPECT_DOUBLE_EQ(region->station->x, -1);
  EXPECT_DOUBLE_EQ(region->station->y, 3);
}

TEST(Region, AMapWithoutStationOrAreas) {
  std::string error;
  auto region = ParseRegion(R"({"areas": []})", &error);
  ASSERT_TRUE(region.has_value()) << error;
  EXPECT_TRUE(region->allowed.empty());
  EXPECT_FALSE(region->station.has_value());
}

TEST(Region, MalformedEntriesGiveNoRegion) {
  // A skipped entry could be an obstacle, so any malformed one refuses the whole map.
  const char* bad[] = {
      "not json",
      R"({"no_areas": []})",
      R"({"areas": [{"outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}],
                     "properties": {"type": "obstacle", "active": "yes"}}]})",
      R"({"areas": [{"outline": [{"x": 0, "y": 0}, {"x": 1}, {"x": 1, "y": 1}], "properties": {"type": "obstacle"}}]})",
      R"({"areas": [{"outline": [{"x": 0, "y": 0}, {"x": 1, "y": 0}], "properties": {"type": "mow"}}]})",
      R"({"areas": [{"properties": {"type": "nav"}}]})",
      R"({"areas": [{"outline": [], "properties": 5}]})",
      R"({"areas": [{"outline": [], "properties": {"type": 3}}]})",
      R"({"areas": [], "docking_stations": [{"position": {"x": 0, "y": 0}}]})",
      R"({"areas": [], "docking_stations": {"position": {"x": 0, "y": 0}, "heading": 0}})",
  };
  for (const char* text : bad) {
    std::string error;
    EXPECT_FALSE(ParseRegion(text, &error).has_value()) << text;
    EXPECT_NE(error, "") << text;
  }
}

TEST(Region, StationZoneRunsAgainstTheHeading) {
  // Backing in going west (heading pi): the zone reaches 0.15 m past the station to the west and 2.1 m to the east.
  const Ring zone = StationZone(Station{-1, 3, M_PI}, 0.15, 2.1, 0.1);
  EXPECT_TRUE(InsideRing(zone, -1.1, 3));
  EXPECT_FALSE(InsideRing(zone, -1.2, 3));
  EXPECT_TRUE(InsideRing(zone, 1.0, 3.05));
  EXPECT_FALSE(InsideRing(zone, 1.2, 3));
  EXPECT_FALSE(InsideRing(zone, 0, 3.15));
  EXPECT_NEAR(DistanceToRing(zone, 0, 3.6), 0.5, 1e-9);
}
