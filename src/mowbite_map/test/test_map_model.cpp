#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "mowbite_map/map_model.h"

using mowbite_map::json;
using mowbite_map::ParseMap;
using mowbite_map::ToJson;

namespace {

// A map the way the app sends it: closed rings, a mowing area with settings, an obstacle, one dock.
json SampleMap() {
  return json::parse(R"({
    "areas": [
      {"id": "lawn", "properties": {"name": "Front", "type": "mow", "angle": 0.35, "outline_count": 2,
                                    "outline_overlap_count": 1, "outline_offset": 0.1},
       "outline": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 6}, {"x": 0, "y": 6}, {"x": 0, "y": 0}]},
      {"id": "tree", "properties": {"type": "obstacle"},
       "outline": [{"x": 4, "y": 2}, {"x": 5, "y": 2}, {"x": 5, "y": 3}, {"x": 4, "y": 2}]}
    ],
    "docking_stations": [{"id": "dock", "position": {"x": -1.5, "y": 3.0}, "heading": 3.14}]
  })");
}

std::string Error(const json& map) {
  auto result = ParseMap(map);
  EXPECT_FALSE(result.map.has_value());
  return result.error;
}

}  // namespace

TEST(MapModel, ParsesTheMapOfTheApp) {
  auto result = ParseMap(SampleMap());
  ASSERT_TRUE(result.map.has_value()) << result.error;
  const auto& map = *result.map;
  ASSERT_EQ(map.areas.size(), 2u);
  EXPECT_EQ(map.areas[0].id, "lawn");
  EXPECT_EQ(map.areas[0].name, "Front");
  EXPECT_EQ(map.areas[0].type, "mow");
  EXPECT_TRUE(map.areas[0].active);
  EXPECT_TRUE(map.areas[0].mowable);
  EXPECT_DOUBLE_EQ(*map.areas[0].angle, 0.35);
  EXPECT_EQ(*map.areas[0].outline_count, 2);
  EXPECT_EQ(*map.areas[0].outline_overlap_count, 1);
  EXPECT_DOUBLE_EQ(*map.areas[0].outline_offset, 0.1);
  EXPECT_EQ(map.areas[0].outline.size(), 5u);
  EXPECT_EQ(map.areas[1].type, "obstacle");
  EXPECT_FALSE(map.areas[1].angle.has_value());
  ASSERT_EQ(map.docking_stations.size(), 1u);
  EXPECT_EQ(map.docking_stations[0].id, "dock");
  EXPECT_DOUBLE_EQ(map.docking_stations[0].position.x, -1.5);
  EXPECT_DOUBLE_EQ(map.docking_stations[0].heading, 3.14);
  EXPECT_FALSE(map.Empty());
}

TEST(MapModel, WritesWhatItRead) {
  const json in = SampleMap();
  auto result = ParseMap(in);
  ASSERT_TRUE(result.map.has_value());
  const json out = ToJson(*result.map);
  EXPECT_EQ(out, in);
  // The key order of OpenMower: id, properties, outline; position and heading for a dock.
  std::vector<std::string> keys;
  for (const auto& item : out["areas"][0].items()) keys.push_back(item.key());
  EXPECT_EQ(keys, (std::vector<std::string>{"id", "properties", "outline"}));
  keys.clear();
  for (const auto& item : out["docking_stations"][0].items()) keys.push_back(item.key());
  EXPECT_EQ(keys, (std::vector<std::string>{"id", "position", "heading"}));
}

TEST(MapModel, StoresRingsClosedWhateverCameIn) {
  json open = SampleMap();
  auto& outline = open["areas"][0]["outline"];
  outline.erase(outline.end() - 1);
  ASSERT_EQ(outline.size(), 4u);
  auto result = ParseMap(open);
  ASSERT_TRUE(result.map.has_value()) << result.error;
  const auto& ring = result.map->areas[0].outline;
  ASSERT_EQ(ring.size(), 5u);
  EXPECT_EQ(ring.front().x, ring.back().x);
  EXPECT_EQ(ring.front().y, ring.back().y);
  // A closed ring stays at five points and is not closed a second time.
  EXPECT_EQ(ParseMap(SampleMap()).map->areas[0].outline.size(), 5u);
}

TEST(MapModel, TreatsAnAlmostRepeatedFirstPointAsClosing) {
  json map = SampleMap();
  map["areas"][0]["outline"][4] = json{{"x", 1e-12}, {"y", 0.0}};
  auto result = ParseMap(map);
  ASSERT_TRUE(result.map.has_value());
  const auto& ring = result.map->areas[0].outline;
  ASSERT_EQ(ring.size(), 5u);
  EXPECT_EQ(ring.back().x, 0.0);
}

TEST(MapModel, FillsDefaultsAndLeavesThemOutWhenWriting) {
  auto result = ParseMap(json::parse(R"({"areas": [{"id": "a", "outline": [
      {"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]}]})"));
  ASSERT_TRUE(result.map.has_value()) << result.error;
  const auto& area = result.map->areas[0];
  EXPECT_EQ(area.type, "draft");
  EXPECT_TRUE(area.active);
  EXPECT_TRUE(area.mowable);
  EXPECT_TRUE(result.map->docking_stations.empty());
  const json out = ToJson(*result.map);
  EXPECT_EQ(out["areas"][0]["properties"], json::parse(R"({"type": "draft"})"));
  EXPECT_TRUE(out["docking_stations"].is_array());
}

TEST(MapModel, WritesActiveAndMowableOnlyWhenFalse) {
  json map = SampleMap();
  map["areas"][0]["properties"]["active"] = false;
  map["areas"][0]["properties"]["mowable"] = false;
  map["areas"][1]["properties"]["active"] = true;
  auto result = ParseMap(map);
  ASSERT_TRUE(result.map.has_value());
  EXPECT_FALSE(result.map->areas[0].active);
  EXPECT_FALSE(result.map->areas[0].mowable);
  const json out = ToJson(*result.map);
  EXPECT_EQ(out["areas"][0]["properties"]["active"], false);
  EXPECT_EQ(out["areas"][0]["properties"]["mowable"], false);
  EXPECT_FALSE(out["areas"][1]["properties"].contains("active"));
}

TEST(MapModel, KeepsPropertiesItDoesNotKnow) {
  json map = SampleMap();
  map["areas"][0]["properties"]["planner"] = json::parse(R"({"pattern": "stripes", "turns": [1, 2]})");
  map["areas"][0]["properties"]["mow_around"] = true;
  map["areas"][0]["properties"]["angle_min"] = -0.5;
  map["areas"][0]["properties"]["angle_max"] = 0.5;
  map["areas"][1]["properties"]["margin"] = 0.3;
  map["areas"][1]["properties"]["pin"] = true;
  auto result = ParseMap(map);
  ASSERT_TRUE(result.map.has_value());
  const json out = ToJson(*result.map);
  EXPECT_EQ(out["areas"][0]["properties"]["planner"], map["areas"][0]["properties"]["planner"]);
  EXPECT_EQ(out["areas"][0]["properties"]["mow_around"], true);
  EXPECT_DOUBLE_EQ(out["areas"][0]["properties"]["angle_min"].get<double>(), -0.5);
  EXPECT_DOUBLE_EQ(out["areas"][0]["properties"]["angle_max"].get<double>(), 0.5);
  EXPECT_DOUBLE_EQ(out["areas"][1]["properties"]["margin"].get<double>(), 0.3);
  EXPECT_EQ(out["areas"][1]["properties"]["pin"], true);
  // Written once, behind the known ones, and parsing it again changes nothing.
  EXPECT_EQ(ToJson(*ParseMap(out).map), out);
}

TEST(MapModel, TreatsNullPropertiesAsNotSet) {
  json map = SampleMap();
  map["areas"][0]["properties"]["angle"] = nullptr;
  map["areas"][0]["properties"]["name"] = nullptr;
  auto result = ParseMap(map);
  ASSERT_TRUE(result.map.has_value()) << result.error;
  EXPECT_FALSE(result.map->areas[0].angle.has_value());
  EXPECT_TRUE(result.map->areas[0].name.empty());
  EXPECT_FALSE(ToJson(*result.map)["areas"][0]["properties"].contains("angle"));
}

TEST(MapModel, AcceptsAllTypesTheAppWrites) {
  for (const char* type : {"mow", "nav", "obstacle", "decor", "draft"}) {
    json map = SampleMap();
    map["areas"][0]["properties"]["type"] = type;
    EXPECT_TRUE(ParseMap(map).map.has_value()) << type;
    EXPECT_TRUE(mowbite_map::IsAreaType(type)) << type;
  }
  EXPECT_FALSE(mowbite_map::IsAreaType("garden"));
}

TEST(MapModel, DockPropertiesAndDefaults) {
  json map = SampleMap();
  map["docking_stations"][0]["properties"] = json::parse(R"({"name": "Shed", "active": false, "note": 7})");
  auto result = ParseMap(map);
  ASSERT_TRUE(result.map.has_value()) << result.error;
  EXPECT_EQ(result.map->docking_stations[0].name, "Shed");
  EXPECT_FALSE(result.map->docking_stations[0].active);
  EXPECT_EQ(ToJson(*result.map)["docking_stations"][0]["properties"]["note"], 7);
  // Without properties none are written.
  EXPECT_FALSE(ToJson(*ParseMap(SampleMap()).map)["docking_stations"][0].contains("properties"));
}

TEST(MapModel, EmptyMapIsValid) {
  auto result = ParseMap(json::parse(R"({"areas": [], "docking_stations": []})"));
  ASSERT_TRUE(result.map.has_value());
  EXPECT_TRUE(result.map->Empty());
  EXPECT_EQ(ToJson(*result.map).dump(), R"({"areas":[],"docking_stations":[]})");
}

TEST(MapModel, RefusesWhatIsNotAMap) {
  EXPECT_NE(Error(json::array()).find("map must be an object"), std::string::npos);
  EXPECT_NE(Error(json::object()).find("areas must be an array"), std::string::npos);
  EXPECT_NE(Error(json::parse(R"({"areas": {}})")).find("areas must be an array"), std::string::npos);
  EXPECT_NE(Error(json::parse(R"({"areas": [], "docking_stations": 1})")).find("docking_stations"), std::string::npos);
  EXPECT_NE(Error(json::parse(R"({"areas": [3]})")).find("areas[0] must be an object"), std::string::npos);
}

TEST(MapModel, RefusesBadIds) {
  json map = SampleMap();
  map["areas"][1]["id"] = "lawn";
  EXPECT_NE(Error(map).find("'lawn' is used twice"), std::string::npos);
  map = SampleMap();
  map["areas"][0].erase("id");
  EXPECT_NE(Error(map).find("areas[0] needs an id"), std::string::npos);
  map = SampleMap();
  map["areas"][0]["id"] = "";
  EXPECT_NE(Error(map).find("needs an id"), std::string::npos);
  map = SampleMap();
  map["areas"][0]["id"] = 5;
  EXPECT_NE(Error(map).find("needs an id"), std::string::npos);
  map = SampleMap();
  map["areas"][0]["id"] = std::string(129, 'x');
  EXPECT_NE(Error(map).find("longer than"), std::string::npos);
  map = SampleMap();
  const json dock = map["docking_stations"][0];
  map["docking_stations"].push_back(dock);
  EXPECT_NE(Error(map).find("docking_stations[1]"), std::string::npos);
  // An area and a dock may share an id, they are different lists.
  map = SampleMap();
  map["docking_stations"][0]["id"] = "lawn";
  EXPECT_TRUE(ParseMap(map).map.has_value());
}

TEST(MapModel, RefusesBadOutlines) {
  json map = SampleMap();
  map["areas"][0]["outline"] = json::parse(R"([{"x": 0, "y": 0}, {"x": 1, "y": 0}])");
  EXPECT_NE(Error(map).find("at least 3 points"), std::string::npos);
  // Three points where the last repeats the first are only two.
  map["areas"][0]["outline"] = json::parse(R"([{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 0, "y": 0}])");
  EXPECT_NE(Error(map).find("at least 3 points"), std::string::npos);
  map["areas"][0]["outline"] = json::array();
  EXPECT_NE(Error(map).find("at least 3 points"), std::string::npos);
  map["areas"][0].erase("outline");
  EXPECT_NE(Error(map).find("outline must be an array"), std::string::npos);
  map = SampleMap();
  map["areas"][0]["outline"][1] = json::parse(R"({"x": "a", "y": 0})");
  EXPECT_NE(Error(map).find("outline[1].x must be a finite number"), std::string::npos);
  map["areas"][0]["outline"][1] = json::parse(R"({"x": 1})");
  EXPECT_NE(Error(map).find("outline[1] needs x and y"), std::string::npos);
  map["areas"][0]["outline"][1] = 7;
  EXPECT_NE(Error(map).find("outline[1] needs x and y"), std::string::npos);
}

TEST(MapModel, RefusesNumbersThatAreNotFinite) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  json map = SampleMap();
  map["areas"][0]["outline"][2]["y"] = nan;
  EXPECT_NE(Error(map).find("outline[2].y"), std::string::npos);
  map = SampleMap();
  map["areas"][0]["outline"][2]["x"] = kInf;
  EXPECT_NE(Error(map).find("outline[2].x"), std::string::npos);
  map = SampleMap();
  map["areas"][0]["properties"]["angle"] = nan;
  EXPECT_NE(Error(map).find("angle"), std::string::npos);
  map = SampleMap();
  map["docking_stations"][0]["heading"] = kInf;
  EXPECT_NE(Error(map).find("heading"), std::string::npos);
  map = SampleMap();
  map["docking_stations"][0]["position"]["x"] = nan;
  EXPECT_NE(Error(map).find("position.x"), std::string::npos);
}

TEST(MapModel, RefusesBadProperties) {
  const auto with = [](const char* key, const json& value) {
    json map = SampleMap();
    map["areas"][0]["properties"][key] = value;
    return Error(map);
  };
  EXPECT_NE(with("type", "garden").find("unknown type 'garden'"), std::string::npos);
  EXPECT_NE(with("type", 3).find("type must be a string"), std::string::npos);
  EXPECT_NE(with("name", 3).find("name must be a string"), std::string::npos);
  EXPECT_NE(with("active", "yes").find("active must be true or false"), std::string::npos);
  EXPECT_NE(with("active", 1).find("active must be true or false"), std::string::npos);
  EXPECT_NE(with("mowable", 0).find("mowable must be true or false"), std::string::npos);
  EXPECT_NE(with("angle", "x").find("angle must be a finite number"), std::string::npos);
  EXPECT_NE(with("outline_offset", true).find("outline_offset"), std::string::npos);
  EXPECT_NE(with("outline_count", 1.5).find("outline_count must be a whole number"), std::string::npos);
  EXPECT_NE(with("outline_count", -1).find("outline_count must be 0 or more"), std::string::npos);
  EXPECT_NE(with("outline_overlap_count", 3000000000LL).find("outline_overlap_count must be 0 or more"),
            std::string::npos);
  json map = SampleMap();
  map["areas"][0]["properties"] = 5;
  EXPECT_NE(Error(map).find("properties must be an object"), std::string::npos);
}

TEST(MapModel, RefusesBadDocks) {
  json map = SampleMap();
  map["docking_stations"][0].erase("position");
  EXPECT_NE(Error(map).find("needs a position"), std::string::npos);
  map = SampleMap();
  map["docking_stations"][0].erase("heading");
  EXPECT_NE(Error(map).find("needs a heading"), std::string::npos);
  map = SampleMap();
  map["docking_stations"][0]["position"] = json::parse(R"({"x": 1})");
  EXPECT_NE(Error(map).find("position needs x and y"), std::string::npos);
  map = SampleMap();
  map["docking_stations"][0]["properties"] = json::parse(R"({"active": "no"})");
  EXPECT_NE(Error(map).find("active must be true or false"), std::string::npos);
}

TEST(MapModel, NamesTheAreaInTheError) {
  json map = SampleMap();
  map["areas"][1]["properties"]["type"] = "garden";
  EXPECT_EQ(Error(map), "areas[1] 'tree': unknown type 'garden'");
}

TEST(MapModel, TextIsIndentedByTwoLikeOpenMower) {
  EXPECT_EQ(mowbite_map::ToText(mowbite_map::MapData{}), "{\n  \"areas\": [],\n  \"docking_stations\": []\n}");
  const std::string text = mowbite_map::ToText(*ParseMap(SampleMap()).map);
  EXPECT_NE(text.find("\n      \"id\": \"lawn\""), std::string::npos);
  EXPECT_EQ(json::parse(text), ToJson(*ParseMap(SampleMap()).map));
}

TEST(MapModel, ReportsTheStoredAreaProperties) {
  const std::vector<std::string> expected = {"name",  "type", "active", "mowable", "angle", "outline_count",
                                             "outline_overlap_count", "outline_offset"};
  EXPECT_EQ(mowbite_map::AreaPropertyNames(), expected);
}
