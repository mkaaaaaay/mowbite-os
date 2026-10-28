#include <gtest/gtest.h>
#include <stdlib.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

#include "mowbite_map/map_service.h"

using mowbite_map::json;
using mowbite_map::MapService;
using mowbite_map::Notice;

namespace fs = std::filesystem;

namespace {

class TempDir {
 public:
  TempDir() {
    std::string pattern = (fs::temp_directory_path() / "mowbite_service_test_XXXXXX").string();
    path_ = ::mkdtemp(pattern.data());
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path_, ec);
  }
  const fs::path& path() const {
    return path_;
  }

 private:
  fs::path path_;
};

MapService::Config ConfigFor(const TempDir& dir) {
  MapService::Config config;
  config.data_dir = dir.path().string();
  return config;
}

// A map like the app sends it: open rings are fine, the mower closes them.
json AppMap() {
  return json::parse(R"({
    "areas": [
      {"id": "lawn", "properties": {"name": "Front", "type": "mow", "angle": 0.35},
       "outline": [{"x": 0, "y": 0}, {"x": 10, "y": 0}, {"x": 10, "y": 6}, {"x": 0, "y": 6}]},
      {"id": "tree", "properties": {"type": "obstacle"},
       "outline": [{"x": 4, "y": 2}, {"x": 5, "y": 2}, {"x": 5, "y": 3}]}
    ],
    "docking_stations": [{"id": "dock", "position": {"x": -1.5, "y": 3.0}, "heading": 3.14}]
  })");
}

std::string ReadAll(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

void WriteAll(const fs::path& path, const std::string& text) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << text;
}

bool Mentions(const std::vector<Notice>& notices, Notice::Level level, const std::string& part) {
  for (const auto& notice : notices) {
    if (notice.level == level && notice.text.find(part) != std::string::npos) return true;
  }
  return false;
}

}  // namespace

TEST(MapService, AnswersTheTwoMethodsOfOpenMower) {
  EXPECT_EQ(MapService::Methods(), (std::vector<std::string>{"map.replace", "map.area_properties"}));
  TempDir dir;
  MapService service(ConfigFor(dir));
  EXPECT_FALSE(service.Call("map.something", json::array()).has_value());
  EXPECT_FALSE(service.Call("rpc.ping", json()).has_value());
}

TEST(MapService, ReportsWhatItStores) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  auto answer = service.Call("map.area_properties", json());
  ASSERT_TRUE(answer.has_value());
  ASSERT_TRUE(answer->ok);
  const json expected = {"name", "type", "active", "mowable", "angle", "outline_count", "outline_overlap_count",
                         "outline_offset"};
  EXPECT_EQ(answer->result, expected);
  // The planner's settings are not stored by this map, so they are not reported (they are kept anyway).
  for (const char* later : {"mow_around", "angle_min", "angle_max"}) {
    EXPECT_EQ(std::count(answer->result.begin(), answer->result.end(), later), 0) << later;
  }
}

TEST(MapService, StartsWithAnEmptyMapWithoutAFile) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  EXPECT_TRUE(service.map().Empty());
  EXPECT_EQ(json::parse(service.map_text()), json::parse(R"({"areas": [], "docking_stations": []})"));
  EXPECT_TRUE(service.TakeChanged());
  EXPECT_FALSE(service.TakeChanged());
  EXPECT_TRUE(Mentions(service.TakeNotices(), Notice::Level::kInfo, "No map file yet"));
  EXPECT_FALSE(fs::exists(dir.path() / "map.json"));
}

TEST(MapService, ReplacesTheMapAndKeepsItOnDisk) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  service.TakeChanged();
  auto answer = service.Call("map.replace", json::array({AppMap()}));
  ASSERT_TRUE(answer.has_value());
  ASSERT_TRUE(answer->ok) << answer->message;
  EXPECT_EQ(answer->result, "Successfully stored map (2 areas)");
  EXPECT_TRUE(service.TakeChanged());
  EXPECT_EQ(service.map().areas.size(), 2u);
  // Rings are stored closed, the text published is the text on disk.
  EXPECT_EQ(service.map().areas[0].outline.size(), 5u);
  EXPECT_EQ(service.map_text(), ReadAll(dir.path() / "map.json"));
  EXPECT_EQ(json::parse(service.map_text())["areas"][0]["outline"].size(), 5u);
}

TEST(MapService, LoadsTheStoredMapAfterARestart) {
  TempDir dir;
  std::string text;
  {
    MapService service(ConfigFor(dir));
    ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
    text = service.map_text();
  }
  MapService again(ConfigFor(dir));
  EXPECT_EQ(again.map_text(), text);
  EXPECT_EQ(again.map().areas.size(), 2u);
  EXPECT_TRUE(again.TakeChanged());
  EXPECT_TRUE(Mentions(again.TakeNotices(), Notice::Level::kInfo, "2 areas and 1 docking stations from map.json"));
}

TEST(MapService, RefusesAWrongParameterList) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  for (const json& params : {json(), json::array(), json::array({AppMap(), AppMap()}), json(AppMap())}) {
    auto answer = service.Call("map.replace", params);
    ASSERT_TRUE(answer.has_value());
    EXPECT_FALSE(answer->ok);
    EXPECT_EQ(answer->code, -32602);
    EXPECT_EQ(answer->message, "Missing map parameter");
  }
}

TEST(MapService, RefusesAnInvalidMapWithTheReasonAndChangesNothing) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  service.TakeChanged();
  const std::string before = service.map_text();

  json bad = AppMap();
  bad["areas"][1]["outline"] = json::parse(R"([{"x": 0, "y": 0}, {"x": 1, "y": 0}])");
  auto answer = service.Call("map.replace", json::array({bad}));
  ASSERT_TRUE(answer.has_value());
  EXPECT_FALSE(answer->ok);
  EXPECT_EQ(answer->code, -32602);
  EXPECT_EQ(answer->message, "Invalid map: areas[1] 'tree': outline needs at least 3 points");
  EXPECT_FALSE(service.TakeChanged());
  EXPECT_EQ(service.map_text(), before);
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), before);

  answer = service.Call("map.replace", json::array({json::object()}));
  EXPECT_EQ(answer->message, "Invalid map: areas must be an array");
}

TEST(MapService, RefusesAMapAboveTheLimit) {
  TempDir dir;
  MapService::Config config = ConfigFor(dir);
  config.max_bytes = 300;
  MapService service(config);
  auto answer = service.Call("map.replace", json::array({AppMap()}));
  ASSERT_TRUE(answer.has_value());
  EXPECT_FALSE(answer->ok);
  EXPECT_EQ(answer->code, -32602);
  EXPECT_NE(answer->message.find("The map is too large"), std::string::npos) << answer->message;
  EXPECT_NE(answer->message.find("the limit is 300"), std::string::npos) << answer->message;
  EXPECT_TRUE(service.map().Empty());
  EXPECT_FALSE(fs::exists(dir.path() / "map.json"));
}

TEST(MapService, ReportsADiskThatCannotBeWrittenAndKeepsTheOldMap) {
  TempDir dir;
  WriteAll(dir.path() / "file", "not a directory");
  MapService::Config config;
  config.data_dir = (dir.path() / "file" / "sub").string();
  MapService service(config);
  service.TakeChanged();
  auto answer = service.Call("map.replace", json::array({AppMap()}));
  ASSERT_TRUE(answer.has_value());
  EXPECT_FALSE(answer->ok);
  EXPECT_EQ(answer->code, -32603);
  EXPECT_NE(answer->message.find("Cannot store the map: cannot create"), std::string::npos) << answer->message;
  EXPECT_TRUE(service.map().Empty());
  EXPECT_FALSE(service.TakeChanged());
}

TEST(MapService, StoresTheSameMapAgainWithoutTouchingThePreviousVersion) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  json other = AppMap();
  other["areas"][0]["properties"]["name"] = "Back";
  ASSERT_TRUE(service.Call("map.replace", json::array({other}))->ok);
  const std::string previous = ReadAll(dir.path() / "map.json.bak");
  service.TakeChanged();
  // The app saves the map it got back: identical content, nothing to write.
  ASSERT_TRUE(service.Call("map.replace", json::array({other}))->ok);
  EXPECT_FALSE(service.TakeChanged());
  EXPECT_EQ(ReadAll(dir.path() / "map.json.bak"), previous);
  EXPECT_NE(previous.find("Front"), std::string::npos);
}

TEST(MapService, KeepsPropertiesItDoesNotKnow) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  json map = AppMap();
  map["areas"][0]["properties"]["planner"] = json::parse(R"({"pattern": "stripes"})");
  map["areas"][0]["properties"]["mow_around"] = true;
  map["areas"][1]["properties"]["margin"] = 0.3;
  ASSERT_TRUE(service.Call("map.replace", json::array({map}))->ok);
  const json stored = json::parse(ReadAll(dir.path() / "map.json"));
  EXPECT_EQ(stored["areas"][0]["properties"]["planner"], map["areas"][0]["properties"]["planner"]);
  EXPECT_EQ(stored["areas"][0]["properties"]["mow_around"], true);
  EXPECT_EQ(stored["areas"][1]["properties"]["margin"], 0.3);
}

TEST(MapService, WarnsWhenItHadToUseThePreviousVersion) {
  TempDir dir;
  {
    MapService service(ConfigFor(dir));
    ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
    json other = AppMap();
    other["areas"][0]["properties"]["name"] = "Back";
    ASSERT_TRUE(service.Call("map.replace", json::array({other}))->ok);
  }
  WriteAll(dir.path() / "map.json", "{broken");
  MapService service(ConfigFor(dir));
  EXPECT_EQ(service.map().areas[0].name, "Front");
  const auto notices = service.TakeNotices();
  EXPECT_TRUE(Mentions(notices, Notice::Level::kWarn, "from map.json.bak"));
  EXPECT_TRUE(Mentions(notices, Notice::Level::kWarn, "map.json is not valid JSON"));
  // The same map again still repairs map.json, which does not hold it.
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), service.map_text());
}

TEST(MapService, SaysSoWhenNoFileIsUsable) {
  TempDir dir;
  WriteAll(dir.path() / "map.json", "{broken");
  MapService service(ConfigFor(dir));
  EXPECT_TRUE(service.map().Empty());
  EXPECT_TRUE(
      Mentions(service.TakeNotices(), Notice::Level::kError, "No usable map file (map.json is not valid JSON)"));
}
