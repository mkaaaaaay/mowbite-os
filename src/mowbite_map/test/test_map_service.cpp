#include <gtest/gtest.h>
#include <stdlib.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
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

namespace {

std::vector<mowbite_map::Point> Square(double size) {
  return {{0, 0}, {size, 0}, {size, size}, {0, size}};
}

// Ids "id1", "id2", ... so the tests can name them.
MapService::Config NumberedIds(const TempDir& dir) {
  MapService::Config config = ConfigFor(dir);
  auto counter = std::make_shared<int>(0);
  config.new_id = [counter] { return "id" + std::to_string(++*counter); };
  return config;
}

}  // namespace

TEST(MapServiceEdits, AddsAnAreaForTheLogic) {
  TempDir dir;
  MapService service(NumberedIds(dir));
  service.TakeChanged();
  const auto result = service.AddArea("mow", "Front", Square(5));
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.id, "id1");
  ASSERT_EQ(service.map().areas.size(), 1u);
  const auto& area = service.map().areas[0];
  EXPECT_EQ(area.id, "id1");
  EXPECT_EQ(area.name, "Front");
  EXPECT_EQ(area.type, "mow");
  EXPECT_TRUE(area.active);
  // The ring is closed when stored.
  ASSERT_EQ(area.outline.size(), 5u);
  EXPECT_EQ(area.outline.front().x, area.outline.back().x);
  EXPECT_TRUE(service.TakeChanged());
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), service.map_text());
}

TEST(MapServiceEdits, KeepsWhatIsThereWhenAddingAndAfterARestart) {
  TempDir dir;
  json map = AppMap();
  map["areas"][0]["properties"]["planner"] = json::parse(R"({"pattern": "stripes"})");
  {
    MapService service(NumberedIds(dir));
    ASSERT_TRUE(service.Call("map.replace", json::array({map}))->ok);
    const auto result = service.AddArea("nav", "Path", Square(2));
    ASSERT_TRUE(result.ok) << result.message;
    EXPECT_EQ(result.id, "id1");
  }
  MapService again(ConfigFor(dir));
  ASSERT_EQ(again.map().areas.size(), 3u);
  EXPECT_EQ(again.map().areas[2].type, "nav");
  EXPECT_EQ(again.map().areas[0].other_properties["planner"]["pattern"], "stripes");
  EXPECT_EQ(again.map().docking_stations.size(), 1u);
}

TEST(MapServiceEdits, RefusesAnAreaThatIsNotOne) {
  TempDir dir;
  MapService service(NumberedIds(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  service.TakeChanged();
  const std::string before = service.map_text();

  auto result = service.AddArea("mow", "Line", {{0, 0}, {1, 0}});
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("Invalid area 'Line' (mow): "), std::string::npos) << result.message;
  EXPECT_NE(result.message.find("outline needs at least 3 points"), std::string::npos) << result.message;
  EXPECT_TRUE(result.id.empty());

  result = service.AddArea("garden", "Odd", Square(2));
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("unknown type 'garden'"), std::string::npos) << result.message;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  result = service.AddArea("mow", "Nan", {{0, 0}, {1, 0}, {1, nan}});
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("finite number"), std::string::npos) << result.message;

  EXPECT_FALSE(service.TakeChanged());
  EXPECT_EQ(service.map_text(), before);
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), before);
}

TEST(MapServiceEdits, TakesAFreshIdWhenTheSourceRepeats) {
  TempDir dir;
  MapService::Config config = ConfigFor(dir);
  auto ids = std::make_shared<std::vector<std::string>>(std::vector<std::string>{"same", "same", "same", "other"});
  config.new_id = [ids] {
    std::string id = ids->front();
    if (ids->size() > 1) ids->erase(ids->begin());
    return id;
  };
  MapService service(config);
  ASSERT_TRUE(service.AddArea("mow", "One", Square(2)).ok);
  const auto second = service.AddArea("mow", "Two", Square(3));
  ASSERT_TRUE(second.ok) << second.message;
  EXPECT_EQ(second.id, "other");
  EXPECT_EQ(service.map().areas[0].id, "same");
}

TEST(MapServiceEdits, MakesRandomIdsOfThe32CharactersOpenMowerUses) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  const auto first = service.AddArea("mow", "A", Square(2));
  const auto second = service.AddArea("mow", "B", Square(3));
  ASSERT_TRUE(first.ok && second.ok);
  EXPECT_EQ(first.id.size(), 32u);
  EXPECT_NE(first.id, second.id);
  EXPECT_EQ(first.id.find_first_not_of("0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"),
            std::string::npos);
}

TEST(MapServiceEdits, RefusesAnAreaThatMakesTheMapTooLarge) {
  TempDir dir;
  TempDir probe_dir;
  MapService probe(NumberedIds(probe_dir));
  ASSERT_TRUE(probe.AddArea("mow", "Small", Square(2)).ok);
  MapService::Config config = NumberedIds(dir);
  config.max_bytes = probe.map_text().size() + 50;
  MapService service(config);
  ASSERT_TRUE(service.AddArea("mow", "Small", Square(2)).ok);
  const std::string before = service.map_text();
  std::vector<mowbite_map::Point> many;
  for (int i = 0; i < 40; ++i) many.push_back({static_cast<double>(i), static_cast<double>(i % 3)});
  const auto result = service.AddArea("mow", "Big", many);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("The map is too large"), std::string::npos) << result.message;
  EXPECT_EQ(service.map_text(), before);
}

TEST(MapServiceEdits, SetsTheDockAsTheOnlyOne) {
  TempDir dir;
  MapService service(NumberedIds(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  ASSERT_EQ(service.map().docking_stations.size(), 1u);
  const auto result = service.SetDock({2.5, -1.0}, 1.57);
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.id, "id1");
  ASSERT_EQ(service.map().docking_stations.size(), 1u);
  const auto& dock = service.map().docking_stations[0];
  EXPECT_EQ(dock.id, "id1");
  EXPECT_EQ(dock.name, "Docking Station");
  EXPECT_DOUBLE_EQ(dock.position.x, 2.5);
  EXPECT_DOUBLE_EQ(dock.position.y, -1.0);
  EXPECT_DOUBLE_EQ(dock.heading, 1.57);
  EXPECT_EQ(service.map().areas.size(), 2u);
  EXPECT_EQ(json::parse(ReadAll(dir.path() / "map.json"))["docking_stations"][0]["heading"], 1.57);
}

TEST(MapServiceEdits, RefusesADockThatIsNotFinite) {
  TempDir dir;
  MapService service(NumberedIds(dir));
  constexpr double kInf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(service.SetDock({0, 0}, kInf).ok);
  EXPECT_FALSE(service.SetDock({kInf, 0}, 0).ok);
  EXPECT_TRUE(service.map().docking_stations.empty());
}

TEST(MapServiceEdits, TellsTheLogWhatWasStored) {
  TempDir dir;
  MapService service(NumberedIds(dir));
  service.TakeNotices();
  ASSERT_TRUE(service.AddArea("mow", "Front", Square(2)).ok);
  ASSERT_TRUE(service.SetDock({0, 0}, 0).ok);
  const auto notices = service.TakeNotices();
  EXPECT_TRUE(Mentions(notices, Notice::Level::kInfo, "Stored the area 'Front' (mow), id id1"));
  EXPECT_TRUE(Mentions(notices, Notice::Level::kInfo, "Stored the docking station, id id2"));
}
