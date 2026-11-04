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
#include "mowbite_map/sha256.h"

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

// A garden somewhere; the datum a map is recorded in.
const mowbite_map::Datum kDatum{52.0, 9.0, 50.0};

// A service that knows its datum, with the start-up messages taken.
MapService Started(const MapService::Config& config) {
  MapService service(config);
  service.SetDatum(kDatum);
  service.TakeChanged();
  service.TakeStatusChanged();
  service.TakeNotices();
  service.TakeEvents();
  return service;
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
  MapService service = Started(ConfigFor(dir));
  EXPECT_FALSE(service.Call("map.something", json::array()).has_value());
  EXPECT_FALSE(service.Call("rpc.ping", json()).has_value());
}

TEST(MapService, ReportsWhatItStores) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
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

TEST(MapService, ReportsThePlannerPropertiesOnlyWhilePlannerSettingsIsOffered) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  const auto names = [&] { return service.Call("map.area_properties", json())->result; };
  EXPECT_EQ(names().size(), 8u);
  service.SetPlannerSettingsOffered(true);
  const json offered = {"name",          "type",          "active",    "mowable",   "angle",     "outline_count",
                        "outline_overlap_count", "outline_offset", "mow_around", "angle_min", "angle_max"};
  EXPECT_EQ(names(), offered);
  service.SetPlannerSettingsOffered(false);
  EXPECT_EQ(names().size(), 8u);
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
  MapService service = Started(ConfigFor(dir));
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
    MapService service = Started(ConfigFor(dir));
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
  MapService service = Started(ConfigFor(dir));
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
  MapService service = Started(ConfigFor(dir));
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
  MapService service = Started(config);
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
  MapService service = Started(config);
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
  MapService service = Started(ConfigFor(dir));
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
  MapService service = Started(ConfigFor(dir));
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
    MapService service = Started(ConfigFor(dir));
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
  service.SetDatum(kDatum);
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
  MapService service = Started(NumberedIds(dir));
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

TEST(MapServiceEdits, AddsTheObstaclesOfAnAreaInTheSameWrite) {
  TempDir dir;
  MapService service = Started(NumberedIds(dir));
  const std::vector<mowbite_map::Point> tree = {{1, 1}, {2, 1}, {2, 2}};
  const std::vector<mowbite_map::Point> bed = {{3, 3}, {4, 3}, {4, 4}, {3, 4}, {3, 3}};
  const auto result = service.AddArea("mow", "Back", Square(5), {tree, bed});
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.id, "id1");
  ASSERT_EQ(service.map().areas.size(), 3u);
  EXPECT_EQ(service.map().areas[1].id, "id2");
  EXPECT_EQ(service.map().areas[1].type, "obstacle");
  EXPECT_EQ(service.map().areas[1].outline.size(), 4u);
  EXPECT_EQ(service.map().areas[2].id, "id3");
  EXPECT_EQ(service.map().areas[2].outline.size(), 5u);
  MapService again(ConfigFor(dir));
  EXPECT_EQ(again.map().areas.size(), 3u);
}

TEST(MapServiceEdits, StoresNoneOfTheAreaWhenAnObstacleIsInvalid) {
  TempDir dir;
  MapService service = Started(NumberedIds(dir));
  service.TakeChanged();
  const auto result = service.AddArea("mow", "Back", Square(5), {{{1, 1}, {2, 1}}});
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("with 1 obstacles"), std::string::npos) << result.message;
  EXPECT_NE(result.message.find("outline needs at least 3 points"), std::string::npos) << result.message;
  EXPECT_TRUE(service.map().areas.empty());
  EXPECT_FALSE(service.TakeChanged());
}

TEST(MapServiceEdits, GivesTheObstaclesIdsOfTheirOwnWhenTheSourceRepeats) {
  TempDir dir;
  MapService::Config config = ConfigFor(dir);
  auto ids = std::make_shared<std::vector<std::string>>(std::vector<std::string>{"a", "a", "b", "b", "c"});
  config.new_id = [ids] {
    std::string id = ids->front();
    if (ids->size() > 1) ids->erase(ids->begin());
    return id;
  };
  MapService service = Started(config);
  const auto result = service.AddArea("mow", "One", Square(5), {{{1, 1}, {2, 1}, {2, 2}}, {{3, 3}, {4, 3}, {4, 4}}});
  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(service.map().areas.size(), 3u);
  EXPECT_EQ(service.map().areas[0].id, "a");
  EXPECT_EQ(service.map().areas[1].id, "b");
  EXPECT_EQ(service.map().areas[2].id, "c");
}

TEST(MapServiceEdits, KeepsWhatIsThereWhenAddingAndAfterARestart) {
  TempDir dir;
  json map = AppMap();
  map["areas"][0]["properties"]["planner"] = json::parse(R"({"pattern": "stripes"})");
  {
    MapService service = Started(NumberedIds(dir));
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
  MapService service = Started(NumberedIds(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  service.TakeChanged();
  const std::string before = service.map_text();

  auto result = service.AddArea("mow", "Line", {{0, 0}, {1, 0}});
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("Invalid area 'Line' (mow): "), std::string::npos) << result.message;
  EXPECT_EQ(result.code, "INVALID");
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
  MapService service = Started(config);
  ASSERT_TRUE(service.AddArea("mow", "One", Square(2)).ok);
  const auto second = service.AddArea("mow", "Two", Square(3));
  ASSERT_TRUE(second.ok) << second.message;
  EXPECT_EQ(second.id, "other");
  EXPECT_EQ(service.map().areas[0].id, "same");
}

TEST(MapServiceEdits, MakesRandomIdsOfThe32CharactersOpenMowerUses) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
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
  MapService probe = Started(NumberedIds(probe_dir));
  ASSERT_TRUE(probe.AddArea("mow", "Small", Square(2)).ok);
  MapService::Config config = NumberedIds(dir);
  config.max_bytes = probe.map_text().size() + 50;
  MapService service = Started(config);
  ASSERT_TRUE(service.AddArea("mow", "Small", Square(2)).ok);
  const std::string before = service.map_text();
  std::vector<mowbite_map::Point> many;
  for (int i = 0; i < 40; ++i) many.push_back({static_cast<double>(i), static_cast<double>(i % 3)});
  const auto result = service.AddArea("mow", "Big", many);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.message.find("The map is too large"), std::string::npos) << result.message;
  EXPECT_EQ(result.code, "TOO_LARGE");
  EXPECT_EQ(service.map_text(), before);
}

TEST(MapServiceEdits, SetsTheDockAsTheOnlyOne) {
  TempDir dir;
  MapService service = Started(NumberedIds(dir));
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
  MapService service = Started(NumberedIds(dir));
  constexpr double kInf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(service.SetDock({0, 0}, kInf).ok);
  EXPECT_FALSE(service.SetDock({kInf, 0}, 0).ok);
  EXPECT_TRUE(service.map().docking_stations.empty());
}

TEST(MapServiceEdits, TellsTheLogWhatWasStored) {
  TempDir dir;
  MapService service = Started(NumberedIds(dir));
  service.TakeNotices();
  ASSERT_TRUE(service.AddArea("mow", "Front", Square(2)).ok);
  ASSERT_TRUE(service.SetDock({0, 0}, 0).ok);
  const auto notices = service.TakeNotices();
  EXPECT_TRUE(Mentions(notices, Notice::Level::kInfo, "Stored the area 'Front' (mow), id id1"));
  EXPECT_TRUE(Mentions(notices, Notice::Level::kInfo, "Stored the docking station, id id2"));
}

namespace {

using mowbite_map::DatumState;
using mowbite_map::Event;

// kDatum moved north by this many metres.
mowbite_map::Datum Moved(double north_m) {
  mowbite_map::Datum datum = kDatum;
  datum.lat += north_m / 111320.0;
  return datum;
}

bool HasEvent(const std::vector<Event>& events, const std::string& state) {
  for (const auto& event : events) {
    if (event.type == "MAP_DATUM" && event.details["state"] == state) return true;
  }
  return false;
}

// A map stored for kDatum in the directory, as a service of an earlier run left it.
void StoreAppMap(const TempDir& dir) {
  MapService service = Started(ConfigFor(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
}

}  // namespace

TEST(MapDatum, WaitsForTheDatumOfTheGnssAndStoresNothingBeforeIt) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  EXPECT_EQ(service.status().datum, DatumState::kUnknown);
  EXPECT_FALSE(service.status().mowing_allowed);
  EXPECT_NE(service.status().reason.find("not known yet"), std::string::npos);

  auto answer = service.Call("map.replace", json::array({AppMap()}));
  ASSERT_TRUE(answer.has_value());
  EXPECT_FALSE(answer->ok);
  EXPECT_EQ(answer->code, -32000);
  EXPECT_NE(answer->message.find("The datum of the GNSS is not known yet"), std::string::npos) << answer->message;
  EXPECT_FALSE(service.AddArea("mow", "Front", Square(2)).ok);
  EXPECT_FALSE(service.SetDock({0, 0}, 0).ok);
  EXPECT_TRUE(service.map().Empty());
  EXPECT_FALSE(fs::exists(dir.path() / "map.json"));
  // An empty map needs no datum.
  EXPECT_TRUE(service.Call("map.replace", json::array({json::parse(R"({"areas": [], "docking_stations": []})")}))->ok);
}

TEST(MapDatum, AnEmptyMapHasNothingToTie) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  EXPECT_TRUE(service.TakeStatusChanged());
  service.SetDatum(kDatum);
  EXPECT_EQ(service.status().datum, DatumState::kEmpty);
  EXPECT_FALSE(service.status().mowing_allowed);
  EXPECT_TRUE(service.TakeStatusChanged());
  EXPECT_FALSE(service.TakeStatusChanged());
  EXPECT_TRUE(service.TakeEvents().empty());
}

TEST(MapDatum, TiesAStoredMapToTheDatumInUse) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  const auto& status = service.status();
  EXPECT_EQ(status.datum, DatumState::kMatch);
  EXPECT_TRUE(status.mowing_allowed);
  EXPECT_TRUE(status.reason.empty());
  EXPECT_EQ(status.areas, 2u);
  EXPECT_EQ(status.docking_stations, 1u);
  EXPECT_TRUE(service.TakeStatusChanged());
  const json file = json::parse(ReadAll(dir.path() / "map_datum.json"));
  EXPECT_EQ(file["schema"], 1);
  EXPECT_EQ(file["datum_lat"], 52.0);
  EXPECT_EQ(file["datum_long"], 9.0);
  EXPECT_EQ(file["datum_height"], 50.0);
}

TEST(MapDatum, KeepsTheTieAcrossARestart) {
  TempDir dir;
  StoreAppMap(dir);
  MapService again(ConfigFor(dir));
  EXPECT_EQ(again.status().datum, DatumState::kUnknown);
  again.SetDatum(kDatum);
  EXPECT_EQ(again.status().datum, DatumState::kMatch);
  EXPECT_TRUE(again.status().mowing_allowed);
  EXPECT_TRUE(again.TakeEvents().empty());
  EXPECT_TRUE(Mentions(again.TakeNotices(), Notice::Level::kInfo, "The map belongs to the datum in use"));
}

TEST(MapDatum, StopsMowingWhenTheDatumChanged) {
  TempDir dir;
  StoreAppMap(dir);
  MapService again(ConfigFor(dir));
  again.TakeNotices();
  again.SetDatum(Moved(1000.0));
  const auto& status = again.status();
  EXPECT_EQ(status.datum, DatumState::kMismatch);
  EXPECT_FALSE(status.mowing_allowed);
  EXPECT_NE(status.reason.find("recorded for the datum 52.0000000, 9.0000000"), std::string::npos) << status.reason;
  EXPECT_NE(status.reason.find("1000.00 m apart"), std::string::npos) << status.reason;
  EXPECT_TRUE(Mentions(again.TakeNotices(), Notice::Level::kError, "No mowing, the map does not fit the datum"));
  const auto events = again.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_TRUE(HasEvent(events, "mismatch"));
  EXPECT_EQ(events[0].details["reason"], "MAP_MISMATCH");
  EXPECT_EQ(events[0].details["severity"], "error");
  EXPECT_EQ(events[0].details["resume"], "user");
  EXPECT_NE(events[0].details["message"].get<std::string>().find("m apart"), std::string::npos);
  // The same datum again is nothing new.
  again.SetDatum(Moved(1000.0));
  EXPECT_TRUE(again.TakeEvents().empty());
  EXPECT_TRUE(again.TakeNotices().empty());
}

TEST(MapDatum, RefusesEveryWriteForAnotherDatumButEmptyingTheMap) {
  TempDir dir;
  StoreAppMap(dir);
  const std::string before = ReadAll(dir.path() / "map.json");
  MapService service(ConfigFor(dir));
  service.SetDatum(Moved(1000.0));

  auto answer = service.Call("map.replace", json::array({AppMap()}));
  EXPECT_FALSE(answer->ok);
  EXPECT_EQ(answer->code, -32000);
  EXPECT_NE(answer->message.find("send an empty map to start over"), std::string::npos) << answer->message;
  const auto area = service.AddArea("mow", "More", Square(2));
  EXPECT_FALSE(area.ok);
  EXPECT_NE(area.message.find("recorded for the datum"), std::string::npos) << area.message;
  EXPECT_EQ(area.code, "DATUM");
  EXPECT_FALSE(service.SetDock({0, 0}, 0).ok);
  EXPECT_EQ(service.map().areas.size(), 2u);
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), before);

  // The way out: an empty map, then a new recording that gets the datum in use.
  ASSERT_TRUE(service.Call("map.replace", json::array({json::parse(R"({"areas": [], "docking_stations": []})")}))->ok);
  EXPECT_EQ(service.status().datum, DatumState::kEmpty);
  ASSERT_TRUE(service.AddArea("mow", "New", Square(3)).ok);
  EXPECT_EQ(service.status().datum, DatumState::kMatch);
  EXPECT_TRUE(service.status().mowing_allowed);
  EXPECT_DOUBLE_EQ(json::parse(ReadAll(dir.path() / "map_datum.json"))["datum_lat"].get<double>(), Moved(1000.0).lat);
}

TEST(MapDatum, ToleratesTheNoiseOfANewSurvey) {
  TempDir dir;
  StoreAppMap(dir);
  MapService near(ConfigFor(dir));
  near.SetDatum(Moved(0.005));
  EXPECT_EQ(near.status().datum, DatumState::kMatch);
  MapService far(ConfigFor(dir));
  far.SetDatum(Moved(0.05));
  EXPECT_EQ(far.status().datum, DatumState::kMismatch);
}

TEST(MapDatum, DoesNotLookAtTheHeight) {
  TempDir dir;
  StoreAppMap(dir);
  MapService service(ConfigFor(dir));
  mowbite_map::Datum higher = kDatum;
  higher.height = 80.0;
  service.SetDatum(higher);
  EXPECT_EQ(service.status().datum, DatumState::kMatch);
}

TEST(MapDatum, AMapWithoutARecordedDatumIsNotTrusted) {
  TempDir dir;
  StoreAppMap(dir);
  fs::remove(dir.path() / "map_datum.json");
  MapService service(ConfigFor(dir));
  service.TakeNotices();
  service.SetDatum(kDatum);
  EXPECT_EQ(service.status().datum, DatumState::kUnbound);
  EXPECT_FALSE(service.status().mowing_allowed);
  EXPECT_TRUE(HasEvent(service.TakeEvents(), "unbound"));
  EXPECT_TRUE(Mentions(service.TakeNotices(), Notice::Level::kError, "The map has no recorded datum"));

  // Areas cannot be added to a map nobody can vouch for ...
  const auto area = service.AddArea("mow", "More", Square(2));
  EXPECT_FALSE(area.ok);
  EXPECT_NE(area.message.find("send it again with map.replace"), std::string::npos) << area.message;
  EXPECT_FALSE(service.SetDock({0, 0}, 0).ok);
  // ... but sending the whole map again ties it, even if it is the very same map.
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  EXPECT_EQ(service.status().datum, DatumState::kMatch);
  EXPECT_TRUE(fs::exists(dir.path() / "map_datum.json"));
}

TEST(MapDatum, ABrokenDatumFileCountsAsMissing) {
  TempDir dir;
  StoreAppMap(dir);
  WriteAll(dir.path() / "map_datum.json", "{broken");
  MapService service(ConfigFor(dir));
  EXPECT_TRUE(Mentions(service.TakeNotices(), Notice::Level::kWarn, "map_datum.json is not valid JSON"));
  service.SetDatum(kDatum);
  EXPECT_EQ(service.status().datum, DatumState::kUnbound);
}

TEST(MapDatum, TheFirstAreaTiesTheMap) {
  TempDir dir;
  MapService service = Started(NumberedIds(dir));
  ASSERT_TRUE(service.AddArea("mow", "Front", Square(2)).ok);
  EXPECT_EQ(service.status().datum, DatumState::kMatch);
  EXPECT_TRUE(fs::exists(dir.path() / "map_datum.json"));
  EXPECT_EQ(service.status().areas, 1u);
  ASSERT_TRUE(service.SetDock({1, 1}, 0).ok);
  EXPECT_EQ(service.status().docking_stations, 1u);
}

TEST(MapDatum, IgnoresAnInvalidDatum) {
  TempDir dir;
  MapService service(ConfigFor(dir));
  service.TakeNotices();
  service.SetDatum({91.0, 0.0, 0.0});
  service.SetDatum({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  EXPECT_EQ(service.status().datum, DatumState::kUnknown);
  EXPECT_TRUE(Mentions(service.TakeNotices(), Notice::Level::kError, "Ignoring an invalid datum"));
}

TEST(MapDatum, MapWithoutDatumFileStaysSafeWhenTheDatumCannotBeWritten) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  // A directory where the temporary datum file goes makes writing it fail, the map itself is written.
  fs::create_directory(dir.path() / "map_datum.json.tmp");
  const auto answer = service.Call("map.replace", json::array({AppMap()}));
  ASSERT_TRUE(answer.has_value());
  EXPECT_FALSE(answer->ok);
  EXPECT_EQ(answer->code, -32603);
  EXPECT_NE(answer->message.find("Stored the map but not its datum"), std::string::npos) << answer->message;
  EXPECT_EQ(service.map().areas.size(), 2u);
  EXPECT_EQ(service.status().datum, DatumState::kUnbound);
  EXPECT_FALSE(service.status().mowing_allowed);
}

TEST(MapReplaceChecked, StoresTheMapWhenTheVersionIsTheOneRead) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  service.TakeChanged();
  const std::string read = mowbite_map::Sha256Hex(service.map_text());

  json planned = AppMap();
  planned["areas"][0]["properties"]["planner"] = json::parse(R"({"pattern": "stripes"})");
  const auto answer = service.ReplaceChecked(planned.dump(), read);
  ASSERT_TRUE(answer.ok) << answer.message;
  EXPECT_EQ(answer.result, "Successfully stored map (2 areas)");
  EXPECT_TRUE(service.TakeChanged());
  EXPECT_EQ(json::parse(service.map_text())["areas"][0]["properties"]["planner"]["pattern"], "stripes");
  EXPECT_EQ(service.map_text(), ReadAll(dir.path() / "map.json"));
}

TEST(MapReplaceChecked, WritesNothingWhenTheMapChangedMeanwhile) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  const std::string read = mowbite_map::Sha256Hex(service.map_text());
  // The app saved a map in between.
  json app = AppMap();
  app["areas"][0]["properties"]["name"] = "Back";
  ASSERT_TRUE(service.Call("map.replace", json::array({app}))->ok);
  service.TakeChanged();
  const std::string before = service.map_text();
  const std::string previous = ReadAll(dir.path() / "map.json.bak");

  const auto answer = service.ReplaceChecked(AppMap().dump(), read);
  EXPECT_FALSE(answer.ok);
  EXPECT_EQ(answer.code, -32001);
  EXPECT_EQ(answer.message, "map changed meanwhile");
  EXPECT_FALSE(service.TakeChanged());
  EXPECT_EQ(service.map_text(), before);
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), before);
  EXPECT_EQ(ReadAll(dir.path() / "map.json.bak"), previous);
}

TEST(MapReplaceChecked, LetsOnlyTheFirstOfTwoWritersWithTheSameVersionThrough) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  const std::string read = mowbite_map::Sha256Hex(service.map_text());
  json first = AppMap();
  first["areas"][0]["properties"]["name"] = "First";
  json second = AppMap();
  second["areas"][0]["properties"]["name"] = "Second";
  EXPECT_TRUE(service.ReplaceChecked(first.dump(), read).ok);
  const auto late = service.ReplaceChecked(second.dump(), read);
  EXPECT_FALSE(late.ok);
  EXPECT_EQ(late.code, -32001);
  EXPECT_EQ(service.map().areas[0].name, "First");
}

TEST(MapReplaceChecked, SkipsTheCheckWithoutAVersion) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  json other = AppMap();
  other["areas"][0]["properties"]["name"] = "Back";
  const auto answer = service.ReplaceChecked(other.dump(), "");
  ASSERT_TRUE(answer.ok) << answer.message;
  EXPECT_EQ(service.map().areas[0].name, "Back");
}

TEST(MapReplaceChecked, TakesTheVersionOfAMapReadAfterARestart) {
  TempDir dir;
  std::string read;
  {
    MapService service = Started(ConfigFor(dir));
    ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
    read = mowbite_map::Sha256Hex(service.map_text());
  }
  MapService again = Started(ConfigFor(dir));
  EXPECT_EQ(mowbite_map::Sha256Hex(again.map_text()), read);
  EXPECT_TRUE(again.ReplaceChecked(AppMap().dump(), read).ok);
}

TEST(MapReplaceChecked, RefusesAVersionThatIsNotASha256) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  for (const std::string& version : std::vector<std::string>{"abc", std::string(64, 'A'), std::string(65, '0')}) {
    const auto answer = service.ReplaceChecked(AppMap().dump(), version);
    EXPECT_FALSE(answer.ok) << version;
    EXPECT_EQ(answer.code, -32602) << version;
    EXPECT_EQ(answer.message, "expected_sha256 is not 64 lower-case hex digits");
  }
  EXPECT_TRUE(service.map().Empty());
}

TEST(MapReplaceChecked, MakesTheChecksOfMapReplace) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  auto answer = service.ReplaceChecked("{not json", "");
  EXPECT_FALSE(answer.ok);
  EXPECT_EQ(answer.code, -32602);
  EXPECT_EQ(answer.message, "Invalid map: not valid JSON");

  json bad = AppMap();
  bad["areas"][1]["outline"] = json::parse(R"([{"x": 0, "y": 0}, {"x": 1, "y": 0}])");
  answer = service.ReplaceChecked(bad.dump(), mowbite_map::Sha256Hex(service.map_text()));
  EXPECT_FALSE(answer.ok);
  EXPECT_EQ(answer.code, -32602);
  EXPECT_EQ(answer.message, "Invalid map: areas[1] 'tree': outline needs at least 3 points");
  EXPECT_TRUE(service.map().Empty());
  EXPECT_FALSE(fs::exists(dir.path() / "map.json"));
}

TEST(MapReplaceChecked, RefusesAnotherDatumLikeMapReplace) {
  TempDir dir;
  StoreAppMap(dir);
  MapService service(ConfigFor(dir));
  service.SetDatum(Moved(1000.0));
  const auto answer = service.ReplaceChecked(AppMap().dump(), mowbite_map::Sha256Hex(service.map_text()));
  EXPECT_FALSE(answer.ok);
  EXPECT_EQ(answer.code, -32000);
  EXPECT_NE(answer.message.find("send an empty map to start over"), std::string::npos) << answer.message;
}

TEST(MapReplaceChecked, ReportsADiskThatCannotBeWritten) {
  TempDir dir;
  WriteAll(dir.path() / "file", "not a directory");
  MapService::Config config;
  config.data_dir = (dir.path() / "file" / "sub").string();
  MapService service = Started(config);
  const auto answer = service.ReplaceChecked(AppMap().dump(), "");
  EXPECT_FALSE(answer.ok);
  EXPECT_EQ(answer.code, -32603);
  EXPECT_TRUE(service.map().Empty());
}

TEST(MapServiceHold, RefusesEveryReplaceAndChangesNothing) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  ASSERT_TRUE(service.Call("map.replace", json::array({AppMap()}))->ok);
  service.TakeChanged();
  const std::string before = service.map_text();
  const std::string read = mowbite_map::Sha256Hex(before);
  service.SetHold(1800000900);
  json other = AppMap();
  other["areas"][0]["properties"]["name"] = "Back";
  const auto app = service.Call("map.replace", json::array({other}));
  ASSERT_TRUE(app.has_value());
  EXPECT_FALSE(app->ok);
  EXPECT_EQ(app->code, mowbite_map::kRpcHold);
  EXPECT_EQ(app->code, -32004);
  EXPECT_EQ(app->data, json::parse(R"({"reason": "HOLD", "until": 1800000900})"));
  const auto planner = service.ReplaceChecked(other.dump(), read);
  EXPECT_FALSE(planner.ok);
  EXPECT_EQ(planner.code, -32004);
  EXPECT_EQ(planner.data["reason"], "HOLD");
  EXPECT_FALSE(service.TakeChanged());
  EXPECT_EQ(service.map_text(), before);
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), before);
  // Reading goes on.
  EXPECT_TRUE(service.Call("map.area_properties", json())->ok);
  // After the hold the change goes through.
  service.SetHold(0);
  EXPECT_TRUE(service.ReplaceChecked(other.dump(), read).ok);
  EXPECT_TRUE(service.TakeChanged());
}

TEST(MapServiceHold, OtherErrorsCarryNoData) {
  TempDir dir;
  MapService service = Started(ConfigFor(dir));
  const auto answer = service.Call("map.replace", json::array());
  ASSERT_TRUE(answer.has_value());
  EXPECT_EQ(answer->code, mowbite_map::kRpcInvalidParams);
  EXPECT_TRUE(answer->data.is_null());
}
