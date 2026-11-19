#include <gtest/gtest.h>
#include <stdlib.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <vector>

#include "mowbite_map/map_store.h"
#include "mowbite_map/sha256.h"

using mowbite_map::json;
using mowbite_map::MapStore;

namespace fs = std::filesystem;

namespace {

// A scratch directory that goes away with the test.
class TempDir {
 public:
  TempDir() {
    std::string pattern = (fs::temp_directory_path() / "mowbite_map_test_XXXXXX").string();
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

// The text of a map with one mowing area of this id.
std::string Text(const std::string& id) {
  json j = json::parse(R"({"areas": [{"id": "x", "properties": {"type": "mow"}, "outline": [
      {"x": 0, "y": 0}, {"x": 4, "y": 0}, {"x": 4, "y": 3}]}], "docking_stations": []})");
  j["areas"][0]["id"] = id;
  return mowbite_map::ToText(*mowbite_map::ParseMap(j).map);
}

std::string ReadAll(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
}

void WriteAll(const fs::path& path, const std::string& text) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << text;
}

constexpr size_t kLimit = 100000;

}  // namespace

TEST(MapStore, FindsNothingInAnEmptyOrMissingDirectory) {
  TempDir dir;
  auto loaded = MapStore(dir.path().string(), kLimit).Load();
  EXPECT_FALSE(loaded.map.has_value());
  EXPECT_TRUE(loaded.source.empty());
  EXPECT_TRUE(loaded.problem.empty());
  loaded = MapStore((dir.path() / "missing").string(), kLimit).Load();
  EXPECT_FALSE(loaded.map.has_value());
  EXPECT_TRUE(loaded.problem.empty());
}

TEST(MapStore, SavesAndLoadsAgain) {
  TempDir dir;
  MapStore store(dir.path().string(), kLimit);
  ASSERT_EQ(store.Save(Text("a")), "");
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), Text("a"));
  const auto loaded = store.Load();
  ASSERT_TRUE(loaded.map.has_value());
  EXPECT_EQ(loaded.map->areas[0].id, "a");
  EXPECT_EQ(loaded.source, "map.json");
  EXPECT_TRUE(loaded.problem.empty());
  EXPECT_FALSE(fs::exists(dir.path() / "map.json.bak"));
}

TEST(MapStore, KeepsExactlyThePreviousVersion) {
  TempDir dir;
  MapStore store(dir.path().string(), kLimit);
  ASSERT_EQ(store.Save(Text("a")), "");
  ASSERT_EQ(store.Save(Text("b")), "");
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), Text("b"));
  EXPECT_EQ(ReadAll(dir.path() / "map.json.bak"), Text("a"));
  ASSERT_EQ(store.Save(Text("c")), "");
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), Text("c"));
  EXPECT_EQ(ReadAll(dir.path() / "map.json.bak"), Text("b"));
  EXPECT_EQ(store.Load().map->areas[0].id, "c");
}

TEST(MapStore, LeavesOnlyTheTwoFilesBehind) {
  TempDir dir;
  MapStore store(dir.path().string(), kLimit);
  ASSERT_EQ(store.Save(Text("a")), "");
  ASSERT_EQ(store.Save(Text("b")), "");
  std::vector<std::string> names;
  for (const auto& entry : fs::directory_iterator(dir.path())) names.push_back(entry.path().filename().string());
  std::sort(names.begin(), names.end());
  EXPECT_EQ(names, (std::vector<std::string>{"map.json", "map.json.bak"}));
}

TEST(MapStore, IgnoresALeftoverTemporaryFile) {
  TempDir dir;
  WriteAll(dir.path() / "map.json.tmp", "half a wri");
  MapStore store(dir.path().string(), kLimit);
  EXPECT_FALSE(store.Load().map.has_value());
  ASSERT_EQ(store.Save(Text("a")), "");
  EXPECT_FALSE(fs::exists(dir.path() / "map.json.tmp"));
  EXPECT_EQ(store.Load().map->areas[0].id, "a");
}

TEST(MapStore, RefusesAMapAboveTheLimitAndChangesNothing) {
  TempDir dir;
  const size_t limit = Text("a").size() + 20;
  MapStore store(dir.path().string(), limit);
  ASSERT_EQ(store.Save(Text("a")), "");
  const std::string error = store.Save(Text(std::string(60, 'z')));
  EXPECT_NE(error.find("the limit is " + std::to_string(limit)), std::string::npos) << error;
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), Text("a"));
  EXPECT_FALSE(fs::exists(dir.path() / "map.json.bak"));
  EXPECT_FALSE(fs::exists(dir.path() / "map.json.tmp"));
}

TEST(MapStore, FallsBackToThePreviousVersionWhenTheMapIsBroken) {
  TempDir dir;
  MapStore store(dir.path().string(), kLimit);
  ASSERT_EQ(store.Save(Text("a")), "");
  ASSERT_EQ(store.Save(Text("b")), "");
  WriteAll(dir.path() / "map.json", "{broken");
  const auto loaded = store.Load();
  ASSERT_TRUE(loaded.map.has_value());
  EXPECT_EQ(loaded.map->areas[0].id, "a");
  EXPECT_EQ(loaded.source, "map.json.bak");
  EXPECT_EQ(loaded.problem, "map.json is not valid JSON");
}

TEST(MapStore, FallsBackWhenTheMapIsNotAMap) {
  TempDir dir;
  MapStore store(dir.path().string(), kLimit);
  ASSERT_EQ(store.Save(Text("a")), "");
  ASSERT_EQ(store.Save(Text("b")), "");
  WriteAll(dir.path() / "map.json", R"({"areas": [{"id": "x", "outline": []}]})");
  const auto loaded = store.Load();
  ASSERT_TRUE(loaded.map.has_value());
  EXPECT_EQ(loaded.map->areas[0].id, "a");
  EXPECT_NE(loaded.problem.find("at least 3 points"), std::string::npos) << loaded.problem;
}

TEST(MapStore, FallsBackWhenTheMapIsMissing) {
  TempDir dir;
  MapStore store(dir.path().string(), kLimit);
  ASSERT_EQ(store.Save(Text("a")), "");
  ASSERT_EQ(store.Save(Text("b")), "");
  fs::remove(dir.path() / "map.json");
  const auto loaded = store.Load();
  ASSERT_TRUE(loaded.map.has_value());
  EXPECT_EQ(loaded.source, "map.json.bak");
  EXPECT_TRUE(loaded.problem.empty());
}

TEST(MapStore, ReportsBothFilesWhenNeitherIsUsable) {
  TempDir dir;
  WriteAll(dir.path() / "map.json", "{broken");
  WriteAll(dir.path() / "map.json.bak", "[]");
  const auto loaded = MapStore(dir.path().string(), kLimit).Load();
  EXPECT_FALSE(loaded.map.has_value());
  EXPECT_TRUE(loaded.source.empty());
  EXPECT_EQ(loaded.problem, "map.json is not valid JSON; map.json.bak: map must be an object");
}

TEST(MapStore, DoesNotReadAFileAboveTheLimit) {
  TempDir dir;
  const std::string text = Text("a");
  WriteAll(dir.path() / "map.json", text);
  const auto loaded = MapStore(dir.path().string(), text.size() - 1).Load();
  EXPECT_FALSE(loaded.map.has_value());
  EXPECT_NE(loaded.problem.find("the limit is " + std::to_string(text.size() - 1)), std::string::npos)
      << loaded.problem;
}

TEST(MapStore, DoesNotPushABrokenMapOverTheGoodBackup) {
  TempDir dir;
  MapStore store(dir.path().string(), kLimit);
  ASSERT_EQ(store.Save(Text("a")), "");
  ASSERT_EQ(store.Save(Text("b")), "");
  WriteAll(dir.path() / "map.json", "{broken");
  ASSERT_EQ(store.Save(Text("c")), "");
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), Text("c"));
  EXPECT_EQ(ReadAll(dir.path() / "map.json.bak"), Text("a"));
}

TEST(MapStore, CreatesTheDirectory) {
  TempDir dir;
  MapStore store((dir.path() / "one" / "two").string(), kLimit);
  ASSERT_EQ(store.Save(Text("a")), "");
  EXPECT_EQ(store.Load().map->areas[0].id, "a");
}

TEST(MapStore, ReportsWhyItCannotWrite) {
  TempDir dir;
  WriteAll(dir.path() / "file", "not a directory");
  MapStore store((dir.path() / "file" / "sub").string(), kLimit);
  const std::string error = store.Save(Text("a"));
  EXPECT_NE(error.find("cannot create"), std::string::npos) << error;
}

TEST(MapStoreDatum, SavesAndLoadsTheDatumExactly) {
  TempDir dir;
  MapStore store(dir.path().string(), kLimit);
  const mowbite_map::Datum datum{52.123456789012, -9.987654321098, 61.5};
  ASSERT_EQ(store.Save(Text("a")), "");
  const std::string sha = mowbite_map::Sha256Hex(Text("a"));
  ASSERT_EQ(store.SaveDatum(datum, sha), "");
  const auto loaded = store.LoadDatum();
  ASSERT_TRUE(loaded.datum.has_value());
  EXPECT_EQ(loaded.map_sha256, sha);
  EXPECT_EQ(store.Load().sha256, sha);
  EXPECT_EQ(loaded.datum->lat, datum.lat);
  EXPECT_EQ(loaded.datum->lon, datum.lon);
  EXPECT_EQ(loaded.datum->height, datum.height);
  EXPECT_TRUE(loaded.problem.empty());
  const json file = json::parse(ReadAll(dir.path() / "map_datum.json"));
  EXPECT_EQ(file["schema"], 1);
  EXPECT_FALSE(fs::exists(dir.path() / "map_datum.json.tmp"));
  // The map is not touched by it.
  EXPECT_EQ(ReadAll(dir.path() / "map.json"), Text("a"));
}

TEST(MapStoreDatum, MissingFileIsNoProblem) {
  TempDir dir;
  const auto loaded = MapStore(dir.path().string(), kLimit).LoadDatum();
  EXPECT_FALSE(loaded.datum.has_value());
  EXPECT_TRUE(loaded.problem.empty());
}

TEST(MapStoreDatum, DoesNotUseAFileItCannotTrust) {
  TempDir dir;
  MapStore store(dir.path().string(), kLimit);
  const auto problem = [&](const std::string& text) {
    WriteAll(dir.path() / "map_datum.json", text);
    const auto loaded = store.LoadDatum();
    EXPECT_FALSE(loaded.datum.has_value()) << text;
    return loaded.problem;
  };
  EXPECT_EQ(problem("{broken"), "map_datum.json is not valid JSON");
  EXPECT_EQ(problem("[1]"), "map_datum.json is not valid JSON");
  EXPECT_EQ(problem(R"({"datum_lat": 52, "datum_long": 9, "datum_height": 1})"), "map_datum.json has no schema");
  EXPECT_EQ(problem(R"({"schema": 2, "datum_lat": 52, "datum_long": 9, "datum_height": 1})"),
            "map_datum.json has the schema 2, this version reads 1");
  EXPECT_EQ(problem(R"({"schema": 1, "datum_lat": 52, "datum_long": 9})"), "map_datum.json holds no valid datum");
  EXPECT_EQ(problem(R"({"schema": 1, "datum_lat": "52", "datum_long": 9, "datum_height": 1})"),
            "map_datum.json holds no valid datum");
  EXPECT_EQ(problem(R"({"schema": 1, "datum_lat": 91, "datum_long": 9, "datum_height": 1})"),
            "map_datum.json holds no valid datum");
  EXPECT_EQ(problem(std::string(5000, ' ')), "map_datum.json is too large");
  EXPECT_EQ(problem(R"({"schema": 1, "datum_lat": 52, "datum_long": 9, "datum_height": 1, "map_sha256": "ab"})"),
            "map_datum.json holds no valid map_sha256");
}

TEST(MapStoreDatum, AFileOfAnOlderVersionHasNoMapSha) {
  TempDir dir;
  WriteAll(dir.path() / "map_datum.json", R"({"schema": 1, "datum_lat": 52, "datum_long": 9, "datum_height": 1})");
  const auto loaded = MapStore(dir.path().string(), kLimit).LoadDatum();
  ASSERT_TRUE(loaded.datum.has_value());
  EXPECT_TRUE(loaded.map_sha256.empty());
  EXPECT_TRUE(loaded.problem.empty());
}

TEST(MapStoreDatum, RefusesToWriteAnInvalidDatum) {
  TempDir dir;
  MapStore store(dir.path().string(), kLimit);
  const std::string sha = mowbite_map::Sha256Hex("a");
  EXPECT_EQ(store.SaveDatum({91.0, 0.0, 0.0}, sha), "not a valid datum");
  EXPECT_EQ(store.SaveDatum({0.0, 181.0, 0.0}, sha), "not a valid datum");
  EXPECT_EQ(store.SaveDatum({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, sha), "not a valid datum");
  EXPECT_EQ(store.SaveDatum({52.0, 9.0, 0.0}, ""), "not a SHA-256 of a map");
  EXPECT_FALSE(fs::exists(dir.path() / "map_datum.json"));
}

TEST(MapStoreDatum, ReportsWhyItCannotWrite) {
  TempDir dir;
  WriteAll(dir.path() / "file", "not a directory");
  MapStore store((dir.path() / "file" / "sub").string(), kLimit);
  EXPECT_NE(store.SaveDatum({52.0, 9.0, 0.0}, mowbite_map::Sha256Hex("a")).find("cannot create"), std::string::npos);
}

TEST(Datum, MeasuresTheDistanceOnTheGround) {
  const mowbite_map::Datum a{52.0, 9.0, 0.0};
  EXPECT_DOUBLE_EQ(mowbite_map::DistanceM(a, a), 0.0);
  // a thousandth of a degree north is 111.32 m anywhere, east it shrinks with the cosine of the latitude
  EXPECT_NEAR(mowbite_map::DistanceM(a, {52.001, 9.0, 0.0}), 111.32, 0.001);
  EXPECT_NEAR(mowbite_map::DistanceM(a, {52.0, 9.001, 0.0}), 111.32 * std::cos(52.0 * 3.14159265358979 / 180.0), 0.01);
  EXPECT_NEAR(mowbite_map::DistanceM(a, {52.001, 9.0, 99.0}), mowbite_map::DistanceM(a, {52.001, 9.0, 0.0}), 1e-9);
  EXPECT_NEAR(mowbite_map::DistanceM({52.001, 9.0, 0.0}, a), 111.32, 0.001);
}

TEST(Datum, ChecksTheRanges) {
  EXPECT_TRUE(mowbite_map::IsValid({52.0, 9.0, 50.0}));
  EXPECT_TRUE(mowbite_map::IsValid({-90.0, 180.0, -3.0}));
  EXPECT_FALSE(mowbite_map::IsValid({90.1, 0.0, 0.0}));
  EXPECT_FALSE(mowbite_map::IsValid({0.0, -180.1, 0.0}));
  EXPECT_FALSE(mowbite_map::IsValid({0.0, 0.0, std::numeric_limits<double>::infinity()}));
}
