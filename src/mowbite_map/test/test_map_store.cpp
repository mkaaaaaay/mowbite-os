#include <gtest/gtest.h>
#include <stdlib.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

#include "mowbite_map/map_store.h"

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
