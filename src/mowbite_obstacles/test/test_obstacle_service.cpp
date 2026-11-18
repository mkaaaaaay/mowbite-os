#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "mowbite_obstacles/obstacle_service.h"

namespace mowbite_obstacles {
namespace {

namespace fs = std::filesystem;

class TempDir {
 public:
  TempDir() {
    path_ = fs::temp_directory_path() /
            ("obstacles_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
             ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(path_);
  }
  ~TempDir() { fs::remove_all(path_); }
  std::string path() const { return path_.string(); }

 private:
  fs::path path_;
};

std::string ReadFile(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(file), {});
}

Frame At(double stamp, std::vector<Find> finds = {}) {
  Frame frame;
  frame.source = "stereo";
  frame.stamp = stamp;
  frame.sight = true;
  frame.view = {0.466, 0.0, 0.75, -0.75, 0.4, 4.0};
  frame.finds = std::move(finds);
  return frame;
}

Find Ball(double x = 2.0) {
  Find find;
  find.kind = Kind::kObject;
  find.class_name = "ball";
  find.score = 0.9;
  find.x = x;
  find.radius = 0.1;
  find.box_x = 300;
  find.box_y = 200;
  find.box_w = 20;
  find.box_h = 20;
  return find;
}

class ServiceTest : public ::testing::Test {
 protected:
  ServiceTest() { Make(); }

  void Make() {
    ServiceConfig config;
    config.dir = (fs::path(dir.path()) / "obstacles").string();
    service = std::make_unique<ObstacleService>(config, [this](const std::string& text) { warnings.push_back(text); });
    service->Load(1.0e9);
    tracker = Tracker();
    tracker.SetSources({"stereo"});
  }

  void Feed(double t, std::vector<Find> finds = {}) {
    tracker.OnPose({t, 0.0, 0.0, 0.0, 0.0});
    service->OnFrame(tracker, tracker.OnFrame(At(t, std::move(finds)), t), t);
  }

  Json Call(const std::string& method, const Json& params = Json::object()) {
    const Outcome outcome = service->Call(method, params);
    if (!outcome.ok) return Json{{"error", outcome.code}, {"message", outcome.message}};
    return outcome.result;
  }

  std::vector<std::string> Types() {
    std::vector<std::string> types;
    for (const auto& [type, details] : service->TakeEvents()) types.push_back(type);
    return types;
  }

  TempDir dir;
  std::vector<std::string> warnings;
  std::unique_ptr<ObstacleService> service;
  Tracker tracker;
};

constexpr double kT = 1.0e9;

TEST_F(ServiceTest, OffKeepsNothing) {
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  EXPECT_EQ(Call("vision.obstacles.list")["obstacles"].size(), 0u);
  EXPECT_TRUE(Types().empty());
  EXPECT_FALSE(service->PhotosWanted());
}

TEST_F(ServiceTest, ShowListsAConfirmedThingAndTellsOnce) {
  service->SetMode(1);
  Feed(kT, {Ball()});
  EXPECT_TRUE(Types().empty());
  Feed(kT + 0.2, {Ball()});
  Feed(kT + 0.4, {Ball()});
  EXPECT_EQ(Types(), std::vector<std::string>{"OBSTACLE_SEEN"});
  const Json list = Call("vision.obstacles.list");
  EXPECT_EQ(list["mode"], "show");
  EXPECT_EQ(list["photos"], false);
  EXPECT_EQ(list["limit"]["entries"], 200);
  ASSERT_EQ(list["obstacles"].size(), 1u);
  EXPECT_EQ(list["obstacles"][0]["kind"], "OBJECT");
  EXPECT_DOUBLE_EQ(list["obstacles"][0]["last_t"].get<double>(), kT + 0.4);
}

TEST_F(ServiceTest, TheModeIsAtMostThatOfThePlatform) {
  service->SetMode(3);
  EXPECT_EQ(service->mode(), Mode::kShow);
  service->SetModeMax(2);
  EXPECT_EQ(service->mode(), Mode::kStop);
  const Json list = Call("vision.obstacles.list");
  EXPECT_EQ(list["mode"], "stop");
  EXPECT_EQ(list["mode_max"], "stop");
  service->SetModeMax(3);
  EXPECT_EQ(service->mode(), Mode::kAvoid);
  EXPECT_EQ(Call("vision.obstacles.list")["mode_max"], "avoid");
  service->SetModeMax(0);
  EXPECT_EQ(service->mode(), Mode::kOff);
  service->SetModeMax(1);
  service->SetMode(0);
  EXPECT_EQ(service->mode(), Mode::kOff);
}

TEST_F(ServiceTest, AnEntryKeepsTheMostTheMowerDidAboutIt) {
  service->SetMode(1);
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  const uint32_t track = tracker.tracks().begin()->first;
  service->OnAction(track, "would_stop");
  EXPECT_EQ(Call("vision.obstacles.list")["obstacles"][0]["action"], "would_stop");
  service->OnAction(track, "slowed");
  service->OnAction(track, "no_such_action");
  service->OnAction(track + 99, "stopped");
  EXPECT_EQ(Call("vision.obstacles.list")["obstacles"][0]["action"], "would_stop");
  service->OnAction(track, "stopped");
  EXPECT_EQ(Call("vision.obstacles.list")["obstacles"][0]["action"], "stopped");
}

TEST_F(ServiceTest, ACircleDrivenAroundStandsInTheEntryUntilItGoes) {
  service->SetMode(1);
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  const uint32_t track = tracker.tracks().begin()->first;
  Types();
  EXPECT_TRUE(Call("vision.obstacles.list")["obstacles"][0]["keep_out_r"].is_null());
  service->OnAction(track, "avoided", 0.5, 1.25, "lawn", "Lawn");
  Json entry = Call("vision.obstacles.list")["obstacles"][0];
  EXPECT_EQ(entry["action"], "avoided");
  EXPECT_EQ(entry["keep_out_r"], 0.5);
  EXPECT_EQ(entry["left_out"], 1.25);
  const auto events = service->TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].first, "OBSTACLE_AVOIDED");
  EXPECT_EQ(events[0].second["obstacle_id"], entry["id"]);
  EXPECT_EQ(events[0].second["area_id"], "lawn");
  EXPECT_EQ(events[0].second["keep_out_r"], 0.5);
  EXPECT_EQ(events[0].second["x"], entry["x"]);
  EXPECT_EQ(events[0].second["severity"], "info");
  // The track ended; the circle goes all the same, without an event.
  service->OnEnded({track});
  service->OnAction(track, "avoided");
  entry = Call("vision.obstacles.list")["obstacles"][0];
  EXPECT_TRUE(entry["keep_out_r"].is_null());
  EXPECT_TRUE(entry["left_out"].is_null());
  EXPECT_EQ(entry["action"], "avoided");
  EXPECT_TRUE(Types().empty());
}

TEST_F(ServiceTest, ThePhotoOfTheFrameComesWithTheEventAndStays) {
  service->SetMode(1);
  service->SetPhotos(true);
  EXPECT_TRUE(service->PhotosWanted());
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  EXPECT_TRUE(Types().empty()) << "OBSTACLE_SEEN waits for the photo";
  const std::string jpeg = "\xff\xd8 a jpeg \xff\xd9";
  service->OnImage(tracker, {"stereo", kT + 0.2, 640, 480, jpeg});
  auto events = service->TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].second["image"], true);
  EXPECT_EQ(events[0].second["severity"], "info");
  EXPECT_EQ(events[0].second["resume"], "auto");
  const Json entry = Call("vision.obstacles.list")["obstacles"][0];
  ASSERT_TRUE(entry["image"].is_object());
  EXPECT_EQ(entry["image"]["name"], "o1.jpg");
  EXPECT_EQ(entry["image"]["bytes"], jpeg.size());
  EXPECT_EQ(entry["image"]["box"]["w"], 20.0);
  EXPECT_EQ(ReadFile(fs::path(dir.path()) / "obstacles" / "o1.jpg"), jpeg);

  const Json piece = Call("vision.obstacles.image", {{"name", "o1.jpg"}, {"offset", 2}, {"size", 3}});
  EXPECT_EQ(piece["size"], jpeg.size());
  EXPECT_EQ(piece["offset"], 2);
  EXPECT_EQ(piece["data"], "IGEg");
  EXPECT_EQ(piece["type"], "image/jpeg");
  EXPECT_EQ(piece["sha256"], entry["image"]["sha256"]);
  EXPECT_EQ(Call("vision.obstacles.image", {{"name", "o1.jpg"}, {"offset", jpeg.size()}})["data"], "");
}

TEST_F(ServiceTest, WithoutAPhotoTheEventComesLate) {
  service->SetMode(1);
  service->SetPhotos(true);
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  service->Tick(kT + 1.0);
  EXPECT_TRUE(Types().empty());
  service->Tick(kT + 2.3);
  auto events = service->TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].second["image"], false);
}

TEST_F(ServiceTest, AnImageOfAnotherFrameIsNoPhoto) {
  service->SetMode(1);
  service->SetPhotos(true);
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  service->OnImage(tracker, {"stereo", kT + 0.1, 640, 480, "x"});
  service->OnImage(tracker, {"rear", kT + 0.2, 640, 480, "x"});
  EXPECT_TRUE(Call("vision.obstacles.list")["obstacles"][0]["image"].is_null());
}

TEST_F(ServiceTest, ImagesAreTakenOnlyWhileWanted) {
  service->SetMode(1);
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  service->OnImage(tracker, {"stereo", kT + 0.2, 640, 480, "x"});
  EXPECT_TRUE(Call("vision.obstacles.list")["obstacles"][0]["image"].is_null());
  EXPECT_FALSE(fs::exists(fs::path(dir.path()) / "obstacles" / "o1.jpg"));
}

TEST_F(ServiceTest, TheListOutlivesARestartAndStrayPhotosGo) {
  service->SetMode(1);
  service->SetPhotos(true);
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  service->OnImage(tracker, {"stereo", kT + 0.2, 640, 480, "jpeg"});
  service->Flush();
  std::ofstream(fs::path(dir.path()) / "obstacles" / "stray.jpg") << "x";
  const Json before = Call("vision.obstacles.list")["obstacles"];
  Make();
  EXPECT_EQ(Call("vision.obstacles.list")["obstacles"], before);
  EXPECT_FALSE(fs::exists(fs::path(dir.path()) / "obstacles" / "stray.jpg"));
  EXPECT_TRUE(fs::exists(fs::path(dir.path()) / "obstacles" / "o1.jpg"));
}

TEST_F(ServiceTest, AnEntryWhosePhotoIsGoneLosesIt) {
  service->SetMode(1);
  service->SetPhotos(true);
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  service->OnImage(tracker, {"stereo", kT + 0.2, 640, 480, "jpeg"});
  service->Flush();
  fs::remove(fs::path(dir.path()) / "obstacles" / "o1.jpg");
  Make();
  const Json list = Call("vision.obstacles.list")["obstacles"];
  ASSERT_EQ(list.size(), 1u);
  EXPECT_TRUE(list[0]["image"].is_null());
}

TEST_F(ServiceTest, ClearRemovesEntriesWithTheirPhotos) {
  service->SetMode(1);
  service->SetPhotos(true);
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  service->OnImage(tracker, {"stereo", kT + 0.2, 640, 480, "jpeg"});
  service->TakeEvents();
  EXPECT_EQ(Call("vision.obstacles.clear", {{"id", 5}})["error"], -32602);
  EXPECT_EQ(Call("vision.obstacles.clear", {{"id", "1"}})["error"], -32602);
  EXPECT_EQ(Call("vision.obstacles.clear", {{"id", 1}})["removed"], 1);
  auto events = service->TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].first, "OBSTACLES_CLEARED");
  EXPECT_EQ(events[0].second["obstacle_id"], 1);
  EXPECT_FALSE(fs::exists(fs::path(dir.path()) / "obstacles" / "o1.jpg"));
  EXPECT_EQ(Call("vision.obstacles.image", {{"name", "o1.jpg"}})["error"], -32602);
  EXPECT_EQ(Call("vision.obstacles.clear")["removed"], 0);
  EXPECT_TRUE(service->TakeEvents()[0].second["obstacle_id"].is_null());
}

TEST_F(ServiceTest, BadParametersAreRefused) {
  EXPECT_EQ(Call("vision.obstacles.list", {{"since", "x"}})["error"], -32602);
  EXPECT_EQ(Call("vision.obstacles.image", {{"name", "../list.json"}})["error"], -32602);
  EXPECT_EQ(Call("vision.obstacles.image", Json::object())["error"], -32602);
  EXPECT_EQ(Call("camera.list")["error"], -32601);
}

TEST_F(ServiceTest, TheSightIsToldOnceItSettles) {
  service->SetMode(1);
  service->SetSight(false, "TOO_DARK", "stereo", kT);
  service->Tick(kT + 1.0);
  EXPECT_TRUE(Types().empty());
  EXPECT_EQ(Call("vision.obstacles.list")["sight"]["reason"], "TOO_DARK");
  service->Tick(kT + 2.1);
  auto events = service->TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].first, "OBSTACLE_SIGHT");
  EXPECT_EQ(events[0].second["ok"], false);
  EXPECT_EQ(events[0].second["reason"], "TOO_DARK");
  // A short flicker back is not told.
  service->SetSight(true, "", "", kT + 3.0);
  service->SetSight(false, "TOO_DARK", "stereo", kT + 3.5);
  service->Tick(kT + 6.0);
  EXPECT_TRUE(Types().empty());
}

TEST_F(ServiceTest, ANewEntryIsWrittenSoonAFollowingOneSeldom) {
  service->SetMode(1);
  const fs::path file = fs::path(dir.path()) / "obstacles" / "list.json";
  Feed(kT, {Ball()});
  Feed(kT + 0.2, {Ball()});
  service->Tick(kT + 0.3);
  const std::string first = ReadFile(file);
  EXPECT_NE(first.find("\"ball\""), std::string::npos);
  // The entry only follows its track: kept after a minute, not after five seconds.
  Feed(kT + 0.4, {Ball(2.3)});
  service->Tick(kT + 5.4);
  EXPECT_EQ(ReadFile(file), first);
  service->Tick(kT + 60.4);
  const std::string followed = ReadFile(file);
  EXPECT_NE(followed, first);
  // A new entry again after five seconds.
  Feed(kT + 60.6, {Ball(4.0)});
  Feed(kT + 60.8, {Ball(4.0)});
  service->Tick(kT + 61.0);
  EXPECT_EQ(ReadFile(file), followed);
  service->Tick(kT + 65.5);
  EXPECT_NE(ReadFile(file), followed);
}

}  // namespace
}  // namespace mowbite_obstacles
