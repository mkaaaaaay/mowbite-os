#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

#include "mowbite_bridge/params_store.h"
#include "mowbite_bridge/position_history.h"
#include "mowbite_bridge/position_median.h"

using namespace mowbite_bridge;

TEST(Median, EmptyGivesNothing) {
  EXPECT_FALSE(MedianPose({}).has_value());
}

TEST(Median, TakesMiddleValueOfEachAxisSeparately) {
  auto m = MedianPose({{1, 30, 0.1}, {3, 10, 0.3}, {2, 20, 0.2}});
  ASSERT_TRUE(m);
  EXPECT_DOUBLE_EQ(m->x, 2);
  EXPECT_DOUBLE_EQ(m->y, 20);
  EXPECT_NEAR(m->heading, 0.2, 1e-12);
}

TEST(Median, EvenCountTakesUpperMiddle) {
  auto m = MedianPose({{1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}});
  EXPECT_DOUBLE_EQ(m->x, 3);
}

TEST(Median, HeadingAcrossPiStaysNearPi) {
  const double e = 0.02;
  auto m = MedianPose({{0, 0, M_PI - e}, {0, 0, -M_PI + e}, {0, 0, M_PI - e / 2}});
  EXPECT_NEAR(std::fabs(m->heading), M_PI, e);
}

TEST(Median, SingleSampleIsReturnedAsIs) {
  auto m = MedianPose({{1.5, -2.5, -3.0}});
  EXPECT_DOUBLE_EQ(m->x, 1.5);
  EXPECT_DOUBLE_EQ(m->y, -2.5);
  EXPECT_NEAR(m->heading, -3.0, 1e-12);
}

TEST(Params, EmptyByDefault) {
  ParamsStore store;
  EXPECT_EQ(store.Payload(), "{}");
}

TEST(Params, NodeNamePrefixesAndKeysAreSorted) {
  ParamsStore store;
  EXPECT_TRUE(store.Update(R"({"node":"/xbot_positioning","params":{"antenna_offset_y":0.0,"antenna_offset_x":0.18}})"));
  EXPECT_TRUE(store.Update(R"({"node":"tron_gnss/","params":{"datum_lat":52.5,"datum_long":9.5}})"));
  EXPECT_EQ(store.Payload(),
            R"({"/tron_gnss/datum_lat":52.5,"/tron_gnss/datum_long":9.5,)"
            R"("/xbot_positioning/antenna_offset_x":0.18,"/xbot_positioning/antenna_offset_y":0.0})");
}

TEST(Params, SameValuesAgainAreNoChange) {
  ParamsStore store;
  const std::string msg = R"({"node":"/n","params":{"a":1}})";
  EXPECT_TRUE(store.Update(msg));
  EXPECT_FALSE(store.Update(msg));
  EXPECT_TRUE(store.Update(R"({"node":"/n","params":{"a":2}})"));
}

TEST(Params, PasswordsAreNull) {
  ParamsStore store;
  store.Update(R"({"node":"/ntrip","params":{"password":"geheim","user":"x"}})");
  const auto j = json::parse(store.Payload());
  EXPECT_TRUE(j["/ntrip/password"].is_null());
  EXPECT_EQ(j["/ntrip/user"], "x");
  EXPECT_EQ(store.Payload().find("geheim"), std::string::npos);
}

TEST(Params, MalformedMessagesChangeNothing) {
  ParamsStore store(json{{"/a/b", 1}});
  for (const char* msg : {"nope", "[]", R"({"node":"/n"})", R"({"params":{}})", R"({"node":5,"params":{}})",
                          R"({"node":"/n","params":[1]})"}) {
    EXPECT_FALSE(store.Update(msg)) << msg;
  }
  EXPECT_EQ(store.Payload(), R"({"/a/b":1})");
}

TEST(Params, BaseObjectIsKept) {
  ParamsStore store(json{{"/mower_logic/x", 1}});
  store.Update(R"({"node":"/n","params":{"a":true}})");
  EXPECT_EQ(store.Payload(), R"({"/mower_logic/x":1,"/n/a":true})");
}

namespace {

class PositionHistoryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() / ("mowbite_ph_" + std::to_string(::getpid()) + "_" +
                                                       ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::remove_all(dir_);
  }
  void TearDown() override {
    std::filesystem::remove_all(dir_);
  }

  void Init(PositionHistory& h, size_t days = 60, uintmax_t bytes = 50000000) {
    h.init(dir_.string(), [this] { return now_; }, days, bytes);
  }

  static json State(const std::string& job, const std::string& session) {
    return {{"type", "STATE"}, {"job_id", job}, {"session_id", session}};
  }

  // A straight line of points 2 cm apart.
  static void Drive(PositionHistory& h, int count, double x0 = 0) {
    for (int i = 0; i < count; ++i) h.addPoint(x0 + 0.02 * i, 0);
  }

  size_t Files() const {
    size_t n = 0;
    for (const auto& e : std::filesystem::directory_iterator(dir_)) n += e.is_regular_file();
    return n;
  }

  std::filesystem::path dir_;
  double now_ = 1760000000;
};

}  // namespace

TEST_F(PositionHistoryTest, NothingRecordedWithoutSession) {
  PositionHistory h;
  Init(h);
  Drive(h, 100);
  EXPECT_TRUE(h.getHistory()["buffer"].empty());
  EXPECT_TRUE(h.getHistory()["segments"].empty());
  EXPECT_EQ(Files(), 0u);
}

TEST_F(PositionHistoryTest, RecordsWithSessionAndReportsAttributes) {
  PositionHistory h;
  Init(h);
  h.onEvent(State("job1", "s1"));
  Drive(h, 5);
  EXPECT_EQ(h.getHistory()["buffer"].size(), 5u);
  const auto attrs = json(h.getAttributes());
  EXPECT_EQ(attrs["job_id"], "job1");
  EXPECT_EQ(attrs["session_id"], "s1");
  EXPECT_EQ(attrs["blades"], false);
}

TEST_F(PositionHistoryTest, CompactionKeepsRecentContextAndSimplifiesStraightLine) {
  PositionHistory h;
  Init(h);
  h.onEvent(State("job1", "s1"));
  Drive(h, 61);
  const auto j = h.getHistory();
  EXPECT_EQ(j["buffer"].size(), 10u);
  ASSERT_EQ(j["segments"].size(), 1u);
  // 51 straight points shrink to the first; the end of the window is context and not committed yet.
  EXPECT_EQ(j["segments"][0]["points"].size(), 1u);
}

TEST_F(PositionHistoryTest, CornerSurvivesSimplification) {
  PositionHistory h;
  Init(h);
  h.onEvent(State("job1", "s1"));
  for (int i = 0; i < 30; ++i) h.addPoint(0.05 * i, 0);
  for (int i = 1; i <= 40; ++i) h.addPoint(1.45, 0.05 * i);
  h.flush();
  // Decimation may drop the exact corner point, a vertex next to it has to stay.
  const auto points = h.getHistory()["segments"][0]["points"];
  bool corner = false;
  for (const auto& p : points) {
    if (std::fabs(p[0].get<double>() - 1.45) < 0.11 && std::fabs(p[1].get<double>()) < 0.11) corner = true;
  }
  EXPECT_TRUE(corner);
  EXPECT_LT(points.size(), 12u);
}

TEST_F(PositionHistoryTest, BladesChangeOpensBridgedSegment) {
  PositionHistory h;
  Init(h);
  h.onEvent(State("job1", "s1"));
  Drive(h, 20);
  h.onEvent({{"type", "BLADES"}, {"enabled", true}});
  Drive(h, 20, 1.0);
  h.flush();
  const auto segs = h.getHistory()["segments"];
  ASSERT_EQ(segs.size(), 2u);
  EXPECT_EQ(segs[0]["attributes"]["blades"], false);
  EXPECT_EQ(segs[1]["attributes"]["blades"], true);
  EXPECT_EQ(segs[1]["points"][0], segs[0]["points"].back());
}

TEST_F(PositionHistoryTest, WritesFileOnFlushAndReloadsIt) {
  {
    PositionHistory h;
    Init(h);
    h.onEvent(State("job1", "s1"));
    Drive(h, 20);
    h.flush();
    EXPECT_EQ(Files(), 1u);
    const auto list = h.listHistories();
    ASSERT_EQ(list.size(), 1u);
    EXPECT_EQ(list[0]["job_id"], "job1");
    EXPECT_EQ(list[0]["timestamp"], 1760000000);
  }
  PositionHistory again;
  Init(again);
  const auto j = again.getHistory();
  // The loaded segment plus an empty one for the points after the boot.
  ASSERT_EQ(j["segments"].size(), 2u);
  EXPECT_FALSE(j["segments"][0]["points"].empty());
  EXPECT_TRUE(j["segments"][1]["points"].empty());
  EXPECT_EQ(json(again.getAttributes())["job_id"], "job1");
  // session_id comes only from a new STATE event, so the docked mower records nothing.
  EXPECT_EQ(json(again.getAttributes())["session_id"], "");
  again.addPoint(5, 5);
  EXPECT_TRUE(again.getHistory()["buffer"].empty());
}

TEST_F(PositionHistoryTest, AppendsToTheSameFileAcrossFlushes) {
  PositionHistory h;
  Init(h);
  h.onEvent(State("job1", "s1"));
  Drive(h, 70);
  h.periodicFlush();
  Drive(h, 70, 3.0);
  h.flush();
  EXPECT_EQ(Files(), 1u);
  PositionHistory again;
  Init(again);
  size_t points = 0;
  for (const auto& s : again.getHistory("job1")["segments"]) points += s["points"].size();
  size_t live = 0;
  for (const auto& s : h.getHistory()["segments"]) live += s["points"].size();
  EXPECT_EQ(points, live);
}

TEST_F(PositionHistoryTest, NewJobOpensNewFile) {
  PositionHistory h;
  Init(h);
  h.onEvent(State("job1", "s1"));
  Drive(h, 20);
  now_ += 100;
  h.onEvent(State("job2", "s2"));
  Drive(h, 20);
  h.flush();
  EXPECT_EQ(Files(), 2u);
  const auto list = h.listHistories();
  ASSERT_EQ(list.size(), 2u);
  EXPECT_EQ(list[0]["job_id"], "job2");
  EXPECT_EQ(list[1]["job_id"], "job1");
  EXPECT_FALSE(h.getHistory("job1")["segments"].empty());
  EXPECT_TRUE(h.getHistory("nope").contains("error"));
}

TEST_F(PositionHistoryTest, DeleteSkipsTheActiveJob) {
  PositionHistory h;
  Init(h);
  h.onEvent(State("job1", "s1"));
  Drive(h, 20);
  now_ += 100;
  h.onEvent(State("job2", "s2"));
  Drive(h, 20);
  h.flush();
  EXPECT_EQ(h.deleteHistory(std::string("job2"))["deleted"], 0);
  EXPECT_EQ(h.deleteHistory(std::nullopt)["deleted"], 1);
  EXPECT_EQ(Files(), 1u);
  EXPECT_EQ(h.listHistories().size(), 1u);
}

TEST_F(PositionHistoryTest, OldFilesAreRemovedAfterMaxDays) {
  for (const char* job : {"old", "newer"}) {
    PositionHistory h;
    Init(h);
    h.onEvent(State(job, "s"));
    Drive(h, 20);
    h.flush();
    now_ += 2 * 86400;
  }
  now_ += 86400;
  PositionHistory h;
  Init(h, 2);
  // The latest file counts as the active job after a start and stays.
  ASSERT_EQ(Files(), 1u);
  EXPECT_EQ(h.listHistories()[0]["job_id"], "newer");
}

TEST_F(PositionHistoryTest, OldestFilesGoWhenOverTheByteLimit) {
  for (int job = 0; job < 4; ++job) {
    PositionHistory h;
    Init(h);
    h.onEvent(State("job" + std::to_string(job), "s"));
    Drive(h, 40);
    h.flush();
    now_ += 10;
  }
  ASSERT_EQ(Files(), 4u);
  uintmax_t one = 0;
  for (const auto& e : std::filesystem::directory_iterator(dir_)) one = std::max(one, e.file_size());
  PositionHistory h;
  Init(h, 60, one * 2 + one / 2);
  EXPECT_LE(Files(), 3u);
  EXPECT_GE(Files(), 1u);
  // Newest first: the newest survive.
  const auto list = h.listHistories();
  EXPECT_EQ(list[0]["job_id"], "job3");
}

TEST_F(PositionHistoryTest, MalformedEventsAndFilesAreIgnored) {
  PositionHistory h;
  Init(h);
  h.onEvent({{"type", "BLADES"}});
  h.onEvent({{"nope", 1}});
  h.onEvent(json::array());
  std::ofstream(dir_ / "garbage.jsonl") << "x\n";
  std::ofstream(dir_ / "12_job.jsonl") << "{broken\n{\"type\":\"new_segment\"}\n";
  PositionHistory again;
  Init(again);
  EXPECT_NO_THROW(again.getHistory("job"));
}
