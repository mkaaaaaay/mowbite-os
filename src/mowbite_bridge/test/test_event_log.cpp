#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>

#include "mowbite_bridge/event_log.h"

using namespace mowbite_bridge;
namespace fs = std::filesystem;

namespace {

Json Deleted(int n) {
  return Json{{"deleted", n}};
}

class EventLogTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("mowbite_eventlog_" + std::to_string(::getpid()) + "_" +
                                         ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
  }
  void TearDown() override {
    fs::remove_all(dir_);
  }
  void OldDay(const std::string& day, const std::string& line = R"({"type":"OLD"})") {
    fs::create_directories(dir_);
    std::ofstream(dir_ / (day + ".jsonl")) << line << "\n";
  }
  static Json Event(const std::string& type) {
    return Json{{"id", "i"}, {"t", 1.0}, {"type", type}};
  }

  fs::path dir_;
  // 2026-11-02 12:00 UTC.
  double now_ = 1793620800.0;
};

}  // namespace

// B18
TEST_F(EventLogTest, OneFilePerDayReadInFileOrder) {
  EventLog log(dir_.string(), 60, 20000000);
  log.Write(Event("BOOTED"), now_);
  log.Write(Json{{"id", "j"}, {"t", 2.0}, {"type", "STATE"}, {"state", "IDLE"}, {"x", 0.5}}, now_ + 10);
  log.Write(Json::array({1}), now_);
  const std::string today = EventLog::DayOf(now_);
  EXPECT_TRUE(fs::exists(dir_ / (today + ".jsonl")));
  const Json events = log.Read(nullptr, now_);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0]["type"], "BOOTED");
  EXPECT_EQ(events[1].dump(), R"({"id":"j","t":2.0,"type":"STATE","state":"IDLE","x":0.5})");
  EXPECT_EQ(log.Read(Json{{"date", today}}, now_), events);
  EXPECT_EQ(log.Read(Json::array({today}), now_), events);
  EXPECT_EQ(log.Read(Json::object(), now_), events);
}

TEST_F(EventLogTest, UnknownOrInvalidDaysGiveNothing) {
  EventLog log(dir_.string(), 60, 20000000);
  log.Write(Event("BOOTED"), now_);
  for (const Json& p : {Json{{"date", "20200101"}}, Json{{"date", "../etc"}}, Json{{"date", "2026110"}},
                        Json{{"date", 20261102}}, Json::array({5})}) {
    EXPECT_EQ(log.Read(p, now_), Json::array()) << p.dump();
  }
}

TEST_F(EventLogTest, LinesThatAreNoObjectAreLeftOut) {
  OldDay("20200101", "not json");
  std::ofstream(dir_ / "20200101.jsonl", std::ios::app)
      << "[1]\n" << R"({"type":"A"})" << "\n" << std::string(100, '[') << "\n";
  EventLog log(dir_.string(), 60, 20000000);
  const Json events = log.Read(Json{{"date", "20200101"}}, now_);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0]["type"], "A");
}

TEST_F(EventLogTest, DayOfTheLocalTime) {
  const char* before = std::getenv("TZ");
  const std::string saved = before ? before : "";
  ::setenv("TZ", "Pacific/Kiritimati", 1);
  ::tzset();
  // 2026-11-02 12:00 UTC is already 3 November at UTC+14.
  EXPECT_EQ(EventLog::DayOf(now_), "20261103");
  ::setenv("TZ", "UTC", 1);
  ::tzset();
  EXPECT_EQ(EventLog::DayOf(now_), "20261102");
  if (before) {
    ::setenv("TZ", saved.c_str(), 1);
  } else {
    ::unsetenv("TZ");
  }
  ::tzset();
}

TEST_F(EventLogTest, ListNewestFirstAndDelete) {
  OldDay("20200101");
  OldDay("20200103");
  EventLog log(dir_.string(), 60, 20000000);
  log.Write(Event("BOOTED"), now_);
  const std::string today = EventLog::DayOf(now_);
  EXPECT_EQ(log.List(), Json::array({today, "20200103", "20200101"}));
  EXPECT_EQ(log.Erase(Json{{"date", "20200101"}}), Deleted(1));
  EXPECT_EQ(log.Erase(Json{{"date", "20200101"}}), Deleted(0));
  EXPECT_EQ(log.Erase(Json{{"date", "../x"}}), Deleted(0));
  EXPECT_EQ(log.Erase(nullptr), Deleted(2));
  EXPECT_EQ(log.List(), Json::array());
  // Today starts again with the next event.
  log.Write(Event("X"), now_);
  EXPECT_EQ(log.List(), Json::array({today}));
}

TEST_F(EventLogTest, OldestDaysGoBeyondTheDayCount) {
  for (const char* day : {"20200101", "20200102", "20200103", "20200104"}) OldDay(day);
  EventLog log(dir_.string(), 3, 20000000);
  log.Write(Event("BOOTED"), now_);
  EXPECT_EQ(log.List(), Json::array({EventLog::DayOf(now_), "20200104", "20200103"}));
}

TEST_F(EventLogTest, OldestDaysGoFirstOverTheByteLimitAndTodayStopsThere) {
  OldDay("20200101", std::string(60, ' ') + R"({"type":"OLD"})");
  OldDay("20200102", R"({"type":"OLD"})");
  EventLog log(dir_.string(), 60, 200);
  const std::string today = EventLog::DayOf(now_);
  for (int i = 0; i < 50; ++i) log.Write(Event("FILLER"), now_);
  std::uintmax_t total = 0;
  for (const auto& e : fs::directory_iterator(dir_)) total += e.file_size();
  EXPECT_LE(total, 200u);
  EXPECT_FALSE(fs::exists(dir_ / "20200101.jsonl"));
  EXPECT_GT(fs::file_size(dir_ / (today + ".jsonl")), 100u);
  EXPECT_LE(log.Read(nullptr, now_).size(), 6u);
}

TEST_F(EventLogTest, WithoutADirectoryNothingIsKept) {
  EventLog log("", 60, 20000000);
  log.Write(Event("BOOTED"), now_);
  EXPECT_EQ(log.Read(nullptr, now_), Json::array());
  EXPECT_EQ(log.List(), Json::array());
  EXPECT_EQ(log.Erase(nullptr), Deleted(0));
}
