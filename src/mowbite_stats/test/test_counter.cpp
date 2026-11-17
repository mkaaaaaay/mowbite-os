#include <gtest/gtest.h>

#include "mowbite_api/catalog.h"
#include "mowbite_stats/counter.h"

using namespace mowbite_stats;

namespace {

// 2026-11-13 10:00:00 UTC, a whole hour.
constexpr double kHour = 1794564000;
// The first hour of a set clock.
constexpr int64_t kFirstHour = static_cast<int64_t>(kEarliestWall);

Sample At(double s, const std::string& state, bool blade, double x, double y = 0, bool pose = true) {
  Sample sample;
  sample.wall = kHour + s;
  sample.mono = 1000 + s;
  sample.state = state;
  sample.blade = blade;
  sample.pose = pose;
  sample.x = x;
  sample.y = y;
  return sample;
}

Counter Fresh() {
  return Counter(nullptr, nullptr, nullptr, kHour);
}

}  // namespace

TEST(Counter, BladeTimeAndDistanceWithAndWithout) {
  Counter c = Fresh();
  c.Add(At(0, "MOWING", true, 0));
  c.Add(At(1, "MOWING", true, 0.3));
  c.Add(At(2, "MOWING", false, 0.6));
  c.Add(At(3, "DOCKING", false, 0.9));
  EXPECT_DOUBLE_EQ(c.total().blade_s, 2);
  EXPECT_NEAR(c.total().mowed_m, 0.6, 1e-9);
  EXPECT_NEAR(c.total().driven_m, 0.3, 1e-9);
}

TEST(Counter, GapsJumpsAndIdleDoNotCount) {
  Counter c = Fresh();
  c.Add(At(0, "MOWING", true, 0));
  c.Add(At(10, "MOWING", true, 1));   // a gap of the stream
  c.Add(At(11, "MOWING", true, 4));   // a jump of the position
  c.Add(At(12, "MOWING", true, 4, 0, false));
  c.Add(At(13, "MOWING", true, 4.2));  // no pose before
  c.Add(At(14, "IDLE", false, 4.4));
  c.Add(At(15, "IDLE", false, 4.6));  // pushed by hand in IDLE
  EXPECT_DOUBLE_EQ(c.total().blade_s, 4);
  EXPECT_NEAR(c.total().mowed_m, 0.2, 1e-9);
  EXPECT_DOUBLE_EQ(c.total().driven_m, 0);
}

TEST(Counter, ARunLeavesIdle) {
  Counter c = Fresh();
  c.Add(At(0, "IDLE", false, 0));
  c.Add(At(1, "UNDOCKING", false, 0));
  c.Add(At(2, "MOWING", true, 0));
  c.Add(At(3, "PAUSED", false, 0));
  c.Add(At(4, "MOWING", true, 0));
  c.Add(At(5, "IDLE", false, 0));
  c.Add(At(6, "MOWING", true, 0));
  EXPECT_EQ(c.total().runs, 2);
}

TEST(Counter, HoursCloseAndARestartLosesNothing) {
  Counter c = Fresh();
  c.Add(At(3598, "MOWING", true, 0));
  c.Add(At(3599, "MOWING", true, 0.2));
  EXPECT_TRUE(c.Add(At(3600, "MOWING", true, 0.4)).hour_closed);
  ASSERT_EQ(c.HoursJson().size(), 1u);
  EXPECT_EQ(c.HoursJson()[0], json::parse("[1794564000, 1, 0.2, 0.0, 0]"));
  const Counter again(c.StateJson(), c.HoursJson(), c.PartsJson(), kHour + 3700);
  EXPECT_EQ(again.StateJson(), c.StateJson());
  EXPECT_EQ(again.HoursJson(), c.HoursJson());
  // The hour going on has the second after the change.
  EXPECT_EQ(again.StateJson()["hour"]["blade_s"], 1);
}

TEST(Counter, AnEmptyHourIsNotKept) {
  Counter c = Fresh();
  c.Add(At(0, "IDLE", false, 0));
  EXPECT_FALSE(c.Add(At(7200, "IDLE", false, 0)).hour_closed);
  EXPECT_TRUE(c.HoursJson().empty());
}

TEST(Counter, BrokenFilesStartAnew) {
  const Counter c(json::parse(R"({"schema": 2, "since": 5})"), json::parse(R"([[1, 2], "x", [1, -1, 0, 0, 0]])"),
                  json::parse(R"([{"key": "blade", "due_h": 0}])"), kHour);
  EXPECT_EQ(c.StateJson()["since"], static_cast<int64_t>(kHour));
  EXPECT_TRUE(c.HoursJson().empty());
  EXPECT_EQ(c.parts()[0].due_h, 80);
}

TEST(Counter, HoursAreCapped) {
  json rows = json::array();
  for (size_t i = 0; i < kMaxHours + 5; ++i) rows.push_back({kFirstHour + 3600 * static_cast<int64_t>(i), 1, 0, 0, 0});
  const Counter c(nullptr, rows, nullptr, kHour);
  EXPECT_EQ(c.HoursJson().size(), kMaxHours);
  EXPECT_EQ(c.HoursJson()[0][0], kFirstHour + 3600 * 5);
}

TEST(Counter, GetFromAndItsCap) {
  json rows = json::array();
  const size_t most = mowbite_api::kStatsMaxAnswerHours;
  for (size_t i = 0; i < most + 10; ++i) rows.push_back({kFirstHour + 3600 * static_cast<int64_t>(i), 1, 0, 0, 0});
  const Counter c(nullptr, rows, nullptr, kHour);
  const json all = c.Get(0, kHour);
  ASSERT_EQ(all["hours"].size(), most);
  EXPECT_EQ(all["from"], kFirstHour + 3600 * 10);
  EXPECT_EQ(all["hours"][0]["t"], kFirstHour + 3600 * 10);
  const json some = c.Get(kFirstHour + 3600 * 9000, kHour);
  EXPECT_EQ(some["hours"].size(), 10u);
  EXPECT_EQ(some["from"], kFirstHour + 3600 * 9000);
  // Without from: since the turn of the year less a day, cut to the newest as well.
  EXPECT_EQ(c.Get(std::nullopt, kHour)["from"], kFirstHour + 3600 * 10);
  // 2028-01-01 00:00 UTC: all hours lie in the years before.
  EXPECT_EQ(c.Get(std::nullopt, 1830297600)["from"], 1830297600 - 86400);
  EXPECT_EQ(c.Get(std::nullopt, 1830297600)["hours"].size(), 0u);
}

TEST(Counter, WorkBeforeTheClockIsSetGoesIntoItsFirstHour) {
  // One sample a second of the monotonic clock; the wall clock reads about 100 until it is set.
  const auto at = [](double s, double wall, const std::string& state, bool blade, double x) {
    Sample sample = At(s, state, blade, x);
    sample.wall = wall;
    return sample;
  };
  Counter c(nullptr, nullptr, nullptr, 100);
  EXPECT_EQ(c.StateJson()["since"], 0);
  c.Add(at(0, 100, "IDLE", false, 0));
  c.Add(at(1, 101, "MOWING", true, 0));
  c.Add(at(2, 102, "MOWING", true, 0.3));
  c.Add(at(3, 103, "MOWING", true, 0.6));
  EXPECT_TRUE(c.HoursJson().empty());
  EXPECT_EQ(c.Get(0, 103)["hours"].size(), 0u);
  EXPECT_FALSE(c.Add(at(4, kHour + 3598, "MOWING", true, 0.6)).hour_closed);
  c.Add(at(5, kHour + 3599, "MOWING", true, 0.6));
  EXPECT_TRUE(c.Add(at(6, kHour + 3600, "MOWING", true, 0.6)).hour_closed);
  EXPECT_TRUE(c.Add(at(7, kHour + 7200, "MOWING", true, 0.6)).hour_closed);
  EXPECT_DOUBLE_EQ(c.total().blade_s, 6);
  EXPECT_EQ(c.StateJson()["since"], static_cast<int64_t>(kHour + 3598));
  EXPECT_EQ(c.total().runs, 1);
  EXPECT_NEAR(c.total().mowed_m, 0.6, 1e-9);
  const json hours = c.HoursJson();
  ASSERT_EQ(hours.size(), 2u);
  double blade = c.StateJson()["hour"]["blade_s"].get<double>();
  for (size_t i = 0; i < hours.size(); ++i) {
    EXPECT_GE(hours[i][0].get<double>(), kEarliestWall);
    if (i > 0) {
      EXPECT_LT(hours[i - 1][0], hours[i][0]);
    }
    blade += hours[i][1].get<double>();
  }
  EXPECT_EQ(hours[0], json::parse("[1794564000, 4, 0.6, 0.0, 1]"));
  EXPECT_DOUBLE_EQ(blade, c.total().blade_s);
}

TEST(Counter, StoredHoursWithoutASetClockAreLeftOut) {
  const json state = json::parse(
      R"({"schema": 1, "since": 100, "total": {"blade_s": 50, "mowed_m": 0, "driven_m": 0, "runs": 2},
          "hour": {"t": 0, "blade_s": 5, "mowed_m": 0, "driven_m": 0, "runs": 0}})");
  const json hours = json::parse("[[0, 20, 0, 0, 1], [1794564000, 25, 0, 0, 1]]");
  Counter c(state, hours, nullptr, 100);
  EXPECT_EQ(c.HoursJson(), json::parse("[[1794564000, 25, 0.0, 0.0, 1]]"));
  EXPECT_EQ(c.StateJson()["since"], 0);
  EXPECT_EQ(c.StateJson()["hour"]["t"], 0);
  EXPECT_DOUBLE_EQ(c.total().blade_s, 50);
  EXPECT_EQ(c.total().runs, 2);
  // The amounts of the hour without a clock go into the first one with a set clock.
  c.Add(At(3600, "IDLE", false, 0));
  EXPECT_EQ(c.StateJson()["since"], static_cast<int64_t>(kHour + 3600));
  EXPECT_EQ(c.StateJson()["hour"]["t"], static_cast<int64_t>(kHour + 3600));
  EXPECT_EQ(c.StateJson()["hour"]["blade_s"], 5);
  // With now set, since starts at now.
  EXPECT_EQ(Counter(state, hours, nullptr, kHour).StateJson()["since"], static_cast<int64_t>(kHour));
}

TEST(Counter, PartsCountBladeHoursSinceSeenTo) {
  Counter c = Fresh();
  c.Add(At(0, "MOWING", true, 0));
  c.Add(At(3, "MOWING", true, 0));
  EXPECT_NEAR(c.UsedHours(c.parts()[0]), 3.0 / 3600, 1e-12);
  EXPECT_TRUE(c.PartDone("blade", kHour + 3));
  EXPECT_DOUBLE_EQ(c.UsedHours(c.parts()[0]), 0);
  EXPECT_EQ(c.parts()[0].done_at, static_cast<int64_t>(kHour + 3));
  EXPECT_NEAR(c.UsedHours(c.parts()[1]), 3.0 / 3600, 1e-12);
  EXPECT_TRUE(c.PartDue("clean", 150));
  EXPECT_EQ(c.parts()[1].due_h, 150);
  EXPECT_FALSE(c.PartDone("wheels", kHour));
  const Counter again(c.StateJson(), c.HoursJson(), c.PartsJson(), kHour + 10);
  EXPECT_EQ(again.PartsJson(), c.PartsJson());
}
