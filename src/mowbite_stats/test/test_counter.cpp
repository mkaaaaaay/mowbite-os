#include <gtest/gtest.h>

#include "mowbite_stats/counter.h"

using namespace mowbite_stats;

namespace {

// 2026-11-13 10:00:00 UTC, a whole hour.
constexpr double kHour = 1794564000;

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
  c.Add(At(7200, "IDLE", false, 0));
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
  for (size_t i = 0; i < kMaxHours + 5; ++i) rows.push_back({3600 * static_cast<int64_t>(i), 1, 0, 0, 0});
  const Counter c(nullptr, rows, nullptr, kHour);
  EXPECT_EQ(c.HoursJson().size(), kMaxHours);
  EXPECT_EQ(c.HoursJson()[0][0], 3600 * 5);
}

TEST(Counter, GetFromAndItsCap) {
  json rows = json::array();
  for (size_t i = 0; i < kMaxAnswerHours + 10; ++i) rows.push_back({3600 * static_cast<int64_t>(i), 1, 0, 0, 0});
  const Counter c(nullptr, rows, nullptr, kHour);
  const json all = c.Get(0, kHour);
  ASSERT_EQ(all["hours"].size(), kMaxAnswerHours);
  EXPECT_EQ(all["from"], 3600 * 10);
  EXPECT_EQ(all["hours"][0]["t"], 3600 * 10);
  const json some = c.Get(3600 * 9000, kHour);
  EXPECT_EQ(some["hours"].size(), 10u);
  EXPECT_EQ(some["from"], 3600 * 9000);
  // Without from: since the turn of the year, less a day.
  EXPECT_EQ(c.Get(std::nullopt, kHour)["from"], 1767225600 - 86400);
  EXPECT_EQ(c.Get(std::nullopt, kHour)["hours"].size(), 0u);
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
