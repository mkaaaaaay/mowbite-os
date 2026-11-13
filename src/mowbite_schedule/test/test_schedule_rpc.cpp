#include <gtest/gtest.h>

#include <fstream>
#include <regex>
#include <set>
#include <sstream>

#include "mowbite_api/catalog.h"
#include "mowbite_schedule/schedule_rpc.h"

using namespace mowbite_schedule;

namespace {

const char kBerlin[] = "CET-1CEST,M3.5.0,M10.5.0/3";
// 2026-11-12 12:00 UTC.
constexpr int64_t kNoon = 1794484800;

TimeZone Berlin() {
  std::string reason;
  return *TimeZone::Parse(kBerlin, &reason);
}

Scheduler Make(bool available) {
  return Scheduler(SchedulerStart{ScheduleFile(), available, nullptr, nullptr, kNoon});
}

}  // namespace

TEST(ScheduleRpc, TheSunOfTheLocalDayAtTheDatum) {
  const SunTimes sun = SunOfDay(Berlin(), kNoon, 52.52, 13.405);
  ASSERT_TRUE(sun.rise && sun.set);
  // About 07:25 and 16:15 local (CET).
  EXPECT_GT(*sun.rise, kNoon - 6 * 3600);
  EXPECT_LT(*sun.rise, kNoon - 5 * 3600);
  EXPECT_GT(*sun.set, kNoon + 3 * 3600);
  EXPECT_LT(*sun.set, kNoon + 4 * 3600);
  // The same day late in the evening, local.
  const SunTimes evening = SunOfDay(Berlin(), kNoon + 10 * 3600, 52.52, 13.405);
  EXPECT_EQ(evening.set, sun.set);
  EXPECT_FALSE(SunOfDay(Berlin(), kNoon, std::nullopt, 13.4).rise);
  // Polar night at 80 deg north: neither.
  const SunTimes polar = SunOfDay(Berlin(), kNoon, 80.0, 13.4);
  EXPECT_FALSE(polar.rise);
  EXPECT_FALSE(polar.set);
}

TEST(ScheduleRpc, GetTellsAvailabilityClockAndNothingToCome) {
  const Scheduler off = Make(false);
  const json answer = GetAnswer(off, ClockState{false, "never_synced"}, SunTimes{});
  EXPECT_EQ(answer["schema"], 1);
  EXPECT_EQ(answer["version"], 0);
  EXPECT_EQ(answer["available"], false);
  EXPECT_EQ(answer["reason"], "platform_off");
  EXPECT_EQ(answer["clock_ok"], false);
  EXPECT_EQ(answer["clock_reason"], "never_synced");
  EXPECT_TRUE(answer["next_start"].is_null());
  EXPECT_EQ(answer["schedule"]["enabled"], 0);
  EXPECT_TRUE(answer["sun"]["rise"].is_null());
  EXPECT_TRUE(answer["sun"]["set"].is_null());
  const json on = GetAnswer(Make(true), ClockState{true, ""}, SunTimes{100, 200});
  EXPECT_TRUE(on["reason"].is_null());
  EXPECT_TRUE(on["clock_reason"].is_null());
  EXPECT_EQ(on["sun"]["rise"], 100);
  EXPECT_EQ(on["sun"]["set"], 200);
}

TEST(ScheduleRpc, SetTakesScheduleAndVersionOnly) {
  json change;
  const auto refused = [&](const json& params, const std::string& field, const std::string& reason) {
    const std::optional<RpcError> error = CheckSet(params, 3, &change);
    ASSERT_TRUE(error) << params.dump();
    EXPECT_EQ(error->code, kInvalidParams);
    EXPECT_EQ(error->data["field"], field) << params.dump();
    EXPECT_EQ(error->data["reason"], reason) << params.dump();
  };
  refused(json(), "params", "type");
  refused(json::array(), "params", "type");
  refused({{"schedule", json::object()}, {"plans", json::array()}}, "plans", "unknown");
  refused({{"version", 3}}, "schedule", "missing");
  refused({{"schedule", json::object()}, {"version", "3"}}, "version", "type");
  const std::optional<RpcError> stale = CheckSet({{"schedule", json::object()}, {"version", 2}}, 3, &change);
  ASSERT_TRUE(stale);
  EXPECT_EQ(stale->code, -32001);
  EXPECT_EQ(stale->data["version"], 3);
  EXPECT_FALSE(CheckSet({{"schedule", {{"enabled", 1}}}, {"version", 3}}, 3, &change));
  EXPECT_EQ(change, json({{"enabled", 1}}));
  EXPECT_FALSE(CheckSet({{"schedule", {{"min_battery", 50}}}}, 3, &change));
  EXPECT_EQ(change, json({{"min_battery", 50}}));
}

TEST(ScheduleRpc, AnInvalidChangeNamesFieldAndReason) {
  const RpcError error = InvalidError(Invalid{"min_temp", "range", "3, 5, 8 or 10"});
  EXPECT_EQ(error.code, -32602);
  EXPECT_EQ(error.data, json({{"field", "min_temp"}, {"reason", "range"}}));
  EXPECT_EQ(error.message, "min_temp: 3, 5, 8 or 10");
}

TEST(ScheduleRpc, LogTakesALimitFromOneToAHundred) {
  size_t limit = 0;
  EXPECT_FALSE(CheckLog(json(), &limit));
  EXPECT_EQ(limit, 100u);
  EXPECT_FALSE(CheckLog(json::object(), &limit));
  EXPECT_EQ(limit, 100u);
  EXPECT_FALSE(CheckLog({{"limit", 7}}, &limit));
  EXPECT_EQ(limit, 7u);
  EXPECT_EQ(CheckLog({{"limit", 0}}, &limit)->data["reason"], "range");
  EXPECT_EQ(CheckLog({{"limit", 101}}, &limit)->data["reason"], "range");
  EXPECT_EQ(CheckLog({{"limit", 1.5}}, &limit)->data["reason"], "type");
  EXPECT_EQ(CheckLog({{"count", 1}}, &limit)->data["field"], "count");
  EXPECT_EQ(CheckLog(json::array(), &limit)->data["field"], "params");
}

TEST(ScheduleRpc, LeftoversWithoutParamsOnlyRead) {
  json change;
  EXPECT_FALSE(CheckLeftovers(json(), &change));
  EXPECT_TRUE(change.empty());
  EXPECT_FALSE(CheckLeftovers({{"min_m2", 2}}, &change));
  EXPECT_EQ(change, json({{"min_m2", 2}}));
  EXPECT_TRUE(CheckLeftovers("x", &change));
}

// Every word the scheduler writes into its log has its meaning in the catalog, and the catalog no other.
TEST(ScheduleRpc, EveryLogWordIsInTheCatalog) {
  std::ifstream file(SCHEDULER_SOURCE);
  ASSERT_TRUE(file);
  std::stringstream text;
  text << file.rdbuf();
  const std::string source = text.str();
  const std::regex word("\"((skip|waiting|start|stopped|weather|unknown|paused)_[a-z_]+|started)\"");
  std::set<std::string> logged;
  for (auto it = std::sregex_iterator(source.begin(), source.end(), word); it != std::sregex_iterator(); ++it) {
    logged.insert((*it)[1]);
  }
  std::set<std::string> catalog;
  for (const auto& entry : mowbite_api::ScheduleLogWords()) EXPECT_TRUE(catalog.insert(entry.word).second);
  EXPECT_EQ(logged, catalog);
}
