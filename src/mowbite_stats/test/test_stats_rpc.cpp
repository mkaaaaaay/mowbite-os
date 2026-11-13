#include <gtest/gtest.h>

#include <set>

#include "mowbite_api/catalog.h"
#include "mowbite_stats/stats_rpc.h"

using namespace mowbite_stats;

namespace {

constexpr double kNow = 1794564000;

json Reason(const RpcOutcome& out) {
  EXPECT_TRUE(out.error.has_value());
  return out.error ? out.error->data : json();
}

}  // namespace

TEST(StatsRpc, MethodsAreInTheCatalog) {
  std::set<std::string> catalog;
  for (const auto& method : mowbite_api::Methods()) {
    if (method.node == "mowbite_stats") catalog.insert(method.name);
  }
  const auto& methods = StatsMethods();
  EXPECT_EQ(catalog, std::set<std::string>(methods.begin(), methods.end()));
}

TEST(StatsRpc, GetGivesTheKeysOfTheCatalog) {
  Counter c(nullptr, nullptr, nullptr, kNow);
  const RpcOutcome out = HandleStats(c, "stats.get", nullptr, kNow);
  ASSERT_FALSE(out.error);
  std::set<std::string> keys;
  for (const auto& item : out.result.items()) keys.insert(item.key());
  std::set<std::string> catalog;
  for (const auto& method : mowbite_api::Methods()) {
    if (method.name != "stats.get") continue;
    for (const auto& field : method.result.fields) catalog.insert(field.name);
  }
  EXPECT_EQ(keys, catalog);
  EXPECT_FALSE(out.parts_changed);
}

TEST(StatsRpc, WrongParams) {
  Counter c(nullptr, nullptr, nullptr, kNow);
  EXPECT_EQ(Reason(HandleStats(c, "stats.get", json::parse(R"({"to": 1})"), kNow)),
            json::parse(R"({"field": "to", "reason": "unknown"})"));
  EXPECT_EQ(Reason(HandleStats(c, "stats.get", json::parse(R"({"from": "x"})"), kNow))["reason"], "type");
  EXPECT_EQ(Reason(HandleStats(c, "stats.part_done", nullptr, kNow))["field"], "params");
  EXPECT_EQ(Reason(HandleStats(c, "stats.part_done", json::object(), kNow))["reason"], "missing");
  EXPECT_EQ(Reason(HandleStats(c, "stats.part_done", json::parse(R"({"part": "wheels"})"), kNow))["reason"],
            "unknown");
  EXPECT_EQ(Reason(HandleStats(c, "stats.part_due", json::parse(R"({"part": "blade"})"), kNow))["field"], "hours");
  EXPECT_EQ(Reason(HandleStats(c, "stats.part_due", json::parse(R"({"part": "blade", "hours": 0})"), kNow))["reason"],
            "range");
  EXPECT_EQ(
      Reason(HandleStats(c, "stats.part_due", json::parse(R"({"part": "blade", "hours": 10001})"), kNow))["reason"],
      "range");
  EXPECT_EQ(
      Reason(HandleStats(c, "stats.part_due", json::parse(R"({"part": "blade", "hours": 2.5})"), kNow))["reason"],
      "type");
}

TEST(StatsRpc, PartsChange) {
  Counter c(nullptr, nullptr, nullptr, kNow);
  RpcOutcome out = HandleStats(c, "stats.part_due", json::parse(R"({"part": "blade", "hours": 60})"), kNow);
  ASSERT_FALSE(out.error);
  EXPECT_TRUE(out.parts_changed);
  EXPECT_EQ(out.result["parts"][0]["due_h"], 60);
  out = HandleStats(c, "stats.part_done", json::parse(R"({"part": "clean"})"), kNow + 5);
  ASSERT_FALSE(out.error);
  EXPECT_EQ(out.result["parts"][1]["done_at"], static_cast<int64_t>(kNow + 5));
}
