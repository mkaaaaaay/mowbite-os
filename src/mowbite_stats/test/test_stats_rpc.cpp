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

TEST(StatsRpc, HistoryGivesTheKeysOfTheCatalog) {
  SensorHistory h;
  ASSERT_TRUE(h.Describe("v", "Battery", "V", true));
  h.Add("v", 25, kNow - 30);
  const RpcOutcome out = HandleHistory(h, nullptr, kNow);
  ASSERT_FALSE(out.error);
  const mowbite_api::Method* method = nullptr;
  for (const auto& entry : mowbite_api::Methods()) {
    if (entry.name == kHistoryMethod) method = &entry;
  }
  ASSERT_NE(method, nullptr);
  const auto names = [](const std::vector<mowbite_api::Field>& fields) {
    std::set<std::string> out;
    for (const auto& field : fields) out.insert(field.name);
    return out;
  };
  const auto keys = [](const json& object) {
    std::set<std::string> out;
    for (const auto& item : object.items()) out.insert(item.key());
    return out;
  };
  EXPECT_EQ(keys(out.result), names(method->result.fields));
  ASSERT_EQ(out.result["sensors"].size(), 1u);
  for (const auto& field : method->result.fields) {
    if (field.name == "sensors") EXPECT_EQ(keys(out.result["sensors"][0]), names(field.fields));
  }
  EXPECT_EQ(names(method->params.fields), std::set<std::string>({"client", "sensor_ids", "from", "to", "step"}));
  EXPECT_EQ(out.result["sensors"][0]["points"], json::parse("[[1794563940, 25, 25, 25]]"));
}

TEST(StatsRpc, HistoryParams) {
  SensorHistory h;
  ASSERT_TRUE(h.Describe("v", "Battery", "V", true));
  const auto call = [&h](const char* params) { return HandleHistory(h, json::parse(params), kNow); };
  const RpcOutcome good =
      call(R"({"client": "app", "sensor_ids": ["v", "x"], "from": 1794560000, "to": 1794564000, "step": 300})");
  ASSERT_FALSE(good.error);
  EXPECT_EQ(good.result["step"], 300);
  EXPECT_EQ(good.result["from"], 1794560000 - 1794560000 % 300);
  EXPECT_FALSE(HandleHistory(h, json::object(), kNow).error);
  EXPECT_EQ(Reason(call(R"({"since": 1})")), json::parse(R"({"field": "since", "reason": "unknown"})"));
  EXPECT_EQ(Reason(call("[]"))["field"], "params");
  EXPECT_EQ(Reason(call(R"({"client": 1})")), json::parse(R"({"field": "client", "reason": "type"})"));
  EXPECT_EQ(Reason(call(R"({"client": "App"})")), json::parse(R"({"field": "client", "reason": "format"})"));
  EXPECT_EQ(Reason(call(R"({"sensor_ids": "v"})"))["reason"], "type");
  EXPECT_EQ(Reason(call(R"({"sensor_ids": ["v", 1]})"))["field"], "sensor_ids");
  EXPECT_EQ(Reason(call(R"({"from": -1})")), json::parse(R"({"field": "from", "reason": "type"})"));
  EXPECT_EQ(Reason(call(R"({"to": 1.5})")), json::parse(R"({"field": "to", "reason": "type"})"));
  EXPECT_EQ(Reason(call(R"({"from": 2, "to": 1})")), json::parse(R"({"field": "from", "reason": "range"})"));
  EXPECT_EQ(Reason(call(R"({"step": 0})")), json::parse(R"({"field": "step", "reason": "range"})"));
  EXPECT_EQ(Reason(call(R"({"step": 901})"))["reason"], "range");
  EXPECT_EQ(Reason(call(R"({"step": "60"})"))["reason"], "type");
  // A step below 60 is raised, as one between the steps.
  EXPECT_EQ(call(R"({"step": 1})").result["step"], 60);
  EXPECT_EQ(call(R"({"step": 301})").result["step"], 900);
}
