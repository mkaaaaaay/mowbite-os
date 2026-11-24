#include <gtest/gtest.h>

#include <string>

#include "mowbite_hold/lock.h"
#include "mowbite_setup/program.h"

using namespace mowbite_setup;

namespace {

constexpr double kWall = 1700000000.0;

const char* const kChoice = R"({"catalog": 1, "steps": [
  {"id": "platform", "stage": "device", "check": "platform"}]})";

const char* const kTron = R"({"catalog": 1, "steps": [
  {"id": "platform", "stage": "device", "check": "platform"},
  {"id": "variant", "stage": "device", "check": "choice", "params": {"name": "variant", "values": ["a", "b"]},
   "value_texts": {"a": {"de": {"name": "A"}, "en": {"name": "A"}}, "b": {"de": {"name": "B"}, "en": {"name": "B"}}},
   "texts": {"de": {"name": "Bauart"}, "en": {"name": "Build"}}}]})";

json Mowers() {
  return json::array({{{"name", "tron"}, {"texts", json::object()}}, {{"name", "nx60i"}, {"texts", json::object()}}});
}

struct Bench {
  Bench(const char* list, const std::string& chosen, bool fixed = false)
      : program(*ParseSteps(json::parse(list), nullptr), chosen.empty() ? "" : "airseekers_" + chosen) {
    program.SetPlatforms(Mowers(), chosen, fixed);
    World world;
    world.state = "IDLE";
    program.OnWorld(world);
  }
  std::string Choose(const std::string& name, json* result = nullptr) {
    json out;
    const auto refusal = program.ChoosePlatform({{"platform", name}, {"version", program.version()}}, kWall, &out);
    if (result) *result = out;
    return refusal ? refusal->reason : "";
  }
  std::string StepState(const std::string& id) {
    const json state = program.State(0, {});
    for (const auto& step : state["steps"]) {
      if (step["id"] == id) return step["state"];
    }
    return "";
  }
  std::vector<json> Setup() {
    std::vector<json> out;
    for (const Event& event : program.TakeEvents()) {
      if (event.type == "SETUP") out.push_back(event.details);
    }
    return out;
  }
  Program program;
};

TEST(PlatformChoice, TheStageOfTheChoiceOffersTheMowers) {
  Bench bench(kChoice, "");
  EXPECT_EQ(bench.StepState("platform"), "open");
  EXPECT_EQ(bench.program.Platforms(), json({{"platforms", Mowers()}, {"current", ""}, {"fixed", false}}));
  EXPECT_EQ(bench.Choose("yf500"), "UNKNOWN_PLATFORM");
  EXPECT_FALSE(bench.program.TakePlatform());
  json result;
  EXPECT_EQ(bench.Choose("tron", &result), "");
  EXPECT_EQ(result, json({{"restart", true}}));
  EXPECT_EQ(bench.program.TakePlatform(), "tron");
  EXPECT_FALSE(bench.program.TakePlatform());
  const auto events = bench.Setup();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0], json({{"what", "platform"}, {"platform", "tron"}, {"from", ""}}));
  // Nothing was measured, so nothing closes.
  EXPECT_FALSE(bench.program.TakeLock());
}

// The ticks of the stage of the choice leave the open step platform to the choice.
TEST(PlatformChoice, TheTicksLeaveTheStepToTheChoice) {
  Bench bench(kChoice, "");
  bench.program.SetWritable(true);
  for (int i = 0; i < 3; ++i) bench.program.Tick(10.0 + i, kWall + i);
  EXPECT_EQ(bench.StepState("platform"), "open");
}

TEST(PlatformChoice, OnAMowerTheStepIsOkAndTheSameChoiceDoesNothing) {
  Bench bench(kTron, "tron");
  EXPECT_EQ(bench.StepState("platform"), "ok");
  json result;
  EXPECT_EQ(bench.Choose("tron", &result), "");
  EXPECT_EQ(result, json({{"restart", false}}));
  EXPECT_FALSE(bench.program.TakePlatform());
  EXPECT_TRUE(bench.Setup().empty());
}

TEST(PlatformChoice, AFixedPlatformTakesNoOther) {
  Bench bench(kTron, "tron", true);
  EXPECT_EQ(bench.Choose("nx60i"), "PLATFORM_FIXED");
  EXPECT_FALSE(bench.program.TakePlatform());
}

TEST(PlatformChoice, OnlyAMowerAtRestChoosesAnother) {
  Bench bench(kTron, "tron");
  World world;
  world.state = "MOWING";
  bench.program.OnWorld(world);
  EXPECT_EQ(bench.Choose("nx60i"), "BUSY");
  EXPECT_FALSE(bench.program.TakePlatform());
}

TEST(PlatformChoice, AnotherMowerTakesEveryResultBackAndClosesTheLock) {
  Bench bench(kTron, "tron");
  Capabilities capabilities;
  capabilities.safety_measurement_date = "2026-11-24";
  bench.program.OnCapabilities(capabilities);
  ASSERT_FALSE(bench.program.Answer({{"step", "variant"}, {"version", bench.program.version()},
                                      {"values", {{"variant", "a"}}}},
                                     0, kWall));
  ASSERT_EQ(bench.StepState("variant"), "ok");
  bench.program.TakeLock();
  bench.Setup();
  EXPECT_EQ(bench.Choose("nx60i"), "");
  EXPECT_EQ(bench.program.TakePlatform(), "nx60i");
  EXPECT_EQ(bench.StepState("variant"), "open");
  EXPECT_EQ(bench.StepState("platform"), "ok");
  const auto text = bench.program.TakeLock();
  ASSERT_TRUE(text && !text->empty());
  EXPECT_TRUE(mowbite_hold::ParseSafetyLock(*text)->causes.at(0).reset);
  const auto events = bench.Setup();
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.back(), json({{"what", "platform"}, {"platform", "nx60i"}, {"from", "tron"}}));
}

TEST(PlatformChoice, LoadAndResetKeepTheStepOk) {
  Bench bench(kTron, "tron");
  const std::string text = bench.program.Text();
  Bench again(kTron, "tron");
  ASSERT_TRUE(again.program.Load(text, kWall, nullptr));
  EXPECT_EQ(again.StepState("platform"), "ok");
  ASSERT_FALSE(again.program.Reset({{"version", again.program.version()}}, kWall));
  EXPECT_EQ(again.StepState("platform"), "ok");
}

TEST(PlatformChoice, TheStepPlatformIsOnlyTheFirst) {
  json list = json::parse(kTron);
  std::swap(list["steps"][0], list["steps"][1]);
  std::string why;
  EXPECT_FALSE(ParseSteps(list, &why));
  EXPECT_NE(why.find("only the first step"), std::string::npos) << why;
}

}  // namespace
