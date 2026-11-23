#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/logic_params.h"

namespace mowbite_logic {
namespace {

json Request(json params) {
  return json{{"params", std::move(params)}};
}

TEST(LogicParams, TheSettingsOfTheCatalogWithoutPlanningValues) {
  const json values = LogicParams().Values();
  EXPECT_EQ(values.size(), mowbite_api::LogicSettings().size());
  for (const char* name : {"outline_count", "outline_overlap_count", "outline_offset", "tool_width", "mow_angle_offset",
                           "mow_angle_offset_is_absolute", "mow_angle_increment"}) {
    EXPECT_FALSE(values.contains(name)) << name;
  }
  EXPECT_EQ(values["rain_mode"], 1);
  EXPECT_EQ(values["docking_retry_count"], 4);
  EXPECT_EQ(values["dock_at_sunset"], 1);
  EXPECT_DOUBLE_EQ(values["max_position_accuracy"].get<double>(), 0.1);
  EXPECT_DOUBLE_EQ(values["gps_timeout"].get<double>(), 10.0);
}

TEST(LogicParams, DefaultIsTheCatalogs) {
  EXPECT_EQ(LogicParams::Default("rain_mode"), 1.0);
  EXPECT_EQ(LogicParams::Default("rain_mode"), LogicParams().Get("rain_mode"));
  EXPECT_THROW(LogicParams::Default("outline_count"), std::out_of_range);
}

TEST(LogicParams, IntsAreIntegersInTheJson) {
  const json values = LogicParams().Values();
  EXPECT_TRUE(values["rain_delay_minutes"].is_number_integer());
  EXPECT_TRUE(values["automatic_mode"].is_number_integer());
  EXPECT_TRUE(values["gps_wait_time"].is_number_float());
}

TEST(LogicParams, BaseValuesWithinRange) {
  LogicParams params(json{{"max_position_accuracy", 0.05}, {"gps_wait_time", 5.0}, {"gps_timeout", 0.0}, {"x", 1}});
  EXPECT_DOUBLE_EQ(params.Get("max_position_accuracy"), 0.05);
  EXPECT_DOUBLE_EQ(params.Get("gps_wait_time"), 5.0);
  EXPECT_DOUBLE_EQ(params.Get("gps_timeout"), 1.0);
}

TEST(LogicParams, SetAnswersTheValuesTakenByFullName) {
  LogicParams params;
  const auto result = params.Set(Request({{"/mower_logic/gps_wait_time", 20.0}, {"/mower_logic/rain_mode", 2}}));
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_EQ(result.applied, (json{{"/mower_logic/gps_wait_time", 20.0}, {"/mower_logic/rain_mode", 2}}));
  EXPECT_DOUBLE_EQ(params.Get("gps_wait_time"), 20.0);
  EXPECT_DOUBLE_EQ(params.Get("rain_mode"), 2);
}

TEST(LogicParams, OutOfRangeGoesToTheBound) {
  LogicParams params;
  const auto result = params.Set(Request({{"/mower_logic/max_position_accuracy", 1.0},
                                          {"/mower_logic/gps_timeout", 0},
                                          {"/mower_logic/rain_delay_minutes", 5},
                                          {"/mower_logic/docking_retry_count", 2.6}}));
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_DOUBLE_EQ(result.applied["/mower_logic/max_position_accuracy"].get<double>(), 0.3);
  EXPECT_DOUBLE_EQ(result.applied["/mower_logic/gps_timeout"].get<double>(), 1.0);
  EXPECT_EQ(result.applied["/mower_logic/rain_delay_minutes"], 30);
  EXPECT_EQ(result.applied["/mower_logic/docking_retry_count"], 3);
  EXPECT_DOUBLE_EQ(params.Get("max_position_accuracy"), 0.3);
}

TEST(LogicParams, DockingWithinTheBoundsOfMowBiteOs) {
  LogicParams params;
  const auto result = params.Set(Request({{"/mower_logic/docking_approach_distance", 0.2},
                                          {"/mower_logic/docking_retry_count", 9}}));
  ASSERT_TRUE(result.ok) << result.message;
  EXPECT_DOUBLE_EQ(result.applied["/mower_logic/docking_approach_distance"].get<double>(), 0.5);
  EXPECT_EQ(result.applied["/mower_logic/docking_retry_count"], 5);
  ASSERT_TRUE(params.Set(Request({{"/mower_logic/docking_approach_distance", 3.0}})).ok);
  EXPECT_DOUBLE_EQ(params.Get("docking_approach_distance"), 2.0);
}

TEST(LogicParams, AnyBadNameOrValueChangesNothing) {
  LogicParams params;
  const json before = params.Values();
  auto result = params.Set(Request({{"/mower_logic/gps_wait_time", 20.0}, {"/xbot_positioning/antenna_offset_x", 1}}));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message, "No dynamic_reconfigure server found for /xbot_positioning/antenna_offset_x");
  result = params.Set(Request({{"/mower_logic/gps_wait_time", 20.0}, {"/mower_logic/tool_width", 0.2}}));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message, "/mower_logic/tool_width is not a setting of MowBite OS");
  result = params.Set(Request({{"/mower_logic/gps_wait_time", 20.0}, {"/mower_logic/rain_mode", true}}));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message, "/mower_logic/rain_mode must be a number");
  result = params.Set(Request({{"/mower_logic/gps_wait_time", "20"}}));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(params.Values(), before);
}

TEST(LogicParams, RequestWithoutParamsIsRefused) {
  LogicParams params;
  for (const json& request : {json(), json::object(), json{{"params", json::object()}}, json{{"params", 1}},
                              json::array()}) {
    const auto result = params.Set(request);
    EXPECT_FALSE(result.ok) << request.dump();
    EXPECT_EQ(result.message, "Expected {\"params\": {name: value, ...}}");
  }
}

TEST(LogicParams, ColdHasToStayBelowHot) {
  LogicParams params;
  auto result = params.Set(Request({{"/mower_logic/motor_cold_temperature", 70.0}}));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.message, "motor_cold_temperature must be below motor_hot_temperature");
  EXPECT_DOUBLE_EQ(params.Get("motor_cold_temperature"), 40.0);
  // Both in one call count together.
  result = params.Set(
      Request({{"/mower_logic/motor_cold_temperature", 80.0}, {"/mower_logic/motor_hot_temperature", 90.0}}));
  EXPECT_TRUE(result.ok) << result.message;
  result = params.Set(Request({{"/mower_logic/motor_hot_temperature", 80.0}}));
  EXPECT_FALSE(result.ok);
  EXPECT_DOUBLE_EQ(params.Get("motor_hot_temperature"), 90.0);
}

TEST(LogicParams, TextLoadsBack) {
  LogicParams params;
  ASSERT_TRUE(params.Set(Request({{"/mower_logic/gps_timeout", 4.5}, {"/mower_logic/automatic_mode", 1}})).ok);
  LogicParams loaded;
  ASSERT_TRUE(loaded.Load(params.Text()));
  EXPECT_EQ(loaded.Values(), params.Values());
}

TEST(LogicParams, LoadKeepsKnownNamesWithinRange) {
  LogicParams params(json{{"max_position_accuracy", 0.05}});
  ASSERT_TRUE(params.Load(R"({"gps_wait_time": 70, "tool_width": 0.3, "rain_mode": "2"})"));
  EXPECT_DOUBLE_EQ(params.Get("gps_wait_time"), 60.0);
  EXPECT_DOUBLE_EQ(params.Get("rain_mode"), LogicParams().Get("rain_mode"));
  // Not in the file: the base stays.
  EXPECT_DOUBLE_EQ(params.Get("max_position_accuracy"), 0.05);
  EXPECT_FALSE(params.Values().contains("tool_width"));
}

TEST(LogicParams, BadFileChangesNothing) {
  LogicParams params;
  const json before = params.Values();
  EXPECT_FALSE(params.Load("{\"gps_wait_time\": 5"));
  EXPECT_FALSE(params.Load("[1, 2]"));
  EXPECT_FALSE(params.Load(R"({"gps_wait_time": 5, "motor_cold_temperature": 90})"));
  EXPECT_EQ(params.Values(), before);
}

TEST(BladeHeight, ZeroLeavesTheHeightToTheDriver) {
  EXPECT_EQ(BladeHeightMm(0, true, 30, 90), 0);
  EXPECT_EQ(BladeHeightMm(0.4, true, 30, 90), 0);
  EXPECT_EQ(BladeHeightMm(std::nan(""), true, 30, 90), 0);
  EXPECT_EQ(BladeHeightMm(60, false, 0, 0), 0);
  EXPECT_EQ(BladeHeightMm(60, false, 30, 90), 0);
  EXPECT_EQ(BladeHeightMm(60, true, 0, 0), 0);
  EXPECT_EQ(BladeHeightMm(60, true, 90, 30), 0);
}

TEST(BladeHeight, WithinTheRangeOfThePlatform) {
  EXPECT_EQ(BladeHeightMm(60, true, 30, 90), 60);
  EXPECT_EQ(BladeHeightMm(45.6, true, 30, 90), 46);
  EXPECT_EQ(BladeHeightMm(1, true, 30, 90), 30);
  EXPECT_EQ(BladeHeightMm(150, true, 30, 90), 90);
}

TEST(BladeHeight, TheSettingIsZeroUntilSet) {
  LogicParams params;
  EXPECT_EQ(params.Values()["mow_height_mm"], 0);
  ASSERT_TRUE(params.Set(Request({{"/mower_logic/mow_height_mm", 75}})).ok);
  EXPECT_DOUBLE_EQ(params.Get("mow_height_mm"), 75);
  EXPECT_EQ(params.Set(Request({{"/mower_logic/mow_height_mm", 400}})).applied["/mower_logic/mow_height_mm"], 150);
}

}  // namespace
}  // namespace mowbite_logic
