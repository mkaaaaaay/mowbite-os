#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/app_json.h"

using namespace mowbite_bridge;

namespace {

std::string Bson(const Json& j) {
  const auto bytes = Json::to_bson(j);
  return {bytes.begin(), bytes.end()};
}

std::map<std::string, SensorDescription> TwoSensors() {
  SensorDescription battery;
  battery.id = "om_v_battery";
  battery.name = "Battery";
  battery.meaning = Meaning::kVoltage;
  battery.unit = "V";
  battery.min = 20.0;
  battery.max = 30.0;
  battery.critical_low = 21.0;
  SensorDescription state;
  state.id = "om_charge_state";
  state.name = "Charge state";
  state.is_text = true;
  return {{battery.id, battery}, {state.id, state}};
}

}  // namespace

// B6
TEST(Teleop, ReadsTheTwoNumbersOfTheBsonDocument) {
  const auto t = ReadTeleop(Bson({{"vx", 0.2}, {"vz", -0.1}}));
  ASSERT_TRUE(t);
  EXPECT_DOUBLE_EQ(t->vx, 0.2);
  EXPECT_DOUBLE_EQ(t->vz, -0.1);
  EXPECT_TRUE(ReadTeleop(Bson({{"vz", 0.0}, {"extra", "x"}, {"vx", 1}})));
}

TEST(Teleop, AnythingElseGivesNothing) {
  EXPECT_FALSE(ReadTeleop(""));
  EXPECT_FALSE(ReadTeleop("garbage"));
  EXPECT_FALSE(ReadTeleop(Bson({{"vx", 1.0}})));
  EXPECT_FALSE(ReadTeleop(Bson({{"vx", "1"}, {"vz", 0.0}})));
  EXPECT_FALSE(ReadTeleop(Bson({{"vx", 0.1}, {"vz", 0.0}, {"pad", std::string(1100, 'x')}})));
  std::string cut = Bson({{"vx", 0.2}, {"vz", -0.1}});
  cut.pop_back();
  EXPECT_FALSE(ReadTeleop(cut));
}

// B11
TEST(RobotState, FieldsInTheOrderOfTheAppWithSwitchesAsNumbers) {
  RobotStateView s;
  s.battery_percentage = 0.5;
  s.gps_percentage = 0.25;
  s.current_state = "IDLE";
  s.emergency_reason = 4;
  s.is_charging = true;
  s.x = 1.5;
  s.heading_valid = true;
  EXPECT_EQ(RobotStateText(s),
            R"({"battery_percentage":0.5,"gps_percentage":0.25,"current_action_progress":0.0,"current_state":"IDLE",)"
            R"("current_sub_state":"","current_area":-1,"current_path":-1,"current_path_index":-1,"emergency":0,)"
            R"("emergency_reason":4,"is_charging":1,"rain_detected":0,"pose":{"x":1.5,"y":0.0,"heading":0.0,)"
            R"("pos_accuracy":0.0,"heading_accuracy":0.0,"heading_valid":1}})");
}

TEST(RobotState, AFloatOfTheMessageKeepsItsDigits) {
  RobotStateView s;
  s.battery_percentage = static_cast<float>(0.8);
  EXPECT_NE(RobotStateText(s).find(R"("battery_percentage":0.800000011920929,)"), std::string::npos);
}

// B12, the text alone
TEST(Position, TextWithAttributes) {
  EXPECT_EQ(PositionText(-1.0, 0.5, 0.25, {"j1", "s1", true}),
            R"({"x":-1.0,"y":0.5,"heading":0.25,"attributes":{"job_id":"j1","session_id":"s1","blades":true}})");
  EXPECT_EQ(PositionText(0, 0, 0, {}),
            R"({"x":0.0,"y":0.0,"heading":0.0,"attributes":{"job_id":"","session_id":"","blades":false}})");
}

// B13
TEST(Sensors, MeaningNames) {
  EXPECT_STREQ(MeaningName(Meaning::kVoltage), "VOLTAGE");
  EXPECT_STREQ(MeaningName(Meaning::kCurrent), "CURRENT");
  EXPECT_STREQ(MeaningName(Meaning::kTemperature), "TEMPERATURE");
  EXPECT_STREQ(MeaningName(Meaning::kRevolutions), "REVOLUTIONS");
  EXPECT_STREQ(MeaningName(Meaning::kUnknown), "UNKNOWN");
}

TEST(Sensors, InfosSortedByIdWithSwitchesAndZerosForMissingLimits) {
  EXPECT_EQ(SensorInfosText(TwoSensors()),
            R"([{"sensor_id":"om_charge_state","sensor_name":"Charge state","value_type":"STRING",)"
            R"("value_description":"UNKNOWN","unit":"","has_min_max":0,"min_value":0.0,"max_value":0.0,)"
            R"("has_critical_low":0,"lower_critical_value":0.0,"has_critical_high":0,"upper_critical_value":0.0},)"
            R"({"sensor_id":"om_v_battery","sensor_name":"Battery","value_type":"DOUBLE",)"
            R"("value_description":"VOLTAGE","unit":"V","has_min_max":1,"min_value":20.0,"max_value":30.0,)"
            R"("has_critical_low":1,"lower_critical_value":21.0,"has_critical_high":0,"upper_critical_value":0.0}])");
  EXPECT_EQ(SensorInfosText({}), "[]");
}

TEST(Sensors, OneEndOfTheRangeIsNoRange) {
  SensorDescription s;
  s.id = "x";
  s.min = 1.0;
  const Json info = SensorInfo(s);
  EXPECT_EQ(info["has_min_max"], 0);
  EXPECT_EQ(info["min_value"], 0.0);
}

TEST(Sensors, NumberWithSixDecimals) {
  EXPECT_EQ(SensorNumberText(24.5), "24.500000");
  EXPECT_EQ(SensorNumberText(-0.1), "-0.100000");
  EXPECT_EQ(SensorNumberText(0), "0.000000");
}

TEST(Sensors, ListGivesEveryEntryWithValueAndStamp) {
  const std::map<std::string, SensorReading> readings = {{"om_v_battery", {24.5, 1793290000.25}}};
  const Json all = SensorList(TwoSensors(), readings, nullptr);
  ASSERT_EQ(all.size(), 2u);
  EXPECT_EQ(all[1]["sensor_id"], "om_v_battery");
  EXPECT_DOUBLE_EQ(all[1]["value"].get<double>(), 24.5);
  EXPECT_DOUBLE_EQ(all[1]["stamp"].get<double>(), 1793290000.25);
  // The fields of sensor_infos/json first, then value and stamp.
  ASSERT_EQ(all[1].size(), 14u);
  EXPECT_EQ(std::next(all[1].begin(), 12).key(), "value");
  EXPECT_EQ(std::next(all[1].begin(), 13).key(), "stamp");
  EXPECT_TRUE(all[0]["value"].is_null());
  EXPECT_TRUE(all[0]["stamp"].is_null());
  EXPECT_EQ(SensorList(TwoSensors(), readings, Json::object()), all);
  EXPECT_EQ(SensorList(TwoSensors(), readings, Json::array()), all);
  EXPECT_EQ(SensorList(TwoSensors(), readings, Json{{"other", 1}}), all);
}

TEST(Sensors, ListOnlyTheIdsAskedFor) {
  const Json some = SensorList(TwoSensors(), {}, Json{{"ids", {"om_charge_state", "no_such_sensor"}}});
  ASSERT_EQ(some.size(), 1u);
  EXPECT_EQ(some[0]["sensor_id"], "om_charge_state");
  EXPECT_TRUE(SensorList(TwoSensors(), {}, Json{{"ids", Json::array()}}).empty());
  for (const Json& bad : {Json{{"ids", "om_v_battery"}}, Json{{"ids", {1, 2}}}, Json{{"ids", nullptr}},
                          Json::array({"om_v_battery"}), Json("om_v_battery")}) {
    EXPECT_THROW(SensorList(TwoSensors(), {}, bad), std::invalid_argument) << bad.dump();
  }
}

// B14
TEST(Actions, GroupsSortedByPrefixItemsInTheirOrder) {
  const std::map<std::string, std::vector<ActionItem>> groups = {
      {"mower_logic:mowing", {{"pause", "Pause", false}, {"abort_mowing", "Stop", true}}},
      {"mower_logic", {{"reset_emergency", "Unlatch emergency", true}}}};
  EXPECT_EQ(ActionsText(groups),
            R"([{"action_id":"mower_logic/reset_emergency","action_name":"Unlatch emergency","enabled":1},)"
            R"({"action_id":"mower_logic:mowing/pause","action_name":"Pause","enabled":0},)"
            R"({"action_id":"mower_logic:mowing/abort_mowing","action_name":"Stop","enabled":1}])");
  EXPECT_EQ(ActionsText({}), "[]");
}

// I2: the reason of an action that is off, never of one that works.
TEST(Actions, ReasonOnlyWhileOff) {
  const std::map<std::string, std::vector<ActionItem>> off = {
      {"mower_logic:idle", {{"start_mowing", "Begin mowing", false, "NO_POSITION"}}}};
  EXPECT_EQ(ActionsText(off), R"([{"action_id":"mower_logic:idle/start_mowing","action_name":"Begin mowing",)"
                              R"("enabled":0,"reason":"NO_POSITION"}])");
  const std::map<std::string, std::vector<ActionItem>> on = {
      {"mower_logic:idle", {{"start_mowing", "Begin mowing", true, "NO_POSITION"}}}};
  EXPECT_EQ(ActionsText(on), R"([{"action_id":"mower_logic:idle/start_mowing","action_name":"Begin mowing",)"
                             R"("enabled":1}])");
}

// B16, B8
TEST(Fixed, CapabilitiesVersionAndSystemInfo) {
  EXPECT_EQ(CapabilitiesText(), "{\n  \"rpc\": 1,\n  \"map:json\": 1,\n  \"events\": 1\n}");
  EXPECT_EQ(VersionText(""), R"({"version":"UNKNOWN VERSION"})");
  EXPECT_EQ(VersionText("1.2.3"), R"({"version":"1.2.3"})");
  EXPECT_EQ(SystemInfo("", "airseekers_tron", 5).dump(),
            R"({"name":"mowbite-os","version":"UNKNOWN VERSION","platform":"airseekers_tron","schema":)" +
                std::to_string(DataSchema()) + R"(,"api":)" + std::to_string(mowbite_api::kApi) +
                R"(,"mqtt":5,"stop_release":null})");
  EXPECT_EQ(SystemInfo("1.2.3", "", 3)["platform"], "");
  EXPECT_EQ(SystemInfo("1.2.3", "", 3)["mqtt"], 3);
  EXPECT_EQ(SystemInfo("1.2.3", "airseekers_tron", 5, StopRelease(2, 3, 10))["stop_release"].dump(),
            R"({"free_s":2.0,"hold_s":3.0,"max_hold_s":10.0})");
  EXPECT_TRUE(StopRelease(0, 0, 0).is_null());
}
