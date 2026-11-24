#include <gtest/gtest.h>

#include <set>
#include <string>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/compat.h"

using mowbite_bridge::Json;
using mowbite_bridge::OwnParams;
using mowbite_bridge::OwnSensorId;
using mowbite_bridge::SensorDescription;
using mowbite_bridge::ShownAnswer;
using mowbite_bridge::ShownSensor;
using mowbite_bridge::ShownSensorId;
using mowbite_bridge::ShownSensorValue;

// The ids the app of OpenMower reads, each with exactly one own id that is not one of them.
TEST(CompatTable, EveryIdOfOpenMowerHasExactlyOneOwnId) {
  const std::set<std::string> openmower = {"om_v_battery",       "om_v_charge",          "om_charge_current",
                                           "om_charge_state",    "om_mow_motor_rpm",     "om_mow_motor_current",
                                           "om_mow_motor_temp",  "om_gps_accuracy",      "om_left_esc_temp",
                                           "om_right_esc_temp"};
  std::set<std::string> seen;
  std::set<std::string> own;
  for (const auto& sensor : mowbite_api::Sensors()) {
    EXPECT_NE(sensor.id.rfind("om_", 0), 0u) << sensor.id;
    EXPECT_TRUE(own.insert(sensor.id).second) << sensor.id;
    EXPECT_EQ(sensor.openmower_id.empty(), sensor.openmower_name.empty()) << sensor.id;
    if (sensor.openmower_id.empty()) continue;
    EXPECT_TRUE(seen.insert(sensor.openmower_id).second) << sensor.openmower_id;
    EXPECT_EQ(mowbite_api::FindOpenMowerSensor(sensor.openmower_id), &sensor);
    EXPECT_EQ(OwnSensorId(sensor.openmower_id), sensor.id);
    EXPECT_EQ(ShownSensorId(sensor.id, true), sensor.openmower_id);
    EXPECT_EQ(ShownSensorId(sensor.id, false), sensor.id);
  }
  EXPECT_EQ(seen, openmower);
  for (const auto& id : seen) EXPECT_EQ(own.count(id), 0u) << id;
  EXPECT_EQ(mowbite_api::FindOpenMowerSensor("battery_temp"), nullptr);
  EXPECT_EQ(OwnSensorId("battery_temp"), "battery_temp");
  EXPECT_EQ(OwnSensorId("no_such"), "no_such");
}

TEST(Compat, SensorShownWithItsOwnIdAsKey) {
  SensorDescription own;
  own.id = "blade_rpm";
  own.name = "Blade Speed";
  own.unit = "rpm";
  const SensorDescription shown = ShownSensor(own, true);
  EXPECT_EQ(shown.id, "om_mow_motor_rpm");
  EXPECT_EQ(shown.name, "Mow Motor Revolutions");
  EXPECT_EQ(shown.key, "blade_rpm");
  EXPECT_EQ(shown.unit, "rpm");
  const SensorDescription plain = ShownSensor(own, false);
  EXPECT_EQ(plain.id, "blade_rpm");
  EXPECT_EQ(plain.name, "Blade Speed");
  EXPECT_EQ(plain.key, "blade_rpm");
  SensorDescription platform;
  platform.id = "vision_state";
  EXPECT_EQ(ShownSensor(platform, true).id, "vision_state");
  EXPECT_EQ(ShownSensor(platform, true).key, "vision_state");
}

TEST(Compat, ChargeStateInTheWordsOfOpenMower) {
  EXPECT_EQ(ShownSensorValue("charge_state", "idle", true), "Not Charging");
  EXPECT_EQ(ShownSensorValue("charge_state", "charging", true), "Fast Charge");
  EXPECT_EQ(ShownSensorValue("charge_state", "done", true), "Done");
  EXPECT_EQ(ShownSensorValue("charge_state", "charging", false), "charging");
  EXPECT_EQ(ShownSensorValue("os_ntrip_state", "idle", true), "idle");
  EXPECT_EQ(ShownSensorValue("battery_voltage", 25.0, true), 25.0);
}

TEST(Compat, HistoryParamsAndAnswersOfOtherShapesStayAsTheyAre) {
  for (const Json& params : {Json(), Json::array(), Json{{"sensor_ids", "om_v_battery"}}, Json{{"x", 1}}}) {
    EXPECT_EQ(OwnParams("sensors.history", params), params) << params.dump();
  }
  const Json ids = {{"sensor_ids", {"om_gps_accuracy", 3}}};
  EXPECT_EQ(OwnParams("sensors.history", ids), (Json{{"sensor_ids", {"position_accuracy", 3}}}));
  EXPECT_EQ(OwnParams("sensors.list", ids), ids);
  EXPECT_TRUE(mowbite_bridge::ShowsAnswer("sensors.history", true));
  EXPECT_FALSE(mowbite_bridge::ShowsAnswer("sensors.history", false));
  EXPECT_FALSE(mowbite_bridge::ShowsAnswer("stats.get", true));
  for (const Json& result : {Json(), Json{{"sensors", 1}}, Json{{"sensors", {1, Json{{"sensor_id", 2}}}}}}) {
    EXPECT_EQ(ShownAnswer(result), result) << result.dump();
  }
  const Json answer = {{"sensors", {{{"sensor_id", "blade_temp"}, {"key", "blade_temp"}, {"name", "Blade Motor Temp"}}}}};
  EXPECT_EQ(ShownAnswer(answer)["sensors"][0],
            (Json{{"sensor_id", "om_mow_motor_temp"}, {"key", "blade_temp"}, {"name", "Mow Motor Temp"}}));
}

TEST(Compat, BoardForTheAppOfOpenMower) {
  EXPECT_EQ(mowbite_bridge::BoardParams(), R"({"node":"/ll","params":{"board":"MowBite"}})");
}
