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
    EXPECT_EQ(ShownAnswer("sensors.history", result), result) << result.dump();
  }
  const Json answer = {{"sensors", {{{"sensor_id", "blade_temp"}, {"key", "blade_temp"}, {"name", "Blade Motor Temp"}}}}};
  EXPECT_EQ(ShownAnswer("sensors.history", answer)["sensors"][0],
            (Json{{"sensor_id", "om_mow_motor_temp"}, {"key", "blade_temp"}, {"name", "Mow Motor Temp"}}));
}

TEST(Compat, BoardForTheAppOfOpenMower) {
  EXPECT_EQ(mowbite_bridge::BoardParams(), R"({"node":"/ll","params":{"board":"MowBite"}})");
}

// FP26 B: the actions of the logic under the ids of OpenMower, in its groups and with its two of manual mowing.
TEST(Compat, ActionsAsOpenMowerListsThem) {
  using mowbite_bridge::ActionItem;
  const std::map<std::string, std::vector<ActionItem>> nodes = {
      {"logic",
       {{"emergency.reset", "Unlatch emergency", true},
        {"mowing.start", "Begin mowing", false, "NO_POSITION"},
        {"mowing.pause", "Pause", true},
        {"recording.point", "Take a point here", false},
        {"recording.undo", "Undo points", false},
        {"no.such", "Nothing", true}}}};
  EXPECT_EQ(mowbite_bridge::ShownActionsText(nodes, true),
            R"([{"action_id":"mower_logic/reset_emergency","action_name":"Unlatch emergency","enabled":1},)"
            R"({"action_id":"mower_logic:area_recording/collect_point","action_name":"Take a point here","enabled":0},)"
            R"({"action_id":"mower_logic:area_recording/start_manual_mowing","action_name":"Manual blade on",)"
            R"("enabled":0},)"
            R"({"action_id":"mower_logic:area_recording/stop_manual_mowing","action_name":"Manual blade off",)"
            R"("enabled":0},)"
            R"({"action_id":"mower_logic:area_recording/undo_points","action_name":"Undo points","enabled":0},)"
            R"({"action_id":"mower_logic:idle/start_mowing","action_name":"Begin mowing","enabled":0,)"
            R"("reason":"NO_POSITION"},)"
            R"({"action_id":"mower_logic:mowing/pause","action_name":"Pause","enabled":1}])");
  const std::string own = mowbite_bridge::ShownActionsText(nodes, false);
  EXPECT_EQ(own.find("mower_logic"), std::string::npos);
  EXPECT_NE(own.find(R"({"action_id":"mowing.start","action_name":"Begin mowing","enabled":0,"reason":"NO_POSITION"})"),
            std::string::npos);
  EXPECT_NE(own.find(R"("action_id":"no.such")"), std::string::npos);
}

TEST(Compat, ActionsAClientSends) {
  using mowbite_bridge::OwnAction;
  for (const bool openmower : {true, false}) {
    EXPECT_EQ(OwnAction("mower_logic:idle/start_mowing", openmower), "mowing.start");
    EXPECT_EQ(OwnAction("recording.auto_points:off", openmower), "recording.auto_points:off");
    EXPECT_EQ(OwnAction("mower_logic/unknown", openmower), std::nullopt);
    EXPECT_EQ(OwnAction("mower_logic:mowing/unknown", openmower), std::nullopt);
    EXPECT_EQ(OwnAction("", openmower), std::nullopt);
  }
  // A press without a command cancels an announced start as before, only for the app of OpenMower.
  EXPECT_EQ(OwnAction("mower_logic:idle/start_manual_mowing", true), "mower_logic:idle/start_manual_mowing");
  EXPECT_EQ(OwnAction("mower_logic:idle/start_manual_mowing", false), std::nullopt);
}

TEST(Compat, SettingsUnderTheNodeOfOpenMower) {
  const std::string logic = R"({"node":"/logic","params":{"a":1}})";
  EXPECT_EQ(mowbite_bridge::ShownParams(logic, true), R"({"node":"/mower_logic","params":{"a":1}})");
  EXPECT_EQ(mowbite_bridge::ShownParams(logic, false), logic);
  for (const char* other : {R"({"node":"/n","params":{"a":1}})", "broken", "[]"}) {
    EXPECT_EQ(mowbite_bridge::ShownParams(other, true), other);
  }
  const Json sent = {{"params", {{"/mower_logic/a", 1}, {"/logic/b", 2}, {"/mower_logicx/c", 3}, {"/n/d", 4}}}};
  EXPECT_EQ(OwnParams("params.set", sent),
            (Json{{"params", {{"/logic/a", 1}, {"/logic/b", 2}, {"/mower_logicx/c", 3}, {"/n/d", 4}}}}));
  EXPECT_EQ(OwnParams("params.set", Json{{"params", 1}}), (Json{{"params", 1}}));
  EXPECT_TRUE(mowbite_bridge::ShowsAnswer("params.set", true));
  EXPECT_FALSE(mowbite_bridge::ShowsAnswer("params.set", false));
  EXPECT_EQ(ShownAnswer("params.set", Json{{"/logic/a", 1}}), (Json{{"/mower_logic/a", 1}}));
  EXPECT_EQ(ShownAnswer("params.set", Json(3)), Json(3));
}
