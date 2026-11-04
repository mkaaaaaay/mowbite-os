#include <gtest/gtest.h>

#include "mowbite_bridge/health.h"

using namespace mowbite_bridge;

TEST(Health, BrokerFromTheEnvironmentOrLocal) {
  std::string error;
  auto broker = BrokerFromEnvironment(nullptr, nullptr, &error);
  ASSERT_TRUE(broker);
  EXPECT_EQ(broker->host, "127.0.0.1");
  EXPECT_EQ(broker->port, 1883);
  broker = BrokerFromEnvironment("", "", &error);
  ASSERT_TRUE(broker);
  EXPECT_EQ(broker->host, "127.0.0.1");
  EXPECT_EQ(broker->port, 1883);
  broker = BrokerFromEnvironment("mosquitto", "18861", &error);
  ASSERT_TRUE(broker);
  EXPECT_EQ(broker->host, "mosquitto");
  EXPECT_EQ(broker->port, 18861);
  EXPECT_EQ(BrokerFromEnvironment(nullptr, "65535", &error)->port, 65535);
}

TEST(Health, ABadPortIsNoBroker) {
  for (const char* port : {"0", "65536", "-1", "18x", "x", " 1883", "1883 ", "99999999999999999999"}) {
    std::string error;
    EXPECT_FALSE(BrokerFromEnvironment(nullptr, port, &error)) << port;
    EXPECT_EQ(error, std::string("MOWBITE_MQTT_PORT is not a port: ") + port);
  }
}

TEST(Health, ARobotStateNeedsItsCurrentState) {
  EXPECT_EQ(RobotStateProblem(R"({"current_state":"IDLE","emergency":false})"), "");
  EXPECT_EQ(RobotStateProblem(R"({"current_state":"MOWING"})"), "");
  EXPECT_EQ(RobotStateProblem(""), "robot_state/json is no JSON object");
  EXPECT_EQ(RobotStateProblem("{"), "robot_state/json is no JSON object");
  EXPECT_EQ(RobotStateProblem(R"(["IDLE"])"), "robot_state/json is no JSON object");
  EXPECT_EQ(RobotStateProblem(R"({"state":"IDLE"})"), "robot_state/json has no current_state");
  EXPECT_EQ(RobotStateProblem(R"({"current_state":1})"), "robot_state/json has no current_state");
  EXPECT_EQ(RobotStateProblem(R"({"current_state":null})"), "robot_state/json has no current_state");
  EXPECT_EQ(RobotStateProblem(R"({"current_state":""})"), "robot_state/json has an empty current_state");
}

TEST(Health, TheHaltLineIsTheFirstLineOfTheNote) {
  EXPECT_EQ(HaltLine(""), "");
  EXPECT_EQ(HaltLine("\n"), "");
  const std::string line = "MOWBITE_OS_HALT reason=SCHEMA_NEWER data_schema=3 image_schema=2 The data is newer.";
  EXPECT_EQ(HaltLine(line), line);
  EXPECT_EQ(HaltLine(line + "\n"), line);
  EXPECT_EQ(HaltLine(line + "\nmore"), line);
}
