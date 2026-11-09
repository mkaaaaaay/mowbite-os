#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <set>
#include <string>

#include "tron_vision/vision_node.h"

namespace tron_vision {
namespace {

class VisionNodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {
    rclcpp::shutdown();
  }
};

// Display only: the process publishes on the topics of the design and the defaults of every node, nothing else, and
// listens to nothing of the stack.
TEST_F(VisionNodeTest, PublishesOnlyTheTopicsOfTheDesign) {
  auto node = std::make_shared<VisionNode>(rclcpp::NodeOptions());
  const std::set<std::string> allowed = {"/bridge/sensor_info", "/bridge/sensor_value", "/hw/cameras",
                                         "/vision/detections", "/rosout", "/parameter_events"};
  const std::regex jpeg("/camera/[a-z0-9_]+/jpeg");
  const auto graph = node->get_node_graph_interface();
  const auto publishers = graph->get_publisher_names_and_types_by_node("tron_vision", "/");
  EXPECT_TRUE(publishers.count("/bridge/sensor_value"));
  EXPECT_TRUE(publishers.count("/vision/detections"));
  for (const auto& [topic, types] : publishers) {
    EXPECT_TRUE(allowed.count(topic) || std::regex_match(topic, jpeg)) << topic;
  }
  // The clock of every node follows use_sim_time over /parameter_events.
  for (const auto& [topic, types] : graph->get_subscriber_names_and_types_by_node("tron_vision", "/")) {
    EXPECT_EQ(topic, "/parameter_events");
  }
  EXPECT_TRUE(graph->get_client_names_and_types_by_node("tron_vision", "/").empty());
}

// Without the files the state says why, as text on bridge/sensor_value, and the rate is 0.
TEST_F(VisionNodeTest, ReportsTheStateAsSensors) {
  auto node = std::make_shared<VisionNode>(rclcpp::NodeOptions());
  auto listener = std::make_shared<rclcpp::Node>("listener");
  std::string state;
  double rate = -1.0;
  auto sub = listener->create_subscription<mowbite_msgs::msg::SensorValue>(
      "bridge/sensor_value", 100, [&](const mowbite_msgs::msg::SensorValue& m) {
        if (m.sensor_id == "vision_state") state = m.text;
        if (m.sensor_id == "vision_rate") rate = m.value;
      });
  rclcpp::executors::SingleThreadedExecutor spinner;
  spinner.add_node(listener);
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while ((state.empty() || rate < 0.0) && std::chrono::steady_clock::now() < end) {
    spinner.spin_some(std::chrono::milliseconds(100));
  }
  EXPECT_EQ(state, "model_missing: path not set");
  EXPECT_EQ(rate, 0.0);
  EXPECT_EQ(node->ExitCode(), 0);
}

TEST_F(VisionNodeTest, RefusesAnUnknownFit) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"fit", "stretch"}});
  EXPECT_THROW(VisionNode node(options), std::invalid_argument);
}

}  // namespace
}  // namespace tron_vision
