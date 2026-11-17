#include <dlfcn.h>
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mowbite_msgs/msg/camera.hpp>
#include <mowbite_msgs/msg/detections.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "test_util.h"
#include "tron_vision/vision_node.h"

namespace tron_vision {
namespace {

using mowbite_msgs::msg::Camera;

// A gray camera at 15 fps in real time.
class GrayCamera : public tron_vision::Camera {
 public:
  bool Start(std::string&) override {
    return true;
  }
  void Stop() override {}
  int Next(int timeout_ms, Nv12View& frame, std::string&) override {
    const int wait_ms = std::min(timeout_ms, 66);
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    if (wait_ms < 66) return 0;
    frame = test::View(frame_);
    return 1;
  }
  bool SetExposure(const ExposureSetting&, std::string&) override {
    return true;
  }

 private:
  test::Nv12Frame frame_{320, 180, 90, 128, 128};
};

// Spins a listener until `done` or 10 s.
void SpinUntil(const rclcpp::Node::SharedPtr& listener, const std::function<bool()>& done) {
  rclcpp::executors::SingleThreadedExecutor spinner;
  spinner.add_node(listener);
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!done() && std::chrono::steady_clock::now() < end) {
    spinner.spin_some(std::chrono::milliseconds(100));
  }
}

// The core mask the stand-in runtime got for the highest open context, -1 without one.
int OpenCoreMask() {
  void* lib = dlopen(FAKE_NPU_LIB, RTLD_NOW | RTLD_NOLOAD);
  if (lib == nullptr) return -1;
  auto core_mask = reinterpret_cast<int (*)(abi::Context)>(dlsym(lib, "fake_core_mask"));
  int mask = -1;
  for (abi::Context c = 1; core_mask != nullptr && c < 100; c++) {
    if (core_mask(c) >= 0) mask = core_mask(c);
  }
  dlclose(lib);
  return mask;
}

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
  const std::regex jpeg("/camera/[a-z0-9_]+/jpeg(_plain)?");
  const auto graph = node->get_node_graph_interface();
  const auto publishers = graph->get_publisher_names_and_types_by_node("tron_vision", "/");
  EXPECT_TRUE(publishers.count("/bridge/sensor_value"));
  EXPECT_TRUE(publishers.count("/vision/detections"));
  EXPECT_TRUE(publishers.count("/camera/oa_left/jpeg"));
  EXPECT_TRUE(publishers.count("/camera/oa_left/jpeg_plain"));
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

// One entry on hw/cameras, latched: without the files the camera is not set up, the picture size follows the limit.
TEST_F(VisionNodeTest, ReportsTheCameraLatched) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"process", "vision"}, {"rtsp_port", 0}});
  auto node = std::make_shared<VisionNode>(options);
  auto listener = std::make_shared<rclcpp::Node>("listener");
  std::vector<mowbite_msgs::msg::Camera> got;
  rclcpp::executors::SingleThreadedExecutor spinner;
  spinner.add_node(listener);
  // Subscribed late on purpose: the entry has to come from the history.
  spinner.spin_some(std::chrono::milliseconds(1500));
  auto sub = listener->create_subscription<mowbite_msgs::msg::Camera>(
      "hw/cameras", rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local(),
      [&](const mowbite_msgs::msg::Camera& m) { got.push_back(m); });
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (got.empty() && std::chrono::steady_clock::now() < end) {
    spinner.spin_some(std::chrono::milliseconds(100));
  }
  ASSERT_FALSE(got.empty());
  const auto& c = got.back();
  EXPECT_EQ(c.id, "oa_left");
  EXPECT_EQ(c.process, "vision");
  EXPECT_EQ(c.kind, mowbite_msgs::msg::Camera::KIND_OBSTACLE);
  EXPECT_EQ(c.name, "Left camera");
  EXPECT_EQ(c.position, mowbite_msgs::msg::Camera::POSITION_LEFT);
  EXPECT_EQ(c.width, 640U);
  EXPECT_EQ(c.height, 360U);
  EXPECT_TRUE(c.detections);
  EXPECT_EQ(c.protocols, std::vector<std::string>{"mjpeg"});
  EXPECT_FALSE(c.available);
  EXPECT_EQ(c.reason, "NOT_SET_UP");
  EXPECT_EQ(c.detail, "model_missing: path not set");
}

TEST(CameraAvailability, ReasonsOfTheStates) {
  EXPECT_TRUE(CameraAvailability(VisionState::kRunning).available);
  EXPECT_TRUE(CameraAvailability(VisionState::kStarting).available);
  EXPECT_EQ(CameraAvailability(VisionState::kRunning).reason, "");
  EXPECT_EQ(CameraAvailability(VisionState::kLocked).reason, "LOCKED_AFTER_CRASH");
  EXPECT_EQ(CameraAvailability(VisionState::kModelMismatch).reason, "NOT_SET_UP");
  EXPECT_EQ(CameraAvailability(VisionState::kPausedHot).reason, "TOO_HOT");
  for (VisionState s : {VisionState::kOff, VisionState::kNoFrames, VisionState::kCameraError, VisionState::kFailed}) {
    EXPECT_FALSE(CameraAvailability(s).available);
    EXPECT_EQ(CameraAvailability(s).reason, "NO_SIGNAL");
  }
}

TEST_F(VisionNodeTest, RefusesAnUnknownFit) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"fit", "stretch"}});
  EXPECT_THROW(VisionNode node(options), std::invalid_argument);
}

// The right instance: camera, sensors and hardware follow camera_id and sensor_suffix.
TEST_F(VisionNodeTest, RightInstanceReportsItsCameraAndSensors) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {{"process", "right"}, {"camera_id", "oa_right"}, {"sensor_suffix", "_right"}, {"video", false}});
  auto node = std::make_shared<VisionNode>(options);
  EXPECT_EQ(node->get_parameter("sensor_name").as_string(), "m00_b_gc2093");
  EXPECT_EQ(node->get_parameter("video_device").as_string(), "/dev/video44");
  auto listener = std::make_shared<rclcpp::Node>("listener");
  std::vector<Camera> cameras;
  std::map<std::string, std::string> names;
  std::map<std::string, std::string> texts;
  auto info = listener->create_subscription<mowbite_msgs::msg::SensorSpec>(
      "bridge/sensor_info", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [&](const mowbite_msgs::msg::SensorSpec& m) { names[m.id] = m.name; });
  auto values = listener->create_subscription<mowbite_msgs::msg::SensorValue>(
      "bridge/sensor_value", 100, [&](const mowbite_msgs::msg::SensorValue& m) { texts[m.sensor_id] = m.text; });
  auto entries = listener->create_subscription<Camera>(
      "hw/cameras", rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local(),
      [&](const Camera& m) { cameras.push_back(m); });
  SpinUntil(listener, [&] { return names.size() == 3 && texts.count("vision_right_state") && !cameras.empty(); });
  EXPECT_EQ(names["vision_right_state"], "Camera AI right");
  EXPECT_EQ(names["vision_right_last"], "Camera AI right Detections");
  EXPECT_EQ(names["vision_right_rate"], "Camera AI right Rate");
  EXPECT_EQ(texts["vision_right_state"], "model_missing: path not set");
  EXPECT_FALSE(texts.count("vision_state"));
  ASSERT_FALSE(cameras.empty());
  EXPECT_EQ(cameras.back().id, "oa_right");
  EXPECT_EQ(cameras.back().name, "Right camera");
  EXPECT_EQ(cameras.back().position, Camera::POSITION_RIGHT);
  EXPECT_EQ(cameras.back().process, "right");
  EXPECT_TRUE(cameras.back().detections);
}

// Each instance runs the model on its own NPU core and names its camera in the detections.
class VisionNodeDetects : public VisionNodeTest, public ::testing::WithParamInterface<std::tuple<std::string, int>> {};

TEST_P(VisionNodeDetects, OnItsCoreForItsCamera) {
  const auto& [camera_id, core] = GetParam();
  test::TempDir dir;
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"camera_id", camera_id},
                               {"npu_core", core},
                               {"library_path", std::string(FAKE_NPU_LIB)},
                               {"model_path", dir.Write("m.rknn", "classes 3\nhit 0 0.8 8 30 40 2 2 2 2\n")},
                               {"labels_path", dir.Write("labels.txt", "person\ndog\ncat\n")},
                               {"crash_dir", dir.Path() + "/vision"},
                               {"max_rss_mb", 8192},
                               {"video", false}});
  auto node = std::make_shared<VisionNode>(options, std::make_unique<GrayCamera>());
  auto listener = std::make_shared<rclcpp::Node>("listener");
  std::vector<mowbite_msgs::msg::Detections> got;
  auto sub = listener->create_subscription<mowbite_msgs::msg::Detections>(
      "vision/detections", 5, [&](const mowbite_msgs::msg::Detections& m) { got.push_back(m); });
  SpinUntil(listener, [&] { return !got.empty(); });
  ASSERT_FALSE(got.empty());
  EXPECT_EQ(got.back().camera_id, camera_id);
  EXPECT_EQ(got.back().header.frame_id, camera_id);
  ASSERT_EQ(got.back().detections.size(), 1U);
  EXPECT_EQ(got.back().detections[0].class_name, "person");
  // The runtime takes one bit per core.
  EXPECT_EQ(OpenCoreMask(), 1 << core);
}

INSTANTIATE_TEST_SUITE_P(LeftAndRight, VisionNodeDetects,
                         ::testing::Values(std::make_tuple(std::string("oa_left"), 0),
                                           std::make_tuple(std::string("oa_right"), 1)));

// One section for both instances, the right one in its namespace adds its own values on top.
TEST_F(VisionNodeTest, ParameterFileWithASharedAndARightSection) {
  test::TempDir dir;
  const std::string file = dir.Write("tron.yaml",
                                     "/**/tron_vision:\n"
                                     "  ros__parameters:\n"
                                     "    labels_path: /shared/labels.txt\n"
                                     "    infer_rate: 1.5\n"
                                     "    video: false\n"
                                     "/right/tron_vision:\n"
                                     "  ros__parameters:\n"
                                     "    camera_id: oa_right\n"
                                     "    npu_core: 1\n"
                                     "    rtsp_port: 8557\n"
                                     "    sensor_suffix: _right\n");
  rclcpp::NodeOptions left_options;
  left_options.arguments({"--ros-args", "--params-file", file});
  rclcpp::NodeOptions right_options;
  right_options.arguments({"--ros-args", "--params-file", file, "-r", "__ns:=/right"});
  auto left = std::make_shared<VisionNode>(left_options);
  auto right = std::make_shared<VisionNode>(right_options);
  EXPECT_EQ(right->get_fully_qualified_name(), std::string("/right/tron_vision"));
  for (const auto& node : {left, right}) {
    EXPECT_EQ(node->get_parameter("labels_path").as_string(), "/shared/labels.txt");
    EXPECT_EQ(node->get_parameter("infer_rate").as_double(), 1.5);
  }
  EXPECT_EQ(left->get_parameter("camera_id").as_string(), "oa_left");
  EXPECT_EQ(left->get_parameter("npu_core").as_int(), 0);
  EXPECT_EQ(left->get_parameter("rtsp_port").as_int(), 8556);
  EXPECT_EQ(left->get_parameter("sensor_suffix").as_string(), "");
  EXPECT_EQ(right->get_parameter("camera_id").as_string(), "oa_right");
  EXPECT_EQ(right->get_parameter("npu_core").as_int(), 1);
  EXPECT_EQ(right->get_parameter("rtsp_port").as_int(), 8557);
  EXPECT_EQ(right->get_parameter("sensor_suffix").as_string(), "_right");
}

TEST_F(VisionNodeTest, RefusesAnUnknownCameraOrSuffix) {
  for (const auto& [name, value] : std::vector<std::pair<std::string, std::string>>{
           {"camera_id", "oa_rear"}, {"sensor_suffix", "right"}, {"sensor_suffix", "_Right"}}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{name, value}, {"video", false}});
    EXPECT_THROW(VisionNode node(options), std::invalid_argument) << name << " " << value;
  }
}

}  // namespace
}  // namespace tron_vision
