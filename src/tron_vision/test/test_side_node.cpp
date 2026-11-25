#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <mowbite_msgs/msg/camera.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <set>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>

#include "mowbite_watch/watch.h"
#include "test_util.h"
#include "tron_vision/side_node.h"

namespace tron_vision {
namespace {

using mowbite_msgs::msg::Camera;

// A camera whose start never comes back.
class HangingCamera : public tron_vision::Camera {
 public:
  bool Start(std::string&) override {
    std::this_thread::sleep_for(std::chrono::seconds(30));
    return true;
  }
  void Stop() override {}
  int Next(int, Nv12View&, std::string&) override {
    return 0;
  }
  bool SetExposure(const ExposureSetting&, std::string&) override {
    return true;
  }
};

// Spins a node in a thread of its own while it lives, so that the watch of the process sees its executor.
class Spin {
 public:
  explicit Spin(const rclcpp::Node::SharedPtr& node) {
    spinner_.add_node(node);
    // Rounds instead of spin(): a cancel before spin() has begun would be lost.
    thread_ = std::thread([this] {
      while (!stop_) spinner_.spin_once(std::chrono::milliseconds(50));
    });
  }
  ~Spin() {
    stop_ = true;
    thread_.join();
  }
  Spin(const Spin&) = delete;
  Spin& operator=(const Spin&) = delete;

 private:
  rclcpp::executors::SingleThreadedExecutor spinner_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

// One ROS context for all tests: once ROS has shut down, the watch of the process only counts its shutdown limit.
class SideNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }
  // The process name platform.py of the Tron gives, a crash directory of the test, and a memory limit above what the test
  // process itself takes, also under a sanitizer.
  rclcpp::NodeOptions Options() const {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"process", "right"},
                                 {"crash_dir", dir_.Path()},
                                 {"max_rss_mb", 8192},
                                 {"video", video},
                                 {"rtsp_port", 0},
                                 {"encoder_library", std::string(FAKE_MPP_LIB)}});
    return options;
  }

  // Waits for the entry on hw/cameras that `done` accepts, subscribed late: it has to come from the history.
  static std::vector<Camera> Entries(const std::function<bool(const Camera&)>& done) {
    auto listener = std::make_shared<rclcpp::Node>("listener");
    std::vector<Camera> got;
    rclcpp::executors::SingleThreadedExecutor spinner;
    spinner.add_node(listener);
    spinner.spin_some(std::chrono::milliseconds(1500));
    auto sub = listener->create_subscription<Camera>(
        "hw/cameras", rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local(),
        [&](const Camera& m) { got.push_back(m); });
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while ((got.empty() || !done(got.back())) && std::chrono::steady_clock::now() < end) {
      spinner.spin_some(std::chrono::milliseconds(100));
    }
    return got;
  }

  bool video = false;
  std::string Dir() const {
    return dir_.Path();
  }

 private:
  test::TempDir dir_;
};

// Display only: the process publishes the camera and its pictures and the defaults of every node, and listens to
// nothing of the stack.
TEST_F(SideNodeTest, PublishesOnlyTheCameraAndItsPictures) {
  auto node = std::make_shared<SideNode>(Options());
  Spin spin(node);
  const std::set<std::string> allowed = {"/hw/cameras", "/camera/oa_right/jpeg", "/rosout", "/parameter_events"};
  const auto graph = node->get_node_graph_interface();
  const auto publishers = graph->get_publisher_names_and_types_by_node("tron_side", "/");
  EXPECT_TRUE(publishers.count("/hw/cameras"));
  EXPECT_TRUE(publishers.count("/camera/oa_right/jpeg"));
  for (const auto& [topic, types] : publishers) EXPECT_TRUE(allowed.count(topic)) << topic;
  // The clock of every node follows use_sim_time over /parameter_events.
  for (const auto& [topic, types] : graph->get_subscriber_names_and_types_by_node("tron_side", "/")) {
    EXPECT_EQ(topic, "/parameter_events");
  }
  EXPECT_TRUE(graph->get_client_names_and_types_by_node("tron_side", "/").empty());
  const auto cameras = node->get_publishers_info_by_topic("/hw/cameras");
  ASSERT_EQ(cameras.size(), 1U);
  EXPECT_EQ(cameras[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(cameras[0].qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
  const auto pictures = node->get_publishers_info_by_topic("/camera/oa_right/jpeg");
  ASSERT_EQ(pictures.size(), 1U);
  EXPECT_EQ(pictures[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
}

// One entry on hw/cameras, latched: without a viewer the camera is off and can stream.
TEST_F(SideNodeTest, ReportsTheCameraLatched) {
  auto node = std::make_shared<SideNode>(Options());
  Spin spin(node);
  const std::vector<Camera> got = Entries([](const Camera&) { return true; });
  ASSERT_FALSE(got.empty());
  const auto& c = got.back();
  EXPECT_EQ(c.id, "oa_right");
  EXPECT_EQ(c.name, "Right camera");
  EXPECT_EQ(c.process, "right");
  EXPECT_EQ(c.kind, Camera::KIND_OBSTACLE);
  EXPECT_EQ(c.position, Camera::POSITION_RIGHT);
  EXPECT_EQ(c.width, 640U);
  EXPECT_EQ(c.height, 360U);
  EXPECT_FALSE(c.detections);
  EXPECT_EQ(c.protocols, std::vector<std::string>{"mjpeg"});
  EXPECT_TRUE(c.available);
  EXPECT_EQ(c.reason, "");
  EXPECT_EQ(node->ExitCode(), 0);
}

// A camera locked after a crash comes back over camera/<id>/unlock, without a restart of the process.
TEST_F(SideNodeTest, UnlockOverTheServiceFreesTheCamera) {
  std::ofstream(Dir() + "/locked-oa_right") << "1700000000 Oops\n";
  auto node = std::make_shared<SideNode>(Options());
  Spin spin(node);
  auto locked = Entries([](const Camera& c) { return c.reason == Camera::REASON_LOCKED_AFTER_CRASH; });
  ASSERT_FALSE(locked.empty());
  ASSERT_EQ(locked.back().reason, Camera::REASON_LOCKED_AFTER_CRASH);

  auto caller = std::make_shared<rclcpp::Node>("caller");
  auto client = caller->create_client<std_srvs::srv::Trigger>("/camera/oa_right/unlock");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  auto call = [&] {
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    EXPECT_EQ(rclcpp::spin_until_future_complete(caller, future, std::chrono::seconds(5)),
              rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  };
  const auto first = call();
  EXPECT_TRUE(first->success);
  EXPECT_EQ(first->message, "1700000000 Oops");
  const auto freed = Entries([](const Camera& c) { return c.available; });
  ASSERT_FALSE(freed.empty());
  EXPECT_TRUE(freed.back().available);
  const auto second = call();
  EXPECT_FALSE(second->success);
  EXPECT_EQ(second->message, "not locked");
}

// With video the encoder is tried once at the start; then WebRTC comes first, from the RTSP server of the process.
TEST_F(SideNodeTest, OffersVideoOnceTheEncoderWorks) {
  video = true;
  auto node = std::make_shared<SideNode>(Options());
  Spin spin(node);
  const std::vector<Camera> got = Entries([](const Camera& c) { return !c.source.empty(); });
  ASSERT_FALSE(got.empty());
  const auto& c = got.back();
  EXPECT_EQ(c.protocols, (std::vector<std::string>{"webrtc", "mjpeg"}));
  EXPECT_EQ(c.source.rfind("rtsp://127.0.0.1:", 0), 0U) << c.source;
  EXPECT_EQ(c.source.substr(c.source.size() - 9), "/oa_right");
  EXPECT_EQ(c.width, 1280U);
  EXPECT_EQ(c.height, 720U);
  EXPECT_TRUE(c.available);
  EXPECT_EQ(node->ExitCode(), 0);
}

// A loop thread that hangs in the camera, opened for a subscriber of the pictures, ends the process with the code of
// the watch.
TEST_F(SideNodeTest, LoopThatHangsEndsTheProcess) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        auto node = std::make_shared<SideNode>(Options(), std::make_unique<HangingCamera>());
        Spin spin(node);
        auto viewer = std::make_shared<rclcpp::Node>("viewer");
        auto sub = viewer->create_subscription<sensor_msgs::msg::CompressedImage>(
            "camera/oa_right/jpeg", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            [](const sensor_msgs::msg::CompressedImage&) {});
        Spin spin_viewer(viewer);
        std::this_thread::sleep_for(std::chrono::seconds(10));
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(mowbite_watch::kExitCode), "tron_side_loop silent");
}

TEST(SideCameraAvailability, ReasonsOfTheStates) {
  for (PreviewState s : {PreviewState::kOff, PreviewState::kStarting, PreviewState::kRunning}) {
    EXPECT_TRUE(CameraAvailability(s).available);
    EXPECT_EQ(CameraAvailability(s).reason, "");
  }
  EXPECT_EQ(CameraAvailability(PreviewState::kLocked).reason, "LOCKED_AFTER_CRASH");
  EXPECT_EQ(CameraAvailability(PreviewState::kPausedHot).reason, "TOO_HOT");
  for (PreviewState s : {PreviewState::kNoFrames, PreviewState::kCameraError, PreviewState::kFailed}) {
    EXPECT_FALSE(CameraAvailability(s).available);
    EXPECT_EQ(CameraAvailability(s).reason, "NO_SIGNAL");
  }
}

}  // namespace
}  // namespace tron_vision
