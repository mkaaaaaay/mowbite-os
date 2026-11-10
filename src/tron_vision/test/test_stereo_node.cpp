#include <gtest/gtest.h>
#include <linux/videodev2.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <mowbite_msgs/msg/camera.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "fake_video_device.h"
#include "mowbite_watch/watch.h"
#include "rtsp_test_client.h"
#include "test_util.h"
#include "tron_vision/stereo_node.h"

namespace tron_vision {
namespace {

using mowbite_msgs::msg::Camera;
using mowbite_msgs::msg::HardwareStatus;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

// One ROS context for all tests: once ROS has shut down, the watch of the process only counts its shutdown limit.
class StereoNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }
};

uint16_t FreePort() {
  RtspConfig config;
  config.port = 0;
  RtspServer probe(config, [] { return 0.0; });
  std::string error;
  return probe.Listen(error) ? probe.Port() : 0;
}

rclcpp::NodeOptions Options(const std::string& crash_dir, uint16_t port, const std::string& library) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"crash_dir", crash_dir},
                               {"rtsp_port", static_cast<int64_t>(port)},
                               {"encoder_library", library},
                               {"process", "stereo"}});
  return options;
}

// Spins the nodes in a thread of its own and keeps the entries of hw/cameras.
class Spinner {
 public:
  Spinner() : listener_(std::make_shared<rclcpp::Node>("listener")) {
    sub_ = listener_->create_subscription<Camera>("hw/cameras",
                                                  rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local(),
                                                  [this](const Camera& m) {
                                                    std::lock_guard<std::mutex> lock(mutex_);
                                                    cameras_.push_back(m);
                                                  });
    spinning_.add_node(listener_);
  }
  ~Spinner() {
    spinning_.cancel();
    if (thread_.joinable()) thread_.join();
  }
  void Add(const rclcpp::Node::SharedPtr& node) {
    spinning_.add_node(node);
  }
  void Start() {
    thread_ = std::thread([this] { spinning_.spin(); });
  }
  rclcpp::Node& Listener() {
    return *listener_;
  }
  // Waits until the newest entry meets the condition.
  bool WaitForCamera(const std::function<bool(const Camera&)>& meets, double seconds, Camera* got = nullptr) {
    const auto end = Clock::now() + std::chrono::duration<double>(seconds);
    while (Clock::now() < end) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!cameras_.empty() && meets(cameras_.back())) {
          if (got) *got = cameras_.back();
          return true;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

 private:
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<Camera>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor spinning_;
  std::mutex mutex_;
  std::vector<Camera> cameras_;
  std::thread thread_;
};

// hw/status at 10 Hz as tron_mcu sends it, and once more at once when the blade changes.
class Driver {
 public:
  explicit Driver(rclcpp::Node& node, bool blade = false)
      : pub_(node.create_publisher<HardwareStatus>("hw/status", 10)), blade_(blade) {
    thread_ = std::thread([this] {
      while (running_) {
        if (sending_) Send();
        std::this_thread::sleep_for(100ms);
      }
    });
  }
  ~Driver() {
    running_ = false;
    thread_.join();
  }
  void Blade(bool on) {
    blade_ = on;
    Send();
  }
  void Stop() {
    sending_ = false;
  }

 private:
  void Send() {
    HardwareStatus msg;
    msg.telemetry_ok = true;
    msg.blade_on = blade_;
    pub_->publish(msg);
  }

  rclcpp::Publisher<HardwareStatus>::SharedPtr pub_;
  std::atomic<bool> blade_{false};
  std::atomic<bool> sending_{true};
  std::atomic<bool> running_{true};
  std::thread thread_;
};

// The driver model with a frame every 40 ms of real time; notes when the stream starts and stops, for other threads.
class TimedDevice : public test::FakeVideoDevice {
 public:
  TimedDevice() {
    frame_period_us = 40000;
    on_wait = [this](int timeout_ms) {
      if (hang) std::this_thread::sleep_for(30s);
      if (!streaming) return;
      const auto now = Clock::now();
      if (due_ < now - 1s) due_ = now + 40ms;
      std::this_thread::sleep_for(std::min<Clock::duration>(due_ - now, std::chrono::milliseconds(timeout_ms)));
      if (Clock::now() >= due_) {
        Push(0x80, buffer_bytes);
        due_ += 40ms;
      }
    };
  }
  int Ioctl(int fd, unsigned long request, void* arg) override {
    const int result = FakeVideoDevice::Ioctl(fd, request, arg);
    if (request == VIDIOC_STREAMON && result == 0) ons++;
    if (request == VIDIOC_STREAMOFF && result == 0) {
      off_at = Clock::now().time_since_epoch().count();
      offs++;
    }
    return result;
  }

  std::atomic<int> ons{0};
  std::atomic<int> offs{0};
  std::atomic<Clock::rep> off_at{0};
  std::atomic<bool> hang{false};

 private:
  Clock::time_point due_{};
};

// A configured stereo camera on the driver model and the stand-in media library.
struct Rig {
  Rig() {
    std::filesystem::create_directories(dir.Path() + "/sys/video22");
    std::filesystem::create_directories(dir.Path() + "/sys/devices/rkcif-mipi-lvds3");
    std::filesystem::create_directories(dir.Path() + "/markers");
    dir.Write("sys/video22/name", "stream_cif_mipi_id0\n");
    std::filesystem::create_directory_symlink("../devices/rkcif-mipi-lvds3", dir.Path() + "/sys/video22/device");
    dir.Write("stereo", "configured\n");
    dir.Write("boot_id", "6b0a57c2-0000-4000-8000-00000000b007\n");
    // The memory of the test process is not the node's.
    dir.Write("statm", "30000 5000 900 1 0 20000 0\n");
    seams.device = &device;
    seams.crash_flag = dir.Path() + "/crash";
    seams.adjust = [this](StereoConfig& config, StereoCameraConfig& camera) {
      config.setup_file = dir.Path() + "/stereo";
      config.boot_id_file = dir.Path() + "/boot_id";
      config.statm = dir.Path() + "/statm";
      camera.sysfs = dir.Path() + "/sys";
      camera.processes_using = nullptr;
    };
  }
  std::shared_ptr<StereoNode> Make(bool stream_while_mowing = false) {
    rclcpp::NodeOptions options = Options(dir.Path() + "/markers", port, FAKE_MPP_LIB);
    if (stream_while_mowing) options.append_parameter_override("stream_while_mowing", true);
    return std::make_shared<StereoNode>(options, seams);
  }
  std::string Url(const std::string& rest = "") const {
    return "rtsp://127.0.0.1:" + std::to_string(port) + "/stereo" + rest;
  }
  // DESCRIBE, SETUP and PLAY against the node's loop thread.
  bool Play(const test::Client& client) const {
    const auto ask = [&client](const std::string& request) {
      client.Send(request);
      std::string answer;
      for (int i = 0; i < 20 && answer.find("\r\n\r\n") == std::string::npos; i++) answer += client.Receive();
      return answer;
    };
    if (ask("DESCRIBE " + Url() + " RTSP/1.0\r\nCSeq: 1\r\n\r\n").find("200 OK") == std::string::npos) return false;
    const std::string session = test::SessionOf(
        ask("SETUP " + Url("/trackID=0") + " RTSP/1.0\r\nCSeq: 2\r\nTransport: RTP/AVP/TCP;interleaved=0-1\r\n\r\n"));
    return !session.empty() &&
           ask("PLAY " + Url("/") + " RTSP/1.0\r\nCSeq: 3\r\nSession: " + session + "\r\n\r\n").find("200 OK") !=
               std::string::npos;
  }

  test::TempDir dir;
  TimedDevice device;
  StereoNodeSeams seams;
  uint16_t port = FreePort();
};

// Display only: the process publishes its state and its camera, subscribes to hw/status alone and calls nothing.
TEST_F(StereoNodeTest, TopicsOfTheDesignOnly) {
  test::TempDir dir;
  auto node = std::make_shared<StereoNode>(Options(dir.Path(), FreePort(), "/nonexistent/librockchip_mpp.so.1"));
  const std::set<std::string> allowed = {"/bridge/sensor_info", "/bridge/sensor_value", "/hw/cameras", "/rosout",
                                         "/parameter_events"};
  const auto graph = node->get_node_graph_interface();
  const auto publishers = graph->get_publisher_names_and_types_by_node("tron_stereo", "/");
  EXPECT_TRUE(publishers.count("/hw/cameras"));
  EXPECT_TRUE(publishers.count("/bridge/sensor_value"));
  for (const auto& [topic, types] : publishers) EXPECT_TRUE(allowed.count(topic)) << topic;
  const auto subscribers = graph->get_subscriber_names_and_types_by_node("tron_stereo", "/");
  EXPECT_TRUE(subscribers.count("/hw/status"));
  for (const auto& [topic, types] : subscribers) {
    EXPECT_TRUE(topic == "/hw/status" || topic == "/parameter_events") << topic;
  }
  EXPECT_TRUE(graph->get_client_names_and_types_by_node("tron_stereo", "/").empty());
  // Best effort: no feedback to the driver.
  const auto infos = node->get_subscriptions_info_by_topic("/hw/status");
  ASSERT_EQ(infos.size(), 1U);
  EXPECT_EQ(infos[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  // The blade lock holds unless the platform lifts it.
  EXPECT_FALSE(node->get_parameter("stream_while_mowing").as_bool());
}

// One entry on hw/cameras, latched; without the media library the camera is not set up.
TEST_F(StereoNodeTest, ReportsTheCameraLatched) {
  test::TempDir dir;
  const uint16_t port = FreePort();
  auto node = std::make_shared<StereoNode>(Options(dir.Path(), port, "/nonexistent/librockchip_mpp.so.1"));
  rclcpp::executors::SingleThreadedExecutor alone;
  alone.add_node(node);
  alone.spin_some(500ms);
  alone.remove_node(node);
  // Subscribed late on purpose: the entry has to come from the history.
  Spinner spinner;
  spinner.Add(node);
  spinner.Start();
  Camera c;
  ASSERT_TRUE(spinner.WaitForCamera([](const Camera&) { return true; }, 10.0, &c));
  EXPECT_EQ(c.id, "stereo");
  EXPECT_EQ(c.process, "stereo");
  EXPECT_EQ(c.kind, Camera::KIND_STEREO);
  EXPECT_EQ(c.width, 640U);
  EXPECT_EQ(c.height, 480U);
  EXPECT_FALSE(c.detections);
  EXPECT_EQ(c.protocols, std::vector<std::string>{Camera::PROTOCOL_WEBRTC});
  EXPECT_EQ(c.source, "rtsp://127.0.0.1:" + std::to_string(port) + "/stereo");
  EXPECT_FALSE(c.available);
  EXPECT_EQ(c.reason, Camera::REASON_NOT_SET_UP);
  EXPECT_EQ(c.detail.rfind("encoder_missing: ", 0), 0U) << c.detail;
}

TEST_F(StereoNodeTest, TakenPortIsNoSignal) {
  test::TempDir dir;
  RtspConfig taken;
  taken.port = 0;
  RtspServer other(taken, [] { return 0.0; });
  std::string error;
  ASSERT_TRUE(other.Listen(error));
  auto node = std::make_shared<StereoNode>(Options(dir.Path(), other.Port(), "/nonexistent/librockchip_mpp.so.1"));
  Spinner spinner;
  spinner.Add(node);
  spinner.Start();
  Camera c;
  ASSERT_TRUE(spinner.WaitForCamera([](const Camera&) { return true; }, 10.0, &c));
  EXPECT_FALSE(c.available);
  EXPECT_EQ(c.reason, Camera::REASON_NO_SIGNAL);
  EXPECT_EQ(c.detail.rfind("failed: RTSP port " + std::to_string(other.Port()), 0), 0U) << c.detail;
}

TEST_F(StereoNodeTest, RefusesAPortOutOfRange) {
  test::TempDir dir;
  EXPECT_THROW(StereoNode node(Options(dir.Path(), 0, "")), std::invalid_argument);
}

// Over DDS: the stream ends a few tenths of a second after hw/status says the blade runs, and when hw/status stops.
TEST_F(StereoNodeTest, BladeAndSilentDriverEndTheStream) {
  Rig rig;
  auto node = rig.Make();
  Spinner spinner;
  spinner.Add(node);
  Driver driver(spinner.Listener());
  spinner.Start();
  ASSERT_TRUE(spinner.WaitForCamera([](const Camera& c) { return c.available; }, 15.0));
  EXPECT_EQ(rig.device.ons, 1);
  test::Client client(rig.port);
  ASSERT_TRUE(rig.Play(client));
  for (int i = 0; i < 100 && rig.device.ons < 2; i++) std::this_thread::sleep_for(10ms);
  ASSERT_EQ(rig.device.ons, 2);
  std::this_thread::sleep_for(500ms);
  ASSERT_EQ(rig.device.offs, 1);

  const auto blade_at = Clock::now();
  driver.Blade(true);
  for (int i = 0; i < 200 && rig.device.offs < 2; i++) std::this_thread::sleep_for(1ms);
  ASSERT_EQ(rig.device.offs, 2);
  const double took = std::chrono::duration<double>(Clock::duration(rig.device.off_at) - blade_at.time_since_epoch()).count();
  std::printf("blade_on on hw/status to STREAMOFF: %.3f s\n", took);
  EXPECT_LT(took, 0.3);
  Camera c;
  ASSERT_TRUE(spinner.WaitForCamera([](const Camera& m) { return !m.available; }, 2.0, &c));
  EXPECT_EQ(c.reason, Camera::REASON_BLADE_RUNNING);

  // Blade off: available again without another self-test.
  driver.Blade(false);
  ASSERT_TRUE(spinner.WaitForCamera([](const Camera& m) { return m.available; }, 2.0));
  EXPECT_EQ(rig.device.ons, 2);

  // hw/status stops: not available within the age limit and a step.
  const auto silent_at = Clock::now();
  driver.Stop();
  ASSERT_TRUE(spinner.WaitForCamera([](const Camera& m) { return !m.available; }, 3.0, &c));
  EXPECT_EQ(c.reason, Camera::REASON_DRIVER_MISSING);
  EXPECT_LT(std::chrono::duration<double>(Clock::now() - silent_at).count(), 1.0 + 0.2 + 0.3);
}

// Reads the stream for this long; the bytes that came, 0 once the server has closed.
size_t ReadStream(const test::Client& client, std::chrono::milliseconds time) {
  size_t bytes = 0;
  const auto end = Clock::now() + time;
  while (Clock::now() < end) {
    const std::string got = client.Receive();
    if (got == "<closed>") return 0;
    bytes += got.size();
  }
  return bytes;
}

// stream_while_mowing: the stream goes on next to the blade and without hw/status; the self-test still waits for
// the blade to stop.
TEST_F(StereoNodeTest, StreamWhileMowingKeepsTheStream) {
  Rig rig;
  auto node = rig.Make(true);
  Spinner spinner;
  spinner.Add(node);
  Driver driver(spinner.Listener(), true);
  spinner.Start();
  ASSERT_TRUE(
      spinner.WaitForCamera([](const Camera& m) { return m.reason == Camera::REASON_BLADE_RUNNING; }, 5.0));
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(rig.device.ons, 0);

  driver.Blade(false);
  ASSERT_TRUE(spinner.WaitForCamera([](const Camera& m) { return m.available; }, 15.0));
  EXPECT_EQ(rig.device.ons, 1);
  test::Client client(rig.port);
  ASSERT_TRUE(rig.Play(client));
  for (int i = 0; i < 100 && rig.device.ons < 2; i++) std::this_thread::sleep_for(10ms);
  ASSERT_EQ(rig.device.ons, 2);

  driver.Blade(true);
  EXPECT_GT(ReadStream(client, 1000ms), 0U);
  // Longer than the age limit of hw/status.
  driver.Stop();
  EXPECT_GT(ReadStream(client, 2000ms), 0U);
  EXPECT_EQ(rig.device.offs, 1);
  EXPECT_TRUE(spinner.WaitForCamera([](const Camera& m) { return m.available; }, 0.1));
}

// A loop thread that hangs in the camera ends the process with the code of the watch.
TEST_F(StereoNodeTest, LoopThatHangsEndsTheProcess) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        Rig rig;
        rig.device.hang = true;
        auto node = rig.Make();
        Spinner spinner;
        spinner.Add(node);
        Driver driver(spinner.Listener());
        spinner.Start();
        std::this_thread::sleep_for(10s);
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(mowbite_watch::kExitCode), "tron_stereo_loop silent");
}

TEST(StereoAvailability, ReasonsOfTheStates) {
  for (StereoState s : {StereoState::kReady, StereoState::kStreaming}) {
    EXPECT_TRUE(StereoAvailability(s).available);
    EXPECT_EQ(StereoAvailability(s).reason, "");
  }
  const std::vector<std::pair<StereoState, std::string>> reasons = {
      {StereoState::kPausedBlade, "BLADE_RUNNING"},     {StereoState::kPausedDriver, "DRIVER_MISSING"},
      {StereoState::kWaitingIdle, "SELF_TEST"},         {StereoState::kSelftest, "SELF_TEST"},
      {StereoState::kWaitingSetup, "NOT_SET_UP"},       {StereoState::kNotConfigured, "NOT_SET_UP"},
      {StereoState::kEncoderMissing, "NOT_SET_UP"},     {StereoState::kLocked, "LOCKED_AFTER_CRASH"},
      {StereoState::kPausedHot, "TOO_HOT"},             {StereoState::kOff, "NO_SIGNAL"},
      {StereoState::kNoFrames, "NO_SIGNAL"},            {StereoState::kStalled, "NO_SIGNAL"},
      {StereoState::kEncoderError, "NO_SIGNAL"},        {StereoState::kFailed, "NO_SIGNAL"}};
  for (const auto& [state, reason] : reasons) {
    EXPECT_FALSE(StereoAvailability(state).available) << StateName(state);
    EXPECT_EQ(StereoAvailability(state).reason, reason) << StateName(state);
  }
}

}  // namespace
}  // namespace tron_vision
