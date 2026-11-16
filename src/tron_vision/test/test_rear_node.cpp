#include <gtest/gtest.h>
#include <linux/videodev2.h>
#include <turbojpeg.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <filesystem>
#include <functional>
#include <memory>
#include <mowbite_msgs/msg/camera.hpp>
#include <mowbite_msgs/msg/dock_marker.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <set>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <thread>
#include <vector>

#include "dock_jpeg.h"
#include "fake_dma_heap.h"
#include "fake_video_device.h"
#include "mowbite_watch/watch.h"
#include "test_util.h"
#include "tron_vision/rear_node.h"

namespace tron_vision {
namespace {

using diagnostic_msgs::msg::DiagnosticArray;
using mowbite_msgs::msg::Camera;
using mowbite_msgs::msg::DockMarker;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

// One ROS context for all tests: once ROS has shut down, the watch of the process only counts its shutdown limit.
class RearNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }
};

// The marker half a meter behind the camera of the rear scene, as the camera sends it.
const std::vector<uint8_t>& MarkerJpeg() {
  static const std::vector<uint8_t> jpeg = [] {
    test::Scene s = test::RearScene();
    s.markers = {test::Behind(s, 0.5, 0.03, 0.1)};
    return test::EncodeJpeg(test::Render(s));
  }();
  return jpeg;
}

// The UVC camera with a frame every 33 ms of real time while it streams.
class TimedRearDevice : public test::FakeVideoDevice {
 public:
  TimedRearDevice() {
    caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    stride = 0;
    buffer_bytes = 1920 * 1080 * 2;
    frame_period_us = 33333;
    controls[V4L2_CID_EXPOSURE_AUTO] = {0, 3, V4L2_EXPOSURE_APERTURE_PRIORITY, 0};
    controls[V4L2_CID_EXPOSURE_ABSOLUTE] = {1, 5000, 313, 0};
    controls[V4L2_CID_GAIN] = {0, 63, 10, 0};
    on_wait = [this](int timeout_ms) {
      if (hang) std::this_thread::sleep_for(30s);
      if (!streaming) return;
      const auto now = Clock::now();
      if (due_ < now - 1s) due_ = now + 33ms;
      std::this_thread::sleep_for(std::min<Clock::duration>(due_ - now, std::chrono::milliseconds(timeout_ms)));
      if (Clock::now() >= due_) {
        // Taken one frame period ago on the monotonic clock, as the driver stamps it; PushBytes adds one period.
        timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count() -
                       2 * frame_period_us;
        PushBytes(MarkerJpeg());
        due_ += 33ms;
      }
    };
  }
  int Ioctl(int fd, unsigned long request, void* arg) override {
    const int result = FakeVideoDevice::Ioctl(fd, request, arg);
    if (request == VIDIOC_STREAMON && result == 0) ons++;
    if (request == VIDIOC_STREAMOFF && result == 0) offs++;
    return result;
  }

  std::atomic<int> ons{0};
  std::atomic<int> offs{0};
  std::atomic<bool> hang{false};

 private:
  Clock::time_point due_{};
};

// Spins the nodes in a thread of its own, keeps what comes on hw/dock_marker and tron_rear/marker_pixels and sends
// requests.
class Spinner {
 public:
  Spinner() : listener_(std::make_shared<rclcpp::Node>("listener")) {
    markers_sub_ = listener_->create_subscription<DockMarker>(
        "hw/dock_marker", rclcpp::QoS(rclcpp::KeepLast(100)).reliable(), [this](const DockMarker& m) {
          std::lock_guard<std::mutex> lock(mutex_);
          markers_.push_back({m, listener_->get_clock()->now()});
        });
    pixels_sub_ = listener_->create_subscription<DiagnosticArray>(
        "tron_rear/marker_pixels", rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(), [this](const DiagnosticArray& m) {
          std::lock_guard<std::mutex> lock(mutex_);
          pixels_.push_back(m);
        });
    request_pub_ = listener_->create_publisher<std_msgs::msg::Empty>(
        "hw/dock_marker_request", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
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
  void Request() {
    request_pub_->publish(std_msgs::msg::Empty());
  }
  // Requests every 0.5 s for this long.
  void RequestFor(std::chrono::milliseconds time) {
    const auto end = Clock::now() + time;
    while (Clock::now() < end) {
      Request();
      std::this_thread::sleep_for(500ms);
    }
  }
  bool WaitFor(const std::function<bool(const DockMarker&)>& meets, double seconds, DockMarker* got = nullptr) {
    const auto end = Clock::now() + std::chrono::duration<double>(seconds);
    while (Clock::now() < end) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!markers_.empty() && meets(markers_.back().msg)) {
          if (got) *got = markers_.back().msg;
          return true;
        }
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  struct Received {
    DockMarker msg;
    rclcpp::Time at;
  };
  std::vector<Received> Markers() {
    std::lock_guard<std::mutex> lock(mutex_);
    return markers_;
  }
  std::vector<DiagnosticArray> Pixels() {
    std::lock_guard<std::mutex> lock(mutex_);
    return pixels_;
  }

 private:
  rclcpp::Node::SharedPtr listener_;
  rclcpp::Subscription<DockMarker>::SharedPtr markers_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr pixels_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr request_pub_;
  rclcpp::executors::SingleThreadedExecutor spinning_;
  std::mutex mutex_;
  std::vector<Received> markers_;
  std::vector<DiagnosticArray> pixels_;
  std::thread thread_;
};

// The rear camera of the scene with its calibration as parameters.
struct Rig {
  Rig() {
    // Drawn here: in the loop thread it would take longer than the watch allows in a build with sanitizers.
    MarkerJpeg();
    std::filesystem::create_directories(dir.Path() + "/markers");
    dir.Write("statm", "30000 5000 900 1 0 20000 0\n");
    seams.device = &device;
    seams.heap = &heap;
    seams.crash_flag = dir.Path() + "/crash";
    seams.adjust = [this](RearConfig& config, RearCameraConfig& camera) {
      config.statm = dir.Path() + "/statm";
      camera.device = dir.Write("mowbite-rear", "");
      camera.processes_using = nullptr;
    };
  }
  rclcpp::NodeOptions Options(bool calibrated = true) const {
    const test::Scene s = test::RearScene();
    rclcpp::NodeOptions options;
    options.append_parameter_override("crash_dir", dir.Path() + "/markers");
    // As platform.py of the Tron names the process.
    options.append_parameter_override("process", "rear");
    options.append_parameter_override("video", video);
    options.append_parameter_override("rtsp_port", 0);
    options.append_parameter_override("encoder_library", std::string(FAKE_MPP_LIB));
    if (calibrated) {
      const LensModel& l = s.lens;
      const CameraMount& m = s.mount;
      const std::vector<double> lens = {l.fx, l.fy, l.cx, l.cy, l.k1, l.k2, l.p1, l.p2, l.k3};
      options.append_parameter_override("lens", lens);
      options.append_parameter_override("mount", std::vector<double>{m.x, m.y, m.z, m.yaw, m.pitch, m.roll});
      options.append_parameter_override("marker_side", s.side);
      options.append_parameter_override("marker_pattern", static_cast<int64_t>(test::kDockPattern));
    }
    return options;
  }
  std::shared_ptr<RearNode> Make(bool calibrated = true) {
    return std::make_shared<RearNode>(Options(calibrated), seams);
  }

  test::TempDir dir;
  TimedRearDevice device;
  test::FakeDmaHeap heap;
  RearNodeSeams seams;
  bool video = false;
};

// It measures and shows and nothing else: one subscription, four publishers, no client.
TEST_F(RearNodeTest, TopicsOfTheDesignOnly) {
  Rig rig;
  auto node = rig.Make();
  const std::set<std::string> allowed = {"/hw/dock_marker", "/tron_rear/marker_pixels", "/hw/cameras",
                                         "/camera/rear/jpeg", "/rosout", "/parameter_events"};
  const auto graph = node->get_node_graph_interface();
  const auto publishers = graph->get_publisher_names_and_types_by_node("tron_rear", "/");
  for (const std::string& topic : allowed) EXPECT_TRUE(publishers.count(topic)) << topic;
  for (const auto& [topic, types] : publishers) EXPECT_TRUE(allowed.count(topic)) << topic;
  const auto subscribers = graph->get_subscriber_names_and_types_by_node("tron_rear", "/");
  EXPECT_TRUE(subscribers.count("/hw/dock_marker_request"));
  for (const auto& [topic, types] : subscribers) {
    EXPECT_TRUE(topic == "/hw/dock_marker_request" || topic == "/parameter_events") << topic;
  }
  EXPECT_TRUE(graph->get_client_names_and_types_by_node("tron_rear", "/").empty());
  const auto out = node->get_publishers_info_by_topic("/hw/dock_marker");
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(out[0].qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
  const auto cameras = node->get_publishers_info_by_topic("/hw/cameras");
  ASSERT_EQ(cameras.size(), 1U);
  EXPECT_EQ(cameras[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(cameras[0].qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
  const auto pictures = node->get_publishers_info_by_topic("/camera/rear/jpeg");
  ASSERT_EQ(pictures.size(), 1U);
  EXPECT_EQ(pictures[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  const auto in = node->get_subscriptions_info_by_topic("/hw/dock_marker_request");
  ASSERT_EQ(in.size(), 1U);
  EXPECT_EQ(in[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(in[0].qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST_F(RearNodeTest, RefusesWrongLimits) {
  Rig rig;
  rclcpp::NodeOptions options = rig.Options();
  options.append_parameter_override("frame_gap_s", 1.0);
  EXPECT_THROW(RearNode node(options, rig.seams), std::invalid_argument);
  options = rig.Options();
  options.append_parameter_override("eval_rate", 0.0);
  EXPECT_THROW(RearNode node(options, rig.seams), std::invalid_argument);
}

// The recording is off by default; a wrong value of it leaves it off and the node runs.
TEST_F(RearNodeTest, RecordingOffByDefaultAndOnlyBelowRun) {
  struct Case {
    std::string dir;
    int64_t max_mb;
    double keep_min;
    size_t expect_bytes;
  };
  const std::vector<Case> cases = {
      {"", -1, 0.0, 0},
      {"/run/mowbite-os/rear-record", 200, 90.0, 200UL << 20},
      {"/run/mowbite-os/rear-record", 301, 90.0, 0},
      {"/run/mowbite-os/rear-record", -1, 90.0, 0},
      {"/userdata/rear-record", 200, 90.0, 0},
      {"/run/../userdata", 200, 90.0, 0},
      {"/run/mowbite-os/rear-record", 200, 0.0, 0},
  };
  for (const Case& c : cases) {
    Rig rig;
    RecordLimits seen;
    rig.seams.adjust = [&seen, adjust = rig.seams.adjust](RearConfig& config, RearCameraConfig& camera) {
      adjust(config, camera);
      seen = config.record;
    };
    rclcpp::NodeOptions options = rig.Options();
    if (!c.dir.empty()) {
      options.append_parameter_override("record_dir", c.dir);
      options.append_parameter_override("record_max_mb", c.max_mb);
      options.append_parameter_override("record_keep_min", c.keep_min);
    }
    EXPECT_NO_THROW(RearNode node(options, rig.seams)) << c.dir << " " << c.max_mb;
    EXPECT_EQ(seen.max_bytes, c.expect_bytes) << c.dir << " " << c.max_mb << " " << c.keep_min;
    if (c.expect_bytes > 0) {
      EXPECT_EQ(seen.dir, c.dir);
      EXPECT_DOUBLE_EQ(seen.keep_s, c.keep_min * 60.0);
    }
  }
}

// A value of the wrong type, as a device file may carry it, does not end the node: a whole number where a decimal one
// is meant and the other way round count, anything else leaves the recording off.
TEST_F(RearNodeTest, RecordingTakesWholeNumbersAndSurvivesWrongTypes) {
  for (const bool numbers : {true, false}) {
    Rig rig;
    RecordLimits seen;
    rig.seams.adjust = [&seen, adjust = rig.seams.adjust](RearConfig& config, RearCameraConfig& camera) {
      adjust(config, camera);
      seen = config.record;
    };
    rclcpp::NodeOptions options = rig.Options();
    if (numbers) {
      options.append_parameter_override("record_max_mb", 200.0);
      options.append_parameter_override("record_keep_min", 120);
      options.append_parameter_override("record_interval_s", 1);
    } else {
      options.append_parameter_override("record_dir", 7);
      options.append_parameter_override("record_max_mb", "200");
      options.append_parameter_override("record_keep_min", true);
      options.append_parameter_override("record_interval_s", "fast");
    }
    EXPECT_NO_THROW(RearNode node(options, rig.seams)) << numbers;
    EXPECT_EQ(seen.max_bytes, numbers ? 200UL << 20 : 0UL);
    if (numbers) {
      EXPECT_DOUBLE_EQ(seen.keep_s, 7200.0);
      EXPECT_DOUBLE_EQ(seen.interval_s, 1.0);
    }
  }
}

// Without calibration: OFF with NOT_SET_UP at least once a second, a request does not open the camera.
TEST_F(RearNodeTest, NotSetUpWithoutCalibration) {
  Rig rig;
  auto node = rig.Make(false);
  Spinner spinner;
  spinner.Add(node);
  spinner.Start();
  DockMarker m;
  ASSERT_TRUE(spinner.WaitFor([](const DockMarker&) { return true; }, 5.0, &m));
  EXPECT_EQ(m.state, DockMarker::STATE_OFF);
  EXPECT_FALSE(m.valid);
  EXPECT_EQ(m.reason, "NOT_SET_UP");
  EXPECT_EQ(m.detail, "no lens, no camera mount, no marker side, no marker pattern");
  EXPECT_EQ(m.header.frame_id, "base_link");
  spinner.RequestFor(1500ms);
  ASSERT_TRUE(spinner.WaitFor([](const DockMarker& d) { return d.state == DockMarker::STATE_FAULT; }, 1.0, &m));
  EXPECT_EQ(m.reason, "NOT_SET_UP");
  EXPECT_TRUE(rig.device.opened.empty());
  const auto got = spinner.Markers();
  for (size_t i = 1; i < got.size(); i++) EXPECT_LT((got[i].at - got[i - 1].at).seconds(), 1.0) << i;
}

// Over DDS: a request starts the camera, each image gives a measurement stamped with its capture time; three seconds
// after the last request the camera is off again.
TEST_F(RearNodeTest, MeasuresWhileAsked) {
  Rig rig;
  auto node = rig.Make();
  Spinner spinner;
  spinner.Add(node);
  spinner.Start();
  ASSERT_TRUE(spinner.WaitFor([](const DockMarker& m) { return m.state == DockMarker::STATE_OFF; }, 5.0));
  spinner.RequestFor(2000ms);
  const auto last_request = Clock::now() - 500ms;
  EXPECT_EQ(rig.device.ons, 1);
  ASSERT_TRUE(spinner.WaitFor([](const DockMarker& m) { return m.state == DockMarker::STATE_OFF; }, 4.0));
  const double off_after = std::chrono::duration<double>(Clock::now() - last_request).count();
  std::printf("last request to OFF: %.2f s\n", off_after);
  EXPECT_GT(off_after, 2.9);
  EXPECT_LT(off_after, 3.5);
  EXPECT_EQ(rig.device.offs, 1);
  EXPECT_FALSE(std::filesystem::exists(rig.dir.Path() + "/markers/stream-rear"));

  int valid = 0;
  int starting = 0;
  for (const auto& [m, at] : spinner.Markers()) {
    if (m.state == DockMarker::STATE_STARTING) starting++;
    if (m.state != DockMarker::STATE_MEASURING) continue;
    ASSERT_TRUE(m.valid) << m.reason << " " << m.detail;
    valid++;
    EXPECT_NEAR(m.x, -0.7, 0.003);
    EXPECT_NEAR(m.y, 0.04, 0.003);
    EXPECT_GT(m.covariance[0], 0.0);
    EXPECT_EQ(m.reason, "");
    EXPECT_EQ(m.header.frame_id, "base_link");
    // Taken one frame before it came, measured and sent within a few tens of milliseconds.
    const double age = (at - rclcpp::Time(m.header.stamp)).seconds();
    EXPECT_GT(age, 0.03);
    EXPECT_LT(age, 0.5);
  }
  EXPECT_GE(starting, 1);
  std::printf("valid measurements: %d\n", valid);
  EXPECT_GE(valid, 20);
  const auto pixels = spinner.Pixels();
  ASSERT_FALSE(pixels.empty());
  const auto& status = pixels.back().status.at(0);
  EXPECT_EQ(status.message, "found");
  ASSERT_GE(status.values.size(), 1U);
  EXPECT_EQ(status.values[0].key, "side_px");
  EXPECT_GT(std::atof(status.values[0].value.c_str()), 50.0);
  // The camera got its exposure back.
  EXPECT_EQ(rig.device.controls[V4L2_CID_EXPOSURE_AUTO].value, V4L2_EXPOSURE_APERTURE_PRIORITY);
  EXPECT_EQ(rig.device.controls[V4L2_CID_EXPOSURE_ABSOLUTE].value, 313);
}

// One entry on hw/cameras, latched; a subscriber of the pictures opens the camera with its own exposure and gets
// pictures of 720x405; two seconds after it left the camera closes.
TEST_F(RearNodeTest, ReportsTheCameraAndShowsItsPicture) {
  Rig rig;
  auto node = rig.Make(false);
  Spinner spinner;
  spinner.Add(node);
  auto listener = std::make_shared<rclcpp::Node>("viewer");
  std::mutex mutex;
  std::vector<Camera> cameras;
  std::vector<sensor_msgs::msg::CompressedImage> pictures;
  auto cameras_sub = listener->create_subscription<Camera>(
      "hw/cameras", rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local(), [&](const Camera& m) {
        std::lock_guard<std::mutex> lock(mutex);
        cameras.push_back(m);
      });
  spinner.Add(listener);
  spinner.Start();
  const auto wait_until = [&mutex](const std::function<bool()>& done, double seconds) {
    const auto end = Clock::now() + std::chrono::duration<double>(seconds);
    for (;;) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (done()) return true;
      }
      if (Clock::now() >= end) return false;
      std::this_thread::sleep_for(10ms);
    }
  };
  ASSERT_TRUE(wait_until([&] { return !cameras.empty(); }, 5.0));
  {
    std::lock_guard<std::mutex> lock(mutex);
    const Camera& c = cameras.back();
    EXPECT_EQ(c.id, "rear");
    EXPECT_EQ(c.name, "Rear camera");
    EXPECT_EQ(c.process, "rear");
    EXPECT_EQ(c.kind, Camera::KIND_REAR);
    EXPECT_EQ(c.position, Camera::POSITION_REAR);
    EXPECT_EQ(c.width, 720U);
    EXPECT_EQ(c.height, 405U);
    EXPECT_FALSE(c.detections);
    EXPECT_EQ(c.protocols, std::vector<std::string>{"mjpeg"});
    // Without calibration the marker is not set up; the camera still is.
    EXPECT_TRUE(c.available);
    EXPECT_EQ(c.reason, "");
  }
  EXPECT_EQ(rig.device.ons, 0);
  auto pictures_sub = listener->create_subscription<sensor_msgs::msg::CompressedImage>(
      "camera/rear/jpeg", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [&](const sensor_msgs::msg::CompressedImage& m) {
        std::lock_guard<std::mutex> lock(mutex);
        pictures.push_back(m);
      });
  ASSERT_TRUE(wait_until([&] { return pictures.size() >= 5; }, 10.0));
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto& p = pictures.back();
    EXPECT_EQ(p.format, "jpeg");
    EXPECT_EQ(p.header.frame_id, "rear");
    tjhandle h = tjInitDecompress();
    int w = 0;
    int ht = 0;
    int sub = 0;
    int space = 0;
    EXPECT_EQ(tjDecompressHeader3(h, p.data.data(), p.data.size(), &w, &ht, &sub, &space), 0);
    tjDestroy(h);
    EXPECT_EQ(w, 720);
    EXPECT_EQ(ht, 405);
  }
  EXPECT_EQ(rig.device.ons, 1);
  // The camera keeps its own exposure for a viewer.
  EXPECT_EQ(rig.device.controls[V4L2_CID_EXPOSURE_AUTO].value, V4L2_EXPOSURE_APERTURE_PRIORITY);
  pictures_sub.reset();
  const auto left = Clock::now();
  while (rig.device.offs == 0 && Clock::now() < left + 5s) std::this_thread::sleep_for(10ms);
  EXPECT_EQ(rig.device.offs, 1);
  const double closed_after = std::chrono::duration<double>(Clock::now() - left).count();
  EXPECT_GT(closed_after, 1.5);
  EXPECT_LT(closed_after, 3.0);
}

// With video the encoder is tried once at the start and the frames come from the heap; then WebRTC comes first at
// half the size of the camera, from the RTSP server of the process, and the camera stays closed without a client.
TEST_F(RearNodeTest, OffersVideoOnceTheEncoderWorks) {
  Rig rig;
  rig.video = true;
  std::mutex mutex;
  std::vector<Camera> cameras;
  auto node = rig.Make(false);
  auto listener = std::make_shared<rclcpp::Node>("video_viewer");
  auto cameras_sub = listener->create_subscription<Camera>(
      "hw/cameras", rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local(), [&](const Camera& m) {
        std::lock_guard<std::mutex> lock(mutex);
        cameras.push_back(m);
      });
  Spinner spinner;
  spinner.Add(node);
  spinner.Add(listener);
  spinner.Start();
  Camera c;
  const auto end = Clock::now() + 10s;
  for (bool found = false; !found;) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      found = !cameras.empty() && !cameras.back().source.empty();
      if (found) c = cameras.back();
    }
    ASSERT_LT(Clock::now(), end);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(c.protocols, (std::vector<std::string>{"webrtc", "mjpeg"}));
  EXPECT_EQ(c.source.rfind("rtsp://127.0.0.1:", 0), 0U) << c.source;
  EXPECT_EQ(c.source.substr(c.source.size() - 5), "/rear");
  EXPECT_EQ(c.width, 960U);
  EXPECT_EQ(c.height, 540U);
  EXPECT_TRUE(c.available);
  EXPECT_EQ(rig.heap.allocations, 3);
  EXPECT_EQ(rig.device.ons, 0);
}

// A loop thread that hangs in the camera ends the process with the code of the watch.
TEST_F(RearNodeTest, LoopThatHangsEndsTheProcess) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        Rig rig;
        rig.device.hang = true;
        auto node = rig.Make();
        Spinner spinner;
        spinner.Add(node);
        spinner.Start();
        spinner.RequestFor(10s);
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(mowbite_watch::kExitCode), "tron_rear_loop silent");
}

TEST(DockReasonText, NamesOfTheMessage) {
  const std::vector<std::pair<DockReason, std::string>> names = {
      {DockReason::kNone, ""},
      {DockReason::kNoMarker, "NO_MARKER"},
      {DockReason::kMarkerClipped, "MARKER_CLIPPED"},
      {DockReason::kMarkerAmbiguous, "MARKER_AMBIGUOUS"},
      {DockReason::kTooDark, "TOO_DARK"},
      {DockReason::kTooBright, "TOO_BRIGHT"},
      {DockReason::kBlurred, "BLURRED"},
      {DockReason::kFitRejected, "FIT_REJECTED"},
      {DockReason::kNoSignal, "NO_SIGNAL"},
      {DockReason::kNotSetUp, "NOT_SET_UP"},
      {DockReason::kLockedAfterCrash, "LOCKED_AFTER_CRASH"},
      {DockReason::kTooHot, "TOO_HOT"}};
  for (const auto& [reason, name] : names) EXPECT_EQ(DockReasonText(reason), name);
}

// The entry goes out the first time, on another availability or reason, and on another offer with the same
// availability, as when the encoder starts while the camera is available.
TEST(UpdateAvailability, OnlyOnAChange) {
  Camera entry;
  bool published = false;
  EXPECT_TRUE(UpdateAvailability(entry, published, {false, ""}, false));
  EXPECT_TRUE(published);
  EXPECT_FALSE(UpdateAvailability(entry, published, {false, ""}, false));
  EXPECT_TRUE(UpdateAvailability(entry, published, {true, ""}, false));
  EXPECT_TRUE(entry.available);
  EXPECT_FALSE(UpdateAvailability(entry, published, {true, ""}, false));
  EXPECT_TRUE(UpdateAvailability(entry, published, {true, ""}, true));
  EXPECT_TRUE(UpdateAvailability(entry, published, {false, "TOO_HOT"}, false));
  EXPECT_TRUE(UpdateAvailability(entry, published, {false, "NO_SIGNAL"}, false));
  EXPECT_EQ(entry.reason, "NO_SIGNAL");
  EXPECT_FALSE(UpdateAvailability(entry, published, {false, "NO_SIGNAL"}, false));
}

}  // namespace
}  // namespace tron_vision
