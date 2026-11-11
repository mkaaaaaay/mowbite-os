#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "dock_jpeg.h"
#include "test_util.h"
#include "tron_vision/rear_loop.h"

namespace tron_vision {
namespace {

using test::Behind;
using test::RearScene;
using test::Scene;

constexpr char kBoot[] = "6b0a57c2-0000-4000-8000-00000000b007";
constexpr double kFramePeriod = 1.0 / 30.0;

// The images of the tests, drawn once.
struct Images {
  Images() {
    Scene s = RearScene();
    s.markers = {Behind(s, 0.5, 0.03, 0.1)};
    marker = test::EncodeJpeg(test::Render(s));
    s.markers[0].lean = 0.35;
    s.markers[0].x = s.mount.x - 0.3;
    leaning = test::EncodeJpeg(test::Render(s));
    s.markers[0] = Behind(s, 0.5, 0.0, 0.0);
    s.exposure = 2.5;
    s.background = 4.0;
    glowing = test::EncodeJpeg(test::Render(s));
    empty = test::EncodeJpeg(test::Render(RearScene()));
    broken = std::vector<uint8_t>(marker.begin(), marker.begin() + static_cast<long>(marker.size() / 3));
  }
  std::vector<uint8_t> marker;
  std::vector<uint8_t> leaning;
  std::vector<uint8_t> glowing;
  std::vector<uint8_t> empty;
  std::vector<uint8_t> broken;
};

const Images& Drawn() {
  static const Images images;
  return images;
}

// The rear camera at 30 fps on the clock of the test; every call is noted.
class FakeDockCamera : public DockCamera {
 public:
  explicit FakeDockCamera(double& clock) : clock_(clock) {}

  bool Present(std::string& error) override {
    if (!present) error = "no /dev/mowbite-rear";
    return present;
  }
  bool Open(std::string& error) override {
    calls += "open ";
    if (fail_open) {
      error = "open /dev/mowbite-rear: Device or resource busy";
      return false;
    }
    open = true;
    return true;
  }
  bool Start(std::string& error) override {
    calls += "start ";
    if (fail_start) {
      error = "stream on: Input/output error";
      return false;
    }
    streaming = true;
    return true;
  }
  void Stop() override {
    if (streaming) calls += "stop ";
    streaming = false;
  }
  void Close() override {
    Stop();
    if (open) calls += "close ";
    open = false;
    manual = false;
  }
  int Next(int timeout_ms, CapturedFrame& frame, std::string& error) override {
    if (!streaming || fail_next) {
      error = "dequeue buffer: No such device";
      return -1;
    }
    if (jpeg == nullptr) {
      clock_ += timeout_ms / 1000.0;
      return 0;
    }
    clock_ += kFramePeriod;
    frame.data = jpeg->data();
    frame.bytes = jpeg->size();
    frame.sequence = ++sequence;
    frame.timestamp_us = std::llround((clock_ - lag) * 1e6);
    return 1;
  }
  bool ManualExposure(ExposureRange& range, ExposureSetting& now, std::string& error) override {
    if (!has_manual) {
      error = "control 10094850 disabled";
      return false;
    }
    manual = true;
    range = {1, 5000, 0, 63};
    now = {313, 10};
    return true;
  }
  bool SetExposure(const ExposureSetting& setting, std::string& error) override {
    settings.push_back(setting);
    if (!manual) error = "exposure not manual";
    return manual;
  }

  std::string calls;
  bool present = true;
  bool open = false;
  bool streaming = false;
  bool manual = false;
  bool has_manual = true;
  bool fail_open = false;
  bool fail_start = false;
  bool fail_next = false;
  // nullptr: no frames.
  const std::vector<uint8_t>* jpeg = nullptr;
  // Time from capture to hand-over.
  double lag = 0.02;
  uint32_t sequence = 0;
  std::vector<ExposureSetting> settings;

 private:
  double& clock_;
};

struct Rig {
  Rig() {
    std::filesystem::create_directories(dir.Path() + "/markers");
    dir.Write("boot_id", std::string(kBoot) + "\n");
    dir.Write("statm", "30000 5000 900 1 0 20000 0\n");
    const Scene s = RearScene();
    config.lens = s.lens;
    config.mount = s.mount;
    config.marker_side = s.side;
    config.marker.pattern = test::kDockPattern;
    config.request_hold_s = 3.0;
    config.report_period_s = 1.0;
    config.marker_dir = dir.Path() + "/markers";
    config.statm = dir.Path() + "/statm";
  }
  RearLoop& Make() {
    guard = std::make_unique<CrashGuard>(config.marker_dir, dir.Path() + "/crash", dir.Path() + "/boot_id");
    loop = std::make_unique<RearLoop>(
        config, camera, *guard, [this] { return clock; }, [this] { return request_at; },
        [this](double s) { clock += s; }, [this](const DockReport& r) { reports.push_back(r); });
    return *loop;
  }
  // Steps until the clock reaches `until`; while `asking`, a request every second as the logic sends them.
  bool RunUntil(double until) {
    while (clock < until) {
      if (asking && (request_at < 0.0 || clock - request_at >= 1.0)) request_at = clock;
      if (!loop->Step()) return false;
    }
    return true;
  }
  std::vector<DockReport> Of(DockState state) const {
    std::vector<DockReport> out;
    for (const DockReport& r : reports) {
      if (r.state == state) out.push_back(r);
    }
    return out;
  }
  bool Armed() const {
    return std::filesystem::exists(config.marker_dir + "/stream-rear");
  }

  test::TempDir dir;
  double clock = 100.0;
  double request_at = -1.0;
  bool asking = false;
  RearConfig config;
  FakeDockCamera camera{clock};
  std::unique_ptr<CrashGuard> guard;
  std::unique_ptr<RearLoop> loop;
  std::vector<DockReport> reports;
};

TEST(RearLoop, OffReportsOncePerPeriodAndOpensNothing) {
  Rig rig;
  rig.Make();
  ASSERT_TRUE(rig.RunUntil(104.0));
  ASSERT_GE(rig.reports.size(), 4U);
  EXPECT_LE(rig.reports.size(), 6U);
  for (size_t i = 0; i < rig.reports.size(); i++) {
    const DockReport& r = rig.reports[i];
    EXPECT_EQ(r.state, DockState::kOff);
    EXPECT_EQ(r.reason, DockReason::kNone);
    EXPECT_FALSE(r.image);
    EXPECT_FALSE(r.pose.valid);
    if (i > 0) {
      EXPECT_LT(r.stamp - rig.reports[i - 1].stamp, 1.0);
    }
  }
  EXPECT_EQ(rig.camera.calls, "");
}

// Asked for two seconds, a request each second: 15 images a second, each with its capture time, then off 3 s after
// the last request.
TEST(RearLoop, MeasuresWhileAskedAndStopsWhenTheRequestsRunOut) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.Make();
  rig.asking = true;
  bool armed_while_measuring = false;
  while (rig.clock < 102.0) {
    ASSERT_TRUE(rig.RunUntil(rig.clock + 0.01));
    armed_while_measuring = armed_while_measuring || rig.Armed();
  }
  rig.asking = false;
  const double last_request = rig.request_at;
  ASSERT_TRUE(rig.RunUntil(106.0));
  EXPECT_TRUE(armed_while_measuring);
  EXPECT_FALSE(rig.Armed());
  EXPECT_EQ(rig.camera.calls, "open start stop close ");

  ASSERT_GE(rig.reports.size(), 2U);
  EXPECT_EQ(rig.reports[0].state, DockState::kStarting);
  EXPECT_EQ(rig.reports[0].reason, DockReason::kNone);
  const std::vector<DockReport> measured = rig.Of(DockState::kMeasuring);
  int in_one_second = 0;
  for (const DockReport& r : measured) {
    ASSERT_TRUE(r.image);
    ASSERT_TRUE(r.pose.valid) << static_cast<int>(r.reason) << " " << r.detail;
    EXPECT_EQ(r.reason, DockReason::kNone);
    EXPECT_NEAR(r.pose.x, -0.7, 0.002);
    EXPECT_NEAR(r.pose.y, 0.04, 0.002);
    EXPECT_GT(r.pose.covariance[0], 0.0);
    EXPECT_TRUE(r.manual_exposure);
    if (r.stamp >= 101.0 && r.stamp < 102.0) in_one_second++;
  }
  EXPECT_GE(in_one_second, 14);
  EXPECT_LE(in_one_second, 16);
  // Each stamp is the capture time of its frame, which came 20 ms before the hand-over.
  const double first_frame = rig.reports[0].stamp + kFramePeriod - 0.02;
  EXPECT_NEAR(measured.front().stamp, first_frame, 1e-6);
  EXPECT_EQ(rig.reports.back().state, DockState::kOff);
  const std::vector<DockReport> off = rig.Of(DockState::kOff);
  ASSERT_FALSE(off.empty());
  EXPECT_NEAR(off.front().stamp, last_request + 3.0, 0.15);
  EXPECT_LE(measured.back().stamp, last_request + 3.0);
  EXPECT_GE(measured.back().stamp, last_request + 3.0 - 0.15);
}

TEST(RearLoop, ReasonOfEachImage) {
  EXPECT_EQ(ReasonOf(MarkerResult::kFound), DockReason::kNone);
  EXPECT_EQ(ReasonOf(MarkerResult::kNoMarker), DockReason::kNoMarker);
  EXPECT_EQ(ReasonOf(MarkerResult::kClipped), DockReason::kMarkerClipped);
  EXPECT_EQ(ReasonOf(MarkerResult::kAmbiguous), DockReason::kMarkerAmbiguous);
  EXPECT_EQ(ReasonOf(MarkerResult::kTooDark), DockReason::kTooDark);
  EXPECT_EQ(ReasonOf(MarkerResult::kTooBright), DockReason::kTooBright);
  EXPECT_EQ(ReasonOf(MarkerResult::kBlurred), DockReason::kBlurred);

  Rig rig;
  rig.Make();
  rig.asking = true;
  const auto last_after = [&rig](const std::vector<uint8_t>& image) {
    rig.camera.jpeg = &image;
    EXPECT_TRUE(rig.RunUntil(rig.clock + 0.2));
    return rig.reports.back();
  };
  DockReport r = last_after(Drawn().empty);
  EXPECT_EQ(r.state, DockState::kMeasuring);
  EXPECT_FALSE(r.pose.valid);
  EXPECT_EQ(r.reason, DockReason::kNoMarker);
  r = last_after(Drawn().leaning);
  EXPECT_FALSE(r.pose.valid);
  EXPECT_EQ(r.reason, DockReason::kFitRejected);
  EXPECT_EQ(r.detail.rfind("fit error ", 0), 0U) << r.detail;
  EXPECT_EQ(r.detection.result, MarkerResult::kFound);
  r = last_after(Drawn().glowing);
  EXPECT_FALSE(r.pose.valid);
  EXPECT_EQ(r.reason, DockReason::kTooBright);
  r = last_after(Drawn().marker);
  EXPECT_TRUE(r.pose.valid);
  EXPECT_EQ(r.reason, DockReason::kNone);
  EXPECT_EQ(r.detail, "");
}

// Without calibration nothing opens: OFF says so, a request ends in FAULT until the requests run out.
TEST(RearLoop, NotSetUp) {
  Rig rig;
  rig.config.not_set_up = "no lens";
  rig.Make();
  ASSERT_TRUE(rig.RunUntil(100.5));
  ASSERT_EQ(rig.reports.size(), 1U);
  EXPECT_EQ(rig.reports[0].state, DockState::kOff);
  EXPECT_EQ(rig.reports[0].reason, DockReason::kNotSetUp);
  EXPECT_EQ(rig.reports[0].detail, "no lens");
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(103.05));
  const std::vector<DockReport> fault = rig.Of(DockState::kFault);
  ASSERT_GE(fault.size(), 3U);
  for (const DockReport& r : fault) EXPECT_EQ(r.reason, DockReason::kNotSetUp);
  rig.asking = false;
  ASSERT_TRUE(rig.RunUntil(106.5));
  EXPECT_EQ(rig.reports.back().state, DockState::kOff);
  EXPECT_EQ(rig.reports.back().reason, DockReason::kNotSetUp);
  EXPECT_EQ(rig.camera.calls, "");
}

// The marker of a stream in an earlier boot together with the crash flag locks the camera.
TEST(RearLoop, LockedAfterAKernelCrash) {
  Rig rig;
  rig.dir.Write("markers/stream-rear", "an-earlier-boot\n");
  rig.dir.Write("crash", "panic in uvcvideo\n");
  rig.Make();
  ASSERT_TRUE(rig.RunUntil(100.5));
  EXPECT_EQ(rig.reports.back().state, DockState::kOff);
  EXPECT_EQ(rig.reports.back().reason, DockReason::kLockedAfterCrash);
  EXPECT_NE(rig.reports.back().detail.find("panic in uvcvideo"), std::string::npos) << rig.reports.back().detail;
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(101.5));
  EXPECT_EQ(rig.reports.back().state, DockState::kFault);
  EXPECT_EQ(rig.reports.back().reason, DockReason::kLockedAfterCrash);
  EXPECT_EQ(rig.camera.calls, "");
  EXPECT_TRUE(std::filesystem::exists(rig.config.marker_dir + "/locked-rear"));
}

TEST(RearLoop, MissingCameraIsNoSignal) {
  Rig rig;
  rig.camera.present = false;
  rig.Make();
  ASSERT_TRUE(rig.RunUntil(100.5));
  EXPECT_EQ(rig.reports.back().reason, DockReason::kNoSignal);
  EXPECT_EQ(rig.reports.back().detail, "no /dev/mowbite-rear");
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(101.5));
  EXPECT_EQ(rig.reports.back().state, DockState::kFault);
  EXPECT_EQ(rig.camera.calls, "");
}

// No frame: STARTING within each second, FAULT after first_frame_s; no new try within the requests, the next one starts
// anew.
TEST(RearLoop, NoFramesIsAFaultUntilTheRequestsRunOut) {
  Rig rig;
  rig.Make();
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(106.0));
  const std::vector<DockReport> starting = rig.Of(DockState::kStarting);
  ASSERT_EQ(starting.size(), 4U);
  const std::vector<DockReport> fault = rig.Of(DockState::kFault);
  ASSERT_GE(fault.size(), 3U);
  EXPECT_NEAR(fault.front().stamp, 103.0, 0.11);
  EXPECT_EQ(fault.front().reason, DockReason::kNoSignal);
  EXPECT_EQ(fault.front().detail, "no usable image within 3.0 s");
  EXPECT_EQ(rig.camera.calls, "open start stop close ");
  EXPECT_FALSE(rig.Armed());
  rig.asking = false;
  ASSERT_TRUE(rig.RunUntil(109.5));
  EXPECT_EQ(rig.reports.back().state, DockState::kOff);
  EXPECT_EQ(rig.reports.back().reason, DockReason::kNone);
  rig.camera.jpeg = &Drawn().marker;
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(110.0));
  EXPECT_EQ(rig.reports.back().state, DockState::kMeasuring);
  EXPECT_EQ(rig.camera.calls, "open start stop close open start ");
}

// Broken images are reported each as NO_SIGNAL; without a usable one for frame_gap_s it is a fault.
TEST(RearLoop, BrokenImagesThenAGap) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.Make();
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(100.5));
  rig.camera.jpeg = &Drawn().broken;
  const size_t before = rig.reports.size();
  ASSERT_TRUE(rig.RunUntil(101.5));
  ASSERT_GT(rig.reports.size(), before + 5);
  const DockReport& broken = rig.reports[before + 1];
  EXPECT_EQ(broken.state, DockState::kMeasuring);
  EXPECT_TRUE(broken.image);
  EXPECT_EQ(broken.reason, DockReason::kNoSignal);
  EXPECT_EQ(broken.detail.rfind("broken image: ", 0), 0U) << broken.detail;
  const std::vector<DockReport> fault = rig.Of(DockState::kFault);
  ASSERT_EQ(fault.size(), 1U);
  EXPECT_EQ(fault[0].detail, "no usable image within 0.5 s");
  EXPECT_NEAR(fault[0].stamp, 101.0, 0.1);
}

TEST(RearLoop, AnImageWithABrokenTimeIsNotMeasured) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.camera.lag = 2.0;
  rig.Make();
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(100.3));
  const std::vector<DockReport> measured = rig.Of(DockState::kMeasuring);
  ASSERT_FALSE(measured.empty());
  EXPECT_FALSE(measured[0].pose.valid);
  EXPECT_EQ(measured[0].reason, DockReason::kNoSignal);
  EXPECT_EQ(measured[0].detail, "image time off by 2.000 s");
  ASSERT_TRUE(rig.RunUntil(103.5));
  EXPECT_EQ(rig.reports.back().state, DockState::kFault);
  EXPECT_EQ(rig.reports.back().detail, "no usable image within 3.0 s");
}

TEST(RearLoop, CameraFailuresAreFaults) {
  for (int which = 0; which < 3; which++) {
    Rig rig;
    rig.camera.jpeg = &Drawn().marker;
    rig.camera.fail_open = which == 0;
    rig.camera.fail_start = which == 1;
    rig.camera.fail_next = which == 2;
    rig.Make();
    rig.asking = true;
    ASSERT_TRUE(rig.RunUntil(101.5));
    const std::vector<DockReport> fault = rig.Of(DockState::kFault);
    ASSERT_FALSE(fault.empty()) << which;
    EXPECT_EQ(fault[0].reason, DockReason::kNoSignal);
    const std::vector<std::string> details = {"open /dev/mowbite-rear: Device or resource busy",
                                              "stream on: Input/output error", "dequeue buffer: No such device"};
    EXPECT_EQ(fault[0].detail, details[which]);
    EXPECT_FALSE(rig.camera.open);
    EXPECT_FALSE(rig.Armed());
    EXPECT_EQ(rig.camera.calls.find("open", 1), std::string::npos) << rig.camera.calls;
  }
}

// The exposure time starts capped against motion blur; on a glowing marker the light goes down, the gain first.
TEST(RearLoop, ExposureFollowsTheMarker) {
  Rig rig;
  rig.camera.jpeg = &Drawn().glowing;
  rig.Make();
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(101.5));
  const std::vector<ExposureSetting>& set = rig.camera.settings;
  ASSERT_GE(set.size(), 5U);
  // 31.3 ms of the camera capped at 5 ms; the gain in the units of its control, which start at 0.
  EXPECT_EQ(set[0].exposure, 50);
  EXPECT_EQ(set[0].gain, 10);
  EXPECT_EQ(set[1].exposure, 50);
  EXPECT_EQ(set[1].gain, 5);
  for (size_t i = 1; i < set.size(); i++) {
    EXPECT_LT(set[i].exposure * (set[i].gain + 1), set[i - 1].exposure * (set[i - 1].gain + 1)) << i;
  }
  EXPECT_LT(set.back().exposure, 50);
  EXPECT_EQ(set.back().gain, 0);
  // Each report names the setting its image was taken with.
  const std::vector<DockReport> measured = rig.Of(DockState::kMeasuring);
  ASSERT_FALSE(measured.empty());
  EXPECT_EQ(measured[0].exposure.exposure, 50);
  EXPECT_EQ(measured[0].exposure.gain, 10);
  EXPECT_EQ(measured[0].reason, DockReason::kTooBright);
}

TEST(RearLoop, CameraWithoutManualExposureKeepsItsOwn) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.camera.has_manual = false;
  rig.Make();
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(100.5));
  EXPECT_EQ(rig.reports[0].state, DockState::kStarting);
  EXPECT_EQ(rig.reports[0].detail, "automatic exposure: control 10094850 disabled");
  EXPECT_TRUE(rig.camera.settings.empty());
  const std::vector<DockReport> measured = rig.Of(DockState::kMeasuring);
  ASSERT_FALSE(measured.empty());
  EXPECT_TRUE(measured.back().pose.valid);
  EXPECT_FALSE(measured.back().manual_exposure);
}

TEST(RearLoop, TooHotStopsTheCamera) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.config.temp_file = rig.dir.Write("temp", "60000\n");
  rig.config.check_s = 0.1;
  rig.Make();
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(100.5));
  EXPECT_EQ(rig.reports.back().state, DockState::kMeasuring);
  rig.dir.Write("temp", "81000\n");
  ASSERT_TRUE(rig.RunUntil(100.8));
  EXPECT_EQ(rig.reports.back().state, DockState::kFault);
  EXPECT_EQ(rig.reports.back().reason, DockReason::kTooHot);
  EXPECT_FALSE(rig.camera.open);
  rig.asking = false;
  ASSERT_TRUE(rig.RunUntil(104.0));
  EXPECT_EQ(rig.reports.back().state, DockState::kOff);
  EXPECT_EQ(rig.reports.back().reason, DockReason::kTooHot);
}

TEST(RearLoop, MemoryOverTheLimitEndsTheProcess) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.Make();
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(100.5));
  rig.dir.Write("statm", "300000 50000 900 1 0 20000 0\n");
  EXPECT_FALSE(rig.RunUntil(110.0));
  EXPECT_EQ(rig.loop->ExitCode(), kExitMemory);
  EXPECT_FALSE(rig.loop->Step());
  EXPECT_EQ(rig.reports.back().state, DockState::kFault);
  EXPECT_EQ(rig.reports.back().reason, DockReason::kNoSignal);
  EXPECT_EQ(rig.reports.back().detail.rfind("resident memory ", 0), 0U) << rig.reports.back().detail;
  EXPECT_FALSE(rig.camera.open);
  EXPECT_FALSE(rig.Armed());
}

}  // namespace
}  // namespace tron_vision
