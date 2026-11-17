#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "dock_jpeg.h"
#include "fake_dma_heap.h"
#include "fake_encoder.h"
#include "rtsp_test_client.h"
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
    clock_ += open_s;
    if (fail_open) {
      error = "open /dev/mowbite-rear: Device or resource busy";
      return false;
    }
    open = true;
    return true;
  }
  bool Start(std::string& error) override {
    calls += "start ";
    clock_ += start_s;
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
    if (open) {
      calls += "close ";
      clock_ += close_s;
    }
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
    if (fail_set_from >= 0 && settings.size() >= static_cast<size_t>(fail_set_from)) {
      error = "set control 9963795: Broken pipe";
      return false;
    }
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
  // Not negative: SetExposure fails once this many settings went through.
  int fail_set_from = -1;
  // What the calls cost on the clock of the test.
  double open_s = 0.0;
  double start_s = 0.0;
  double close_s = 0.0;
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
        [this](double s) { clock += s; },
        [this](const DockReport& r) {
          reports.push_back(r);
          sent_at.push_back(clock);
          pictures_at_report.push_back(loop->Picture().Count());
        },
        stream.get(), frames.get());
    return *loop;
  }
  // H.264 for an RTSP client on any free port, from an encoder of the test, in frames of a heap of the test.
  void WithVideo() {
    RtspConfig rtsp;
    rtsp.port = 0;
    rtsp.paths = {"rear"};
    server = std::make_unique<RtspServer>(rtsp, [this] { return clock; });
    std::string error;
    ASSERT_TRUE(server->Listen(error)) << error;
    stream = std::make_unique<H264Stream>(encoder, *server, 0.5, [this] { return clock; });
    EncoderSettings& e = config.video.encoder;
    e.format = PixelFormat::kNv12;
    e.width = e.line_bytes = 960;
    e.height = 540;
    e.lines = 544;
    frames = std::make_unique<DmaFrames>(kSystemHeap, &heap);
    ASSERT_TRUE(frames->Open(e, 3, error)) << error;
  }
  std::string Ask(const test::Client& client, const std::string& request) {
    client.Send(request);
    RunUntil(clock + 0.3);
    return client.Receive();
  }
  std::string Url(const std::string& rest = "") const {
    return "rtsp://127.0.0.1:" + std::to_string(server->Port()) + "/rear" + rest;
  }
  // A client that has asked for the stream and sent PLAY.
  std::unique_ptr<test::Client> Play() {
    auto client = std::make_unique<test::Client>(server->Port());
    Ask(*client, "DESCRIBE " + Url() + " RTSP/1.0\r\nCSeq: 1\r\n\r\n");
    const std::string setup =
        Ask(*client, "SETUP " + Url("/trackID=0") +
                         " RTSP/1.0\r\nCSeq: 2\r\nTransport: RTP/AVP/TCP;interleaved=0-1\r\n\r\n");
    client->Send("PLAY " + Url("/") + " RTSP/1.0\r\nCSeq: 3\r\nSession: " + test::SessionOf(setup) + "\r\n\r\n");
    return client;
  }
  // One step; while `asking`, a request every second as the logic sends them, while `viewing` a viewer.
  bool Step() {
    if (asking && (request_at < 0.0 || clock - request_at >= 1.0)) request_at = clock;
    loop->SetViewWanted(viewing);
    return loop->Step();
  }
  // Steps until the clock reaches `until`.
  bool RunUntil(double until) {
    while (clock < until) {
      if (!Step()) return false;
    }
    return true;
  }
  uint64_t Pictures() const {
    return loop->Picture().Count();
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
  bool viewing = false;
  RearConfig config;
  FakeDockCamera camera{clock};
  std::unique_ptr<CrashGuard> guard;
  test::FakeEncoder encoder{clock};
  std::unique_ptr<RtspServer> server;
  std::unique_ptr<H264Stream> stream;
  test::FakeDmaHeap heap;
  std::unique_ptr<DmaFrames> frames;
  std::unique_ptr<RearLoop> loop;
  std::vector<DockReport> reports;
  std::vector<double> sent_at;
  // The count of pictures when each report went out.
  std::vector<uint64_t> pictures_at_report;
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

  ASSERT_GE(rig.reports.size(), 3U);
  // The state before the camera opens, then the start.
  EXPECT_EQ(rig.reports[0].state, DockState::kOff);
  EXPECT_EQ(rig.reports[1].state, DockState::kStarting);
  EXPECT_EQ(rig.reports[1].reason, DockReason::kNone);
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
  const double first_frame = rig.reports[1].stamp + kFramePeriod - 0.02;
  EXPECT_NEAR(measured.front().stamp, first_frame, 1e-6);
  EXPECT_EQ(rig.reports.back().state, DockState::kOff);
  const std::vector<DockReport> off = rig.Of(DockState::kOff);
  ASSERT_GE(off.size(), 2U);
  EXPECT_NEAR(off[1].stamp, last_request + 3.0, 0.15);
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

// With a recording each evaluated image goes to it as it came, at most one per interval, with its reason and the
// setting it was taken with; a recording that fails leaves the measurement alone.
TEST(RearLoop, RecordsTheEvaluatedImagesWithTheirSetting) {
  Rig rig;
  rig.camera.jpeg = &Drawn().glowing;
  rig.config.record.dir = rig.dir.Path() + "/rec";
  rig.config.record.max_bytes = 50UL << 20;
  rig.config.record.interval_s = 0.5;
  rig.Make();
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(101.6));
  std::vector<std::string> texts;
  for (const auto& e : std::filesystem::directory_iterator(rig.config.record.dir)) {
    if (e.path().extension() != ".txt") continue;
    std::ifstream in(e.path());
    texts.emplace_back(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    std::ifstream jpg(e.path().parent_path() / (e.path().stem().string() + ".jpg"), std::ios::binary);
    const std::vector<uint8_t> image{std::istreambuf_iterator<char>(jpg), std::istreambuf_iterator<char>()};
    EXPECT_EQ(image, Drawn().glowing);
  }
  ASSERT_GE(texts.size(), 2U);
  EXPECT_LE(texts.size(), 4U);
  std::sort(texts.begin(), texts.end());
  EXPECT_NE(texts[0].find("\nreason " + std::to_string(static_cast<int>(DockReason::kTooBright)) + "\n"),
            std::string::npos)
      << texts[0];
  EXPECT_NE(texts[0].find("\nexposure 50\ngain 10\n"), std::string::npos) << texts[0];
  EXPECT_NE(texts[0].find("\nmanual 1\n"), std::string::npos) << texts[0];
  EXPECT_TRUE(rig.loop->RecordError().empty());

  std::filesystem::remove_all(rig.config.record.dir);
  rig.dir.Write("rec", "a file where the directory was");
  const size_t before = rig.Of(DockState::kMeasuring).size();
  ASSERT_TRUE(rig.RunUntil(102.5));
  EXPECT_NE(rig.loop->RecordError().find("recording ended"), std::string::npos);
  EXPECT_GE(rig.Of(DockState::kMeasuring).size(), before + 10);
}

// A setting the camera refuses for good ends the request, at the start as well as on a step of the rule.
TEST(RearLoop, ARefusedExposureIsAFault) {
  for (int from = 0; from < 2; from++) {
    Rig rig;
    rig.camera.jpeg = &Drawn().glowing;
    rig.camera.fail_set_from = from;
    rig.Make();
    rig.asking = true;
    ASSERT_TRUE(rig.RunUntil(101.5));
    const std::vector<DockReport> fault = rig.Of(DockState::kFault);
    ASSERT_FALSE(fault.empty()) << from;
    EXPECT_EQ(fault[0].reason, DockReason::kNoSignal);
    EXPECT_EQ(fault[0].detail, "exposure: set control 9963795: Broken pipe");
    EXPECT_EQ(rig.Of(DockState::kMeasuring).empty(), from == 0) << from;
    EXPECT_EQ(rig.camera.settings.size(), static_cast<size_t>(from));
    EXPECT_FALSE(rig.camera.open);
    EXPECT_FALSE(rig.Armed());
  }
}

TEST(RearLoop, CameraWithoutManualExposureKeepsItsOwn) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.camera.has_manual = false;
  rig.Make();
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(100.5));
  ASSERT_GE(rig.reports.size(), 2U);
  EXPECT_EQ(rig.reports[1].state, DockState::kStarting);
  EXPECT_EQ(rig.reports[1].detail, "automatic exposure: control 10094850 disabled");
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

// A viewer alone: the camera opens with its own exposure, pictures of 720x405 come at most 5 a second, the reports
// stay those of OFF, and two seconds after the last sign of the viewer the camera closes.
TEST(RearLoop, AViewerAloneGetsPicturesWithTheExposureOfTheCamera) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.Make();
  rig.viewing = true;
  ASSERT_TRUE(rig.RunUntil(102.0));
  EXPECT_EQ(rig.camera.calls, "open start ");
  EXPECT_FALSE(rig.camera.manual);
  EXPECT_TRUE(rig.camera.settings.empty());
  EXPECT_TRUE(rig.Armed());
  EXPECT_GE(rig.Pictures(), 9U);
  EXPECT_LE(rig.Pictures(), 11U);
  EXPECT_EQ(rig.loop->Picture().Width(), 720U);
  EXPECT_EQ(rig.loop->Picture().Height(), 405U);
  EXPECT_LE(rig.loop->Picture().Jpeg().size(), 100U * 1024);
  EXPECT_TRUE(rig.loop->Viewable().available);
  ASSERT_GE(rig.reports.size(), 2U);
  for (const DockReport& r : rig.reports) {
    EXPECT_EQ(r.state, DockState::kOff);
    EXPECT_EQ(r.reason, DockReason::kNone);
  }
  const double last = rig.clock;
  ASSERT_TRUE(rig.Step());
  rig.viewing = false;
  double closed = 0.0;
  while (rig.camera.open && rig.clock < last + 5.0) {
    closed = rig.clock;
    ASSERT_TRUE(rig.Step());
  }
  EXPECT_NEAR(closed, last + 2.0, kFramePeriod + 1e-6);
  EXPECT_EQ(rig.camera.calls, "open start stop close ");
  EXPECT_FALSE(rig.Armed());
}

// The live picture needs no calibration; the reports still say NOT_SET_UP.
TEST(RearLoop, AViewerWithoutCalibration) {
  Rig rig;
  rig.config.not_set_up = "no lens";
  rig.camera.jpeg = &Drawn().marker;
  rig.Make();
  rig.viewing = true;
  ASSERT_TRUE(rig.RunUntil(101.0));
  EXPECT_EQ(rig.camera.calls, "open start ");
  EXPECT_GE(rig.Pictures(), 4U);
  EXPECT_TRUE(rig.loop->Viewable().available);
  ASSERT_FALSE(rig.reports.empty());
  EXPECT_EQ(rig.reports.back().state, DockState::kOff);
  EXPECT_EQ(rig.reports.back().reason, DockReason::kNotSetUp);
}

// A request takes the camera from the viewer: it opens anew with the manual exposure of the marker and measures; when
// the requests run out, OFF, then the viewer gets the camera back with its own exposure.
TEST(RearLoop, ARequestGoesFirstAndTheViewerComesBack) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.Make();
  rig.viewing = true;
  ASSERT_TRUE(rig.RunUntil(101.0));
  const uint64_t before = rig.Pictures();
  rig.asking = true;
  ASSERT_TRUE(rig.RunUntil(103.0));
  EXPECT_EQ(rig.camera.calls, "open start stop close open start ");
  EXPECT_TRUE(rig.camera.manual);
  EXPECT_FALSE(rig.camera.settings.empty());
  EXPECT_GT(rig.Pictures(), before + 5);
  const std::vector<DockReport> measured = rig.Of(DockState::kMeasuring);
  ASSERT_GE(measured.size(), 20U);
  for (const DockReport& r : measured) {
    EXPECT_TRUE(r.pose.valid);
    EXPECT_TRUE(r.manual_exposure);
  }
  rig.asking = false;
  ASSERT_TRUE(rig.RunUntil(106.5));
  const std::vector<DockReport> off = rig.Of(DockState::kOff);
  ASSERT_FALSE(off.empty());
  EXPECT_GT(off.back().stamp, measured.back().stamp);
  EXPECT_EQ(rig.reports.back().state, DockState::kOff);
  EXPECT_EQ(rig.camera.calls, "open start stop close open start stop close open start ");
  EXPECT_FALSE(rig.camera.manual);
  const uint64_t after_off = rig.Pictures();
  ASSERT_TRUE(rig.RunUntil(107.5));
  EXPECT_GE(rig.Pictures(), after_off + 4);
}

// For the same frames and clock the reports with a viewer are those without one, up to OFF after the requests.
TEST(RearLoop, AViewerChangesNoReport) {
  std::vector<std::vector<DockReport>> runs;
  for (bool viewing : {false, true}) {
    Rig rig;
    rig.camera.jpeg = &Drawn().marker;
    rig.Make();
    rig.viewing = viewing;
    rig.asking = true;
    while (rig.clock < 101.0) {
      ASSERT_TRUE(rig.Step());
      // Glowing for a while, so that the exposure moves too.
      rig.camera.jpeg = rig.clock < 100.5 ? &Drawn().marker : &Drawn().glowing;
    }
    rig.camera.jpeg = &Drawn().marker;
    ASSERT_TRUE(rig.RunUntil(102.0));
    rig.asking = false;
    ASSERT_TRUE(rig.RunUntil(106.0));
    EXPECT_EQ(rig.Pictures() > 0, viewing);
    std::vector<DockReport> until_off;
    for (const DockReport& r : rig.reports) {
      until_off.push_back(r);
      if (r.state == DockState::kOff) break;
    }
    ASSERT_EQ(until_off.back().state, DockState::kOff);
    runs.push_back(until_off);
  }
  ASSERT_EQ(runs[0].size(), runs[1].size());
  for (size_t i = 0; i < runs[0].size(); i++) {
    const DockReport& a = runs[0][i];
    const DockReport& b = runs[1][i];
    EXPECT_EQ(a.state, b.state) << i;
    EXPECT_EQ(a.stamp, b.stamp) << i;
    EXPECT_EQ(a.reason, b.reason) << i;
    EXPECT_EQ(a.detail, b.detail) << i;
    EXPECT_EQ(a.pose.valid, b.pose.valid) << i;
    EXPECT_EQ(a.exposure.exposure, b.exposure.exposure) << i;
    EXPECT_EQ(a.exposure.gain, b.exposure.gain) << i;
  }
}

// While measuring, the picture of a frame comes after its report, also from frames the evaluation drops; a broken
// image gives none.
TEST(RearLoop, PicturesWhileMeasuringComeAfterTheReport) {
  Rig rig;
  rig.config.eval_rate = 2.0;
  rig.camera.jpeg = &Drawn().marker;
  rig.Make();
  rig.viewing = true;
  rig.asking = true;
  int with_report = 0;
  int without_report = 0;
  while (rig.clock < 102.0) {
    const uint64_t pictures = rig.Pictures();
    const size_t reports = rig.reports.size();
    ASSERT_TRUE(rig.Step());
    if (rig.Pictures() == pictures) continue;
    if (rig.reports.size() == reports) {
      without_report++;
      continue;
    }
    with_report++;
    EXPECT_EQ(rig.reports.back().state, DockState::kMeasuring);
    EXPECT_EQ(rig.pictures_at_report.back(), pictures);
  }
  EXPECT_GE(with_report, 1);
  EXPECT_GE(without_report, 5);
  rig.camera.jpeg = &Drawn().broken;
  const uint64_t pictures = rig.Pictures();
  ASSERT_TRUE(rig.RunUntil(102.4));
  EXPECT_EQ(rig.Pictures(), pictures);
}

// Failures while the viewer alone has the camera: NO_SIGNAL for the viewer, three attempts five seconds apart, never
// a FAULT; a new viewing starts with three attempts again.
TEST(RearLoop, FailuresOfTheViewerAreNoFault) {
  for (int which = 0; which < 4; which++) {
    Rig rig;
    rig.camera.jpeg = which == 3 ? &Drawn().broken : which == 2 ? nullptr : &Drawn().marker;
    rig.camera.fail_open = which == 0;
    rig.camera.fail_start = which == 1;
    rig.Make();
    rig.viewing = true;
    ASSERT_TRUE(rig.RunUntil(100.5 + (which == 2 ? 3.0 : 0.0)));
    const ViewAvailability view = rig.loop->Viewable();
    EXPECT_FALSE(view.available) << which;
    EXPECT_EQ(view.reason, DockReason::kNoSignal) << which;
    const std::vector<std::string> details = {"open /dev/mowbite-rear: Device or resource busy",
                                              "stream on: Input/output error", "no image within 3.0 s",
                                              "broken images: "};
    EXPECT_EQ(view.detail.rfind(details[which], 0), 0U) << view.detail;
    EXPECT_FALSE(rig.camera.open);
    EXPECT_FALSE(rig.Armed());
    ASSERT_TRUE(rig.RunUntil(125.0));
    const auto opens = [&rig] {
      size_t n = 0;
      for (size_t at = rig.camera.calls.find("open"); at != std::string::npos;
           at = rig.camera.calls.find("open", at + 1)) {
        n++;
      }
      return n;
    };
    EXPECT_EQ(opens(), 3U) << which << ": " << rig.camera.calls;
    EXPECT_EQ(rig.Pictures(), 0U);
    for (const DockReport& r : rig.reports) {
      EXPECT_EQ(r.state, DockState::kOff);
      EXPECT_EQ(r.reason, DockReason::kNone);
    }
    rig.viewing = false;
    ASSERT_TRUE(rig.RunUntil(128.0));
    EXPECT_TRUE(rig.loop->Viewable().available);
    rig.viewing = true;
    ASSERT_TRUE(rig.RunUntil(128.5));
    EXPECT_EQ(opens(), 4U) << which << ": " << rig.camera.calls;
  }
}

TEST(RearLoop, TooHotStopsTheViewer) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.config.temp_file = rig.dir.Write("temp", "60000\n");
  rig.config.check_s = 0.1;
  rig.Make();
  rig.viewing = true;
  ASSERT_TRUE(rig.RunUntil(100.5));
  EXPECT_TRUE(rig.camera.open);
  rig.dir.Write("temp", "81000\n");
  ASSERT_TRUE(rig.RunUntil(100.8));
  EXPECT_FALSE(rig.camera.open);
  EXPECT_EQ(rig.loop->Viewable().reason, DockReason::kTooHot);
  rig.dir.Write("temp", "70000\n");
  ASSERT_TRUE(rig.RunUntil(106.0));
  EXPECT_TRUE(rig.camera.open);
  EXPECT_TRUE(rig.loop->Viewable().available);
  EXPECT_EQ(rig.camera.calls, "open start stop close open start ");
  EXPECT_TRUE(rig.Of(DockState::kFault).empty());
}

TEST(RearLoop, LockedOrMissingCameraHasNoViewer) {
  Rig locked;
  locked.dir.Write("markers/stream-rear", "an-earlier-boot\n");
  locked.dir.Write("crash", "panic in uvcvideo\n");
  locked.Make();
  ASSERT_TRUE(locked.Step());
  ViewAvailability view = locked.loop->Viewable();
  EXPECT_FALSE(view.available);
  EXPECT_EQ(view.reason, DockReason::kLockedAfterCrash);
  EXPECT_NE(view.detail.find("panic in uvcvideo"), std::string::npos) << view.detail;
  locked.viewing = true;
  ASSERT_TRUE(locked.RunUntil(110.0));
  EXPECT_EQ(locked.camera.calls, "");

  Rig missing;
  missing.camera.present = false;
  missing.Make();
  missing.viewing = true;
  ASSERT_TRUE(missing.RunUntil(110.0));
  view = missing.loop->Viewable();
  EXPECT_EQ(view.reason, DockReason::kNoSignal);
  EXPECT_EQ(view.detail, "no /dev/mowbite-rear");
  EXPECT_EQ(missing.camera.calls, "");
}

// Opening, starting and closing the camera take time. The state goes out before each such step, so hw/dock_marker is
// never silent for its period while a viewer comes, a request takes the camera over, and both leave again.
TEST(RearLoop, SlowCameraStepsLeaveNoGapInTheReports) {
  for (const double shift : {0.0, 0.2, 0.4, 0.6}) {
    Rig rig;
    rig.camera.jpeg = &Drawn().marker;
    rig.camera.open_s = 0.25;
    rig.camera.start_s = 0.25;
    rig.camera.close_s = 0.15;
    rig.Make();
    ASSERT_TRUE(rig.RunUntil(101.0 + shift));
    rig.viewing = true;
    ASSERT_TRUE(rig.RunUntil(104.0 + 2.0 * shift));
    rig.asking = true;
    ASSERT_TRUE(rig.RunUntil(105.5 + 2.0 * shift));
    rig.asking = false;
    ASSERT_TRUE(rig.RunUntil(111.0 + 3.0 * shift));
    rig.viewing = false;
    ASSERT_TRUE(rig.RunUntil(115.0));
    EXPECT_FALSE(rig.Of(DockState::kMeasuring).empty());
    EXPECT_TRUE(rig.Of(DockState::kFault).empty());
    EXPECT_GT(rig.Pictures(), 0U);
    for (size_t i = 1; i < rig.sent_at.size(); i++) {
      EXPECT_LT(rig.sent_at[i] - rig.sent_at[i - 1], rig.config.report_period_s)
          << "shift " << shift << ", report " << i << " at " << rig.sent_at[i];
    }
  }
}

// The RTSP client is a viewer: the camera opens for it with its own exposure and every image goes out as H.264 at
// half size; without a client the encoder only gave its parameter sets, and when the client leaves it closes.
TEST(RearLoop, H264ForTheRtspClient) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.WithVideo();
  rig.Make();
  ASSERT_TRUE(rig.RunUntil(103.0));
  EXPECT_TRUE(rig.loop->VideoActive());
  EXPECT_EQ(rig.encoder.opens, 1);
  EXPECT_FALSE(rig.encoder.is_open);
  EXPECT_EQ(rig.camera.calls, "");
  EXPECT_EQ(rig.stream->Frames(), 0U);
  {
    const std::unique_ptr<test::Client> client = rig.Play();
    ASSERT_TRUE(rig.RunUntil(rig.clock + 2.0));
    EXPECT_EQ(rig.camera.calls, "open start ");
    EXPECT_FALSE(rig.camera.manual);
    EXPECT_EQ(rig.encoder.opens, 2);
    EXPECT_TRUE(rig.encoder.is_open);
    EXPECT_EQ(rig.encoder.settings.width, 960U);
    EXPECT_EQ(rig.encoder.settings.height, 540U);
    EXPECT_EQ(rig.encoder.settings.lines, 544U);
    EXPECT_GT(rig.stream->Frames(), 45U);
    // No JPEG without a subscriber of the pictures.
    EXPECT_EQ(rig.Pictures(), 0U);
    EXPECT_TRUE(rig.loop->Viewable().available);
    EXPECT_TRUE(rig.loop->VideoError().empty()) << rig.loop->VideoError();
    EXPECT_NE(client->Receive().find('$'), std::string::npos);
    // The drawn image is gray; the rows below the picture repeat its last one.
    EXPECT_EQ(rig.heap.allocations, 3);
    const std::vector<uint8_t>& frame = rig.heap.memory[test::FakeDmaHeap::kFirstBufferFd];
    const uint8_t* uv = frame.data() + 960 * 544;
    EXPECT_NEAR(uv[135 * 960 + 480], 128, 1);
    EXPECT_NEAR(uv[135 * 960 + 481], 128, 1);
    EXPECT_TRUE(std::equal(frame.begin() + 539 * 960, frame.begin() + 540 * 960, frame.begin() + 543 * 960));
  }
  ASSERT_TRUE(rig.RunUntil(rig.clock + 0.5));
  EXPECT_FALSE(rig.encoder.is_open);
  ASSERT_TRUE(rig.RunUntil(rig.clock + 3.0));
  EXPECT_EQ(rig.camera.calls, "open start stop close ");
}

// After three failures of the encoder the stream is withdrawn and the pictures stay.
TEST(RearLoop, AFailingEncoderLeavesThePictures) {
  Rig rig;
  rig.camera.jpeg = &Drawn().marker;
  rig.WithVideo();
  rig.encoder.fail_open = true;
  rig.Make();
  ASSERT_TRUE(rig.RunUntil(112.0));
  EXPECT_FALSE(rig.loop->VideoActive());
  EXPECT_EQ(rig.encoder.opens, 3);
  EXPECT_EQ(rig.loop->VideoError(), "video: mpp_init -1002");
  test::Client client(rig.server->Port());
  EXPECT_NE(rig.Ask(client, "DESCRIBE " + rig.Url() + " RTSP/1.0\r\nCSeq: 1\r\n\r\n").find("503"),
            std::string::npos);
  rig.viewing = true;
  ASSERT_TRUE(rig.RunUntil(114.0));
  EXPECT_GE(rig.Pictures(), 7U);
  EXPECT_EQ(rig.encoder.opens, 3);
}

// With an RTSP client next to a subscriber of the pictures the reports are those without one, and the stream goes on
// while the marker measures.
TEST(RearLoop, AnRtspClientChangesNoReport) {
  std::vector<std::vector<DockReport>> runs;
  for (bool video : {false, true}) {
    Rig rig;
    rig.camera.jpeg = &Drawn().marker;
    if (video) rig.WithVideo();
    // Encoding costs no time on the clock, so that both runs see the same frames.
    rig.encoder.encode_s = 0.0;
    rig.Make();
    rig.viewing = true;
    std::unique_ptr<test::Client> client;
    if (video) client = rig.Play();
    ASSERT_TRUE(rig.RunUntil(101.0));
    const uint64_t before = video ? rig.stream->Frames() : 0;
    rig.asking = true;
    ASSERT_TRUE(rig.RunUntil(103.0));
    if (video) {
      EXPECT_GT(before, 0U);
      EXPECT_GT(rig.stream->Frames(), before + 40);
    }
    rig.asking = false;
    ASSERT_TRUE(rig.RunUntil(106.0));
    EXPECT_EQ(rig.camera.calls, "open start stop close open start stop close open start ");
    std::vector<DockReport> until_off;
    bool measured = false;
    for (const DockReport& r : rig.reports) {
      until_off.push_back(r);
      measured = measured || r.state == DockState::kMeasuring;
      if (measured && r.state == DockState::kOff) break;
    }
    ASSERT_EQ(until_off.back().state, DockState::kOff);
    runs.push_back(until_off);
  }
  ASSERT_EQ(runs[0].size(), runs[1].size());
  int valid = 0;
  for (size_t i = 0; i < runs[0].size(); i++) {
    const DockReport& a = runs[0][i];
    const DockReport& b = runs[1][i];
    EXPECT_EQ(a.state, b.state) << i;
    EXPECT_EQ(a.stamp, b.stamp) << i;
    EXPECT_EQ(a.reason, b.reason) << i;
    EXPECT_EQ(a.detail, b.detail) << i;
    EXPECT_EQ(a.pose.valid, b.pose.valid) << i;
    EXPECT_EQ(a.exposure.exposure, b.exposure.exposure) << i;
    EXPECT_EQ(a.exposure.gain, b.exposure.gain) << i;
    valid += b.pose.valid ? 1 : 0;
  }
  EXPECT_GE(valid, 20);
}

}  // namespace
}  // namespace tron_vision
