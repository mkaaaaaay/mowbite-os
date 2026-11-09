#include <dlfcn.h>
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>

#include "test_util.h"
#include "tron_vision/vision_loop.h"

namespace tron_vision {
namespace {

constexpr uint32_t kWidth = 320;
constexpr uint32_t kHeight = 180;

// Camera at 15 fps whose luma follows exposure times gain: target luma at 1120 x 2048 in "dark" scenes.
class FakeCamera : public Camera {
 public:
  explicit FakeCamera(double& clock) : clock_(clock), frame_(kWidth, kHeight, 16, 128, 128) {}

  bool Start(std::string& error) override {
    starts++;
    if (fail_start) {
      error = "busy";
      return false;
    }
    streaming = true;
    return true;
  }
  void Stop() override {
    streaming = false;
  }
  int Next(int timeout_ms, Nv12View& f, std::string& error) override {
    if (fail_next) {
      error = "dequeue failed";
      return -1;
    }
    if (!delivers) {
      clock_ += timeout_ms / 1000.0;
      return 0;
    }
    clock_ += 1.0 / 15.0;
    const double light = static_cast<double>(setting.exposure) * setting.gain * scene;
    const auto luma = static_cast<uint8_t>(std::min(255.0, 16.0 + 84.0 * light / (1120.0 * 2048.0)));
    std::fill(frame_.data.begin(), frame_.data.begin() + kWidth * kHeight, luma);
    for (size_t i = kWidth * kHeight; i < frame_.data.size(); i += 2) {
      frame_.data[i] = u;
      frame_.data[i + 1] = v;
    }
    f.y = frame_.data.data();
    f.uv = frame_.data.data() + kWidth * kHeight;
    f.width = kWidth;
    f.height = kHeight;
    f.stride = kWidth;
    frames++;
    return 1;
  }
  bool SetExposure(const ExposureSetting& s, std::string&) override {
    setting = s;
    return true;
  }

  int starts = 0;
  int frames = 0;
  bool streaming = false;
  bool fail_start = false;
  bool fail_next = false;
  bool delivers = true;
  double scene = 1.0;
  uint8_t u = 128;
  uint8_t v = 128;
  ExposureSetting setting;

 private:
  double& clock_;
  test::Nv12Frame frame_;
};

struct Rig {
  explicit Rig(const std::string& model_text, const std::string& labels = "person\ndog\ncat\n") {
    config.library_path = FAKE_NPU_LIB;
    config.model_path = tmp.Write("m.rknn", model_text);
    config.labels_path = tmp.Write("labels.txt", labels);
    config.temp_file = tmp.Write("temp", "40000\n");
  }
  VisionLoop& Loop() {
    if (!loop) {
      loop = std::make_unique<VisionLoop>(config, camera, guard, [this] { return Now(); });
    }
    return *loop;
  }
  // The clock of the loop: camera time plus the real time the test took.
  double Now() const {
    return clock + std::chrono::duration<double>(std::chrono::steady_clock::now() - real_start).count();
  }
  // Steps for `seconds` of camera time, idle steps move the clock on by 0.1 s.
  void Run(double seconds) {
    const double end = clock + seconds;
    for (int i = 0; i < 100000 && clock < end; i++) {
      const int frames = camera.frames;
      const double before = clock;
      if (!Loop().Step()) return;
      if (camera.frames == frames && clock == before) clock += 0.1;
    }
  }

  test::TempDir tmp;
  double clock = 100.0;
  std::chrono::steady_clock::time_point real_start = std::chrono::steady_clock::now();
  FakeCamera camera{clock};
  CrashGuard guard{tmp.Path() + "/vision", tmp.Path() + "/crash", tmp.Write("boot_id", "boot-1\n")};
  VisionConfig config;
  std::unique_ptr<VisionLoop> loop;
};

constexpr const char* kPersonModel = "classes 3\nhit 0 0.8 8 30 40 2 2 2 2\n";

TEST(VisionLoop, PicturesOnlyWhileWatchedAndRunning) {
  Rig rig(kPersonModel);
  ASSERT_TRUE(rig.Loop().Load());
  rig.Run(2.0);
  EXPECT_EQ(rig.Loop().GetPreview().Count(), 0U);
  rig.Loop().SetPreviewWanted(true);
  const double start = rig.Now();
  rig.Run(2.0);
  const uint64_t made = rig.Loop().GetPreview().Count();
  // 15 frames per second, at most 5 pictures per second of the loop clock.
  EXPECT_GE(made, 8U);
  EXPECT_LE(static_cast<double>(made), 5.0 * (rig.Now() - start) + 1.0);
  EXPECT_EQ(rig.Loop().GetPreview().Width(), kWidth);
  rig.Loop().SetPreviewWanted(false);
  rig.Run(2.0);
  EXPECT_EQ(rig.Loop().GetPreview().Count(), made);
}

TEST(VisionLoop, RefusesTheSmallModelAndNeverStreams) {
  Rig rig("classes 1\n", "stone\nleaf\n");
  EXPECT_FALSE(rig.Loop().Load());
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kModelMismatch);
  EXPECT_STREQ(StateName(rig.Loop().Status().state), "model_mismatch");
  rig.Run(30.0);
  EXPECT_EQ(rig.camera.starts, 0);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kModelMismatch);
}

TEST(VisionLoop, HeatKeepsTheReasonOfAStoppedCamera) {
  Rig rig("classes 1\n", "stone\nleaf\n");
  EXPECT_FALSE(rig.Loop().Load());
  rig.tmp.Write("temp", "85000\n");
  rig.Run(30.0);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kModelMismatch);
}

TEST(VisionLoop, MissingFilesAndBrokenModel) {
  Rig missing(kPersonModel);
  missing.config.model_path = missing.tmp.Path() + "/none.rknn";
  EXPECT_FALSE(missing.Loop().Load());
  EXPECT_EQ(missing.Loop().Status().state, VisionState::kModelMissing);
  EXPECT_NE(missing.Loop().Status().detail.find("none.rknn"), std::string::npos);

  Rig unset(kPersonModel);
  unset.config.library_path.clear();
  EXPECT_FALSE(unset.Loop().Load());
  EXPECT_EQ(unset.Loop().Status().detail, "path not set");

  Rig broken("fail_init\n");
  EXPECT_FALSE(broken.Loop().Load());
  EXPECT_EQ(broken.Loop().Status().state, VisionState::kModelError);
  broken.Run(10.0);
  EXPECT_EQ(broken.camera.starts, 0);
}

TEST(VisionLoop, DetectsAndReports) {
  Rig rig(kPersonModel);
  ASSERT_TRUE(rig.Loop().Load()) << rig.Loop().Status().detail;
  EXPECT_NE(rig.Loop().Status().detail.find("3 classes"), std::string::npos);
  rig.Run(5.0);
  const VisionStatus& st = rig.Loop().Status();
  EXPECT_EQ(st.state, VisionState::kRunning) << st.detail;
  EXPECT_EQ(rig.camera.starts, 1);
  EXPECT_EQ(st.last, "person 0.80");
  EXPECT_NEAR(st.rate, 2.0, 0.3);
  // The clock of the rig also runs in real time, so a slow build sees more.
  EXPECT_GE(st.inferences, 9U);
  EXPECT_EQ(st.frame_width, kWidth);
  EXPECT_EQ(st.frame_height, kHeight);
  ASSERT_EQ(rig.Loop().Detections().size(), 1U);
  const Detection& d = rig.Loop().Detections()[0];
  // Cell (30,40) of stride 8 with 2 cells each side: model x 228..260, y 308..340; crop of 320x180 is 135x180 at 92.5.
  const float k = 180.0F / 640.0F;
  EXPECT_NEAR(d.x1, 92.5F + 228.0F * k, 0.1F);
  EXPECT_NEAR(d.x2, 92.5F + 260.0F * k, 0.1F);
  EXPECT_NEAR(d.y1, 308.0F * k, 0.1F);
  EXPECT_TRUE(std::filesystem::exists(rig.tmp.Path() + "/vision/stream-oa_left"));
  rig.Loop().Shutdown();
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_FALSE(std::filesystem::exists(rig.tmp.Path() + "/vision/stream-oa_left"));
}

TEST(VisionLoop, InferenceRateIsBounded) {
  Rig rig(kPersonModel);
  rig.config.infer_rate = 50.0;
  ASSERT_TRUE(rig.Loop().Load());
  const auto began = std::chrono::steady_clock::now();
  rig.Run(5.0);
  // A late frame lets the next inference come sooner, so single intervals vary; the count holds. The clock of the rig
  // also runs in real time.
  const double seconds = 5.0 + std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
  const auto count = static_cast<double>(rig.Loop().Status().inferences);
  EXPECT_LE(count, 5.0 * seconds + 2.0);
  EXPECT_GE(count, 4.0 * 5.0);
}

TEST(VisionLoop, NoFramesStopsAfterThreeAttempts) {
  Rig rig(kPersonModel);
  rig.camera.delivers = false;
  ASSERT_TRUE(rig.Loop().Load());
  rig.Run(120.0);
  EXPECT_EQ(rig.camera.starts, 3);
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kNoFrames);
  EXPECT_NE(rig.Loop().Status().detail.find("within 2.00 s"), std::string::npos) << rig.Loop().Status().detail;
  EXPECT_FALSE(std::filesystem::exists(rig.tmp.Path() + "/vision/stream-oa_left"));
}

TEST(VisionLoop, FramesThatStopCountAsAttempt) {
  Rig rig(kPersonModel);
  ASSERT_TRUE(rig.Loop().Load());
  rig.Run(2.0);
  ASSERT_EQ(rig.Loop().Status().state, VisionState::kRunning);
  rig.camera.delivers = false;
  rig.Run(1.5);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kNoFrames);
  EXPECT_NE(rig.Loop().Status().detail.find("within 1.00 s"), std::string::npos) << rig.Loop().Status().detail;
  rig.camera.delivers = true;
  rig.Run(10.0);
  EXPECT_EQ(rig.camera.starts, 2);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kRunning);
}

TEST(VisionLoop, CameraErrorsCountAsAttempts) {
  Rig rig(kPersonModel);
  rig.camera.fail_start = true;
  ASSERT_TRUE(rig.Loop().Load());
  rig.Run(60.0);
  EXPECT_EQ(rig.camera.starts, 3);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kCameraError);
  EXPECT_EQ(rig.Loop().Status().detail, "busy");
  EXPECT_FALSE(std::filesystem::exists(rig.tmp.Path() + "/vision/stream-oa_left"));

  Rig next(kPersonModel);
  next.camera.fail_next = true;
  ASSERT_TRUE(next.Loop().Load());
  next.Run(60.0);
  EXPECT_EQ(next.camera.starts, 3);
  EXPECT_EQ(next.Loop().Status().state, VisionState::kCameraError);
}

TEST(VisionLoop, LockedCameraNeverStarts) {
  Rig rig(kPersonModel);
  std::filesystem::create_directories(rig.tmp.Path() + "/vision");
  rig.tmp.Write("vision/locked-oa_left", "1700000000 Oops\n");
  ASSERT_TRUE(rig.Loop().Load());
  rig.Run(30.0);
  EXPECT_EQ(rig.camera.starts, 0);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kLocked);
}

TEST(VisionLoop, ExposureFollowsTheScene) {
  Rig rig(kPersonModel);
  rig.camera.scene = 0.5;  // needs gain 4096
  ASSERT_TRUE(rig.Loop().Load());
  rig.Run(10.0);
  EXPECT_EQ(rig.camera.setting.exposure, 1120);
  EXPECT_NEAR(rig.camera.setting.gain, 4096, 4096 * 0.15);
  rig.camera.scene = 200.0;  // backlight: even 1120 x 64 is far too bright
  rig.Run(10.0);
  EXPECT_EQ(rig.camera.setting.gain, 64);
  EXPECT_LT(rig.camera.setting.exposure, 300);
  EXPECT_EQ(rig.Loop().Status().exposure.exposure, rig.camera.setting.exposure);
}

TEST(VisionLoop, WhiteBalanceOnlyForTheInferenceWhenSwitchedOn) {
  using LastInput = const uint8_t* (*)(size_t*);
  for (bool on : {false, true}) {
    Rig rig(kPersonModel);
    rig.config.white_balance_inference = on;
    rig.camera.u = 113;
    rig.camera.v = 132;
    ASSERT_TRUE(rig.Loop().Load());
    rig.Run(5.0);
    EXPECT_LT(rig.Loop().Status().display_gains.r, 256);
    EXPECT_GT(rig.Loop().Status().display_gains.b, 256);
    void* lib = dlopen(FAKE_NPU_LIB, RTLD_NOW | RTLD_NOLOAD);
    ASSERT_NE(lib, nullptr);
    auto last = reinterpret_cast<LastInput>(dlsym(lib, "fake_last_input"));
    ASSERT_NE(last, nullptr);
    size_t size = 0;
    const uint8_t* rgb = last(&size);
    ASSERT_EQ(size, 640U * 480U * 3U);
    const size_t mid = (320 * 480 + 240) * 3;
    if (on) {
      EXPECT_NEAR(rgb[mid], rgb[mid + 2], 3);
    } else {
      EXPECT_GT(rgb[mid] - rgb[mid + 2], 15);
    }
    dlclose(lib);
  }
}

TEST(VisionLoop, SlowInferenceEndsTheProcess) {
  Rig rig("classes 3\nrun_ms 3\n");
  rig.config.slow_run_s = 0.001;
  ASSERT_TRUE(rig.Loop().Load());
  rig.Run(30.0);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kFailed);
  EXPECT_EQ(rig.Loop().Status().exit_code, kExitSlowNpu);
  EXPECT_FALSE(rig.Loop().Step());
  EXPECT_FALSE(rig.camera.streaming);
}

TEST(VisionLoop, NpuErrorEndsTheProcess) {
  Rig rig("classes 3\nfail_run\n");
  ASSERT_TRUE(rig.Loop().Load());
  rig.Run(5.0);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kFailed);
  EXPECT_NE(rig.Loop().Status().detail.find("rknn_run"), std::string::npos);
}

TEST(VisionLoop, MemoryLimitEndsTheProcess) {
  Rig rig(kPersonModel);
  rig.config.statm = rig.tmp.Write("statm", "999999 999999 0 0 0 0 0\n");
  ASSERT_TRUE(rig.Loop().Load());
  EXPECT_FALSE(rig.Loop().Step());
  EXPECT_EQ(rig.Loop().Status().exit_code, kExitMemory);
  EXPECT_EQ(rig.camera.starts, 0);
}

TEST(VisionLoop, PausesWhenHot) {
  Rig rig(kPersonModel);
  ASSERT_TRUE(rig.Loop().Load());
  rig.Run(12.0);
  ASSERT_EQ(rig.Loop().Status().state, VisionState::kRunning);
  rig.tmp.Write("temp", "81000\n");
  rig.Run(12.0);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kPausedHot);
  EXPECT_FALSE(rig.camera.streaming);
  rig.tmp.Write("temp", "76000\n");
  rig.Run(12.0);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kPausedHot);
  rig.tmp.Write("temp", "74000\n");
  rig.Run(20.0);
  EXPECT_EQ(rig.Loop().Status().state, VisionState::kRunning);
  // A pause is no failed attempt.
  EXPECT_EQ(rig.camera.starts, 2);
}

}  // namespace
}  // namespace tron_vision
