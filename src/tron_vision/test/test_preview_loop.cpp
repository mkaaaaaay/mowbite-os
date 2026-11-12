#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>

#include "test_util.h"
#include "tron_vision/preview_loop.h"

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
    f = test::View(frame_);
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
  Rig() {
    config.temp_file = tmp.Write("temp", "40000\n");
    config.statm = tmp.Write("statm", "30000 5000 900 1 0 20000 0\n");
  }
  PreviewLoop& Loop() {
    if (!loop) loop = std::make_unique<PreviewLoop>(config, camera, guard, [this] { return Now(); });
    return *loop;
  }
  // The clock of the loop is the camera time alone, so the load of the machine changes no result.
  double Now() const {
    return clock;
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
  PreviewState State() {
    return Loop().Status().state;
  }

  test::TempDir tmp;
  double clock = 100.0;
  FakeCamera camera{clock};
  CrashGuard guard{tmp.Path() + "/vision", tmp.Path() + "/crash", tmp.Write("boot_id", "boot-1\n")};
  PreviewLoopConfig config;
  std::unique_ptr<PreviewLoop> loop;
};

TEST(PreviewLoop, NoStartWithoutAViewer) {
  Rig rig;
  rig.Run(30.0);
  EXPECT_EQ(rig.camera.starts, 0);
  EXPECT_EQ(rig.State(), PreviewState::kOff);
  EXPECT_STREQ(StateName(rig.State()), "off");
  EXPECT_EQ(rig.Loop().GetPreview().Count(), 0U);
}

TEST(PreviewLoop, StartsForAViewerWithPicturesAtTheRate) {
  Rig rig;
  rig.Loop().SetWanted(true);
  const double start = rig.Now();
  rig.Run(2.0);
  EXPECT_EQ(rig.camera.starts, 1);
  EXPECT_EQ(rig.State(), PreviewState::kRunning) << rig.Loop().Status().detail;
  EXPECT_TRUE(std::filesystem::exists(rig.tmp.Path() + "/vision/stream-oa_right"));
  // 15 frames per second, at most 5 pictures per second of the loop clock.
  const uint64_t made = rig.Loop().GetPreview().Count();
  EXPECT_GE(made, 8U);
  EXPECT_LE(static_cast<double>(made), 5.0 * (rig.Now() - start) + 1.0);
  EXPECT_EQ(rig.Loop().GetPreview().Width(), kWidth);
  EXPECT_EQ(rig.Loop().GetPreview().Height(), kHeight);
}

TEST(PreviewLoop, StopsTwoSecondsAfterTheLastViewer) {
  Rig rig;
  rig.Loop().SetWanted(true);
  rig.Run(1.0);
  ASSERT_TRUE(rig.camera.streaming);
  rig.Loop().SetWanted(false);
  rig.Loop().Step();
  const double last = rig.Now();
  rig.Run(1.7);
  EXPECT_TRUE(rig.camera.streaming);
  while (rig.camera.streaming && rig.clock < last + 5.0) rig.Loop().Step();
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_NEAR(rig.Now(), last + 2.0, 0.15);
  EXPECT_EQ(rig.State(), PreviewState::kOff);
  EXPECT_FALSE(std::filesystem::exists(rig.tmp.Path() + "/vision/stream-oa_right"));
  const uint64_t made = rig.Loop().GetPreview().Count();
  rig.Run(5.0);
  EXPECT_EQ(rig.Loop().GetPreview().Count(), made);
  // A viewer who comes back gets the camera at once.
  rig.Loop().SetWanted(true);
  rig.Run(0.5);
  EXPECT_EQ(rig.camera.starts, 2);
  EXPECT_EQ(rig.State(), PreviewState::kRunning);
}

TEST(PreviewLoop, NoFramesThreeAttemptsPerViewing) {
  Rig rig;
  rig.camera.delivers = false;
  rig.Loop().SetWanted(true);
  rig.Run(60.0);
  EXPECT_EQ(rig.camera.starts, 3);
  EXPECT_FALSE(rig.camera.streaming);
  EXPECT_EQ(rig.State(), PreviewState::kNoFrames);
  EXPECT_NE(rig.Loop().Status().detail.find("within 2.00 s, attempt 3 of 3"), std::string::npos)
      << rig.Loop().Status().detail;
  // The viewer leaves: the next one starts with three attempts again.
  rig.Loop().SetWanted(false);
  rig.Run(3.0);
  EXPECT_EQ(rig.State(), PreviewState::kOff);
  rig.camera.delivers = true;
  rig.Loop().SetWanted(true);
  rig.Run(1.0);
  EXPECT_EQ(rig.camera.starts, 4);
  EXPECT_EQ(rig.State(), PreviewState::kRunning);
}

TEST(PreviewLoop, FramesThatStopCountAsAttempt) {
  Rig rig;
  rig.Loop().SetWanted(true);
  rig.Run(2.0);
  ASSERT_EQ(rig.State(), PreviewState::kRunning);
  rig.camera.delivers = false;
  rig.Run(1.5);
  EXPECT_EQ(rig.State(), PreviewState::kNoFrames);
  EXPECT_NE(rig.Loop().Status().detail.find("within 1.00 s"), std::string::npos) << rig.Loop().Status().detail;
  rig.camera.delivers = true;
  rig.Run(10.0);
  EXPECT_EQ(rig.camera.starts, 2);
  EXPECT_EQ(rig.State(), PreviewState::kRunning);
}

TEST(PreviewLoop, CameraErrorsCountAsAttempts) {
  Rig rig;
  rig.camera.fail_start = true;
  rig.Loop().SetWanted(true);
  rig.Run(60.0);
  EXPECT_EQ(rig.camera.starts, 3);
  EXPECT_EQ(rig.State(), PreviewState::kCameraError);
  EXPECT_EQ(rig.Loop().Status().detail, "busy");
  EXPECT_FALSE(std::filesystem::exists(rig.tmp.Path() + "/vision/stream-oa_right"));

  Rig next;
  next.camera.fail_next = true;
  next.Loop().SetWanted(true);
  next.Run(60.0);
  EXPECT_EQ(next.camera.starts, 3);
  EXPECT_EQ(next.State(), PreviewState::kCameraError);
  next.Loop().SetWanted(false);
  next.Run(3.0);
  EXPECT_EQ(next.State(), PreviewState::kOff);
}

TEST(PreviewLoop, LockedIsKnownBeforeAViewerAndNeverStarts) {
  Rig rig;
  std::filesystem::create_directories(rig.tmp.Path() + "/vision");
  rig.tmp.Write("vision/locked-oa_right", "1700000000 Oops\n");
  ASSERT_TRUE(rig.Loop().Step());
  EXPECT_EQ(rig.State(), PreviewState::kLocked);
  EXPECT_NE(rig.Loop().Status().detail.find("Oops"), std::string::npos) << rig.Loop().Status().detail;
  rig.Loop().SetWanted(true);
  rig.Run(30.0);
  EXPECT_EQ(rig.camera.starts, 0);
  EXPECT_EQ(rig.State(), PreviewState::kLocked);
}

TEST(PreviewLoop, PausesWhenHotAndResumesForTheViewer) {
  Rig rig;
  rig.Loop().SetWanted(true);
  rig.Run(12.0);
  ASSERT_EQ(rig.State(), PreviewState::kRunning);
  rig.tmp.Write("temp", "81000\n");
  rig.Run(12.0);
  EXPECT_EQ(rig.State(), PreviewState::kPausedHot);
  EXPECT_EQ(rig.Loop().Status().detail, "SoC 81 C");
  EXPECT_FALSE(rig.camera.streaming);
  rig.tmp.Write("temp", "76000\n");
  rig.Run(12.0);
  EXPECT_EQ(rig.State(), PreviewState::kPausedHot);
  rig.tmp.Write("temp", "74000\n");
  rig.Run(20.0);
  EXPECT_EQ(rig.State(), PreviewState::kRunning);
  // A pause is no failed attempt.
  EXPECT_EQ(rig.camera.starts, 2);
}

TEST(PreviewLoop, HeatIsCheckedWithoutAViewer) {
  Rig rig;
  rig.tmp.Write("temp", "81000\n");
  rig.Run(1.0);
  EXPECT_EQ(rig.State(), PreviewState::kPausedHot);
  rig.tmp.Write("temp", "74000\n");
  rig.Run(12.0);
  EXPECT_EQ(rig.State(), PreviewState::kOff);
  EXPECT_EQ(rig.camera.starts, 0);
}

TEST(PreviewLoop, MemoryLimitEndsTheProcess) {
  Rig rig;
  rig.config.statm = rig.tmp.Write("statm", "999999 999999 0 0 0 0 0\n");
  rig.Loop().SetWanted(true);
  EXPECT_FALSE(rig.Loop().Step());
  EXPECT_EQ(rig.State(), PreviewState::kFailed);
  EXPECT_EQ(rig.Loop().Status().exit_code, kExitMemory);
  EXPECT_FALSE(rig.Loop().Step());
  EXPECT_EQ(rig.camera.starts, 0);
}

TEST(PreviewLoop, ExposureAndWhiteBalanceFollowTheScene) {
  Rig rig;
  rig.camera.scene = 0.5;  // needs gain 4096
  rig.camera.u = 113;
  rig.camera.v = 132;
  rig.Loop().SetWanted(true);
  rig.Run(10.0);
  EXPECT_EQ(rig.camera.setting.exposure, 1120);
  EXPECT_NEAR(rig.camera.setting.gain, 4096, 4096 * 0.15);
  EXPECT_EQ(rig.Loop().Status().exposure.gain, rig.camera.setting.gain);
  EXPECT_LT(rig.Loop().Status().display_gains.r, 256);
  EXPECT_GT(rig.Loop().Status().display_gains.b, 256);
}

}  // namespace
}  // namespace tron_vision
