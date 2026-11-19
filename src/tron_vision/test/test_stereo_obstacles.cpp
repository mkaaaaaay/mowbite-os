#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <memory>
#include <vector>

#include "stereo_scene.h"
#include "test_util.h"
#include "tron_vision/frame_convert.h"
#include "tron_vision/jpeg_luma.h"
#include "tron_vision/model_check.h"
#include "tron_vision/obstacle_core.h"
#include "tron_vision/obstacle_sight.h"
#include "tron_vision/stereo_depth.h"
#include "tron_vision/stereo_lens.h"

namespace tron_vision {
namespace {

using namespace std::chrono_literals;
using test::Block;
using test::Scene;

constexpr char kCalibration[] =
    "# fisheye, 16.11.\n"
    "size 640 480\n"
    "left 446.43 447.85 318.99 232.69 -0.0336 -0.3081 0.6034 -0.3920\n"
    "right 445.88 448.23 314.35 231.34 -0.0666 -0.1297 0.2200 -0.1225\n"
    "rotation 0.99996 0.00309 0.00856 -0.00312 0.99999 0.00287 -0.00855 -0.00290 0.99996\n"
    "translation -0.06033 0.00015 0.00325\n";

// The depth along the view of the left eye at a pixel, where its ray meets the face of the block.
double DepthAt(const Scene& s, const Block& b, const Point2& pixel) {
  Point2 ray;
  EXPECT_TRUE(ToRay(s.calibration.left, pixel, ray));
  const test::V3 d = test::CameraToBase(s.mount, {ray.x, ray.y, 1.0});
  return (b.near - s.mount.x) / d.x;
}

// The box of the face of a block in the left image.
PixelBox BoxOf(const Scene& s, const Block& b) {
  PixelBox box{1e9, 1e9, -1e9, -1e9};
  for (const double y : {b.right, b.left}) {
    for (const double z : {s.ground_z, s.ground_z + b.height}) {
      Point2 p;
      EXPECT_TRUE(s.Project({b.near, y, z}, p));
      box.x1 = std::min(box.x1, p.x);
      box.y1 = std::min(box.y1, p.y);
      box.x2 = std::max(box.x2, p.x);
      box.y2 = std::max(box.y2, p.y);
    }
  }
  return box;
}

TEST(FisheyeLens, PixelAndRayRoundTrip) {
  const StereoCalibration c = test::TronCalibration();
  for (double y = 0.0; y < 480.0; y += 40.0) {
    for (double x = 0.0; x < 640.0; x += 40.0) {
      Point2 ray;
      // The calibration of 16.11. holds up to about 350 px from the center; the corners lie beyond.
      if (std::hypot(x - c.left.cx, y - c.left.cy) > 350.0) continue;
      ASSERT_TRUE(ToRay(c.left, {x, y}, ray)) << x << " " << y;
      const Point2 back = ToPixel(c.left, ray);
      EXPECT_NEAR(back.x, x, 1e-6);
      EXPECT_NEAR(back.y, y, 1e-6);
    }
  }
  Point2 ray;
  ASSERT_TRUE(ToRay(c.left, {c.left.cx, c.left.cy}, ray));
  EXPECT_NEAR(ray.x, 0.0, 1e-12);
  EXPECT_NEAR(ray.y, 0.0, 1e-12);
}

TEST(FisheyeLens, EdgeOfTheImageIsAboutFortyThreeDegrees) {
  const StereoCalibration c = test::TronCalibration();
  Point2 left;
  Point2 right;
  ASSERT_TRUE(ToRay(c.left, {0.0, c.left.cy}, left));
  ASSERT_TRUE(ToRay(c.left, {639.0, c.left.cy}, right));
  // 86 degrees across (K0 16.11.).
  const double across = (std::atan(-left.x) + std::atan(right.x)) * 180.0 / M_PI;
  EXPECT_NEAR(across, 86.0, 2.0);
}

TEST(FisheyeLens, FoldingDistortionIsRefused) {
  FisheyeLens lens{400.0, 400.0, 320.0, 240.0, -0.5, 0.0, 0.0, 0.0};
  Point2 ray;
  // theta (1 - 0.5 theta^2) peaks at 0.38 below theta 0.82: farther out has no ray.
  EXPECT_FALSE(ToRay(lens, {320.0 + 400.0 * 0.6, 240.0}, ray));
  EXPECT_TRUE(ToRay(lens, {320.0 + 400.0 * 0.2, 240.0}, ray));
}

TEST(StereoCalibration, ReadsTheFile) {
  test::TempDir tmp;
  StereoCalibration c;
  std::string error;
  ASSERT_TRUE(ReadStereoCalibration(tmp.Write("c.txt", kCalibration), c, error)) << error;
  EXPECT_EQ(c.width, 640U);
  EXPECT_DOUBLE_EQ(c.left.fx, 446.43);
  EXPECT_DOUBLE_EQ(c.right.k4, -0.1225);
  EXPECT_DOUBLE_EQ(c.rotation[8], 0.99996);
  EXPECT_DOUBLE_EQ(c.translation[0], -0.06033);
  EXPECT_NEAR(DisparityAtOneMeter(c), 446.43 * 0.06042, 0.05);
}

TEST(StereoCalibration, RefusesWhatDoesNotFit) {
  test::TempDir tmp;
  const std::string good = kCalibration;
  const auto replaced = [&good](const std::string& from, const std::string& to) {
    std::string text = good;
    text.replace(text.find(from), from.size(), to);
    return text;
  };
  const std::vector<std::pair<std::string, std::string>> cases = {
      {"missing line", replaced("size 640 480\n", "")},
      {"repeated line", good + "size 640 480\n"},
      {"unknown name", good + "skew 0\n"},
      {"too few numbers", replaced("translation -0.06033 0.00015 0.00325", "translation -0.06 0.0")},
      {"word for a number", replaced("translation -0.06033", "translation x")},
      {"no rotation", replaced("rotation 0.99996", "rotation 0.5")},
      {"right eye on the left", replaced("translation -0.06033", "translation 0.06033")},
      {"center outside", replaced("left 446.43 447.85 318.99", "left 446.43 447.85 700.0")},
      {"lens folds back", replaced("-0.0336 -0.3081 0.6034 -0.3920", "-0.9 0.0 0.0 0.0")},
  };
  for (const auto& [name, text] : cases) {
    StereoCalibration c;
    std::string error;
    EXPECT_FALSE(ReadStereoCalibration(tmp.Write("c.txt", text), c, error)) << name;
    EXPECT_FALSE(error.empty()) << name;
  }
  StereoCalibration c;
  std::string error;
  EXPECT_FALSE(ReadStereoCalibration(tmp.Path() + "/none.txt", c, error));
  EXPECT_NE(error.find("missing"), std::string::npos);
}

TEST(StereoCalibration, RightPixelMovesByTheDisparity) {
  const StereoCalibration c = test::TronCalibration();
  Point2 ray;
  ASSERT_TRUE(ToRay(c.left, {320.0, 240.0}, ray));
  for (const double depth : {0.5, 1.0, 2.0, 3.0}) {
    Point2 right;
    ASSERT_TRUE(ToRightPixel(c, ray, depth, right));
    EXPECT_NEAR(320.0 - right.x, DisparityAtOneMeter(c) / depth, 1.5) << depth;
  }
}

TEST(StereoDepth, MatchFindsTheDepthOfATexturedFace) {
  Scene s;
  for (const double near : {1.0, 1.6, 2.5}) {
    s.blocks = {{near, near + 0.3, -0.2, 0.2, 0.5}};
    const std::vector<uint8_t> frame = s.Render();
    const StereoPair pair = PairOf(frame.data(), 640, 480, 2560);
    const PixelBox box = BoxOf(s, s.blocks[0]);
    int found = 0;
    int close = 0;
    for (const double fx : {0.2, 0.35, 0.5, 0.65, 0.8}) {
      for (const double fy : {0.2, 0.4, 0.6, 0.8}) {
        const Point2 p{box.x1 + fx * (box.x2 - box.x1), box.y1 + fy * (box.y2 - box.y1)};
        double depth = 0.0;
        if (!MatchPoint(s.calibration, pair, p, MatchParams{}, depth)) continue;
        found++;
        const double expected = DepthAt(s, s.blocks[0], {std::round(p.x), std::round(p.y)});
        // Twice the noise of a quarter pixel of disparity the calibration showed.
        if (std::abs(depth - expected) < 0.5 * expected * expected / DisparityAtOneMeter(s.calibration) + 0.002) {
          close++;
        }
      }
    }
    // Single points may match wrong; the box takes their median.
    EXPECT_GE(found, 16) << near;
    EXPECT_GE(close, found * 8 / 10) << near;
  }
}

TEST(StereoDepth, NoMatchWithoutTextureOrOutOfRange) {
  Scene s;
  s.cell = 100.0;
  const std::vector<uint8_t> flat = s.Render();
  double depth = 0.0;
  EXPECT_FALSE(MatchPoint(s.calibration, PairOf(flat.data(), 640, 480, 2560), {320.0, 400.0}, MatchParams{}, depth));
  // Farther than the range: the face at 4 m.
  Scene far;
  far.blocks = {{4.0, 4.3, -1.0, 1.0, 1.5}};
  const std::vector<uint8_t> frame = far.Render();
  const PixelBox box = BoxOf(far, far.blocks[0]);
  EXPECT_FALSE(MatchPoint(far.calibration, PairOf(frame.data(), 640, 480, 2560),
                          {0.5 * (box.x1 + box.x2), 0.5 * (box.y1 + box.y2)}, MatchParams{}, depth));
  // Near the edge of the image the patch does not fit.
  EXPECT_FALSE(
      MatchPoint(far.calibration, PairOf(frame.data(), 640, 480, 2560), {1.0, 240.0}, MatchParams{}, depth));
}

TEST(StereoDepth, BoxDepthIsTheMedianOfItsLowerPart) {
  Scene s;
  s.blocks = {{1.4, 1.7, -0.15, 0.15, 0.4}};
  const std::vector<uint8_t> frame = s.Render();
  const PixelBox box = BoxOf(s, s.blocks[0]);
  const BoxDepth d = DepthOfBox(s.calibration, PairOf(frame.data(), 640, 480, 2560), box, MatchParams{},
                                BoxDepthParams{});
  ASSERT_TRUE(d.ok);
  EXPECT_EQ(d.points, 32);
  EXPECT_GE(d.matched, 16);
  EXPECT_NEAR(d.depth, DepthAt(s, s.blocks[0], d.pixel), 0.02);
  // A box with nothing to match.
  Scene flat;
  flat.cell = 100.0;
  const std::vector<uint8_t> plain = flat.Render();
  EXPECT_FALSE(DepthOfBox(flat.calibration, PairOf(plain.data(), 640, 480, 2560), box, MatchParams{},
                          BoxDepthParams{})
                   .ok);
}

TEST(StereoDepth, GroundFillsTheLowerHalf) {
  Scene s;
  const std::vector<uint8_t> frame = s.Render();
  EXPECT_GT(MatchedShare(s.calibration, PairOf(frame.data(), 640, 480, 2560), MatchParams{}), 0.6);
  std::vector<uint8_t> gray(frame.size(), 128);
  EXPECT_EQ(MatchedShare(s.calibration, PairOf(gray.data(), 640, 480, 2560), MatchParams{}), 0.0);
}

// A frame of both eyes, each of one luma, the lower half of the left one optionally another.
std::vector<uint8_t> Flat(uint8_t left, uint8_t right, int low_left = -1) {
  std::vector<uint8_t> f(4 * 640 * 480, 128);
  for (int y = 0; y < 480; y++) {
    for (int x = 0; x < 640; x++) {
      f[y * 2560 + 2 * x] = (low_left >= 0 && y >= 240) ? static_cast<uint8_t>(low_left) : left;
      f[y * 2560 + 1280 + 2 * x] = right;
    }
  }
  return f;
}

SightSample Sample(const std::vector<uint8_t>& frame, double taken) {
  SightSample s = MeasureSight(PairOf(frame.data(), 640, 480, 2560), 245);
  s.taken = taken;
  return s;
}

TEST(Sight, MeasuresLumaBrightAndEdges) {
  Scene scene;
  const std::vector<uint8_t> textured = scene.Render();
  const SightSample t = Sample(textured, 0.0);
  EXPECT_GT(t.edges_left, 5.0);
  EXPECT_NEAR(t.edges_left / t.edges_right, 1.0, 0.2);
  const SightSample half = Sample(Flat(100, 100, 250), 0.0);
  EXPECT_NEAR(half.luma_low, 250.0, 0.5);
  EXPECT_NEAR(half.bright_low, 1.0, 1e-9);
  EXPECT_NEAR(half.bright_all, 0.5, 0.02);
  EXPECT_EQ(half.edges_right, 0.0);
  EXPECT_NE(Sample(Flat(100, 100), 0.0).field, Sample(Flat(101, 100), 0.0).field);
}

TEST(Sight, NoFrameAndStillFramesGiveNoSignal) {
  SightCheck check{SightLimits{}};
  std::string detail;
  EXPECT_EQ(check.Reason(0.0, detail), SightReason::kNoSignal);
  Scene scene;
  const std::vector<uint8_t> frame = scene.Render();
  check.Frame(1.0);
  check.Add(Sample(frame, 1.0));
  EXPECT_EQ(check.Reason(1.4, detail), SightReason::kNone) << detail;
  EXPECT_EQ(check.Reason(1.6, detail), SightReason::kNoSignal);
  for (int i = 1; i < 5; i++) {
    check.Frame(1.0 + 0.2 * i);
    check.Add(Sample(frame, 1.0 + 0.2 * i));
    EXPECT_EQ(check.Reason(1.0 + 0.2 * i, detail), i < 4 ? SightReason::kNone : SightReason::kNoSignal) << i;
  }
  EXPECT_NE(detail.find("alike"), std::string::npos);
  check.Reset();
  EXPECT_EQ(check.Reason(2.0, detail), SightReason::kNoSignal);
  EXPECT_EQ(detail, "no frame yet");
}

// Fed with a frame every 0.2 s from `from` to `to`; the reason at the end.
SightReason Feed(SightCheck& check, const std::vector<uint8_t>& frame, double from, double to, bool max_gain = true,
                 double matched = -1.0) {
  // A different field each time, so the frames do not stand still.
  std::vector<uint8_t> f = frame;
  int n = 0;
  for (double t = from; t <= to + 1e-9; t += 0.2) {
    f[(360 * 2560) + 2 * 320] = static_cast<uint8_t>(f[(360 * 2560) + 2 * 320] ^ (++n & 1));
    check.Frame(t);
    SightSample s = Sample(f, t);
    s.at_max_gain = max_gain;
    s.matched = matched;
    check.Add(s);
  }
  std::string detail;
  return check.Reason(to, detail);
}

TEST(Sight, TooDarkAfterTwoSecondsAtHighestGainAfterTenAtAnyGain) {
  SightCheck check{SightLimits{}};
  EXPECT_EQ(Feed(check, Flat(30, 30), 0.0, 1.8), SightReason::kNone);
  EXPECT_EQ(Feed(check, Flat(30, 30), 2.0, 2.2), SightReason::kTooDark);
  // Back only from 45.
  EXPECT_EQ(Feed(check, Flat(40, 40), 2.4, 4.0), SightReason::kTooDark);
  EXPECT_EQ(Feed(check, Flat(46, 46), 4.2, 4.2), SightReason::kNone);
  SightCheck low_gain{SightLimits{}};
  EXPECT_EQ(Feed(low_gain, Flat(30, 30), 0.0, 9.8, false), SightReason::kNone);
  EXPECT_EQ(Feed(low_gain, Flat(30, 30), 10.0, 10.0, false), SightReason::kTooDark);
}

TEST(Sight, TooBrightAfterOneSecondBackBelowTenPercent) {
  SightCheck check{SightLimits{}};
  EXPECT_EQ(Feed(check, Flat(100, 100, 250), 0.0, 0.8), SightReason::kNone);
  EXPECT_EQ(Feed(check, Flat(100, 100, 250), 1.0, 1.0), SightReason::kTooBright);
  EXPECT_EQ(Feed(check, Flat(100, 100), 1.2, 1.2), SightReason::kNone);
}

TEST(Sight, CoveredByLumaEdgesOrMissingDepth) {
  SightCheck dark_eye{SightLimits{}};
  EXPECT_EQ(Feed(dark_eye, Flat(120, 60), 0.0, 1.8), SightReason::kNone);
  EXPECT_EQ(Feed(dark_eye, Flat(120, 60), 2.0, 2.0), SightReason::kCovered);
  EXPECT_EQ(Feed(dark_eye, Flat(120, 110), 2.2, 2.2), SightReason::kNone);

  Scene scene;
  std::vector<uint8_t> blurred = scene.Render();
  // A film over the right eye: its picture gone, its mean luma kept.
  const double mean = Sample(blurred, 0.0).luma_right;
  for (int y = 0; y < 480; y++) {
    for (int x = 0; x < 640; x++) blurred[y * 2560 + 1280 + 2 * x] = static_cast<uint8_t>(std::lround(mean));
  }
  SightCheck edges{SightLimits{}};
  EXPECT_EQ(Feed(edges, blurred, 0.0, 2.2), SightReason::kCovered);

  SightCheck depth{SightLimits{}};
  const std::vector<uint8_t> good = scene.Render();
  EXPECT_EQ(Feed(depth, good, 0.0, 3.0, true, 0.8), SightReason::kNone);
  EXPECT_EQ(Feed(depth, good, 3.2, 5.4, true, 0.1), SightReason::kCovered);
  // Standing, the depth is not asked.
  EXPECT_EQ(Feed(depth, good, 5.6, 5.6, true, -1.0), SightReason::kNone);
}

TEST(ConvertYuyv, PadsTheEyeIntoTheUprightInput) {
  std::vector<uint8_t> frame = Flat(180, 60);
  const Placement p = PlaceFrame(640, 480, Fit::kPad, kModelWidth, kModelHeight);
  EXPECT_EQ(p.dst_w, 480U);
  EXPECT_EQ(p.dst_h, 360U);
  EXPECT_EQ(p.dst_y, 140U);
  std::vector<uint8_t> rgb;
  ConvertYuyv(frame.data(), 640, 480, 2560, p, false, kModelWidth, kModelHeight, rgb);
  ASSERT_EQ(rgb.size(), static_cast<size_t>(kModelWidth) * kModelHeight * 3);
  const auto at = [&rgb](int x, int y, int c) { return rgb[(static_cast<size_t>(y) * kModelWidth + x) * 3 + c]; };
  // Luma 180 with neutral chroma: (180 - 16) * 298 / 256.
  EXPECT_EQ(at(240, 320, 0), 191);
  EXPECT_EQ(at(240, 320, 1), 191);
  EXPECT_EQ(at(240, 320, 2), 191);
  // Only the left eye, and the bars above and below gray.
  EXPECT_EQ(at(470, 320, 0), 191);
  EXPECT_EQ(at(240, 10, 0), at(240, 630, 0));
  EXPECT_NE(at(240, 10, 0), 191);
}

TEST(YuyvJpeg, EncodesTheLeftEye) {
  Scene scene;
  const std::vector<uint8_t> frame = scene.Render();
  YuyvJpeg jpeg;
  std::vector<uint8_t> out;
  std::string error;
  ASSERT_TRUE(jpeg.Encode(frame.data(), 640, 480, 2560, 80, out, error)) << error;
  EXPECT_LT(out.size(), 200U * 1024);
  JpegLuma luma;
  GrayView view;
  ASSERT_TRUE(luma.Decode(out.data(), out.size(), 640, 480, view, error)) << error;
  EXPECT_NEAR(view.data[400 * view.stride + 320], frame[400 * 2560 + 640], 30);
  EXPECT_FALSE(jpeg.Encode(frame.data(), 639, 480, 2560, 80, out, error));
}

// The model input of an eye pixel: the eye scaled by 3/4 into 480 x 360, 140 lines below the top.
Point2 ModelPixel(double x, double y) {
  return {0.75 * x, 0.75 * y + 140.0};
}

// A hit line for the stand-in runtime at stride 8 that gives about this box of the left eye.
std::string HitFor(const PixelBox& box, int cls, double score = 0.9) {
  const Point2 a = ModelPixel(box.x1, box.y1);
  const Point2 b = ModelPixel(box.x2, box.y2);
  const int col = static_cast<int>(std::floor(0.5 * (a.x + b.x) / 8.0));
  const int row = static_cast<int>(std::floor(0.5 * (a.y + b.y) / 8.0));
  const auto side = [](double v) { return std::to_string(std::clamp(std::lround(v), 0L, 15L)); };
  return "hit " + std::to_string(cls) + " " + std::to_string(score) + " 8 " + std::to_string(col) + " " +
         std::to_string(row) + " " + side(col + 0.5 - a.x / 8.0) + " " + side(row + 0.5 - a.y / 8.0) + " " +
         side(b.x / 8.0 - col - 0.5) + " " + side(b.y / 8.0 - row - 0.5) + "\n";
}

struct CoreRig {
  explicit CoreRig(const std::string& hits = "") {
    config.library_path = FAKE_NPU_LIB;
    config.model_path = tmp.Write("m.rknn", "classes 5\n" + hits);
    config.labels_path = tmp.Write("labels.txt", "person\ndog\ncat\nmanhole\nstone\n");
    config.calibration_path = tmp.Write("c.txt", kCalibration);
    config.have_mount = true;
    config.mount = scene.mount;
    config.person = {"person"};
    config.animal = {"dog", "cat"};
    config.ignore = {"manhole"};
    camera.state = StereoState::kReady;
    camera.capturing = true;
  }
  ObstacleCore& Core() {
    if (!core) core = std::make_unique<ObstacleCore>(config, [this] { return clock; });
    return *core;
  }
  // One frame of the scene to the core, then the reports until one with sight or `wait_s` of real time.
  // `stamp_offset` (s) shifts the frame time of the driver against the clock.
  bool Analyse(ObstacleReport& report, bool moving = false, double wait_s = 3.0, double stamp_offset = 0.0) {
    if (frame.empty()) frame = scene.Render();
    // A camera never gives two frames alike.
    frame[360 * 2560 + 2 * 330] ^= 1;
    const auto stamp = static_cast<int64_t>((clock + stamp_offset) * 1e6);
    CapturedFrame f{0, frame.data(), frame.size(), 2560, ++sequence, stamp, -1};
    Core().OnFrame(f, clock, {moving, true});
    const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(wait_s);
    while (std::chrono::steady_clock::now() < until) {
      clock += 0.0005;
      if (Core().Poll(clock, camera, report) && report.reason == SightReason::kNone) return true;
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  test::TempDir tmp;
  Scene scene;
  ObstacleConfig config;
  StereoStatus camera;
  double clock = 50.0;
  uint32_t sequence = 0;
  std::vector<uint8_t> frame;
  std::unique_ptr<ObstacleCore> core;
};

TEST(ObstacleCore, NotSetUpSaysWhatIsMissing) {
  CoreRig no_mount;
  no_mount.config.have_mount = false;
  EXPECT_FALSE(no_mount.Core().Load());
  ObstacleReport report;
  ASSERT_TRUE(no_mount.Core().Poll(1.0, no_mount.camera, report));
  EXPECT_EQ(report.reason, SightReason::kNotSetUp);
  EXPECT_EQ(report.detail, "no mount");

  CoreRig no_calibration;
  no_calibration.config.calibration_path = no_calibration.tmp.Path() + "/none.txt";
  EXPECT_FALSE(no_calibration.Core().Load());
  ASSERT_TRUE(no_calibration.Core().Poll(1.0, no_calibration.camera, report));
  EXPECT_EQ(report.reason, SightReason::kNotSetUp);
  EXPECT_NE(report.detail.find("calibration"), std::string::npos);

  CoreRig no_model;
  no_model.config.model_path = no_model.tmp.Path() + "/none.rknn";
  EXPECT_FALSE(no_model.Core().Load());
  ASSERT_TRUE(no_model.Core().Poll(1.0, no_model.camera, report));
  EXPECT_NE(report.detail.find("model"), std::string::npos);

  CoreRig from_platform;
  from_platform.config.not_set_up = "no wheel_radius";
  EXPECT_FALSE(from_platform.Core().Load());

  // Looking at the sky the camera sees no ground.
  CoreRig up;
  up.config.mount.pitch = -0.8;
  EXPECT_FALSE(up.Core().Load());

  // Class lists that do not fit the model: a person could count as an object.
  CoreRig wrong_class;
  wrong_class.config.animal = {"dog", "hedgehog"};
  EXPECT_FALSE(wrong_class.Core().Load());
  ASSERT_TRUE(wrong_class.Core().Poll(1.0, wrong_class.camera, report));
  EXPECT_EQ(report.reason, SightReason::kNotSetUp);
  EXPECT_NE(report.detail.find("hedgehog"), std::string::npos);

  CoreRig misspelt_person;
  misspelt_person.config.person = {"Person"};
  EXPECT_FALSE(misspelt_person.Core().Load());

  CoreRig no_person;
  no_person.config.person.clear();
  EXPECT_FALSE(no_person.Core().Load());
  ASSERT_TRUE(no_person.Core().Poll(1.0, no_person.camera, report));
  EXPECT_NE(report.detail.find("kind_person"), std::string::npos);

  CoreRig person_height;
  person_height.config.known_height = {{"person", 1.7}};
  EXPECT_FALSE(person_height.Core().Load());

  CoreRig unknown_height;
  unknown_height.config.known_height = {{"brick", 0.1}};
  EXPECT_FALSE(unknown_height.Core().Load());
}

TEST(ObstacleCore, KindsFromThePlatformLists) {
  CoreRig rig;
  EXPECT_EQ(rig.Core().KindOf("person"), ObstacleKind::kPerson);
  EXPECT_EQ(rig.Core().KindOf("cat"), ObstacleKind::kAnimal);
  EXPECT_EQ(rig.Core().KindOf("manhole"), ObstacleKind::kIgnore);
  EXPECT_EQ(rig.Core().KindOf("stone"), ObstacleKind::kObject);
  EXPECT_EQ(rig.Core().KindOf("anything else"), ObstacleKind::kObject);
}

TEST(ObstacleCore, FindsAThingOnTheGroundWithItsSpread) {
  Scene scene;
  const Block block{1.5, 1.8, -0.15, 0.15, 0.4};
  scene.blocks = {block};
  CoreRig rig(HitFor(BoxOf(scene, block), 0));
  rig.scene = scene;
  ASSERT_TRUE(rig.Core().Load());
  ObstacleReport report;
  ASSERT_TRUE(rig.Analyse(report));
  EXPECT_DOUBLE_EQ(report.taken, 50.0);
  ASSERT_EQ(report.obstacles.size(), 1U);
  const FoundObstacle& o = report.obstacles[0];
  EXPECT_EQ(o.kind, ObstacleKind::kPerson);
  EXPECT_EQ(o.class_name, "person");
  EXPECT_NEAR(o.score, 0.9, 0.01);
  EXPECT_EQ(o.from, PositionFrom::kBoth);
  // The face at 1.5 m, its footprint 0.15 behind; the box of the model is a few pixels off.
  EXPECT_GT(o.spot.x, 1.35);
  EXPECT_LT(o.spot.x, 1.8);
  EXPECT_NEAR(o.spot.y, 0.0, 0.08);
  EXPECT_GT(o.spot.cov_xx, 0.0);
  EXPECT_GT(o.spot.cov_yy, 0.0);
  EXPECT_LT(std::sqrt(o.spot.cov_xx), 0.3);
  EXPECT_FALSE(o.cut_bottom);
  // The view: ahead, about 86 degrees, from the lower edge of the image on.
  EXPECT_GT(report.view.angle_left, 0.6);
  EXPECT_LT(report.view.angle_right, -0.6);
  EXPECT_GT(report.view.range_min, 0.1);
  EXPECT_DOUBLE_EQ(report.view.range_max, 3.0);
  EXPECT_EQ(rig.Core().Status().analyses, 1U);
  EXPECT_NE(rig.Core().Status().last.find("person 0.9"), std::string::npos);
}

TEST(ObstacleCore, IgnoredClassesAreLeftOut) {
  Scene scene;
  const Block block{1.5, 1.8, -0.15, 0.15, 0.1};
  scene.blocks = {block};
  CoreRig rig(HitFor(BoxOf(scene, block), 3));
  rig.scene = scene;
  ASSERT_TRUE(rig.Core().Load());
  ObstacleReport report;
  ASSERT_TRUE(rig.Analyse(report));
  EXPECT_TRUE(report.obstacles.empty());
  EXPECT_EQ(rig.Core().Status().last, "none");
}

// The size probe moves an object far out with a spread that still holds the depth of the stereo; the point under the
// box stays in charge, so the thing ends up no farther than where it stands.
TEST(ObstacleCore, TheSizeProbeLeavesTheGroundInCharge) {
  Scene scene;
  const Block block{1.5, 1.8, -0.15, 0.15, 0.4};
  scene.blocks = {block};
  CoreRig sized(HitFor(BoxOf(scene, block), 4));
  sized.scene = scene;
  // A stone of 1.7 m whose box is only 0.4 m high would be far away by its size.
  sized.config.known_height = {{"stone", 1.7}};
  ASSERT_TRUE(sized.Core().Load());
  ObstacleReport with_size;
  ASSERT_TRUE(sized.Analyse(with_size));
  ASSERT_EQ(with_size.obstacles.size(), 1U);
  EXPECT_LT(with_size.obstacles[0].spot.x, block.near + 0.5 * (block.left - block.right) + 0.08);
  EXPECT_GT(with_size.obstacles[0].spot.x, block.near - 0.1);
}

// A result of a frame the driver stamped long ago gives no sight; a stamp ahead counts from the submission.
TEST(ObstacleCore, ResultsOfOldFramesAreDropped) {
  Scene scene;
  const Block block{1.5, 1.8, -0.15, 0.15, 0.4};
  scene.blocks = {block};
  CoreRig rig(HitFor(BoxOf(scene, block), 0));
  rig.scene = scene;
  ASSERT_TRUE(rig.Core().Load());
  ObstacleReport report;
  ASSERT_TRUE(rig.Analyse(report));
  rig.clock += 0.3;
  EXPECT_FALSE(rig.Analyse(report, false, 0.5, -2.0));
  EXPECT_EQ(report.reason, SightReason::kNoSignal);
  rig.clock += 0.3;
  EXPECT_TRUE(rig.Analyse(report, false, 3.0, 5.0));
  EXPECT_EQ(report.obstacles.size(), 1U);
}

TEST(ObstacleCore, PhotosOnlyWhenWantedAndAtMostOnceASecond) {
  Scene scene;
  const Block block{1.5, 1.8, -0.15, 0.15, 0.4};
  scene.blocks = {block};
  CoreRig rig(HitFor(BoxOf(scene, block), 1));
  rig.scene = scene;
  ASSERT_TRUE(rig.Core().Load());
  ObstacleReport report;
  ASSERT_TRUE(rig.Analyse(report));
  EXPECT_TRUE(report.jpeg.empty());
  rig.Core().SetPhotosWanted(true);
  rig.clock += 0.3;
  ASSERT_TRUE(rig.Analyse(report));
  ASSERT_FALSE(report.jpeg.empty());
  EXPECT_LE(report.jpeg.size(), 100U * 1024);
  EXPECT_EQ(report.width, 640U);
  EXPECT_EQ(report.height, 480U);
  rig.clock += 0.3;
  ASSERT_TRUE(rig.Analyse(report));
  EXPECT_TRUE(report.jpeg.empty());
  rig.clock += 1.0;
  ASSERT_TRUE(rig.Analyse(report));
  EXPECT_FALSE(report.jpeg.empty());
  // Nothing found, no photo.
  CoreRig empty;
  ASSERT_TRUE(empty.Core().Load());
  empty.Core().SetPhotosWanted(true);
  ASSERT_TRUE(empty.Analyse(report));
  EXPECT_TRUE(report.obstacles.empty());
  EXPECT_TRUE(report.jpeg.empty());
}

TEST(ObstacleCore, AtTheFixedRate) {
  CoreRig rig;
  ASSERT_TRUE(rig.Core().Load());
  ObstacleReport report;
  ASSERT_TRUE(rig.Analyse(report));
  const double first = rig.clock;
  // Frames come at 24 per second: the next analysis starts 0.2 s after the last one.
  int analysed = 0;
  for (int i = 0; i < 24; i++) {
    rig.clock = first + i / 24.0;
    if (rig.Analyse(report, false, 0.15)) analysed++;
  }
  EXPECT_GE(analysed, 4);
  EXPECT_LE(analysed, 6);
}

TEST(ObstacleCore, WithoutSightAReportPerPeriodWithTheReason) {
  CoreRig rig;
  ASSERT_TRUE(rig.Core().Load());
  ObstacleReport report;
  const std::vector<std::pair<StereoState, SightReason>> cases = {
      {StereoState::kPausedHot, SightReason::kTooHot},
      {StereoState::kLocked, SightReason::kLockedAfterCrash},
      {StereoState::kSelftest, SightReason::kSelfTest},
      {StereoState::kWaitingIdle, SightReason::kSelfTest},
      {StereoState::kWaitingSetup, SightReason::kNotSetUp},
      {StereoState::kPausedBlade, SightReason::kNoSignal},
      {StereoState::kStalled, SightReason::kNoSignal},
  };
  rig.camera.capturing = false;
  double now = 10.0;
  for (const auto& [state, reason] : cases) {
    rig.camera.state = state;
    rig.camera.detail = "why";
    now += 0.25;
    ASSERT_TRUE(rig.Core().Poll(now, rig.camera, report)) << StateName(state);
    EXPECT_EQ(report.reason, reason) << StateName(state);
    EXPECT_EQ(report.detail, "why");
    EXPECT_DOUBLE_EQ(report.taken, now);
    // Not again before the period is over.
    EXPECT_FALSE(rig.Core().Poll(now + 0.1, rig.camera, report));
  }
  // Running, but no analysis yet.
  rig.camera.capturing = true;
  ASSERT_TRUE(rig.Core().Poll(now + 0.25, rig.camera, report));
  EXPECT_EQ(report.reason, SightReason::kNoSignal);
}

TEST(ObstacleCore, ADarkPictureHasNoSight) {
  CoreRig rig;
  rig.frame = Flat(20, 20);
  ASSERT_TRUE(rig.Core().Load());
  ObstacleReport report;
  // Each frame a little different in the check field, so it does not stand still.
  for (int i = 0; i < 6; i++) {
    rig.frame[360 * 2560 + 2 * 320] = static_cast<uint8_t>(20 + (i & 1));
    rig.clock += 0.2;
    EXPECT_TRUE(rig.Analyse(report, false, 0.5)) << i;
  }
  for (int i = 6; i < 15; i++) {
    rig.frame[360 * 2560 + 2 * 320] = static_cast<uint8_t>(20 + (i & 1));
    rig.clock += 0.2;
    rig.Analyse(report, false, 0.1);
  }
  EXPECT_EQ(report.reason, SightReason::kTooDark);
}

TEST(ObstacleCore, AResultOfAnEarlierStartIsDropped) {
  CoreRig rig("run_ms 100\n");
  ASSERT_TRUE(rig.Core().Load());
  rig.frame = rig.scene.Render();
  CapturedFrame f{0, rig.frame.data(), rig.frame.size(), 2560, 1, 50000000, -1};
  rig.Core().OnFrame(f, rig.clock, {});
  rig.Core().OnStop();
  ObstacleReport report;
  const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
  while (std::chrono::steady_clock::now() < until) {
    if (rig.Core().Poll(rig.clock, rig.camera, report)) {
      EXPECT_NE(report.reason, SightReason::kNone);
    }
    std::this_thread::sleep_for(5ms);
  }
}

TEST(ObstacleCore, AHangingModelEndsTheProcess) {
  CoreRig rig("run_ms 1200\n");
  rig.config.hang_s = 0.5;
  ASSERT_TRUE(rig.Core().Load());
  rig.frame = rig.scene.Render();
  CapturedFrame f{0, rig.frame.data(), rig.frame.size(), 2560, 1, 50000000, -1};
  rig.Core().OnFrame(f, rig.clock, {});
  ObstacleReport report;
  rig.Core().Poll(rig.clock + 0.1, rig.camera, report);
  EXPECT_FALSE(rig.Core().Poll(rig.clock + 0.6, rig.camera, report));
  EXPECT_EQ(rig.Core().Status().exit_code, kExitSlowNpu);
}

}  // namespace
}  // namespace tron_vision
