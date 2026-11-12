#include <gtest/gtest.h>

#include "test_util.h"
#include "tron_vision/frame_convert.h"

namespace tron_vision {
namespace {

TEST(PlaceFrame, CropTakesTheCenteredThreeByFourPart) {
  const Placement p = PlaceFrame(1920, 1080, Fit::kCrop, 480, 640);
  EXPECT_FLOAT_EQ(p.src_x, 555.0F);
  EXPECT_FLOAT_EQ(p.src_y, 0.0F);
  EXPECT_FLOAT_EQ(p.src_w, 810.0F);
  EXPECT_FLOAT_EQ(p.src_h, 1080.0F);
  EXPECT_EQ(p.dst_x, 0U);
  EXPECT_EQ(p.dst_w, 480U);
  EXPECT_EQ(p.dst_h, 640U);
}

TEST(PlaceFrame, PadKeepsTheWholeFrame) {
  const Placement p = PlaceFrame(1920, 1080, Fit::kPad, 480, 640);
  EXPECT_FLOAT_EQ(p.src_w, 1920.0F);
  EXPECT_EQ(p.dst_w, 480U);
  EXPECT_EQ(p.dst_h, 270U);
  EXPECT_EQ(p.dst_y, 185U);
}

TEST(PlaceFrame, ToSourceUndoesThePlacement) {
  const Placement crop = PlaceFrame(1920, 1080, Fit::kCrop, 480, 640);
  Detection d{3, 0.5F, 0.0F, 320.0F, 240.0F, 640.0F};
  Detection s = ToSource(d, crop);
  EXPECT_NEAR(s.x1, 555.0F, 1e-3F);
  EXPECT_NEAR(s.y1, 540.0F, 1e-3F);
  EXPECT_NEAR(s.x2, 960.0F, 1e-3F);
  EXPECT_NEAR(s.y2, 1080.0F, 1e-3F);
  EXPECT_EQ(s.class_id, 3U);

  const Placement pad = PlaceFrame(1920, 1080, Fit::kPad, 480, 640);
  d = Detection{0, 0.5F, 0.0F, 0.0F, 480.0F, 640.0F};  // reaches into the bars
  s = ToSource(d, pad);
  EXPECT_NEAR(s.y1, 0.0F, 1e-3F);
  EXPECT_NEAR(s.y2, 1080.0F, 1e-3F);
  EXPECT_NEAR(s.x2, 1920.0F, 1e-3F);
}

TEST(ConvertFrame, GrayStaysGrayAndBarsArePadGray) {
  const test::Nv12Frame f(64, 36, 126, 128, 128);
  const Placement p = PlaceFrame(f.width, f.height, Fit::kPad, 48, 64);
  std::vector<uint8_t> rgb;
  ConvertFrame(test::View(f), p, ColorGains{}, false, 48, 64, rgb);
  ASSERT_EQ(rgb.size(), 48U * 64U * 3U);
  EXPECT_EQ(rgb[0], 114);  // bar above
  const size_t mid = (32 * 48 + 24) * 3;
  EXPECT_NEAR(rgb[mid], 128, 1);
  EXPECT_NEAR(rgb[mid + 1], 128, 1);
  EXPECT_NEAR(rgb[mid + 2], 128, 1);
}

TEST(ConvertFrame, ColorsAndSwap) {
  // Strong red in BT.601 limited range: Y 81, U 90, V 240.
  const test::Nv12Frame f(32, 32, 81, 90, 240);
  const Placement p = PlaceFrame(f.width, f.height, Fit::kCrop, 24, 32);
  std::vector<uint8_t> rgb;
  ConvertFrame(test::View(f), p, ColorGains{}, false, 24, 32, rgb);
  EXPECT_GT(rgb[0], 240);
  EXPECT_LT(rgb[1], 20);
  EXPECT_LT(rgb[2], 20);
  ConvertFrame(test::View(f), p, ColorGains{}, true, 24, 32, rgb);
  EXPECT_LT(rgb[0], 20);
  EXPECT_GT(rgb[2], 240);
}

TEST(ConvertFrame, BilinearLuma) {
  // Left half black, right half white: the output column at the edge lies between.
  test::Nv12Frame f(4, 2, 16, 128, 128);
  f.data[2] = f.data[3] = f.data[6] = f.data[7] = 235;
  Placement p;
  p.src_w = 4.0F;
  p.src_h = 2.0F;
  p.dst_w = 2;
  p.dst_h = 1;
  std::vector<uint8_t> rgb;
  ConvertFrame(test::View(f), p, ColorGains{}, false, 2, 1, rgb);
  // Output column 0 samples source x 0.5, column 1 x 2.5.
  EXPECT_LT(rgb[0], 5);
  EXPECT_GT(rgb[3], 250);
}

TEST(GrayWorld, NeutralForGrayWarmForWarm) {
  FrameStats gray{120.0F, 128.0F, 128.0F};
  const ColorGains g = GrayWorld(gray);
  EXPECT_NEAR(g.r, 256, 2);
  EXPECT_NEAR(g.b, 256, 2);
  // M2b (09.11.): lamp light gave U 113, V 132, a warm cast.
  FrameStats warm{77.0F, 113.0F, 132.0F};
  const ColorGains w = GrayWorld(warm);
  EXPECT_LT(w.r, 256);
  EXPECT_GT(w.b, 256);
  EXPECT_EQ(w.g, 256);
  FrameStats dark{16.0F, 128.0F, 128.0F};
  EXPECT_EQ(GrayWorld(dark).r, 256);
}

TEST(GrayWorld, AppliedGainsNeutralizeTheCast) {
  const test::Nv12Frame f(32, 32, 77, 113, 132);
  const Placement p = PlaceFrame(f.width, f.height, Fit::kCrop, 24, 32);
  const ColorGains gains = GrayWorld(MeasureFrame(test::View(f), p));
  std::vector<uint8_t> rgb;
  ConvertFrame(test::View(f), p, gains, false, 24, 32, rgb);
  EXPECT_NEAR(rgb[0], rgb[1], 2);
  EXPECT_NEAR(rgb[2], rgb[1], 2);
}

TEST(MeasureFrame, MeansOverTheRegion) {
  test::Nv12Frame f(64, 32, 50, 100, 150);
  const FrameStats s = MeasureFrame(test::View(f), PlaceFrame(64, 32, Fit::kCrop, 3, 4));
  EXPECT_FLOAT_EQ(s.y, 50.0F);
  EXPECT_FLOAT_EQ(s.u, 100.0F);
  EXPECT_FLOAT_EQ(s.v, 150.0F);
}

TEST(ShiftChroma, MovesUAndVWithinTheirRange) {
  test::Nv12Frame f(8, 4, 50, 100, 250);
  uint8_t* uv = f.data.data() + 8 * 4;
  ShiftChroma(uv, 8, 4, 8, -20, 10);
  const FrameStats s = MeasureFrame(test::View(f), PlaceFrame(8, 4, Fit::kPad, 8, 4));
  EXPECT_FLOAT_EQ(s.y, 50.0F);
  EXPECT_FLOAT_EQ(s.u, 120.0F);
  EXPECT_FLOAT_EQ(s.v, 240.0F);
  ShiftChroma(uv, 8, 4, 8, 200, -100);
  EXPECT_EQ(uv[0], 0);
  EXPECT_EQ(uv[1], 255);
  EXPECT_EQ(uv[14], 0);
  EXPECT_EQ(uv[15], 255);
}

TEST(ShiftChroma, LeavesThePaddingOfALine) {
  std::vector<uint8_t> uv(2 * 12, 128);
  ShiftChroma(uv.data(), 8, 4, 12, 28, 0);
  EXPECT_EQ(uv[0], 100);
  EXPECT_EQ(uv[1], 128);
  EXPECT_EQ(uv[8], 128);
  EXPECT_EQ(uv[12], 100);
  EXPECT_EQ(uv[20], 128);
}

}  // namespace
}  // namespace tron_vision
