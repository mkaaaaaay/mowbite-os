#include <gtest/gtest.h>
#include <turbojpeg.h>

#include <algorithm>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "dock_jpeg.h"
#include "test_util.h"
#include "tron_vision/preview.h"

namespace tron_vision {
namespace {

// Width and height read back from a JPEG; 0 if it does not decode.
std::pair<int, int> JpegSize(const std::vector<uint8_t>& jpeg) {
  tjhandle h = tjInitDecompress();
  int w = 0;
  int ht = 0;
  int sub = 0;
  int space = 0;
  if (tjDecompressHeader3(h, jpeg.data(), jpeg.size(), &w, &ht, &sub, &space) != 0) w = ht = 0;
  tjDestroy(h);
  return {w, ht};
}

const uint8_t* At(const std::vector<uint8_t>& rgb, uint32_t width, uint32_t x, uint32_t y) {
  return rgb.data() + (static_cast<size_t>(y) * width + x) * 3;
}

TEST(Preview, BoxOutlinesInTheClassColor) {
  std::vector<uint8_t> rgb(100 * 50 * 3, 0);
  DrawBoxes(rgb, 100, 50, {{0, 0.9F, 20, 10, 60, 40}, {1, 0.5F, -50, -50, 500, 500}}, 0.5F);
  // First box: 10..30 x 5..20 in the image, red outline, inside untouched.
  EXPECT_EQ(At(rgb, 100, 10, 5)[0], 255);
  EXPECT_EQ(At(rgb, 100, 29, 19)[0], 255);
  EXPECT_EQ(At(rgb, 100, 20, 12)[0], 0);
  // Second box is clipped to the image and drawn green along its border.
  EXPECT_EQ(At(rgb, 100, 99, 49)[1], 230);
  EXPECT_EQ(At(rgb, 100, 50, 25)[1], 0);
}

TEST(Preview, BoxOutlinesInNv12OnEvenPixels) {
  test::Nv12Frame frame(64, 32, 16, 128, 128);
  Nv12Buffer b;
  b.y = frame.data.data();
  b.uv = b.y + 64 * 32;
  b.width = 64;
  b.height = 32;
  b.stride = 64;
  DrawBoxes(b, {{0, 0.9F, 9.4F, 5.0F, 30.6F, 20.0F}, {1, 0.5F, -10, -10, 100, 100}}, 4);
  const auto y = [&](uint32_t px, uint32_t py) { return frame.data[py * 64 + px]; };
  const auto u = [&](uint32_t px, uint32_t py) { return frame.data[64 * 32 + (py / 2) * 64 + (px & ~1U)]; };
  // First box: corners 8,4 and 32,20 after rounding to even pixels, red outline 4 wide, inside untouched.
  EXPECT_EQ(y(8, 4), 120);
  EXPECT_EQ(u(8, 4), 100);
  EXPECT_EQ(y(31, 19), 120);
  EXPECT_EQ(y(11, 7), 120);
  EXPECT_EQ(y(12, 8), 16);
  EXPECT_EQ(u(12, 8), 128);
  EXPECT_EQ(y(7, 4), 16);
  // Second box is clipped to the frame and drawn green along its border.
  EXPECT_EQ(y(63, 31), y(0, 0));
  EXPECT_NE(y(0, 0), 16);
  EXPECT_EQ(y(32, 16), 16);
}

TEST(Preview, JpegOfTheScaledFrame) {
  test::Nv12Frame frame(1920, 1080, 120, 110, 150);
  Preview preview(PreviewConfig{});
  std::string error;
  ASSERT_TRUE(preview.Make(test::View(frame), ColorGains{}, {{0, 0.9F, 300, 300, 900, 900}}, 10.0, error)) << error;
  EXPECT_EQ(preview.Count(), 1U);
  EXPECT_EQ(preview.Width(), 640U);
  EXPECT_EQ(preview.Height(), 360U);
  EXPECT_EQ(JpegSize(preview.Jpeg()), std::make_pair(640, 360));
  EXPECT_LE(preview.Jpeg().size(), 100U * 1024);
}

TEST(Preview, AtMostTheConfiguredRate) {
  test::Nv12Frame frame(640, 480, 120, 128, 128);
  PreviewConfig config;
  config.max_rate = 5.0;
  Preview preview(config);
  std::string error;
  int made = 0;
  for (double t = 0.0; t < 2.0; t += 1.0 / 30.0) {
    if (preview.Due(t) && preview.Make(test::View(frame), ColorGains{}, {}, t, error)) made++;
  }
  EXPECT_GE(made, 9);
  EXPECT_LE(made, 10);
  config.max_rate = 50.0;
  Preview capped(config);
  made = 0;
  for (double t = 0.0; t < 2.0; t += 1.0 / 30.0) {
    if (capped.Due(t) && capped.Make(test::View(frame), ColorGains{}, {}, t, error)) made++;
  }
  EXPECT_LE(made, 10);
  // After a pause one picture, then the rate again.
  made = 0;
  for (double t = 10.0; t < 10.9; t += 1.0 / 30.0) {
    if (capped.Due(t) && capped.Make(test::View(frame), ColorGains{}, {}, t, error)) made++;
  }
  EXPECT_LE(made, 5);
}

TEST(Preview, QualityStepsDownToKeepTheLimit) {
  test::Nv12Frame frame(640, 480, 0, 128, 128);
  std::mt19937 noise(7);
  for (uint8_t& b : frame.data) b = static_cast<uint8_t>(noise());
  PreviewConfig config;
  config.max_bytes = 60 * 1024;
  Preview preview(config);
  std::string error;
  ASSERT_TRUE(preview.Make(test::View(frame), ColorGains{}, {}, 0.0, error)) << error;
  EXPECT_LT(preview.Quality(), config.quality);
  if (preview.Count() == 1) {
    EXPECT_LE(preview.Jpeg().size(), config.max_bytes);
  }

  // Even the lowest quality is too big: no picture, the last one stays.
  config.max_bytes = 1000;
  Preview tiny(config);
  ASSERT_TRUE(tiny.Make(test::View(frame), ColorGains{}, {}, 0.0, error)) << error;
  EXPECT_EQ(tiny.Count(), 0U);
  EXPECT_EQ(tiny.Quality(), config.min_quality);
}

TEST(Preview, QualityComesBackWithRoom) {
  test::Nv12Frame flat(640, 480, 90, 128, 128);
  PreviewConfig config;
  std::string error;
  test::Nv12Frame noisy(640, 480, 0, 128, 128);
  std::mt19937 noise(3);
  for (uint8_t& b : noisy.data) b = static_cast<uint8_t>(noise());
  config.max_bytes = 60 * 1024;
  Preview limited(config);
  ASSERT_TRUE(limited.Make(test::View(noisy), ColorGains{}, {}, 0.0, error));
  const int low = limited.Quality();
  for (int i = 1; i <= 20; i++) ASSERT_TRUE(limited.Make(test::View(flat), ColorGains{}, {}, i, error));
  EXPECT_GT(limited.Quality(), low);
  EXPECT_EQ(limited.Quality(), config.quality);
}

// A gray image with a bright left half, as the rear camera would send it.
test::SceneImage HalfBright(uint32_t width, uint32_t height) {
  test::SceneImage image;
  image.width = width;
  image.height = height;
  image.px.assign(static_cast<size_t>(width) * height, 40);
  for (uint32_t y = 0; y < height; y++) {
    std::fill_n(image.px.begin() + static_cast<long>(y) * width, width / 2, 200);
  }
  return image;
}

TEST(JpegDecoder, ScalesInTheDecoder) {
  EXPECT_EQ(ScaledSize(1920, {3, 8}), 720U);
  EXPECT_EQ(ScaledSize(1080, {3, 8}), 405U);
  EXPECT_EQ(ScaledSize(1081, {1, 2}), 541U);
  const std::vector<uint8_t> jpeg = test::EncodeJpeg(HalfBright(1920, 1080));
  ASSERT_FALSE(jpeg.empty());
  JpegDecoder decoder;
  std::vector<uint8_t> rgb;
  std::string error;
  ASSERT_TRUE(decoder.Decode(jpeg.data(), jpeg.size(), 1920, 1080, {3, 8}, rgb, error)) << error;
  ASSERT_EQ(rgb.size(), 720U * 405U * 3U);
  EXPECT_NEAR(At(rgb, 720, 100, 200)[0], 200, 4);
  EXPECT_NEAR(At(rgb, 720, 620, 200)[1], 40, 4);
  // Full size too.
  ASSERT_TRUE(decoder.Decode(jpeg.data(), jpeg.size(), 1920, 1080, {1, 1}, rgb, error)) << error;
  EXPECT_EQ(rgb.size(), 1920U * 1080U * 3U);
}

TEST(JpegDecoder, RefusesOtherSizesBrokenImagesAndOtherScales) {
  const std::vector<uint8_t> jpeg = test::EncodeJpeg(HalfBright(640, 480));
  JpegDecoder decoder;
  std::vector<uint8_t> rgb;
  std::string error;
  EXPECT_FALSE(decoder.Decode(jpeg.data(), jpeg.size(), 1920, 1080, {3, 8}, rgb, error));
  EXPECT_EQ(error, "JPEG of 640x480");
  const std::vector<uint8_t> cut(jpeg.begin(), jpeg.begin() + static_cast<long>(jpeg.size() / 3));
  error.clear();
  EXPECT_FALSE(decoder.Decode(cut.data(), cut.size(), 640, 480, {3, 8}, rgb, error));
  EXPECT_EQ(error.rfind("JPEG: ", 0), 0U) << error;
  const std::vector<uint8_t> noise = {1, 2, 3, 4, 5, 6, 7, 8};
  EXPECT_FALSE(decoder.Decode(noise.data(), noise.size(), 640, 480, {3, 8}, rgb, error));
  EXPECT_EQ(error.rfind("JPEG header: ", 0), 0U) << error;
  EXPECT_FALSE(decoder.Decode(jpeg.data(), jpeg.size(), 640, 480, {1, 3}, rgb, error));
  EXPECT_EQ(error, "no decoder scale 1/3");
}

// The limits of Preview without a frame: rate, size and quality on an RGB image.
TEST(BoundedJpeg, RateAndSizeOfAnRgbImage) {
  std::vector<uint8_t> rgb(720 * 405 * 3, 90);
  JpegLimits limits;
  limits.max_rate = 2.0;
  BoundedJpeg pictures(limits);
  std::string error;
  int made = 0;
  for (double t = 0.0; t < 2.0; t += 1.0 / 30.0) {
    if (pictures.Due(t) && pictures.Make(rgb, 720, 405, t, error)) made++;
  }
  EXPECT_EQ(made, 4);
  EXPECT_EQ(pictures.Count(), 4U);
  EXPECT_EQ(pictures.Width(), 720U);
  EXPECT_EQ(pictures.Height(), 405U);
  EXPECT_EQ(JpegSize(pictures.Jpeg()), std::make_pair(720, 405));
  EXPECT_LE(pictures.Jpeg().size(), limits.max_bytes);
  EXPECT_FALSE(pictures.Make(rgb, 720, 404, 10.0, error));
  EXPECT_EQ(error, "RGB buffer does not match the image size");
  EXPECT_EQ(pictures.Count(), 4U);
}

}  // namespace
}  // namespace tron_vision
