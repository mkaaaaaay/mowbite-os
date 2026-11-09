#include <gtest/gtest.h>
#include <turbojpeg.h>

#include <random>

#include "test_util.h"
#include "tron_vision/preview.h"

namespace tron_vision {
namespace {

Nv12View View(const test::Nv12Frame& f) {
  Nv12View v;
  v.y = f.data.data();
  v.uv = f.data.data() + static_cast<size_t>(f.width) * f.height;
  v.width = f.width;
  v.height = f.height;
  v.stride = f.width;
  return v;
}

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

TEST(Preview, JpegOfTheScaledFrame) {
  test::Nv12Frame frame(1920, 1080, 120, 110, 150);
  Preview preview(PreviewConfig{});
  std::string error;
  ASSERT_TRUE(preview.Make(View(frame), ColorGains{}, {{0, 0.9F, 300, 300, 900, 900}}, 10.0, error)) << error;
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
    if (preview.Due(t) && preview.Make(View(frame), ColorGains{}, {}, t, error)) made++;
  }
  EXPECT_GE(made, 9);
  EXPECT_LE(made, 10);
  config.max_rate = 50.0;
  Preview capped(config);
  made = 0;
  for (double t = 0.0; t < 2.0; t += 1.0 / 30.0) {
    if (capped.Due(t) && capped.Make(View(frame), ColorGains{}, {}, t, error)) made++;
  }
  EXPECT_LE(made, 10);
  // After a pause one picture, then the rate again.
  made = 0;
  for (double t = 10.0; t < 10.9; t += 1.0 / 30.0) {
    if (capped.Due(t) && capped.Make(View(frame), ColorGains{}, {}, t, error)) made++;
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
  ASSERT_TRUE(preview.Make(View(frame), ColorGains{}, {}, 0.0, error)) << error;
  EXPECT_LT(preview.Quality(), config.quality);
  if (preview.Count() == 1) {
    EXPECT_LE(preview.Jpeg().size(), config.max_bytes);
  }

  // Even the lowest quality is too big: no picture, the last one stays.
  config.max_bytes = 1000;
  Preview tiny(config);
  ASSERT_TRUE(tiny.Make(View(frame), ColorGains{}, {}, 0.0, error)) << error;
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
  ASSERT_TRUE(limited.Make(View(noisy), ColorGains{}, {}, 0.0, error));
  const int low = limited.Quality();
  for (int i = 1; i <= 20; i++) ASSERT_TRUE(limited.Make(View(flat), ColorGains{}, {}, i, error));
  EXPECT_GT(limited.Quality(), low);
  EXPECT_EQ(limited.Quality(), config.quality);
}

}  // namespace
}  // namespace tron_vision
