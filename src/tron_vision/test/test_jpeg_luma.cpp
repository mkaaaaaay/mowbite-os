#include <gtest/gtest.h>

#include <cstdlib>

#include "dock_jpeg.h"
#include "tron_vision/jpeg_luma.h"

namespace tron_vision {
namespace {

test::SceneImage Gradient(uint32_t width, uint32_t height) {
  test::SceneImage image;
  image.width = width;
  image.height = height;
  for (uint32_t y = 0; y < height; y++) {
    for (uint32_t x = 0; x < width; x++) image.px.push_back(static_cast<uint8_t>(16 + (x + 2 * y) % 220));
  }
  return image;
}

TEST(JpegLuma, DecodesTheLumaAtFullSize) {
  const test::SceneImage image = Gradient(320, 240);
  const std::vector<uint8_t> jpeg = test::EncodeJpeg(image, 95);
  ASSERT_FALSE(jpeg.empty());
  JpegLuma decoder;
  GrayView luma;
  std::string error;
  ASSERT_TRUE(decoder.Decode(jpeg.data(), jpeg.size(), 320, 240, luma, error)) << error;
  EXPECT_EQ(luma.width, 320U);
  EXPECT_EQ(luma.height, 240U);
  EXPECT_EQ(luma.stride, 320U);
  int worst = 0;
  for (size_t i = 0; i < image.px.size(); i++) worst = std::max(worst, std::abs(luma.data[i] - image.px[i]));
  EXPECT_LE(worst, 12);
}

TEST(JpegLuma, RefusesAnotherSizeAndBrokenData) {
  const std::vector<uint8_t> jpeg = test::EncodeJpeg(Gradient(320, 240));
  JpegLuma decoder;
  GrayView luma;
  std::string error;
  EXPECT_FALSE(decoder.Decode(jpeg.data(), jpeg.size(), 1920, 1080, luma, error));
  EXPECT_EQ(error, "JPEG of 320x240");
  const std::vector<uint8_t> noise(4000, 0x5a);
  EXPECT_FALSE(decoder.Decode(noise.data(), noise.size(), 320, 240, luma, error));
  EXPECT_NE(error.find("JPEG header"), std::string::npos) << error;
  // Cut short, as when the USB transfer of a frame breaks off.
  EXPECT_FALSE(decoder.Decode(jpeg.data(), jpeg.size() / 2, 320, 240, luma, error));
  EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace tron_vision
