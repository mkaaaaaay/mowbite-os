#include <gtest/gtest.h>

#include "tron_vision/lens_model.h"

namespace tron_vision {
namespace {

// Wide lens with strong barrel distortion, 1920 x 1080.
const LensModel kLens = {1540.499, 1539.740, 986.294, 520.626, -0.373942, 0.165317, -0.000099, -0.000519, 0.0};

TEST(LensModel, ToPixelFollowsThePlumbBobFormula) {
  // By hand: r2 = 0.05, radial factor 0.9817161925, then the tangential terms.
  Point2 px = ToPixel(kLens, {0.2, -0.1});
  EXPECT_NEAR(px.x, 1288.662725, 1e-5);
  EXPECT_NEAR(px.y, 369.488526, 1e-5);
  px = ToPixel(kLens, {-0.5, 0.3});
  EXPECT_NEAR(px.x, 298.628429, 1e-5);
  EXPECT_NEAR(px.y, 932.807207, 1e-5);
}

TEST(LensModel, WithoutDistortionItIsAPinhole) {
  const LensModel pinhole = {500.0, 600.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const Point2 px = ToPixel(pinhole, {0.1, -0.2});
  EXPECT_DOUBLE_EQ(px.x, 370.0);
  EXPECT_DOUBLE_EQ(px.y, 120.0);
  Point2 ray;
  ASSERT_TRUE(ToRay(pinhole, px, ray));
  EXPECT_NEAR(ray.x, 0.1, 1e-12);
  EXPECT_NEAR(ray.y, -0.2, 1e-12);
}

TEST(LensModel, ToRayUndoesToPixelOverTheWholeImage) {
  for (int y = 0; y <= 1080; y += 60) {
    for (int x = 0; x <= 1920; x += 60) {
      const Point2 px{static_cast<double>(x), static_cast<double>(y)};
      Point2 ray;
      ASSERT_TRUE(ToRay(kLens, px, ray)) << x << "," << y;
      const Point2 back = ToPixel(kLens, ray);
      EXPECT_NEAR(back.x, px.x, 1e-5) << x << "," << y;
      EXPECT_NEAR(back.y, px.y, 1e-5) << x << "," << y;
    }
  }
}

TEST(LensModel, ToRayRefusesALensWithoutFocalLength) {
  Point2 ray;
  EXPECT_FALSE(ToRay(LensModel{}, {10.0, 10.0}, ray));
}

TEST(LensModel, ToRayRefusesWhereTheModelFoldsBack) {
  // With this much barrel distortion the image of the model ends inside the frame.
  LensModel lens = kLens;
  lens.k1 = -3.0;
  lens.k2 = 0.0;
  Point2 ray;
  EXPECT_FALSE(ToRay(lens, {0.0, 0.0}, ray));
}

}  // namespace
}  // namespace tron_vision
