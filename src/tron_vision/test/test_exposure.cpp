#include <gtest/gtest.h>

#include "tron_vision/exposure.h"

namespace tron_vision {
namespace {

TEST(Exposure, InBandNoChange) {
  const ExposureSetting now{1120, 2048};
  const ExposureSetting next = NextExposure(now, 105.0F, ExposureLimits{});
  EXPECT_EQ(next.exposure, 1120);
  EXPECT_EQ(next.gain, 2048);
}

TEST(Exposure, DarkRaisesTheGainAtMostTwofold) {
  // M2b: gain 512 gave Y 30 under lamp light.
  const ExposureSetting next = NextExposure({1120, 512}, 30.0F, ExposureLimits{});
  EXPECT_EQ(next.exposure, 1120);
  EXPECT_EQ(next.gain, 1024);
  // Close below the band: (100-16)/(80-16) = 1.3125.
  EXPECT_EQ(NextExposure({1120, 1000}, 80.0F, ExposureLimits{}).gain, 1313);
}

TEST(Exposure, GainStopsAtItsMaximum) {
  const ExposureSetting next = NextExposure({1120, 6000}, 20.0F, ExposureLimits{});
  EXPECT_EQ(next.exposure, 1120);
  EXPECT_EQ(next.gain, 8192);
}

TEST(Exposure, BrightLowersTheGainFirst) {
  const ExposureSetting next = NextExposure({1120, 1024}, 200.0F, ExposureLimits{});
  EXPECT_EQ(next.exposure, 1120);
  EXPECT_EQ(next.gain, 512);
}

TEST(Exposure, BacklightLowersTheExposureOnlyAtMinimumGain) {
  // Gain 100 halves to 50, below the minimum: gain 64 and the rest from the exposure.
  ExposureSetting next = NextExposure({1120, 100}, 220.0F, ExposureLimits{});
  EXPECT_EQ(next.gain, 64);
  EXPECT_EQ(next.exposure, 875);
  next = NextExposure({875, 64}, 220.0F, ExposureLimits{});
  EXPECT_EQ(next.gain, 64);
  EXPECT_EQ(next.exposure, 438);
  next = NextExposure({6, 64}, 250.0F, ExposureLimits{});
  EXPECT_EQ(next.exposure, 4);
}

TEST(Exposure, DarkAfterBacklightRaisesTheExposureBeforeTheGain) {
  ExposureSetting next = NextExposure({400, 64}, 40.0F, ExposureLimits{});
  EXPECT_EQ(next.gain, 64);
  EXPECT_EQ(next.exposure, 800);
  next = NextExposure({800, 64}, 40.0F, ExposureLimits{});
  EXPECT_EQ(next.exposure, 1120);
  EXPECT_EQ(next.gain, 91);
}

TEST(Exposure, OutOfRangeInputIsClampedFirst) {
  const ExposureSetting next = NextExposure({5000, 10}, 30.0F, ExposureLimits{});
  EXPECT_EQ(next.exposure, 1120);
  EXPECT_EQ(next.gain, 128);
}

}  // namespace
}  // namespace tron_vision
