#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "test_util.h"
#include "tron_vision/gain_level.h"
#include "tron_vision/text_file.h"

namespace tron_vision {
namespace {

// Feeds one value per second from `from_s` for `seconds`; returns the state after the last.
GainWishState Feed(GainWishState state, double from_s, int seconds, float luma, const GainWishLimits& limits = {}) {
  for (int i = 0; i < seconds; ++i) state = NextGainWish(state, from_s + i, luma, limits);
  return state;
}

GainWishState On(GainLevel level, double at_s) {
  GainWishState state;
  state.level = level;
  state.started = true;
  state.last_at = at_s;
  state.changed_at = at_s;
  return state;
}

TEST(GainLevel, NamesGoBothWays) {
  for (GainLevel level : {GainLevel::kDay, GainLevel::kDusk, GainLevel::kNight}) {
    GainLevel back = GainLevel::kDay;
    ASSERT_TRUE(GainLevelFromName(GainLevelName(level), back));
    EXPECT_EQ(back, level);
  }
  GainLevel level = GainLevel::kDusk;
  EXPECT_FALSE(GainLevelFromName("noon", level));
  EXPECT_FALSE(GainLevelFromName("", level));
  EXPECT_EQ(level, GainLevel::kDusk);
}

TEST(GainLevel, LineHoldsLevelAndBothClocks) {
  EXPECT_EQ(GainWishLine(GainLevel::kNight, 1234.5678, 1794800000.25), "night 1234.568 1794800000.250\n");
  GainLevel level = GainLevel::kDay;
  EXPECT_TRUE(GainLevelFromLine("dusk 12.000 34.000", level));
  EXPECT_EQ(level, GainLevel::kDusk);
  EXPECT_FALSE(GainLevelFromLine("", level));
  EXPECT_FALSE(GainLevelFromLine("dusky 12.000 34.000", level));
}

TEST(GainLevel, MeanLumaTakesTheLeftHalfOnly) {
  const uint32_t width = 64, height = 16, line_bytes = 2 * width + 8;
  std::vector<uint8_t> frame(static_cast<size_t>(line_bytes) * height, 0);
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      frame[static_cast<size_t>(y) * line_bytes + 2 * x] = x < width / 2 ? 80 : 250;
      frame[static_cast<size_t>(y) * line_bytes + 2 * x + 1] = 128;
    }
  }
  EXPECT_FLOAT_EQ(MeanLumaLeft(frame.data(), width, height, line_bytes), 80.0F);
  EXPECT_FLOAT_EQ(MeanLumaLeft(frame.data(), width, height, line_bytes, 1), 80.0F);
  EXPECT_FLOAT_EQ(MeanLumaLeft(frame.data(), 0, height, line_bytes), 0.0F);
  EXPECT_FLOAT_EQ(MeanLumaLeft(nullptr, width, height, line_bytes), 0.0F);
  EXPECT_FLOAT_EQ(MeanLumaLeft(frame.data(), width, height, line_bytes, 0), 0.0F);
}

TEST(GainWish, DarkForLongEnoughGoesUpALevel) {
  GainWishState state = Feed({}, 100.0, 20, 60.0F);
  EXPECT_EQ(state.level, GainLevel::kDay);
  state = NextGainWish(state, 120.0, 60.0F, {});
  EXPECT_EQ(state.level, GainLevel::kDusk);
  // 5 s to settle, then 20 s dark again.
  state = Feed(state, 121.0, 24, 60.0F);
  EXPECT_EQ(state.level, GainLevel::kDusk);
  state = NextGainWish(state, 145.0, 60.0F, {});
  EXPECT_EQ(state.level, GainLevel::kNight);
  state = Feed(state, 146.0, 600, 60.0F);
  EXPECT_EQ(state.level, GainLevel::kNight);
}

TEST(GainWish, AShortDarkSpellChangesNothing) {
  GainWishState state = Feed({}, 100.0, 19, 60.0F);
  state = Feed(state, 119.0, 2, 140.0F);
  state = Feed(state, 121.0, 19, 60.0F);
  EXPECT_EQ(state.level, GainLevel::kDay);
}

TEST(GainWish, DayStaysAtAnyLight) {
  EXPECT_EQ(Feed({}, 100.0, 4000, 145.0F).level, GainLevel::kDay);
  EXPECT_EQ(Feed({}, 100.0, 4000, 250.0F).level, GainLevel::kDay);
}

TEST(GainWish, BurntOutGoesDownWithoutTheWait) {
  GainWishState state = Feed(On(GainLevel::kNight, 100.0), 101.0, 9, 230.0F);
  EXPECT_EQ(state.level, GainLevel::kNight);
  state = NextGainWish(state, 110.0, 230.0F, {});
  EXPECT_EQ(state.level, GainLevel::kDusk);
  EXPECT_FALSE(state.trying_down);
  state = Feed(state, 111.0, 10, 230.0F);
  EXPECT_EQ(state.level, GainLevel::kDay);
}

TEST(GainWish, TriesTheLowerLevelAfterTheWaitWithReserve) {
  GainWishState state = Feed(On(GainLevel::kDusk, 0.0), 1.0, 599, 145.0F);
  EXPECT_EQ(state.level, GainLevel::kDusk);
  state = NextGainWish(state, 600.0, 145.0F, {});
  EXPECT_EQ(state.level, GainLevel::kDay);
  EXPECT_TRUE(state.trying_down);
}

TEST(GainWish, NoTryWithoutReserve) {
  EXPECT_EQ(Feed(On(GainLevel::kDusk, 0.0), 1.0, 4000, 110.0F).level, GainLevel::kDusk);
}

TEST(GainWish, ATryThatGoesDarkDoublesTheWait) {
  GainWishState state = Feed(On(GainLevel::kDusk, 0.0), 1.0, 600, 145.0F);
  ASSERT_EQ(state.level, GainLevel::kDay);
  // Dark from 601 on: settled at 605, up again 20 s later.
  state = Feed(state, 601.0, 25, 70.0F);
  ASSERT_EQ(state.level, GainLevel::kDusk);
  EXPECT_DOUBLE_EQ(state.changed_at, 625.0);
  EXPECT_DOUBLE_EQ(state.try_down_wait, 1200.0);
  state = Feed(state, 626.0, 1199, 145.0F);
  EXPECT_EQ(state.level, GainLevel::kDusk);
  state = NextGainWish(state, 1825.0, 145.0F, {});
  EXPECT_EQ(state.level, GainLevel::kDay);
  state = Feed(state, 1826.0, 25, 70.0F);
  EXPECT_DOUBLE_EQ(state.try_down_wait, 2400.0);
  state = Feed(state, 1851.0, 2400, 145.0F);
  state = Feed(state, 4251.0, 25, 70.0F);
  EXPECT_DOUBLE_EQ(state.try_down_wait, 3600.0);
  state = Feed(state, 4276.0, 3600, 145.0F);
  state = Feed(state, 7876.0, 25, 70.0F);
  EXPECT_DOUBLE_EQ(state.try_down_wait, 3600.0);
}

TEST(GainWish, ATryThatHoldsSetsTheWaitBack) {
  GainWishState state = Feed(On(GainLevel::kNight, 0.0), 1.0, 600, 145.0F);
  state = Feed(state, 601.0, 25, 70.0F);
  ASSERT_EQ(state.level, GainLevel::kNight);
  ASSERT_DOUBLE_EQ(state.try_down_wait, 1200.0);
  state = Feed(state, 626.0, 1200, 145.0F);
  ASSERT_EQ(state.level, GainLevel::kDusk);
  state = Feed(state, 1826.0, 60, 145.0F);
  EXPECT_FALSE(state.trying_down);
  EXPECT_DOUBLE_EQ(state.try_down_wait, 0.0);
  // Dark later on is the evening, not the try.
  state = Feed(state, 1886.0, 21, 70.0F);
  EXPECT_EQ(state.level, GainLevel::kNight);
  EXPECT_DOUBLE_EQ(state.try_down_wait, 0.0);
}

TEST(GainWish, AGapStartsTheDarkTimeAnew) {
  GainWishState state = Feed({}, 100.0, 15, 60.0F);
  // No stream for ten minutes.
  state = Feed(state, 715.0, 20, 60.0F);
  EXPECT_EQ(state.level, GainLevel::kDay);
  state = NextGainWish(state, 735.0, 60.0F, {});
  EXPECT_EQ(state.level, GainLevel::kDusk);
}

TEST(GainWish, TheTimeOnALevelCountsOverAGap) {
  GainWishState state = Feed(On(GainLevel::kNight, 0.0), 1.0, 10, 145.0F);
  // The next stream is in the morning.
  state = NextGainWish(state, 30000.0, 145.0F, {});
  EXPECT_EQ(state.level, GainLevel::kDusk);
  EXPECT_TRUE(state.trying_down);
}

TEST(GainWish, AClockThatJumpsBackCountsAsAGap) {
  GainWishState state = Feed({}, 100.0, 15, 60.0F);
  state = Feed(state, 50.0, 20, 60.0F);
  EXPECT_EQ(state.level, GainLevel::kDay);
}

TEST(GainWishFile, IsWrittenWholeIntoADirectoryItMakes) {
  test::TempDir dir;
  const std::string path = dir.Path() + "/gain/wish";
  std::string error;
  ASSERT_TRUE(WriteGainWish(path, GainLevel::kDusk, 12.5, 1794800000.0, error)) << error;
  EXPECT_EQ(FirstLine(path), "dusk 12.500 1794800000.000");
  ASSERT_TRUE(WriteGainWish(path, GainLevel::kNight, 22.5, 1794800010.0, error)) << error;
  EXPECT_EQ(FirstLine(path), "night 22.500 1794800010.000");
  EXPECT_FALSE(std::filesystem::exists(dir.Path() + "/gain/wish.new"));
}

TEST(GainWishFile, SaysWhyItCouldNotBeWritten) {
  test::TempDir dir;
  dir.Write("gain", "no directory\n");
  std::string error;
  EXPECT_FALSE(WriteGainWish(dir.Path() + "/gain/wish", GainLevel::kDay, 1.0, 2.0, error));
  EXPECT_NE(error.find("gain"), std::string::npos) << error;
}

TEST(FileLock, CountsAsHeldWithoutAFile) {
  test::TempDir dir;
  EXPECT_TRUE(FileLock("", 0.0).held());
  EXPECT_TRUE(FileLock(dir.Path() + "/none", 0.0).held());
}

TEST(FileLock, IsNotHeldWhileAnotherHoldsIt) {
  test::TempDir dir;
  dir.Write("lock", "");
  const std::string path = dir.Path() + "/lock";
  {
    const FileLock first(path, 0.0);
    EXPECT_TRUE(first.held());
    EXPECT_FALSE(FileLock(path, 0.03).held());
  }
  EXPECT_TRUE(FileLock(path, 0.0).held());
}

}  // namespace
}  // namespace tron_vision
