#include <gtest/gtest.h>

#include "mowbite_watch/tick_gap.h"

namespace mowbite_watch {
namespace {

TEST(TickGap, TicksOnTimeAlwaysJudge) {
  TickGap gap(0.3);
  for (double t = 0; t < 5; t += 0.1) EXPECT_TRUE(gap.Judge(t));
}

TEST(TickGap, FirstTickAfterAStallDoesNotJudge) {
  TickGap gap(0.3);
  EXPECT_TRUE(gap.Judge(0.0));
  EXPECT_TRUE(gap.Judge(0.1));
  EXPECT_FALSE(gap.Judge(2.3));
  EXPECT_TRUE(gap.Judge(2.4));
}

TEST(TickGap, ExactlyTheGapIsOnTime) {
  TickGap gap(0.5);
  EXPECT_TRUE(gap.Judge(1.0));
  EXPECT_TRUE(gap.Judge(1.5));
}

TEST(TickGap, AlwaysLateJudgesEveryOtherTick) {
  TickGap gap(0.3);
  EXPECT_TRUE(gap.Judge(0.0));
  EXPECT_FALSE(gap.Judge(1.0));
  EXPECT_TRUE(gap.Judge(2.0));
  EXPECT_FALSE(gap.Judge(3.0));
  EXPECT_TRUE(gap.Judge(4.0));
}

TEST(TickGap, TheFirstTickJudges) {
  TickGap gap(0.3);
  EXPECT_TRUE(gap.Judge(100.0));
}

}  // namespace
}  // namespace mowbite_watch
