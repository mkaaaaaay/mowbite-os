#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "mowbite_localization/fix_gate.h"

using namespace mowbite_localization;

namespace {

GnssFix Fixed(double x, double y, double accuracy = 0.02) {
  GnssFix fix;
  fix.flags = 1 | kFlagRtkFixed;
  fix.accuracy = accuracy;
  fix.x = x;
  fix.y = y;
  return fix;
}

// Classifies fixes at 5 Hz from t, all at (x, y), with the estimate expecting the antenna there too.
FixResult Feed(FixGate& gate, double& t, int count, double x, double y) {
  FixResult result = FixResult::kNotReady;
  for (int i = 0; i < count; ++i) {
    t += 0.2;
    result = gate.Classify(t, Eigen::Vector2d(x, y), Eigen::Vector2d(x, y));
  }
  return result;
}

// L3: only RTK fixed with a good accuracy counts.
TEST(L3FixesThatCount, OnlyRtkFixedWithinTheAccuracy) {
  FixGate gate({});
  EXPECT_FALSE(gate.Reject(Fixed(1, 1)));
  GnssFix fl = Fixed(1, 1);
  fl.flags = 1 | 4;
  EXPECT_EQ(gate.Reject(fl), FixResult::kNotFixed);
  EXPECT_EQ(gate.Reject(Fixed(1, 1, 0.11)), FixResult::kInaccurate);
  EXPECT_EQ(gate.Reject(Fixed(1, 1, std::numeric_limits<double>::quiet_NaN())), FixResult::kInaccurate);
  EXPECT_EQ(gate.Reject(Fixed(std::numeric_limits<double>::quiet_NaN(), 1)), FixResult::kInaccurate);
  gate.SetMaxAccuracy(0.01);
  EXPECT_EQ(gate.Reject(Fixed(1, 1, 0.02)), FixResult::kInaccurate);
}

// L4: the first fix starts a warmup, ten more that agree end it.
TEST(L4Warmup, TheFirstFixAndTenMoreSetTheEstimate) {
  FixGate gate({});
  double t = 100;
  EXPECT_EQ(Feed(gate, t, 1, 3, 4), FixResult::kRestarted);
  EXPECT_TRUE(gate.warming_up());
  for (int i = 0; i < 10; ++i) EXPECT_EQ(Feed(gate, t, 1, 3, 4), FixResult::kWarmup);
  EXPECT_FALSE(gate.initialized());
  EXPECT_EQ(Feed(gate, t, 1, 3, 4), FixResult::kInitialized);
  EXPECT_TRUE(gate.initialized());
  EXPECT_EQ(gate.reason(), WarmupReason::kFirst);
  EXPECT_EQ(Feed(gate, t, 1, 3, 4), FixResult::kFused);
}

TEST(L4Warmup, AFixFarFromTheLastStartsItAgain) {
  FixGate gate({});
  double t = 100;
  Feed(gate, t, 6, 3, 4);
  EXPECT_EQ(Feed(gate, t, 1, 9, 4), FixResult::kRestarted);
  for (int i = 0; i < 10; ++i) EXPECT_EQ(Feed(gate, t, 1, 9, 4), FixResult::kWarmup);
  EXPECT_EQ(Feed(gate, t, 1, 9, 4), FixResult::kInitialized);
}

TEST(L4Warmup, FiveSecondsWithoutAFixStartItAgain) {
  FixGate gate({});
  double t = 100;
  Feed(gate, t, 12, 3, 4);
  ASSERT_TRUE(gate.initialized());
  t += 4.9;
  EXPECT_EQ(Feed(gate, t, 1, 3, 4), FixResult::kRestarted);
  EXPECT_FALSE(gate.initialized());
  EXPECT_EQ(gate.reason(), WarmupReason::kGap);
  EXPECT_EQ(Feed(gate, t, 10, 3, 4), FixResult::kWarmup);
  EXPECT_EQ(Feed(gate, t, 1, 3, 4), FixResult::kInitialized);
  EXPECT_EQ(gate.reason(), WarmupReason::kGap);
}

TEST(L4Warmup, AShorterGapKeepsTheFusion) {
  FixGate gate({});
  double t = 100;
  Feed(gate, t, 12, 3, 4);
  t += 4.5;
  EXPECT_EQ(Feed(gate, t, 1, 3, 4), FixResult::kFused);
}

TEST(L4Warmup, ForgetMakesTheNextOneAFirstOne) {
  FixGate gate({});
  double t = 100;
  Feed(gate, t, 12, 3, 4);
  gate.Forget();
  EXPECT_FALSE(gate.initialized());
  EXPECT_FALSE(gate.last_accepted());
  EXPECT_EQ(Feed(gate, t, 1, 3, 4), FixResult::kRestarted);
  Feed(gate, t, 11, 3, 4);
  EXPECT_EQ(gate.reason(), WarmupReason::kFirst);
}

// L5: outliers are dropped, the eleventh in a row starts a warmup on GNSS.
TEST(L5Outliers, TheEleventhInARowStartsAWarmup) {
  FixGate gate({});
  double t = 100;
  Feed(gate, t, 12, 10, 20);
  const double accepted = *gate.last_accepted();
  for (int i = 0; i < 10; ++i) {
    t += 0.2;
    EXPECT_EQ(gate.Classify(t, Eigen::Vector2d(16, 20), Eigen::Vector2d(10, 20)), FixResult::kOutlier);
  }
  EXPECT_TRUE(gate.initialized());
  EXPECT_DOUBLE_EQ(*gate.last_accepted(), accepted);
  t += 0.2;
  EXPECT_EQ(gate.Classify(t, Eigen::Vector2d(16, 20), Eigen::Vector2d(10, 20)), FixResult::kOutlierReset);
  EXPECT_FALSE(gate.initialized());
  EXPECT_EQ(gate.reason(), WarmupReason::kOutliers);
  EXPECT_EQ(Feed(gate, t, 10, 16.01, 20), FixResult::kWarmup);
  EXPECT_EQ(Feed(gate, t, 1, 16.01, 20), FixResult::kInitialized);
  EXPECT_EQ(gate.reason(), WarmupReason::kOutliers);
}

TEST(L5Outliers, AnInlierBreaksTheRun) {
  FixGate gate({});
  double t = 100;
  Feed(gate, t, 12, 10, 20);
  for (int i = 0; i < 10; ++i) {
    t += 0.2;
    gate.Classify(t, Eigen::Vector2d(16, 20), Eigen::Vector2d(10, 20));
  }
  EXPECT_EQ(Feed(gate, t, 1, 10, 20), FixResult::kFused);
  t += 0.2;
  EXPECT_EQ(gate.Classify(t, Eigen::Vector2d(16, 20), Eigen::Vector2d(10, 20)), FixResult::kOutlier);
}

TEST(L5Outliers, JustUnderFiveMetresIsNoOutlier) {
  FixGate gate({});
  double t = 100;
  Feed(gate, t, 12, 10, 20);
  t += 0.2;
  EXPECT_EQ(gate.Classify(t, Eigen::Vector2d(14.99, 20), Eigen::Vector2d(10, 20)), FixResult::kFused);
}

}  // namespace
