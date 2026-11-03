#include <gtest/gtest.h>

#include "xcore_driver/odometry.h"

using namespace xcore;

TEST(Odometry, StraightAndTurning) {
  OdometryConfig config;
  config.ticks_per_meter = 1000;
  config.wheel_distance = 0.5;
  config.right_sign = 1;
  TickOdometry odom(config);
  EXPECT_FALSE(odom.OnTicks(100, 200, 1000000));
  // 20 ms, 10 ticks each: 0.5 m/s.
  auto t = odom.OnTicks(110, 210, 1020000);
  ASSERT_TRUE(t);
  EXPECT_NEAR(t->linear, 0.5, 1e-9);
  EXPECT_NEAR(t->angular, 0, 1e-9);
  EXPECT_NEAR(t->dt, 0.02, 1e-12);
  // Left back, right forward: turning left on the spot.
  t = odom.OnTicks(100, 220, 1040000);
  ASSERT_TRUE(t);
  EXPECT_NEAR(t->linear, 0, 1e-9);
  EXPECT_NEAR(t->angular, (0.5 - -0.5) / 0.5, 1e-9);
}

TEST(Odometry, CountersAndClockWrap) {
  OdometryConfig config;
  config.ticks_per_meter = 1000;
  config.right_sign = 1;
  TickOdometry odom(config);
  odom.OnTicks(0xFFFFFFF0u, 5, 0xFFFFFFFFull - 5000);
  auto t = odom.OnTicks(0x0000000Au, 0xFFFFFFFBu, 15000);
  ASSERT_TRUE(t);
  // 26 ticks forward and 10 back in 20 ms.
  EXPECT_NEAR(t->dt, 0.020001, 1e-9);
  EXPECT_NEAR(t->left, 26 / 1000.0 / 0.020001, 1e-9);
  EXPECT_NEAR(t->right, -10 / 1000.0 / 0.020001, 1e-9);
}

TEST(Odometry, SignsAndGaps) {
  OdometryConfig config;
  config.ticks_per_meter = 1000;
  config.right_sign = -1;
  TickOdometry odom(config);
  odom.OnTicks(0, 0, 0);
  auto t = odom.OnTicks(10, static_cast<uint32_t>(-10), 20000);
  ASSERT_TRUE(t);
  EXPECT_NEAR(t->left, 0.5, 1e-9);
  EXPECT_NEAR(t->right, 0.5, 1e-9);
  // A gap gives nothing, the pair after it counts again.
  EXPECT_FALSE(odom.OnTicks(20, 0, 1000000));
  EXPECT_TRUE(odom.OnTicks(30, 0, 1020000));
  // The same time stamp twice.
  EXPECT_FALSE(odom.OnTicks(30, 0, 1020000));
  odom.Reset();
  EXPECT_FALSE(odom.OnTicks(30, 0, 1040000));
}

TEST(Odometry, RightCounterFallsGoingForwardByDefault) {
  OdometryConfig config;
  TickOdometry odom(config);
  odom.OnTicks(0, 0, 0);
  auto t = odom.OnTicks(81, static_cast<uint32_t>(-81), 40000);
  ASSERT_TRUE(t);
  EXPECT_NEAR(t->linear, 81 / 2025.0 / 0.04, 1e-9);
  EXPECT_NEAR(t->angular, 0, 1e-9);
}
