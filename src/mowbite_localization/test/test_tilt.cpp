#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

#include "mowbite_localization/tilt.h"

using namespace mowbite_localization;

namespace {

constexpr double kDeg = M_PI / 180.0;
constexpr double kG = 9.80665;

// Rz(yaw) Ry(pitch) Rx(roll).
Eigen::Matrix3d Rotation(double roll, double pitch, double yaw = 0.0) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// What a resting accelerometer of a base at that tilt reads, its IMU turned by the mounting.
Eigen::Vector3d Reading(const Tilt& tilt, const Tilt& mounting = {}) {
  return Rotation(mounting.roll, mounting.pitch).transpose() * Rotation(tilt.roll, tilt.pitch).transpose() *
         Eigen::Vector3d(0.0, 0.0, kG);
}

TEST(TiltFromGravity, LevelRollAndPitch) {
  const Tilt level = TiltFromGravity(Eigen::Vector3d(0.0, 0.0, kG), {});
  EXPECT_EQ(level.roll, 0.0);
  EXPECT_EQ(level.pitch, 0.0);
  // Right side down: gravity leans to +y in the frame of the base.
  const Tilt rolled = TiltFromGravity(Reading({10.0 * kDeg, 0.0}), {});
  EXPECT_NEAR(rolled.roll, 10.0 * kDeg, 1e-12);
  EXPECT_NEAR(rolled.pitch, 0.0, 1e-12);
  const Tilt pitched = TiltFromGravity(Reading({0.0, -7.0 * kDeg}), {});
  EXPECT_NEAR(pitched.roll, 0.0, 1e-12);
  EXPECT_NEAR(pitched.pitch, -7.0 * kDeg, 1e-12);
}

TEST(TiltFromGravity, TheMountingOfTheGeometryReadsLevel) {
  // imu_roll and imu_pitch are what the IMU reads on level ground: roll atan2(ay, az), pitch atan2(-ax, hypot(ay, az)).
  const Tilt mounting{2.0 * kDeg, -1.5 * kDeg};
  const Eigen::Vector3d level = Reading({}, mounting);
  EXPECT_NEAR(std::atan2(level(1), level(2)), mounting.roll, 1e-12);
  EXPECT_NEAR(std::atan2(-level(0), std::hypot(level(1), level(2))), mounting.pitch, 1e-12);
  const Tilt base = TiltFromGravity(level, mounting);
  EXPECT_NEAR(base.roll, 0.0, 1e-12);
  EXPECT_NEAR(base.pitch, 0.0, 1e-12);
  const Tilt slope = TiltFromGravity(Reading({8.0 * kDeg, 12.0 * kDeg}, mounting), mounting);
  EXPECT_NEAR(slope.roll, 8.0 * kDeg, 1e-12);
  EXPECT_NEAR(slope.pitch, 12.0 * kDeg, 1e-12);
}

TEST(TiltFromGravity, NothingUsableIsLevel) {
  const Tilt zero = TiltFromGravity(Eigen::Vector3d::Zero(), {});
  EXPECT_EQ(zero.roll, 0.0);
  EXPECT_EQ(zero.pitch, 0.0);
  const Tilt nan = TiltFromGravity(Eigen::Vector3d(0.0, NAN, kG), {});
  EXPECT_EQ(nan.roll, 0.0);
  EXPECT_EQ(nan.pitch, 0.0);
}

TEST(GravityFromOrientation, TheYawDoesNotCount) {
  const Eigen::Quaterniond q(Rotation(-8.0 * kDeg, 5.0 * kDeg, 30.0 * kDeg));
  const Tilt tilt = TiltFromGravity(GravityFromOrientation(q.x(), q.y(), q.z(), q.w()), {});
  EXPECT_NEAR(tilt.roll, -8.0 * kDeg, 1e-12);
  EXPECT_NEAR(tilt.pitch, 5.0 * kDeg, 1e-12);
  // Not of unit length.
  const Tilt scaled = TiltFromGravity(GravityFromOrientation(2.0 * q.x(), 2.0 * q.y(), 2.0 * q.z(), 2.0 * q.w()), {});
  EXPECT_NEAR(scaled.roll, -8.0 * kDeg, 1e-12);
  EXPECT_NEAR(scaled.pitch, 5.0 * kDeg, 1e-12);
}

TEST(TiltedLever, TheAntennaLeansDownhill) {
  const Eigen::Vector3d lever(0.1, 0.0, 0.3);
  const Eigen::Vector2d level = TiltedLever(lever, {});
  EXPECT_EQ(level(0), 0.1);
  EXPECT_EQ(level(1), 0.0);
  // Right side 10 degrees down: 5.2 cm to the right.
  const Eigen::Vector2d rolled = TiltedLever(lever, {10.0 * kDeg, 0.0});
  EXPECT_NEAR(rolled(0), 0.1, 1e-12);
  EXPECT_NEAR(rolled(1), -0.3 * std::sin(10.0 * kDeg), 1e-12);
  EXPECT_NEAR(rolled(1), -0.0521, 1e-4);
  // Front 10 degrees down: ahead by the height and a little less by the offset.
  const Eigen::Vector2d pitched = TiltedLever(lever, {0.0, 10.0 * kDeg});
  EXPECT_NEAR(pitched(0), 0.1 * std::cos(10.0 * kDeg) + 0.3 * std::sin(10.0 * kDeg), 1e-12);
  EXPECT_NEAR(pitched(1), 0.0, 1e-12);
  // Both: the horizontal part of the turned lever.
  const Eigen::Vector3d turned = Rotation(6.0 * kDeg, -9.0 * kDeg) * Eigen::Vector3d(0.1, 0.05, 0.3);
  const Eigen::Vector2d both = TiltedLever(Eigen::Vector3d(0.1, 0.05, 0.3), {6.0 * kDeg, -9.0 * kDeg});
  EXPECT_NEAR(both(0), turned(0), 1e-12);
  EXPECT_NEAR(both(1), turned(1), 1e-12);
}

TEST(GravityFilter, FollowsWithItsTimeConstantAndIgnoresPushes) {
  GravityFilter filter(1.0, 0.1, 2.0);
  const auto first = filter.Add(0.0, Eigen::Vector3d(0.0, 0.0, kG));
  ASSERT_TRUE(first);
  EXPECT_EQ(*first, Eigen::Vector3d(0.0, 0.0, 1.0));
  // A push of 0.5 g while driving does not count.
  const auto pushed = filter.Add(0.02, Eigen::Vector3d(0.5 * kG, 0.0, kG));
  ASSERT_TRUE(pushed);
  EXPECT_EQ(*pushed, Eigen::Vector3d(0.0, 0.0, 1.0));
  // Onto a 10 degree side slope: after one time constant 63 % of the way.
  const Eigen::Vector3d slope = Reading({10.0 * kDeg, 0.0});
  std::optional<Eigen::Vector3d> gravity;
  for (int i = 1; i <= 50; ++i) gravity = filter.Add(0.02 + 0.02 * i, slope);
  ASSERT_TRUE(gravity);
  EXPECT_NEAR(TiltFromGravity(*gravity, {}).roll, 10.0 * kDeg * (1.0 - std::exp(-1.0)), 0.2 * kDeg);
  for (int i = 51; i <= 500; ++i) gravity = filter.Add(0.02 + 0.02 * i, slope);
  EXPECT_NEAR(TiltFromGravity(*gravity, {}).roll, 10.0 * kDeg, 0.01 * kDeg);
}

TEST(GravityFilter, OnlyPushesForLongerThanTheHoldIsNoGravity) {
  GravityFilter filter(1.0, 0.1, 2.0);
  ASSERT_TRUE(filter.Add(0.0, Eigen::Vector3d(0.0, 0.0, kG)));
  EXPECT_TRUE(filter.Add(2.0, Eigen::Vector3d(0.0, 0.0, 2.0 * kG)));
  EXPECT_FALSE(filter.Add(2.1, Eigen::Vector3d(0.0, 0.0, 2.0 * kG)));
  EXPECT_FALSE(filter.Add(2.2, Eigen::Vector3d(0.0, NAN, kG)));
  // The next good sample starts it again at once.
  const auto again = filter.Add(2.3, Reading({0.0, 5.0 * kDeg}));
  ASSERT_TRUE(again);
  EXPECT_NEAR(TiltFromGravity(*again, {}).pitch, 5.0 * kDeg, 1e-12);
}

}  // namespace
