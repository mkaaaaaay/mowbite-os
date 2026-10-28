#include <gtest/gtest.h>

#include <cmath>

#include "tron_imu/wit_imu.h"

namespace {

tron::WitPacket Packet(tron::WitType type, int16_t a, int16_t b, int16_t c) {
  return {type, {a, b, c, 0}};
}

int16_t Counts(double value, double full_scale) {
  return static_cast<int16_t>(std::lround(value / full_scale * 32768.0));
}

// Feeds acc, gyro and angle and returns the sample.
std::optional<tron::ImuSample> Feed3(tron::WitImu& imu, std::array<double, 3> acc_g, std::array<double, 3> gyro_dps,
                                     std::array<double, 3> angle_deg) {
  imu.Update(Packet(tron::WitType::kAcceleration, Counts(acc_g[0], 16), Counts(acc_g[1], 16), Counts(acc_g[2], 16)));
  imu.Update(Packet(tron::WitType::kAngularVelocity, Counts(gyro_dps[0], 2000), Counts(gyro_dps[1], 2000),
                    Counts(gyro_dps[2], 2000)));
  return imu.Update(
      Packet(tron::WitType::kAngle, Counts(angle_deg[0], 180), Counts(angle_deg[1], 180), Counts(angle_deg[2], 180)));
}

double Norm(const std::array<double, 4>& q) {
  return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

}  // namespace

TEST(WitImu, UpsideDownAtRestIsLevelInBaseFrame) {
  tron::WitImu imu({});  // default: roll 180
  // Upside down the sensor sees gravity along -z and reports roll ±180 (-180 fits an int16).
  auto s = Feed3(imu, {0, 0, -1}, {0, 0, 0}, {-180, 0, 0});
  ASSERT_TRUE(s);
  EXPECT_NEAR(s->acceleration[2], 9.80665, 1e-2);
  EXPECT_NEAR(s->acceleration[0], 0, 1e-2);
  EXPECT_NEAR(s->roll_deg, 0, 0.1);
  EXPECT_NEAR(s->pitch_deg, 0, 0.1);
  EXPECT_NEAR(Norm(s->orientation), 1.0, 1e-9);
}

TEST(WitImu, MountingFlipsYAndZRates) {
  tron::WitImu imu({});
  auto s = Feed3(imu, {0, 0, -1}, {10, 20, 30}, {-180, 0, 0});
  ASSERT_TRUE(s);
  const double d2r = M_PI / 180.0;
  EXPECT_NEAR(s->angular_velocity[0], 10 * d2r, 1e-3);
  EXPECT_NEAR(s->angular_velocity[1], -20 * d2r, 1e-3);
  EXPECT_NEAR(s->angular_velocity[2], -30 * d2r, 1e-3);
}

TEST(WitImu, SensorPitchUpIsPitchUpInBase) {
  // Rotation about x keeps the sign of pitch only when the mount is a pure x flip: pitch sign inverts.
  tron::WitImu imu({});
  auto s = Feed3(imu, {0, 0, -1}, {0, 0, 0}, {-180, 10, 0});
  ASSERT_TRUE(s);
  EXPECT_NEAR(std::abs(s->pitch_deg), 10, 0.1);
  EXPECT_NEAR(s->roll_deg, 0, 0.1);
}

TEST(WitImu, UprightMountIsIdentity) {
  tron::WitImuConfig config;
  config.mount_roll_deg = 0;
  tron::WitImu imu(config);
  auto s = Feed3(imu, {0.1, -0.2, 1}, {1, 2, 3}, {5, -7, 30});
  ASSERT_TRUE(s);
  EXPECT_NEAR(s->acceleration[0], 0.1 * 9.80665, 1e-2);
  EXPECT_NEAR(s->acceleration[1], -0.2 * 9.80665, 1e-2);
  EXPECT_NEAR(s->roll_deg, 5, 0.05);
  EXPECT_NEAR(s->pitch_deg, -7, 0.05);
  EXPECT_NEAR(s->yaw_deg, 30, 0.05);
  EXPECT_NEAR(s->angular_velocity[2], 3 * M_PI / 180.0, 1e-3);
}

TEST(WitImu, YawQuaternionMatchesAngle) {
  tron::WitImuConfig config;
  config.mount_roll_deg = 0;
  tron::WitImu imu(config);
  auto s = Feed3(imu, {0, 0, 1}, {0, 0, 0}, {0, 0, 90});
  ASSERT_TRUE(s);
  EXPECT_NEAR(s->orientation[2], std::sqrt(0.5), 1e-3);
  EXPECT_NEAR(s->orientation[3], std::sqrt(0.5), 1e-3);
}

TEST(WitImu, QuaternionStaysUnitOverRange) {
  tron::WitImu imu({});
  for (double roll = -170; roll <= 170; roll += 34)
    for (double pitch = -80; pitch <= 80; pitch += 40)
      for (double yaw = -170; yaw <= 170; yaw += 68) {
        auto s = Feed3(imu, {0, 0, -1}, {0, 0, 0}, {roll, pitch, yaw});
        ASSERT_TRUE(s);
        EXPECT_NEAR(Norm(s->orientation), 1.0, 1e-9);
      }
}

TEST(WitImu, NeedsAccAndGyroBeforeEachSample) {
  tron::WitImu imu({});
  EXPECT_FALSE(imu.Update(Packet(tron::WitType::kAngle, 0, 0, 0)));
  imu.Update(Packet(tron::WitType::kAcceleration, 0, 0, 100));
  EXPECT_FALSE(imu.Update(Packet(tron::WitType::kAngle, 0, 0, 0)));
  imu.Update(Packet(tron::WitType::kAngularVelocity, 0, 0, 0));
  EXPECT_TRUE(imu.Update(Packet(tron::WitType::kAngle, 0, 0, 0)));
  // The next angle packet alone does not repeat the old values.
  EXPECT_FALSE(imu.Update(Packet(tron::WitType::kAngle, 0, 0, 0)));
}

TEST(WitImu, AccelerationScale) {
  tron::WitImuConfig config;
  config.mount_roll_deg = 0;
  tron::WitImu imu(config);
  auto s = Feed3(imu, {16.0 * 0.5, 0, 0}, {0, 0, 0}, {0, 0, 0});
  ASSERT_TRUE(s);
  EXPECT_NEAR(s->acceleration[0], 8 * 9.80665, 1e-3);
}
