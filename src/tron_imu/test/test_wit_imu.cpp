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

tron::WitImuConfig UpsideDown() {
  tron::WitImuConfig config;
  config.mount_roll_deg = 180;
  return config;
}

TEST(WitImu, UprightByDefault) {
  tron::WitImu imu({});
  auto s = Feed3(imu, {0, 0, 1}, {0, 0, 0}, {0, 0, 0});
  ASSERT_TRUE(s);
  EXPECT_NEAR(s->acceleration[2], 9.80665, 1e-2);
}

TEST(WitImu, UpsideDownAtRestIsLevelInBaseFrame) {
  tron::WitImu imu(UpsideDown());
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
  tron::WitImu imu(UpsideDown());
  auto s = Feed3(imu, {0, 0, -1}, {10, 20, 30}, {-180, 0, 0});
  ASSERT_TRUE(s);
  const double d2r = M_PI / 180.0;
  EXPECT_NEAR(s->angular_velocity[0], 10 * d2r, 1e-3);
  EXPECT_NEAR(s->angular_velocity[1], -20 * d2r, 1e-3);
  EXPECT_NEAR(s->angular_velocity[2], -30 * d2r, 1e-3);
}

TEST(WitImu, SensorPitchUpIsPitchUpInBase) {
  // Rotation about x keeps the sign of pitch only when the mount is a pure x flip: pitch sign inverts.
  tron::WitImu imu(UpsideDown());
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
  tron::WitImu imu(UpsideDown());
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

namespace {

tron::ImuSample Resting(double a_z, double rate = 0.0) {
  tron::ImuSample s{};
  s.acceleration = {0, 0, a_z};
  s.angular_velocity = {0, 0, rate};
  return s;
}

// Feeds the sample at 50 Hz from `from` for `seconds` and returns the last answer.
bool Hold(tron::UpsideDownCheck& check, const tron::ImuSample& sample, double from, double seconds) {
  bool upside_down = false;
  for (double t = from; t <= from + seconds + 1e-9; t += 0.02) upside_down = check.Update(sample, t);
  return upside_down;
}

}  // namespace

TEST(UpsideDownCheck, GravityUpAtRestForOneSecond) {
  tron::UpsideDownCheck check;
  EXPECT_FALSE(Hold(check, Resting(-9.81), 0.0, 0.9));
  EXPECT_TRUE(Hold(check, Resting(-9.81), 0.92, 0.2));
}

TEST(UpsideDownCheck, UprightOnASlopeOrJackedUpIsFine) {
  tron::UpsideDownCheck check;
  // 3.6 deg jacked up, 31 deg the steepest slope of the vendor, and level.
  EXPECT_FALSE(Hold(check, Resting(9.81 * std::cos(3.6 * M_PI / 180)), 0.0, 3.0));
  EXPECT_FALSE(Hold(check, Resting(9.81 * std::cos(31 * M_PI / 180)), 3.0, 3.0));
  EXPECT_FALSE(Hold(check, Resting(9.81), 6.0, 3.0));
}

TEST(UpsideDownCheck, MovingOrBriefDoesNotCount) {
  tron::UpsideDownCheck check;
  EXPECT_FALSE(Hold(check, Resting(-9.81, 1.0), 0.0, 3.0));
  for (double t = 3.0; t < 10.0; t += 0.6) {
    EXPECT_FALSE(Hold(check, Resting(-9.81), t, 0.5));
    EXPECT_FALSE(check.Update(Resting(9.81), t + 0.55));
  }
}

TEST(UpsideDownCheck, EndsAfterOneSecondUpright) {
  tron::UpsideDownCheck check;
  ASSERT_TRUE(Hold(check, Resting(-9.81), 0.0, 1.1));
  // Moving does not end it, a brief upright reading neither.
  EXPECT_TRUE(Hold(check, Resting(-9.81, 2.0), 1.2, 1.0));
  EXPECT_TRUE(Hold(check, Resting(9.81), 2.3, 0.5));
  EXPECT_TRUE(check.Update(Resting(-9.81), 2.9));
  EXPECT_FALSE(Hold(check, Resting(9.81), 3.0, 1.1));
}
