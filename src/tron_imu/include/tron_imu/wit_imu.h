#ifndef TRON_IMU_WIT_IMU_H
#define TRON_IMU_WIT_IMU_H

#include <array>
#include <optional>

#include "tron_imu/wit_parser.h"

namespace tron {

struct ImuSample {
  std::array<double, 4> orientation;       // x, y, z, w; base frame in the world frame of the sensor
  std::array<double, 3> acceleration;      // m/s^2, base frame
  std::array<double, 3> angular_velocity;  // rad/s, base frame
  double roll_deg, pitch_deg, yaw_deg;     // base frame, ZYX
};

struct WitImuConfig {
  // Rotation of the sensor axes into the base axes (ZYX). The JY61P of the Tron sits upright (measured 07.11.).
  double mount_roll_deg = 0.0;
  double mount_pitch_deg = 0.0;
  double mount_yaw_deg = 0.0;
};

// Collects acceleration, angular velocity and angle packets and emits one sample per angle packet, but only when both
// other packets arrived since the last sample.
class WitImu {
 public:
  explicit WitImu(const WitImuConfig& config);
  std::optional<ImuSample> Update(const WitPacket& packet);

 private:
  std::array<std::array<double, 3>, 3> mount_;  // vector in base = mount_ * vector in sensor
  std::array<double, 3> acc_g_{};
  std::array<double, 3> gyro_dps_{};
  bool have_acc_ = false;
  bool have_gyro_ = false;
};

// Gravity pointing up in the base frame while the sensor rests: a wrong mount_roll_deg, or a mower on its back. Either
// way no sample may go on. Starts after kUpsideDownTime s at rest with a_z below kUpsideDownAccel, ends after as long
// with a_z above -kUpsideDownAccel; a slope or a jacked-up mower stays far from both.
class UpsideDownCheck {
 public:
  static constexpr double kUpsideDownAccel = -7.0;
  static constexpr double kUpsideDownTime = 1.0;
  // rad/s on every axis: faster counts as moving.
  static constexpr double kRestRate = 0.5;

  // True while the IMU counts as upside down.
  bool Update(const ImuSample& sample, double now);
  bool upside_down() const {
    return upside_down_;
  }

 private:
  bool upside_down_ = false;
  std::optional<double> since_;
};

}  // namespace tron

#endif  // TRON_IMU_WIT_IMU_H
