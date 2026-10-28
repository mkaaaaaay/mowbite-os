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
  // Rotation of the sensor axes into the base axes (ZYX). The JY61P sits upside down: roll 180.
  double mount_roll_deg = 180.0;
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

}  // namespace tron

#endif  // TRON_IMU_WIT_IMU_H
