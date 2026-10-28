#include "tron_imu/wit_imu.h"

#include <algorithm>
#include <cmath>

namespace tron {

namespace {

using Mat = std::array<std::array<double, 3>, 3>;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kGravity = 9.80665;

// R = Rz(yaw) * Ry(pitch) * Rx(roll)
Mat FromRpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
           {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
           {-sp, cp * sr, cp * cr}}};
}

Mat Mul(const Mat& a, const Mat& b) {
  Mat r{};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++) r[i][j] += a[i][k] * b[k][j];
  return r;
}

Mat Transpose(const Mat& a) {
  Mat r{};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) r[i][j] = a[j][i];
  return r;
}

std::array<double, 3> Apply(const Mat& m, const std::array<double, 3>& v) {
  std::array<double, 3> r{};
  for (int i = 0; i < 3; i++) r[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
  return r;
}

std::array<double, 4> ToQuaternion(const Mat& m) {
  const double trace = m[0][0] + m[1][1] + m[2][2];
  double x, y, z, w;
  if (trace > 0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (m[2][1] - m[1][2]) / s;
    y = (m[0][2] - m[2][0]) / s;
    z = (m[1][0] - m[0][1]) / s;
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    w = (m[2][1] - m[1][2]) / s;
    x = 0.25 * s;
    y = (m[0][1] + m[1][0]) / s;
    z = (m[0][2] + m[2][0]) / s;
  } else if (m[1][1] > m[2][2]) {
    const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
    w = (m[0][2] - m[2][0]) / s;
    x = (m[0][1] + m[1][0]) / s;
    y = 0.25 * s;
    z = (m[1][2] + m[2][1]) / s;
  } else {
    const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    w = (m[1][0] - m[0][1]) / s;
    x = (m[0][2] + m[2][0]) / s;
    y = (m[1][2] + m[2][1]) / s;
    z = 0.25 * s;
  }
  return {x, y, z, w};
}

}  // namespace

WitImu::WitImu(const WitImuConfig& config)
    : mount_(FromRpy(config.mount_roll_deg * kDegToRad, config.mount_pitch_deg * kDegToRad,
                     config.mount_yaw_deg * kDegToRad)) {
}

std::optional<ImuSample> WitImu::Update(const WitPacket& packet) {
  switch (packet.type) {
    case WitType::kAcceleration:
      for (size_t i = 0; i < 3; i++) acc_g_[i] = packet.raw[i] / 32768.0 * 16.0;
      have_acc_ = true;
      return std::nullopt;
    case WitType::kAngularVelocity:
      for (size_t i = 0; i < 3; i++) gyro_dps_[i] = packet.raw[i] / 32768.0 * 2000.0;
      have_gyro_ = true;
      return std::nullopt;
    case WitType::kAngle: break;
  }
  if (!have_acc_ || !have_gyro_) return std::nullopt;
  have_acc_ = have_gyro_ = false;

  const double roll = packet.raw[0] / 32768.0 * 180.0 * kDegToRad;
  const double pitch = packet.raw[1] / 32768.0 * 180.0 * kDegToRad;
  const double yaw = packet.raw[2] / 32768.0 * 180.0 * kDegToRad;
  // The sensor orientation in its own world frame, turned into the orientation of the base frame.
  const Mat r_wb = Mul(FromRpy(roll, pitch, yaw), Transpose(mount_));

  ImuSample s;
  s.orientation = ToQuaternion(r_wb);
  s.acceleration = Apply(mount_, {acc_g_[0] * kGravity, acc_g_[1] * kGravity, acc_g_[2] * kGravity});
  s.angular_velocity = Apply(mount_, {gyro_dps_[0] * kDegToRad, gyro_dps_[1] * kDegToRad, gyro_dps_[2] * kDegToRad});
  s.pitch_deg = std::asin(std::clamp(-r_wb[2][0], -1.0, 1.0)) * kRadToDeg;
  s.roll_deg = std::atan2(r_wb[2][1], r_wb[2][2]) * kRadToDeg;
  s.yaw_deg = std::atan2(r_wb[1][0], r_wb[0][0]) * kRadToDeg;
  return s;
}

}  // namespace tron
