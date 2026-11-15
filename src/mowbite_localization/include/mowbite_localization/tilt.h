#ifndef MOWBITE_LOCALIZATION_TILT_H
#define MOWBITE_LOCALIZATION_TILT_H

#include <Eigen/Core>
#include <optional>

namespace mowbite_localization {

// Roll and pitch of base_link (rad): its orientation is Rz(yaw) Ry(pitch) Rx(roll), as in REP 103. Positive roll lowers
// the right side, positive pitch the front.
struct Tilt {
  double roll = 0.0;
  double pitch = 0.0;
};

// The tilt of base_link that gravity says: the reading of a resting accelerometer (any length) in the frame of the IMU
// as its driver reports it, with that frame turned by mounting against base_link (robot.geometry imu_roll, imu_pitch:
// what the IMU reads on level ground).
Tilt TiltFromGravity(const Eigen::Vector3d& gravity, const Tilt& mounting);

// Gravity (unit length) in the frame of an IMU whose orientation is the quaternion x, y, z, w.
Eigen::Vector3d GravityFromOrientation(double x, double y, double z, double w);

// The horizontal part of a lever (m, in the frame of base_link from the ground point under it) on a base tilted that
// much, in the frame of base_link turned by its heading alone.
Eigen::Vector2d TiltedLever(const Eigen::Vector3d& lever, const Tilt& tilt);

// Gravity of an IMU without orientation: its acceleration low-passed with the time constant tau s, from samples
// within tolerance g of 1 g only, so that pushes while driving count little and bumps not at all.
class GravityFilter {
 public:
  explicit GravityFilter(double tau = 1.0, double tolerance = 0.1, double hold = 2.0);

  // A sample (m/s²) at t (s); the filtered direction while a sample within the tolerance came at most hold s ago.
  std::optional<Eigen::Vector3d> Add(double t, const Eigen::Vector3d& acceleration);

 private:
  double tau_;
  double tolerance_;
  double hold_;
  std::optional<double> last_;
  Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_TILT_H
