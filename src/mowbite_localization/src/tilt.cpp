#include "mowbite_localization/tilt.h"

#include <algorithm>
#include <cmath>

namespace mowbite_localization {

namespace {

constexpr double kGravity = 9.80665;

}  // namespace

Tilt TiltFromGravity(const Eigen::Vector3d& gravity, const Tilt& mounting) {
  if (!gravity.allFinite() || gravity.norm() <= 0.0) return Tilt{};
  // The IMU reads gravity turned by the transpose of Ry(pitch) Rx(roll) of its mounting: turned back into base_link.
  const double cr = std::cos(mounting.roll), sr = std::sin(mounting.roll);
  const double cp = std::cos(mounting.pitch), sp = std::sin(mounting.pitch);
  const Eigen::Vector3d rolled(gravity(0), cr * gravity(1) - sr * gravity(2), sr * gravity(1) + cr * gravity(2));
  const Eigen::Vector3d base(cp * rolled(0) + sp * rolled(2), rolled(1), -sp * rolled(0) + cp * rolled(2));
  return Tilt{std::atan2(base(1), base(2)), std::atan2(-base(0), std::hypot(base(1), base(2)))};
}

Eigen::Vector3d GravityFromOrientation(double x, double y, double z, double w) {
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm <= 0.0) return Eigen::Vector3d::UnitZ();
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  // The up axis of the world in the frame of the IMU: the last row of its rotation matrix.
  return {2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)};
}

Eigen::Vector2d TiltedLever(const Eigen::Vector3d& lever, const Tilt& tilt) {
  const double cr = std::cos(tilt.roll), sr = std::sin(tilt.roll);
  const double cp = std::cos(tilt.pitch), sp = std::sin(tilt.pitch);
  // Ry(pitch) Rx(roll) lever, its x and y.
  return {cp * lever(0) + sp * (sr * lever(1) + cr * lever(2)), cr * lever(1) - sr * lever(2)};
}

GravityFilter::GravityFilter(double tau, double tolerance, double hold)
    : tau_(tau), tolerance_(tolerance), hold_(hold) {
}

std::optional<Eigen::Vector3d> GravityFilter::Add(double t, const Eigen::Vector3d& acceleration) {
  const double norm = acceleration.norm();
  const bool usable = std::isfinite(t) && acceleration.allFinite() &&
                      std::abs(norm - kGravity) <= tolerance_ * kGravity;
  const bool fresh = last_ && std::isfinite(t) && t - *last_ <= hold_;
  if (!usable) return fresh ? std::optional(gravity_) : std::nullopt;
  const Eigen::Vector3d unit = acceleration / norm;
  if (fresh) {
    const double share = 1.0 - std::exp(-std::max(0.0, t - *last_) / tau_);
    gravity_ += share * (unit - gravity_);
  } else {
    gravity_ = unit;
  }
  last_ = t;
  return gravity_;
}

}  // namespace mowbite_localization
