#ifndef MOWBITE_LOCALIZATION_POSE_FILTER_H
#define MOWBITE_LOCALIZATION_POSE_FILTER_H

#include <Eigen/Core>

namespace mowbite_localization {

// x, y (m) and heading (rad, counterclockwise from east) of base_link in the map frame.
using State = Eigen::Vector3d;
using Covariance = Eigen::Matrix3d;
using AntennaJacobian = Eigen::Matrix<double, 2, 3>;

// To (-pi, pi].
double WrapAngle(double angle);

// Where an antenna at lever (m, in the frame of base_link) is for a state, and how that changes with the state.
Eigen::Vector2d AntennaPosition(const State& state, const Eigen::Vector2d& lever);
AntennaJacobian AntennaPositionJacobian(const State& state, const Eigen::Vector2d& lever);

struct FilterNoise {
  // Random walk of the position, m per square root of m driven, and of the heading, rad per square root of s the gyro
  // is integrated.
  double odometry = 0.03;
  double gyro = 0.005;
};

// Extended Kalman filter of the pose of base_link: the distance of the wheels and the turn of the gyro move it, GNSS
// positions of the antenna and headings from its course correct it. Updates use the Joseph form.
class PoseFilter {
 public:
  explicit PoseFilter(const FilterNoise& noise);

  void Reset(const State& state, const Covariance& covariance);
  // Moves distance m along the mean heading of a turn by turn rad; gyro_time s of gyro integration add their noise.
  void Predict(double distance, double turn, double gyro_time);
  // A position of the antenna: innovation is the measured minus the expected position, jacobian the one of the
  // expected position, variance that of each axis. Without correct_heading only the position moves.
  void UpdatePosition(const Eigen::Vector2d& innovation, const AntennaJacobian& jacobian, double variance,
                      bool correct_heading = true);
  // A heading: innovation is the measured minus the expected heading.
  void UpdateHeading(double innovation, double variance);
  // Sets the position and its variance, keeps the heading; the two are no longer correlated.
  void SetPosition(double x, double y, double variance);
  // Sets the heading and its variance, keeps the position; the two are no longer correlated.
  void SetHeading(double heading, double variance);
  // Moves the state by delta without touching the covariance.
  void Shift(const State& delta);

  const State& state() const { return state_; }
  const Covariance& covariance() const { return covariance_; }

 private:
  template <int N>
  void Update(const Eigen::Matrix<double, N, 1>& innovation, const Eigen::Matrix<double, N, 3>& jacobian,
              const Eigen::Matrix<double, N, N>& noise, bool correct_heading = true);

  FilterNoise noise_;
  State state_ = State::Zero();
  Covariance covariance_ = Covariance::Identity();
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_POSE_FILTER_H
