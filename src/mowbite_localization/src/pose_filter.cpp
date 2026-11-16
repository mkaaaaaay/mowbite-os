#include "mowbite_localization/pose_filter.h"

#include <Eigen/LU>
#include <cmath>

namespace mowbite_localization {

double WrapAngle(double angle) {
  const double wrapped = std::remainder(angle, 2.0 * M_PI);
  return wrapped <= -M_PI ? wrapped + 2.0 * M_PI : wrapped;
}

Eigen::Vector2d AntennaPosition(const State& state, const Eigen::Vector2d& lever) {
  const double c = std::cos(state(2));
  const double s = std::sin(state(2));
  return {state(0) + c * lever(0) - s * lever(1), state(1) + s * lever(0) + c * lever(1)};
}

AntennaJacobian AntennaPositionJacobian(const State& state, const Eigen::Vector2d& lever) {
  const double c = std::cos(state(2));
  const double s = std::sin(state(2));
  AntennaJacobian jacobian;
  jacobian << 1.0, 0.0, -s * lever(0) - c * lever(1),
              0.0, 1.0, c * lever(0) - s * lever(1);
  return jacobian;
}

PoseFilter::PoseFilter(const FilterNoise& noise) : noise_(noise) {
}

void PoseFilter::Reset(const State& state, const Covariance& covariance) {
  state_ = state;
  state_(2) = WrapAngle(state_(2));
  covariance_ = covariance;
}

void PoseFilter::Predict(double distance, double turn, double gyro_time, double slip) {
  const double mid = state_(2) + 0.5 * turn;
  Covariance transition = Covariance::Identity();
  transition(0, 2) = -distance * std::sin(mid);
  transition(1, 2) = distance * std::cos(mid);
  state_(0) += distance * std::cos(mid);
  state_(1) += distance * std::sin(mid);
  state_(2) = WrapAngle(state_(2) + turn);
  Covariance process = Covariance::Zero();
  process(0, 0) = process(1, 1) = noise_.odometry * noise_.odometry * std::abs(distance);
  process(2, 2) = noise_.gyro * noise_.gyro * gyro_time;
  covariance_ = transition * covariance_ * transition.transpose() + process;
  if (slip > 0.0) {
    for (int axis : {0, 1}) {
      const double sigma = std::sqrt(std::max(0.0, covariance_(axis, axis))) + slip;
      covariance_(axis, axis) = sigma * sigma;
    }
  }
}

template <int N>
void PoseFilter::Update(const Eigen::Matrix<double, N, 1>& innovation, const Eigen::Matrix<double, N, 3>& jacobian,
                        const Eigen::Matrix<double, N, N>& noise, bool correct_heading) {
  const Eigen::Matrix<double, N, N> s = jacobian * covariance_ * jacobian.transpose() + noise;
  Eigen::Matrix<double, 3, N> gain = covariance_ * jacobian.transpose() * s.inverse();
  if (!correct_heading) gain.row(2).setZero();
  const Covariance keep = Covariance::Identity() - gain * jacobian;
  covariance_ = keep * covariance_ * keep.transpose() + gain * noise * gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  state_ += gain * innovation;
  state_(2) = WrapAngle(state_(2));
}

void PoseFilter::UpdatePosition(const Eigen::Vector2d& innovation, const AntennaJacobian& jacobian, double variance,
                                bool correct_heading) {
  Update<2>(innovation, jacobian, Eigen::Matrix2d::Identity() * variance, correct_heading);
}

void PoseFilter::UpdatePosition(const Eigen::Vector2d& innovation, const AntennaJacobian& jacobian,
                                const Eigen::Matrix2d& noise, bool correct_heading) {
  Update<2>(innovation, jacobian, noise, correct_heading);
}

void PoseFilter::UpdateHeading(double innovation, double variance) {
  Update<1>(Eigen::Matrix<double, 1, 1>(innovation), Eigen::Matrix<double, 1, 3>(0.0, 0.0, 1.0),
            Eigen::Matrix<double, 1, 1>(variance));
}

void PoseFilter::SetPosition(double x, double y, double variance) {
  state_(0) = x;
  state_(1) = y;
  covariance_.topLeftCorner<2, 2>() = Eigen::Matrix2d::Identity() * variance;
  covariance_.block<2, 1>(0, 2).setZero();
  covariance_.block<1, 2>(2, 0).setZero();
}

void PoseFilter::SetHeading(double heading, double variance) {
  state_(2) = WrapAngle(heading);
  covariance_(2, 2) = variance;
  covariance_.block<2, 1>(0, 2).setZero();
  covariance_.block<1, 2>(2, 0).setZero();
}

void PoseFilter::Shift(const State& delta) {
  state_ += delta;
  state_(2) = WrapAngle(state_(2));
}

}  // namespace mowbite_localization
