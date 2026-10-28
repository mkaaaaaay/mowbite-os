// State and models of the 2D filter. Based on SystemModel.hpp, PositionMeasurementModel.hpp,
// SpeedMeasurementModel.hpp and OrientationMeasurementModel2.hpp of xbot_positioning (OpenMower), MIT License,
// Copyright (c) 2022 Clemens Elflein. See LICENSE.xbot_positioning.
#ifndef MOWBITE_LOCALIZATION_MODELS_H
#define MOWBITE_LOCALIZATION_MODELS_H

#include <cmath>
#include <kalman/LinearizedMeasurementModel.hpp>
#include <kalman/LinearizedSystemModel.hpp>

namespace mowbite_localization {

// x/y in m, theta in rad counter-clockwise from x, vx in m/s along the heading, vr in rad/s.
template <typename T>
class State : public Kalman::Vector<T, 5> {
 public:
  KALMAN_VECTOR(State, T, 5)

  static constexpr size_t X = 0;
  static constexpr size_t Y = 1;
  static constexpr size_t THETA = 2;
  static constexpr size_t VX = 3;
  static constexpr size_t VR = 4;

  T theta() const {
    return (*this)[THETA];
  }
  T vx() const {
    return (*this)[VX];
  }
  T vr() const {
    return (*this)[VR];
  }
  T& theta() {
    return (*this)[THETA];
  }
  T& vx() {
    return (*this)[VX];
  }
  T& vr() {
    return (*this)[VR];
  }
};

// Wheel speed and gyro rate drive the prediction.
template <typename T>
class Control : public Kalman::Vector<T, 2> {
 public:
  KALMAN_VECTOR(Control, T, 2)

  static constexpr size_t V = 0;
  static constexpr size_t DTHETA = 1;

  T v() const {
    return (*this)[V];
  }
  T dtheta() const {
    return (*this)[DTHETA];
  }
  T& v() {
    return (*this)[V];
  }
  T& dtheta() {
    return (*this)[DTHETA];
  }
};

// Unicycle: turn by the gyro rate first, then move along the new heading. vx and vr are kept.
template <typename T>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>> {
 public:
  using S = State<T>;
  using C = Control<T>;

  void SetDt(double dt) {
    dt_ = dt;
  }

  S f(const S& x, const C& u) const override {
    S next;
    const T theta = x.theta() + u.dtheta() * dt_;
    next.theta() = theta;
    next.x() = x.x() + std::cos(theta) * u.v() * dt_;
    next.y() = x.y() + std::sin(theta) * u.v() * dt_;
    next.vx() = x.vx();
    next.vr() = x.vr();
    return next;
  }

 protected:
  void updateJacobians(const S& x, const C& u) override {
    this->F.setIdentity();
    const T theta = x.theta() + u.dtheta() * dt_;
    this->F(S::X, S::THETA) = -std::sin(theta) * u.v() * dt_;
    this->F(S::Y, S::THETA) = std::cos(theta) * u.v() * dt_;
    this->W.setIdentity();
  }

 private:
  double dt_ = 0;
};

// Position of the GPS antenna in the map.
template <typename T>
class PositionMeasurement : public Kalman::Vector<T, 2> {
 public:
  KALMAN_VECTOR(PositionMeasurement, T, 2)
};

template <typename T>
class PositionMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, PositionMeasurement<T>> {
 public:
  using S = State<T>;
  using M = PositionMeasurement<T>;

  PositionMeasurementModel() {
    this->H.setIdentity();
    this->V.setIdentity();
  }

  M h(const S& x) const override {
    M m;
    const T c = std::cos(x.theta()), s = std::sin(x.theta());
    m.x() = x.x() + c * antenna_offset_x - s * antenna_offset_y;
    m.y() = x.y() + s * antenna_offset_x + c * antenna_offset_y;
    return m;
  }

  double antenna_offset_x = 0;
  double antenna_offset_y = 0;

 protected:
  // As in the original, without the heading terms of the antenna offset: a large first jump would spin the heading.
  void updateJacobians(const S&) override {
    this->H.setIdentity();
  }
};

// Wheel speed and gyro rate as measurement of vx and vr.
template <typename T>
class SpeedMeasurement : public Kalman::Vector<T, 2> {
 public:
  KALMAN_VECTOR(SpeedMeasurement, T, 2)

  static constexpr size_t VX = 0;
  static constexpr size_t VR = 1;

  T vx() const {
    return (*this)[VX];
  }
  T vr() const {
    return (*this)[VR];
  }
  T& vx() {
    return (*this)[VX];
  }
  T& vr() {
    return (*this)[VR];
  }
};

template <typename T>
class SpeedMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, SpeedMeasurement<T>> {
 public:
  using S = State<T>;
  using M = SpeedMeasurement<T>;

  SpeedMeasurementModel() {
    this->V.setIdentity();
    this->H.setZero();
    this->H(M::VX, S::VX) = 1;
    this->H(M::VR, S::VR) = 1;
  }

  M h(const S& x) const override {
    M m;
    m.vx() = x.vx();
    m.vr() = x.vr();
    return m;
  }
};

// Velocity of the GPS antenna in the map (east, north), which gives the heading while driving.
template <typename T>
class MotionMeasurement : public Kalman::Vector<T, 2> {
 public:
  KALMAN_VECTOR(MotionMeasurement, T, 2)
};

template <typename T>
class MotionMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, MotionMeasurement<T>> {
 public:
  using S = State<T>;
  using M = MotionMeasurement<T>;

  MotionMeasurementModel() {
    this->H.setIdentity();
    this->V.setIdentity();
  }

  M h(const S& x) const override {
    M m;
    const T c = std::cos(x.theta()), s = std::sin(x.theta());
    m.x() = x.vx() * c - s * antenna_offset_x * x.vr() - c * antenna_offset_y * x.vr();
    m.y() = x.vx() * s + c * antenna_offset_x * x.vr() - s * antenna_offset_y * x.vr();
    return m;
  }

  double antenna_offset_x = 0;
  double antenna_offset_y = 0;

 protected:
  // Only the heading column, as in the original: the motion vector corrects the heading, not the speeds.
  void updateJacobians(const S& x) override {
    this->H.setZero();
    const T c = std::cos(x.theta()), s = std::sin(x.theta());
    this->H(0, S::THETA) = -x.vx() * s - antenna_offset_x * x.vr() * c + s * antenna_offset_y * x.vr();
    this->H(1, S::THETA) = x.vx() * c - antenna_offset_x * x.vr() * s - c * antenna_offset_y * x.vr();
  }
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_MODELS_H
