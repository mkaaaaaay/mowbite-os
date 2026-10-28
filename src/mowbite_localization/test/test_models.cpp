#include <gtest/gtest.h>

#include <cmath>

#include "mowbite_localization/models.h"

namespace mowbite_localization {
namespace {

using S = State<double>;
using C = Control<double>;

struct TestSystem : SystemModel<double> {
  using SystemModel<double>::F;
  using SystemModel<double>::updateJacobians;
};

struct TestMotion : MotionMeasurementModel<double> {
  using MotionMeasurementModel<double>::H;
  using MotionMeasurementModel<double>::updateJacobians;
};

S MakeState(double x, double y, double theta, double vx, double vr) {
  S s;
  s.x() = x;
  s.y() = y;
  s.theta() = theta;
  s.vx() = vx;
  s.vr() = vr;
  return s;
}

C MakeControl(double v, double dtheta) {
  C u;
  u.v() = v;
  u.dtheta() = dtheta;
  return u;
}

TEST(SystemModel, MovesAlongTheHeading) {
  SystemModel<double> sys;
  sys.SetDt(0.1);
  S next = sys.f(MakeState(1, 2, 0, 0, 0), MakeControl(0.3, 0));
  EXPECT_NEAR(next.x(), 1.03, 1e-12);
  EXPECT_NEAR(next.y(), 2.0, 1e-12);
  EXPECT_NEAR(next.theta(), 0.0, 1e-12);
}

TEST(SystemModel, TurnsBeforeMoving) {
  SystemModel<double> sys;
  sys.SetDt(1.0);
  S next = sys.f(MakeState(0, 0, 0, 0.4, 0.2), MakeControl(1.0, M_PI / 2));
  EXPECT_NEAR(next.theta(), M_PI / 2, 1e-12);
  EXPECT_NEAR(next.x(), 0.0, 1e-12);
  EXPECT_NEAR(next.y(), 1.0, 1e-12);
  // Speeds in the state are kept, only the measurement changes them.
  EXPECT_DOUBLE_EQ(next.vx(), 0.4);
  EXPECT_DOUBLE_EQ(next.vr(), 0.2);
}

TEST(SystemModel, JacobianMatchesNumericDerivative) {
  TestSystem sys;
  sys.SetDt(0.02);
  const S x = MakeState(3, -1, 0.7, 0.25, 0.1);
  const C u = MakeControl(0.3, -0.2);
  sys.updateJacobians(x, u);
  const double eps = 1e-6;
  for (int col = 0; col < 5; ++col) {
    S plus = x, minus = x;
    plus[col] += eps;
    minus[col] -= eps;
    const S d = (sys.f(plus, u) - sys.f(minus, u)) / (2 * eps);
    for (int row = 0; row < 5; ++row) EXPECT_NEAR(sys.F(row, col), d[row], 1e-6) << row << "," << col;
  }
}

TEST(PositionMeasurementModel, AddsTheRotatedAntennaOffset) {
  PositionMeasurementModel<double> pm;
  pm.antenna_offset_x = 0.3;
  pm.antenna_offset_y = 0.1;
  auto m = pm.h(MakeState(5, 6, M_PI / 2, 0, 0));
  EXPECT_NEAR(m.x(), 4.9, 1e-12);
  EXPECT_NEAR(m.y(), 6.3, 1e-12);
  m = pm.h(MakeState(5, 6, 0, 0, 0));
  EXPECT_NEAR(m.x(), 5.3, 1e-12);
  EXPECT_NEAR(m.y(), 6.1, 1e-12);
}

TEST(SpeedMeasurementModel, MeasuresSpeedAndRate) {
  SpeedMeasurementModel<double> sm;
  auto m = sm.h(MakeState(1, 2, 3, 0.25, -0.1));
  EXPECT_DOUBLE_EQ(m.vx(), 0.25);
  EXPECT_DOUBLE_EQ(m.vr(), -0.1);
}

TEST(MotionMeasurementModel, ForwardSpeedAlongTheHeading) {
  MotionMeasurementModel<double> mm;
  auto m = mm.h(MakeState(0, 0, M_PI / 2, 0.2, 0));
  EXPECT_NEAR(m.x(), 0.0, 1e-12);
  EXPECT_NEAR(m.y(), 0.2, 1e-12);
}

TEST(MotionMeasurementModel, AntennaAheadMovesSidewaysWhenTurning) {
  MotionMeasurementModel<double> mm;
  mm.antenna_offset_x = 0.3;
  auto m = mm.h(MakeState(0, 0, 0, 0, 1.0));
  EXPECT_NEAR(m.x(), 0.0, 1e-12);
  EXPECT_NEAR(m.y(), 0.3, 1e-12);
}

TEST(MotionMeasurementModel, HeadingColumnMatchesNumericDerivative) {
  TestMotion mm;
  mm.antenna_offset_x = 0.25;
  mm.antenna_offset_y = -0.05;
  const S x = MakeState(1, 1, -2.1, 0.3, 0.4);
  mm.updateJacobians(x);
  const double eps = 1e-6;
  S plus = x, minus = x;
  plus.theta() += eps;
  minus.theta() -= eps;
  const MotionMeasurement<double> d = (mm.h(plus) - mm.h(minus)) / (2 * eps);
  EXPECT_NEAR(mm.H(0, S::THETA), d[0], 1e-6);
  EXPECT_NEAR(mm.H(1, S::THETA), d[1], 1e-6);
  // Speeds are not corrected by the motion vector.
  EXPECT_EQ(mm.H(0, S::VX), 0.0);
  EXPECT_EQ(mm.H(1, S::VR), 0.0);
}

}  // namespace
}  // namespace mowbite_localization
