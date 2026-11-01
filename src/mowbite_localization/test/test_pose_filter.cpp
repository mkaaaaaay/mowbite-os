#include <gtest/gtest.h>

#include <Eigen/LU>

#include <cmath>

#include "mowbite_localization/pose_filter.h"

using namespace mowbite_localization;

namespace {

TEST(Angles, WrapToTheHalfOpenRange) {
  EXPECT_NEAR(WrapAngle(3 * M_PI), M_PI, 1e-12);
  EXPECT_NEAR(WrapAngle(-M_PI), M_PI, 1e-12);
  EXPECT_NEAR(WrapAngle(-3 * M_PI / 2), M_PI / 2, 1e-12);
  EXPECT_NEAR(WrapAngle(10.0), 10.0 - 4 * M_PI, 1e-12);
  EXPECT_DOUBLE_EQ(WrapAngle(0.5), 0.5);
}

TEST(Antenna, TurnsWithTheHeading) {
  const Eigen::Vector2d antenna = AntennaPosition(State(1.0, 2.0, M_PI / 2), Eigen::Vector2d(0.3, 0.1));
  // Facing north: 0.3 m ahead is north, 0.1 m to the left is west.
  EXPECT_NEAR(antenna(0), 0.9, 1e-12);
  EXPECT_NEAR(antenna(1), 2.3, 1e-12);
}

TEST(Antenna, JacobianMatchesTheNumericDerivative) {
  const State state(1.0, -2.0, 0.7);
  const Eigen::Vector2d lever(0.25, -0.08);
  const AntennaJacobian jacobian = AntennaPositionJacobian(state, lever);
  for (int i = 0; i < 3; ++i) {
    State step = State::Zero();
    step(i) = 1e-6;
    const Eigen::Vector2d numeric =
        (AntennaPosition(state + step, lever) - AntennaPosition(state - step, lever)) / 2e-6;
    EXPECT_NEAR(jacobian(0, i), numeric(0), 1e-8);
    EXPECT_NEAR(jacobian(1, i), numeric(1), 1e-8);
  }
}

TEST(Predict, ClosesAQuarterCircleExactly) {
  PoseFilter filter({});
  filter.Reset(State::Zero(), Covariance::Identity() * 1e-4);
  // 100 steps of a quarter circle of radius 1 m.
  const double turn = M_PI / 2 / 100;
  for (int i = 0; i < 100; ++i) filter.Predict(2.0 * std::sin(turn / 2), turn, 0.02);
  EXPECT_NEAR(filter.state()(0), 1.0, 1e-9);
  EXPECT_NEAR(filter.state()(1), 1.0, 1e-9);
  EXPECT_NEAR(filter.state()(2), M_PI / 2, 1e-12);
}

TEST(Predict, NoiseGrowsWithTheWayAndTheGyroTime) {
  FilterNoise noise;
  noise.odometry = 0.1;
  noise.gyro = 0.01;
  PoseFilter filter(noise);
  filter.Reset(State::Zero(), Covariance::Zero());
  filter.Predict(-2.0, 0.0, 0.0);
  EXPECT_NEAR(filter.covariance()(0, 0), 0.01 * 2.0, 1e-12);
  EXPECT_NEAR(filter.covariance()(2, 2), 0.0, 1e-12);
  filter.Predict(0.0, 0.0, 4.0);
  EXPECT_NEAR(filter.covariance()(2, 2), 1e-4 * 4.0, 1e-12);
  // The heading error spreads across the way driven next.
  filter.Predict(1.0, 0.0, 0.0);
  EXPECT_NEAR(filter.covariance()(1, 1), 0.01 * 3.0 + 4e-4, 1e-12);
  EXPECT_NEAR(filter.covariance()(1, 2), 4e-4, 1e-12);
}

TEST(Update, PositionPullsTowardsTheFixByTheVariances) {
  PoseFilter filter({});
  filter.Reset(State(0.0, 0.0, 0.0), Covariance::Identity() * 0.01);
  const Eigen::Vector2d lever(0.0, 0.0);
  filter.UpdatePosition(Eigen::Vector2d(0.2, 0.0), AntennaPositionJacobian(filter.state(), lever), 0.01);
  // Equal variances: half way, and half the variance.
  EXPECT_NEAR(filter.state()(0), 0.1, 1e-12);
  EXPECT_NEAR(filter.covariance()(0, 0), 0.005, 1e-12);
  EXPECT_NEAR(filter.state()(2), 0.0, 1e-12);
}

TEST(Update, KeepsACovariance) {
  PoseFilter filter({});
  Covariance covariance = Covariance::Identity() * 0.01;
  covariance(0, 2) = covariance(2, 0) = 0.005;
  filter.Reset(State(0.0, 0.0, 0.3), covariance);
  const Eigen::Vector2d lever(0.3, 0.0);
  for (int i = 0; i < 100; ++i) {
    filter.UpdatePosition(Eigen::Vector2d(0.01, -0.01), AntennaPositionJacobian(filter.state(), lever), 0.0004);
    filter.UpdateHeading(0.01, 0.004);
    filter.Predict(0.006, 0.001, 0.02);
  }
  // Symmetric and positive definite.
  EXPECT_TRUE(filter.covariance().isApprox(filter.covariance().transpose()));
  EXPECT_GT(filter.covariance().determinant(), 0.0);
  EXPECT_GT(filter.covariance().diagonal().minCoeff(), 0.0);
}

TEST(Update, TheLeverMakesTheHeadingObservable) {
  PoseFilter filter({});
  filter.Reset(State(0.0, 0.0, 0.0), Eigen::Vector3d(1e-6, 1e-6, 0.01).asDiagonal());
  // The antenna 0.5 m ahead shows up 0.05 m to the left: the body turned, not moved.
  const Eigen::Vector2d lever(0.5, 0.0);
  filter.UpdatePosition(Eigen::Vector2d(0.0, 0.05), AntennaPositionJacobian(filter.state(), lever), 1e-6);
  EXPECT_NEAR(filter.state()(2), 0.1, 0.01);
  EXPECT_NEAR(filter.state()(1), 0.0, 0.002);
}

TEST(Update, HeadingWrapsAcrossPi) {
  PoseFilter filter({});
  filter.Reset(State(0.0, 0.0, M_PI - 0.05), Covariance::Identity() * 0.01);
  filter.UpdateHeading(0.1, 1e-9);
  EXPECT_NEAR(filter.state()(2), -M_PI + 0.05, 1e-6);
}

TEST(Set, PositionAndHeadingDropTheirCorrelation) {
  PoseFilter filter({});
  Covariance covariance = Covariance::Identity() * 0.01;
  covariance(0, 2) = covariance(2, 0) = 0.005;
  filter.Reset(State(1.0, 2.0, 0.3), covariance);
  filter.SetPosition(3.0, 4.0, 0.0004);
  EXPECT_DOUBLE_EQ(filter.state()(0), 3.0);
  EXPECT_DOUBLE_EQ(filter.state()(2), 0.3);
  EXPECT_DOUBLE_EQ(filter.covariance()(0, 0), 0.0004);
  EXPECT_DOUBLE_EQ(filter.covariance()(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(filter.covariance()(2, 2), 0.01);
  filter.SetHeading(4.0, 0.001);
  EXPECT_NEAR(filter.state()(2), 4.0 - 2 * M_PI, 1e-12);
  EXPECT_DOUBLE_EQ(filter.covariance()(2, 2), 0.001);
}

}  // namespace
