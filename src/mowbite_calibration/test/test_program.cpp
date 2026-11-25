#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "mowbite_calibration/program.h"

using namespace mowbite_calibration;

namespace {

constexpr double kDeg = M_PI / 180.0;

double Wrap(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

// A mower on a flat lawn that follows the commands of the program like a driver: the wheels (track 0.4 m) with a dead
// zone, a watchdog of 0.5 s, and faults on demand. It reports an IMU and the pose at 50 Hz, the wheels at 20 Hz and
// RTK fixes of the antenna at 5 Hz.
struct Mower {
  double x = 2;
  double y = 3;
  double heading = 0;
  double v = 0;
  double w = 0;
  double antenna_x = 0.10;
  double antenna_y = 0.02;
  double track = 0.4;
  double dead_zone = 0;
  double v_scale = 1;
  double w_scale = 1;
  // Sideways (m/s, towards +y of the map) while it turns: the pivot wanders.
  double slip = 0;
  double gyro_sign = 1;
  double gyro_bias = 0;
  double tilt = 0;
  // The IMU mounted nose up (rad, gravity reads -sin on x on level ground), and the ground sloping up towards
  // slope_heading (map frame) by slope (rad).
  double mount_pitch = 0;
  double slope = 0;
  double slope_heading = 0;
  double gravity = 9.80665;
  bool fixed = true;
  double accuracy = 0.012;
  double gnss_period = 0.2;
  double imu_period = 0.02;
  bool imu_on = true;
  bool wheels_on = true;
  bool pose_on = true;
  bool gnss_on = true;
  bool heading_valid = true;
  double heading_error = 0;
  double pose_accuracy = 0.015;
  // The pose stays where the mower was when it froze.
  bool pose_frozen = false;
  double pose_x = 0;
  double pose_y = 0;
  double cmd_v = 0;
  double cmd_w = 0;
  double cmd_s = -1e9;
  // Truth while it runs: the least forward speed asked for, and how far it turned.
  double min_cmd_v = std::numeric_limits<double>::infinity();
  double turned = 0;

  void Command(const Twist2& t, double now) {
    cmd_v = t.linear;
    cmd_w = t.angular;
    cmd_s = now;
    min_cmd_v = std::min(min_cmd_v, t.linear);
  }

  void Move(double dt, double now) {
    double cv = cmd_v;
    double cw = cmd_w;
    if (now - cmd_s > 0.5) cv = cw = 0;
    double left = cv - cw * track / 2;
    double right = cv + cw * track / 2;
    if (std::abs(left) < dead_zone) left = 0;
    if (std::abs(right) < dead_zone) right = 0;
    v = (left + right) / 2 * v_scale;
    w = (right - left) / track * w_scale;
    heading = Wrap(heading + w * dt);
    turned += w * dt;
    x += v * std::cos(heading) * dt;
    y += v * std::sin(heading) * dt + (std::abs(w) > 0.1 ? slip * dt : 0);
  }

  double AntennaX() const {
    return x + std::cos(heading) * antenna_x - std::sin(heading) * antenna_y;
  }
  double AntennaY() const {
    return y + std::sin(heading) * antenna_x + std::cos(heading) * antenna_y;
  }
};

class ProgramTest : public ::testing::Test {
 protected:
  ProgramTest() : program_(config_) {
  }

  // A new program for the next case of a test.
  void Rebuild() {
    program_ = Program(config_);
    progress_ = 0;
    edges_.clear();
  }

  // Runs the world in steps of 10 ms until `until` or until the program stopped, if it ran.
  void Run(double until, const std::function<void()>& each = {}) {
    while (now_ < until - 1e-9) {
      now_ += 0.01;
      step_++;
      mower_.Move(0.01, now_);
      if (step_ % static_cast<long>(std::lround(mower_.imu_period / 0.01)) == 0 && mower_.imu_on) {
        ImuSample imu;
        imu.gz = mower_.gyro_sign * mower_.w + mower_.gyro_bias;
        // Up of the ground in the frame of the mower, then turned into the IMU's frame by its mounting.
        const double up_ahead = -std::sin(mower_.slope) * std::cos(mower_.heading - mower_.slope_heading);
        const double up_left = std::sin(mower_.slope) * std::sin(mower_.heading - mower_.slope_heading);
        const double up_z = std::cos(mower_.slope);
        const double m = -mower_.mount_pitch;
        const double gx = up_ahead * std::cos(m) + up_z * std::sin(m);
        const double gz = -up_ahead * std::sin(m) + up_z * std::cos(m);
        imu.ax = mower_.gravity * (gx * std::cos(mower_.tilt) + gz * std::sin(mower_.tilt));
        imu.ay = mower_.gravity * up_left;
        imu.az = mower_.gravity * (-gx * std::sin(mower_.tilt) + gz * std::cos(mower_.tilt));
        program_.OnImu(imu, now_);
      }
      if (!mower_.pose_frozen) {
        mower_.pose_x = mower_.x;
        mower_.pose_y = mower_.y;
      }
      if (step_ % 2 == 0 && mower_.pose_on) {
        program_.OnPose({mower_.pose_x, mower_.pose_y, Wrap(mower_.heading + mower_.heading_error),
                         mower_.heading_valid, mower_.pose_accuracy},
                        now_);
      }
      if (step_ % static_cast<long>(std::lround(mower_.gnss_period / 0.01)) == 0 && mower_.gnss_on) {
        program_.OnGnss({mower_.AntennaX(), mower_.AntennaY(), mower_.accuracy, mower_.fixed}, now_);
      }
      if (step_ % 5 == 0) {
        if (mower_.wheels_on) program_.OnWheels(mower_.v, mower_.w, now_);
        const bool was_running = program_.Running();
        if (auto cmd = program_.Tick(now_)) {
          mower_.Command(*cmd, now_);
          commands_++;
          last_cmd_ = *cmd;
        }
        if (was_running && program_.Running()) {
          const double p = program_.Progress(now_);
          EXPECT_GE(p, progress_ - 1e-9);
          progress_ = p;
        }
        auto edges = program_.TakeEdges();
        edges_.insert(edges_.end(), edges.begin(), edges.end());
        if (each) each();
        if (was_running && !program_.Running()) {
          ended_s_ = now_;
          return;
        }
      }
    }
  }

  void StartOn(const Lane& lane) {
    Run(now_ + 2.0);
    ASSERT_TRUE(program_.Readiness(now_).empty()) << program_.Readiness(now_)[0].message;
    start_s_ = now_;
    ASSERT_TRUE(program_.Start(lane, now_));
  }

  // The reason the program ended with, after running at most `seconds` more.
  std::string EndsWith(double seconds, const std::function<void()>& each = {}) {
    Run(now_ + seconds, each);
    if (program_.Running()) return "still running";
    return program_.outcome() ? program_.outcome()->reason : "no outcome";
  }

  void RunUntilPhase(Phase phase, int round, double limit = 400) {
    const double until = now_ + limit;
    while (now_ < until && program_.Running() && !(program_.phase() == phase && program_.round() == round)) {
      Run(now_ + 0.05);
    }
    ASSERT_EQ(program_.phase(), phase);
    ASSERT_EQ(program_.round(), round);
  }

  std::vector<std::string> EdgeNames(bool starts) const {
    std::vector<std::string> names;
    for (const auto& e : edges_) {
      if (e.start == starts) names.push_back(std::string(PhaseName(e.phase)) + std::to_string(e.round));
    }
    return names;
  }

  ProgramConfig config_;
  Program program_;
  Mower mower_;
  double now_ = 100.0;
  long step_ = 0;
  double start_s_ = 0;
  double ended_s_ = 0;
  double progress_ = 0;
  int commands_ = 0;
  Twist2 last_cmd_;
  std::vector<PhaseEdge> edges_;
};

}  // namespace

TEST_F(ProgramTest, DrivesTheProgramAndEndsWhereItBegan) {
  StartOn(Lane{2, 3, 0.0, 3.0});
  const double expected_s = program_.ExpectedSeconds(Lane{2, 3, 0.0, 3.0});
  std::vector<std::pair<double, double>> line_ends;
  std::vector<double> turns;
  double turn_start = 0;
  Phase last = Phase::kStart;
  Run(now_ + 400, [&] {
    const Phase p = program_.phase();
    if (last == Phase::kLine && p != Phase::kLine) line_ends.emplace_back(mower_.x, mower_.y);
    if (p == Phase::kSpinLeft || p == Phase::kSpinRight) {
      if (last != p) turn_start = mower_.turned;
    } else if (last == Phase::kSpinLeft || last == Phase::kSpinRight) {
      turns.push_back(mower_.turned - turn_start);
    }
    last = p;
  });
  ASSERT_FALSE(program_.Running());
  ASSERT_TRUE(program_.outcome());
  EXPECT_EQ(program_.outcome()->reason, "DONE") << program_.outcome()->message;
  EXPECT_DOUBLE_EQ(program_.Progress(now_), 1.0);
  // About 4 min without ALIGN, never backwards, at most the speeds of the program.
  EXPECT_GT(ended_s_ - start_s_, 220);
  EXPECT_LT(ended_s_ - start_s_, 300);
  // The estimate of calibration/status within a tenth.
  EXPECT_NEAR(ended_s_ - start_s_, expected_s, 0.1 * expected_s);
  EXPECT_GE(mower_.min_cmd_v, 0.0);
  // Out to the far end and back, twice; each turn 540 deg, left at the far end and right at the start.
  ASSERT_EQ(line_ends.size(), 4u);
  for (size_t i = 0; i < 4; i++) {
    EXPECT_NEAR(line_ends[i].first, i % 2 == 0 ? 5.0 : 2.0, 0.05) << i;
    EXPECT_NEAR(line_ends[i].second, 3.0, 0.03) << i;
  }
  ASSERT_EQ(turns.size(), 4u);
  for (size_t i = 0; i < 4; i++) EXPECT_NEAR(turns[i], (i % 2 == 0 ? 1 : -1) * 3 * M_PI, 3 * kDeg) << i;
  EXPECT_NEAR(mower_.x, 2.0, 0.05);
  EXPECT_NEAR(mower_.y, 3.0, 0.05);
  EXPECT_LT(std::abs(Wrap(mower_.heading)), 5 * kDeg);
  // The phases as the recording marks them, without ALIGN when the mower faces the lane already.
  const std::vector<std::string> expected = {"stand0", "line1", "stand1", "spin_left1", "stand1",
                                             "line2",  "stand2", "spin_right2", "stand2", "line3",
                                             "stand3", "spin_left3", "stand3", "line4", "stand4",
                                             "spin_right4", "stand4"};
  EXPECT_EQ(EdgeNames(true), expected);
  EXPECT_EQ(EdgeNames(false), expected);
  EXPECT_EQ(program_.phase(), Phase::kEnd);
}

TEST_F(ProgramTest, AlignsToTheLaneFirst) {
  mower_.heading = 100 * kDeg;
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kStand, 0, 30);
  EXPECT_LT(std::abs(Wrap(mower_.heading)), 5 * kDeg);
  ASSERT_FALSE(edges_.empty());
  EXPECT_EQ(edges_.front().phase, Phase::kAlign);
  // Turned right, the shorter way.
  EXPECT_LT(mower_.turned, -90 * kDeg);
  EXPECT_EQ(EndsWith(400), "DONE");
}

TEST_F(ProgramTest, NoAlignWithinFiveDegrees) {
  mower_.heading = 4 * kDeg;
  StartOn(Lane{2, 3, 0.0, 3.0});
  Run(now_ + 1.0);
  ASSERT_FALSE(edges_.empty());
  EXPECT_EQ(edges_.front().phase, Phase::kStand);
  EXPECT_EQ(program_.round(), 0);
}

TEST_F(ProgramTest, GyroOffsetFromTheStandKeepsTheTurns) {
  // 1.1 deg/s: 36 deg over a turn if it were not taken out.
  mower_.gyro_bias = 0.02;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400), "DONE");
  EXPECT_LT(std::abs(Wrap(mower_.heading)), 5 * kDeg);
}

TEST_F(ProgramTest, StartOnlyOnceAndWaitsForTheSensors) {
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_FALSE(program_.Start(Lane{2, 3, 0.0, 3.0}, now_));
  // Without RTK the start gives up after start_timeout.
  Program fresh(config_);
  fresh.OnImu({0, 0, 0, 0, 0, 9.81}, 1.0);
  fresh.OnWheels(0, 0, 1.0);
  fresh.OnPose({2, 3, 0, true, 0.01}, 1.0);
  ASSERT_TRUE(fresh.Start(Lane{2, 3, 0.0, 3.0}, 1.0));
  EXPECT_FALSE(fresh.Tick(1.05));
  EXPECT_TRUE(fresh.Running());
  for (double t = 1.1; t < 4.2; t += 0.05) {
    fresh.OnImu({0, 0, 0, 0, 0, 9.81}, t);
    fresh.OnWheels(0, 0, t);
    fresh.OnPose({2, 3, 0, true, 0.01}, t);
    fresh.Tick(t);
  }
  EXPECT_FALSE(fresh.Running());
  ASSERT_TRUE(fresh.outcome());
  EXPECT_EQ(fresh.outcome()->reason, "NO_RTK");
}

TEST_F(ProgramTest, StoppedByTheApp) {
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kLine, 1);
  Run(now_ + 2.0);
  const auto zero = program_.Stop("Stopped by calibration.abort", now_);
  ASSERT_TRUE(zero);
  EXPECT_EQ(zero->linear, 0.0);
  EXPECT_EQ(zero->angular, 0.0);
  EXPECT_FALSE(program_.Running());
  EXPECT_EQ(program_.outcome()->reason, "STOPPED");
  EXPECT_FALSE(program_.Tick(now_ + 0.05));
  EXPECT_FALSE(program_.Stop("again", now_ + 0.1));
  // The line ends in the edges too.
  const auto edges = program_.TakeEdges();
  edges_.insert(edges_.end(), edges.begin(), edges.end());
  EXPECT_EQ(EdgeNames(true).size(), EdgeNames(false).size());
  EXPECT_EQ(edges_.back().phase, Phase::kLine);
}

TEST_F(ProgramTest, EmergencyStopsWithOneZero) {
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kLine, 1);
  Run(now_ + 3.0);
  program_.OnEmergency(true, now_);
  const int before = commands_;
  EXPECT_EQ(EndsWith(0.1), "EMERGENCY");
  EXPECT_EQ(commands_, before + 1);
  EXPECT_EQ(last_cmd_.linear, 0.0);
  EXPECT_EQ(last_cmd_.angular, 0.0);
}

TEST_F(ProgramTest, NoRtkForOneSecond) {
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kLine, 1);
  mower_.fixed = false;
  const double lost = now_;
  EXPECT_EQ(EndsWith(2.0), "NO_RTK");
  EXPECT_GT(ended_s_ - lost, 0.8);
  EXPECT_LT(ended_s_ - lost, 1.3);
  // Fixed, but not accurate enough, counts the same.
  Rebuild();
  mower_ = Mower();
  StartOn(Lane{2, 3, 0.0, 3.0});
  mower_.accuracy = 0.05;
  EXPECT_EQ(EndsWith(2.0), "NO_RTK");
}

TEST_F(ProgramTest, PoseLostOrWithoutHeading) {
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kLine, 1);
  mower_.pose_on = false;
  EXPECT_EQ(EndsWith(1.0), "POSE_LOST");
  Rebuild();
  mower_ = Mower();
  StartOn(Lane{2, 3, 0.0, 3.0});
  Run(now_ + 12.0);
  mower_.heading_valid = false;
  EXPECT_EQ(EndsWith(0.2), "POSE_LOST");
}

TEST_F(ProgramTest, APoseThatIsNotAccurateOrJumps) {
  // At the start it waits for an accurate pose, then gives up.
  Run(now_ + 2.0);
  mower_.pose_accuracy = 0.3;
  Run(now_ + 0.5);
  ASSERT_TRUE(program_.Start(Lane{2, 3, 0.0, 3.0}, now_));
  EXPECT_EQ(EndsWith(5.0), "POSITION_ACCURACY");
  EXPECT_EQ(program_.round(), 0);
  // On a line once it lasts hold_time.
  Rebuild();
  mower_ = Mower();
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kLine, 1);
  mower_.pose_accuracy = 0.3;
  const double bad = now_;
  EXPECT_EQ(EndsWith(2.0), "POSITION_ACCURACY");
  EXPECT_GT(ended_s_ - bad, 0.4);
  // A jump of the localization ends it at once.
  Rebuild();
  mower_ = Mower();
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kLine, 1);
  program_.OnPositionJump(0.4, now_);
  EXPECT_EQ(EndsWith(0.1), "POSITION_JUMP");
  // Not while no drive runs.
  Rebuild();
  mower_ = Mower();
  program_.OnPositionJump(0.4, now_);
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(5.0), "still running");
}

TEST_F(ProgramTest, ALineEndsByTheAntennaWhenThePoseStands) {
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kLine, 1);
  mower_.pose_frozen = true;
  EXPECT_EQ(EndsWith(40.0), "LEFT_LANE");
  EXPECT_NE(program_.outcome()->message.find("along"), std::string::npos);
  EXPECT_LT(mower_.x, 2.0 + 3.0 + 0.3 + 0.15);
}

TEST_F(ProgramTest, NotStandingInAStand) {
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kStand, 1);
  // The zero got lost: the mower rolls on.
  EXPECT_EQ(EndsWith(5.0, [&] { mower_.Command({0.1, 0.0}, now_); }), "NOT_STANDING");
  Rebuild();
  mower_ = Mower();
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kStand, 1);
  EXPECT_EQ(EndsWith(5.0, [&] { mower_.Command({0.0, 0.3}, now_); }), "NOT_STANDING");
}

TEST_F(ProgramTest, AGyroOffsetBeyondItsLimit) {
  mower_.gyro_bias = 0.08;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(30.0), "GYRO_OFFSET");
  EXPECT_EQ(program_.round(), 0);
  EXPECT_EQ(edges_.back().phase, Phase::kStand);
  EXPECT_FALSE(edges_.back().start);
}

TEST_F(ProgramTest, SensorMissing) {
  StartOn(Lane{2, 3, 0.0, 3.0});
  Run(now_ + 5.0);
  mower_.imu_on = false;
  EXPECT_EQ(EndsWith(1.0), "SENSOR_MISSING");
  EXPECT_NE(program_.outcome()->message.find("IMU"), std::string::npos);
  Rebuild();
  mower_ = Mower();
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kLine, 1);
  mower_.wheels_on = false;
  EXPECT_EQ(EndsWith(1.0), "SENSOR_MISSING");
  EXPECT_NE(program_.outcome()->message.find("wheel"), std::string::npos);
}

TEST_F(ProgramTest, LeftLaneOnALine) {
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kLine, 1);
  // The localization is 25 deg off: following it, the mower leaves the line.
  mower_.heading_error = 25 * kDeg;
  EXPECT_EQ(EndsWith(30.0), "LEFT_LANE");
  EXPECT_EQ(program_.phase(), Phase::kEnd);
  ASSERT_FALSE(edges_.empty());
  EXPECT_EQ(edges_.back().phase, Phase::kLine);
  EXPECT_FALSE(edges_.back().start);
}

TEST_F(ProgramTest, LeftLaneWhenThePivotWanders) {
  mower_.slip = 0.05;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400), "LEFT_LANE");
  EXPECT_NE(program_.outcome()->message.find("pivot"), std::string::npos);
  EXPECT_EQ(edges_.back().phase, Phase::kSpinLeft);
}

TEST_F(ProgramTest, NotTurningInTheDeadZone) {
  // 0.3 rad/s on the spot is 0.06 m/s per wheel: below a dead zone of 0.07 nothing turns.
  mower_.dead_zone = 0.07;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400), "NOT_TURNING");
  EXPECT_EQ(edges_.back().phase, Phase::kSpinLeft);
  // And turning at half the rate.
  Rebuild();
  mower_ = Mower();
  mower_.w_scale = 0.5;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400), "NOT_TURNING");
}

TEST_F(ProgramTest, TurningInJerksCountsByItsAverage) {
  // Half a second at 0.6 of the rate and half a second at 1.2, as wheels near their dead band do: 0.9 on average.
  StartOn(Lane{2, 3, 0.0, 3.0});
  const std::string end = EndsWith(400, [this] { mower_.w_scale = std::fmod(now_, 1.0) < 0.5 ? 0.6 : 1.2; });
  EXPECT_EQ(end, "DONE") << (program_.outcome() ? program_.outcome()->message : "");
}

TEST_F(ProgramTest, JerksBelowTheAverageStillEndIt) {
  // 0.2 and 1.0 of the rate in turns: 0.6 on average, below turn_min_ratio.
  StartOn(Lane{2, 3, 0.0, 3.0});
  const std::string end = EndsWith(400, [this] { mower_.w_scale = std::fmod(now_, 1.0) < 0.5 ? 0.2 : 1.0; });
  EXPECT_EQ(end, "NOT_TURNING");
  double rate = -1;
  for (const auto& [name, value] : program_.outcome()->numbers) {
    if (name == "rate") rate = value;
  }
  EXPECT_NEAR(rate, 0.6 * 0.3, 0.03);
}

TEST_F(ProgramTest, AFasterSpinRateTurnsAtIt) {
  config_.spin_rate = 0.5;
  Rebuild();
  double fastest = 0;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400, [&] { fastest = std::max(fastest, std::abs(mower_.cmd_w)); }), "DONE");
  EXPECT_NEAR(fastest, 0.5, 1e-9);
  // The driver's limit stays above it.
  Rebuild();
  program_.SetSpeedLimits(0.3, 0.3);
  fastest = 0;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400, [&] { fastest = std::max(fastest, std::abs(mower_.cmd_w)); }), "DONE");
  EXPECT_NEAR(fastest, 0.3, 1e-9);
}

TEST_F(ProgramTest, NotMovingOnALine) {
  mower_.v_scale = 0.3;
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kLine, 1);
  const double begin = now_;
  EXPECT_EQ(EndsWith(30), "NOT_MOVING");
  EXPECT_GT(ended_s_ - begin, 3.0);
  EXPECT_LT(ended_s_ - begin, 5.0);
}

TEST_F(ProgramTest, OverspeedOnALineAndInATurn) {
  mower_.v_scale = 1.6;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400), "OVERSPEED");
  EXPECT_EQ(edges_.back().phase, Phase::kLine);
  Rebuild();
  mower_ = Mower();
  mower_.w_scale = 1.6;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400), "OVERSPEED");
  EXPECT_EQ(edges_.back().phase, Phase::kSpinLeft);
}

TEST_F(ProgramTest, GyroAgainstTheWheels) {
  mower_.gyro_sign = -1;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400), "GYRO_SIGN");
  EXPECT_EQ(edges_.back().phase, Phase::kSpinLeft);
}

TEST_F(ProgramTest, GravityAndSlopeAtTheStartAndInAStand) {
  mower_.gravity = -9.80665;
  Run(now_ + 2.0);
  ASSERT_TRUE(program_.Start(Lane{2, 3, 0.0, 3.0}, now_));
  EXPECT_EQ(EndsWith(1.0), "IMU_GRAVITY");
  Rebuild();
  mower_ = Mower();
  mower_.gravity = 11.0;
  Run(now_ + 2.0);
  ASSERT_TRUE(program_.Start(Lane{2, 3, 0.0, 3.0}, now_));
  EXPECT_EQ(EndsWith(1.0), "IMU_GRAVITY");
  Rebuild();
  mower_ = Mower();
  mower_.tilt = 7 * kDeg;
  Run(now_ + 2.0);
  ASSERT_TRUE(program_.Start(Lane{2, 3, 0.0, 3.0}, now_));
  EXPECT_EQ(EndsWith(1.0), "SLOPE");
  // A slope at the far end of the lane: the stand there tells.
  Rebuild();
  mower_ = Mower();
  StartOn(Lane{2, 3, 0.0, 3.0});
  RunUntilPhase(Phase::kStand, 1);
  mower_.tilt = 7 * kDeg;
  EXPECT_EQ(EndsWith(3.0), "SLOPE");
  EXPECT_EQ(edges_.back().phase, Phase::kStand);
}

TEST_F(ProgramTest, TheSlopeCountsFromTheStoredMounting) {
  // The IMU sits 7 degrees nose up. Stored as level and measured, that is a slope of 7 degrees.
  mower_.mount_pitch = 7 * kDeg;
  Run(now_ + 2.0);
  ASSERT_EQ(program_.Readiness(now_).size(), 1u);
  EXPECT_EQ(program_.Readiness(now_)[0].reason, "SLOPE");
  // Stored as it is: level.
  program_.SetMounting(0.0, 7 * kDeg, 0.0);
  EXPECT_TRUE(program_.Readiness(now_).empty());
  // Stored as level but not measured yet: the allowance of the mounting is added to the limit.
  program_.SetMounting(0.0, 0.0, 5 * kDeg);
  EXPECT_TRUE(program_.Readiness(now_).empty());
  mower_.mount_pitch = 11 * kDeg;
  Run(now_ + 1.0);
  ASSERT_EQ(program_.Readiness(now_).size(), 1u);
  EXPECT_EQ(program_.Readiness(now_)[0].reason, "SLOPE");
  EXPECT_NEAR(program_.Readiness(now_)[0].numbers[1].second, 10.0, 1e-9);
}

TEST_F(ProgramTest, StandsFacingTheOtherWayTellTheSlopeWithoutTheMounting) {
  // Unknown mounting of 6 degrees on ground sloping 3 degrees: the start passes on the allowance, the stands half a
  // turn apart find the slope of 3 degrees, and the drive runs to its end.
  mower_.mount_pitch = 6 * kDeg;
  mower_.slope = 3 * kDeg;
  mower_.slope_heading = 0.7;
  program_.SetMounting(0.0, 0.0, 5 * kDeg);
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400), "DONE");
  // On 6 degrees the first stand with a partner ends it: the far end, after the first turn.
  Rebuild();
  mower_ = Mower();
  mower_.slope = 6 * kDeg;
  mower_.slope_heading = 0.0;
  program_.SetMounting(0.0, 0.0, 5 * kDeg);
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400), "SLOPE");
  EXPECT_NE(program_.outcome()->message.find("ground slopes"), std::string::npos);
  EXPECT_NEAR(program_.outcome()->numbers[0].second, 6.0, 0.2);
  EXPECT_EQ(edges_.back().phase, Phase::kStand);
  EXPECT_EQ(edges_.back().round, 1);
  EXPECT_EQ(EdgeNames(true).size(), 5u);
}

TEST_F(ProgramTest, TimeoutOfAPhase) {
  config_.timeout_factor = 1.0;
  config_.timeout_extra = 0.5;
  Rebuild();
  // 60 % of the speed is above NOT_MOVING, but the line takes too long.
  mower_.v_scale = 0.6;
  StartOn(Lane{2, 3, 0.0, 3.0});
  EXPECT_EQ(EndsWith(400), "TIMEOUT");
  EXPECT_NE(program_.outcome()->message.find("line"), std::string::npos);
}

TEST_F(ProgramTest, ReadinessNamesWhatIsMissing) {
  Run(now_ + 2.0);
  EXPECT_TRUE(program_.Readiness(now_).empty());
  const auto reasons = [&] {
    std::vector<std::string> out;
    for (const auto& p : program_.Readiness(now_)) out.push_back(p.reason);
    return out;
  };
  mower_.imu_period = 0.1;
  Run(now_ + 2.0);
  EXPECT_EQ(reasons(), std::vector<std::string>{"SENSOR_MISSING"});
  mower_.imu_period = 0.02;
  mower_.wheels_on = false;
  Run(now_ + 2.0);
  EXPECT_EQ(reasons(), std::vector<std::string>{"SENSOR_MISSING"});
  mower_.wheels_on = true;
  mower_.fixed = false;
  Run(now_ + 2.0);
  EXPECT_EQ(reasons(), std::vector<std::string>{"NO_RTK"});
  mower_.fixed = true;
  mower_.gnss_period = 0.7;
  Run(now_ + 3.0);
  EXPECT_EQ(reasons(), std::vector<std::string>{"GNSS_RATE"});
  mower_.gnss_period = 0.2;
  mower_.tilt = 7 * kDeg;
  Run(now_ + 2.0);
  EXPECT_EQ(reasons(), std::vector<std::string>{"SLOPE"});
  ASSERT_FALSE(program_.Readiness(now_)[0].numbers.empty());
  EXPECT_EQ(program_.Readiness(now_)[0].numbers[0].first, "slope");
  mower_.tilt = 0;
  mower_.gravity = -9.80665;
  Run(now_ + 2.0);
  EXPECT_EQ(reasons(), std::vector<std::string>{"IMU_GRAVITY"});
}

// The driver's limits cap the lines; the bound for OVERSPEED keeps the margin of the check above them.
TEST(ProgramLimits, SpeedLimitsOfTheDriver) {
  Program program{ProgramConfig{}};
  program.SetSpeedLimits(0.16298, 0.3);
  EXPECT_DOUBLE_EQ(program.config().line_speed, 0.16298);
  EXPECT_DOUBLE_EQ(program.config().max_speed, 1.3 * 0.16298 + 0.05);
  EXPECT_LE(program.config().line_min_speed, 0.16298);
  // At 0.3 m/s the bound stays at its own 0.3.
  Program tron{ProgramConfig{}};
  tron.SetSpeedLimits(0.3, 0.3);
  EXPECT_DOUBLE_EQ(tron.config().line_speed, 0.2);
  EXPECT_DOUBLE_EQ(tron.config().max_speed, 0.3);
  // No limit leaves the configuration as it is.
  Program none{ProgramConfig{}};
  none.SetSpeedLimits(0, 0);
  EXPECT_DOUBLE_EQ(none.config().max_speed, 0.3);
}
