#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <sstream>

#include <mowbite_localization/dock_estimator_journal.h>
#include <mowbite_localization/estimator_journal.h>
#include <mowbite_logic/logic_journal.h>
#include <mowbite_safety/safety_journal.h>
#include <tron_mcu/mcu_journal.h>

#include "mowbite_replay/cores.h"

namespace fs = std::filesystem;
using mowbite_journal::CheckOutcome;
using mowbite_journal::Journal;
using mowbite_journal::Journaled;

namespace {

// Each core runs a short scenario with a journal; the replay of the run has to be bit-identical.
class CoreReplay : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("replay_test_" + std::to_string(getpid()) + "_" +
                                        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    options_.dir = dir_.string();
    options_.reserve_mb = 0;
    std::string error;
    journal_ = Journal::Create(options_, &error);
    ASSERT_TRUE(journal_) << error;
  }
  void TearDown() override {
    fs::remove_all(dir_);
  }

  // Closes the journal and replays it.
  mowbite_journal::CheckReport Replay() {
    journal_.reset();
    return mowbite_journal::CheckRuns(dir_.string(), mowbite_replay::Cores());
  }

  static std::string Text(const mowbite_journal::CheckReport& report) {
    std::string text;
    for (const auto& line : report.lines) text += line + "\n";
    return text;
  }

  fs::path dir_;
  mowbite_journal::Options options_;
  std::unique_ptr<Journal> journal_;
};

TEST_F(CoreReplay, McuControl) {
  using C = tron::McuControl;
  tron::McuControlConfig config;
  config.allow_blade = true;
  config.imu_timeout_s = 0.5;
  config.imu_start_timeout_s = 2.0;
  config.expected_cutter_version = "1.2.3";
  {
    Journaled<C> mcu(journal_.get(), config);
    tron::WallClock wall;
    mcu.Call<&C::OnConnected>(0.0);
    mcu.Call<&C::OnVersions>(tron::VersionInfo{{1, 2, 3}, {4, 5, 6}, {0, 0, 0}});
    for (int i = 0; i < 300; ++i) {
      const double now = 0.01 * i;
      tron::SensorInfo sensor;
      sensor.lift = i > 150 && i < 160;
      mcu.Call<&C::OnSensorInfo>(sensor, now);
      tron::ImuCounts imu;
      imu.pitch = static_cast<int16_t>(i);
      mcu.Call<&C::OnImu>(imu, now);
      mcu.Call<&C::OnDockContactSample>(i < 20);
      if (i % 10 == 0) mcu.Call<&C::OnCmdVel>(0.1f, -0.05f, now);
      if (i % 50 == 0) mcu.Call<&C::OnBladeRpm>(i, now);
      if (i == 100) {
        uint32_t reason = 0;
        mcu.Call<&C::SetBlade>(tron::BladeRequest{true, 80, 50, 1.0}, now, &reason);
        mcu.Call<&C::SetBlade>(tron::BladeRequest{true, 0, 0, 5.0}, now, nullptr);
      }
      if (i == 170) mcu.Call<&C::ReleaseEmergency>(now, 0u);
      if (i == 200) mcu.Call<&C::LatchEmergency>(tron::kReasonTilt);
      if (i == 250) mcu.Call<&C::SetCharging>(false, now);
      if (i == 260) mcu.Call<&C::SetHold>(i % 2 == 0);
      mcu.Call<&C::Tick>(now, wall);
    }
  }
  const auto report = Replay();
  EXPECT_EQ(report.outcome, CheckOutcome::kSame) << Text(report);
  EXPECT_NE(Text(report).find("mcu: bit-identical"), std::string::npos) << Text(report);
}

TEST_F(CoreReplay, SafetyFilter) {
  using F = mowbite_safety::SafetyFilter;
  using mowbite_safety::Source;
  {
    Journaled<F> safety(journal_.get(), mowbite_safety::SafetyConfig{});
    safety.Call<&F::SetMap>(std::string(R"({"areas": []})"));
    safety.Call<&F::SetMapFits>(true);
    safety.Call<&F::SetMaxPositionAccuracy>(0.05);
    safety.Call<&F::OnCapabilities>(true, 0.3, 0.3);
    for (int i = 0; i < 200; ++i) {
      const double now = 0.02 * i;
      safety.Call<&F::OnHardware>(false, 0u, now);
      safety.Call<&F::OnTilt>(i > 120 && i < 190 ? 40.0 : 2.0, now);
      safety.Call<&F::OnPose>(0.01 * i, 0.0, 0.02, true, now);
      safety.Call<&F::OnCommand>(i % 3 == 0 ? Source::kTeleop : Source::kFollower, 0.2, std::nan(""), now);
      safety.Call<&F::Tick>(now);
      safety.Call<&F::TakeLatchRequest>(now);
      if (i == 195) safety.Call<&F::OnRelease>(now);
    }
  }
  const auto report = Replay();
  EXPECT_EQ(report.outcome, CheckOutcome::kSame) << Text(report);
  EXPECT_NE(Text(report).find("safety: bit-identical"), std::string::npos) << Text(report);
}

TEST_F(CoreReplay, Estimator) {
  using E = mowbite_localization::Estimator;
  mowbite_localization::EstimatorConfig config;
  config.gyro.calibration_time = 0.5;
  {
    Journaled<E> estimator(journal_.get(), config);
    estimator.Call<&E::SetAntennaOffset>(0.1, 0.0, 0.0);
    estimator.Call<&E::SetWheelScale>(1000.0, 990.0);
    estimator.Call<&E::SetGpsLimits>(0.1, 10.0);
    for (int i = 0; i < 500; ++i) {
      const double t = 0.01 * i;
      estimator.Call<&E::AddImu>(t, i < 100 ? 0.001 : 0.02);
      estimator.Call<&E::AddWheels>(t, i < 100 ? 0.0 : 0.3, 0.02);
      if (i % 10 == 0) {
        mowbite_localization::GnssFix fix;
        fix.solution = 4;
        fix.accuracy = 0.02;
        fix.x = 0.003 * i;
        fix.y = 0.001 * i;
        fix.motion_valid = i > 100;
        fix.motion_x = 0.3;
        estimator.Call<&E::AddFix>(fix, t);
      }
      estimator.Call<&E::TakeJump>();
      estimator.Call<&E::TakeAligned>();
      estimator.Call<&E::TakeGyroWarnings>();
      if (i == 300) estimator.Call<&E::SetHeading>(0.5, t);
      if (i == 400) estimator.Call<&E::SetPose>(mowbite_localization::Pose2{1.0, 2.0, 0.3});
      if (i == 450) estimator.Call<&E::SetGpsEnabled>(false);
    }
  }
  const auto report = Replay();
  EXPECT_EQ(report.outcome, CheckOutcome::kSame) << Text(report);
  EXPECT_NE(Text(report).find("localization: bit-identical, 2556 calls"), std::string::npos) << Text(report);
}

TEST_F(CoreReplay, DockEstimator) {
  using D = mowbite_localization::DockEstimator;
  mowbite_localization::DockConfig config;
  config.marker_x = -0.175;
  {
    Journaled<D> dock(journal_.get(), config);
    dock.Call<&D::SetDockedHeading>(M_PI);
    dock.Call<&D::SetHint>(M_PI, 0.0);
    for (int i = 0; i < 400; ++i) {
      const double t = 0.02 * i;
      const bool still = i < 200;
      dock.Call<&D::AddMotion>(mowbite_localization::Motion{t, still ? 0.0 : -0.002, 0.0001, 0.02, still});
      if (i % 5 == 0 && i > 60) {
        mowbite_localization::MarkerImage image;
        image.time = t - 0.1;
        image.x = -0.975 + (still ? 0.0 : 0.002 * (i - 200));
        image.y = 0.001 * (i % 3);
        image.covariance = {1e-5, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0, 0.0, 0.0};
        if (i == 300) image.x += 0.5;
        dock.Call<&D::AddMarker>(image, t);
      }
    }
  }
  const auto report = Replay();
  EXPECT_EQ(report.outcome, CheckOutcome::kSame) << Text(report);
  EXPECT_NE(Text(report).find("dock_localization: bit-identical"), std::string::npos) << Text(report);
}

TEST_F(CoreReplay, LogicCore) {
  using L = mowbite_logic::LogicCore;
  using namespace mowbite_logic;
  LogicConfig config;
  config.id_seed = 42;
  config.mow_with_blade = true;
  {
    Journaled<L> logic(journal_.get(), config, {{"api", 6}});
    HardwareSnapshot hw;
    hw.telemetry_ok = true;
    hw.battery_ok = true;
    hw.battery_voltage = 24.5f;
    hw.battery_percentage = 80;
    hw.blade.ok = hw.left.ok = hw.right.ok = true;
    PoseSnapshot pose;
    pose.x = 1;
    pose.y = 2;
    pose.position_accuracy = 0.02f;
    pose.orientation_accuracy = 0.01f;
    pose.orientation_valid = true;
    logic.Call<&L::OnMapStatus>(MapDatum::kMatch, std::string());
    logic.Call<&L::OnCapabilities>(true, true);
    logic.Call<&L::OnMowMap>(std::vector<JobArea>{{"front", "Front", true}, {"bed", "Bed", false}}, Reason());
    for (int i = 0; i < 100; ++i) {
      const double now = 0.1 * i;
      hw.time_s = now;
      hw.emergency = i >= 40 && i < 45;
      hw.emergency_reason = hw.emergency ? kEmergencyLift : 0;
      logic.Call<&L::OnHardwareStatus>(hw);
      pose.x = 1 + 0.01 * i;
      logic.Call<&L::OnPose>(pose);
      logic.Call<&L::OnTick>(now);
      if (i == 10) logic.Call<&L::OnAction>(std::string("mower_logic:idle/start_mowing"));
      if (i == 50) logic.Call<&L::Command>(std::string("emergency.reset"), json::object(), Reason());
      if (i == 51) logic.Call<&L::OnReleaseAnswered>(now);
      if (i == 60) logic.Call<&L::Command>(std::string("mowing.start"), json{{"areas", {"front"}}}, Reason());
      logic.Call<&L::NextJobStep>(now);
      logic.Call<&L::TakeEffects>();
    }
  }
  const auto report = Replay();
  EXPECT_EQ(report.outcome, CheckOutcome::kSame) << Text(report);
  EXPECT_NE(Text(report).find("logic: bit-identical"), std::string::npos) << Text(report);
}

}  // namespace
