#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include <mowbite_localization/dock_estimator_journal.h>
#include <mowbite_localization/estimator_journal.h>
#include <mowbite_journal/timeline.h>
#include <mowbite_logic/logic_journal.h>
#include <mowbite_safety/safety_journal.h>
#include <mowbite_schedule/schedule_journal.h>
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

// Journals from before the dock source: no dock_max_position_accuracy and dock_accuracy_per_m in the config and no
// OnDockPose in the table, nor the methods that came after it; they ran with a limit the same everywhere.
TEST(SafetyJournal, ConfigFromBeforeTheDockSourceTakesTheDefault) {
  using F = mowbite_safety::SafetyFilter;
  mowbite_safety::SafetyConfig config;
  config.dock_max_position_accuracy = 0.07;
  config.max_position_accuracy = 0.08;
  auto definition = mowbite_journal::CoreDefinition<F>(config, nlohmann::json::object());
  definition["config"].erase("dock_max_position_accuracy");
  definition["config"].erase("dock_accuracy_per_m");
  auto& methods = definition["methods"];
  const auto dock_pose = std::find_if(methods.begin(), methods.end(),
                                      [](const nlohmann::json& method) { return method.at("name") == "OnDockPose"; });
  ASSERT_NE(dock_pose, methods.end());
  methods.erase(dock_pose, methods.end());
  std::vector<std::string> names, signatures;
  for (const auto& method : methods) {
    names.push_back(method.at("name"));
    signatures.push_back(method.at("signature"));
  }
  definition["table"] = mowbite_journal::detail::TableChecksum(names, signatures);

  const auto read = definition.at("config").get<mowbite_safety::SafetyConfig>();
  EXPECT_EQ(read.dock_max_position_accuracy, 0.03);
  EXPECT_EQ(read.dock_accuracy_per_m, 0.0);
  EXPECT_EQ(read.max_position_accuracy, 0.08);
  EXPECT_NO_THROW(mowbite_journal::Replayer<F>{definition});
}

TEST(SafetyJournal, ConfigWithoutAnOlderFieldIsRefused) {
  nlohmann::json config = mowbite_safety::SafetyConfig{};
  config.erase("max_position_accuracy");
  EXPECT_THROW(config.get<mowbite_safety::SafetyConfig>(), nlohmann::json::exception);
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

// Journals replay while their table is the start of today's: the methods of the logic keep name, place and signature,
// new ones only come at the end. The checksum covers the first kTableLength entries.
TEST(LogicJournal, TheTableIsOnlyAppendedTo) {
  using L = mowbite_logic::LogicCore;
  constexpr size_t kTableLength = 72;
  constexpr char kTableChecksum[] = "4e3fe60ff0f8c9de";
  const auto definition = mowbite_journal::CoreDefinition<L>(mowbite_logic::LogicConfig(), nlohmann::json::object());
  const auto& methods = definition.at("methods");
  ASSERT_GE(methods.size(), kTableLength);
  std::vector<std::string> names, signatures;
  for (size_t i = 0; i < kTableLength; ++i) {
    names.push_back(methods[i].at("name"));
    signatures.push_back(methods[i].at("signature"));
  }
  EXPECT_EQ(mowbite_journal::detail::TableChecksum(names, signatures), kTableChecksum)
      << "a method of the journal table changed: new methods only at its end";
}

// The bytes of a double as a call has them (little endian) and as CBOR has them (big endian).
std::string DoubleBytes(double value, bool big_endian) {
  std::string bytes(sizeof(value), '\0');
  std::memcpy(bytes.data(), &value, sizeof(value));
  if (big_endian) std::reverse(bytes.begin(), bytes.end());
  return bytes;
}

std::string Unpacked(const std::string& run) {
  std::string bytes;
  for (const auto& path : mowbite_journal::ListRun(run).segments) {
    mowbite_journal::Segment segment;
    std::string error;
    EXPECT_TRUE(mowbite_journal::ReadSegment(path, &segment, &error)) << error;
    bytes.append(segment.raw.begin(), segment.raw.end());
  }
  return bytes;
}

// Days since 1970 of the date an ISO text or a run name starts with.
int64_t DayOf(const std::string& text) {
  std::tm tm{};
  const bool dashes = text.size() > 4 && text[4] == '-';
  tm.tm_year = std::stoi(text.substr(0, 4)) - 1900;
  tm.tm_mon = std::stoi(text.substr(dashes ? 5 : 4, 2)) - 1;
  tm.tm_mday = std::stoi(text.substr(dashes ? 8 : 6, 2));
  return static_cast<int64_t>(timegm(&tm)) / 86400;
}

nlohmann::json FirstHead(const std::string& run) {
  mowbite_journal::Segment segment;
  std::string error;
  EXPECT_TRUE(mowbite_journal::ReadSegment(mowbite_journal::ListRun(run).segments.at(0), &segment, &error)) << error;
  const auto& head = segment.records.at(0);
  EXPECT_EQ(head.method, mowbite_journal::kHead);
  return nlohmann::json::from_cbor(head.data, head.data + head.size);
}

std::vector<nlohmann::json> Exported(const std::string& run, const std::string& stream, const std::string& method) {
  std::ostringstream out;
  std::string error;
  EXPECT_TRUE(mowbite_journal::ExportJsonl(run, mowbite_replay::Cores(), {stream, method}, out, &error)) << error;
  std::vector<nlohmann::json> lines;
  std::istringstream in(out.str());
  std::string line;
  while (std::getline(in, line)) lines.push_back(nlohmann::json::parse(line));
  return lines;
}

// The logic with its datum and clock, the schedule asking the weather at its place and the MCU sending a heartbeat:
// the copy has none of the coordinates, the wall clock 1001 days earlier and reads and replays.
TEST_F(CoreReplay, AnonymizedCopyHasNoPlace) {
  using L = mowbite_logic::LogicCore;
  using S = mowbite_schedule::Scheduler;
  using C = tron::McuControl;
  constexpr double kLatitude = 51.123456;
  constexpr double kLongitude = 7.654321;
  constexpr char kBerlin[] = "CET-1CEST,M3.5.0,M10.5.0/3";
  std::string reason;
  // Monday 15 June 2026, 10:00 in Berlin.
  const int64_t wall = mowbite_schedule::TimeZone::Parse(kBerlin, &reason)->ToUnix({2026, 6, 15}, 600);
  {
    mowbite_logic::LogicConfig config;
    config.id_seed = 7;
    Journaled<L> logic(journal_.get(), config, {{"api", 6}});
    logic.Call<&L::OnClock>(true, std::string(), static_cast<double>(wall) - 100.0);
    logic.Call<&L::OnDatum>(kLatitude, kLongitude);
    logic.Call<&L::OnTick>(100.0);
    logic.Call<&L::TakeEffects>();

    mowbite_schedule::ScheduleFile file;
    const nlohmann::json change = {{"enabled", 1},
                                   {"tz", kBerlin},
                                   {"skip_forecast", 1},
                                   {"plans", {{{"days", {1, 2, 3, 4, 5, 6, 7}}, {"time", "10:00"}}}}};
    ASSERT_FALSE(mowbite_schedule::ApplySchedule(change, true, 0, &file));
    file.changed_at = wall - 36000;
    Journaled<S> schedule(journal_.get(), mowbite_schedule::SchedulerStart{file, true, nullptr, nullptr, wall - 36000});
    mowbite_schedule::Facts facts;
    facts.wall = wall;
    facts.mono = 100.0;
    facts.clock_ok = true;
    facts.logic_fresh = true;
    facts.state = "IDLE";
    facts.docked = true;
    facts.battery = 100;
    facts.rain_sensor = true;
    facts.latitude = kLatitude;
    facts.longitude = kLongitude;
    facts.areas = {{"a", true}};
    schedule.Call<&S::Tick>(facts);
    ASSERT_TRUE(schedule.Call<&S::TakeOutput>().weather);

    Journaled<C> mcu(journal_.get(), tron::McuControlConfig{});
    mcu.Call<&C::OnConnected>(0.0);
    mcu.Call<&C::Tick>(0.0, tron::WallClock{2026, 6, 15, 8, 0, 0, 0});
    const uint16_t port = journal_->Define({{"name", "port.tx"}, {"kind", "raw"}});
    const uint8_t frame[4] = {0xA5, 26, 6, 15};
    journal_->Append(port, mowbite_journal::kRawBytes, frame, sizeof(frame));
  }
  journal_.reset();
  const auto runs = mowbite_journal::ListRuns(dir_.string());
  ASSERT_EQ(runs.size(), 1u);
  const std::string copy = (dir_ / "copy").string();
  std::string error;
  ASSERT_TRUE(mowbite_replay::AnonymizeRun(runs[0], copy, mowbite_journal::AnonymizeOptions{}, &error)) << error;
  EXPECT_FALSE(mowbite_replay::AnonymizeRun(runs[0], copy, mowbite_journal::AnonymizeOptions{}, &error));

  const std::string before = Unpacked(runs[0]);
  const std::string after = Unpacked(copy);
  for (double value : {kLatitude, kLongitude}) {
    EXPECT_NE(before.find(DoubleBytes(value, false)), std::string::npos);
    EXPECT_NE(before.find(DoubleBytes(value, true)), std::string::npos);
    EXPECT_EQ(after.find(DoubleBytes(value, false)), std::string::npos) << value;
    EXPECT_EQ(after.find(DoubleBytes(value, true)), std::string::npos) << value;
  }
  for (const char* text : {"51.123456", "7.654321"}) {
    EXPECT_EQ(after.find(text), std::string::npos) << text;
  }
  EXPECT_NE(after.find(DoubleBytes(52.0, false)), std::string::npos);
  EXPECT_NE(after.find(DoubleBytes(9.0, false)), std::string::npos);

  const auto head_before = FirstHead(runs[0]);
  const auto head_after = FirstHead(copy);
  for (const char* key : {"start_utc", "segment_start_utc", "run"}) {
    EXPECT_EQ(DayOf(head_before.at(key)) - DayOf(head_after.at(key)), 1001) << key;
  }
  EXPECT_EQ(head_after.at("boot_id"), "00000000-0000-0000-0000-000000000000");
  EXPECT_NE(head_before.at("boot_id"), head_after.at("boot_id"));

  const int64_t shift = int64_t{1001} * 86400;
  const auto ticks = Exported(copy, "schedule", "Tick");
  ASSERT_EQ(ticks.size(), 1u);
  EXPECT_EQ(ticks[0]["args"][0]["wall"], wall - shift);
  EXPECT_EQ(ticks[0]["args"][0]["latitude"], 52.0);
  EXPECT_EQ(ticks[0]["args"][0]["longitude"], 9.0);
  const auto outputs = Exported(copy, "schedule", "TakeOutput");
  ASSERT_EQ(outputs.size(), 1u);
  EXPECT_EQ(outputs[0]["result"]["weather"]["latitude"], 52.0);
  const auto clocks = Exported(copy, "logic", "OnClock");
  ASSERT_EQ(clocks.size(), 1u);
  EXPECT_EQ(clocks[0]["args"][2], static_cast<double>(wall - 100 - shift));
  const auto mcu_ticks = Exported(copy, "mcu", "Tick");
  ASSERT_EQ(mcu_ticks.size(), 1u);
  const auto& clock = mcu_ticks[0]["args"][1];
  EXPECT_EQ((std::vector<int>{clock["year"].get<int>(), clock["month"].get<int>(), clock["day"].get<int>()}),
            (std::vector<int>{2023, 9, 18}));
  const auto port = Exported(copy, "port.tx", "bytes");
  ASSERT_EQ(port.size(), 1u);
  EXPECT_EQ(port[0]["hex"], "00000000");

  const auto report = mowbite_journal::CheckRuns(copy, mowbite_replay::Cores());
  EXPECT_NE(report.outcome, CheckOutcome::kError) << Text(report);
  EXPECT_EQ(Text(report).find("cannot"), std::string::npos) << Text(report);
  EXPECT_NE(Text(report).find("mcu: bit-identical, 2 calls"), std::string::npos) << Text(report);
  error.clear();
  const std::string timeline = mowbite_journal::Timeline({copy}, INT64_MIN, INT64_MAX, 1 << 20, &error);
  EXPECT_TRUE(error.empty()) << error;
  EXPECT_NE(timeline.find("\"method\":\"OnDatum\""), std::string::npos) << timeline;
}

}  // namespace
