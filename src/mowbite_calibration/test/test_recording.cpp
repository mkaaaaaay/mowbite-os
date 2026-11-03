#include <gtest/gtest.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <utime.h>
#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "mowbite_calibration/recording.h"

namespace mowbite_calibration {
namespace {

namespace fs = std::filesystem;

// 2026-11-03 12:00:00 UTC.
constexpr double kNow = 1793707200.0;

class RecordingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char pattern[] = "/tmp/mowbite_calibration_XXXXXX";
    folder_ = mkdtemp(pattern);
  }
  void TearDown() override {
    fs::remove_all(folder_);
  }

  std::vector<std::string> Names() const {
    std::vector<std::string> names;
    for (const auto& item : fs::directory_iterator(folder_)) names.push_back(item.path().filename().string());
    std::sort(names.begin(), names.end());
    return names;
  }

  static std::vector<json> Read(const std::string& path) {
    gzFile file = gzopen(path.c_str(), "rb");
    std::string text;
    char buffer[4096];
    int n;
    while ((n = gzread(file, buffer, sizeof(buffer))) > 0) text.append(buffer, n);
    gzclose(file);
    std::vector<json> lines;
    size_t start = 0;
    for (size_t end = text.find('\n'); end != std::string::npos; end = text.find('\n', start)) {
      lines.push_back(json::parse(text.substr(start, end - start)));
      start = end + 1;
    }
    EXPECT_EQ(start, text.size());
    return lines;
  }

  // A finished recording named for wall_t, old by its time of change.
  void Old(double wall_t, size_t bytes) {
    Recording recording;
    ASSERT_TRUE(recording.Open(folder_, json::object(), wall_t, RecordingLimits{100, 1u << 30, 1e9}, nullptr));
    const std::string path = recording.End(wall_t, "enough", "");
    {
      std::ofstream pad(path, std::ios::app | std::ios::binary);
      pad << std::string(bytes, 'x');
    }
    struct utimbuf times{static_cast<time_t>(wall_t), static_cast<time_t>(wall_t)};
    utime(path.c_str(), &times);
  }

  std::string folder_;
};

json Head() {
  return json{{"mode", "program"},
              {"started", kNow},
              {"params", {{"ticks_per_m", 1000.0}, {"wheel_distance_m", 0.395}, {"antenna_offset_x", 0.0},
                          {"antenna_offset_y", 0.0}}},
              {"basis", {{"ticks_per_m", 1000.0}, {"wheel_distance_m", 0.40}}},
              {"imu_frame", "base_link"},
              {"imu", {{"roll", 0.0}, {"pitch", 0.0}}},
              {"program", {{"rounds", 4}, {"lane_length", 3.0}}}};
}

TEST_F(RecordingTest, HeadLinesAndEndInTheFormat) {
  Recording recording;
  std::string why;
  ASSERT_TRUE(recording.Open(folder_, Head(), kNow, RecordingLimits{}, &why)) << why;
  // Written under another name until it ends.
  EXPECT_EQ(Names(), std::vector<std::string>{"calib-20261103T120000Z.jsonl.gz.part"});
  EXPECT_TRUE(recording.Line(kNow + 0.01, "imu", {{"gx", 0.01}, {"gy", 0.0}, {"gz", -0.02}, {"ax", 0.1},
                                                  {"ay", 0.0}, {"az", 9.8}}));
  EXPECT_TRUE(recording.Line(kNow + 0.2, "gps", {{"x", 1.0}, {"y", 2.0}, {"z", 0.1}, {"acc", 0.01}, {"fix", "fixed"},
                                                 {"vh", nullptr}, {"mh", nullptr}}));
  EXPECT_TRUE(recording.Line(kNow + 0.25, "phase", {{"name", "stand"}, {"edge", "start"}}));
  const std::string path = recording.End(kNow + 1.0, "enough", "");
  EXPECT_EQ(path, folder_ + "/calib-20261103T120000Z.jsonl.gz");
  EXPECT_FALSE(recording.open());
  EXPECT_EQ(recording.finished(), path);
  EXPECT_EQ(Names(), std::vector<std::string>{"calib-20261103T120000Z.jsonl.gz"});

  const auto lines = Read(path);
  ASSERT_EQ(lines.size(), 5u);
  EXPECT_EQ(lines[0]["format"], "mowbite-calib/1");
  EXPECT_EQ(lines[0]["source"], "mowbite-os");
  EXPECT_EQ(lines[0]["basis"]["wheel_distance_m"], 0.40);
  EXPECT_LE(Depth(lines[0]), 3);
  EXPECT_EQ(lines[1]["k"], "imu");
  EXPECT_DOUBLE_EQ(lines[1]["t"].get<double>(), kNow + 0.01);
  EXPECT_EQ(lines[2]["fix"], "fixed");
  EXPECT_TRUE(lines[2]["vh"].is_null());
  EXPECT_EQ(lines[3]["name"], "stand");
  EXPECT_EQ(lines[4]["k"], "end");
  EXPECT_EQ(lines[4]["why"], "enough");
  EXPECT_FALSE(lines[4].contains("reason"));
  for (const auto& line : lines) {
    if (line.contains("k")) {
      EXPECT_EQ(Depth(line), 1);
    }
  }
}

TEST_F(RecordingTest, OnlyFiniteFlatShortLines) {
  Recording recording;
  ASSERT_TRUE(recording.Open(folder_, Head(), kNow, RecordingLimits{}, nullptr));
  EXPECT_FALSE(recording.Line(kNow, "imu", {{"gx", std::numeric_limits<double>::quiet_NaN()}}));
  EXPECT_FALSE(recording.Line(kNow, "imu", {{"gx", std::numeric_limits<double>::infinity()}}));
  EXPECT_FALSE(recording.Line(kNow, "pose", {{"x", json::array({1, 2})}}));
  EXPECT_FALSE(recording.Line(kNow, "pose", {{"x", {{"a", 1}}}}));
  EXPECT_FALSE(recording.Line(kNow, "gps", {{"fix", std::string(5000, 'f')}}));
  EXPECT_FALSE(recording.Line(std::numeric_limits<double>::quiet_NaN(), "imu", {{"gx", 0.0}}));
  EXPECT_EQ(recording.dropped(), 6u);
  const auto lines = Read(recording.End(kNow + 1, "error", "NO_RTK"));
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[1]["reason"], "NO_RTK");
  // A head deeper than 3 levels, or with a number that is not finite, is refused.
  Recording deep;
  std::string why;
  EXPECT_FALSE(deep.Open(folder_, json{{"a", {{"b", {{"c", {{"d", 1}}}}}}}}, kNow + 5, RecordingLimits{}, &why));
  EXPECT_FALSE(why.empty());
  EXPECT_FALSE(deep.Open(folder_, json{{"started", std::numeric_limits<double>::infinity()}}, kNow + 5,
                         RecordingLimits{}, nullptr));
}

TEST_F(RecordingTest, ARecordingEndsAtItsSize) {
  Recording recording;
  RecordingLimits limits;
  limits.max_bytes = 2000;
  ASSERT_TRUE(recording.Open(folder_, Head(), kNow, limits, nullptr));
  int written = 0;
  // Numbers that do not compress well.
  for (int i = 0; i < 100000 && recording.open(); i++) {
    if (recording.Line(kNow + i * 0.01, "imu", {{"gx", std::sin(i * 1.3) * 1.234567}, {"az", std::cos(i) * 9.81}}))
      written++;
  }
  EXPECT_FALSE(recording.open());
  EXPECT_GT(written, 10);
  const auto lines = Read(recording.finished());
  EXPECT_EQ(lines.back()["why"], "size");
  EXPECT_LT(fs::file_size(recording.finished()), 2000u + 66000u);
  // Nothing more goes in, and End finds nothing to end.
  EXPECT_FALSE(recording.Line(kNow, "imu", {{"gx", 0.0}}));
  EXPECT_EQ(recording.End(kNow, "enough", ""), "");
}

TEST_F(RecordingTest, PruneKeepsTheNewestWithinTheLimits) {
  RecordingLimits limits;
  limits.max_runs = 4;
  limits.max_bytes = 10000;
  limits.max_days = 60;
  Old(kNow - 70 * 86400.0, 10);   // too old
  Old(kNow - 5 * 86400.0, 7000);  // over the bytes once the newer ones count
  Old(kNow - 4 * 86400.0, 4000);
  Old(kNow - 3 * 86400.0, 10);
  Old(kNow - 2 * 86400.0, 10);
  std::ofstream(folder_ + "/calib-20261101T000000Z.jsonl.gz.part") << "x";
  std::ofstream(folder_ + "/notes.txt") << "not ours";
  const auto removed = Prune(folder_, kNow, limits);
  EXPECT_EQ(removed.size(), 3u);
  EXPECT_EQ(Names(), (std::vector<std::string>{"calib-20261030T120000Z.jsonl.gz", "calib-20261031T120000Z.jsonl.gz",
                                               "calib-20261101T120000Z.jsonl.gz", "notes.txt"}));
  // keep stays even when the limits would take it.
  limits.max_runs = 1;
  Prune(folder_, kNow, limits, folder_ + "/calib-20261030T120000Z.jsonl.gz");
  EXPECT_EQ(Names(), (std::vector<std::string>{"calib-20261030T120000Z.jsonl.gz", "calib-20261101T120000Z.jsonl.gz",
                                               "notes.txt"}));
}

TEST_F(RecordingTest, OpenMakesRoomForTheNewOne) {
  RecordingLimits limits;
  limits.max_runs = 2;
  Old(kNow - 2 * 86400.0, 10);
  Old(kNow - 1 * 86400.0, 10);
  Recording recording;
  ASSERT_TRUE(recording.Open(folder_, Head(), kNow, limits, nullptr));
  recording.End(kNow + 1, "stop", "STOPPED");
  EXPECT_EQ(Names(), (std::vector<std::string>{"calib-20261102T120000Z.jsonl.gz", "calib-20261103T120000Z.jsonl.gz"}));
  // A second one in the same second gets a name of its own.
  ASSERT_TRUE(recording.Open(folder_, Head(), kNow, RecordingLimits{}, nullptr));
  EXPECT_EQ(recording.End(kNow + 1, "stop", ""), folder_ + "/calib-20261103T120000Z-2.jsonl.gz");
}

}  // namespace
}  // namespace mowbite_calibration
