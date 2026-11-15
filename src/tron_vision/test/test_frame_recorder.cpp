#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#include "test_util.h"
#include "tron_vision/frame_recorder.h"

namespace tron_vision {
namespace {

namespace fs = std::filesystem;

const std::vector<uint8_t> kImage(100, 0xAB);
const std::string kText = "reason 4\n";
constexpr size_t kPair = 100 + 9;

std::string Read(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// The names in the directory.
std::set<std::string> Names(const std::string& dir) {
  std::set<std::string> out;
  for (const auto& e : fs::directory_iterator(dir)) out.insert(e.path().filename().string());
  return out;
}

RecordLimits Limits(const std::string& dir, size_t pairs) {
  RecordLimits limits;
  limits.dir = dir;
  limits.max_bytes = pairs * kPair;
  limits.keep_s = 3600.0;
  limits.interval_s = 0.5;
  return limits;
}

bool Put(FrameRecorder& recorder, double taken, std::string& error) {
  return recorder.Write(taken, kImage.data(), kImage.size(), kText, error);
}

TEST(FrameRecorder, OffWithoutBytes) {
  test::TempDir dir;
  FrameRecorder recorder(Limits(dir.Path() + "/rec", 0));
  EXPECT_FALSE(recorder.Active());
  EXPECT_FALSE(recorder.Due(1.0));
  std::string error;
  EXPECT_FALSE(Put(recorder, 1.0, error));
  EXPECT_FALSE(fs::exists(dir.Path() + "/rec"));
}

TEST(FrameRecorder, KeepsTheImageAsItCameWithItsTextAtTheInterval) {
  test::TempDir dir;
  FrameRecorder recorder(Limits(dir.Path() + "/rec", 10));
  ASSERT_TRUE(recorder.Active());
  std::string error;
  ASSERT_TRUE(recorder.Due(1.0));
  ASSERT_TRUE(Put(recorder, 1.0, error)) << error;
  EXPECT_FALSE(recorder.Due(1.4));
  EXPECT_TRUE(recorder.Due(1.5));
  EXPECT_EQ(Names(dir.Path() + "/rec"), (std::set<std::string>{"1000.jpg", "1000.txt"}));
  EXPECT_EQ(Read(dir.Path() + "/rec/1000.jpg"), std::string(kImage.begin(), kImage.end()));
  EXPECT_EQ(Read(dir.Path() + "/rec/1000.txt"), kText);
}

TEST(FrameRecorder, TheOldestGoOverTheBytes) {
  test::TempDir dir;
  FrameRecorder recorder(Limits(dir.Path(), 3));
  std::string error;
  for (double t : {1.0, 2.0, 3.0, 4.0}) ASSERT_TRUE(Put(recorder, t, error)) << error;
  EXPECT_EQ(Names(dir.Path()),
            (std::set<std::string>{"2000.jpg", "2000.txt", "3000.jpg", "3000.txt", "4000.jpg", "4000.txt"}));
}

TEST(FrameRecorder, TheOldestGoAfterTheKeepTime) {
  test::TempDir dir;
  RecordLimits limits = Limits(dir.Path(), 10);
  limits.keep_s = 2.0;
  FrameRecorder recorder(limits);
  std::string error;
  for (double t : {1.0, 2.0, 3.5}) ASSERT_TRUE(Put(recorder, t, error)) << error;
  EXPECT_EQ(Names(dir.Path()), (std::set<std::string>{"2000.jpg", "2000.txt", "3500.jpg", "3500.txt"}));
}

TEST(FrameRecorder, ImagesOfAnEarlierRunCount) {
  test::TempDir dir;
  dir.Write("500.jpg", std::string(100, 'x'));
  dir.Write("500.txt", kText);
  dir.Write("700.jpg", std::string(100, 'x'));
  dir.Write("700.txt", kText);
  dir.Write("notes.jpg", "not ours");
  FrameRecorder recorder(Limits(dir.Path(), 2));
  std::string error;
  ASSERT_TRUE(Put(recorder, 1.0, error)) << error;
  EXPECT_EQ(Names(dir.Path()), (std::set<std::string>{"700.jpg", "700.txt", "1000.jpg", "1000.txt", "notes.jpg"}));
}

TEST(FrameRecorder, AnImageOverTheLimitEndsTheRecording) {
  test::TempDir dir;
  FrameRecorder recorder(Limits(dir.Path(), 1));
  const std::vector<uint8_t> big(kPair + 1, 0);
  std::string error;
  EXPECT_FALSE(recorder.Write(1.0, big.data(), big.size(), kText, error));
  EXPECT_NE(error.find("over the limit"), std::string::npos) << error;
  EXPECT_FALSE(recorder.Active());
  EXPECT_TRUE(Names(dir.Path()).empty());
}

TEST(FrameRecorder, AFailedWriteEndsTheRecordingAndLeavesNoPart) {
  test::TempDir dir;
  const std::string rec = dir.Path() + "/rec";
  FrameRecorder recorder(Limits(rec, 10));
  ASSERT_TRUE(recorder.Active());
  fs::remove_all(rec);
  dir.Write("rec", "a file where the directory was");
  std::string error;
  EXPECT_FALSE(Put(recorder, 1.0, error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(recorder.Active());
  EXPECT_FALSE(recorder.Due(2.0));
  EXPECT_EQ(Names(dir.Path()), (std::set<std::string>{"rec"}));
}

TEST(FrameRecorder, ADirectoryThatCannotBeMadeLeavesItOff) {
  test::TempDir dir;
  dir.Write("file", "x");
  FrameRecorder recorder(Limits(dir.Path() + "/file/rec", 10));
  EXPECT_FALSE(recorder.Active());
}

}  // namespace
}  // namespace tron_vision
