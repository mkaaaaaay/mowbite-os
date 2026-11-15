#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "mowbite_journal/format.h"
#include "mowbite_journal/journal.h"
#include "mowbite_journal/reader.h"
#include "mowbite_journal/run_index.h"

namespace fs = std::filesystem;

namespace mowbite_journal {
namespace {

void Wait(double seconds) {
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

class RunIndexTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() / ("run_index_test_" + std::to_string(getpid()) + "_" +
                                        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    options_.dir = dir_.string();
    options_.platform = "sim";
    options_.reserve_mb = 0;
    options_.flush_s = 0.05;
  }
  void TearDown() override {
    fs::remove_all(dir_);
  }

  // Records for the given time into a journal of its own; the journal is open for the whole time.
  static void Feed(Journal& journal, double seconds) {
    const uint16_t stream = journal.Define({{"name", "feed"}, {"kind", "raw"}});
    const uint8_t bytes[] = {1, 2, 3, 4};
    for (int i = 0; i < static_cast<int>(seconds / 0.02); ++i) {
      journal.Append(stream, kRawBytes, bytes, sizeof(bytes));
      Wait(0.02);
    }
  }

  // A run that ended, with its folder.
  std::string Finished(double seconds) {
    std::string error;
    auto journal = Journal::Create(options_, &error);
    EXPECT_TRUE(journal) << error;
    if (!journal) return {};
    const std::string run = journal->GetStats().run_dir;
    Feed(*journal, seconds);
    return run;
  }

  fs::path dir_;
  Options options_;
};

TEST_F(RunIndexTest, AFinishedRunWithTwoSegments) {
  options_.segment_s = 0.15;
  const std::string run = Finished(0.6);
  const auto runs = InspectRuns(dir_.string(), true);
  ASSERT_EQ(runs.size(), 1u);
  const RunInfo& info = runs[0];
  EXPECT_EQ(info.dir, run);
  EXPECT_EQ(info.name, fs::path(run).filename().string());
  EXPECT_FALSE(info.active);
  EXPECT_TRUE(info.ended);
  EXPECT_EQ(info.end_reason, kEndExit);
  EXPECT_EQ(info.pid, getpid());
  EXPECT_FALSE(info.process.empty());
  EXPECT_FALSE(info.boot_id.empty());
  EXPECT_EQ(info.platform, "sim");
  EXPECT_FALSE(info.revision.empty());
  EXPECT_FALSE(info.image.empty());
  EXPECT_TRUE(info.missing.empty()) << info.missing;
  EXPECT_TRUE(info.problem.empty()) << info.problem;
  ASSERT_GE(info.segments.size(), 2u);
  for (size_t i = 0; i < info.segments.size(); ++i) {
    const SegmentInfo& segment = info.segments[i];
    EXPECT_EQ(segment.index, i);
    EXPECT_TRUE(segment.closed) << i;
    EXPECT_GT(segment.start_ns, 0);
    EXPECT_EQ(segment.start_utc.size(), 20u) << segment.start_utc;
    EXPECT_EQ(segment.bytes, fs::file_size(segment.path));
    if (i > 0) {
      EXPECT_GT(segment.start_ns, info.segments[i - 1].start_ns);
    }
  }
}

TEST_F(RunIndexTest, WithoutTheEndOnlyHeadsAreRead) {
  options_.segment_s = 0.15;
  const std::string run = Finished(0.6);
  const RunInfo info = InspectRun(run, false);
  ASSERT_GE(info.segments.size(), 2u);
  EXPECT_FALSE(info.ended);
  EXPECT_TRUE(info.end_reason.empty());
  EXPECT_TRUE(info.segments.front().closed);
  EXPECT_FALSE(info.segments.back().closed);
  EXPECT_TRUE(info.problem.empty()) << info.problem;
}

TEST_F(RunIndexTest, ARunningJournalIsActiveAndHasNotEnded) {
  std::string error;
  auto journal = Journal::Create(options_, &error);
  ASSERT_TRUE(journal) << error;
  const std::string run = journal->GetStats().run_dir;
  Feed(*journal, 0.3);
  const RunInfo running = InspectRun(run, true);
  EXPECT_TRUE(running.active);
  EXPECT_FALSE(running.ended);
  ASSERT_EQ(running.segments.size(), 1u);
  EXPECT_FALSE(running.segments[0].closed);
  EXPECT_EQ(running.pid, getpid());
  EXPECT_TRUE(running.problem.empty()) << running.problem;
  journal.reset();
  const RunInfo done = InspectRun(run, true);
  EXPECT_FALSE(done.active);
  EXPECT_TRUE(done.ended);
  EXPECT_TRUE(done.segments[0].closed);
}

TEST_F(RunIndexTest, ARotationIsNoEndOfTheProcess) {
  options_.segment_s = 0.15;
  const std::string run = Finished(0.6);
  // Without the last segment, the one before ends with its rotation.
  RunInfo info = InspectRun(run, false);
  ASSERT_GE(info.segments.size(), 2u);
  fs::remove(info.segments.back().path);
  info = InspectRun(run, true);
  EXPECT_TRUE(info.segments.back().closed);
  EXPECT_FALSE(info.ended);
  EXPECT_EQ(info.end_reason, kEndRotate);
}

TEST_F(RunIndexTest, CutOffAndBrokenFilesAreProblemsNotCrashes) {
  options_.segment_s = 0.15;
  const std::string run = Finished(0.6);
  RunInfo info = InspectRun(run, false);
  ASSERT_GE(info.segments.size(), 2u);
  const std::string last = info.segments.back().path;
  fs::resize_file(last, fs::file_size(last) - 5);
  info = InspectRun(run, true);
  EXPECT_FALSE(info.ended);
  EXPECT_FALSE(info.segments.back().closed);
  EXPECT_NE(info.problem.find("cut off in a block"), std::string::npos) << info.problem;

  // A first segment cut off in its head: the run takes its fields from the next one.
  fs::resize_file(info.segments.front().path, 20);
  info = InspectRun(run, true);
  EXPECT_NE(info.problem.find("000000.mbj"), std::string::npos) << info.problem;
  EXPECT_EQ(info.pid, getpid());
  EXPECT_EQ(info.segments.front().start_ns, 0);

  // A block header that claims 4 GB, a file of text and an empty file.
  {
    std::ofstream bad(fs::path(run) / "000000.mbj", std::ios::binary | std::ios::trunc);
    const uint32_t header[] = {0xFFFFFFFFu, 16u, 0x12345678u, 0x12345678u, 0x12345678u, 0x12345678u};
    bad.write(kMagic, 4);
    bad.write(reinterpret_cast<const char*>(&kFormatVersion), 4);
    bad.write(reinterpret_cast<const char*>(header), sizeof(header));
  }
  std::ofstream(fs::path(run) / "000090.mbj") << "no journal";
  std::ofstream(fs::path(run) / "000091.mbj");
  info = InspectRun(run, true);
  EXPECT_FALSE(info.ended);
  EXPECT_NE(info.problem.find("000000.mbj: broken first block"), std::string::npos) << info.problem;
  EXPECT_NE(info.problem.find("000090.mbj: no journal segment"), std::string::npos) << info.problem;
  EXPECT_NE(info.problem.find("000091.mbj"), std::string::npos) << info.problem;

  Segment segment;
  std::string error;
  ASSERT_TRUE(ReadSegment((fs::path(run) / "000000.mbj").string(), &segment, &error)) << error;
  EXPECT_NE(segment.problem.find("broken block"), std::string::npos) << segment.problem;
}

TEST_F(RunIndexTest, FoldersThatAreNoRunsAreSkipped) {
  const std::string run = Finished(0.2);
  fs::create_directories(dir_ / "incidents" / "x");
  fs::copy_file(fs::path(run) / "000000.mbj", dir_ / "incidents" / "000000.mbj");
  fs::copy_file(fs::path(run) / "000000.mbj", dir_ / "incidents" / "x" / "000000.mbj");
  fs::create_directories(dir_ / "20260101T000000Z_empty_1");
  const auto runs = InspectRuns(dir_.string(), false);
  ASSERT_EQ(runs.size(), 1u);
  EXPECT_EQ(runs[0].dir, run);
}

TEST_F(RunIndexTest, InspectSegmentReadsHeadAndSize) {
  const std::string run = Finished(0.2);
  SegmentInfo info;
  std::string error;
  ASSERT_TRUE(InspectSegment(run + "/000000.mbj", &info, &error)) << error;
  EXPECT_EQ(info.index, 0u);
  EXPECT_GT(info.start_ns, 0);
  EXPECT_EQ(info.bytes, fs::file_size(run + "/000000.mbj"));
  std::ofstream(fs::path(run) / "000007.mbj") << "short";
  EXPECT_FALSE(InspectSegment(run + "/000007.mbj", &info, &error));
  EXPECT_EQ(info.index, 7u);
  EXPECT_EQ(info.bytes, 5u);
}

TEST_F(RunIndexTest, TheIndexReadsWhatNoLongerChangesOnce) {
  options_.segment_s = 0.15;
  RunIndex index(dir_.string());
  std::string error;
  auto journal = Journal::Create(options_, &error);
  ASSERT_TRUE(journal) << error;
  const std::string run = journal->GetStats().run_dir;
  Feed(*journal, 0.4);
  const size_t first = index.Refresh(true).at(0).segments.size();
  EXPECT_GE(first, 2u);
  EXPECT_EQ(index.heads_read(), first);
  EXPECT_TRUE(index.Refresh(true).at(0).active);

  Feed(*journal, 0.4);
  uint64_t heads = index.heads_read();
  const size_t second = index.Refresh(true).at(0).segments.size();
  EXPECT_GT(second, first);
  // The new segments, and the open one again only if its head was not written yet.
  EXPECT_GE(index.heads_read() - heads, second - first);
  EXPECT_LE(index.heads_read() - heads, second - first + 1);

  journal.reset();
  index.Refresh(true);
  heads = index.heads_read();
  const RunInfo done = index.Refresh(true).at(0);
  EXPECT_FALSE(done.active);
  EXPECT_TRUE(done.ended);
  EXPECT_TRUE(done.segments.back().closed);
  EXPECT_TRUE(done.problem.empty()) << done.problem;
  EXPECT_EQ(index.heads_read(), heads);
  EXPECT_EQ(index.ends_read(), 1u);
  for (const auto& segment : done.segments) EXPECT_EQ(segment.bytes, fs::file_size(segment.path));

  // Another run appears and goes again.
  const fs::path copy = dir_ / "20260101T000000Z_copy_1";
  fs::copy(run, copy);
  heads = index.heads_read();
  const auto& both = index.Refresh(true);
  ASSERT_EQ(both.size(), 2u);
  EXPECT_EQ(both[0].dir, copy.string());
  EXPECT_TRUE(both[0].ended);
  EXPECT_EQ(index.heads_read() - heads, both[0].segments.size());
  EXPECT_EQ(index.ends_read(), 2u);
  fs::remove_all(copy);
  const auto& one = index.Refresh(true);
  ASSERT_EQ(one.size(), 1u);
  EXPECT_EQ(one[0].dir, run);
  EXPECT_EQ(index.ends_read(), 2u);
}

}  // namespace
}  // namespace mowbite_journal
