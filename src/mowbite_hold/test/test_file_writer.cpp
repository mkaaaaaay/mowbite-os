#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_hold/file_io.h"
#include "mowbite_hold/file_writer.h"

namespace mowbite_hold {
namespace {

using std::chrono::milliseconds;
using std::chrono::seconds;
using Wait = FileWriter::Wait;

// A disk that holds every write until it is let go, and remembers what it wrote.
class Disk {
 public:
  bool Write(const std::string& path, const std::string& text, std::string* error) {
    std::unique_lock<std::mutex> lock(mutex_);
    ++started_;
    changed_.notify_all();
    changed_.wait(lock, [&] { return released_ > 0; });
    --released_;
    written_.emplace_back(path, text);
    if (fail_) *error = "full";
    return !fail_;
  }
  void Release(int writes = 1) {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ += writes;
    changed_.notify_all();
  }
  void WaitStarted(int writes) {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [&] { return started_ >= writes; });
  }
  std::vector<std::pair<std::string, std::string>> Written() {
    std::lock_guard<std::mutex> lock(mutex_);
    return written_;
  }
  void Fail() {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_ = true;
  }
  void Heal() {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_ = false;
  }

 private:
  std::mutex mutex_;
  std::condition_variable changed_;
  int started_ = 0;
  int released_ = 0;
  bool fail_ = false;
  std::vector<std::pair<std::string, std::string>> written_;
};

FileWriter::Write On(Disk* disk) {
  return [disk](const std::string& path, const std::string& text, std::string* error) {
    return disk->Write(path, text, error);
  };
}

TEST(FileWriter, OnlyTheLatestTextOfAFileInTheOrderOfTheLatestTexts) {
  Disk disk;
  {
    FileWriter writer(On(&disk), [](const FileWriter::Done&) {});
    writer.Submit("session", "s1");
    disk.WaitStarted(1);
    // While s1 is on its way: s2 is replaced by s3 and goes behind the checkpoint.
    writer.Submit("session", "s2");
    writer.Submit("checkpoint", "c1");
    writer.Submit("session", "s3");
    disk.Release(3);
  }
  using W = std::vector<std::pair<std::string, std::string>>;
  EXPECT_EQ(disk.Written(), (W{{"session", "s1"}, {"checkpoint", "c1"}, {"session", "s3"}}));
}

TEST(FileWriter, WaitForReturnsOnceTheFileIsWrittenAndReported) {
  Disk disk;
  std::atomic<int> reports{0};
  FileWriter writer(On(&disk), [&](const FileWriter::Done& done) {
    EXPECT_TRUE(done.ok);
    ++reports;
  });
  writer.Submit("checkpoint", "c1");
  writer.Submit("session", "s1");
  EXPECT_EQ(writer.WaitFor("session", milliseconds(50)), Wait::kTimeout);
  disk.Release();
  // The checkpoint ahead of it is written, the session not yet.
  EXPECT_EQ(writer.WaitFor("session", milliseconds(50)), Wait::kTimeout);
  EXPECT_EQ(writer.WaitFor("checkpoint", seconds(5)), Wait::kWritten);
  disk.Release();
  EXPECT_EQ(writer.WaitFor("session", seconds(5)), Wait::kWritten);
  EXPECT_EQ(reports, 2);
  // Nothing written for it: no write went through.
  EXPECT_EQ(writer.WaitFor("unknown", milliseconds(0)), Wait::kFailed);
}

// The wait tells how the last write of the file went, so the logic acts only on a record that is on the disk.
TEST(FileWriter, WaitForTellsTheLastWriteOfTheFile) {
  Disk disk;
  FileWriter writer(On(&disk), [](const FileWriter::Done&) {});
  disk.Release(4);
  writer.Submit("checkpoint", "c1");
  ASSERT_EQ(writer.WaitFor("checkpoint", seconds(5)), Wait::kWritten);
  disk.Fail();
  writer.Submit("session", "s1");
  EXPECT_EQ(writer.WaitFor("session", seconds(5)), Wait::kFailed);
  // Another file is not touched by it.
  EXPECT_EQ(writer.WaitFor("checkpoint", milliseconds(0)), Wait::kWritten);
  disk.Heal();
  writer.Submit("session", "s2");
  EXPECT_EQ(writer.WaitFor("session", seconds(5)), Wait::kWritten);
  disk.Fail();
  writer.Submit("session", "s3");
  EXPECT_EQ(writer.WaitFor("session", seconds(5)), Wait::kFailed);
}

TEST(FileWriter, BusyWhileAWriteRuns) {
  Disk disk;
  FileWriter writer(On(&disk), [](const FileWriter::Done&) {});
  EXPECT_EQ(writer.Busy().count(), 0);
  writer.Submit("session", "s1");
  disk.WaitStarted(1);
  std::this_thread::sleep_for(milliseconds(20));
  EXPECT_GE(writer.Busy(), milliseconds(20));
  disk.Release();
  ASSERT_EQ(writer.WaitFor("session", seconds(5)), Wait::kWritten);
  EXPECT_EQ(writer.Busy().count(), 0);
}

TEST(FileWriter, ReportsAFailureWithItsTime) {
  Disk disk;
  disk.Fail();
  std::vector<FileWriter::Done> reports;
  std::mutex mutex;
  {
    FileWriter writer(On(&disk), [&](const FileWriter::Done& done) {
      std::lock_guard<std::mutex> lock(mutex);
      reports.push_back(done);
    });
    writer.Submit("session", "s1");
    disk.WaitStarted(1);
    std::this_thread::sleep_for(milliseconds(10));
    disk.Release();
  }
  ASSERT_EQ(reports.size(), 1u);
  EXPECT_EQ(reports[0].path, "session");
  EXPECT_FALSE(reports[0].ok);
  EXPECT_EQ(reports[0].error, "full");
  EXPECT_GE(reports[0].took, milliseconds(10));
}

TEST(FileWriter, TheEndWritesWhatIsQueued) {
  Disk disk;
  disk.Release(10);
  {
    FileWriter writer(On(&disk), [](const FileWriter::Done&) {});
    writer.Submit("a", "1");
    writer.Submit("b", "2");
    writer.Submit("c", "3");
  }
  EXPECT_EQ(disk.Written().size(), 3u);
}

TEST(FileWriter, WritesRealFilesWhole) {
  const std::filesystem::path folder = std::filesystem::temp_directory_path() / "mowbite_file_writer_test";
  std::filesystem::remove_all(folder);
  std::filesystem::create_directories(folder);
  const std::string path = (folder / "os-session.json").string();
  {
    FileWriter writer(WriteFileAtomic, [](const FileWriter::Done& done) { EXPECT_TRUE(done.ok) << done.error; });
    for (int i = 0; i < 50; ++i) writer.Submit(path, "text " + std::to_string(i) + "\n");
    ASSERT_EQ(writer.WaitFor(path, seconds(10)), Wait::kWritten);
  }
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);
  EXPECT_EQ(line, "text 49");
  EXPECT_FALSE(std::filesystem::exists(path + ".tmp"));
  std::filesystem::remove_all(folder);
}

}  // namespace
}  // namespace mowbite_hold
