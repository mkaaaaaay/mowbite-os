#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "mowbite_hold/hold.h"

namespace mowbite_hold {
namespace {

constexpr int64_t kNow = 1800000000;

class HoldDir : public ::testing::Test {
 protected:
  void SetUp() override {
    char pattern[] = "/tmp/mowbite_hold_XXXXXX";
    ASSERT_NE(::mkdtemp(pattern), nullptr);
    dir_ = pattern;
    path_ = dir_ + "/hold.json";
  }
  void TearDown() override {
    std::filesystem::permissions(dir_, std::filesystem::perms::owner_all, std::filesystem::perm_options::add);
    std::filesystem::remove_all(dir_);
  }
  void Put(const std::string& text) {
    // Written next to it and renamed, like the logic does: a new file each time.
    std::ofstream(path_ + ".new") << text;
    std::filesystem::rename(path_ + ".new", path_);
  }
  std::string dir_;
  std::string path_;
};

TEST(ParseHold, TakesBothTimes) {
  const auto record = ParseHold(R"({"until": 1800000600, "set_at": 1800000000})");
  ASSERT_TRUE(record);
  EXPECT_EQ(record->until, 1800000600);
  EXPECT_EQ(record->set_at, 1800000000);
}

TEST(ParseHold, IgnoresOtherKeys) {
  EXPECT_TRUE(ParseHold(R"({"until": 5, "set_at": 1, "by": "updater"})"));
}

TEST(ParseHold, RefusesWhatIsNoHold) {
  for (const char* text :
       {"", "{}", "[]", "null", "1800000600", R"({"until": 1800000600})", R"({"set_at": 1800000000})",
        R"({"until": 1800000600.5, "set_at": 1})", R"({"until": "1800000600", "set_at": 1})",
        R"({"until": true, "set_at": 1})", R"({"until": -5, "set_at": 1})", R"({"until": 0, "set_at": 1})",
        R"({"until": 5, "set_at": -1})", R"({"until": 18446744073709551615, "set_at": 1})",
        R"({"until": 1800000600, "set_at": 1)"}) {
    EXPECT_FALSE(ParseHold(text)) << text;
  }
}

TEST(ParseHold, ReadsItsOwnText) {
  const HoldRecord record{1800000600, 1800000000};
  const auto back = ParseHold(HoldText(record));
  ASSERT_TRUE(back);
  EXPECT_EQ(back->until, record.until);
  EXPECT_EQ(back->set_at, record.set_at);
}

TEST(HoldSeconds, IsTheRestWithinTheLimit) {
  EXPECT_EQ(HoldSeconds({kNow + 600, kNow}, kNow), 600);
  EXPECT_EQ(HoldSeconds({kNow + 600, kNow}, kNow + 600), 0);
  EXPECT_EQ(HoldSeconds({kNow + 600, kNow}, kNow + 900), 0);
  // A clock that went back, or a wrong one after a power loss, never makes a hold longer than the limit.
  EXPECT_EQ(HoldSeconds({kNow + 600, kNow}, kNow - 86400), kMaxHoldSeconds);
  EXPECT_EQ(HoldSeconds({kNow + 86400, kNow}, kNow), kMaxHoldSeconds);
}

TEST(HoldTimer, EndsOnTheMonotonicClock) {
  HoldTimer timer;
  EXPECT_FALSE(timer.Active(0));
  EXPECT_EQ(timer.Until(0), 0);
  timer.Start({kNow + 600, kNow}, 100.0, kNow);
  EXPECT_TRUE(timer.Active(100.0));
  EXPECT_EQ(timer.Until(100.0), kNow + 600);
  EXPECT_TRUE(timer.Active(699.9));
  EXPECT_FALSE(timer.Active(700.0));
  EXPECT_EQ(timer.Until(700.0), 0);
  // The record stays until it is cleared, so the owner can tell an ended hold from none.
  EXPECT_TRUE(timer.record());
  timer.Clear();
  EXPECT_FALSE(timer.record());
  EXPECT_FALSE(timer.Active(100.0));
}

TEST(HoldTimer, AnUntilInThePastEndsAtOnce) {
  HoldTimer timer;
  timer.Start({kNow - 1, kNow - 100}, 50.0, kNow);
  EXPECT_FALSE(timer.Active(50.0));
}

TEST_F(HoldDir, WritesReadsAndRemoves) {
  EXPECT_EQ(ReadHoldFile(path_).state, HoldFile::State::kMissing);
  std::string error;
  ASSERT_TRUE(WriteHoldFile(path_, {kNow + 60, kNow}, &error)) << error;
  const HoldFile file = ReadHoldFile(path_);
  ASSERT_EQ(file.state, HoldFile::State::kRecord);
  EXPECT_EQ(file.record.until, kNow + 60);
  EXPECT_EQ(file.record.set_at, kNow);
  EXPECT_FALSE(std::filesystem::exists(path_ + ".tmp"));
  struct stat st {};
  ASSERT_EQ(::stat(path_.c_str(), &st), 0);
  // Readable for the drivers, written by the logic only (0644 less the umask).
  EXPECT_EQ(st.st_mode & 0133, 0u);
  EXPECT_NE(st.st_mode & 0400, 0u);
  ASSERT_TRUE(RemoveHoldFile(path_, &error)) << error;
  EXPECT_FALSE(std::filesystem::exists(path_));
  // Not there counts as removed.
  EXPECT_TRUE(RemoveHoldFile(path_, &error)) << error;
}

TEST_F(HoldDir, AWriteReplacesTheHold) {
  std::string error;
  ASSERT_TRUE(WriteHoldFile(path_, {kNow + 60, kNow}, &error));
  ASSERT_TRUE(WriteHoldFile(path_, {kNow + 900, kNow + 10}, &error));
  EXPECT_EQ(ReadHoldFile(path_).record.until, kNow + 900);
}

TEST_F(HoldDir, AFailedWriteLeavesNothing) {
  std::string error;
  EXPECT_FALSE(WriteHoldFile(dir_ + "/missing/hold.json", {kNow + 60, kNow}, &error));
  EXPECT_NE(error.find("cannot write"), std::string::npos);
}

TEST_F(HoldDir, WhatIsNoHoldIsUnreadable) {
  Put("not json");
  EXPECT_EQ(ReadHoldFile(path_).state, HoldFile::State::kUnreadable);
  Put(std::string(5000, ' ') + R"({"until": 5, "set_at": 1})");
  const HoldFile large = ReadHoldFile(path_);
  EXPECT_EQ(large.state, HoldFile::State::kUnreadable);
  EXPECT_NE(large.error.find("larger"), std::string::npos);
  std::filesystem::remove(path_);
  std::filesystem::create_directory(path_);
  EXPECT_EQ(ReadHoldFile(path_).state, HoldFile::State::kUnreadable);
}

TEST_F(HoldDir, AFileWithoutReadRightsIsUnreadable) {
  if (::geteuid() == 0) GTEST_SKIP() << "root reads any file";
  std::string error;
  ASSERT_TRUE(WriteHoldFile(path_, {kNow + 60, kNow}, &error));
  std::filesystem::permissions(path_, std::filesystem::perms::none);
  EXPECT_EQ(ReadHoldFile(path_).state, HoldFile::State::kUnreadable);
}

TEST_F(HoldDir, WatchWithoutPathNeverHolds) {
  HoldWatch watch("");
  EXPECT_FALSE(watch.Poll(0, kNow).held);
}

TEST_F(HoldDir, WatchHoldsWhileTheFileSays) {
  HoldWatch watch(path_);
  EXPECT_FALSE(watch.Poll(0, kNow).held);
  std::string error;
  ASSERT_TRUE(WriteHoldFile(path_, {kNow + 60, kNow}, &error));
  HoldView view = watch.Poll(1, kNow + 1);
  EXPECT_TRUE(view.held);
  EXPECT_FALSE(view.unreadable);
  EXPECT_EQ(view.until, kNow + 60);
  EXPECT_TRUE(watch.Poll(59.9, kNow + 59).held);
  // Ended on the monotonic clock, while the file is still there.
  view = watch.Poll(60.1, kNow + 60);
  EXPECT_FALSE(view.held);
  EXPECT_EQ(view.until, 0);
  ASSERT_TRUE(RemoveHoldFile(path_, &error));
  EXPECT_FALSE(watch.Poll(61, kNow + 61).held);
}

TEST_F(HoldDir, WatchReadsAnUnchangedFileOnce) {
  std::string error;
  ASSERT_TRUE(WriteHoldFile(path_, {kNow + 100, kNow}, &error));
  HoldWatch watch(path_);
  EXPECT_TRUE(watch.Poll(0, kNow).held);
  // The wall clock stands still (or went back): read again, the hold would start over. It is not read again.
  EXPECT_FALSE(watch.Poll(100.5, kNow).held);
  EXPECT_FALSE(watch.Poll(5000, kNow).held);
}

TEST_F(HoldDir, WatchTakesANewHold) {
  std::string error;
  ASSERT_TRUE(WriteHoldFile(path_, {kNow + 10, kNow}, &error));
  HoldWatch watch(path_);
  EXPECT_TRUE(watch.Poll(0, kNow).held);
  EXPECT_FALSE(watch.Poll(11, kNow + 11).held);
  ASSERT_TRUE(WriteHoldFile(path_, {kNow + 1000, kNow + 20}, &error));
  const HoldView view = watch.Poll(20, kNow + 20);
  EXPECT_TRUE(view.held);
  EXPECT_EQ(view.until, kNow + 1000);
  EXPECT_TRUE(watch.Poll(999, kNow + 999).held);
  EXPECT_FALSE(watch.Poll(1000, kNow + 1000).held);
}

TEST_F(HoldDir, WatchAfterARestartHoldsAtOnce) {
  std::string error;
  ASSERT_TRUE(WriteHoldFile(path_, {kNow + 600, kNow - 300}, &error));
  // A driver that starts with the file there: held from its first poll, for what is left.
  HoldWatch watch(path_);
  const HoldView view = watch.Poll(5, kNow);
  EXPECT_TRUE(view.held);
  EXPECT_TRUE(watch.Poll(604.9, kNow + 599).held);
  EXPECT_FALSE(watch.Poll(605, kNow + 600).held);
}

TEST_F(HoldDir, WatchCapsAWrongClock) {
  std::string error;
  // Written with a clock a day ahead, read after a power loss with the right one.
  ASSERT_TRUE(WriteHoldFile(path_, {kNow + 86400 + 600, kNow + 86400}, &error));
  HoldWatch watch(path_);
  EXPECT_TRUE(watch.Poll(0, kNow).held);
  EXPECT_TRUE(watch.Poll(kMaxHoldSeconds - 0.1, kNow).held);
  EXPECT_FALSE(watch.Poll(kMaxHoldSeconds, kNow).held);
}

TEST_F(HoldDir, WatchHoldsWhatItCannotRead) {
  HoldWatch watch(path_);
  Put("{\"until\": ");
  HoldView view = watch.Poll(0, kNow);
  EXPECT_TRUE(view.held);
  EXPECT_TRUE(view.unreadable);
  EXPECT_FALSE(view.error.empty());
  // No time limit for a file it cannot read: held until the file changes or goes.
  EXPECT_TRUE(watch.Poll(1e6, kNow + 1000000).held);
  Put(R"({"until": 1800000060, "set_at": 1800000000})");
  view = watch.Poll(1e6 + 1, kNow);
  EXPECT_TRUE(view.held);
  EXPECT_FALSE(view.unreadable);
  EXPECT_EQ(view.until, kNow + 60);
  Put("garbage");
  EXPECT_TRUE(watch.Poll(1e6 + 2, kNow).unreadable);
  std::filesystem::remove(path_);
  view = watch.Poll(1e6 + 3, kNow);
  EXPECT_FALSE(view.held);
  EXPECT_FALSE(view.unreadable);
}

TEST_F(HoldDir, WatchHoldsWhenItCannotLook) {
  // A file where the folder should be: stat fails with another error than "not there".
  std::ofstream(dir_ + "/plain") << "x";
  HoldWatch watch(dir_ + "/plain/hold.json");
  const HoldView view = watch.Poll(0, kNow);
  EXPECT_TRUE(view.held);
  EXPECT_TRUE(view.unreadable);
  EXPECT_NE(view.error.find("cannot stat"), std::string::npos);
}

TEST_F(HoldDir, WatchHoldsAFolderInItsPlace) {
  std::filesystem::create_directory(path_);
  HoldWatch watch(path_);
  EXPECT_TRUE(watch.Poll(0, kNow).unreadable);
}

}  // namespace
}  // namespace mowbite_hold
