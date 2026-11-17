#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "mowbite_hold/lock.h"

namespace mowbite_hold {
namespace {

class LockDir : public ::testing::Test {
 protected:
  void SetUp() override {
    char pattern[] = "/tmp/mowbite_lock_XXXXXX";
    ASSERT_NE(::mkdtemp(pattern), nullptr);
    dir_ = pattern;
    path_ = dir_ + "/" + kSafetyLockFile;
  }
  void TearDown() override {
    std::filesystem::permissions(dir_, std::filesystem::perms::owner_all, std::filesystem::perm_options::add);
    std::filesystem::remove_all(dir_);
  }
  void Put(const std::string& text) {
    std::ofstream(path_ + ".new") << text;
    std::filesystem::rename(path_ + ".new", path_);
  }
  std::string dir_;
  std::string path_;
};

LockRecord Failed(uint32_t failures) {
  LockRecord record;
  record.failures = failures;
  LockCause cause;
  cause.step = "stop_moving";
  cause.part = "stop";
  cause.reason = "LIMIT";
  cause.value = 0.62;
  cause.limit = 0.5;
  cause.t = 1800000000;
  cause.firmware = "mainboard 1.2";
  record.causes.push_back(cause);
  return record;
}

TEST(ParseSafetyLock, ReadsItsOwnText) {
  LockRecord record = Failed(3);
  LockCause reset;
  reset.reset = true;
  reset.reason = "UNREADABLE";
  reset.t = 1800000100;
  record.causes.push_back(reset);
  const auto read = ParseSafetyLock(SafetyLockText(record));
  ASSERT_TRUE(read);
  EXPECT_EQ(read->failures, 3u);
  ASSERT_EQ(read->causes.size(), 2u);
  EXPECT_EQ(read->causes[0].step, "stop_moving");
  EXPECT_EQ(read->causes[0].part, "stop");
  EXPECT_EQ(read->causes[0].reason, "LIMIT");
  EXPECT_DOUBLE_EQ(*read->causes[0].value, 0.62);
  EXPECT_DOUBLE_EQ(*read->causes[0].limit, 0.5);
  EXPECT_EQ(read->causes[0].t, 1800000000);
  EXPECT_EQ(read->causes[0].firmware, "mainboard 1.2");
  EXPECT_FALSE(read->causes[0].reset);
  EXPECT_TRUE(read->causes[1].reset);
  EXPECT_EQ(read->causes[1].reason, "UNREADABLE");
}

TEST(ParseSafetyLock, TakesACauseWithoutNumbers) {
  const auto read = ParseSafetyLock(R"({"failures": 1, "causes": [{"step": "imu", "reason": "NOT_SEEN",
                                       "value": null, "limit": null}]})");
  ASSERT_TRUE(read);
  EXPECT_FALSE(read->causes[0].value);
  EXPECT_FALSE(read->causes[0].limit);
}

TEST(ParseSafetyLock, RefusesWhatIsNoLock) {
  for (const char* text :
       {"", "{}", "[]", "null", R"({"failures": 1})", R"({"causes": [{"step": "imu", "reason": "LIMIT"}]})",
        R"({"failures": 1, "causes": []})", R"({"failures": -1, "causes": [{"step": "imu", "reason": "LIMIT"}]})",
        R"({"failures": 1.5, "causes": [{"step": "imu", "reason": "LIMIT"}]})",
        R"({"failures": 4294967296, "causes": [{"step": "imu", "reason": "LIMIT"}]})",
        R"({"failures": 1, "causes": [{"reason": "LIMIT"}]})", R"({"failures": 1, "causes": [{"step": "imu"}]})",
        R"({"failures": 1, "causes": [{"step": "imu", "reason": 3}]})",
        R"({"failures": 1, "causes": [{"step": "imu", "reason": "LIMIT", "value": "1"}]})",
        R"({"failures": 1, "causes": [{"step": "imu", "reason": "LIMIT", "t": 1.5}]})",
        R"({"failures": 1, "causes": [{"reset": false, "reason": "RESET"}]})",
        R"({"failures": 1, "causes": [{"reset": true}]})", R"({"failures": 1, "causes": ["imu"]})",
        R"({"failures": 1, "causes": [{"step": "imu", "reason": "LIMIT"}])"}) {
    EXPECT_FALSE(ParseSafetyLock(text)) << text;
  }
}

TEST(SafetyMeasured, NeedsTheDateAndNoClosedLock) {
  SafetyLock open;
  SafetyLock closed;
  closed.closed = true;
  EXPECT_TRUE(SafetyMeasured("2026-11-17", open));
  EXPECT_FALSE(SafetyMeasured("", open));
  EXPECT_FALSE(SafetyMeasured("2026-11-17", closed));
  EXPECT_FALSE(SafetyMeasured("", closed));
}

TEST_F(LockDir, AMissingFileIsOpen) {
  const SafetyLock lock = ReadSafetyLock(path_);
  EXPECT_FALSE(lock.closed);
  EXPECT_FALSE(lock.unreadable);
  EXPECT_EQ(lock.failures, 0u);
}

TEST_F(LockDir, ACauseCloses) {
  Put(SafetyLockText(Failed(2)));
  const SafetyLock lock = ReadSafetyLock(path_);
  EXPECT_TRUE(lock.closed);
  EXPECT_FALSE(lock.unreadable);
  EXPECT_EQ(lock.failures, 2u);
  EXPECT_NE(lock.text.find("stop_moving"), std::string::npos) << lock.text;
}

TEST_F(LockDir, WhatCannotBeReadCloses) {
  for (const std::string& text : {std::string("{"), std::string(R"({"failures": 1, "causes": []})"),
                                  std::string(70000, ' ')}) {
    Put(text);
    const SafetyLock lock = ReadSafetyLock(path_);
    EXPECT_TRUE(lock.closed) << text.size();
    EXPECT_TRUE(lock.unreadable) << text.size();
    EXPECT_EQ(lock.failures, 0u);
  }
}

TEST_F(LockDir, AFileWithoutReadRightsCloses) {
  if (::geteuid() == 0) GTEST_SKIP() << "root reads any file";
  Put(SafetyLockText(Failed(1)));
  ::chmod(path_.c_str(), 0);
  EXPECT_TRUE(ReadSafetyLock(path_).closed);
  EXPECT_TRUE(ReadSafetyLock(path_).unreadable);
}

TEST_F(LockDir, AFolderInItsPlaceCloses) {
  std::filesystem::create_directory(path_);
  EXPECT_TRUE(ReadSafetyLock(path_).closed);
}

TEST_F(LockDir, WatchWithoutPathIsOpen) {
  SafetyLockWatch watch("");
  EXPECT_FALSE(watch.Poll().closed);
}

TEST_F(LockDir, WatchFollowsTheFile) {
  SafetyLockWatch watch(path_);
  EXPECT_FALSE(watch.Poll().closed);
  Put(SafetyLockText(Failed(1)));
  EXPECT_TRUE(watch.Poll().closed);
  EXPECT_EQ(watch.last().failures, 1u);
  Put(SafetyLockText(Failed(2)));
  EXPECT_EQ(watch.Poll().failures, 2u);
  std::filesystem::remove(path_);
  EXPECT_FALSE(watch.Poll().closed);
  EXPECT_EQ(watch.last().failures, 0u);
}

TEST_F(LockDir, WatchReadsAnUnchangedFileOnce) {
  Put(SafetyLockText(Failed(1)));
  SafetyLockWatch watch(path_);
  EXPECT_TRUE(watch.Poll().closed);
  // Taking the read right away does not open it: the file did not change, so it is not read again.
  if (::geteuid() != 0) {
    ::chmod(path_.c_str(), 0);
    EXPECT_TRUE(watch.Poll().closed);
    EXPECT_FALSE(watch.last().unreadable);
  }
}

TEST_F(LockDir, WatchClosesWhenItCannotLook) {
  if (::geteuid() == 0) GTEST_SKIP() << "root looks into any folder";
  SafetyLockWatch watch(path_);
  ::chmod(dir_.c_str(), 0);
  EXPECT_TRUE(watch.Poll().closed);
  EXPECT_TRUE(watch.last().unreadable);
}

}  // namespace
}  // namespace mowbite_hold
