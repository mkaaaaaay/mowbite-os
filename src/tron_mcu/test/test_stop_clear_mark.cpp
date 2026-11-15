#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "tron_mcu/stop_clear_mark.h"

using tron::StopClearMark;

namespace {

class StopClearMarkTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char pattern[] = "/tmp/tron_mark_XXXXXX";
    ASSERT_NE(mkdtemp(pattern), nullptr);
    dir_ = pattern;
  }
  void TearDown() override {
    unlink((dir_ + "/stop-clear-mark").c_str());
    unlink((dir_ + "/file").c_str());
    rmdir(dir_.c_str());
  }

  std::string dir_;
  std::string error_;
};

TEST_F(StopClearMarkTest, WrittenItIsPresentRemovedItIsAbsent) {
  StopClearMark mark(dir_ + "/stop-clear-mark");
  EXPECT_EQ(mark.Check(error_), StopClearMark::State::kAbsent);
  ASSERT_TRUE(mark.Write(error_)) << error_;
  EXPECT_EQ(mark.Check(error_), StopClearMark::State::kPresent);
  std::ifstream in(mark.path());
  std::stringstream text;
  text << in.rdbuf();
  EXPECT_EQ(text.str(), "STOP cleared on the MCU during an open emergency\n");
  // Written again it stays one file.
  ASSERT_TRUE(mark.Write(error_)) << error_;
  ASSERT_TRUE(mark.Remove(error_)) << error_;
  EXPECT_EQ(mark.Check(error_), StopClearMark::State::kAbsent);
  // Gone already counts as removed.
  EXPECT_TRUE(mark.Remove(error_)) << error_;
}

TEST_F(StopClearMarkTest, AnUnreadablePlaceIsUnknownAndCannotBeWritten) {
  std::ofstream(dir_ + "/file") << "x";
  StopClearMark mark(dir_ + "/file/stop-clear-mark");
  EXPECT_EQ(mark.Check(error_), StopClearMark::State::kUnknown);
  EXPECT_NE(error_.find("lstat"), std::string::npos);
  EXPECT_FALSE(mark.Write(error_));
}

TEST_F(StopClearMarkTest, AMissingDirectoryIsAbsentAndCannotBeWritten) {
  StopClearMark mark(dir_ + "/missing/stop-clear-mark");
  EXPECT_EQ(mark.Check(error_), StopClearMark::State::kAbsent);
  EXPECT_FALSE(mark.Write(error_));
  EXPECT_NE(error_.find("open"), std::string::npos);
}

}  // namespace
