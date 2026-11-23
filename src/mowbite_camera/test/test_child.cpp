#include <gtest/gtest.h>

#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "mowbite_camera/child.h"
#include "test_util.h"

namespace mowbite_camera {
namespace {

namespace fs = std::filesystem;

ChildSpec Shell(const std::string& script) {
  ChildSpec spec;
  spec.program = "/bin/sh";
  spec.args = {"-c", script};
  return spec;
}

// How the child ended, waiting about 5 s for it at most.
std::string EndOf(Child& child) {
  std::string how = "still running";
  for (int round = 0; round < 500 && child.Running(&how); ++round) ::usleep(10000);
  return how;
}

bool WaitForFile(const fs::path& path) {
  for (int round = 0; round < 500 && !fs::exists(path); ++round) ::usleep(10000);
  return fs::exists(path);
}

class ChildTest : public ::testing::Test {
 protected:
  TempDir temp_;
  const fs::path& dir_ = temp_.path();
  std::string error_;
  Child child_;
};

TEST_F(ChildTest, TellsHowItsProgramEnded) {
  EXPECT_FALSE(child_.Running());
  ASSERT_TRUE(child_.Start(Shell("exit 3"), &error_)) << error_;
  EXPECT_EQ(EndOf(child_), "exit code 3");
  EXPECT_FALSE(child_.Running());
  // What cannot be run ends like a program, and the next one may start.
  ChildSpec missing;
  missing.program = "/nowhere/program";
  ASSERT_TRUE(child_.Start(missing, &error_)) << error_;
  EXPECT_EQ(EndOf(child_), "exit code 127");
}

TEST_F(ChildTest, OnlyOneAtATime) {
  ASSERT_TRUE(child_.Start(Shell("exec sleep 30"), &error_)) << error_;
  EXPECT_FALSE(child_.Start(Shell("exit 0"), &error_));
  EXPECT_NE(error_, "");
  EXPECT_TRUE(child_.Running());
}

TEST_F(ChildTest, GetsItsEnvironmentItsDirectoryAndItsLimits) {
  ::setenv("MOWBITE_CHILD_TEST", "replaced", 1);
  ChildSpec spec = Shell("echo \"$GOMEMLIMIT $MOWBITE_CHILD_TEST $PATH\" > out; pwd >> out; nice >> out; "
                         "cat /proc/self/oom_score_adj >> out; mv out done");
  spec.environment = {"GOMEMLIMIT=64MiB", "MOWBITE_CHILD_TEST=set"};
  spec.directory = dir_.string();
  spec.nice = 7;
  spec.oom_score_adj = 1000;
  ASSERT_TRUE(child_.Start(spec, &error_)) << error_;
  EXPECT_EQ(EndOf(child_), "exit code 0");
  std::ifstream file(dir_ / "done");
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(file, line)) lines.push_back(line);
  const int nice = std::max(7, ::getpriority(PRIO_PROCESS, 0));
  const std::vector<std::string> expected = {std::string("64MiB set ") + ::getenv("PATH"), dir_.string(),
                                             std::to_string(nice), "1000"};
  EXPECT_EQ(lines, expected);
  ::unsetenv("MOWBITE_CHILD_TEST");
}

TEST_F(ChildTest, HasNoDescriptorOfItsParentAndNoInput) {
  int fds[2];
  ASSERT_EQ(::pipe(fds), 0);
  const std::string inherited = "/proc/self/fd/" + std::to_string(fds[1]);
  ASSERT_TRUE(child_.Start(Shell("[ -e " + inherited + " ] && exit 5; "
                                 "[ \"$(readlink /proc/self/fd/0)\" = /dev/null ] || exit 6; exit 0"),
                           &error_));
  EXPECT_EQ(EndOf(child_), "exit code 0");
  ::close(fds[0]);
  ::close(fds[1]);
}

TEST_F(ChildTest, StopEndsItWithSigint) {
  ASSERT_TRUE(child_.Start(Shell("exec sleep 30"), &error_)) << error_;
  const auto before = std::chrono::steady_clock::now();
  child_.Stop(5.0);
  EXPECT_LT(std::chrono::steady_clock::now() - before, std::chrono::seconds(2));
  EXPECT_FALSE(child_.Running());
}

TEST_F(ChildTest, StopEndsWhatIgnoresSigintAfterTheGrace) {
  ChildSpec spec = Shell("trap '' INT; touch ready; exec sleep 30");
  spec.directory = dir_.string();
  ASSERT_TRUE(child_.Start(spec, &error_)) << error_;
  ASSERT_TRUE(WaitForFile(dir_ / "ready"));
  const auto before = std::chrono::steady_clock::now();
  child_.Stop(0.3);
  const auto took = std::chrono::steady_clock::now() - before;
  EXPECT_GE(took, std::chrono::milliseconds(300));
  EXPECT_LT(took, std::chrono::seconds(3));
  EXPECT_FALSE(child_.Running());
}

// Interrupt returns at once; what ignores SIGINT ends with the first look after the grace.
TEST_F(ChildTest, InterruptWaitsForNothing) {
  ChildSpec spec = Shell("trap '' INT; touch ready; exec sleep 30");
  spec.directory = dir_.string();
  ASSERT_TRUE(child_.Start(spec, &error_)) << error_;
  ASSERT_TRUE(WaitForFile(dir_ / "ready"));
  const auto before = std::chrono::steady_clock::now();
  EXPECT_FALSE(child_.Interrupted());
  child_.Interrupt(0.3);
  EXPECT_TRUE(child_.Interrupted());
  EXPECT_LT(std::chrono::steady_clock::now() - before, std::chrono::milliseconds(100));
  EXPECT_TRUE(child_.Running());
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  std::string how;
  EXPECT_FALSE(child_.Running(&how));
  EXPECT_EQ(how, "signal 9");
  EXPECT_FALSE(child_.Interrupted());
}

TEST_F(ChildTest, EndsWithTheThreadThatStartedIt) {
  // The thread stays until the program runs: the child asks for the signal as its first step, before that.
  std::thread starter([this] {
    ChildSpec spec = Shell("touch ready; exec sleep 30");
    spec.directory = dir_.string();
    ASSERT_TRUE(child_.Start(spec, &error_)) << error_;
    ASSERT_TRUE(WaitForFile(dir_ / "ready"));
  });
  starter.join();
  EXPECT_EQ(EndOf(child_), "signal 9");
}

}  // namespace
}  // namespace mowbite_camera
