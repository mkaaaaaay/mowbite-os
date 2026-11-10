#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "mowbite_watch/watch.h"

namespace mowbite_watch {
namespace {

using std::chrono::duration;
using std::chrono::steady_clock;

// Short limits, so each case takes well under a second.
Options Short() {
  Options options;
  options.start_limit = 0.4;
  options.shutdown_limit = 0.3;
  options.period = 0.02;
  options.deadman = 0.3;
  options.end_deadman = 0.3;
  return options;
}

void Sleep(double seconds) {
  std::this_thread::sleep_for(duration<double>(seconds));
}

std::string Folder() {
  char path[] = "/tmp/mowbite_watch_XXXXXX";
  return mkdtemp(path);
}

std::string Read(const std::string& path) {
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

// The death tests fork (style fast): the watch thread starts only in the child, and paths made before stay the same.
using WatchDeath = ::testing::Test;

TEST_F(WatchDeath, ASignedNodeOutlivesTheKernelTimer) {
  EXPECT_EXIT(
      {
        auto* runtime = new Runtime(Short());
        Watch watch("mowbite_logic", 0.2, *runtime);
        // Ten times the kernel timer: only set again at every round does it not fire.
        for (int i = 0; i < 150; ++i) {
          watch.Sign();
          Sleep(0.02);
        }
        _exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST_F(WatchDeath, ASilentNodeEndsTheProcessWithItsLineAndNote) {
  const std::string note = Folder() + "/watch_exit.json";
  const auto start = steady_clock::now();
  EXPECT_EXIT(
      {
        Options options = Short();
        options.note = note;
        auto* runtime = new Runtime(options);
        Watch other("mowbite_map", 5.0, *runtime);
        Watch watch("mowbite_bridge", 0.2, *runtime);
        watch.Mark("MQTT input");
        watch.Sign();
        for (;;) {
          other.Sign();
          Sleep(0.02);
        }
      },
      ::testing::ExitedWithCode(kExitCode),
      "mowbite_watch: mowbite_bridge silent for 0\\.[23] s \\(limit 0\\.2 s\\), last 'MQTT input': ending the "
      "process");
  EXPECT_LT(duration<double>(steady_clock::now() - start).count(), 2.0);
  const std::string text = Read(note);
  EXPECT_NE(text.find("{\"node\":\"mowbite_bridge\",\"seconds\":0."), std::string::npos) << text;
  EXPECT_NE(text.find(",\"limit\":0.2}"), std::string::npos) << text;
}

TEST_F(WatchDeath, ANodeThatLeftNoLongerCounts) {
  EXPECT_EXIT(
      {
        Options options = Short();
        options.expected = {"mowbite_follower"};
        auto* runtime = new Runtime(options);
        { Watch follower("mowbite_follower", 0.1, *runtime); }
        Sleep(1.0);
        _exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST_F(WatchDeath, AnExpectedNodeThatNeverComesEndsTheProcess) {
  EXPECT_EXIT(
      {
        Options options = Short();
        options.expected = std::vector<std::string>({"mowbite_logic", "tron_gnss"});
        auto* runtime = new Runtime(options);
        Watch logic("mowbite_logic", 5.0, *runtime);
        for (;;) {
          logic.Sign();
          Sleep(0.02);
        }
      },
      ::testing::ExitedWithCode(kExitCode), "mowbite_watch: tron_gnss did not start within 0\\.4 s");
}

void Stall() {
  for (;;) pause();
}

TEST_F(WatchDeath, AStalledWatchThreadMeetsTheKernelTimerAlsoWithSignalsBlocked) {
  const auto start = steady_clock::now();
  EXPECT_EXIT(
      {
        sigset_t all;
        sigfillset(&all);
        pthread_sigmask(SIG_BLOCK, &all, nullptr);
        Options options = Short();
        options.before_judge = Stall;
        auto* runtime = new Runtime(options);
        Watch watch("mowbite_logic", 0.1, *runtime);
        Sleep(5.0);
        _exit(0);
      },
      ::testing::KilledBySignal(SIGKILL), "");
  EXPECT_LT(duration<double>(steady_clock::now() - start).count(), 2.0);
}

TEST_F(WatchDeath, AStalledWatchThreadMeetsAKernelTimerOfSecondsAsWell) {
  const auto start = steady_clock::now();
  EXPECT_EXIT(
      {
        Options options = Short();
        options.deadman = 1.2;
        options.before_judge = Stall;
        new Runtime(options);
        Sleep(5.0);
        _exit(0);
      },
      ::testing::KilledBySignal(SIGKILL), "");
  const double took = duration<double>(steady_clock::now() - start).count();
  EXPECT_GT(took, 1.0);
  EXPECT_LT(took, 3.0);
}

std::atomic<int> rounds{0};

void Count() {
  ++rounds;
}

TEST_F(WatchDeath, TheWatchJudgesOncePerPeriod) {
  EXPECT_EXIT(
      {
        Options options = Short();
        options.period = 0.1;
        options.before_judge = Count;
        new Runtime(options);
        Sleep(0.55);
        _exit(rounds >= 4 && rounds <= 7 ? 0 : 100 + rounds);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST_F(WatchDeath, AllPlacesTakenIsAnErrorAtTheNode) {
  EXPECT_EXIT(
      {
        auto* runtime = new Runtime(Short());
        for (int i = 0; i < kMaxPlaces; ++i) runtime->Enter("n" + std::to_string(i), 5.0);
        try {
          Watch one_more("one_more", 5.0, *runtime);
        } catch (const std::length_error&) {
          _exit(0);
        }
        _exit(1);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST_F(WatchDeath, TheNoteIsWrittenWholeAndReadable) {
  const std::string note = Folder() + "/watch_exit.json";
  // A rest of an earlier end next to it does not stay in the note.
  std::ofstream(note + ".tmp") << std::string(300, 'x');
  EXPECT_EXIT(
      {
        Options options = Short();
        options.note = note;
        auto* runtime = new Runtime(options);
        Watch watch("mowbite_map", 0.1, *runtime);
        Sleep(5.0);
        _exit(0);
      },
      ::testing::ExitedWithCode(kExitCode), "");
  const std::string text = Read(note);
  EXPECT_EQ(text.rfind("{\"node\":\"mowbite_map\",\"seconds\":0.", 0), 0u) << text;
  EXPECT_EQ(text.back(), '\n');
  EXPECT_EQ(text.find('x'), std::string::npos) << text;
  struct stat st {};
  ASSERT_EQ(stat(note.c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 07777, 0644u);
}

TEST_F(WatchDeath, ANoteThatBlocksDoesNotHoldTheEnd) {
  const std::string note = Folder() + "/watch_exit.json";
  ASSERT_EQ(mkfifo((note + ".tmp").c_str(), 0600), 0);
  const auto start = steady_clock::now();
  EXPECT_EXIT(
      {
        Options options = Short();
        options.note = note;
        // Only the timer set at the verdict ends it this soon.
        options.deadman = 2.0;
        auto* runtime = new Runtime(options);
        Watch watch("mowbite_safety", 0.1, *runtime);
        Sleep(5.0);
        _exit(0);
      },
      ::testing::KilledBySignal(SIGKILL), "mowbite_safety silent for");
  EXPECT_LT(duration<double>(steady_clock::now() - start).count(), 1.5);
}

TEST_F(WatchDeath, AfterTheShutdownSilenceDoesNotCountOnlyItsLimit) {
  const std::string note = Folder() + "/watch_exit.json";
  const auto start = steady_clock::now();
  EXPECT_EXIT(
      {
        Options options = Short();
        options.note = note;
        auto* runtime = new Runtime(options);
        Watch watch("mowbite_logic", 0.05, *runtime);
        runtime->Shutdown();
        Sleep(5.0);
        _exit(0);
      },
      ::testing::ExitedWithCode(kExitCode), "still running 0\\.[34] s after the shutdown \\(limit 0\\.3 s\\)");
  const double took = duration<double>(steady_clock::now() - start).count();
  EXPECT_GT(took, 0.3);
  EXPECT_LT(took, 2.0);
  // An end after the shutdown names no node.
  EXPECT_FALSE(std::ifstream(note).good());
  EXPECT_EXIT(
      {
        auto* runtime = new Runtime(Short());
        Watch watch("mowbite_logic", 0.05, *runtime);
        runtime->Shutdown();
        Sleep(0.2);
        _exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST_F(WatchDeath, TheFactorStretchesTheLimits) {
  EXPECT_EXIT(
      {
        Options options = Short();
        options.factor = 4.0;
        auto* runtime = new Runtime(options);
        Watch watch("mowbite_logic", 0.1, *runtime);
        Sleep(0.3);
        _exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST(WatchOptions, FromTheEnvironment) {
  std::string warning = "from before";
  unsetenv(kNodesEnv);
  unsetenv(kNoteEnv);
  unsetenv(kFactorEnv);
  Options options = OptionsFromEnvironment(&warning);
  EXPECT_TRUE(options.expected.empty());
  EXPECT_TRUE(options.note.empty());
  EXPECT_EQ(options.factor, 1.0);
  EXPECT_EQ(warning, "");
  setenv(kNodesEnv, "tron_gnss,,mowbite_logic,", 1);
  setenv(kNoteEnv, "/data/watch_exit.json", 1);
  setenv(kFactorEnv, "3", 1);
  options = OptionsFromEnvironment(&warning);
  EXPECT_EQ(options.expected, (std::vector<std::string>{"tron_gnss", "mowbite_logic"}));
  EXPECT_EQ(options.note, "/data/watch_exit.json");
  EXPECT_EQ(options.factor, 3.0);
  EXPECT_NE(warning.find("times 3"), std::string::npos) << warning;
  setenv(kNodesEnv, "a,b", 1);
  EXPECT_EQ(OptionsFromEnvironment(&warning).expected, (std::vector<std::string>{"a", "b"}));
  for (const char* edge : {"1", "100"}) {
    setenv(kFactorEnv, edge, 1);
    EXPECT_EQ(OptionsFromEnvironment(&warning).factor, std::stod(edge)) << edge;
    EXPECT_EQ(warning.find("not a factor"), std::string::npos) << edge;
  }
  EXPECT_NE(warning.find("times 100"), std::string::npos) << warning;
  setenv(kFactorEnv, "1", 1);
  OptionsFromEnvironment(&warning);
  EXPECT_EQ(warning, "");
  setenv(kFactorEnv, "1.5", 1);
  OptionsFromEnvironment(&warning);
  EXPECT_NE(warning.find("times 1.5"), std::string::npos) << warning;
  // Only longer, never shorter or off.
  for (const char* bad : {"0.5", "0", "-1", "x", "", "1e9", "100.5"}) {
    setenv(kFactorEnv, bad, 1);
    EXPECT_EQ(OptionsFromEnvironment(&warning).factor, 1.0) << bad;
    EXPECT_NE(warning.find("not a factor"), std::string::npos) << bad;
  }
  unsetenv(kNodesEnv);
  unsetenv(kNoteEnv);
  unsetenv(kFactorEnv);
}

}  // namespace
}  // namespace mowbite_watch
