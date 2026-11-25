#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <thread>

#include "test_util.h"
#include "tron_vision/crash_guard.h"

namespace tron_vision {
namespace {

namespace fs = std::filesystem;

struct Guarded {
  test::TempDir tmp;
  std::string dir = tmp.Path() + "/vision";
  std::string flag = tmp.Path() + "/crash";
  std::string boot = tmp.Write("boot_id", "boot-1\n");
  CrashGuard guard{dir, flag, boot};
};

TEST(CrashGuard, FreeWithoutMarker) {
  Guarded s;
  std::string detail;
  EXPECT_TRUE(s.guard.MayStream("oa_left", detail));
}

TEST(CrashGuard, ArmWritesTheBootIdAndDisarmRemovesIt) {
  Guarded s;
  std::string error;
  ASSERT_TRUE(s.guard.Arm("oa_left", error)) << error;
  const std::string marker = s.dir + "/stream-oa_left";
  ASSERT_TRUE(fs::exists(marker));
  std::ifstream in(marker);
  std::string line;
  std::getline(in, line);
  EXPECT_EQ(line, "boot-1");
  s.guard.Disarm("oa_left");
  EXPECT_FALSE(fs::exists(marker));
}

TEST(CrashGuard, MarkerOfThisBootIsAProcessEnd) {
  Guarded s;
  std::string error;
  ASSERT_TRUE(s.guard.Arm("oa_left", error));
  s.tmp.Write("crash", "1700000000 Kernel panic\n");  // crash of an earlier boot, not during this stream
  std::string detail;
  EXPECT_TRUE(s.guard.MayStream("oa_left", detail));
  EXPECT_FALSE(fs::exists(s.dir + "/stream-oa_left"));
}

TEST(CrashGuard, MarkerOfAnEarlierBootWithoutCrashIsAShutdown) {
  Guarded s;
  std::string error;
  ASSERT_TRUE(s.guard.Arm("oa_left", error));
  s.tmp.Write("boot_id", "boot-2\n");
  std::string detail;
  EXPECT_TRUE(s.guard.MayStream("oa_left", detail));
}

TEST(CrashGuard, MarkerOfAnEarlierBootWithCrashLocksUntilDeleted) {
  Guarded s;
  std::string error;
  ASSERT_TRUE(s.guard.Arm("oa_left", error));
  s.tmp.Write("boot_id", "boot-2\n");
  s.tmp.Write("crash", "1700000000 Oops\n");
  std::string detail;
  EXPECT_FALSE(s.guard.MayStream("oa_left", detail));
  EXPECT_NE(detail.find("Oops"), std::string::npos) << detail;
  EXPECT_TRUE(fs::exists(s.dir + "/locked-oa_left"));
  EXPECT_FALSE(fs::exists(s.dir + "/stream-oa_left"));

  // Still locked in the next boot without crash flag.
  fs::remove(s.flag);
  s.tmp.Write("boot_id", "boot-3\n");
  EXPECT_FALSE(s.guard.MayStream("oa_left", detail));
  // Other cameras are not affected.
  EXPECT_TRUE(s.guard.MayStream("oa_right", detail));

  fs::remove(s.dir + "/locked-oa_left");
  EXPECT_TRUE(s.guard.MayStream("oa_left", detail));
}

TEST(CrashGuard, CrashAtAKnownSiteOutsideTheCamerasDoesNotLock) {
  Guarded s;
  std::string error;
  ASSERT_TRUE(s.guard.Arm("stereo", error));
  s.tmp.Write("boot_id", "boot-2\n");
  s.tmp.Write("crash", "1795582798 Kernel panic - not syncing: Oops: Fatal exception at bluesleep_read_proc_btwrite\n");
  std::string detail;
  EXPECT_TRUE(s.guard.MayStream("stereo", detail)) << detail;
  EXPECT_FALSE(fs::exists(s.dir + "/locked-stereo"));
  EXPECT_FALSE(fs::exists(s.dir + "/stream-stereo"));
}

TEST(CrashGuard, CrashAtAnyOtherSiteOrWithoutOneLocks) {
  for (const char* reason : {"1 Kernel panic - not syncing: Oops: Fatal exception at rkcif_stream_stop",
                             "1 Kernel panic - not syncing: Oops: Fatal exception at bluesleep_read_proc_btwrite2",
                             "1 Kernel panic - not syncing: Oops: Fatal exception",
                             "1 Kernel panic - not syncing: RCU Stall"}) {
    Guarded s;
    std::string error;
    ASSERT_TRUE(s.guard.Arm("stereo", error));
    s.tmp.Write("boot_id", "boot-2\n");
    s.tmp.Write("crash", std::string(reason) + "\n");
    std::string detail;
    EXPECT_FALSE(s.guard.MayStream("stereo", detail)) << reason;
    EXPECT_TRUE(fs::exists(s.dir + "/locked-stereo")) << reason;
  }
}

TEST(CrashGuard, UnlockDeletesOnlyTheLockAndTellsItsReason) {
  Guarded s;
  std::string error;
  ASSERT_TRUE(s.guard.Arm("oa_left", error));
  s.tmp.Write("boot_id", "boot-2\n");
  s.tmp.Write("crash", "1700000000 Oops\n");
  std::string detail;
  ASSERT_FALSE(s.guard.MayStream("oa_left", detail));
  s.tmp.Write("vision/verified-stereo", "x\n");
  std::string text;
  ASSERT_TRUE(s.guard.Unlock("oa_left", text));
  EXPECT_EQ(text, "1700000000 Oops");
  EXPECT_FALSE(fs::exists(s.dir + "/locked-oa_left"));
  EXPECT_TRUE(fs::exists(s.dir + "/verified-stereo"));
  EXPECT_TRUE(s.guard.MayStream("oa_left", detail));
  EXPECT_FALSE(s.guard.Unlock("oa_left", text));
  EXPECT_EQ(text, "not locked");
}

TEST(UnlockRequests, TheLoopThreadAnswersTheService) {
  UnlockRequests requests;
  std::atomic<bool> stop{false};
  int served = 0;
  std::thread loop([&] {
    while (!stop) {
      requests.Serve([&](std::string& text) {
        served++;
        text = "unlocked";
        return true;
      });
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
  std::string text;
  EXPECT_TRUE(requests.Ask(std::chrono::seconds(2), text));
  EXPECT_EQ(text, "unlocked");
  EXPECT_TRUE(requests.Ask(std::chrono::seconds(2), text));
  stop = true;
  loop.join();
  EXPECT_EQ(served, 2);
}

TEST(UnlockRequests, NoAnswerWithoutTheLoopAndNoLateUnlock) {
  UnlockRequests requests;
  std::string text;
  EXPECT_FALSE(requests.Ask(std::chrono::milliseconds(20), text));
  EXPECT_EQ(text, "no answer");
  bool ran = false;
  requests.Serve([&](std::string&) { return ran = true; });
  EXPECT_FALSE(ran);
}

TEST(CrashGuard, ArmFailsWithoutBootIdOrDirectory) {
  test::TempDir tmp;
  std::string error;
  CrashGuard no_boot(tmp.Path() + "/v", tmp.Path() + "/crash", tmp.Path() + "/none");
  EXPECT_FALSE(no_boot.Arm("oa_left", error));
  CrashGuard no_dir(tmp.Path() + "/a/b", tmp.Path() + "/crash", tmp.Write("boot_id", "x\n"));
  EXPECT_FALSE(no_dir.Arm("oa_left", error));
}

}  // namespace
}  // namespace tron_vision
