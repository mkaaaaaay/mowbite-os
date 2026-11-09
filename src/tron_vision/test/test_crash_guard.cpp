#include <gtest/gtest.h>

#include <filesystem>

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
