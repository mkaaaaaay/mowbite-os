#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "mowbite_hold/restart.h"

namespace mowbite_hold {
namespace {

const char kBoot[] = "6f1c0a52-4a7e-4d0e-9c1e-2b1f00000001";
const char kOtherBoot[] = "6f1c0a52-4a7e-4d0e-9c1e-2b1f00000002";

SessionFile Session(const std::string& state, const std::string& boot, bool clean, const std::string& job = "") {
  SessionFile file;
  file.present = true;
  file.record = SessionRecord{state, job, boot, 1800000000, clean};
  return file;
}

const std::vector<std::string> kStates = {"IDLE",      "MOWING",   "PAUSED",  "AREA_RECORDING",
                                          "UNDOCKING", "DOCKING",  "DRIVING", "CALIBRATING"};

TEST(JudgeStart, RuleTableOfCauseAndState) {
  for (const auto& state : kStates) {
    const bool works = state != "IDLE";
    // Same boot, not clean: the process ended.
    Verdict v = JudgeStart(Session(state, kBoot, false), kBoot, std::nullopt);
    EXPECT_EQ(v.cause, Cause::kOsCrash) << state;
    EXPECT_EQ(v.hold, works) << state;
    EXPECT_EQ(v.state_before, state);
    // Another boot, not clean, no kernel record: the power went or the mower hung.
    v = JudgeStart(Session(state, kOtherBoot, false), kBoot, std::nullopt);
    EXPECT_EQ(v.cause, Cause::kPowerLoss) << state;
    EXPECT_EQ(v.hold, works) << state;
    // A kernel record holds in every state, clean or not.
    for (bool clean : {false, true}) {
      v = JudgeStart(Session(state, kOtherBoot, clean), kBoot, std::string("1800000100 Kernel panic - not syncing: x"));
      EXPECT_EQ(v.cause, Cause::kKernelCrash) << state;
      EXPECT_TRUE(v.hold) << state;
      EXPECT_EQ(v.crashed_at, 1800000100);
      EXPECT_EQ(v.detail, "Kernel panic - not syncing: x");
    }
    // A clean stop is no cause, in this boot or another.
    for (const char* boot : {kBoot, kOtherBoot}) {
      v = JudgeStart(Session(state, boot, true), kBoot, std::nullopt);
      EXPECT_EQ(v.cause, Cause::kNone) << state;
      EXPECT_FALSE(v.hold) << state;
    }
  }
}

TEST(JudgeStart, AStallOfTheWatchNamesTheNodeOfAnOsCrash) {
  const Stall stall{"mowbite_bridge", 10.2, 10.0};
  Verdict v = JudgeStart(Session("MOWING", kBoot, false), kBoot, std::nullopt, stall);
  EXPECT_EQ(v.cause, Cause::kOsCrash);
  EXPECT_TRUE(v.hold);
  ASSERT_TRUE(v.stalled);
  EXPECT_EQ(v.stalled->node, "mowbite_bridge");
  EXPECT_EQ(v.detail, "The watch ended MowBite OS: mowbite_bridge gave no sign for 10.2 s (limit 10.0 s)");
  EXPECT_EQ(RestartLine(v, true),
            "Unexpected restart OS_CRASH after MOWING: The watch ended MowBite OS: mowbite_bridge gave no sign for "
            "10.2 s (limit 10.0 s); held until emergency.reset");
  v = JudgeStart(Session("IDLE", kBoot, false), kBoot, std::nullopt, stall);
  EXPECT_FALSE(v.hold);
  EXPECT_TRUE(v.stalled);
  // Only an end in this boot can be the watch's; a clean stop is no end at all.
  for (const auto& other : {JudgeStart(Session("MOWING", kOtherBoot, false), kBoot, std::nullopt, stall),
                            JudgeStart(Session("MOWING", kOtherBoot, false), kBoot, std::string("1 panic"), stall),
                            JudgeStart(Session("MOWING", kBoot, true), kBoot, std::nullopt, stall)}) {
    EXPECT_FALSE(other.stalled);
    EXPECT_EQ(other.detail.find("watch"), std::string::npos) << other.detail;
  }
  EXPECT_EQ(RestartLine(JudgeStart({}, kBoot, std::string("1 panic")), false),
            "Unexpected restart KERNEL_CRASH after an unknown state: panic");
}

TEST(JudgeStart, KernelRecordCountsOnlyAtTheFirstStartOfItsBoot) {
  const std::optional<std::string> record = std::string("1800000100 Oops#1 Part1");
  EXPECT_EQ(JudgeStart(Session("MOWING", kBoot, true), kBoot, record).cause, Cause::kNone);
  const Verdict v = JudgeStart(Session("IDLE", kBoot, false), kBoot, record);
  EXPECT_EQ(v.cause, Cause::kOsCrash);
  EXPECT_FALSE(v.hold);
}

TEST(JudgeStart, WithoutSession) {
  SessionFile none;
  EXPECT_EQ(JudgeStart(none, kBoot, std::nullopt).cause, Cause::kNone);
  const Verdict v = JudgeStart(none, kBoot, std::string("garbled"));
  EXPECT_EQ(v.cause, Cause::kKernelCrash);
  EXPECT_TRUE(v.hold);
  EXPECT_EQ(v.crashed_at, 0);
  EXPECT_EQ(v.detail, "garbled");
  EXPECT_EQ(v.state_before, "");
  EXPECT_EQ(JudgeStart(none, kBoot, std::string("")).detail, "The kernel crashed");
}

TEST(JudgeStart, UnreadableSessionHolds) {
  SessionFile broken;
  broken.present = true;
  broken.error = "x is not a session";
  const Verdict v = JudgeStart(broken, kBoot, std::nullopt);
  EXPECT_EQ(v.cause, Cause::kPowerLoss);
  EXPECT_TRUE(v.hold);
  EXPECT_EQ(v.state_before, "");
  EXPECT_NE(v.detail.find("x is not a session"), std::string::npos);
}

TEST(JudgeStart, UnknownBootIdIsAnotherBoot) {
  const Verdict v = JudgeStart(Session("MOWING", kBoot, false, "J1"), "", std::nullopt);
  EXPECT_EQ(v.cause, Cause::kPowerLoss);
  EXPECT_TRUE(v.hold);
  EXPECT_EQ(v.job_id_before, "J1");
}

TEST(JudgeStart, DetailIsClipped) {
  const Verdict v = JudgeStart(SessionFile(), kBoot, "1 " + std::string(500, 'a') + "\n");
  EXPECT_EQ(v.detail.size(), 200u);
}

// The logic waits exactly when the new record would hold after a crash and the one surely on the disk would not.
TEST(MustWaitForSession, OnlyWhenTheHoldBegins) {
  const auto record = [](const std::string& state, bool clean) {
    return SessionRecord{state, "", kBoot, 1800000000, clean};
  };
  // What a start in another boot (POWER_LOSS) would say about the record.
  const auto holds = [](const SessionRecord& r) {
    SessionFile file;
    file.present = true;
    file.record = r;
    return JudgeStart(file, kOtherBoot, std::nullopt).hold;
  };
  for (const auto& next_state : kStates) {
    for (const bool next_clean : {false, true}) {
      const SessionRecord next = record(next_state, next_clean);
      EXPECT_EQ(HoldsOnRestart(next), holds(next)) << next_state << next_clean;
      EXPECT_EQ(MustWaitForSession(false, next), holds(next)) << next_state << next_clean;
      EXPECT_FALSE(MustWaitForSession(true, next)) << next_state << next_clean;
    }
  }
  EXPECT_TRUE(MustWaitForSession(false, record("MOWING", false)));
  EXPECT_TRUE(MustWaitForSession(false, record("DRIVING", false)));
  EXPECT_FALSE(MustWaitForSession(true, record("PAUSED", false)));
  EXPECT_FALSE(MustWaitForSession(false, record("IDLE", false)));
  EXPECT_FALSE(MustWaitForSession(false, record("MOWING", true)));
}

TEST(SessionState, IdleTakesItsDrive) {
  EXPECT_EQ(SessionState("IDLE", ""), "IDLE");
  EXPECT_EQ(SessionState("IDLE", "DRIVING"), "DRIVING");
  EXPECT_EQ(SessionState("IDLE", "CALIBRATING"), "CALIBRATING");
  EXPECT_EQ(SessionState("AREA_RECORDING", "RECORDING_OUTLINE"), "AREA_RECORDING");
}

TEST(ParseSession, ReadsItsOwnText) {
  const SessionRecord record{"MOWING", "J1", kBoot, 1800000000, false};
  const auto parsed = ParseSession(SessionText(record));
  ASSERT_TRUE(parsed);
  EXPECT_EQ(parsed->state, "MOWING");
  EXPECT_EQ(parsed->job_id, "J1");
  EXPECT_EQ(parsed->boot_id, kBoot);
  EXPECT_EQ(parsed->since, 1800000000);
  EXPECT_FALSE(parsed->clean);
}

TEST(ParseSession, RefusesWhatIsNoSession) {
  for (const char* text :
       {"", "{}", "[]", "null", R"({"state": "", "job_id": "", "boot_id": "b", "since": 1, "clean": false})",
        R"({"state": "IDLE", "job_id": "", "boot_id": "b", "since": 1})",
        R"({"state": "IDLE", "job_id": "", "boot_id": "b", "since": 1.5, "clean": false})",
        R"({"state": "IDLE", "job_id": null, "boot_id": "b", "since": 1, "clean": false})",
        R"({"state": "IDLE", "job_id": "", "boot_id": "b", "since": 1, "clean": 0})"}) {
    EXPECT_FALSE(ParseSession(text)) << text;
  }
}

TEST(CauseName, Names) {
  EXPECT_STREQ(CauseName(Cause::kKernelCrash), "KERNEL_CRASH");
  EXPECT_STREQ(CauseName(Cause::kOsCrash), "OS_CRASH");
  EXPECT_STREQ(CauseName(Cause::kPowerLoss), "POWER_LOSS");
  EXPECT_STREQ(CauseName(Cause::kNone), "");
}

TEST(HostLatch, UnexpectedRestartIsBit21AndLatchable) {
  EXPECT_EQ(kReasonUnexpectedRestart, 2097152u);
  EXPECT_TRUE(kHostLatchReasons & kReasonUnexpectedRestart);
  // LATCH, TIMEOUT_INPUTS, STOP, LIFT, COLLISION and the board's own bits stay the driver's.
  EXPECT_EQ(kHostLatchReasons & ((1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) | (1u << 5) | (1u << 16)), 0u);
}

class RestartDir : public ::testing::Test {
 protected:
  void SetUp() override {
    char pattern[] = "/tmp/mowbite_restart_XXXXXX";
    ASSERT_NE(::mkdtemp(pattern), nullptr);
    dir_ = pattern;
  }
  void TearDown() override {
    std::filesystem::permissions(dir_, std::filesystem::perms::owner_all, std::filesystem::perm_options::add);
    std::filesystem::remove_all(dir_);
  }
  std::string dir_;
};

TEST_F(RestartDir, SessionFileStates) {
  const std::string path = dir_ + "/" + kSessionFile;
  SessionFile file = ReadSessionFile(path);
  EXPECT_FALSE(file.present);
  std::string error;
  ASSERT_TRUE(WriteRecordFile(path, SessionText({"DOCKING", "", kBoot, 5, false}), &error)) << error;
  EXPECT_FALSE(std::filesystem::exists(path + ".tmp"));
  file = ReadSessionFile(path);
  ASSERT_TRUE(file.present && file.record);
  EXPECT_EQ(file.record->state, "DOCKING");
  std::ofstream(path) << "{\"state\": \"MOW";
  file = ReadSessionFile(path);
  EXPECT_TRUE(file.present);
  EXPECT_FALSE(file.record);
  EXPECT_NE(file.error.find("is not a session"), std::string::npos);
  std::ofstream(path) << std::string(5000, ' ');
  EXPECT_NE(ReadSessionFile(path).error.find("larger"), std::string::npos);
}

TEST_F(RestartDir, RestartHoldFile) {
  const std::string path = dir_ + "/" + kRestartHoldFile;
  EXPECT_FALSE(RestartHoldPresent(path));
  Verdict verdict = JudgeStart(Session("MOWING", kBoot, false, "J7"), kBoot, std::nullopt);
  std::string error;
  ASSERT_TRUE(WriteRecordFile(path, RestartHoldText(verdict, 1800000200), &error)) << error;
  EXPECT_TRUE(RestartHoldPresent(path));
  std::ifstream in(path);
  const nlohmann::json j = nlohmann::json::parse(in);
  EXPECT_EQ(j["cause"], "OS_CRASH");
  EXPECT_TRUE(j["crashed_at"].is_null());
  EXPECT_EQ(j["state_before"], "MOWING");
  EXPECT_EQ(j["job_id_before"], "J7");
  EXPECT_EQ(j["since"], 1800000200);
  ASSERT_TRUE(RemoveRecordFile(path, &error)) << error;
  EXPECT_FALSE(RestartHoldPresent(path));
  EXPECT_TRUE(RemoveRecordFile(path, &error)) << "a file that is not there counts as removed";
}

TEST_F(RestartDir, HoldThatCannotBeLookedAtCounts) {
  if (::geteuid() == 0) GTEST_SKIP() << "root reads any folder";
  const std::string path = dir_ + "/sub/" + kRestartHoldFile;
  std::filesystem::create_directory(dir_ + "/sub");
  std::ofstream(path) << "{}";
  std::filesystem::permissions(dir_ + "/sub", std::filesystem::perms::none);
  EXPECT_TRUE(RestartHoldPresent(path));
  std::filesystem::permissions(dir_ + "/sub", std::filesystem::perms::owner_all);
}

TEST_F(RestartDir, StallOfTheExitNote) {
  const std::string path = dir_ + "/" + kExitNoteFile;
  EXPECT_FALSE(ReadStall(path));
  std::ofstream(path) << R"({"exit_code": 86, "restart": 1, "max_restarts": 3, "t": 1800000000.5,
      "stalled": {"node": "mowbite_safety", "seconds": 2.1, "limit": 2.0}})";
  const std::optional<Stall> stall = ReadStall(path);
  ASSERT_TRUE(stall);
  EXPECT_EQ(stall->node, "mowbite_safety");
  EXPECT_DOUBLE_EQ(stall->seconds, 2.1);
  EXPECT_DOUBLE_EQ(stall->limit, 2.0);
  for (const char* text : {R"({"exit_code": -9, "restart": 1})", R"({"stalled": null})", R"({"stalled": {}})",
                           R"({"stalled": {"node": 1, "seconds": 2, "limit": 2}})",
                           R"({"stalled": {"node": "a", "seconds": "2", "limit": 2}})",
                           R"({"stalled": {"node": "a", "seconds": 2}})", "{\"stalled", "[]"}) {
    std::ofstream(path) << text;
    EXPECT_FALSE(ReadStall(path)) << text;
  }
}

TEST_F(RestartDir, BootIdAndKernelRecord) {
  std::ofstream(dir_ + "/boot_id") << kBoot << "\n";
  EXPECT_EQ(ReadBootId(dir_ + "/boot_id"), kBoot);
  EXPECT_EQ(ReadBootId(dir_ + "/none"), "");
  EXPECT_FALSE(ReadKernelRecord(""));
  EXPECT_FALSE(ReadKernelRecord(dir_ + "/crash"));
  std::ofstream(dir_ + "/crash") << "1800000100 Kernel panic - not syncing: probe\n";
  EXPECT_EQ(*ReadKernelRecord(dir_ + "/crash"), "1800000100 Kernel panic - not syncing: probe\n");
}

}  // namespace
}  // namespace mowbite_hold
