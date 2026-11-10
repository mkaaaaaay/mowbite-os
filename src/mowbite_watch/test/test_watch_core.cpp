#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "mowbite_watch/watch.h"
#include "mowbite_watch/watch_core.h"

namespace mowbite_watch {
namespace {

constexpr int64_t kS = 1000000000;

WatchCore Core(const std::vector<std::string>& expected = {}) {
  return WatchCore(expected, 0, 60 * kS, 20 * kS);
}

TEST(WatchCore, SignsInTimeFindNothing) {
  WatchCore core = Core();
  const int a = core.Enter("mowbite_logic", 10 * kS, 0);
  for (int64_t t = kS; t <= 60 * kS; t += kS) {
    core.Sign(a, t);
    EXPECT_EQ(core.Judge(t + kS / 2).finding, Finding::kNone);
  }
}

TEST(WatchCore, ExactlyAtTheLimitIsInTime) {
  WatchCore core = Core();
  const int a = core.Enter("mowbite_safety", 2 * kS, 0);
  core.Sign(a, 5 * kS);
  EXPECT_EQ(core.Judge(7 * kS).finding, Finding::kNone);
  const Verdict verdict = core.Judge(7 * kS + 1);
  EXPECT_EQ(verdict.finding, Finding::kSilent);
  EXPECT_STREQ(verdict.node, "mowbite_safety");
  EXPECT_EQ(verdict.seconds_ns, 2 * kS + 1);
  EXPECT_EQ(verdict.limit_ns, 2 * kS);
  EXPECT_EQ(verdict.mark, nullptr);
}

TEST(WatchCore, EachNodeHasItsOwnLimit) {
  WatchCore core = Core();
  const int s2 = core.Enter("mowbite_safety", 2 * kS, 0);
  const int logic = core.Enter("mowbite_logic", 10 * kS, 0);
  core.Sign(logic, 3 * kS);
  EXPECT_STREQ(core.Judge(3 * kS).node, "mowbite_safety");
  core.Sign(s2, 3 * kS);
  EXPECT_EQ(core.Judge(5 * kS).finding, Finding::kNone);
  core.Sign(s2, 12 * kS);
  EXPECT_EQ(core.Judge(13 * kS).finding, Finding::kNone);
  EXPECT_STREQ(core.Judge(13 * kS + 1).node, "mowbite_logic");
}

TEST(WatchCore, OfTwoSilentNodesTheOlderIsNamed) {
  WatchCore core = Core();
  const int a = core.Enter("a", kS, 0);
  const int b = core.Enter("b", kS, 0);
  const int c = core.Enter("c", kS, 0);
  core.Sign(a, 3 * kS);
  core.Sign(b, 2 * kS);
  core.Sign(c, 9 * kS);
  const Verdict verdict = core.Judge(5 * kS);
  EXPECT_EQ(verdict.finding, Finding::kSilent);
  EXPECT_STREQ(verdict.node, "b");
  EXPECT_EQ(verdict.seconds_ns, 3 * kS);
}

TEST(WatchCore, OfTwoNodesSilentSinceTheSameTimeTheFirstIsNamed) {
  WatchCore core = Core();
  core.Enter("first", kS, 0);
  core.Enter("second", kS, 0);
  EXPECT_STREQ(core.Judge(2 * kS).node, "first");
}

TEST(WatchCore, TheMarkOfTheSilentNodeIsNamed) {
  WatchCore core = Core();
  const int bridge = core.Enter("mowbite_bridge", kS, 0);
  const int map = core.Enter("mowbite_map", kS, 0);
  core.SetMark(map, "save");
  core.SetMark(bridge, "connect");
  core.SetMark(bridge, "MQTT input");
  core.Sign(map, 2 * kS);
  const Verdict verdict = core.Judge(2 * kS);
  EXPECT_STREQ(verdict.node, "mowbite_bridge");
  EXPECT_STREQ(verdict.mark, "MQTT input");
}

TEST(WatchCore, ANodeThatLeftNoLongerCounts) {
  WatchCore core = Core({"mowbite_follower"});
  const int follower = core.Enter("mowbite_follower", kS, 0);
  const int logic = core.Enter("mowbite_logic", kS, 0);
  core.Leave(follower);
  core.Sign(logic, 100 * kS);
  EXPECT_EQ(core.Judge(100 * kS).finding, Finding::kNone);
}

TEST(WatchCore, AnExpectedNodeMissingAfterTheStartLimit) {
  WatchCore core(std::vector<std::string>{"mowbite_logic", "mowbite_bridge", "mowbite_map"}, 5 * kS, 60 * kS, 20 * kS);
  const int logic = core.Enter("mowbite_logic", 10 * kS, 6 * kS);
  const int map = core.Enter("mowbite_map", 10 * kS, 6 * kS);
  for (int64_t t = 6 * kS; t <= 70 * kS; t += kS) {
    core.Sign(logic, t);
    core.Sign(map, t);
  }
  EXPECT_EQ(core.Judge(65 * kS).finding, Finding::kNone);
  const Verdict verdict = core.Judge(65 * kS + 1);
  EXPECT_EQ(verdict.finding, Finding::kMissing);
  EXPECT_STREQ(verdict.node, "mowbite_bridge");
  EXPECT_EQ(verdict.seconds_ns, 60 * kS + 1);
  EXPECT_EQ(verdict.limit_ns, 60 * kS);
  // Late, it counts like the others.
  const int bridge = core.Enter("mowbite_bridge", 10 * kS, 66 * kS);
  core.Sign(bridge, 70 * kS);
  EXPECT_EQ(core.Judge(70 * kS).finding, Finding::kNone);
}

TEST(WatchCore, ASilentNodeComesBeforeAMissingOne) {
  WatchCore core(std::vector<std::string>{"a", "b"}, 0, 60 * kS, 20 * kS);
  core.Enter("a", 10 * kS, 0);
  const Verdict verdict = core.Judge(61 * kS);
  EXPECT_EQ(verdict.finding, Finding::kSilent);
  EXPECT_STREQ(verdict.node, "a");
}

TEST(WatchCore, ANodeNotExpectedIsWatchedAsWell) {
  WatchCore core = Core({"a"});
  core.Enter("a", 10 * kS, 0);
  const int test = core.Enter("probe", kS, 0);
  core.Sign(test, 0);
  EXPECT_STREQ(core.Judge(2 * kS).node, "probe");
}

TEST(WatchCore, AfterTheShutdownOnlyItsLimitCounts) {
  WatchCore core = Core({"a", "b"});
  core.Enter("a", kS, 0);
  core.Shutdown(5 * kS);
  // Neither silence nor the missing node count any more; a second shutdown does not move the start of the limit.
  core.Shutdown(20 * kS);
  EXPECT_EQ(core.Judge(25 * kS).finding, Finding::kNone);
  const Verdict verdict = core.Judge(25 * kS + 1);
  EXPECT_EQ(verdict.finding, Finding::kShutdown);
  EXPECT_EQ(verdict.seconds_ns, 20 * kS + 1);
  EXPECT_EQ(verdict.limit_ns, 20 * kS);
}

TEST(WatchCore, AllPlacesTakenIsAnError) {
  WatchCore core = Core();
  for (int i = 0; i < kMaxPlaces; ++i) EXPECT_EQ(core.Enter("n" + std::to_string(i), kS, 0), i);
  EXPECT_EQ(core.Enter("one_more", kS, 0), -1);
  // A place left is not given again.
  core.Leave(3);
  EXPECT_EQ(core.Enter("one_more", kS, 0), -1);
}

TEST(WatchCore, NamesAndListsBeyondTheFieldsAreRefused) {
  WatchCore core = Core();
  EXPECT_THROW(core.Enter(std::string(kNameSize, 'x'), kS, 0), std::length_error);
  EXPECT_EQ(core.Enter(std::string(kNameSize - 1, 'x'), kS, 0), 0);
  EXPECT_THROW(Core(std::vector<std::string>(kMaxPlaces + 1, "x")), std::length_error);
  EXPECT_NO_THROW(Core(std::vector<std::string>(kMaxPlaces, "x")));
}

TEST(WatchText, NoteAndLine) {
  char buf[512];
  Verdict silent{Finding::kSilent, "mowbite_bridge", 10'240'000'000, 10 * kS, "MQTT input"};
  NoteText(silent, buf, sizeof(buf));
  EXPECT_STREQ(buf, "{\"node\":\"mowbite_bridge\",\"seconds\":10.2,\"limit\":10.0}\n");
  EndLine(silent, buf, sizeof(buf));
  EXPECT_STREQ(buf,
               "mowbite_watch: mowbite_bridge silent for 10.2 s (limit 10.0 s), last 'MQTT input': ending the "
               "process\n");
  Verdict missing{Finding::kMissing, "tron_gnss", 60'500'000'000, 60 * kS, nullptr};
  EndLine(missing, buf, sizeof(buf));
  EXPECT_STREQ(buf, "mowbite_watch: tron_gnss did not start within 60.0 s: ending the process\n");
  Verdict shutdown{Finding::kShutdown, "", 20'060'000'000, 20 * kS, nullptr};
  EndLine(shutdown, buf, sizeof(buf));
  EXPECT_STREQ(buf, "mowbite_watch: still running 20.1 s after the shutdown (limit 20.0 s): ending the process\n");
  // A name from the environment stays JSON; a short buffer cuts.
  Verdict odd{Finding::kMissing, "a\"b\\c", 0, 0, nullptr};
  NoteText(odd, buf, sizeof(buf));
  EXPECT_STREQ(buf, "{\"node\":\"a?b?c\",\"seconds\":0.0,\"limit\":0.0}\n");
  EXPECT_EQ(NoteText(silent, buf, 10), 9u);
  EXPECT_STREQ(buf, "{\"node\":\"");
}

}  // namespace
}  // namespace mowbite_watch
