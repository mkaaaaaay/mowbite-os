#include <gtest/gtest.h>

#include "mowbite_bridge/warn_log.h"

using namespace mowbite_bridge;

namespace {

WarnLog Filled() {
  WarnLog log(10);
  log.Add(1.0, kLogInfo, "n", "info is dropped");
  log.Add(2.0, kLogWarn, "a", "w1");
  log.Add(3.0, kLogError, "b", "e1");
  log.Add(4.0, kLogFatal, "c", "f1");
  log.Add(5.0, kLogWarn, "a", "w2");
  return log;
}

std::vector<std::string> Messages(const Json& lines) {
  std::vector<std::string> out;
  for (const auto& line : lines) out.push_back(line["msg"]);
  return out;
}

}  // namespace

// B21
TEST(WarnLog, OnlyWarnAndAboveOldestFirst) {
  const Json all = Filled().Recent(nullptr);
  EXPECT_EQ(Messages(all), (std::vector<std::string>{"w1", "e1", "f1", "w2"}));
  EXPECT_EQ(all[0].dump(), R"({"t":2.0,"level":"WARN","node":"a","msg":"w1"})");
  EXPECT_EQ(all[1]["level"], "ERROR");
  EXPECT_EQ(all[2]["level"], "FATAL");
}

TEST(WarnLog, SinceLevelAndLimit) {
  const WarnLog log = Filled();
  EXPECT_EQ(Messages(log.Recent({{"since", 3.0}})), (std::vector<std::string>{"f1", "w2"}));
  EXPECT_EQ(Messages(log.Recent({{"level", "ERROR"}})), (std::vector<std::string>{"e1", "f1"}));
  EXPECT_EQ(Messages(log.Recent({{"level", "fatal"}})), (std::vector<std::string>{"f1"}));
  EXPECT_EQ(Messages(log.Recent({{"level", "INFO"}})).size(), 4u);
  EXPECT_EQ(Messages(log.Recent({{"level", "LOUD"}})).size(), 4u);
  EXPECT_EQ(Messages(log.Recent({{"limit", 2}})), (std::vector<std::string>{"f1", "w2"}));
  EXPECT_TRUE(log.Recent({{"limit", 0}}).empty());
  for (const Json& all : {Json{{"limit", -1}}, Json{{"limit", "x"}}, Json::array({5}), Json("x"), Json::object()}) {
    EXPECT_EQ(log.Recent(all).size(), 4u) << all.dump();
  }
  EXPECT_EQ(Messages(log.Recent({{"since", 1.5}, {"level", "ERROR"}, {"limit", 1}})), (std::vector<std::string>{"f1"}));
}

TEST(WarnLog, KeepsTheNewestLines) {
  WarnLog log(3);
  for (int i = 0; i < 5; ++i) log.Add(i, kLogWarn, "n", std::to_string(i));
  EXPECT_EQ(Messages(log.Recent(nullptr)), (std::vector<std::string>{"2", "3", "4"}));
}

TEST(WarnLog, LoginsAreHidden) {
  WarnLog log;
  log.Add(1.0, kLogWarn, "n", "ntrip://user:secret@caster:2101/MOUNT");
  log.Add(2.0, kLogError, "n", "password=hunter2 token: abc");
  EXPECT_EQ(Messages(log.Recent(nullptr)),
            (std::vector<std::string>{"ntrip://***@caster:2101/MOUNT", "password=*** token: ***"}));
}

TEST(WarnLog, BytesThatAreNotUtf8AreReplacedAndLongLinesCut) {
  WarnLog log;
  log.Add(1.0, kLogWarn, "n\xff", "caster Gr\xe4" "fin");
  log.Add(2.0, kLogWarn, "n", std::string(1999, 'x') + "\xe4\xe4\xe4");
  log.Add(3.0, kLogWarn, "n", std::string(2000, 'y'));
  const Json all = log.Recent(nullptr);
  ASSERT_EQ(all.size(), 3u);
  EXPECT_NO_THROW(all.dump());
  EXPECT_EQ(all[0]["node"], "n\xef\xbf\xbd");
  EXPECT_EQ(all[0]["msg"], "caster Gr\xef\xbf\xbd" "fin");
  EXPECT_EQ(all[1]["msg"], std::string(1999, 'x') + "\xef\xbf\xbd...");
  EXPECT_EQ(all[2]["msg"], std::string(2000, 'y'));
}
