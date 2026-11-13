#include <gtest/gtest.h>

#include "mowbite_net/notify_config.h"

using namespace mowbite_net;

namespace {

std::string Field(const std::optional<Rejection>& bad) {
  return bad ? bad->field + "/" + bad->reason : "";
}

NotifyConfig Set(const char* params) {
  NotifyConfig config = DefaultConfig();
  EXPECT_EQ(Field(ApplySet(json::parse(params), &config)), "");
  return config;
}

}  // namespace

TEST(NotifyConfig, Defaults) {
  const NotifyConfig config = DefaultConfig();
  EXPECT_TRUE(config.enabled);
  EXPECT_FALSE(config.stored);
  EXPECT_EQ(config.server, "https://ntfy.sh");
  EXPECT_EQ(config.events, (std::vector<std::string>{"emergency", "dock_failed", "undock_failed", "nav_error",
                                                     "spinup", "standstill", "refused", "maintenance"}));
  EXPECT_EQ(config.remind, 30);
  EXPECT_EQ(config.emergency_wait, 10);
}

TEST(NotifyConfig, SetTakesEveryKey) {
  const NotifyConfig c = Set(R"({"enabled": false, "server": "http://ntfy.local:8080/", "topic": "mowbite-ab_1",
      "token": "tk_secret", "lang": "de", "name": "Röbi", "url": "https://mower.local/", "events": ["done", "dark"],
      "remind": 0, "emergency_wait": 600})");
  EXPECT_FALSE(c.enabled);
  EXPECT_EQ(c.server, "http://ntfy.local:8080");
  EXPECT_EQ(c.topic, "mowbite-ab_1");
  EXPECT_EQ(c.token, "tk_secret");
  EXPECT_EQ(c.lang, "de");
  EXPECT_EQ(c.name, "Röbi");
  EXPECT_EQ(c.url, "https://mower.local");
  EXPECT_EQ(c.events, (std::vector<std::string>{"done", "dark"}));
  EXPECT_EQ(c.remind, 0);
  EXPECT_EQ(c.emergency_wait, 600);
  EXPECT_TRUE(c.stored);
}

TEST(NotifyConfig, SetRefusesAllOrNothing) {
  const std::vector<std::pair<const char*, const char*>> cases = {
      {R"([])", "params/type"},
      {R"({"color": 1})", "color/unknown"},
      {R"({"enabled": 1})", "enabled/type"},
      {R"({"server": "ftp://x"})", "server/format"},
      {R"({"server": "https://a b"})", "server/format"},
      {R"({"url": "javascript:x"})", "url/format"},
      {R"({"topic": "a/b"})", "topic/format"},
      {R"({"topic": "x\n"})", "topic/format"},
      {R"({"token": "a b"})", "token/format"},
      {R"({"lang": "fr"})", "lang/range"},
      {R"({"name": "12345678901234567890123456789012345678901"})", "name/range"},
      {R"({"events": "done"})", "events/type"},
      {R"({"events": ["done", "fire"]})", "events/unknown"},
      {R"({"events": ["done", "done"]})", "events/duplicate"},
      {R"({"remind": 1441})", "remind/range"},
      {R"({"remind": 1.5})", "remind/type"},
      {R"({"emergency_wait": -1})", "emergency_wait/range"},
      {R"({"topic": "ok", "lang": "xx"})", "lang/range"},
  };
  for (const auto& [params, expected] : cases) {
    NotifyConfig config = DefaultConfig();
    EXPECT_EQ(Field(ApplySet(json::parse(params), &config)), expected) << params;
    EXPECT_EQ(config, DefaultConfig()) << params;
  }
}

TEST(NotifyConfig, TokenKeptDroppedAndNeverAnswered) {
  NotifyConfig c = Set(R"({"token": "tk_secret"})");
  ASSERT_FALSE(ApplySet(json::parse(R"({"topic": "t"})"), &c));
  EXPECT_EQ(c.token, "tk_secret");
  const json answer = ConfigAnswer(c, 0);
  EXPECT_TRUE(answer["has_token"].get<bool>());
  EXPECT_FALSE(answer.contains("token"));
  EXPECT_EQ(answer.dump().find("tk_secret"), std::string::npos);
  ASSERT_FALSE(ApplySet(json::parse(R"({"token": ""})"), &c));
  EXPECT_FALSE(ConfigAnswer(c, 0)["has_token"].get<bool>());
}

TEST(NotifyConfig, FileRoundTrip) {
  NotifyConfig c = Set(R"({"topic": "t1", "token": "tk", "events": ["rain"]})");
  ASSERT_FALSE(AddPush(json::parse(R"({"address": "https://push.example/up/abc"})"), &c));
  std::string error;
  EXPECT_EQ(ConfigFromJson(ConfigToJson(c), &error), c);
  EXPECT_EQ(error, "");
}

TEST(NotifyConfig, BrokenFileGivesTheDefaults) {
  std::string error;
  EXPECT_EQ(ConfigFromJson(json::parse(R"({"schema": 1, "lang": "fr", "stored": true})"), &error), DefaultConfig());
  EXPECT_EQ(error, "lang: de or en");
  error.clear();
  EXPECT_EQ(ConfigFromJson(json::parse("[1]"), &error), DefaultConfig());
  EXPECT_FALSE(error.empty());
  error.clear();
  EXPECT_EQ(ConfigFromJson(nullptr, &error), DefaultConfig());
  EXPECT_EQ(error, "");
}

TEST(NotifyConfig, PushesTenAtMostOldestGoes) {
  NotifyConfig c = DefaultConfig();
  for (int i = 0; i < 12; ++i) {
    ASSERT_FALSE(AddPush({{"address", "https://up.example/" + std::to_string(i)}}, &c));
  }
  ASSERT_FALSE(AddPush({{"address", "https://up.example/11"}}, &c));
  ASSERT_EQ(c.pushes.size(), 10u);
  EXPECT_EQ(c.pushes.front(), "https://up.example/2");
  EXPECT_EQ(Field(AddPush(json::parse(R"({"address": "mailto:x"})"), &c)), "address/format");
  EXPECT_EQ(Field(AddPush(json::parse(R"({})"), &c)), "address/missing");
  ASSERT_FALSE(RemovePush({{"address", "https://up.example/5"}}, &c));
  EXPECT_EQ(c.pushes.size(), 9u);
  std::optional<std::string> address;
  EXPECT_EQ(Field(CheckTest({{"address", "https://up.example/5"}}, c, &address)), "address/unknown");
  EXPECT_FALSE(CheckTest({{"address", "https://up.example/6"}}, c, &address));
  EXPECT_EQ(address, "https://up.example/6");
  address.reset();
  EXPECT_FALSE(CheckTest(nullptr, c, &address));
  EXPECT_FALSE(address);
}

TEST(NotifyConfig, SnoozeAndLog) {
  int64_t seconds = -1;
  EXPECT_FALSE(CheckSnooze(json::parse(R"({"seconds": 3600})"), &seconds));
  EXPECT_EQ(seconds, 3600);
  EXPECT_EQ(Field(CheckSnooze(json::parse(R"({"seconds": 86401})"), &seconds)), "seconds/range");
  EXPECT_EQ(Field(CheckSnooze(nullptr, &seconds)), "params/type");
  size_t limit = 0;
  EXPECT_FALSE(CheckLog(nullptr, &limit));
  EXPECT_EQ(limit, 50u);
  EXPECT_FALSE(CheckLog(json::parse(R"({"limit": 3})"), &limit));
  EXPECT_EQ(limit, 3u);
  EXPECT_EQ(Field(CheckLog(json::parse(R"({"limit": 0})"), &limit)), "limit/range");
}
