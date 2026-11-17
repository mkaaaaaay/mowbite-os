#include <gtest/gtest.h>

#include <map>

#include "mowbite_net/outbox.h"

using namespace mowbite_net;

namespace {

class FakeHttp : public Http {
 public:
  HttpResult Send(const HttpRequest& request) override {
    sent.push_back(request);
    const auto it = status.find(request.url);
    return {it == status.end() ? 200 : it->second, "", ""};
  }
  std::vector<HttpRequest> sent;
  std::map<std::string, long> status;
};

NotifyConfig Config() {
  NotifyConfig c = DefaultConfig();
  c.topic = "mowbite-t";
  c.token = "tk_secret";
  c.lang = "de";
  c.url = "https://mower.local";
  c.pushes = {"https://up.example/a", "https://up.example/b"};
  c.stored = true;
  return c;
}

Push Problem() {
  Push p;
  p.kind = "dock_failed";
  p.phase = "new";
  p.problem = true;
  p.prio = 4;
  p.tag = "warning";
  p.title = "Andocken fehlgeschlagen";
  p.message = "Robi steht draußen.";
  return p;
}

}  // namespace

TEST(Outbox, NtfyRequest) {
  const std::optional<HttpRequest> r = NtfyRequest(Problem(), Config());
  ASSERT_TRUE(r);
  EXPECT_EQ(r->url, "https://ntfy.sh/");
  EXPECT_EQ(r->headers, (std::vector<std::string>{"Content-Type: application/json", "Authorization: Bearer tk_secret"}));
  const json body = json::parse(r->body);
  EXPECT_EQ(body, json::parse(R"({"topic": "mowbite-t", "title": "Andocken fehlgeschlagen",
      "message": "Robi steht draußen.", "priority": 4, "tags": ["warning"],
      "click": "https://mower.local/activity/",
      "actions": [{"action": "view", "label": "MowBite öffnen", "url": "https://mower.local/activity/"}]})"));
  NotifyConfig none = Config();
  none.topic.clear();
  EXPECT_FALSE(NtfyRequest(Problem(), none));
  none = Config();
  none.url.clear();
  none.token.clear();
  const std::optional<HttpRequest> plain = NtfyRequest(Problem(), none);
  ASSERT_TRUE(plain);
  EXPECT_EQ(plain->headers.size(), 1u);
  EXPECT_FALSE(json::parse(plain->body).contains("click"));
}

TEST(Outbox, PhoneRequestCarriesNoToken) {
  const HttpRequest r = PhoneRequest(Problem(), Config(), "https://up.example/a", 1794564000.7);
  EXPECT_EQ(r.url, "https://up.example/a");
  EXPECT_EQ(json::parse(r.body), json::parse(R"({"title": "Andocken fehlgeschlagen", "message": "Robi steht draußen.",
      "kind": "dock_failed", "phase": "new", "problem": true, "prio": 4, "t": 1794564000, "lang": "de",
      "path": "/activity/", "api": "https://mower.local"})"));
  EXPECT_EQ(r.body.find("tk_secret"), std::string::npos);
  for (const auto& header : r.headers) EXPECT_EQ(header.find("tk_secret"), std::string::npos);
}

TEST(Outbox, DeliverToAllAndDropGonePhones) {
  FakeHttp http;
  http.status["https://up.example/b"] = 410;
  const Delivery d = Deliver(http, Problem(), Config(), 0);
  EXPECT_EQ(http.sent.size(), 3u);
  EXPECT_TRUE(d.sent);
  EXPECT_FALSE(d.again);
  EXPECT_EQ(d.gone, (std::vector<std::string>{"https://up.example/b"}));
}

TEST(Outbox, TheStopEndsADeliveryBetweenTwoTargets) {
  FakeHttp http;
  const Delivery d = Deliver(http, Problem(), Config(), 0, std::nullopt, [&http] { return !http.sent.empty(); });
  ASSERT_EQ(http.sent.size(), 1u);
  EXPECT_EQ(http.sent[0].url, "https://ntfy.sh/");
  EXPECT_TRUE(d.stopped);
  EXPECT_FALSE(d.sent);
  EXPECT_FALSE(d.again);
  FakeHttp all;
  const Delivery full = Deliver(all, Problem(), Config(), 0, std::nullopt, [] { return false; });
  EXPECT_EQ(all.sent.size(), 3u);
  EXPECT_FALSE(full.stopped);
  EXPECT_TRUE(full.sent);
}

TEST(Outbox, DeliverToOnePhone) {
  FakeHttp http;
  const Delivery d = Deliver(http, Problem(), Config(), 0, std::string("https://up.example/b"));
  ASSERT_EQ(http.sent.size(), 1u);
  EXPECT_EQ(http.sent[0].url, "https://up.example/b");
  EXPECT_TRUE(d.sent);
}

TEST(Outbox, NoAnswerIsTriedAgainARefusalNot) {
  FakeHttp http;
  for (const char* url : {"https://ntfy.sh/", "https://up.example/a", "https://up.example/b"}) http.status[url] = 0;
  EXPECT_TRUE(Deliver(http, Problem(), Config(), 0).again);
  for (auto& [url, status] : http.status) status = 403;
  const Delivery d = Deliver(http, Problem(), Config(), 0);
  EXPECT_FALSE(d.sent);
  EXPECT_FALSE(d.again);
  NotifyConfig none = Config();
  none.topic.clear();
  none.pushes.clear();
  EXPECT_FALSE(Deliver(http, Problem(), none, 0).targets);
}

TEST(Outbox, ThreeTriesAndTwentyAtMost) {
  Outbox box;
  box.Add(Problem(), Config(), 0, 0);
  std::optional<Outbox::Item> item = box.Take(0);
  ASSERT_TRUE(item);
  box.Again(*item, 0);
  EXPECT_FALSE(box.Due(29));
  item = box.Take(30);
  ASSERT_TRUE(item);
  box.Again(*item, 30);
  item = box.Take(90);
  ASSERT_TRUE(item);
  box.Again(*item, 90);
  EXPECT_TRUE(box.Empty());
  EXPECT_EQ(box.dropped(), 1u);
  for (int i = 0; i < 25; ++i) box.Add(Problem(), Config(), i, 0);
  EXPECT_EQ(box.dropped(), 6u);
}
