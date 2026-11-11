#include <gtest/gtest.h>

#include "local_test_server.h"
#include "mowbite_camera/stream_server.h"

namespace mowbite_camera {
namespace {

using Ids = std::vector<std::string>;

TEST(ParseSessions, TakesTheIdsOfTheList) {
  Ids ids = {"old"};
  ASSERT_TRUE(ParseSessions(R"({"itemCount":2,"pageCount":1,"items":[
      {"id":"3f1c-01","path":"stereo","query":"t=x","state":"read"},{"path":"no id"},{"id":"3f1c-02"}]})",
                            &ids));
  EXPECT_EQ(ids, (Ids{"3f1c-01", "3f1c-02"}));
  ASSERT_TRUE(ParseSessions(R"({"itemCount":0,"pageCount":0,"items":[]})", &ids));
  EXPECT_EQ(ids, Ids());
}

TEST(ParseSessions, RefusesWhatIsNoWholeList) {
  Ids ids = {"kept"};
  for (const char* body : {"", "[]", "{}", R"({"items":{}})", R"({"pageCount":2,"items":[{"id":"a1"}]})"}) {
    EXPECT_FALSE(ParseSessions(body, &ids)) << body;
  }
  EXPECT_EQ(ids, Ids{"kept"});
}

TEST(MediaServer, IsInstalledWhenItsProgramCanBeRun) {
  EXPECT_TRUE(MediaServer("/bin/sh", "/tmp").Installed());
  EXPECT_FALSE(MediaServer("/nowhere/mediamtx", "/tmp").Installed());
  EXPECT_FALSE(MediaServer("/etc/hostname", "/tmp").Installed());
  EXPECT_FALSE(MediaServer("", "/tmp").Installed());
}

TEST(StreamApi, AsksForTheSessions) {
  OneShotServer server("HTTP/1.1 200 OK\r\n\r\n{\"pageCount\":1,\"items\":[{\"id\":\"ab-1\"}]}");
  Ids ids;
  ASSERT_TRUE(StreamSessions({server.port(), "mowbite", "secret"}, &ids));
  EXPECT_EQ(ids, Ids{"ab-1"});
  EXPECT_EQ(server.request().rfind("GET /v3/webrtcsessions/list HTTP/1.0\r\n", 0), 0u) << server.request();
}

TEST(StreamApi, AnAnswerThatIsNoListIsNone) {
  OneShotServer refused("HTTP/1.1 401 Unauthorized\r\n\r\n{\"items\":[]}");
  Ids ids;
  EXPECT_FALSE(StreamSessions({refused.port(), "mowbite", "wrong"}, &ids));
}

TEST(StreamApi, EndsASessionByItsId) {
  OneShotServer server("HTTP/1.1 200 OK\r\n\r\n{\"status\":\"ok\"}");
  EXPECT_TRUE(KickStreamSession({server.port(), "mowbite", "secret"}, "c0ffee-12"));
  EXPECT_EQ(server.request().rfind("POST /v3/webrtcsessions/kick/c0ffee-12 HTTP/1.0\r\n", 0), 0u) << server.request();
}

TEST(StreamApi, AnIdThatIsNoneIsNotSent) {
  OneShotServer server("HTTP/1.1 200 OK\r\n\r\n");
  for (const char* id : {"", "a b", "x/../y", "1\r\nHost: x", "zz"}) {
    EXPECT_FALSE(KickStreamSession({server.port(), "mowbite", "secret"}, id)) << id;
  }
  EXPECT_EQ(server.request(), "");
}

}  // namespace
}  // namespace mowbite_camera
