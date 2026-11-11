#include <gtest/gtest.h>

#include <chrono>

#include "local_test_server.h"
#include "mowbite_camera/http_client.h"

namespace mowbite_camera {
namespace {

LocalServer At(uint16_t port) {
  return {port, "mowbite", "secret"};
}

TEST(Base64, EncodesWithPadding) {
  EXPECT_EQ(Base64(""), "");
  EXPECT_EQ(Base64("f"), "Zg==");
  EXPECT_EQ(Base64("fo"), "Zm8=");
  EXPECT_EQ(Base64("foo"), "Zm9v");
  EXPECT_EQ(Base64("foobar"), "Zm9vYmFy");
  EXPECT_EQ(Base64(std::string("\xff\xfe\x00", 3)), "//4A");
}

TEST(LocalRequest, SendsOneRequestAndTakesTheAnswer) {
  OneShotServer server("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n{\"items\":[]}");
  const HttpReply reply = LocalRequest(At(server.port()), "GET", "/v3/webrtcsessions/list", 2.0);
  EXPECT_EQ(reply.status, 200) << reply.error;
  EXPECT_EQ(reply.body, "{\"items\":[]}");
  EXPECT_EQ(server.request(), "GET /v3/webrtcsessions/list HTTP/1.0\r\nHost: 127.0.0.1\r\nAuthorization: Basic " +
                                  Base64("mowbite:secret") + "\r\nContent-Length: 0\r\n\r\n");
}

TEST(LocalRequest, PassesOnAStatusThatIsNoSuccess) {
  OneShotServer server("HTTP/1.1 401 Unauthorized\r\nWww-Authenticate: Basic\r\n\r\n");
  const HttpReply reply = LocalRequest(At(server.port()), "POST", "/v3/webrtcsessions/kick/1", 2.0);
  EXPECT_EQ(reply.status, 401);
  EXPECT_EQ(reply.body, "");
  EXPECT_EQ(reply.error, "");
}

TEST(LocalRequest, GivesUpWithoutAnAnswerInTime) {
  OneShotServer silent("");
  const auto before = std::chrono::steady_clock::now();
  const HttpReply reply = LocalRequest(At(silent.port()), "GET", "/", 0.2);
  EXPECT_EQ(reply.status, 0);
  EXPECT_EQ(reply.error, "no answer in time");
  EXPECT_LT(std::chrono::steady_clock::now() - before, std::chrono::seconds(2));
}

TEST(LocalRequest, WhatIsNoAnswerOfHttpIsNone) {
  for (const char* answer : {"hello\r\n\r\n", "HTTP/1.1 200", "HTTP/1.1 20 OK\r\n\r\n", "HTTP/1.1 2000 OK\r\n\r\n"}) {
    OneShotServer server(answer);
    const HttpReply reply = LocalRequest(At(server.port()), "GET", "/", 2.0);
    EXPECT_EQ(reply.status, 0) << answer;
    EXPECT_EQ(reply.error, "not an answer of HTTP") << answer;
  }
}

TEST(LocalRequest, NobodyListens) {
  uint16_t port = 0;
  {
    OneShotServer gone("x");
    port = gone.port();
  }
  const HttpReply reply = LocalRequest(At(port), "GET", "/", 2.0);
  EXPECT_EQ(reply.status, 0);
  EXPECT_EQ(reply.error.rfind("connect: ", 0), 0u) << reply.error;
}

}  // namespace
}  // namespace mowbite_camera
