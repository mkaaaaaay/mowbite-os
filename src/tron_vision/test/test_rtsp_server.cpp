#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "rtsp_test_client.h"
#include "tron_vision/rtsp_server.h"

namespace tron_vision {
namespace {

using test::Client;

const ParameterSets kSets{{0x67, 0x42, 0xe0, 0x1f, 0xaa}, {0x68, 0xce, 0x3c, 0x80}};

struct Rig {
  explicit Rig(RtspConfig config = {}) : server(WithAnyPort(config), [this] { return clock; }) {
    std::string error;
    listening = server.Listen(error);
    server.SetAvailable(kSets);
  }
  static RtspConfig WithAnyPort(RtspConfig config) {
    config.port = 0;
    return config;
  }
  std::string Url(const std::string& rest = "") const {
    return "rtsp://127.0.0.1:" + std::to_string(server.Port()) + "/stereo" + rest;
  }
  // Sends one request and returns the answer.
  std::string Ask(const Client& client, const std::string& request) {
    client.Send(request);
    server.Poll(100);
    server.Poll(0);
    return client.Receive();
  }
  // Session id after SETUP and PLAY.
  std::string Play(const Client& client) {
    server.Poll(100);
    const std::string setup = Ask(client, "SETUP " + Url("/trackID=0") +
                                              " RTSP/1.0\r\nCSeq: 1\r\nTransport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n");
    const std::string session = test::SessionOf(setup);
    if (session.empty()) return "";
    Ask(client, "PLAY " + Url("/") + " RTSP/1.0\r\nCSeq: 2\r\nSession: " + session + "\r\n\r\n");
    return session;
  }

  double clock = 100.0;
  RtspServer server;
  bool listening = false;
};

TEST(RtspServer, ListensOnLoopbackOnly) {
  Rig rig;
  ASSERT_TRUE(rig.listening);
  ASSERT_NE(rig.server.Port(), 0);
  // A second server cannot take the same port on the loopback address.
  RtspConfig same;
  same.port = rig.server.Port();
  RtspServer second(same, [] { return 0.0; });
  std::string error;
  EXPECT_FALSE(second.Listen(error));
  EXPECT_NE(error.find("127.0.0.1:" + std::to_string(rig.server.Port())), std::string::npos) << error;
  // The port is not bound on the other addresses of the host.
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in any{};
  any.sin_family = AF_INET;
  any.sin_addr.s_addr = htonl(0x7F000002);
  any.sin_port = htons(rig.server.Port());
  EXPECT_NE(connect(fd, reinterpret_cast<sockaddr*>(&any), sizeof(any)), 0);
  close(fd);
}

TEST(RtspServer, OptionsAndKeepAlive) {
  Rig rig;
  Client client(rig.server.Port());
  ASSERT_TRUE(client.Connected());
  rig.server.Poll(100);
  EXPECT_EQ(rig.Ask(client, "OPTIONS " + rig.Url() + " RTSP/1.0\r\nCSeq: 7\r\n\r\n"),
            "RTSP/1.0 200 OK\r\nCSeq: 7\r\nPublic: OPTIONS, DESCRIBE, SETUP, PLAY, GET_PARAMETER, TEARDOWN\r\n\r\n");
  EXPECT_EQ(rig.Ask(client, "GET_PARAMETER " + rig.Url() + " RTSP/1.0\r\ncseq: 8\r\n\r\n"),
            "RTSP/1.0 200 OK\r\nCSeq: 8\r\n\r\n");
  EXPECT_EQ(rig.Ask(client, "ANNOUNCE " + rig.Url() + " RTSP/1.0\r\nCSeq: 9\r\nContent-Length: 3\r\n\r\nv=0"),
            "RTSP/1.0 405 Method Not Allowed\r\nCSeq: 9\r\nAllow: OPTIONS, DESCRIBE, SETUP, PLAY, GET_PARAMETER, "
            "TEARDOWN\r\n\r\n");
  EXPECT_EQ(rig.Ask(client, "OPTIONS " + rig.Url() + " RTSP/1.0\r\n\r\n"), "RTSP/1.0 400 Bad Request\r\n\r\n");
  EXPECT_EQ(rig.Ask(client, "GET / HTTP/1.1\r\nCSeq: 1\r\n\r\n"), "RTSP/1.0 400 Bad Request\r\nCSeq: 1\r\n\r\n");
}

TEST(RtspServer, DescribeOffersTheStream) {
  Rig rig;
  Client client(rig.server.Port());
  rig.server.Poll(100);
  const std::string answer =
      rig.Ask(client, "DESCRIBE " + rig.Url() + " RTSP/1.0\r\nCSeq: 2\r\nAccept: application/sdp\r\n\r\n");
  const std::string sdp = StreamSdp(kSets, "stereo");
  EXPECT_EQ(answer, "RTSP/1.0 200 OK\r\nCSeq: 2\r\nContent-Base: " + rig.Url("/") +
                        "\r\nContent-Type: application/sdp\r\nContent-Length: " + std::to_string(sdp.size()) +
                        "\r\n\r\n" + sdp);
  EXPECT_NE(sdp.find("m=video 0 RTP/AVP 96\r\n"), std::string::npos);
  EXPECT_NE(sdp.find("a=rtpmap:96 H264/90000\r\n"), std::string::npos);
  EXPECT_NE(sdp.find("a=fmtp:96 packetization-mode=1;profile-level-id=42e01f;sprop-parameter-sets=Z0LgH6o=,aM48gA==\r\n"),
            std::string::npos)
      << sdp;
  EXPECT_EQ(rig.Ask(client, "DESCRIBE rtsp://127.0.0.1/other RTSP/1.0\r\nCSeq: 3\r\n\r\n"),
            "RTSP/1.0 404 Not Found\r\nCSeq: 3\r\n\r\n");
  EXPECT_EQ(rig.Ask(client, "DESCRIBE rtsp://127.0.0.1/stereo2 RTSP/1.0\r\nCSeq: 4\r\n\r\n"),
            "RTSP/1.0 404 Not Found\r\nCSeq: 4\r\n\r\n");
}

TEST(RtspServer, UnavailableAnswers503AndDropsTheClient) {
  Rig rig;
  Client player(rig.server.Port());
  ASSERT_FALSE(rig.Play(player).empty());
  ASSERT_TRUE(rig.server.Playing());
  rig.server.SetUnavailable();
  EXPECT_FALSE(rig.server.Playing());
  EXPECT_EQ(player.Receive(), "<closed>");

  Client next(rig.server.Port());
  rig.server.Poll(100);
  EXPECT_EQ(rig.Ask(next, "DESCRIBE " + rig.Url() + " RTSP/1.0\r\nCSeq: 1\r\n\r\n"),
            "RTSP/1.0 503 Service Unavailable\r\nCSeq: 1\r\n\r\n");
  EXPECT_EQ(rig.Ask(next, "SETUP " + rig.Url("/trackID=0") +
                              " RTSP/1.0\r\nCSeq: 2\r\nTransport: RTP/AVP/TCP;interleaved=0-1\r\n\r\n"),
            "RTSP/1.0 503 Service Unavailable\r\nCSeq: 2\r\n\r\n");
  // The client that got the 503 stays connected and is served once the stream is back.
  rig.server.SetAvailable(kSets);
  EXPECT_NE(rig.Ask(next, "DESCRIBE " + rig.Url() + " RTSP/1.0\r\nCSeq: 3\r\n\r\n").find("200 OK"), std::string::npos);
}

TEST(RtspServer, SetupTakesOnlyTheRtspConnection) {
  Rig rig;
  Client client(rig.server.Port());
  rig.server.Poll(100);
  EXPECT_EQ(rig.Ask(client, "SETUP " + rig.Url("/trackID=0") +
                                " RTSP/1.0\r\nCSeq: 1\r\nTransport: RTP/AVP;unicast;client_port=5000-5001\r\n\r\n"),
            "RTSP/1.0 461 Unsupported Transport\r\nCSeq: 1\r\n\r\n");
  EXPECT_EQ(rig.Ask(client, "SETUP " + rig.Url("/trackID=0") + " RTSP/1.0\r\nCSeq: 2\r\n\r\n"),
            "RTSP/1.0 461 Unsupported Transport\r\nCSeq: 2\r\n\r\n");
  EXPECT_EQ(rig.Ask(client, "PLAY " + rig.Url("/") + " RTSP/1.0\r\nCSeq: 3\r\nSession: 1\r\n\r\n"),
            "RTSP/1.0 455 Method Not Valid in This State\r\nCSeq: 3\r\n\r\n");
  const std::string setup = rig.Ask(
      client, "SETUP " + rig.Url("/trackID=0") + " RTSP/1.0\r\nCSeq: 4\r\nTransport: RTP/AVP/TCP;unicast;interleaved=4-5\r\n\r\n");
  EXPECT_EQ(setup.rfind("RTSP/1.0 200 OK\r\nCSeq: 4\r\nTransport: RTP/AVP/TCP;unicast;interleaved=4-5\r\nSession: ", 0), 0U)
      << setup;
  EXPECT_NE(setup.find(";timeout=60\r\n"), std::string::npos) << setup;
  EXPECT_EQ(rig.Ask(client, "PLAY " + rig.Url("/") + " RTSP/1.0\r\nCSeq: 5\r\nSession: nobody\r\n\r\n"),
            "RTSP/1.0 454 Session Not Found\r\nCSeq: 5\r\n\r\n");
  EXPECT_FALSE(rig.server.Playing());
  EXPECT_FALSE(rig.server.SendRtp({{1, 2, 3}}));
}

TEST(RtspServer, PlaySendsFramedPacketsAndDropsRtcp) {
  Rig rig;
  Client client(rig.server.Port());
  const std::string session = rig.Play(client);
  ASSERT_FALSE(session.empty());
  ASSERT_TRUE(rig.server.Playing());

  ASSERT_TRUE(rig.server.SendRtp({{0x80, 0x60, 0, 1, 9, 9}}));
  EXPECT_EQ(client.Receive(), std::string("$\0\0\6\x80\x60\0\1\x09\x09", 10));

  // A receiver report, then a request split over two segments with another report in between.
  const std::string report("$\1\0\4\x81\xc9\0\0", 8);
  client.Send(report + "GET_PARAMETER " + rig.Url("/") + " RTSP/1.0\r\nCSe");
  rig.server.Poll(100);
  EXPECT_EQ(client.Receive(), "");
  EXPECT_EQ(rig.Ask(client, "q: 3\r\nSession: " + session + "\r\n\r\n" + report),
            "RTSP/1.0 200 OK\r\nCSeq: 3\r\nSession: " + session + "\r\n\r\n");
  EXPECT_TRUE(rig.server.Playing());

  EXPECT_EQ(rig.Ask(client, "TEARDOWN " + rig.Url("/") + " RTSP/1.0\r\nCSeq: 4\r\nSession: " + session + "\r\n\r\n"),
            "RTSP/1.0 200 OK\r\nCSeq: 4\r\nSession: " + session + "\r\n\r\n");
  EXPECT_FALSE(rig.server.Playing());
  EXPECT_EQ(client.Receive(), "<closed>");
}

TEST(RtspServer, OneClientAtATime) {
  Rig rig;
  Client first(rig.server.Port());
  rig.server.Poll(100);
  Client second(rig.server.Port());
  rig.server.Poll(100);
  EXPECT_EQ(second.Receive(), "<closed>");
  EXPECT_NE(rig.Ask(first, "OPTIONS " + rig.Url() + " RTSP/1.0\r\nCSeq: 1\r\n\r\n").find("200 OK"), std::string::npos);
  rig.server.DropClient();
  EXPECT_EQ(first.Receive(), "<closed>");
  Client third(rig.server.Port());
  rig.server.Poll(100);
  EXPECT_NE(rig.Ask(third, "OPTIONS " + rig.Url() + " RTSP/1.0\r\nCSeq: 1\r\n\r\n").find("200 OK"), std::string::npos);
}

TEST(RtspServer, ClientThatLeavesEndsThePlay) {
  Rig rig;
  {
    Client client(rig.server.Port());
    ASSERT_FALSE(rig.Play(client).empty());
    ASSERT_TRUE(rig.server.Playing());
  }
  rig.server.Poll(100);
  EXPECT_FALSE(rig.server.Playing());
}

TEST(RtspServer, SilentClientIsDropped) {
  Rig rig;
  Client client(rig.server.Port());
  ASSERT_FALSE(rig.Play(client).empty());
  rig.clock += 59.0;
  rig.server.Poll(0);
  EXPECT_TRUE(rig.server.Playing());
  // Anything the client sends counts as a sign of life.
  client.Send(std::string("$\1\0\0", 4));
  rig.server.Poll(100);
  rig.clock += 59.0;
  rig.server.Poll(0);
  EXPECT_TRUE(rig.server.Playing());
  rig.clock += 2.0;
  rig.server.Poll(0);
  EXPECT_FALSE(rig.server.Playing());
  EXPECT_EQ(client.Receive(), "<closed>");
}

TEST(RtspServer, EndlessRequestIsDropped) {
  Rig rig;
  Client client(rig.server.Port());
  rig.server.Poll(100);
  client.Send(std::string(9000, 'x'));
  for (int i = 0; i < 5; i++) rig.server.Poll(20);
  EXPECT_EQ(client.Receive(), "<closed>");
}

TEST(RtspServer, ClientThatDoesNotReadIsDropped) {
  RtspConfig config;
  config.send_timeout_ms = 20;
  Rig rig(config);
  Client client(rig.server.Port());
  ASSERT_FALSE(rig.Play(client).empty());
  const std::vector<std::vector<uint8_t>> picture{std::vector<uint8_t>(1200, 0x55)};
  int sent = 0;
  while (sent < 100000 && rig.server.SendRtp(picture)) sent++;
  EXPECT_GT(sent, 10);
  EXPECT_LT(sent, 100000);
  EXPECT_FALSE(rig.server.Playing());
}

TEST(RtspServer, PictureForAClientThatReadsSlowlyHasOneDeadline) {
  RtspConfig config;
  config.send_timeout_ms = 100;
  Rig rig(config);
  Client client(rig.server.Port());
  ASSERT_FALSE(rig.Play(client).empty());
  // Each packet alone would find room within the deadline, the whole picture does not.
  std::atomic<bool> reading{true};
  std::thread reader([&] {
    while (reading && client.Receive() != "<closed>") std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });
  const std::vector<std::vector<uint8_t>> picture(640, std::vector<uint8_t>(65000, 0x55));
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(rig.server.SendRtp(picture));
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(1));
  EXPECT_FALSE(rig.server.Playing());
  reading = false;
  reader.join();
}

}  // namespace
}  // namespace tron_vision
