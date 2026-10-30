#include <gtest/gtest.h>

#include <string>

#include "mowbite_ntrip/ntrip_client.h"
#include "rtcm_frames.h"

namespace mowbite_ntrip {
namespace {

const char kGga[] = "$GNGGA,120001.00,5200.0000,N,00900.0000,E,4,20,0.6,50.0,M,0.0,M,1.0,0000*5C";

NtripConfig Config(const std::string& mountpoint = "M") {
  NtripConfig c;
  c.host = "h";
  c.mountpoint = mountpoint;
  c.user = "u";
  c.password = "p";
  return c;
}

struct Fixture : ::testing::Test {
  NtripClient client{ClientTiming()};
  std::vector<std::vector<uint8_t>> frames;

  void Bytes(const std::string& text, double now) {
    client.OnBytes(reinterpret_cast<const uint8_t*>(text.data()), text.size(), now, frames);
  }
  void Bytes(const std::vector<uint8_t>& bytes, double now) {
    client.OnBytes(bytes.data(), bytes.size(), now, frames);
  }
  // Connects at now and gets the header and one frame.
  void Stream(double now) {
    ASSERT_TRUE(client.ShouldConnect(now));
    client.BeginAttempt(now);
    Bytes("ICY 200 OK\r\n", now);
    Bytes(MakeFrame(1005, 19), now);
  }
};

TEST_F(Fixture, OffWithoutAccess) {
  EXPECT_EQ(client.state(), ClientState::kOff);
  EXPECT_FALSE(client.ShouldConnect(0));
  client.SetConfig(std::nullopt, 0);
  EXPECT_FALSE(client.ShouldConnect(100));
  EXPECT_STREQ(StateName(client.state()), "off");
}

TEST_F(Fixture, ConnectsAtOnceAndStreams) {
  client.SetConfig(Config(), 10);
  EXPECT_EQ(client.state(), ClientState::kWaiting);
  ASSERT_TRUE(client.ShouldConnect(10));
  const std::string request = client.BeginAttempt(10);
  EXPECT_EQ(request.rfind("GET /M HTTP/1.1\r\n", 0), 0u);
  EXPECT_EQ(client.state(), ClientState::kConnecting);
  EXPECT_FALSE(client.ShouldConnect(10));
  Bytes("HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n", 10.5);
  EXPECT_EQ(client.state(), ClientState::kConnected);
  auto frame = MakeFrame(1077, 40);
  char size[8];
  std::snprintf(size, sizeof(size), "%zx\r\n", frame.size());
  Bytes(std::string(size) + std::string(frame.begin(), frame.end()) + "\r\n", 11);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0], frame);
  EXPECT_EQ(client.frames(), 1u);
  EXPECT_NEAR(*client.last_frame_age(12), 1.0, 1e-9);
  EXPECT_EQ(client.attempts(), 1u);
  EXPECT_FALSE(client.CloseReason(25));
  EXPECT_TRUE(client.TakeEvents().empty());
}

TEST_F(Fixture, NoAnswerOrNoDataCloses) {
  client.SetConfig(Config(), 0);
  client.BeginAttempt(0);
  EXPECT_FALSE(client.CloseReason(15));
  EXPECT_EQ(*client.CloseReason(15.1), "no answer for 15 s");
  client.OnClosed(15.1, *client.CloseReason(15.1));
  EXPECT_EQ(client.last_error(), "no answer for 15 s");
  Stream(20.1);
  EXPECT_EQ(*client.CloseReason(35.2), "no data for 15 s");
}

TEST_F(Fixture, RetryDoublesUpToTheLimitAndFramesResetIt) {
  client.SetConfig(Config(), 0);
  double now = 0;
  const double expected[] = {5, 10, 20, 40, 80, 160, 300, 300};
  for (double wait : expected) {
    ASSERT_TRUE(client.ShouldConnect(now));
    client.BeginAttempt(now);
    client.OnClosed(now, "connect: Connection refused");
    EXPECT_EQ(client.state(), ClientState::kWaiting);
    EXPECT_NEAR(client.retry_in(now), wait, 1e-9);
    EXPECT_FALSE(client.ShouldConnect(now + wait - 0.01));
    now += wait;
  }
  EXPECT_EQ(client.last_error(), "connect: Connection refused");
  Stream(now);
  client.OnClosed(now + 1, "closed by the caster");
  EXPECT_NEAR(client.retry_in(now + 1), 5.0, 1e-9);
}

TEST_F(Fixture, RejectedWaitsLongAndReportsOnce) {
  client.SetConfig(Config(), 0);
  client.BeginAttempt(0);
  Bytes("HTTP/1.1 401 Unauthorized\r\n\r\n", 0.2);
  EXPECT_EQ(client.state(), ClientState::kRejected);
  EXPECT_EQ(*client.CloseReason(0.2), "login refused (401)");
  client.OnClosed(0.2, "");
  EXPECT_EQ(client.last_error(), "login refused (401)");
  EXPECT_NEAR(client.retry_in(0.2), 300.0, 1e-9);
  EXPECT_FALSE(client.ShouldConnect(300));
  auto events = client.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, "REJECTED");
  EXPECT_EQ(events[0].severity, "error");
  EXPECT_EQ(events[0].resume, "user");
  EXPECT_EQ(events[0].code, 401);
  // Refused again: no second event.
  ASSERT_TRUE(client.ShouldConnect(300.2));
  client.BeginAttempt(300.2);
  Bytes("HTTP/1.1 401 Unauthorized\r\n\r\n", 300.3);
  client.OnClosed(300.3, "");
  EXPECT_TRUE(client.TakeEvents().empty());
  EXPECT_EQ(client.state(), ClientState::kRejected);
}

TEST_F(Fixture, ChangedAccessEndsTheWaitAndMayBeReportedAgain) {
  client.SetConfig(Config(), 0);
  client.BeginAttempt(0);
  Bytes("HTTP/1.1 401 Unauthorized\r\n\r\n", 0);
  client.OnClosed(0, "");
  client.TakeEvents();
  client.SetConfig(Config(), 10);  // the same: nothing changes
  EXPECT_FALSE(client.ShouldConnect(10));
  client.SetConfig(Config("N"), 10);
  EXPECT_EQ(client.state(), ClientState::kWaiting);
  ASSERT_TRUE(client.ShouldConnect(10));
  client.BeginAttempt(10);
  Bytes("HTTP/1.1 401 Unauthorized\r\n\r\n", 10);
  EXPECT_EQ(client.TakeEvents().size(), 1u);
}

TEST_F(Fixture, ChangedAccessWhileConnectedReconnectsAtOnce) {
  client.SetConfig(Config(), 0);
  Stream(0);
  client.SetConfig(Config("N"), 5);
  ASSERT_TRUE(client.CloseReason(5));
  EXPECT_EQ(*client.CloseReason(5), "access changed");
  client.OnClosed(5, "");
  EXPECT_TRUE(client.ShouldConnect(5));
  EXPECT_EQ(client.BeginAttempt(5).rfind("GET /N ", 0), 0u);
}

TEST_F(Fixture, RemovedAccessSwitchesOffWithoutLost) {
  client.SetConfig(Config(), 0);
  Stream(0);
  client.SetConfig(std::nullopt, 5);
  EXPECT_EQ(*client.CloseReason(5), "access removed");
  client.OnClosed(5, "");
  EXPECT_EQ(client.state(), ClientState::kOff);
  EXPECT_FALSE(client.ShouldConnect(500));
  client.Tick(500);
  EXPECT_TRUE(client.TakeEvents().empty());
}

TEST_F(Fixture, LostAfterTheGraceThenRestoredOnce) {
  client.SetConfig(Config(), 0);
  Stream(0);
  client.OnClosed(1, "closed by the caster");
  // Reconnects fail for a while: no event per attempt.
  double now = 1;
  for (int i = 0; i < 5; i++) {
    now += client.retry_in(now);
    client.BeginAttempt(now);
    client.OnClosed(now, "connect: Connection refused");
    client.Tick(now);
  }
  ASSERT_GT(now, 30.0);
  auto events = client.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, "LOST");
  EXPECT_EQ(events[0].severity, "warning");
  EXPECT_EQ(events[0].resume, "auto");
  EXPECT_EQ(events[0].limit, 30.0);
  client.Tick(now + 100);
  EXPECT_TRUE(client.TakeEvents().empty());
  now += client.retry_in(now);
  Stream(now);
  events = client.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, "RESTORED");
  EXPECT_EQ(events[0].severity, "info");
  Bytes(MakeFrame(1005, 19), now + 1);
  EXPECT_TRUE(client.TakeEvents().empty());
}

TEST_F(Fixture, ShortGapGivesNoEvent) {
  client.SetConfig(Config(), 0);
  Stream(0);
  client.OnClosed(20, "closed by the caster");
  Stream(25);
  client.Tick(30);
  EXPECT_TRUE(client.TakeEvents().empty());
}

TEST_F(Fixture, RejectedAfterStreamingGivesNoLost) {
  client.SetConfig(Config(), 0);
  Stream(0);
  client.OnClosed(1, "closed by the caster");
  client.BeginAttempt(6);
  Bytes("HTTP/1.1 401 Unauthorized\r\n\r\n", 6);
  client.OnClosed(6, "");
  client.Tick(200);
  auto events = client.TakeEvents();
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, "REJECTED");
}

TEST_F(Fixture, GgaRightAfterTheAnswerThenEveryInterval) {
  client.SetConfig(Config(), 0);
  client.SetGga(kGga);
  client.BeginAttempt(0);
  EXPECT_FALSE(client.TakeGga(0));  // no answer yet
  Bytes("ICY 200 OK\r\n", 0.5);
  auto gga = client.TakeGga(0.5);
  ASSERT_TRUE(gga);
  EXPECT_EQ(*gga, std::string(kGga) + "\r\n");
  EXPECT_FALSE(client.TakeGga(10.4));
  EXPECT_TRUE(client.TakeGga(10.5));
}

TEST_F(Fixture, NoGgaWithoutAPosition) {
  client.SetConfig(Config(), 0);
  client.SetGga("$GNRMC,120001.00,A*00");
  client.SetGga("garbage");
  Stream(0);
  EXPECT_FALSE(client.TakeGga(0));
}

TEST_F(Fixture, CasterErrorsCloseWithTheirLine) {
  client.SetConfig(Config(), 0);
  client.BeginAttempt(0);
  Bytes("SOURCETABLE 200 OK\r\n", 0);
  EXPECT_EQ(*client.CloseReason(0), "mountpoint not offered: SOURCETABLE 200 OK");
  client.OnClosed(0, "");
  EXPECT_EQ(client.state(), ClientState::kWaiting);
  EXPECT_NEAR(client.retry_in(0), 5.0, 1e-9);
}

TEST_F(Fixture, CrcErrorsAreCounted) {
  client.SetConfig(Config(), 0);
  Stream(0);
  auto bad = MakeFrame(1005, 19);
  bad[8] ^= 0xFF;
  Bytes(bad, 1);
  EXPECT_EQ(client.crc_errors(), 1u);
  EXPECT_EQ(client.frames(), 1u);
}

TEST_F(Fixture, ErrorsNeverCarryTheAccess) {
  NtripConfig c = Config();
  c.password = "s3c.r3t.pw";
  client.SetConfig(c, 0);
  client.BeginAttempt(0);
  Bytes("HTTP/1.1 401 Unauthorized\r\n\r\n", 0);
  EXPECT_EQ(client.last_error().find("s3c.r3t.pw"), std::string::npos);
  EXPECT_EQ(client.last_error().find("u:"), std::string::npos);
}

}  // namespace
}  // namespace mowbite_ntrip
