#include <gtest/gtest.h>

#include <string>

#include "mowbite_api/catalog.h"
#include "mowbite_ntrip/ntrip_client.h"
#include "rtcm_frames.h"

namespace mowbite_ntrip {
namespace {

// The events of the client; each goes out as CORRECTIONS and is checked against the catalog of the interface.
std::vector<ClientEvent> Take(NtripClient& client) {
  std::vector<ClientEvent> events = client.TakeEvents();
  for (const auto& event : events) {
    for (const auto& problem : mowbite_api::CheckEvent("CORRECTIONS", CorrectionsDetails(event))) {
      ADD_FAILURE() << problem;
    }
  }
  return events;
}

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
  EXPECT_TRUE(Take(client).empty());
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
  auto events = Take(client);
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
  EXPECT_TRUE(Take(client).empty());
  EXPECT_EQ(client.state(), ClientState::kRejected);
}

TEST_F(Fixture, ChangedAccessEndsTheWaitAndMayBeReportedAgain) {
  client.SetConfig(Config(), 0);
  client.BeginAttempt(0);
  Bytes("HTTP/1.1 401 Unauthorized\r\n\r\n", 0);
  client.OnClosed(0, "");
  Take(client);
  client.SetConfig(Config(), 10);  // the same: nothing changes
  EXPECT_FALSE(client.ShouldConnect(10));
  client.SetConfig(Config("N"), 10);
  EXPECT_EQ(client.state(), ClientState::kWaiting);
  ASSERT_TRUE(client.ShouldConnect(10));
  client.BeginAttempt(10);
  Bytes("HTTP/1.1 401 Unauthorized\r\n\r\n", 10);
  EXPECT_EQ(Take(client).size(), 1u);
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
  EXPECT_TRUE(Take(client).empty());
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
  auto events = Take(client);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, "LOST");
  EXPECT_EQ(events[0].severity, "warning");
  EXPECT_EQ(events[0].resume, "auto");
  EXPECT_EQ(events[0].limit, 30.0);
  client.Tick(now + 100);
  EXPECT_TRUE(Take(client).empty());
  now += client.retry_in(now);
  Stream(now);
  events = Take(client);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, "RESTORED");
  EXPECT_EQ(events[0].severity, "info");
  Bytes(MakeFrame(1005, 19), now + 1);
  EXPECT_TRUE(Take(client).empty());
}

TEST_F(Fixture, ShortGapGivesNoEvent) {
  client.SetConfig(Config(), 0);
  Stream(0);
  client.OnClosed(20, "closed by the caster");
  Stream(25);
  client.Tick(30);
  EXPECT_TRUE(Take(client).empty());
}

TEST_F(Fixture, RejectedAfterStreamingGivesNoLost) {
  client.SetConfig(Config(), 0);
  Stream(0);
  client.OnClosed(1, "closed by the caster");
  client.BeginAttempt(6);
  Bytes("HTTP/1.1 401 Unauthorized\r\n\r\n", 6);
  client.OnClosed(6, "");
  client.Tick(200);
  auto events = Take(client);
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

TEST_F(Fixture, FirstSourceGivesNoEventAndIsSaved) {
  client.SetConfig(Config(), 0);
  Stream(0);
  EXPECT_TRUE(Take(client).empty());
  const auto saved = client.TakeSourceToSave();
  ASSERT_TRUE(saved);
  EXPECT_EQ(*saved, (CorrectionSource{"h", 2101, "M"}));
  EXPECT_FALSE(client.TakeSourceToSave());
  // More frames and a new connection to the same source change nothing.
  Bytes(MakeFrame(1005, 19), 1);
  client.OnClosed(2, "closed by the caster");
  Stream(7);
  EXPECT_TRUE(Take(client).empty());
  EXPECT_FALSE(client.TakeSourceToSave());
}

TEST_F(Fixture, AnotherSourceGivesOneEventWithBoth) {
  client.SetLastSource(CorrectionSource{"caster.example", 2102, "OLD"});
  client.SetConfig(Config("NEW"), 0);
  client.BeginAttempt(0);
  Bytes("ICY 200 OK\r\n", 0);
  // Not before the first frame: only then do its corrections reach the receiver.
  EXPECT_TRUE(Take(client).empty());
  EXPECT_FALSE(client.TakeSourceToSave());
  Bytes(MakeFrame(1005, 19), 0.5);
  auto events = Take(client);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, "SOURCE_CHANGED");
  EXPECT_EQ(events[0].severity, "warning");
  EXPECT_EQ(events[0].resume, "");
  EXPECT_NE(events[0].message.find("may be offset"), std::string::npos);
  ASSERT_TRUE(events[0].source && events[0].previous);
  EXPECT_EQ(*events[0].source, (CorrectionSource{"h", 2101, "NEW"}));
  EXPECT_EQ(*events[0].previous, (CorrectionSource{"caster.example", 2102, "OLD"}));
  EXPECT_EQ(client.TakeSourceToSave(), (CorrectionSource{"h", 2101, "NEW"}));
  // Once per change.
  Bytes(MakeFrame(1005, 19), 1);
  client.OnClosed(2, "closed by the caster");
  Stream(7);
  EXPECT_TRUE(Take(client).empty());
}

TEST_F(Fixture, SameSourceInOtherSpellingVersionOrLoginGivesNoEvent) {
  client.SetLastSource(CorrectionSource{"H", 2101, "/M"});
  NtripConfig config = Config();
  config.version = 1;
  client.SetConfig(config, 0);
  Stream(0);
  EXPECT_TRUE(Take(client).empty());
  // Saved again as now spelled.
  EXPECT_EQ(client.TakeSourceToSave(), (CorrectionSource{"h", 2101, "M"}));
  config.user = "v";
  client.SetConfig(config, 1);
  client.OnClosed(1, "");
  Stream(1);
  EXPECT_TRUE(Take(client).empty());
  EXPECT_FALSE(client.TakeSourceToSave());
}

TEST_F(Fixture, SourceWithoutFramesChangesNothing) {
  // A typo that never streams: the source used last stays.
  client.SetLastSource(CorrectionSource{"h", 2101, "M"});
  client.SetConfig(Config("TYPO"), 0);
  client.BeginAttempt(0);
  Bytes("HTTP/1.1 401 Unauthorized\r\n\r\n", 0);
  client.OnClosed(0, "");
  auto events = Take(client);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, "REJECTED");
  EXPECT_FALSE(client.TakeSourceToSave());
  client.SetConfig(Config(), 1);
  Stream(1);
  EXPECT_TRUE(Take(client).empty());
  EXPECT_FALSE(client.TakeSourceToSave());
}

TEST_F(Fixture, RemovedAccessKeepsTheSourceUsedLast) {
  client.SetConfig(Config(), 0);
  Stream(0);
  client.TakeSourceToSave();
  // Removing gives no event, nor does the same source again; another one does.
  client.SetConfig(std::nullopt, 1);
  client.OnClosed(1, "");
  client.SetConfig(Config(), 2);
  Stream(2);
  EXPECT_TRUE(Take(client).empty());
  client.SetConfig(std::nullopt, 3);
  client.OnClosed(3, "");
  client.SetConfig(Config("N"), 4);
  Stream(4);
  auto events = Take(client);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, "SOURCE_CHANGED");
  EXPECT_EQ(events[0].previous->mountpoint, "M");
  EXPECT_EQ(events[0].source->mountpoint, "N");
}

TEST_F(Fixture, ChangeWhileConnectedCountsFromTheNewFrames) {
  client.SetConfig(Config(), 0);
  Stream(0);
  client.TakeSourceToSave();
  client.SetConfig(Config("N"), 5);
  // Frames still coming on the old connection count for nothing.
  Bytes(MakeFrame(1005, 19), 5);
  EXPECT_EQ(frames.size(), 1u);
  EXPECT_TRUE(Take(client).empty());
  client.OnClosed(5, "");
  Stream(5);
  auto events = Take(client);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].previous->mountpoint, "M");
}

TEST_F(Fixture, LostThenAnotherSourceGivesBoth) {
  client.SetConfig(Config(), 0);
  Stream(0);
  client.OnClosed(1, "closed by the caster");
  client.Tick(40);
  ASSERT_EQ(Take(client).size(), 1u);
  client.SetConfig(Config("N"), 40);
  Stream(40);
  auto events = Take(client);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].reason, "SOURCE_CHANGED");
  EXPECT_EQ(events[1].reason, "RESTORED");
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
