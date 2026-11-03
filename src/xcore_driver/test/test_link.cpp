#include <gtest/gtest.h>

#include "xcore_driver/link.h"
#include "xcore_driver/services.h"

using namespace xcore;

namespace {

const Endpoint kBoard{0xAC104E96, 49154};  // 172.16.78.150
const Endpoint kHost{0xAC104E01, 50000};   // 172.16.78.1

LinkConfig Config() {
  LinkConfig c;
  c.host = kHost;
  Writer ticks;
  ticks.F64(2025.0);
  Writer distance;
  distance.F64(0.3556);
  c.services[kEmergencyService] = {};
  c.services[kDiffDriveService] = {{diff_drive::kRegTicksPerMeter, ticks.Take()},
                                   {diff_drive::kRegWheelDistance, distance.Take()}};
  return c;
}

Bytes AnnouncementPacket(uint16_t sid, const ServiceDescription& description, uint16_t port = kBoard.port) {
  Announcement a;
  a.service_id = sid;
  a.ip = "172.16.78.150";
  a.port = port;
  a.description = description;
  Header h;
  h.type = MessageType::kAdvertisement;
  h.flags = kFlagReboot;
  return EncodePacket(h, EncodeAnnouncement(a));
}

Bytes FromBoard(uint16_t sid, MessageType type, uint8_t arg1 = 0, uint16_t arg2 = 0, const Bytes& payload = {},
                uint64_t timestamp = 0) {
  Header h;
  h.type = type;
  h.service_id = sid;
  h.arg1 = arg1;
  h.arg2 = arg2;
  h.timestamp = timestamp;
  return EncodePacket(h, payload);
}

struct Harness {
  XcoreLink link{Config()};
  std::vector<LinkEvent> events;
  std::vector<Output> outputs;

  void Announce(uint16_t sid, const ServiceDescription& d, double now, uint16_t port = kBoard.port) {
    auto bytes = AnnouncementPacket(sid, d, port);
    link.OnAnnouncement(bytes.data(), bytes.size(), now, events);
  }
  void Receive(const Bytes& bytes, double now, Endpoint from = kBoard) {
    link.OnDatagram(bytes.data(), bytes.size(), from, now, outputs, events);
  }
  std::vector<Packet> Tick(double now) {
    std::vector<Packet> packets;
    for (const auto& d : link.Tick(now, events)) {
      EXPECT_EQ(d.to, kBoard);
      packets.push_back(*DecodePacket(d.bytes.data(), d.bytes.size()));
    }
    return packets;
  }
  bool Has(LinkEvent::Kind kind, uint16_t sid) const {
    for (const auto& e : events) {
      if (e.kind == kind && e.service_id == sid) return true;
    }
    return false;
  }
};

}  // namespace

TEST(Link, ClaimsAnAnnouncedService) {
  Harness h;
  EXPECT_TRUE(h.Tick(0).empty());
  h.Announce(kEmergencyService, ExpectedEmergency(), 0);
  auto out = h.Tick(0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].header.type, MessageType::kClaim);
  EXPECT_EQ(out[0].header.service_id, kEmergencyService);
  EXPECT_EQ(out[0].header.arg1, kClaimRequest);
  // Host address as a number, little endian; port; interval in µs.
  EXPECT_EQ(out[0].payload, (Bytes{0x01, 0x4E, 0x10, 0xAC, 0x50, 0xC3, 0x40, 0x42, 0x0F, 0x00}));
  // Repeated every second until acknowledged.
  EXPECT_TRUE(h.Tick(0.5).empty());
  EXPECT_EQ(h.Tick(1.0).size(), 1u);
  h.Receive(FromBoard(kEmergencyService, MessageType::kClaim, kClaimAck), 1.1);
  EXPECT_EQ(h.link.State(kEmergencyService), ServiceState::kRunning);
  EXPECT_TRUE(h.Has(LinkEvent::Kind::kRunning, kEmergencyService));
  EXPECT_TRUE(h.Tick(2.0).empty());
  // The same announcement again changes nothing.
  h.Announce(kEmergencyService, ExpectedEmergency(), 2.0);
  EXPECT_EQ(h.link.State(kEmergencyService), ServiceState::kRunning);
}

TEST(Link, ConfiguresAServiceWithRegisters) {
  Harness h;
  h.Announce(kDiffDriveService, ExpectedDiffDrive(), 0);
  h.Tick(0);
  h.Receive(FromBoard(kDiffDriveService, MessageType::kClaim, kClaimAck), 0.1);
  EXPECT_EQ(h.link.State(kDiffDriveService), ServiceState::kConfiguring);
  auto out = h.Tick(0.1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].header.type, MessageType::kTransaction);
  EXPECT_EQ(out[0].header.arg1, kTransactionConfig);
  auto sections = DecodeSections(out[0].payload);
  ASSERT_TRUE(sections);
  ASSERT_EQ(sections->size(), 2u);
  EXPECT_EQ(Reader((*sections)[1].value).F64(), 0.3556);
  // A request after the configuration went out: it is sent again.
  h.Receive(FromBoard(kDiffDriveService, MessageType::kConfigurationRequest), 1.1);
  ASSERT_EQ(h.Tick(1.1).size(), 1u);
  EXPECT_TRUE(h.Tick(1.5).empty());
  // Data makes it run at once.
  Writer ticks;
  ticks.U32(5);
  ticks.U32(7);
  h.Receive(FromBoard(kDiffDriveService, MessageType::kTransaction, kTransactionData, 0,
                      EncodeSections({{diff_drive::kOutTicks, ticks.Take()}, {diff_drive::kOutLeftStatus, {200}}}),
                      123456),
            1.6);
  EXPECT_EQ(h.link.State(kDiffDriveService), ServiceState::kRunning);
  ASSERT_EQ(h.outputs.size(), 2u);
  EXPECT_EQ(h.outputs[0].channel_id, diff_drive::kOutTicks);
  EXPECT_EQ(h.outputs[0].board_time_us, 123456u);
  // A running service that asks again lost its configuration.
  h.Receive(FromBoard(kDiffDriveService, MessageType::kConfigurationRequest), 2.0);
  EXPECT_TRUE(h.Has(LinkEvent::Kind::kRestarted, kDiffDriveService));
  EXPECT_EQ(h.Tick(2.0).size(), 1u);
}

TEST(Link, RunsAfterTheRequestsStop) {
  Harness h;
  h.Announce(kDiffDriveService, ExpectedDiffDrive(), 0);
  h.Tick(0);
  // The acknowledgement got lost, a request comes instead.
  h.Receive(FromBoard(kDiffDriveService, MessageType::kConfigurationRequest), 1.0);
  EXPECT_EQ(h.link.State(kDiffDriveService), ServiceState::kConfiguring);
  h.Tick(1.0);
  h.Receive(FromBoard(kDiffDriveService, MessageType::kHeartbeat), 1.4);
  h.Receive(FromBoard(kDiffDriveService, MessageType::kHeartbeat), 1.9);
  h.Tick(2.0);
  EXPECT_EQ(h.link.State(kDiffDriveService), ServiceState::kConfiguring);
  h.Receive(FromBoard(kDiffDriveService, MessageType::kHeartbeat), 2.4);
  h.Tick(2.5);
  EXPECT_EQ(h.link.State(kDiffDriveService), ServiceState::kRunning);
}

TEST(Link, RefusesServicesThatDoNotMatch) {
  Harness h;
  ServiceDescription other = ExpectedDiffDrive();
  other.version = 2;
  h.Announce(kDiffDriveService, other, 0);
  EXPECT_EQ(h.link.State(kDiffDriveService), ServiceState::kIncompatible);
  ASSERT_TRUE(h.Has(LinkEvent::Kind::kIncompatible, kDiffDriveService));
  EXPECT_TRUE(h.Tick(0).empty());
  // The same once more gives no second event.
  h.events.clear();
  h.Announce(kDiffDriveService, other, 1);
  EXPECT_TRUE(h.events.empty());
  // A register of another size than the configuration.
  ServiceDescription narrow = ExpectedDiffDrive();
  narrow.registers[0].type = *ParseValueType("float");
  h.Announce(kDiffDriveService, narrow, 2);
  ASSERT_FALSE(h.events.empty());
  EXPECT_NE(h.events.back().text.find("register 0"), std::string::npos);
  // A fitting one is claimed.
  h.Announce(kDiffDriveService, ExpectedDiffDrive(), 3);
  EXPECT_EQ(h.link.State(kDiffDriveService), ServiceState::kClaiming);
  // Services not in the configuration are ignored.
  h.Announce(kMowerService, ExpectedMower(), 3);
  EXPECT_EQ(h.link.State(kMowerService), ServiceState::kUnseen);
}

TEST(Link, LosesASilentServiceAndClaimsItAgain) {
  Harness h;
  h.Announce(kEmergencyService, ExpectedEmergency(), 0);
  h.Tick(0);
  h.Receive(FromBoard(kEmergencyService, MessageType::kClaim, kClaimAck), 0.1);
  h.Receive(FromBoard(kEmergencyService, MessageType::kHeartbeat), 0.55);
  EXPECT_TRUE(h.Tick(2.0).empty());
  auto out = h.Tick(2.1);
  EXPECT_TRUE(h.Has(LinkEvent::Kind::kLost, kEmergencyService));
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].header.type, MessageType::kClaim);
  EXPECT_FALSE(h.link.Input(kEmergencyService, emergency::kInHighLevel, Bytes(4, 0)));
}

TEST(Link, NewEndpointIsARestart) {
  Harness h;
  h.Announce(kEmergencyService, ExpectedEmergency(), 0);
  h.Tick(0);
  h.Receive(FromBoard(kEmergencyService, MessageType::kClaim, kClaimAck), 0.1);
  h.Announce(kEmergencyService, ExpectedEmergency(), 0.5, 49160);
  EXPECT_TRUE(h.Has(LinkEvent::Kind::kRestarted, kEmergencyService));
  EXPECT_EQ(h.link.State(kEmergencyService), ServiceState::kClaiming);
  auto out = h.link.Tick(0.5, h.events);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].to.port, 49160);
  // The old endpoint is not heard any more.
  h.Receive(FromBoard(kEmergencyService, MessageType::kClaim, kClaimAck), 0.6);
  EXPECT_EQ(h.link.State(kEmergencyService), ServiceState::kClaiming);
  EXPECT_EQ(h.link.dropped(), 1u);
}

TEST(Link, NeverClaimsOnADebugOrBootPort) {
  for (uint16_t port : {8007, 10000, 65102, 65103, 65104}) {
    Harness h;
    h.Announce(kEmergencyService, ExpectedEmergency(), 0, port);
    EXPECT_TRUE(h.Has(LinkEvent::Kind::kIncompatible, kEmergencyService)) << port;
    EXPECT_EQ(h.link.State(kEmergencyService), ServiceState::kIncompatible);
    EXPECT_TRUE(h.link.Tick(0, h.events).empty()) << port;
  }
}

TEST(Link, InputsAndOutputsAreChecked) {
  Harness h;
  h.Announce(kEmergencyService, ExpectedEmergency(), 0);
  h.Tick(0);
  EXPECT_FALSE(h.link.Input(kEmergencyService, emergency::kInHighLevel, Bytes(4, 0)));
  h.Receive(FromBoard(kEmergencyService, MessageType::kClaim, kClaimAck), 0.1);
  auto d = h.link.Input(kEmergencyService, emergency::kInHighLevel, Bytes{1, 0, 0, 0});
  ASSERT_TRUE(d);
  auto p = DecodePacket(d->bytes.data(), d->bytes.size());
  EXPECT_EQ(p->header.type, MessageType::kData);
  EXPECT_EQ(p->header.arg2, emergency::kInHighLevel);
  EXPECT_FALSE(h.link.Input(kEmergencyService, emergency::kInHighLevel, Bytes(6, 0)));
  EXPECT_FALSE(h.link.Input(kEmergencyService, 9, Bytes(4, 0)));
  // Output of the wrong size and from someone else are dropped.
  h.Receive(FromBoard(kEmergencyService, MessageType::kData, 0, emergency::kOutReason, Bytes{1, 0, 0}), 0.2);
  h.Receive(FromBoard(kEmergencyService, MessageType::kData, 0, emergency::kOutReason, Bytes{1, 0}), 0.2,
            Endpoint{0xAC104E05, 49154});
  EXPECT_TRUE(h.outputs.empty());
  h.Receive(FromBoard(kEmergencyService, MessageType::kData, 0, emergency::kOutReason, Bytes{1, 0}), 0.2);
  ASSERT_EQ(h.outputs.size(), 1u);
  EXPECT_EQ(h.link.dropped(), 2u);
}

TEST(Link, RtcmMayRunLongerThanAnnounced) {
  LinkConfig c;
  c.host = kHost;
  c.services[kGpsService] = {};
  XcoreLink link(c);
  std::vector<LinkEvent> events;
  std::vector<Output> outputs;
  auto a = AnnouncementPacket(kGpsService, ExpectedGps());
  link.OnAnnouncement(a.data(), a.size(), 0, events);
  link.Tick(0, events);
  auto ack = FromBoard(kGpsService, MessageType::kClaim, kClaimAck);
  link.OnDatagram(ack.data(), ack.size(), kBoard, 0.1, outputs, events);
  link.Tick(0.1, events);
  auto heartbeat = FromBoard(kGpsService, MessageType::kHeartbeat);
  link.OnDatagram(heartbeat.data(), heartbeat.size(), kBoard, 1.0, outputs, events);
  link.Tick(1.7, events);
  ASSERT_EQ(link.State(kGpsService), ServiceState::kRunning);
  EXPECT_TRUE(link.Input(kGpsService, gps::kInRtcm, Bytes(1000, 0xD3)));
  EXPECT_TRUE(link.Input(kGpsService, gps::kInRtcm, Bytes(kMaxPayload, 0xD3)));
  EXPECT_FALSE(link.Input(kGpsService, gps::kInRtcm, Bytes(kMaxPayload + 1, 0xD3)));
}

TEST(Link, Rpc) {
  Harness h;
  EXPECT_FALSE(h.link.CallRpc(kEmergencyService, 0, 0));
  h.Announce(kEmergencyService, ExpectedEmergency(), 0);
  h.Tick(0);
  h.Receive(FromBoard(kEmergencyService, MessageType::kClaim, kClaimAck), 0.1);
  ASSERT_TRUE(h.link.CallRpc(kEmergencyService, 1, 0.1));
  auto out = h.Tick(0.1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].header.type, MessageType::kRpcRequest);
  EXPECT_EQ(out[0].header.arg1, 1);
  uint16_t call = out[0].header.arg2;
  h.Receive(FromBoard(kEmergencyService, MessageType::kRpcResponse, 0, call, Bytes{1, 0}), 0.2);
  ASSERT_FALSE(h.events.empty());
  EXPECT_EQ(h.events.back().kind, LinkEvent::Kind::kRpcResult);
  EXPECT_EQ(h.events.back().function_id, 1);
  EXPECT_EQ(h.events.back().value, (Bytes{1, 0}));
  // Unanswered: three attempts a second apart, then it fails.
  h.link.CallRpc(kEmergencyService, 0, 1.0);
  int sent = 0;
  for (int i = 10; i <= 40; i++) {
    double t = i * 0.1;
    h.Receive(FromBoard(kEmergencyService, MessageType::kHeartbeat), t);
    sent += static_cast<int>(h.Tick(t).size());
  }
  EXPECT_EQ(sent, 3);
  EXPECT_TRUE(h.Has(LinkEvent::Kind::kRpcFailed, kEmergencyService));
}

namespace {

// Claims the emergency service at the given board port and lets it run; returns the time after.
double RunEmergency(Harness& h, double now, uint16_t port = kBoard.port) {
  h.link.Tick(now, h.events);
  h.Receive(FromBoard(kEmergencyService, MessageType::kClaim, kClaimAck), now + 0.1, Endpoint{kBoard.ip, port});
  EXPECT_EQ(h.link.State(kEmergencyService), ServiceState::kRunning);
  return now + 0.1;
}

int Count(const Harness& h, LinkEvent::Kind kind) {
  int n = 0;
  for (const auto& e : h.events) n += e.kind == kind;
  return n;
}

}  // namespace

TEST(Link, TwoLossesAtTheSameEndpointMeanAnotherHost) {
  Harness h;
  h.Announce(kEmergencyService, ExpectedEmergency(), 0);
  double t = RunEmergency(h, 0);
  // Another host claims it: the board answers there, this side hears nothing for 1.5 s.
  h.Tick(t + 1.6);
  EXPECT_EQ(Count(h, LinkEvent::Kind::kLost), 1);
  EXPECT_EQ(Count(h, LinkEvent::Kind::kContested), 0);
  t = RunEmergency(h, t + 1.6);
  h.Tick(t + 1.6);
  ASSERT_EQ(Count(h, LinkEvent::Kind::kContested), 1);
  for (const auto& e : h.events) {
    if (e.kind == LinkEvent::Kind::kContested) {
      EXPECT_NE(e.text.find("another host"), std::string::npos);
    }
  }
}

TEST(Link, LossesAfterARestartOrFarApartAreNoContest) {
  Harness h;
  h.Announce(kEmergencyService, ExpectedEmergency(), 0);
  double t = RunEmergency(h, 0);
  h.Tick(t + 1.6);
  // The board restarts and announces a new endpoint: the count starts again.
  h.Announce(kEmergencyService, ExpectedEmergency(), t + 2, 49160);
  t = RunEmergency(h, t + 2, 49160);
  h.link.Tick(t + 1.6, h.events);
  EXPECT_EQ(Count(h, LinkEvent::Kind::kContested), 0);
  // A loss while only claiming counts not; nor does one more than 10 s after the last.
  h.link.Tick(t + 5, h.events);
  t = RunEmergency(h, t + 11, 49160);
  h.link.Tick(t + 1.6, h.events);
  EXPECT_EQ(Count(h, LinkEvent::Kind::kLost), 3);
  EXPECT_EQ(Count(h, LinkEvent::Kind::kContested), 0);
}

TEST(Link, NeverWritesTheChargeCurrentOverride) {
  LinkConfig c = Config();
  Writer empty;
  empty.F32(24.5f);
  c.services[kPowerService] = {{power::kRegEmptyVoltage, empty.Take()},
                               {power::kRegOverrideChargeCurrentLimit, Bytes{1}}};
  XcoreLink link(c);
  ServiceDescription d = ExpectedPower();
  d.registers.push_back(Channel{power::kRegOverrideChargeCurrentLimit, "override", *ParseValueType("uint8_t")});
  Announcement a;
  a.service_id = kPowerService;
  a.ip = "172.16.78.150";
  a.port = kBoard.port;
  a.description = d;
  Header header;
  header.type = MessageType::kAdvertisement;
  auto bytes = EncodePacket(header, EncodeAnnouncement(a));
  std::vector<LinkEvent> events;
  std::vector<Output> outputs;
  link.OnAnnouncement(bytes.data(), bytes.size(), 0, events);
  link.Tick(0, events);
  auto ack = FromBoard(kPowerService, MessageType::kClaim, kClaimAck);
  link.OnDatagram(ack.data(), ack.size(), kBoard, 0.1, outputs, events);
  auto out = link.Tick(0.1, events);
  ASSERT_EQ(out.size(), 1u);
  auto packet = DecodePacket(out[0].bytes.data(), out[0].bytes.size());
  ASSERT_TRUE(packet);
  EXPECT_EQ(packet->header.arg1, kTransactionConfig);
  auto sections = DecodeSections(packet->payload);
  ASSERT_TRUE(sections);
  ASSERT_EQ(sections->size(), 1u);
  EXPECT_EQ((*sections)[0].id, power::kRegEmptyVoltage);
  EXPECT_TRUE(ForbiddenRegister(kPowerService, 10));
  EXPECT_FALSE(ForbiddenRegister(kDiffDriveService, 10));
}
