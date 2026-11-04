#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <random>
#include <thread>

#include "xcore_driver/schema.h"
#include "xcore_driver/services.h"
#include "xcore_driver/wire.h"
#include "xcore_driver/xcore_io.h"

using namespace xcore;

namespace {

// A board with only the emergency service on the loopback: announces it, acknowledges the claim, sends its
// heartbeat and notes when the host's emergency input arrives.
class LoopbackBoard {
 public:
  explicit LoopbackBoard(uint16_t group_port) : group_port_(group_port) {
    std::string error;
    EXPECT_TRUE(socket_.Bind(0x7F000001, 0, false, error)) << error;
    sender_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    in_addr loopback{};
    loopback.s_addr = htonl(0x7F000001);
    setsockopt(sender_, IPPROTO_IP, IP_MULTICAST_IF, &loopback, sizeof(loopback));
    int one = 1;
    setsockopt(sender_, IPPROTO_IP, IP_MULTICAST_LOOP, &one, sizeof(one));
    thread_ = std::thread([this] { Run(); });
  }
  ~LoopbackBoard() {
    running_ = false;
    thread_.join();
    ::close(sender_);
  }

  // Monotonic seconds of each emergency input from the host.
  std::vector<double> inputs() {
    std::lock_guard<std::mutex> lock(mutex_);
    return inputs_;
  }

 private:
  void Announce() {
    Announcement a;
    a.service_id = kEmergencyService;
    a.ip = "127.0.0.1";
    a.port = socket_.LocalPort();
    a.description = ExpectedEmergency();
    Header h;
    h.type = MessageType::kAdvertisement;
    Bytes bytes = EncodePacket(h, EncodeAnnouncement(a));
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_port = htons(group_port_);
    inet_pton(AF_INET, kAnnouncementGroup, &to.sin_addr);
    sendto(sender_, bytes.data(), bytes.size(), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
  }

  void Reply(MessageType type, uint8_t arg1) {
    if (!host_) return;
    Header h;
    h.type = type;
    h.service_id = kEmergencyService;
    h.arg1 = arg1;
    Bytes bytes = EncodePacket(h, {});
    socket_.Send(bytes.data(), bytes.size(), *host_);
  }

  void Run() {
    uint8_t buffer[kMaxDatagram + 1];
    double next_announce = 0;
    double next_heartbeat = 0;
    while (running_) {
      const double now = MonotonicSeconds();
      if (now >= next_announce) {
        next_announce = now + 0.5;
        Announce();
      }
      if (now >= next_heartbeat) {
        next_heartbeat = now + 0.2;
        Reply(MessageType::kHeartbeat, 0);
      }
      Endpoint from;
      const long n = socket_.Receive(buffer, sizeof(buffer), from, 10);
      if (n <= 0) continue;
      auto packet = DecodePacket(buffer, static_cast<size_t>(n));
      if (!packet || packet->header.service_id != kEmergencyService) continue;
      host_ = from;
      if (packet->header.type == MessageType::kClaim) Reply(MessageType::kClaim, kClaimAck);
      if (packet->header.type == MessageType::kData && packet->header.arg2 == emergency::kInHighLevel) {
        std::lock_guard<std::mutex> lock(mutex_);
        inputs_.push_back(MonotonicSeconds());
      }
    }
  }

  uint16_t group_port_;
  UdpSocket socket_;
  int sender_ = -1;
  std::optional<Endpoint> host_;
  std::atomic<bool> running_{true};
  std::thread thread_;
  std::mutex mutex_;
  std::vector<double> inputs_;
};

IoConfig LoopbackConfig() {
  std::mt19937 random(std::random_device{}());
  IoConfig config;
  config.interface = "lo";
  // An address and a port of its own, so that tests side by side do not meet.
  config.bind_ip = 0x7F000000 | (1 + random() % 250) << 16 | (1 + random() % 250) << 8 | (2 + random() % 250);
  config.group_port = static_cast<uint16_t>(42000 + random() % 6000);
  config.services[kEmergencyService] = {};
  return config;
}

}  // namespace

// The emergency input reaches the board every 0.25 s while the output thread hangs for 2 s after taking its first
// entries: the I/O thread only fills the outbox, which never waits for the consumer.
TEST(XcoreIo, KeepsThePeriodWhileTheOutputHangs) {
  const IoConfig config = LoopbackConfig();
  LoopbackBoard board(config.group_port);
  XcoreIo io(config, ControlConfig{}, InputMap{});
  auto& out = io.outbox();
  io.Start();

  // Claimed and running: the first input arrives.
  const double start = MonotonicSeconds();
  const auto poll = std::chrono::milliseconds(15);
  while (board.inputs().empty() && MonotonicSeconds() - start < 10.0) std::this_thread::sleep_for(poll);
  ASSERT_FALSE(board.inputs().empty()) << "the board was not claimed";
  EXPECT_TRUE(io.open());

  // The consumer takes what is there and hangs, the way a publish blocked in DDS would.
  out.log.TakeAll();
  const size_t before = board.inputs().size();
  const double hang_start = MonotonicSeconds();
  std::this_thread::sleep_for(std::chrono::seconds(2));
  const double hang_end = MonotonicSeconds();

  auto inputs = board.inputs();
  std::vector<double> during;
  for (size_t i = before; i < inputs.size(); i++) {
    if (inputs[i] >= hang_start && inputs[i] <= hang_end) during.push_back(inputs[i]);
  }
  ASSERT_GE(during.size(), 7u) << "inputs during the hang";
  double largest = during.front() - hang_start;
  for (size_t i = 1; i < during.size(); i++) largest = std::max(largest, during[i] - during[i - 1]);
  // The board's period plus 0.1 s for a loaded machine.
  EXPECT_LE(largest, 0.35);
  // Nothing from the I/O thread waited: no tick came late by more than 50 ms.
  EXPECT_EQ(out.late_ticks.Take().count, 0u);
  EXPECT_EQ(out.send_failed.Take(), 0u);

  io.Stop();
  EXPECT_FALSE(io.open());
}

// What the I/O thread has to say lands in the outbox: the service runs, and the block changed from the start.
TEST(XcoreIo, ReportsThroughTheOutbox) {
  const IoConfig config = LoopbackConfig();
  LoopbackBoard board(config.group_port);
  XcoreIo io(config, ControlConfig{}, InputMap{});
  auto& out = io.outbox();
  io.Start();
  bool running = false;
  bool listening = false;
  const double start = MonotonicSeconds();
  while (!running && MonotonicSeconds() - start < 10.0) {
    out.wakeup.WaitFor(std::chrono::milliseconds(100));
    for (const auto& line : out.log.TakeAll()) {
      if (line.text.rfind("Listening for the board", 0) == 0) listening = true;
      if (line.text == "Service emergency (1) runs") running = true;
    }
  }
  EXPECT_TRUE(listening);
  EXPECT_TRUE(running);
  // Telemetry is missing (no emergency data from this board): the status is blocked and was asked for at once.
  EXPECT_TRUE(out.status_due.exchange(false));
  const auto snapshot = io.Snapshot();
  EXPECT_TRUE(snapshot.linked);
  EXPECT_TRUE(snapshot.status.blocked);
  EXPECT_NE(BlockOf(snapshot.status), 0u);
  io.Stop();
}

TEST(XcoreIo, BlockOfLeavesOutTheRest) {
  ControlStatus a;
  ControlStatus b;
  b.lift = true;
  b.moving = true;
  b.blade_on = true;
  EXPECT_EQ(BlockOf(a), BlockOf(b));
  b.held = true;
  EXPECT_NE(BlockOf(a), BlockOf(b));
  b = a;
  b.emergency_reason = kReasonStop;
  EXPECT_NE(BlockOf(a), BlockOf(b));
}
