#ifndef XCORE_DRIVER_XCORE_IO_H
#define XCORE_DRIVER_XCORE_IO_H

#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "mowbite_outbox/outbox.h"
#include "xcore_driver/gnss.h"
#include "xcore_driver/link.h"
#include "xcore_driver/udp.h"
#include "xcore_driver/xcore_control.h"

namespace xcore {

struct IoConfig {
  std::string interface = "eth0";
  // Host byte order.
  uint32_t bind_ip = 0;
  std::string group = kAnnouncementGroup;
  uint16_t group_port = kAnnouncementPort;
  uint16_t expected_major = 1;
  int contest_losses = 2;
  double contest_window_s = 10.0;
  std::map<uint16_t, std::vector<Section>> services;
  // The outputs of the GPS service go to the output thread; without it they are dropped.
  bool gnss = false;
};

// Seconds on the monotonic clock, the time of the I/O thread and the control.
double MonotonicSeconds();
// Wall clock in ns since the epoch: the stamp of a message, taken when its values were read.
int64_t WallNanoseconds();

struct TwistReading {
  int64_t stamp_ns = 0;
  WheelTwist twist;
};

struct ImuReading {
  int64_t stamp_ns = 0;
  // Acceleration x, y, z, then rotation x, y, z.
  std::array<double, 6> axes{};
};

struct GpsReading {
  int64_t stamp_ns = 0;
  std::vector<Output> outputs;
  // Since the last RTCM passed to the board; none before the first.
  std::optional<double> correction_age_s;
};

// What hw/status and hw/capabilities are made of, taken under one lock.
struct IoSnapshot {
  ControlStatus status;
  bool linked = false;
  uint64_t link_received = 0;
  uint64_t link_dropped = 0;
  std::string firmware_version;
};

// The part of hw/status that goes out at once when it changes: emergency, latch, reason bits and the hold.
uint64_t BlockOf(const ControlStatus& status);

// Why another host could talk to the board from this machine: UDP sockets of other programs at bind_ip or at the
// announcement port, in the network namespace of this process. nullopt when there are none. Reads /proc: never from
// the I/O thread while the board is claimed.
std::optional<std::string> ForeignHosts(uint32_t bind_ip, uint16_t group_port);

// From the I/O thread to the output thread. Nothing here waits for the consumer.
struct IoOutbox {
  mowbite_outbox::Wakeup wakeup;
  // ~1.3 s of the streams at their rates: a longer hang of the output stops the mower anyway (S2, the logic).
  mowbite_outbox::Ring<TwistReading> twists{64, mowbite_outbox::Full::kDropOldest};
  mowbite_outbox::Ring<ImuReading> imu{128, mowbite_outbox::Full::kDropOldest};
  mowbite_outbox::Ring<GpsReading> gps{8, mowbite_outbox::Full::kDropOldest};
  mowbite_outbox::LogQueue log{&wakeup};
  // Datagrams the socket did not take.
  mowbite_outbox::Counter send_failed;
  // RTCM bytes dropped while the GPS service did not run.
  mowbite_outbox::Counter rtcm_dropped_closed;
  // Ticks later than kLateTickS, with the largest delay in s.
  mowbite_outbox::Peak late_ticks;
  // hw/status at once (a block changed) and hw/capabilities (the firmware answered).
  std::atomic<bool> status_due{false};
  std::atomic<bool> capabilities_due{false};
};

// The I/O thread of the driver without ROS: owns both sockets, reads announcements and service datagrams, runs link
// and control every 10 ms and sends their datagrams. It calls nothing but the sockets, the control under its lock,
// the outbox and the clocks: no DDS, no log line, no file while the board is claimed.
class XcoreIo {
 public:
  static constexpr double kTickS = 0.01;
  // A tick this much after its time counts as late.
  static constexpr double kLateTickS = 0.05;

  XcoreIo(const IoConfig& config, const ControlConfig& control, const InputMap& inputs);
  ~XcoreIo();
  XcoreIo(const XcoreIo&) = delete;
  XcoreIo& operator=(const XcoreIo&) = delete;

  void Start();
  // Asks the loop to end; it sends the stop inputs and closes the sockets. Safe from any thread.
  void RequestStop();
  // RequestStop and wait for the end of the loop.
  void Stop();

  IoOutbox& outbox() {
    return outbox_;
  }

  // From the executor and the output thread: computing under the lock, nothing else.
  void OnCmdVel(double linear, double angular);
  bool LatchEmergency(uint32_t bits);
  // The caller has checked for other hosts (ForeignHosts) before.
  std::optional<ReleaseRefusal> ReleaseEmergency();
  BladeResult SetBlade(const BladeRequest& request, uint32_t* emergency_reason);
  // False when the queue was full; the bytes dropped so far in rtcm_dropped_bytes.
  bool PushRtcm(const uint8_t* data, size_t size);
  uint64_t rtcm_dropped_bytes();
  void SetHold(bool held);
  void LatchForeignHost(const std::string& why);
  // While the sockets are open, i.e. the board is or is being claimed.
  bool open() const {
    return open_;
  }
  IoSnapshot Snapshot();

 private:
  void Loop();
  bool Open(double now);
  void Close();
  void HandleDatagram(const uint8_t* data, size_t size, const Endpoint& from);
  void Tick(double now);
  void Send(const std::vector<Datagram>& datagrams, std::optional<double> computed = std::nullopt);
  void AskFirmware(double now);
  // The link while the service socket is open; called with mutex_ held.
  XcoreLink& Link();
  void HandleEvents(const std::vector<LinkEvent>& events);
  bool OnFirmwareAnswer(const LinkEvent& e, double now);
  // Wakes the output thread for hw/status when the block changed since the last request.
  void CheckBlock();
  void Shutdown();

  const IoConfig config_;
  IoOutbox outbox_;
  std::mutex mutex_;
  XcoreControl control_;
  std::optional<XcoreLink> link_;
  std::optional<uint16_t> major_;
  std::optional<double> next_rpc_s_;
  std::string firmware_version_;
  RtcmQueue rtcm_;
  std::optional<double> last_rtcm_s_;

  // Only the I/O thread.
  UdpSocket announcements_;
  UdpSocket service_socket_;
  uint64_t requested_block_ = 0;
  mowbite_outbox::Throttle open_problem_{10.0};
  mowbite_outbox::Throttle foreign_at_start_{10.0};

  std::atomic<bool> open_{false};
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace xcore

#endif  // XCORE_DRIVER_XCORE_IO_H
