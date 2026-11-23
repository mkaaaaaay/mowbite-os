#ifndef XCORE_DRIVER_LINK_H
#define XCORE_DRIVER_LINK_H

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "xcore_driver/schema.h"
#include "xcore_driver/udp.h"
#include "xcore_driver/wire.h"

namespace xcore {

struct LinkConfig {
  // Where the board sends to: the address and port of the host's service socket.
  Endpoint host;
  // Asked for in the claim; the board sends its heartbeat every (interval - 0.1 s) / 2.
  uint32_t heartbeat_interval_us = 1000000;
  double claim_retry_s = 1.0;
  // A service that sent nothing for this long counts as lost and is claimed again.
  double service_timeout_s = 1.5;
  // A configured service runs once it asked for no configuration for this long, or sooner when it sends data.
  double config_settle_s = 1.5;
  double rpc_timeout_s = 1.0;
  int rpc_attempts = 3;
  // A running service lost this often within contest_window_s, without announcing a new endpoint in between, has
  // another host claiming it: the board answers whoever claimed last.
  int contest_losses = 2;
  double contest_window_s = 10.0;
  // The services to claim, each with its configuration in register order; an empty list for services without
  // registers. Services not listed are left alone. Forbidden registers (ForbiddenRegister) are dropped.
  std::map<uint16_t, std::vector<Section>> services;
};

enum class ServiceState { kUnseen, kClaiming, kConfiguring, kRunning, kIncompatible };

const char* ServiceStateText(ServiceState state);

struct LinkEvent {
  enum class Kind { kRunning, kLost, kRestarted, kIncompatible, kRpcResult, kRpcFailed, kContested };
  Kind kind = Kind::kRunning;
  uint16_t service_id = 0;
  std::string text;
  // RPC: function, status (0 ok, 1 busy, 2 error) and the raw return value.
  uint8_t function_id = 0;
  uint8_t status = 0;
  Bytes value;
};

struct Output {
  uint16_t service_id = 0;
  uint16_t channel_id = 0;
  Bytes value;
  uint64_t board_time_us = 0;
};

struct Datagram {
  Endpoint to;
  Bytes bytes;
};

// The host side of the services of one xCore: claims the services listed in the config once their announcement
// matches what the driver expects, configures them, watches their heartbeat and claims them again when they restart
// or fall silent. Time is passed in; nothing here touches a socket.
class XcoreLink {
 public:
  explicit XcoreLink(const LinkConfig& config);

  // A datagram from the announcement group.
  // sender_ip is where the announcement came from: a service must announce its own address.
  void OnAnnouncement(const uint8_t* data, size_t size, uint32_t sender_ip, double now, std::vector<LinkEvent>& events);
  // A datagram on the service socket; outputs of running services are appended.
  void OnDatagram(const uint8_t* data, size_t size, const Endpoint& from, double now, std::vector<Output>& outputs,
                  std::vector<LinkEvent>& events);
  std::vector<Datagram> Tick(double now, std::vector<LinkEvent>& events);

  // A value for an input; nullopt unless the service runs and the value fits the input's announced type.
  std::optional<Datagram> Input(uint16_t service_id, uint16_t channel_id, const Bytes& value);
  // Queues a call without parameters; the result comes as an event. False unless the service is claimed.
  bool CallRpc(uint16_t service_id, uint8_t function_id, double now);

  ServiceState State(uint16_t service_id) const;
  bool AllRunning() const;
  uint64_t received() const {
    return received_;
  }
  uint64_t dropped() const {
    return dropped_;
  }

 private:
  struct Rpc {
    uint8_t function_id;
    uint16_t call_id;
    int attempts;
    double next_s;
  };
  struct Service {
    ServiceState state = ServiceState::kUnseen;
    Endpoint endpoint;
    std::optional<ServiceDescription> description;
    double last_heard_s = 0;
    double next_claim_s = 0;
    // When the configuration went out last, and the last request for it.
    std::optional<double> config_sent_s;
    double last_request_s = 0;
    std::vector<Rpc> rpcs;
    std::string incompatible;
    // When the service was lost while running, since its last announcement.
    std::vector<double> running_losses_s;
  };

  Datagram Make(const Service& service, uint16_t service_id, MessageType type, uint8_t arg1, uint16_t arg2,
                const Bytes& payload);
  void SetRunning(uint16_t id, Service& service, std::vector<LinkEvent>& events);
  void Lose(uint16_t id, Service& service, const std::string& why, double now, std::vector<LinkEvent>& events);
  bool NeedsConfig(uint16_t id) const;
  // Problems of the configuration against the announced registers.
  std::vector<std::string> CheckConfig(uint16_t id, const ServiceDescription& description) const;

  LinkConfig config_;
  std::map<uint16_t, Service> services_;
  uint16_t sequence_ = 0;
  uint16_t next_call_id_ = 1;
  uint64_t received_ = 0;
  uint64_t dropped_ = 0;
};

}  // namespace xcore

#endif  // XCORE_DRIVER_LINK_H
