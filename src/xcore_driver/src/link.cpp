#include "xcore_driver/link.h"

#include <algorithm>

#include "xcore_driver/services.h"

namespace xcore {

namespace {

const Channel* Find(const std::vector<Channel>& channels, uint16_t id) {
  for (const auto& c : channels) {
    if (c.id == id) return &c;
  }
  return nullptr;
}

LinkEvent Event(LinkEvent::Kind kind, uint16_t service_id, const std::string& text) {
  LinkEvent e;
  e.kind = kind;
  e.service_id = service_id;
  e.text = text;
  return e;
}

}  // namespace

const char* ServiceStateText(ServiceState state) {
  switch (state) {
    case ServiceState::kUnseen:
      return "unseen";
    case ServiceState::kClaiming:
      return "claiming";
    case ServiceState::kConfiguring:
      return "configuring";
    case ServiceState::kRunning:
      return "running";
    case ServiceState::kIncompatible:
      return "incompatible";
  }
  return "unknown";
}

XcoreLink::XcoreLink(const LinkConfig& config) : config_(config) {
  for (const auto& entry : config_.services) services_[entry.first];
}

bool XcoreLink::NeedsConfig(uint16_t id) const {
  // A service with registers waits for a configuration, even if none of them is required.
  auto it = services_.find(id);
  return it != services_.end() && it->second.description && !it->second.description->registers.empty();
}

std::vector<std::string> XcoreLink::CheckConfig(uint16_t id, const ServiceDescription& description) const {
  std::vector<std::string> problems;
  for (const auto& section : config_.services.at(id)) {
    const Channel* reg = Find(description.registers, section.id);
    if (!reg) {
      problems.push_back("register " + std::to_string(section.id) + " is not announced");
    } else if (!FitsType(reg->type, section.value.size())) {
      problems.push_back("register " + std::to_string(section.id) + " (" + reg->name + ") does not take " +
                         std::to_string(section.value.size()) + " bytes");
    }
  }
  return problems;
}

void XcoreLink::OnAnnouncement(const uint8_t* data, size_t size, double now, std::vector<LinkEvent>& events) {
  auto packet = DecodePacket(data, size);
  if (!packet || packet->header.type != MessageType::kAdvertisement) return;
  auto announcement = ParseAnnouncement(packet->payload.data(), packet->payload.size());
  if (!announcement) return;
  uint16_t id = announcement->service_id;
  auto it = services_.find(id);
  if (it == services_.end()) return;
  Service& s = it->second;
  Endpoint endpoint;
  // A service without a port yet cannot be claimed.
  if (!ParseIpv4(announcement->ip, endpoint.ip) || announcement->port == 0) return;
  endpoint.port = announcement->port;
  if (s.state != ServiceState::kUnseen && s.state != ServiceState::kIncompatible && endpoint == s.endpoint) return;

  std::vector<std::string> problems;
  const ExpectedService* expected = ExpectedFor(id);
  if (expected) problems = CheckDescription(*expected, announcement->description);
  if (problems.empty()) problems = CheckConfig(id, announcement->description);
  if (!problems.empty()) {
    std::string text;
    for (const auto& p : problems) text += (text.empty() ? "" : "; ") + p;
    if (s.state != ServiceState::kIncompatible || s.incompatible != text) {
      events.push_back(Event(LinkEvent::Kind::kIncompatible, id, text));
    }
    s.state = ServiceState::kIncompatible;
    s.incompatible = text;
    s.endpoint = endpoint;
    return;
  }
  if (s.state != ServiceState::kUnseen && s.state != ServiceState::kIncompatible) {
    events.push_back(Event(LinkEvent::Kind::kRestarted, id, "announced at a new endpoint"));
  }
  s = Service{};
  s.state = ServiceState::kClaiming;
  s.endpoint = endpoint;
  s.description = announcement->description;
  s.next_claim_s = now;
}

void XcoreLink::SetRunning(uint16_t id, Service& s, std::vector<LinkEvent>& events) {
  if (s.state == ServiceState::kRunning) return;
  s.state = ServiceState::kRunning;
  events.push_back(Event(LinkEvent::Kind::kRunning, id, ""));
}

void XcoreLink::Lose(uint16_t id, Service& s, const std::string& why, std::vector<LinkEvent>& events) {
  events.push_back(Event(LinkEvent::Kind::kLost, id, why));
  for (const auto& rpc : s.rpcs) {
    LinkEvent e = Event(LinkEvent::Kind::kRpcFailed, id, "service lost");
    e.function_id = rpc.function_id;
    events.push_back(e);
  }
  s.rpcs.clear();
  s.config_sent_s.reset();
  // The endpoint stays: a service that only missed messages takes the claim again, a restarted one announces anew.
  s.state = ServiceState::kClaiming;
}

void XcoreLink::OnDatagram(const uint8_t* data, size_t size, const Endpoint& from, double now,
                           std::vector<Output>& outputs, std::vector<LinkEvent>& events) {
  auto packet = DecodePacket(data, size);
  if (!packet) {
    dropped_++;
    return;
  }
  const Header& h = packet->header;
  auto it = services_.find(h.service_id);
  if (it == services_.end() || !(it->second.endpoint == from)) {
    dropped_++;
    return;
  }
  Service& s = it->second;
  if (s.state == ServiceState::kUnseen || s.state == ServiceState::kIncompatible) {
    dropped_++;
    return;
  }
  received_++;
  s.last_heard_s = now;
  uint16_t id = h.service_id;
  switch (h.type) {
    case MessageType::kClaim:
      if (h.arg1 != kClaimAck || s.state != ServiceState::kClaiming) break;
      if (NeedsConfig(id)) {
        s.state = ServiceState::kConfiguring;
        s.config_sent_s.reset();
      } else {
        SetRunning(id, s, events);
      }
      break;
    case MessageType::kConfigurationRequest:
      s.last_request_s = now;
      if (s.state == ServiceState::kRunning) {
        // A running service that asks again lost its configuration.
        events.push_back(Event(LinkEvent::Kind::kRestarted, id, "asks for its configuration again"));
        s.state = ServiceState::kConfiguring;
        s.config_sent_s.reset();
      } else if (s.state == ServiceState::kClaiming) {
        // The acknowledgement got lost.
        s.state = ServiceState::kConfiguring;
        s.config_sent_s.reset();
      }
      break;
    case MessageType::kData:
    case MessageType::kTransaction: {
      if (h.type == MessageType::kTransaction && h.arg1 != kTransactionData) break;
      if (s.state == ServiceState::kClaiming) break;
      std::vector<Section> sections;
      if (h.type == MessageType::kData) {
        sections.push_back(Section{h.arg2, packet->payload});
      } else if (auto decoded = DecodeSections(packet->payload)) {
        sections = std::move(*decoded);
      } else {
        dropped_++;
        break;
      }
      if (s.state == ServiceState::kConfiguring) SetRunning(id, s, events);
      for (auto& section : sections) {
        const Channel* out = Find(s.description->outputs, section.id);
        if (!out || !FitsType(out->type, section.value.size())) {
          dropped_++;
          continue;
        }
        outputs.push_back(Output{id, section.id, std::move(section.value), h.timestamp});
      }
      break;
    }
    case MessageType::kRpcResponse: {
      auto rpc = std::find_if(s.rpcs.begin(), s.rpcs.end(), [&](const Rpc& r) { return r.call_id == h.arg2; });
      if (rpc == s.rpcs.end()) break;
      LinkEvent e = Event(LinkEvent::Kind::kRpcResult, id, "");
      e.function_id = rpc->function_id;
      e.status = h.arg1;
      e.value = packet->payload;
      events.push_back(e);
      s.rpcs.erase(rpc);
      break;
    }
    default:
      break;
  }
}

Datagram XcoreLink::Make(const Service& s, uint16_t service_id, MessageType type, uint8_t arg1, uint16_t arg2,
                         const Bytes& payload) {
  Header h;
  h.type = type;
  h.service_id = service_id;
  h.arg1 = arg1;
  h.arg2 = arg2;
  h.sequence = ++sequence_;
  return Datagram{s.endpoint, EncodePacket(h, payload)};
}

std::vector<Datagram> XcoreLink::Tick(double now, std::vector<LinkEvent>& events) {
  std::vector<Datagram> out;
  for (auto& [id, s] : services_) {
    bool claimed = s.state == ServiceState::kConfiguring || s.state == ServiceState::kRunning;
    if (claimed && now - s.last_heard_s > config_.service_timeout_s) {
      Lose(id, s, "silent for " + std::to_string(now - s.last_heard_s).substr(0, 4) + " s", events);
      claimed = false;
    }
    if (s.state == ServiceState::kClaiming && now >= s.next_claim_s) {
      Writer w;
      w.U32(config_.host.ip);
      w.U16(config_.host.port);
      w.U32(config_.heartbeat_interval_us);
      out.push_back(Make(s, id, MessageType::kClaim, kClaimRequest, 0, w.Take()));
      s.next_claim_s = now + config_.claim_retry_s;
    }
    if (s.state == ServiceState::kConfiguring) {
      if (!s.config_sent_s || s.last_request_s > *s.config_sent_s) {
        out.push_back(Make(s, id, MessageType::kTransaction, kTransactionConfig, 0,
                           EncodeSections(config_.services.at(id))));
        s.config_sent_s = now;
      } else if (now - std::max(*s.config_sent_s, s.last_request_s) >= config_.config_settle_s) {
        SetRunning(id, s, events);
      }
    }
    if (s.state != ServiceState::kRunning) continue;
    for (auto rpc = s.rpcs.begin(); rpc != s.rpcs.end();) {
      if (now < rpc->next_s) {
        ++rpc;
      } else if (rpc->attempts >= config_.rpc_attempts) {
        LinkEvent e = Event(LinkEvent::Kind::kRpcFailed, id, "no answer");
        e.function_id = rpc->function_id;
        events.push_back(e);
        rpc = s.rpcs.erase(rpc);
      } else {
        out.push_back(Make(s, id, MessageType::kRpcRequest, rpc->function_id, rpc->call_id, {}));
        rpc->attempts++;
        rpc->next_s = now + config_.rpc_timeout_s;
        ++rpc;
      }
    }
  }
  return out;
}

std::optional<Datagram> XcoreLink::Input(uint16_t service_id, uint16_t channel_id, const Bytes& value) {
  auto it = services_.find(service_id);
  if (it == services_.end() || it->second.state != ServiceState::kRunning) return std::nullopt;
  const Channel* in = Find(it->second.description->inputs, channel_id);
  if (!in || in->type.base == ValueType::Base::kBlob) return std::nullopt;
  // Byte arrays (RTCM) may run longer than announced, up to one datagram: the firmware passes them on as a stream.
  bool byte_stream = in->type.base == ValueType::Base::kU8 && in->type.count && value.size() <= kMaxPayload;
  if (!FitsType(in->type, value.size()) && !byte_stream) return std::nullopt;
  return Make(it->second, service_id, MessageType::kData, 0, channel_id, value);
}

bool XcoreLink::CallRpc(uint16_t service_id, uint8_t function_id, double now) {
  auto it = services_.find(service_id);
  if (it == services_.end()) return false;
  ServiceState state = it->second.state;
  if (state != ServiceState::kRunning && state != ServiceState::kConfiguring) return false;
  it->second.rpcs.push_back(Rpc{function_id, next_call_id_++, 0, now});
  if (next_call_id_ == 0) next_call_id_ = 1;
  return true;
}

ServiceState XcoreLink::State(uint16_t service_id) const {
  auto it = services_.find(service_id);
  return it == services_.end() ? ServiceState::kUnseen : it->second.state;
}

bool XcoreLink::AllRunning() const {
  for (const auto& entry : services_) {
    if (entry.second.state != ServiceState::kRunning) return false;
  }
  return !services_.empty();
}

}  // namespace xcore
