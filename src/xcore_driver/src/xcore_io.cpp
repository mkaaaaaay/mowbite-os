#include "xcore_driver/xcore_io.h"

#include <dirent.h>
#include <unistd.h>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <string_view>

#include "xcore_driver/services.h"
#include "xcore_driver/socket_table.h"

namespace xcore {

namespace {

using mowbite_outbox::Format;

// Functions of the meta service.
constexpr uint8_t kGetFirmwareVersion = 0;
constexpr uint8_t kGetMajorVersion = 1;
// A firmware question without an answer is asked again after this long.
constexpr double kRpcRetryS = 5.0;
// Datagrams computed this long before they would go out are dropped: the thread stalled in between.
constexpr double kStaleSendS = 0.1;
// Without a socket the loop waits this long before it tries again.
constexpr double kClosedWaitS = 0.05;

const char* ServiceName(uint16_t id) {
  switch (id) {
    case kEmergencyService: return "emergency";
    case kDiffDriveService: return "diff drive";
    case kMowerService: return "mower";
    case kImuService: return "imu";
    case kPowerService: return "power";
    case kGpsService: return "gps";
    case kInputService: return "input";
    case kMetaService: return "meta";
    default: return "unknown";
  }
}

// The inode of a link target socket:[<inode>], nothing for any other target.
std::optional<uint64_t> SocketInode(std::string_view target) {
  constexpr std::string_view kPrefix = "socket:[";
  if (target.size() < kPrefix.size() + 2 || target.substr(0, kPrefix.size()) != kPrefix || target.back() != ']') {
    return std::nullopt;
  }
  const char* first = target.data() + kPrefix.size();
  const char* last = target.data() + target.size() - 1;
  uint64_t inode = 0;
  const auto [end, error] = std::from_chars(first, last, inode);
  if (error != std::errc() || end != last) return std::nullopt;
  return inode;
}

}  // namespace

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

int64_t WallNanoseconds() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}

std::string MajorVersionLine(uint16_t major, uint16_t expected) {
  if (major == expected) return Format("Firmware major version %u as expected", major);
  return Format("Firmware major version %u, expected %u: motion and blade stay blocked", major, expected);
}

uint64_t BlockOf(const ControlStatus& status) {
  return status.emergency_reason | (uint64_t{status.blocked} << 32) | (uint64_t{status.latched} << 33) |
         (uint64_t{status.held} << 34);
}

namespace {

std::optional<std::string> ForeignHostsOnce(uint32_t bind_ip, uint16_t group_port) {
  std::vector<ForeignSocket> found;
  std::vector<std::string> tables;
  for (const char* path : {"/proc/net/udp", "/proc/net/udp6"}) {
    std::ifstream file(path);
    if (!file && std::string(path) == "/proc/net/udp") return std::string("cannot read /proc/net/udp");
    std::stringstream text;
    text << file.rdbuf();
    tables.push_back(text.str());
  }
  // Own sockets after the tables: one opened in between is not in them.
  std::set<uint64_t> own;
  if (DIR* dir = opendir("/proc/self/fd")) {
    // readdir is safe for a stream no other thread uses; this one is local.
    while (const dirent* entry = readdir(dir)) {  // NOLINT(concurrency-mt-unsafe)
      char path[300];
      char target[64];
      std::snprintf(path, sizeof(path), "/proc/self/fd/%s", entry->d_name);
      const ssize_t n = readlink(path, target, sizeof(target) - 1);
      if (n <= 0) continue;
      if (const std::optional<uint64_t> inode = SocketInode(std::string_view(target, static_cast<size_t>(n)))) {
        own.insert(*inode);
      }
    }
    closedir(dir);
  } else {
    return std::string("cannot read /proc/self/fd");
  }
  for (const auto& table : tables) {
    auto sockets = ForeignSockets(table, bind_ip, group_port, own);
    found.insert(found.end(), sockets.begin(), sockets.end());
  }
  if (found.empty()) return std::nullopt;
  std::string text = "another program holds UDP sockets at " + Ipv4Text(bind_ip) + " or port " +
                     std::to_string(group_port) + ":";
  for (const auto& socket : found) text += " " + ForeignSocketText(socket);
  return text;
}

}  // namespace

// A hit counts only when a second reading finds it too: a socket this process closed between reading the tables and
// its own descriptors would otherwise look foreign.
std::optional<std::string> ForeignHosts(uint32_t bind_ip, uint16_t group_port) {
  if (!ForeignHostsOnce(bind_ip, group_port)) return std::nullopt;
  return ForeignHostsOnce(bind_ip, group_port);
}

XcoreIo::XcoreIo(const IoConfig& config, const ControlConfig& control, const InputMap& inputs)
    : config_(config), control_(control, inputs) {
}

XcoreIo::~XcoreIo() {
  Stop();
}

void XcoreIo::Start() {
  thread_ = std::thread([this] { Loop(); });
}

void XcoreIo::RequestStop() {
  running_ = false;
}

void XcoreIo::Stop() {
  RequestStop();
  if (thread_.joinable()) thread_.join();
}

void XcoreIo::OnCmdVel(double linear, double angular) {
  std::lock_guard<std::mutex> lock(mutex_);
  control_.OnCmdVel(linear, angular, MonotonicSeconds());
}

bool XcoreIo::LatchEmergency(uint32_t bits) {
  std::lock_guard<std::mutex> lock(mutex_);
  return control_.LatchEmergency(bits);
}

std::optional<ReleaseRefusal> XcoreIo::ReleaseEmergency(uint32_t latch_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  return control_.ReleaseEmergency(MonotonicSeconds(), latch_count);
}

void XcoreIo::OnSetupMotion(bool active) {
  std::lock_guard<std::mutex> lock(mutex_);
  control_.OnSetupMotion(active, MonotonicSeconds());
}

std::string XcoreIo::Silence(double seconds, bool blade) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = MonotonicSeconds();
  std::string refused = control_.Silence(seconds, now, blade);
  silent_ = control_.Silent(now);
  return refused;
}

std::string XcoreIo::Mute() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string refused = control_.EndRefusal();
  if (refused.empty()) muted_ = true;
  return refused;
}

BladeResult XcoreIo::SetBlade(const BladeRequest& request, uint32_t* emergency_reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  return control_.SetBlade(request, MonotonicSeconds(), emergency_reason);
}

bool XcoreIo::PushRtcm(const uint8_t* data, size_t size) {
  std::lock_guard<std::mutex> lock(mutex_);
  return rtcm_.Push(data, size);
}

uint64_t XcoreIo::rtcm_dropped_bytes() {
  std::lock_guard<std::mutex> lock(mutex_);
  return rtcm_.dropped_bytes();
}

void XcoreIo::SetHold(bool held) {
  std::lock_guard<std::mutex> lock(mutex_);
  control_.SetHold(held);
}

void XcoreIo::LatchForeignHost(const std::string& why) {
  std::lock_guard<std::mutex> lock(mutex_);
  control_.LatchForeignHost(why, MonotonicSeconds());
}

IoSnapshot XcoreIo::Snapshot() {
  IoSnapshot snapshot;
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot.status = control_.Status(MonotonicSeconds());
  if (link_) {
    snapshot.linked = true;
    snapshot.link_received = link_->received();
    snapshot.link_dropped = link_->dropped();
  }
  snapshot.firmware_version = firmware_version_;
  return snapshot;
}

void XcoreIo::CheckBlock() {
  uint64_t block;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block = BlockOf(control_.Status(MonotonicSeconds()));
  }
  if (block == requested_block_) return;
  requested_block_ = block;
  outbox_.status_due = true;
  outbox_.wakeup.Notify();
}

bool XcoreIo::Open(double now) {
  uint32_t interface_ip = 0;
  std::string error;
  if (!InterfaceAddress(config_.interface, interface_ip)) {
    if (open_problem_.Allow(now)) outbox_.log.Error("Interface " + config_.interface + " has no IPv4 address");
    return false;
  }
  uint32_t group_ip = 0;
  if (!ParseIpv4(config_.group, group_ip) || !announcements_.Bind(group_ip, config_.group_port, true, error) ||
      !announcements_.JoinGroup(config_.group, interface_ip, error)) {
    if (open_problem_.Allow(now)) {
      outbox_.log.Error(Format("Cannot listen to %s:%u on %s: %s", config_.group.c_str(), config_.group_port,
                               config_.interface.c_str(), error.c_str()));
    }
    announcements_.Close();
    return false;
  }
  if (!service_socket_.Bind(config_.bind_ip, 0, false, error)) {
    if (open_problem_.Allow(now)) {
      outbox_.log.Error(Format("Cannot bind to %s: %s", Ipv4Text(config_.bind_ip).c_str(), error.c_str()));
    }
    announcements_.Close();
    return false;
  }
  LinkConfig link;
  link.host = {config_.bind_ip, service_socket_.LocalPort()};
  link.services = config_.services;
  link.contest_losses = config_.contest_losses;
  link.contest_window_s = config_.contest_window_s;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    link_.emplace(link);
    for (const auto& entry : config_.services) control_.OnServiceRestart(entry.first, MonotonicSeconds());
  }
  open_ = true;
  outbox_.log.Info(Format("Listening for the board on %s:%u via %s (%s), services answer to %s:%u",
                          config_.group.c_str(), config_.group_port, config_.interface.c_str(),
                          Ipv4Text(interface_ip).c_str(), Ipv4Text(config_.bind_ip).c_str(),
                          service_socket_.LocalPort()));
  return true;
}

void XcoreIo::Close() {
  open_ = false;
  announcements_.Close();
  service_socket_.Close();
  // Without a socket no tick runs: every service counts as lost at once, nothing as ready, the firmware as unknown.
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = MonotonicSeconds();
  for (const auto& entry : config_.services) control_.OnServiceRestart(entry.first, now);
  control_.SetServicesReady(false, true);
  major_.reset();
  next_rpc_s_.reset();
}

void XcoreIo::Mark(const char* where) const {
  if (config_.watch.mark) config_.watch.mark(where);
}

void XcoreIo::Loop() {
  uint8_t buffer[kMaxDatagram + 1];
  // 0: no tick due yet, so the first one after a start or a closed socket counts as on time.
  double next_tick = 0;
  double next_open = 0;
  while (running_) {
    if (config_.watch.sign) config_.watch.sign();
    double now = MonotonicSeconds();
    if (!service_socket_.IsOpen()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        control_.Alive(now);
      }
      if (now >= next_open) {
        next_open = now + 1.0;
        // A second host at the board takes over whatever this driver claims: no start beside one. The board is
        // not claimed yet, so reading /proc here holds up nothing.
        Mark("foreign hosts");
        if (auto foreign = ForeignHosts(config_.bind_ip, config_.group_port)) {
          if (foreign_at_start_.Allow(now)) outbox_.log.Error("Board not claimed, checking again: " + *foreign);
        } else {
          Mark("open");
          Open(now);
        }
      }
      if (!service_socket_.IsOpen()) {
        CheckBlock();
        next_tick = 0;
        std::this_thread::sleep_for(std::chrono::duration<double>(kClosedWaitS));
        continue;
      }
    }
    Endpoint from;
    long n;
    Mark("receive");
    while ((n = announcements_.Receive(buffer, sizeof(buffer), from, 0)) > 0) {
      if (static_cast<size_t>(n) > kMaxDatagram) continue;
      std::vector<LinkEvent> events;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        Link().OnAnnouncement(buffer, static_cast<size_t>(n), from.ip, MonotonicSeconds(), events);
      }
      HandleEvents(events);
    }
    int wait_ms = std::clamp(static_cast<int>((next_tick - now) * 1000.0), 0, 5);
    n = service_socket_.Receive(buffer, sizeof(buffer), from, wait_ms);
    if (n < 0) {
      outbox_.log.Error("Receive error on the service socket, opening it again");
      Close();
      continue;
    }
    if (n > 0 && static_cast<size_t>(n) <= kMaxDatagram) HandleDatagram(buffer, static_cast<size_t>(n), from);

    now = MonotonicSeconds();
    if (now >= next_tick) {
      if (next_tick > 0 && now - next_tick > kLateTickS) outbox_.late_ticks.Add(now - next_tick);
      next_tick = now + kTickS;
      Mark("tick");
      Tick(now);
    }
    // So that the logic learns of a block before a blade request it refuses, not only at the next status.
    CheckBlock();
  }
  Shutdown();
}

void XcoreIo::HandleDatagram(const uint8_t* data, size_t size, const Endpoint& from) {
  std::vector<Output> outputs;
  std::vector<LinkEvent> events;
  std::vector<WheelTwist> twists;
  std::vector<ImuReading> imu;
  GpsReading gps;
  const int64_t stamp = WallNanoseconds();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = MonotonicSeconds();
    Link().OnDatagram(data, size, from, now, outputs, events);
    for (auto& out : outputs) {
      if (out.service_id == kImuService && out.channel_id == imu::kOutAxes) {
        if (auto axes = DecodeDoubles(out.value, 9)) {
          control_.OnImu((*axes)[0], (*axes)[1], (*axes)[2], now);
          ImuReading reading;
          reading.stamp_ns = stamp;
          std::copy_n(axes->begin(), 6, reading.axes.begin());
          imu.push_back(reading);
        }
        continue;
      }
      if (out.service_id == kGpsService) {
        if (config_.gnss) gps.outputs.push_back(std::move(out));
        continue;
      }
      if (auto twist = control_.OnOutput(out.service_id, out.channel_id, out.value, out.board_time_us, now)) {
        twists.push_back(*twist);
      }
    }
    if (!gps.outputs.empty() && last_rtcm_s_) gps.correction_age_s = now - *last_rtcm_s_;
  }
  HandleEvents(events);
  const bool any = !twists.empty() || !imu.empty() || !gps.outputs.empty();
  for (const auto& twist : twists) outbox_.twists.Push(TwistReading{stamp, twist});
  for (const auto& reading : imu) outbox_.imu.Push(reading);
  if (!gps.outputs.empty()) {
    gps.stamp_ns = stamp;
    outbox_.gps.Push(std::move(gps));
  }
  if (any) outbox_.wakeup.Notify();
}

void XcoreIo::Tick(double now) {
  std::vector<Datagram> datagrams;
  std::vector<LinkEvent> events;
  std::vector<std::string> errors;
  uint64_t rtcm_dropped = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (major_ == std::nullopt && next_rpc_s_ && now >= *next_rpc_s_) AskFirmware(now);
    // Silent or ended nothing goes out: the link would count configurations and requests as sent.
    if (!control_.Silent(now) && !muted_) datagrams = Link().Tick(now, events);
    control_.SetServicesReady(Link().AllRunning() && major_ == config_.expected_major,
                              !major_ || *major_ == config_.expected_major);
    for (const auto& value : control_.Tick(now)) {
      if (auto datagram = Link().Input(value.service_id, value.channel_id, value.value)) {
        datagrams.push_back(*datagram);
      }
    }
    silent_ = control_.Silent(now);
    // The queued RTCM to a running GPS service, dropped while it does not run or the driver is silent.
    if (rtcm_.size() > 0) {
      if (silent_ || Link().State(kGpsService) != ServiceState::kRunning) {
        rtcm_dropped = rtcm_.size();
        rtcm_.Clear();
      } else {
        for (const auto& piece : rtcm_.Take()) {
          if (auto datagram = Link().Input(kGpsService, gps::kInRtcm, piece)) {
            datagrams.push_back(*datagram);
            last_rtcm_s_ = now;
          }
        }
      }
    }
    errors = control_.TakeErrors();
  }
  if (rtcm_dropped > 0) outbox_.rtcm_dropped_closed.Add(rtcm_dropped);
  for (const auto& error : errors) outbox_.log.Error("Emergency latched: " + error);
  Send(datagrams, now);
  HandleEvents(events);
}

void XcoreIo::Send(const std::vector<Datagram>& datagrams, std::optional<double> computed) {
  if (silent_ || muted_) return;
  for (size_t i = 0; i < datagrams.size(); i++) {
    const auto& d = datagrams[i];
    const double age = computed ? MonotonicSeconds() - *computed : 0.0;
    if (age > kStaleSendS) {
      outbox_.log.Warn(Format("Dropped %zu datagrams computed %.2f s ago", datagrams.size() - i, age));
      return;
    }
    // An empty datagram is a packet that did not fit: it counts as failed, never as sent.
    if (d.bytes.empty() || !service_socket_.Send(d.bytes.data(), d.bytes.size(), d.to)) outbox_.send_failed.Add();
  }
}

XcoreLink& XcoreIo::Link() {
  // Open, which opens the service socket, sets link_ before it returns; nothing resets it.
  return *link_;  // NOLINT(bugprone-unchecked-optional-access)
}

// Called with mutex_ held.
void XcoreIo::AskFirmware(double now) {
  next_rpc_s_.reset();
  bool asked = Link().CallRpc(kMetaService, kGetMajorVersion, now);
  asked = Link().CallRpc(kMetaService, kGetFirmwareVersion, now) && asked;
  if (!asked) next_rpc_s_ = now + kRpcRetryS;
}

void XcoreIo::HandleEvents(const std::vector<LinkEvent>& events) {
  using Kind = LinkEvent::Kind;
  bool capabilities = false;
  std::vector<mowbite_outbox::LogLine> lines;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = MonotonicSeconds();
    for (const auto& e : events) {
      const char* name = ServiceName(e.service_id);
      switch (e.kind) {
        case Kind::kRunning:
          lines.push_back({mowbite_outbox::LogLine::Level::kInfo, Format("Service %s (%u) runs", name, e.service_id)});
          control_.OnServiceRunning(e.service_id, now);
          if (e.service_id == kMetaService) AskFirmware(now);
          break;
        case Kind::kLost:
        case Kind::kRestarted:
          lines.push_back({mowbite_outbox::LogLine::Level::kWarn,
                           Format("Service %s (%u) %s: %s", name, e.service_id,
                                  e.kind == Kind::kLost ? "lost" : "restarted", e.text.c_str())});
          control_.OnServiceRestart(e.service_id, now);
          if (e.service_id == kMetaService) {
            // The board may come back with other firmware.
            major_.reset();
            next_rpc_s_.reset();
          }
          break;
        case Kind::kIncompatible:
          lines.push_back({mowbite_outbox::LogLine::Level::kError,
                           Format("Service %s (%u) is not claimed: %s", name, e.service_id, e.text.c_str())});
          break;
        case Kind::kContested:
          // The driver keeps claiming: each claim stops the service at the board, and the latch reaches the board
          // whenever the emergency service is ours, where only a release clears it.
          control_.LatchForeignHost(std::string("service ") + name + " " + e.text, now);
          break;
        case Kind::kRpcResult:
          capabilities |= OnFirmwareAnswer(e, now);
          break;
        case Kind::kRpcFailed:
          lines.push_back({mowbite_outbox::LogLine::Level::kWarn,
                           Format("Firmware question %u to the board failed: %s", e.function_id, e.text.c_str())});
          if (!next_rpc_s_) next_rpc_s_ = now + kRpcRetryS;
          break;
      }
    }
  }
  for (auto& line : lines) outbox_.log.Add(line.level, std::move(line.text));
  if (capabilities) {
    outbox_.capabilities_due = true;
    outbox_.wakeup.Notify();
  }
}

// Called with mutex_ held; true when the capabilities changed. Its lines go to the outbox, which never waits.
bool XcoreIo::OnFirmwareAnswer(const LinkEvent& e, double now) {
  if (e.service_id != kMetaService) return false;
  if (e.status != 0) {
    outbox_.log.Warn(Format("Firmware question %u answered with status %u", e.function_id, e.status));
    if (!next_rpc_s_) next_rpc_s_ = now + kRpcRetryS;
    return false;
  }
  if (e.function_id == kGetMajorVersion) {
    if (e.value.size() != 2) {
      outbox_.log.Warn(Format("Major version answered with %zu bytes instead of 2: asked again", e.value.size()));
      if (!next_rpc_s_) next_rpc_s_ = now + kRpcRetryS;
      return false;
    }
    major_ = Reader(e.value).U16();
    if (*major_ == config_.expected_major) {
      outbox_.log.Info(MajorVersionLine(*major_, config_.expected_major));
    } else {
      outbox_.log.Error(MajorVersionLine(*major_, config_.expected_major));
    }
    return true;
  }
  if (e.function_id == kGetFirmwareVersion) {
    std::string version = DecodeText(e.value);
    if (version == firmware_version_) return false;
    firmware_version_ = version;
    outbox_.log.Info("xCore firmware " + version);
    return true;
  }
  return false;
}

void XcoreIo::Shutdown() {
  // Stopped during hw/silence the stops go out all the same.
  silent_ = false;
  std::vector<Datagram> datagrams;
  int repeats = 1;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    repeats = control_.repeat_count();
    if (link_ && service_socket_.IsOpen()) {
      for (const auto& value : control_.StopInputs()) {
        if (auto datagram = link_->Input(value.service_id, value.channel_id, value.value)) {
          datagrams.push_back(*datagram);
        }
      }
    }
  }
  // Over UDP each may get lost: the stops go out as often as the driver repeats them.
  std::vector<Datagram> repeated;
  for (int i = 0; i < repeats; i++) repeated.insert(repeated.end(), datagrams.begin(), datagrams.end());
  Send(repeated);
  Close();
}

}  // namespace xcore
