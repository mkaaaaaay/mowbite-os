#include "tron_vision/rtsp_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <random>
#include <utility>

#include "tron_vision/errno_text.h"

namespace tron_vision {

namespace {

constexpr size_t kMaxRequestBytes = 8192;
constexpr char kMethods[] = "OPTIONS, DESCRIBE, SETUP, PLAY, GET_PARAMETER, TEARDOWN";

const char* Phrase(int code) {
  switch (code) {
    case 200: return "OK";
    case 400: return "Bad Request";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 454: return "Session Not Found";
    case 455: return "Method Not Valid in This State";
    case 461: return "Unsupported Transport";
    case 503: return "Service Unavailable";
    default: return "Error";
  }
}

std::string Lower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
  return text;
}

std::string Trim(const std::string& text) {
  const size_t first = text.find_first_not_of(" \t");
  if (first == std::string::npos) return "";
  return text.substr(first, text.find_last_not_of(" \t") - first + 1);
}

// The channel pair of "interleaved=a-b" in a Transport header; false if it asks for another transport.
bool InterleavedChannel(const std::string& transport, uint8_t& rtp_channel) {
  if (transport.find("RTP/AVP/TCP") == std::string::npos) return false;
  rtp_channel = 0;
  const size_t at = transport.find("interleaved=");
  if (at == std::string::npos) return true;
  char* end = nullptr;
  const long channel = std::strtol(transport.c_str() + at + 12, &end, 10);
  if (end == transport.c_str() + at + 12 || channel < 0 || channel > 254) return false;
  rtp_channel = static_cast<uint8_t>(channel);
  return true;
}

void CloseFd(int& fd) {
  if (fd >= 0) close(fd);
  fd = -1;
}

}  // namespace

std::string StreamSdp(const ParameterSets& sets, const std::string& name) {
  return "v=0\r\n"
         "o=- 0 0 IN IP4 127.0.0.1\r\n"
         "s=" + name + "\r\n"
         "c=IN IP4 0.0.0.0\r\n"
         "t=0 0\r\n"
         "m=video 0 RTP/AVP " + std::to_string(RtpPacketizer::kPayloadType) + "\r\n"
         "a=control:trackID=0\r\n"
         "a=rtpmap:" + std::to_string(RtpPacketizer::kPayloadType) + " H264/90000\r\n"
         "a=fmtp:" + std::to_string(RtpPacketizer::kPayloadType) + " packetization-mode=1;profile-level-id=" +
         ProfileLevelId(sets.sps) + ";sprop-parameter-sets=" + Base64(sets.sps.data(), sets.sps.size()) + "," +
         Base64(sets.pps.data(), sets.pps.size()) + "\r\n";
}

RtspServer::RtspServer(RtspConfig config, std::function<double()> now)
    : config_(std::move(config)),
      now_(std::move(now)),
      mounts_(config_.paths.size()),
      connections_(config_.paths.size()),
      session_counter_(std::random_device{}()) {}

RtspServer::~RtspServer() {
  Close();
}

bool RtspServer::Listen(std::string& error) {
  Close();
  listen_fd_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (listen_fd_ < 0) {
    error = ErrnoText("socket");
    return false;
  }
  const int on = 1;
  setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(config_.port);
  socklen_t len = sizeof(addr);
  if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || listen(listen_fd_, 2) != 0 ||
      getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
    error = ErrnoText("listen on 127.0.0.1:" + std::to_string(config_.port));
    CloseFd(listen_fd_);
    return false;
  }
  port_ = ntohs(addr.sin_port);
  return true;
}

void RtspServer::Close() {
  for (Connection& c : connections_) Drop(c);
  CloseFd(listen_fd_);
  port_ = 0;
}

void RtspServer::SetAvailable(const ParameterSets& sets, size_t path) {
  if (path >= mounts_.size()) return;
  mounts_[path].sets = sets;
  mounts_[path].available = true;
}

void RtspServer::SetUnavailable(size_t path) {
  if (path >= mounts_.size()) return;
  mounts_[path].available = false;
  DropClient(path);
}

bool RtspServer::Playing(size_t path) const {
  const Connection* c = ClientOf(path);
  return c != nullptr && c->playing;
}

void RtspServer::DropClient(size_t path) {
  Connection* c = ClientOf(path);
  if (c != nullptr) Drop(*c);
}

RtspServer::Connection* RtspServer::ClientOf(size_t path) {
  for (Connection& c : connections_) {
    if (c.fd >= 0 && c.path == path) return &c;
  }
  return nullptr;
}

const RtspServer::Connection* RtspServer::ClientOf(size_t path) const {
  for (const Connection& c : connections_) {
    if (c.fd >= 0 && c.path == path) return &c;
  }
  return nullptr;
}

void RtspServer::Drop(Connection& c) {
  CloseFd(c.fd);
  c = Connection{};
}

void RtspServer::Accept() {
  const int fd = accept4(listen_fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
  if (fd < 0) return;
  const auto free_slot =
      std::find_if(connections_.begin(), connections_.end(), [](const Connection& c) { return c.fd < 0; });
  if (free_slot == connections_.end()) {
    // As many connections as paths; one more is turned away.
    close(fd);
    return;
  }
  const int on = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
  free_slot->fd = fd;
  free_slot->last_activity = now_();
}

void RtspServer::Poll(int timeout_ms) {
  if (listen_fd_ < 0) return;
  std::vector<pollfd> fds{{listen_fd_, POLLIN, 0}};
  for (const Connection& c : connections_) fds.push_back({c.fd, POLLIN, 0});
  // A negative fd is skipped by poll().
  const int ready = poll(fds.data(), fds.size(), timeout_ms);
  if (ready > 0) {
    for (size_t i = 0; i < connections_.size(); i++) {
      Connection& c = connections_[i];
      if (c.fd >= 0 && c.fd == fds[i + 1].fd && fds[i + 1].revents != 0) Read(c);
    }
    if ((fds[0].revents & POLLIN) != 0) Accept();
  }
  const double now = now_();
  for (Connection& c : connections_) {
    if (c.fd >= 0 && now - c.last_activity > config_.idle_timeout_s) Drop(c);
  }
}

void RtspServer::Read(Connection& c) {
  char chunk[2048];
  for (;;) {
    const ssize_t n = recv(c.fd, chunk, sizeof(chunk), 0);
    if (n > 0) {
      c.input.append(chunk, static_cast<size_t>(n));
      c.last_activity = now_();
      if (!Consume(c)) return;
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
    Drop(c);
    return;
  }
}

bool RtspServer::Consume(Connection& c) {
  while (!c.input.empty()) {
    std::string& input = c.input;
    if (input[0] == '$') {
      // Interleaved frame from the client (RTCP): marker, channel, length, data.
      if (input.size() < 4) return true;
      const size_t length = static_cast<uint8_t>(input[2]) << 8 | static_cast<uint8_t>(input[3]);
      if (input.size() < 4 + length) return true;
      input.erase(0, 4 + length);
      continue;
    }
    const size_t end = input.find("\r\n\r\n");
    if (end == std::string::npos) {
      if (input.size() > kMaxRequestBytes) {
        Drop(c);
        return false;
      }
      return true;
    }
    Request request;
    size_t line_end = input.find("\r\n");
    {
      const std::string line = input.substr(0, line_end);
      const size_t a = line.find(' ');
      const size_t b = line.rfind(' ');
      if (a != std::string::npos && b > a && line.compare(b + 1, std::string::npos, "RTSP/1.0") == 0) {
        request.method = line.substr(0, a);
        request.url = line.substr(a + 1, b - a - 1);
      }
    }
    while (line_end < end) {
      const size_t start = line_end + 2;
      line_end = input.find("\r\n", start);
      const std::string line = input.substr(start, line_end - start);
      const size_t colon = line.find(':');
      if (colon != std::string::npos) request.headers[Lower(line.substr(0, colon))] = Trim(line.substr(colon + 1));
    }
    size_t body = 0;
    const auto length = request.headers.find("content-length");
    if (length != request.headers.end()) body = std::strtoul(length->second.c_str(), nullptr, 10);
    if (body > kMaxRequestBytes) {
      Drop(c);
      return false;
    }
    if (input.size() < end + 4 + body) return true;
    input.erase(0, end + 4 + body);
    Handle(c, request);
    if (c.fd < 0) return false;
  }
  return true;
}

size_t RtspServer::PathOf(const std::string& url) const {
  size_t start = 0;
  const size_t scheme = url.find("://");
  if (scheme != std::string::npos) {
    start = url.find('/', scheme + 3);
    if (start == std::string::npos) return kNoPath;
  }
  std::string path = url.substr(start, url.find('?', start) - start);
  path.erase(0, path.find_first_not_of('/'));
  for (size_t i = 0; i < config_.paths.size(); i++) {
    const std::string& own = config_.paths[i];
    if (path == own || path.rfind(own + "/", 0) == 0) return i;
  }
  return kNoPath;
}

void RtspServer::Reply(Connection& c, const Request& request, int code, const std::string& headers,
                       const std::string& body) {
  std::string out = "RTSP/1.0 " + std::to_string(code) + " " + Phrase(code) + "\r\n";
  const auto cseq = request.headers.find("cseq");
  if (cseq != request.headers.end()) out += "CSeq: " + cseq->second + "\r\n";
  out += headers;
  if (!body.empty()) out += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  out += "\r\n" + body;
  SendAll(c, reinterpret_cast<const uint8_t*>(out.data()), out.size(), SendDeadline());
}

void RtspServer::Handle(Connection& c, const Request& request) {
  if (request.method.empty() || request.headers.count("cseq") == 0) {
    Reply(c, request, 400);
    return;
  }
  const std::string session_header = c.session.empty() ? "" : "Session: " + c.session + "\r\n";
  if (request.method == "OPTIONS") {
    Reply(c, request, 200, std::string("Public: ") + kMethods + "\r\n" + session_header);
    return;
  }
  if (request.method == "GET_PARAMETER") {
    Reply(c, request, 200, session_header);
    return;
  }
  if (request.method == "TEARDOWN") {
    Reply(c, request, 200, session_header);
    Drop(c);
    return;
  }
  const bool describe = request.method == "DESCRIBE";
  const bool setup = request.method == "SETUP";
  if (!describe && !setup && request.method != "PLAY") {
    Reply(c, request, 405, std::string("Allow: ") + kMethods + "\r\n");
    return;
  }
  const size_t path = PathOf(request.url);
  if (path == kNoPath) {
    Reply(c, request, 404);
    return;
  }
  if (c.path != path) {
    if (c.path != kNoPath) {
      // A connection serves the one path it began with.
      Reply(c, request, 455);
      return;
    }
    if (ClientOf(path) != nullptr) {
      // The path has its client; this connection would only hold the slot of another path.
      Reply(c, request, 503);
      Drop(c);
      return;
    }
    c.path = path;
  }
  const Mount& mount = mounts_[path];
  if (!mount.available) {
    Reply(c, request, 503);
    return;
  }
  if (describe) {
    std::string base = request.url.substr(0, request.url.find('?'));
    if (base.empty() || base.back() != '/') base += '/';
    Reply(c, request, 200, "Content-Base: " + base + "\r\nContent-Type: application/sdp\r\n",
          StreamSdp(mount.sets, config_.paths[path]));
    return;
  }
  if (setup) {
    const auto transport = request.headers.find("transport");
    uint8_t channel = 0;
    if (c.playing) {
      Reply(c, request, 455);
      return;
    }
    if (transport == request.headers.end() || !InterleavedChannel(transport->second, channel)) {
      Reply(c, request, 461);
      return;
    }
    c.rtp_channel = channel;
    if (c.session.empty()) c.session = std::to_string(++session_counter_);
    Reply(c, request, 200,
          "Transport: RTP/AVP/TCP;unicast;interleaved=" + std::to_string(channel) + "-" + std::to_string(channel + 1) +
              "\r\nSession: " + c.session + ";timeout=" + std::to_string(static_cast<int>(config_.idle_timeout_s)) +
              "\r\n");
    return;
  }
  if (c.session.empty()) {
    Reply(c, request, 455);
    return;
  }
  const auto session = request.headers.find("session");
  if (session == request.headers.end() || session->second.substr(0, session->second.find(';')) != c.session) {
    Reply(c, request, 454);
    return;
  }
  Reply(c, request, 200, session_header + "Range: npt=0.000-\r\n");
  c.playing = c.fd >= 0;
}

bool RtspServer::SendRtp(const std::vector<std::vector<uint8_t>>& packets, size_t path) {
  Connection* c = ClientOf(path);
  if (c == nullptr || !c->playing) return false;
  const auto give_up = SendDeadline();
  for (const std::vector<uint8_t>& packet : packets) {
    if (packet.size() > 0xFFFF) {
      Drop(*c);
      return false;
    }
    frame_.resize(4 + packet.size());
    frame_[0] = '$';
    frame_[1] = c->rtp_channel;
    frame_[2] = static_cast<uint8_t>(packet.size() >> 8);
    frame_[3] = static_cast<uint8_t>(packet.size());
    std::memcpy(frame_.data() + 4, packet.data(), packet.size());
    if (!SendAll(*c, frame_.data(), frame_.size(), give_up)) return false;
  }
  return true;
}

std::chrono::steady_clock::time_point RtspServer::SendDeadline() const {
  return std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.send_timeout_ms);
}

bool RtspServer::SendAll(Connection& c, const uint8_t* data, size_t size,
                         std::chrono::steady_clock::time_point give_up) {
  if (c.fd < 0) return false;
  size_t sent = 0;
  while (sent < size) {
    const ssize_t n = send(c.fd, data + sent, size - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      const auto left =
          std::chrono::duration_cast<std::chrono::milliseconds>(give_up - std::chrono::steady_clock::now());
      pollfd pfd{c.fd, POLLOUT, 0};
      if (left.count() > 0 && poll(&pfd, 1, static_cast<int>(left.count())) >= 0) continue;
    }
    // A message cut in the middle cannot be resumed; the client has to come again.
    Drop(c);
    return false;
  }
  return true;
}

}  // namespace tron_vision
