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
    : config_(std::move(config)), now_(std::move(now)), session_counter_(std::random_device{}()) {}

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
  DropClient();
  CloseFd(listen_fd_);
  port_ = 0;
}

void RtspServer::SetAvailable(const ParameterSets& sets) {
  sets_ = sets;
  available_ = true;
}

void RtspServer::SetUnavailable() {
  available_ = false;
  DropClient();
}

void RtspServer::DropClient() {
  CloseFd(client_fd_);
  input_.clear();
  session_.clear();
  playing_ = false;
}

void RtspServer::Accept() {
  const int fd = accept4(listen_fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
  if (fd < 0) return;
  if (client_fd_ >= 0) {
    // One client at a time; a second one is turned away.
    close(fd);
    return;
  }
  const int on = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
  client_fd_ = fd;
  last_activity_ = now_();
}

void RtspServer::Poll(int timeout_ms) {
  if (listen_fd_ < 0) return;
  pollfd fds[2] = {{listen_fd_, POLLIN, 0}, {client_fd_, POLLIN, 0}};
  const int ready = poll(fds, client_fd_ >= 0 ? 2 : 1, timeout_ms);
  if (ready > 0) {
    if (client_fd_ >= 0 && fds[1].revents != 0) Read();
    if ((fds[0].revents & POLLIN) != 0) Accept();
  }
  if (client_fd_ >= 0 && now_() - last_activity_ > config_.idle_timeout_s) DropClient();
}

void RtspServer::Read() {
  char chunk[2048];
  for (;;) {
    const ssize_t n = recv(client_fd_, chunk, sizeof(chunk), 0);
    if (n > 0) {
      input_.append(chunk, static_cast<size_t>(n));
      last_activity_ = now_();
      if (!Consume()) return;
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
    DropClient();
    return;
  }
}

bool RtspServer::Consume() {
  while (!input_.empty()) {
    if (input_[0] == '$') {
      // Interleaved frame from the client (RTCP): marker, channel, length, data.
      if (input_.size() < 4) return true;
      const size_t length = static_cast<uint8_t>(input_[2]) << 8 | static_cast<uint8_t>(input_[3]);
      if (input_.size() < 4 + length) return true;
      input_.erase(0, 4 + length);
      continue;
    }
    const size_t end = input_.find("\r\n\r\n");
    if (end == std::string::npos) {
      if (input_.size() > kMaxRequestBytes) {
        DropClient();
        return false;
      }
      return true;
    }
    Request request;
    size_t line_end = input_.find("\r\n");
    {
      const std::string line = input_.substr(0, line_end);
      const size_t a = line.find(' ');
      const size_t b = line.rfind(' ');
      if (a != std::string::npos && b > a && line.compare(b + 1, std::string::npos, "RTSP/1.0") == 0) {
        request.method = line.substr(0, a);
        request.url = line.substr(a + 1, b - a - 1);
      }
    }
    while (line_end < end) {
      const size_t start = line_end + 2;
      line_end = input_.find("\r\n", start);
      const std::string line = input_.substr(start, line_end - start);
      const size_t colon = line.find(':');
      if (colon != std::string::npos) request.headers[Lower(line.substr(0, colon))] = Trim(line.substr(colon + 1));
    }
    size_t body = 0;
    const auto length = request.headers.find("content-length");
    if (length != request.headers.end()) body = std::strtoul(length->second.c_str(), nullptr, 10);
    if (body > kMaxRequestBytes) {
      DropClient();
      return false;
    }
    if (input_.size() < end + 4 + body) return true;
    input_.erase(0, end + 4 + body);
    Handle(request);
    if (client_fd_ < 0) return false;
  }
  return true;
}

bool RtspServer::OwnPath(const std::string& url) const {
  size_t start = 0;
  const size_t scheme = url.find("://");
  if (scheme != std::string::npos) {
    start = url.find('/', scheme + 3);
    if (start == std::string::npos) return false;
  }
  std::string path = url.substr(start, url.find('?', start) - start);
  path.erase(0, path.find_first_not_of('/'));
  return path == config_.path || path.rfind(config_.path + "/", 0) == 0;
}

void RtspServer::Reply(const Request& request, int code, const std::string& headers, const std::string& body) {
  std::string out = "RTSP/1.0 " + std::to_string(code) + " " + Phrase(code) + "\r\n";
  const auto cseq = request.headers.find("cseq");
  if (cseq != request.headers.end()) out += "CSeq: " + cseq->second + "\r\n";
  out += headers;
  if (!body.empty()) out += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  out += "\r\n" + body;
  SendAll(reinterpret_cast<const uint8_t*>(out.data()), out.size(), SendDeadline());
}

void RtspServer::Handle(const Request& request) {
  if (request.method.empty() || request.headers.count("cseq") == 0) {
    Reply(request, 400);
    return;
  }
  const std::string session_header = session_.empty() ? "" : "Session: " + session_ + "\r\n";
  if (request.method == "OPTIONS") {
    Reply(request, 200, std::string("Public: ") + kMethods + "\r\n" + session_header);
    return;
  }
  if (request.method == "GET_PARAMETER") {
    Reply(request, 200, session_header);
    return;
  }
  if (request.method == "TEARDOWN") {
    Reply(request, 200, session_header);
    DropClient();
    return;
  }
  const bool describe = request.method == "DESCRIBE";
  const bool setup = request.method == "SETUP";
  if (!describe && !setup && request.method != "PLAY") {
    Reply(request, 405, std::string("Allow: ") + kMethods + "\r\n");
    return;
  }
  if (!OwnPath(request.url)) {
    Reply(request, 404);
    return;
  }
  if (!available_) {
    Reply(request, 503);
    return;
  }
  if (describe) {
    std::string base = request.url.substr(0, request.url.find('?'));
    if (base.empty() || base.back() != '/') base += '/';
    Reply(request, 200, "Content-Base: " + base + "\r\nContent-Type: application/sdp\r\n",
          StreamSdp(sets_, config_.path));
    return;
  }
  if (setup) {
    const auto transport = request.headers.find("transport");
    uint8_t channel = 0;
    if (playing_) {
      Reply(request, 455);
      return;
    }
    if (transport == request.headers.end() || !InterleavedChannel(transport->second, channel)) {
      Reply(request, 461);
      return;
    }
    rtp_channel_ = channel;
    if (session_.empty()) session_ = std::to_string(++session_counter_);
    Reply(request, 200,
          "Transport: RTP/AVP/TCP;unicast;interleaved=" + std::to_string(channel) + "-" + std::to_string(channel + 1) +
              "\r\nSession: " + session_ + ";timeout=" + std::to_string(static_cast<int>(config_.idle_timeout_s)) +
              "\r\n");
    return;
  }
  if (session_.empty()) {
    Reply(request, 455);
    return;
  }
  const auto session = request.headers.find("session");
  if (session == request.headers.end() || session->second.substr(0, session->second.find(';')) != session_) {
    Reply(request, 454);
    return;
  }
  Reply(request, 200, session_header + "Range: npt=0.000-\r\n");
  playing_ = client_fd_ >= 0;
}

bool RtspServer::SendRtp(const std::vector<std::vector<uint8_t>>& packets) {
  if (!playing_) return false;
  const auto give_up = SendDeadline();
  for (const std::vector<uint8_t>& packet : packets) {
    if (packet.size() > 0xFFFF) {
      DropClient();
      return false;
    }
    frame_.resize(4 + packet.size());
    frame_[0] = '$';
    frame_[1] = rtp_channel_;
    frame_[2] = static_cast<uint8_t>(packet.size() >> 8);
    frame_[3] = static_cast<uint8_t>(packet.size());
    std::memcpy(frame_.data() + 4, packet.data(), packet.size());
    if (!SendAll(frame_.data(), frame_.size(), give_up)) return false;
  }
  return true;
}

std::chrono::steady_clock::time_point RtspServer::SendDeadline() const {
  return std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.send_timeout_ms);
}

bool RtspServer::SendAll(const uint8_t* data, size_t size, std::chrono::steady_clock::time_point give_up) {
  if (client_fd_ < 0) return false;
  size_t sent = 0;
  while (sent < size) {
    const ssize_t n = send(client_fd_, data + sent, size - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(give_up - std::chrono::steady_clock::now());
      pollfd pfd{client_fd_, POLLOUT, 0};
      if (left.count() > 0 && poll(&pfd, 1, static_cast<int>(left.count())) >= 0) continue;
    }
    // A message cut in the middle cannot be resumed; the client has to come again.
    DropClient();
    return false;
  }
  return true;
}

}  // namespace tron_vision
