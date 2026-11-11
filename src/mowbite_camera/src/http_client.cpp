#include "mowbite_camera/http_client.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>

namespace mowbite_camera {

namespace {

using Clock = std::chrono::steady_clock;

// An answer longer than this is not taken.
constexpr size_t kMaxReply = 1u << 20;
constexpr char kHeadEnd[] = "\r\n\r\n";

// Waits until fd is ready for events; false when the deadline came first.
bool Ready(int fd, short events, Clock::time_point deadline) {
  while (true) {
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
    if (left <= 0) return false;
    pollfd waiting{fd, events, 0};
    const int n = ::poll(&waiting, 1, static_cast<int>(left));
    if (n > 0) return true;
    if (n < 0 && errno != EINTR) return false;
  }
}

// The whole exchange on a connected or connecting socket; what went wrong, empty when the answer is in *in.
std::string Exchange(int fd, const sockaddr_in& where, const std::string& out, Clock::time_point deadline,
                     std::string* in) {
  if (::connect(fd, reinterpret_cast<const sockaddr*>(&where), sizeof where) != 0) {
    if (errno != EINPROGRESS) return std::string("connect: ") + std::strerror(errno);
    int failed = 0;
    socklen_t size = sizeof failed;
    if (!Ready(fd, POLLOUT, deadline)) return "no connection in time";
    if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &failed, &size) != 0 || failed != 0) {
      return std::string("connect: ") + std::strerror(failed != 0 ? failed : errno);
    }
  }
  size_t sent = 0;
  while (sent < out.size()) {
    const ssize_t n = ::send(fd, out.data() + sent, out.size() - sent, MSG_NOSIGNAL);
    if (n > 0) {
      sent += static_cast<size_t>(n);
    } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      return std::string("send: ") + std::strerror(errno);
    } else if (!Ready(fd, POLLOUT, deadline)) {
      return "request not taken in time";
    }
  }
  char buffer[4096];
  while (true) {
    const ssize_t n = ::recv(fd, buffer, sizeof buffer, 0);
    if (n == 0) return "";
    if (n > 0) {
      in->append(buffer, static_cast<size_t>(n));
      if (in->size() > kMaxReply) return "answer too long";
    } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      return std::string("recv: ") + std::strerror(errno);
    } else if (!Ready(fd, POLLIN, deadline)) {
      return "no answer in time";
    }
  }
}

}  // namespace

std::string Base64(const std::string& bytes) {
  static constexpr char kDigits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string text;
  for (size_t at = 0; at < bytes.size(); at += 3) {
    const size_t have = std::min<size_t>(3, bytes.size() - at);
    uint32_t group = 0;
    for (size_t i = 0; i < 3; ++i) group = group << 8 | (i < have ? static_cast<unsigned char>(bytes[at + i]) : 0);
    for (size_t i = 0; i < 4; ++i) text.push_back(i <= have ? kDigits[group >> (18 - 6 * i) & 63] : '=');
  }
  return text;
}

HttpReply LocalRequest(const LocalServer& server, const std::string& method, const std::string& target,
                       double timeout_s) {
  HttpReply reply;
  const Clock::time_point deadline =
      Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_s));
  const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    reply.error = std::string("socket: ") + std::strerror(errno);
    return reply;
  }
  sockaddr_in where{};
  where.sin_family = AF_INET;
  where.sin_port = htons(server.port);
  where.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  const std::string out = method + " " + target + " HTTP/1.0\r\nHost: 127.0.0.1\r\nAuthorization: Basic " +
                          Base64(server.user + ":" + server.password) + "\r\nContent-Length: 0\r\n\r\n";
  std::string in;
  reply.error = Exchange(fd, where, out, deadline, &in);
  ::close(fd);
  if (!reply.error.empty()) return reply;

  // "HTTP/1.x NNN", then the fields up to an empty line, then the body.
  const size_t head = in.find(kHeadEnd);
  const bool status_line = in.compare(0, 7, "HTTP/1.") == 0 && in.size() >= 12 && in[8] == ' ' &&
                           in.find_first_not_of("0123456789", 9) == 12;
  if (head == std::string::npos || !status_line) {
    reply.error = "not an answer of HTTP";
    return reply;
  }
  reply.status = std::stoi(in.substr(9, 3));
  reply.body = in.substr(head + sizeof(kHeadEnd) - 1);
  return reply;
}

}  // namespace mowbite_camera
