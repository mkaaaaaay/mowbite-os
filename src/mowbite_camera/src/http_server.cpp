#include "mowbite_camera/http_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>

#include "mowbite_camera/error_text.h"

namespace mowbite_camera {

namespace {

constexpr size_t kReadChunk = 4096;
constexpr size_t kMaxMethod = 16;
constexpr size_t kMaxLengthDigits = 9;
// How long a listener rests after accept failed for want of a resource, in s.
constexpr double kAcceptRest = 1.0;
constexpr char kHeadEnd[] = "\r\n\r\n";
constexpr char kStreamHead[] =
    "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nCache-Control: no-store\r\n";

const char* StatusText(int status) {
  switch (status) {
    case 200: return "OK";
    case 400: return "Bad Request";
    case 401: return "Unauthorized";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 413: return "Content Too Large";
    case 431: return "Request Header Fields Too Large";
    case 503: return "Service Unavailable";
    default: return "Error";
  }
}

std::string Tail(bool any_origin) {
  return std::string(any_origin ? "Access-Control-Allow-Origin: *\r\n" : "") + "Connection: close\r\n\r\n";
}

std::string Lower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
  return text;
}

bool IsRetry(int error) {
  return error == EAGAIN || error == EWOULDBLOCK || error == EINTR;
}

}  // namespace

std::string Trim(const std::string& text) {
  const size_t begin = text.find_first_not_of(" \t");
  if (begin == std::string::npos) return "";
  return text.substr(begin, text.find_last_not_of(" \t") - begin + 1);
}

bool ParseHead(const std::string& head, HttpRequest* request, size_t* content_length) {
  *content_length = 0;
  size_t line_end = head.find("\r\n");
  if (line_end == std::string::npos) return false;
  const std::string line = head.substr(0, line_end);
  const size_t first = line.find(' ');
  const size_t second = first == std::string::npos ? first : line.find(' ', first + 1);
  if (second == std::string::npos || line.find(' ', second + 1) != std::string::npos) return false;
  const std::string method = line.substr(0, first);
  const std::string target = line.substr(first + 1, second - first - 1);
  const std::string version = line.substr(second + 1);
  if (method.empty() || method.size() > kMaxMethod ||
      !std::all_of(method.begin(), method.end(), [](unsigned char c) { return std::isupper(c); })) {
    return false;
  }
  if (target.empty() || target[0] != '/' ||
      !std::all_of(target.begin(), target.end(), [](unsigned char c) { return std::isgraph(c); })) {
    return false;
  }
  if (version != "HTTP/1.1" && version != "HTTP/1.0") return false;

  bool have_length = false;
  for (size_t at = line_end + 2; at < head.size(); at = line_end + 2) {
    line_end = head.find("\r\n", at);
    if (line_end == std::string::npos) return false;
    if (line_end == at) break;
    const std::string field = head.substr(at, line_end - at);
    const size_t colon = field.find(':');
    if (colon == std::string::npos || colon == 0) return false;
    const std::string name = Lower(field.substr(0, colon));
    const std::string value = Trim(field.substr(colon + 1));
    if (name == "transfer-encoding") return false;
    if (name != "content-length") continue;
    if (value.empty() || value.size() > kMaxLengthDigits ||
        !std::all_of(value.begin(), value.end(), [](unsigned char c) { return std::isdigit(c); })) {
      return false;
    }
    const size_t length = std::stoul(value);
    if (have_length && length != *content_length) return false;
    have_length = true;
    *content_length = length;
  }
  const size_t mark = target.find('?');
  request->method = method;
  request->path = target.substr(0, mark);
  request->query = mark == std::string::npos ? "" : target.substr(mark + 1);
  return true;
}

std::string QueryValue(const std::string& query, const std::string& key) {
  size_t at = 0;
  while (at <= query.size()) {
    size_t end = query.find('&', at);
    if (end == std::string::npos) end = query.size();
    const size_t value = at + key.size() + 1;
    if (query.compare(at, key.size() + 1, key + "=") == 0) return query.substr(value, end - value);
    at = end + 1;
  }
  return "";
}

HttpServer::HttpServer(Handler handler, Gone gone, std::function<double()> now)
    : handler_(std::move(handler)), gone_(std::move(gone)), now_(std::move(now)),
      wake_fd_(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)) {}

HttpServer::~HttpServer() {
  for (const Connection& connection : connections_) ::close(connection.fd);
  for (const Listener& listener : listeners_) ::close(listener.fd);
  if (wake_fd_ >= 0) ::close(wake_fd_);
}

int HttpServer::Listen(const std::string& address, uint16_t port, const ListenerRules& rules, std::string* error) {
  sockaddr_in where{};
  where.sin_family = AF_INET;
  where.sin_port = htons(port);
  if (::inet_pton(AF_INET, address.c_str(), &where.sin_addr) != 1) {
    *error = address + " is not an IPv4 address";
    return -1;
  }
  const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    *error = std::string("socket: ") + ErrorText(errno);
    return -1;
  }
  const int on = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof on);
  socklen_t size = sizeof where;
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&where), size) != 0 || ::listen(fd, 16) != 0 ||
      ::getsockname(fd, reinterpret_cast<sockaddr*>(&where), &size) != 0) {
    *error = address + ":" + std::to_string(port) + ": " + ErrorText(errno);
    ::close(fd);
    return -1;
  }
  Listener listener;
  listener.fd = fd;
  listener.port = ntohs(where.sin_port);
  listener.rules = rules;
  listeners_.push_back(listener);
  return static_cast<int>(listeners_.size()) - 1;
}

uint16_t HttpServer::Port(int listener) const {
  return listeners_.at(static_cast<size_t>(listener)).port;
}

double HttpServer::Deadline(const Connection& connection) const {
  const ListenerRules& rules = listeners_[static_cast<size_t>(connection.listener)].rules;
  if (connection.phase == Phase::kRequest) return connection.since + rules.request_seconds;
  if (connection.sent < connection.out.size()) return connection.since + rules.send_seconds;
  return connection.last_picture + rules.picture_seconds;
}

void HttpServer::Poll(int timeout_ms) {
  const double now = now_();
  double wait = std::max(0, timeout_ms) / 1000.0;
  std::vector<pollfd> fds;
  fds.push_back({wake_fd_, POLLIN, 0});
  for (const Listener& listener : listeners_) {
    // A negative descriptor is skipped by poll.
    fds.push_back({now < listener.retry ? -1 : listener.fd, POLLIN, 0});
    if (now < listener.retry) wait = std::min(wait, listener.retry - now);
  }
  const size_t known = connections_.size();
  for (const Connection& connection : connections_) {
    short events = connection.phase == Phase::kAnswer ? 0 : POLLIN;
    if (connection.sent < connection.out.size()) events |= POLLOUT;
    fds.push_back({connection.fd, events, 0});
    wait = std::min(wait, Deadline(connection) - now);
  }
  ::poll(fds.data(), fds.size(), static_cast<int>(std::ceil(std::max(0.0, wait) * 1000.0)));

  const size_t first = 1 + listeners_.size();
  for (size_t i = 0; i < known; ++i) {
    Connection& connection = connections_[i];
    const short events = fds[first + i].revents;
    if (events & POLLIN) Read(connection);
    if ((events & POLLOUT) && !connection.closing) Write(connection);
    if ((events & (POLLERR | POLLHUP | POLLNVAL)) && !(events & POLLIN)) connection.closing = true;
  }
  for (size_t i = 0; i < listeners_.size(); ++i) {
    if (fds[1 + i].revents & POLLIN) Accept(listeners_[i], static_cast<int>(i));
  }
  if (fds[0].revents & POLLIN) {
    uint64_t count = 0;
    while (::read(wake_fd_, &count, sizeof count) > 0) {
    }
  }
  RunTasks();
  const double after = now_();
  for (Connection& connection : connections_) {
    if (after >= Deadline(connection)) connection.closing = true;
  }
  Sweep();
}

void HttpServer::Accept(Listener& listener, int number) {
  while (true) {
    const int fd = ::accept4(listener.fd, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (fd < 0) {
      if (errno == EINTR || errno == ECONNABORTED) continue;
      if (!IsRetry(errno)) listener.retry = now_() + kAcceptRest;
      return;
    }
    if (listener.pending >= listener.rules.max_pending) {
      ::close(fd);
      continue;
    }
    const int on = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof on);
    Connection connection;
    connection.id = next_id_++;
    connection.fd = fd;
    connection.listener = number;
    connection.since = now_();
    connections_.push_back(std::move(connection));
    listener.pending++;
  }
}

void HttpServer::LeavePending(Connection& connection) {
  if (connection.phase == Phase::kRequest) listeners_[static_cast<size_t>(connection.listener)].pending--;
}

void HttpServer::Reply(Connection& connection, int status, const std::string& body) {
  const ListenerRules& rules = listeners_[static_cast<size_t>(connection.listener)].rules;
  LeavePending(connection);
  connection.phase = Phase::kAnswer;
  connection.since = now_();
  connection.sent = 0;
  connection.out = "HTTP/1.1 " + std::to_string(status) + " " + StatusText(status) +
                   "\r\nContent-Type: text/plain\r\nContent-Length: " + std::to_string(body.size()) +
                   "\r\nCache-Control: no-store\r\n" + Tail(rules.any_origin) + body;
}

void HttpServer::Read(Connection& connection) {
  char buffer[kReadChunk];
  const ssize_t n = ::recv(connection.fd, buffer, sizeof buffer, 0);
  if (n < 0 && IsRetry(errno)) return;
  // A viewer has nothing to say after its request.
  if (n <= 0 || connection.phase != Phase::kRequest) {
    connection.closing = true;
    return;
  }
  const ListenerRules& rules = listeners_[static_cast<size_t>(connection.listener)].rules;
  connection.in.append(buffer, static_cast<size_t>(n));
  if (!connection.parsed) {
    const size_t end = connection.in.find(kHeadEnd);
    const size_t head = end == std::string::npos ? connection.in.size() : end + sizeof(kHeadEnd) - 1;
    if (head > rules.max_head) return Reply(connection, 431, "request too long\n");
    if (end == std::string::npos) return;
    if (!ParseHead(connection.in.substr(0, head), &connection.request, &connection.content_length)) {
      return Reply(connection, 400, "bad request\n");
    }
    if (connection.content_length > rules.max_body) return Reply(connection, 413, "body too long\n");
    connection.parsed = true;
    connection.body_at = head;
  }
  if (connection.in.size() < connection.body_at + connection.content_length) return;

  connection.request.listener = connection.listener;
  connection.request.connection = connection.id;
  connection.request.body = connection.in.substr(connection.body_at, connection.content_length);
  connection.in.clear();
  const HttpAnswer answer = handler_(connection.request);
  if (answer.status != 200 || answer.stream.empty()) return Reply(connection, answer.status, answer.body);
  LeavePending(connection);
  connection.phase = Phase::kStream;
  connection.camera = answer.stream;
  connection.since = connection.last_picture = now_();
  connection.sent = 0;
  connection.out = kStreamHead + Tail(rules.any_origin);
}

void HttpServer::Write(Connection& connection) {
  const ssize_t n = ::send(connection.fd, connection.out.data() + connection.sent,
                           connection.out.size() - connection.sent, MSG_NOSIGNAL);
  if (n < 0) {
    if (!IsRetry(errno)) connection.closing = true;
    return;
  }
  connection.sent += static_cast<size_t>(n);
  if (connection.sent < connection.out.size()) return;
  if (connection.phase == Phase::kAnswer) {
    connection.closing = true;
    return;
  }
  connection.out.clear();
  connection.sent = 0;
  if (!connection.next.empty()) {
    const std::string jpeg = std::move(connection.next);
    connection.next.clear();
    StartPicture(connection, jpeg);
  }
}

void HttpServer::StartPicture(Connection& connection, const std::string& jpeg) {
  connection.since = now_();
  connection.sent = 0;
  connection.out = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(jpeg.size()) + "\r\n\r\n";
  connection.out += jpeg;
  connection.out += "\r\n";
}

void HttpServer::Push(const std::string& camera, const std::string& jpeg) {
  if (jpeg.empty()) return;
  for (Connection& connection : connections_) {
    if (connection.phase != Phase::kStream || connection.closing || connection.camera != camera) continue;
    connection.last_picture = now_();
    if (connection.out.empty()) {
      StartPicture(connection, jpeg);
    } else {
      connection.next = jpeg;
    }
  }
}

void HttpServer::Close(uint64_t connection) {
  for (Connection& have : connections_) {
    if (have.id == connection) have.closing = true;
  }
}

void HttpServer::Post(std::function<void()> task) {
  {
    const std::lock_guard<std::mutex> lock(tasks_mutex_);
    tasks_.push_back(std::move(task));
  }
  const uint64_t one = 1;
  if (::write(wake_fd_, &one, sizeof one) < 0) {
    // The counter is full: Poll is awake anyway.
  }
}

void HttpServer::RunTasks() {
  std::vector<std::function<void()>> tasks;
  {
    const std::lock_guard<std::mutex> lock(tasks_mutex_);
    tasks.swap(tasks_);
  }
  for (const auto& task : tasks) task();
}

void HttpServer::Sweep() {
  std::vector<uint64_t> gone;
  for (Connection& connection : connections_) {
    if (!connection.closing) continue;
    ::close(connection.fd);
    LeavePending(connection);
    if (connection.phase == Phase::kStream) gone.push_back(connection.id);
  }
  connections_.erase(std::remove_if(connections_.begin(), connections_.end(),
                                    [](const Connection& connection) { return connection.closing; }),
                     connections_.end());
  for (const uint64_t id : gone) gone_(id);
}

}  // namespace mowbite_camera
