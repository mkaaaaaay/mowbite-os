#ifndef MOWBITE_CAMERA_TEST_LOCAL_TEST_SERVER_H
#define MOWBITE_CAMERA_TEST_LOCAL_TEST_SERVER_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace mowbite_camera {

// A server on 127.0.0.1 for one connection: it reads the head of a request and sends `answer`, or with an empty
// answer keeps the connection open in silence until it is destroyed.
class OneShotServer {
 public:
  explicit OneShotServer(std::string answer) : fd_(::socket(AF_INET, SOCK_STREAM, 0)) {
    sockaddr_in where{};
    where.sin_family = AF_INET;
    where.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t size = sizeof where;
    ::bind(fd_, reinterpret_cast<const sockaddr*>(&where), size);
    ::listen(fd_, 1);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&where), &size);
    port_ = ntohs(where.sin_port);
    thread_ = std::thread([this, answer = std::move(answer)] { Serve(answer); });
  }
  ~OneShotServer() {
    stop_ = true;
    // Ends an accept nobody connected to.
    ::shutdown(fd_, SHUT_RDWR);
    thread_.join();
    ::close(fd_);
  }

  uint16_t port() const {
    return port_;
  }
  // The head of the request, complete once the client has its answer.
  const std::string& request() const {
    return request_;
  }

 private:
  void Serve(const std::string& answer) {
    const int client = ::accept(fd_, nullptr, nullptr);
    if (client < 0) return;
    char buffer[1024];
    while (request_.find("\r\n\r\n") == std::string::npos) {
      const ssize_t n = ::recv(client, buffer, sizeof buffer, 0);
      if (n <= 0) break;
      request_.append(buffer, static_cast<size_t>(n));
    }
    if (answer.empty()) {
      while (!stop_) ::usleep(5000);
    } else if (::send(client, answer.data(), answer.size(), MSG_NOSIGNAL) < 0) {
      request_ += " (answer not sent)";
    }
    ::close(client);
  }

  int fd_;
  uint16_t port_ = 0;
  std::atomic<bool> stop_{false};
  std::string request_;
  std::thread thread_;
};

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_TEST_LOCAL_TEST_SERVER_H
