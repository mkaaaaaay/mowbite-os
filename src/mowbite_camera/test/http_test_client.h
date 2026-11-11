#ifndef MOWBITE_CAMERA_TEST_HTTP_TEST_CLIENT_H
#define MOWBITE_CAMERA_TEST_HTTP_TEST_CLIENT_H

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <string>

namespace mowbite_camera {

// A client on 127.0.0.1 that never waits: the test lets it and the server work in turns.
class TestClient {
 public:
  explicit TestClient(uint16_t port) : fd_(::socket(AF_INET, SOCK_STREAM, 0)) {
    sockaddr_in where{};
    where.sin_family = AF_INET;
    where.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &where.sin_addr);
    connected_ = ::connect(fd_, reinterpret_cast<const sockaddr*>(&where), sizeof where) == 0;
    ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL) | O_NONBLOCK);
  }
  ~TestClient() {
    Close();
  }
  TestClient(const TestClient&) = delete;
  TestClient& operator=(const TestClient&) = delete;

  bool connected() const {
    return connected_;
  }
  void Send(const std::string& text) {
    if (::send(fd_, text.data(), text.size(), MSG_NOSIGNAL) != static_cast<ssize_t>(text.size())) closed_ = true;
  }
  // Takes what has come and notes whether the server ended the connection.
  void Take() {
    char buffer[65536];
    while (fd_ >= 0 && !closed_) {
      const ssize_t n = ::recv(fd_, buffer, sizeof buffer, 0);
      if (n > 0) {
        in_.append(buffer, static_cast<size_t>(n));
      } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
        closed_ = true;
      } else {
        break;
      }
    }
  }
  void Close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }
  const std::string& in() const {
    return in_;
  }
  bool closed() const {
    return closed_;
  }

 private:
  int fd_;
  bool connected_ = false;
  bool closed_ = false;
  std::string in_;
};

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_TEST_HTTP_TEST_CLIENT_H
