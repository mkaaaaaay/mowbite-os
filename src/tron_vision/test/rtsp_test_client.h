#ifndef TRON_VISION_RTSP_TEST_CLIENT_H
#define TRON_VISION_RTSP_TEST_CLIENT_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstdint>
#include <string>

namespace tron_vision::test {

// Blocking TCP client; the server runs in the same thread through Poll().
class Client {
 public:
  explicit Client(uint16_t port) {
    fd_ = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    connected_ = connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
    timeval wait{0, 200000};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &wait, sizeof(wait));
  }
  ~Client() {
    close(fd_);
  }
  // Connected, and nothing failed to go out.
  bool Connected() const {
    return connected_ && sent_;
  }
  void Send(const std::string& data) const {
    sent_ = send(fd_, data.data(), data.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(data.size()) && sent_;
  }
  // What arrives within the receive timeout; "<closed>" once the server has closed.
  std::string Receive() const {
    char buf[4096];
    const ssize_t n = recv(fd_, buf, sizeof(buf), 0);
    if (n == 0) return "<closed>";
    return n > 0 ? std::string(buf, static_cast<size_t>(n)) : "";
  }

 private:
  int fd_ = -1;
  bool connected_ = false;
  mutable bool sent_ = true;
};

// The session id of a SETUP answer; empty without one.
inline std::string SessionOf(const std::string& setup) {
  const size_t at = setup.find("Session: ");
  if (at == std::string::npos) return "";
  return setup.substr(at + 9, setup.find(';', at) - at - 9);
}

}  // namespace tron_vision::test

#endif  // TRON_VISION_RTSP_TEST_CLIENT_H
