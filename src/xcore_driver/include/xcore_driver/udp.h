#ifndef XCORE_DRIVER_UDP_H
#define XCORE_DRIVER_UDP_H

#include <cstddef>
#include <cstdint>
#include <string>

namespace xcore {

constexpr char kAnnouncementGroup[] = "233.255.255.0";
constexpr uint16_t kAnnouncementPort = 4242;

struct Endpoint {
  // Host byte order.
  uint32_t ip = 0;
  uint16_t port = 0;
  bool operator==(const Endpoint& other) const {
    return ip == other.ip && port == other.port;
  }
};

// Ports of the board that are never sent to: 8007 restarts it into the bootloader, 10000 and 65102-65104 pass raw
// bytes to the GPS and the motor controllers, the latter ignoring every duty of the firmware while connected.
bool ForbiddenPort(uint16_t port);

// "172.16.78.1" to host byte order; false for anything else.
bool ParseIpv4(const std::string& text, uint32_t& ip);
std::string Ipv4Text(uint32_t ip);
// The IPv4 address of a network interface such as "eth0"; false if it has none.
bool InterfaceAddress(const std::string& name, uint32_t& ip);

// A non-blocking UDP socket. Sockets close themselves.
class UdpSocket {
 public:
  UdpSocket() = default;
  ~UdpSocket();
  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  // Bound to ip:port (0 picks a free port); the socket may share the port with other listeners when reuse is set.
  bool Bind(uint32_t ip, uint16_t port, bool reuse, std::string& error);
  // Receives the group's datagrams on the interface with this address.
  bool JoinGroup(const std::string& group, uint32_t interface_ip, std::string& error);
  // Waits up to timeout_ms for a datagram; returns its size, 0 on timeout, -1 on error.
  long Receive(uint8_t* buffer, size_t size, Endpoint& from, int timeout_ms);
  // Refuses a forbidden port.
  bool Send(const uint8_t* data, size_t size, const Endpoint& to);
  // The port the socket is bound to.
  uint16_t LocalPort() const;
  bool IsOpen() const {
    return fd_ >= 0;
  }
  void Close();

 private:
  int fd_ = -1;
};

}  // namespace xcore

#endif  // XCORE_DRIVER_UDP_H
