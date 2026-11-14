#include "xcore_driver/udp.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>

#include "mowbite_hold/file_io.h"

namespace xcore {

namespace {

sockaddr_in Address(uint32_t ip, uint16_t port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(ip);
  addr.sin_port = htons(port);
  return addr;
}

}  // namespace

bool ForbiddenPort(uint16_t port) {
  return port == 8007 || port == 10000 || (port >= 65102 && port <= 65104);
}

bool ParseIpv4(const std::string& text, uint32_t& ip) {
  in_addr addr{};
  if (inet_pton(AF_INET, text.c_str(), &addr) != 1) return false;
  ip = ntohl(addr.s_addr);
  return true;
}

std::string Ipv4Text(uint32_t ip) {
  in_addr addr{};
  addr.s_addr = htonl(ip);
  char text[INET_ADDRSTRLEN] = {};
  inet_ntop(AF_INET, &addr, text, sizeof(text));
  return text;
}

bool InterfaceAddress(const std::string& name, uint32_t& ip) {
  ifaddrs* list = nullptr;
  if (::getifaddrs(&list) != 0) return false;
  bool found = false;
  for (ifaddrs* entry = list; entry; entry = entry->ifa_next) {
    if (!entry->ifa_addr || entry->ifa_addr->sa_family != AF_INET || name != entry->ifa_name) continue;
    ip = ntohl(reinterpret_cast<sockaddr_in*>(entry->ifa_addr)->sin_addr.s_addr);
    found = true;
    break;
  }
  ::freeifaddrs(list);
  return found;
}

UdpSocket::~UdpSocket() {
  Close();
}

void UdpSocket::Close() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

bool UdpSocket::Bind(uint32_t ip, uint16_t port, bool reuse, std::string& error) {
  Close();
  fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd_ < 0) {
    error = mowbite_hold::ErrorText(errno);
    return false;
  }
  int one = 1;
  if (reuse && ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) != 0) {
    error = mowbite_hold::ErrorText(errno);
    Close();
    return false;
  }
  sockaddr_in addr = Address(ip, port);
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    error = mowbite_hold::ErrorText(errno);
    Close();
    return false;
  }
  return true;
}

bool UdpSocket::JoinGroup(const std::string& group, uint32_t interface_ip, std::string& error) {
  ip_mreq request{};
  if (inet_pton(AF_INET, group.c_str(), &request.imr_multiaddr) != 1) {
    error = "bad group " + group;
    return false;
  }
  request.imr_interface.s_addr = htonl(interface_ip);
  if (::setsockopt(fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &request, sizeof(request)) != 0) {
    error = mowbite_hold::ErrorText(errno);
    return false;
  }
  return true;
}

long UdpSocket::Receive(uint8_t* buffer, size_t size, Endpoint& from, int timeout_ms) {
  if (fd_ < 0) return -1;
  pollfd p{fd_, POLLIN, 0};
  int ready = ::poll(&p, 1, timeout_ms);
  if (ready < 0) return errno == EINTR ? 0 : -1;
  if (ready == 0) return 0;
  sockaddr_in addr{};
  socklen_t length = sizeof(addr);
  ssize_t n = ::recvfrom(fd_, buffer, size, MSG_TRUNC, reinterpret_cast<sockaddr*>(&addr), &length);
  if (n < 0) return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) ? 0 : -1;
  from.ip = ntohl(addr.sin_addr.s_addr);
  from.port = ntohs(addr.sin_port);
  // A datagram longer than the buffer was cut; the caller sees its full size and drops it.
  return static_cast<long>(n);
}

bool UdpSocket::Send(const uint8_t* data, size_t size, const Endpoint& to) {
  if (fd_ < 0 || ForbiddenPort(to.port)) return false;
  sockaddr_in addr = Address(to.ip, to.port);
  ssize_t n = ::sendto(fd_, data, size, 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  return n == static_cast<ssize_t>(size);
}

uint16_t UdpSocket::LocalPort() const {
  sockaddr_in addr{};
  socklen_t length = sizeof(addr);
  if (fd_ < 0 || ::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &length) != 0) return 0;
  return ntohs(addr.sin_port);
}

}  // namespace xcore
