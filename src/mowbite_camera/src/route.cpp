#include "mowbite_camera/route.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace mowbite_camera {

std::string DefaultRouteAddress() {
  // An address for documentation (RFC 5737): only a default route leads there. Connecting a UDP socket makes the
  // kernel choose the route and the source address without sending a packet.
  sockaddr_in far{};
  far.sin_family = AF_INET;
  far.sin_port = htons(9);
  ::inet_pton(AF_INET, "192.0.2.1", &far.sin_addr);
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) return "";
  sockaddr_in own{};
  socklen_t size = sizeof own;
  char text[INET_ADDRSTRLEN] = "";
  if (::connect(fd, reinterpret_cast<const sockaddr*>(&far), sizeof far) == 0 &&
      ::getsockname(fd, reinterpret_cast<sockaddr*>(&own), &size) == 0 &&
      ntohl(own.sin_addr.s_addr) != INADDR_ANY && ntohl(own.sin_addr.s_addr) >> 24 != IN_LOOPBACKNET) {
    ::inet_ntop(AF_INET, &own.sin_addr, text, sizeof text);
  }
  ::close(fd);
  return text;
}

}  // namespace mowbite_camera
