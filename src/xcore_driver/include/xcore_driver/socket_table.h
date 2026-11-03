#ifndef XCORE_DRIVER_SOCKET_TABLE_H
#define XCORE_DRIVER_SOCKET_TABLE_H

#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace xcore {

// A UDP socket of another program that could talk to the board as a second host.
struct ForeignSocket {
  uint32_t ip = 0;
  uint16_t port = 0;
  uint64_t inode = 0;
  bool ipv6 = false;
};

std::string ForeignSocketText(const ForeignSocket& socket);

// The sockets in a table of the kernel (text of /proc/net/udp or /proc/net/udp6) that sit on bind_ip at a port of
// 1024 or above, or on announcement_port at any address, leaving out the inodes in own. System services on bind_ip
// (DHCP on 67, DNS on 53) are no hosts of the board; an IPv6 socket counts with the wildcard or an IPv4 mapped address.
std::vector<ForeignSocket> ForeignSockets(const std::string& table, uint32_t bind_ip, uint16_t announcement_port,
                                          const std::set<uint64_t>& own);

}  // namespace xcore

#endif  // XCORE_DRIVER_SOCKET_TABLE_H
