#include "xcore_driver/socket_table.h"

#include <arpa/inet.h>

#include <cstdlib>
#include <sstream>

#include "xcore_driver/udp.h"

namespace xcore {

namespace {

constexpr uint16_t kFirstUnprivilegedPort = 1024;

bool Hex32(const std::string& text, uint32_t& value) {
  if (text.size() != 8) return false;
  char* end = nullptr;
  unsigned long v = std::strtoul(text.c_str(), &end, 16);
  if (end != text.c_str() + text.size()) return false;
  value = static_cast<uint32_t>(v);
  return true;
}

// "0100007F:1F90" or 32 hex digits and a port; the kernel prints each 32-bit word of the address as it lies in memory.
bool ParseLocal(const std::string& field, ForeignSocket& socket, bool& any) {
  auto colon = field.find(':');
  if (colon == std::string::npos) return false;
  const std::string address = field.substr(0, colon);
  char* end = nullptr;
  unsigned long port = std::strtoul(field.c_str() + colon + 1, &end, 16);
  if (*end != '\0' || port > 0xFFFF) return false;
  socket.port = static_cast<uint16_t>(port);
  if (address.size() == 8) {
    uint32_t word = 0;
    if (!Hex32(address, word)) return false;
    socket.ip = ntohl(word);
    any = socket.ip == 0;
    return true;
  }
  if (address.size() != 32) return false;
  uint32_t words[4];
  for (size_t i = 0; i < 4; i++) {
    if (!Hex32(address.substr(i * 8, 8), words[i])) return false;
  }
  socket.ipv6 = true;
  any = words[0] == 0 && words[1] == 0 && words[2] == 0 && words[3] == 0;
  const bool mapped = words[0] == 0 && words[1] == 0 && ntohl(words[2]) == 0x0000FFFFu;
  socket.ip = mapped ? ntohl(words[3]) : 0;
  return true;
}

}  // namespace

std::string ForeignSocketText(const ForeignSocket& socket) {
  std::ostringstream text;
  text << (socket.ipv6 ? "udp6 " : "udp ") << Ipv4Text(socket.ip) << ":" << socket.port << " (inode " << socket.inode
       << ")";
  return text.str();
}

std::vector<ForeignSocket> ForeignSockets(const std::string& table, uint32_t bind_ip, uint16_t announcement_port,
                                          const std::set<uint64_t>& own) {
  std::vector<ForeignSocket> out;
  std::istringstream lines(table);
  std::string line;
  while (std::getline(lines, line)) {
    std::istringstream fields(line);
    std::vector<std::string> f;
    std::string field;
    while (fields >> field && f.size() < 10) f.push_back(field);
    // sl, local, remote, state, queues, timer, retransmits, uid, timeout, inode; the header line does not parse.
    if (f.size() < 10) continue;
    ForeignSocket socket;
    bool any = false;
    if (!ParseLocal(f[1], socket, any)) continue;
    char* end = nullptr;
    socket.inode = std::strtoull(f[9].c_str(), &end, 10);
    if (*end != '\0' || own.count(socket.inode)) continue;
    const bool on_bind_ip = !any && socket.ip == bind_ip && socket.port >= kFirstUnprivilegedPort;
    if (on_bind_ip || socket.port == announcement_port) out.push_back(socket);
  }
  return out;
}

}  // namespace xcore
