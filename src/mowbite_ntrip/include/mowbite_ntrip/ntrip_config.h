#ifndef MOWBITE_NTRIP_NTRIP_CONFIG_H
#define MOWBITE_NTRIP_NTRIP_CONFIG_H

#include <optional>
#include <string>

namespace mowbite_ntrip {

// Access to a caster, from <data_dir>/ntrip.yaml. The user fills it in on the mower; it never goes into logs or status.
struct NtripConfig {
  std::string host;
  int port = 2101;
  std::string mountpoint;
  std::string user;
  std::string password;
  // 2: NTRIP 2.0 (HTTP/1.1, chunked); 1: NTRIP 1.0 (ICY 200 OK).
  int version = 2;

  bool operator==(const NtripConfig& other) const {
    return host == other.host && port == other.port && mountpoint == other.mountpoint && user == other.user &&
           password == other.password && version == other.version;
  }
  bool operator!=(const NtripConfig& other) const {
    return !(*this == other);
  }
};

// Reads the flat file: one "key: value" per line, the value optionally in single or double quotes, '#' starts a comment
// outside of quotes. Keys: host, port, mountpoint, user, password, version. Unknown keys and invalid values are errors;
// error names the line and the key, never a value.
std::optional<NtripConfig> ParseNtripConfig(const std::string& text, std::string& error);

}  // namespace mowbite_ntrip

#endif  // MOWBITE_NTRIP_NTRIP_CONFIG_H
