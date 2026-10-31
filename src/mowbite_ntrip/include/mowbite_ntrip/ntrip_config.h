#ifndef MOWBITE_NTRIP_NTRIP_CONFIG_H
#define MOWBITE_NTRIP_NTRIP_CONFIG_H

#include <sys/types.h>

#include <cstddef>
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

// Longest host, mountpoint, user or password in bytes.
constexpr size_t kMaxValueLength = 255;

// What is wrong with a host, mountpoint (without its leading '/'), user or password given as key; empty if nothing.
// Host and mountpoint are printable ASCII without spaces. Never contains the value.
std::string CheckValue(const std::string& key, const std::string& value);

// The mountpoint as the file and ntrip.set store it: a leading '/' falls away.
std::string NormalMountpoint(const std::string& mountpoint);

// Reads the flat file: one "key: value" per line, the value optionally in single or double quotes ('' in single quotes
// stands for one, as in YAML), '#' starts a comment outside of quotes. Keys: host, port, mountpoint, user, password,
// version. Unknown keys and invalid values are errors; error names the line and a known key, never a value or an
// unknown key.
std::optional<NtripConfig> ParseNtripConfig(const std::string& text, std::string& error);

// The file text of config, which ParseNtripConfig reads back as the same config.
std::string FormatNtripConfig(const NtripConfig& config);

// The access file as found.
struct AccessFile {
  bool exists = false;
  // nullopt when missing, unreadable or invalid.
  std::optional<NtripConfig> config;
  // Why an existing file cannot be used, never with a value from it.
  std::string error;
  // Group or others may read it.
  bool readable_by_others = false;
};
AccessFile ReadAccessFile(const std::string& path);

// Writes config to path with mode 600 by ReplaceFile. error names the file and the cause, never a value.
bool WriteAccessFile(const std::string& path, const NtripConfig& config, std::string& error);
// Writes text to path: a new file beside it with mode from the start, synced, then renamed over the old one. error
// names the file and the cause.
bool ReplaceFile(const std::string& path, const std::string& text, mode_t mode, std::string& error);
// Removes the file; true when it is gone, also when there was none.
bool RemoveAccessFile(const std::string& path, std::string& error);

}  // namespace mowbite_ntrip

#endif  // MOWBITE_NTRIP_NTRIP_CONFIG_H
