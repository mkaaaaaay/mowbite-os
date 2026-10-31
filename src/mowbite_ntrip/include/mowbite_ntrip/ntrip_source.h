#ifndef MOWBITE_NTRIP_NTRIP_SOURCE_H
#define MOWBITE_NTRIP_NTRIP_SOURCE_H

#include <optional>
#include <string>

#include "mowbite_ntrip/ntrip_config.h"

namespace mowbite_ntrip {

// Where the corrections come from: caster and mountpoint, never the login. Another source may have another reference
// and offset the positions from the map (G24), so the client compares each new one with the source used last.
struct CorrectionSource {
  std::string host;
  int port = 2101;
  std::string mountpoint;

  bool operator==(const CorrectionSource& other) const {
    return host == other.host && port == other.port && mountpoint == other.mountpoint;
  }
  bool operator!=(const CorrectionSource& other) const {
    return !(*this == other);
  }
};

CorrectionSource SourceOf(const NtripConfig& config);

// The same caster and mountpoint: the host in any case, the mountpoint without a leading '/', the port as it is. The
// version and the login do not count, nor does a name lookup: an address of the same caster counts as another one.
bool SameSource(const CorrectionSource& a, const CorrectionSource& b);

// The source used last, in <data_dir>/ntrip_source.json: {"host", "port", "mountpoint"}.
struct SourceFile {
  bool exists = false;
  // nullopt when missing, unreadable or invalid.
  std::optional<CorrectionSource> source;
  // Why an existing file cannot be used.
  std::string error;
};
SourceFile ReadSourceFile(const std::string& path);

// Saves source by ReplaceFile with mode 644, only if it reads back as the same. error names the file and the cause.
bool WriteSourceFile(const std::string& path, const CorrectionSource& source, std::string& error);

}  // namespace mowbite_ntrip

#endif  // MOWBITE_NTRIP_NTRIP_SOURCE_H
