#ifndef MOWBITE_NTRIP_NTRIP_ACCESS_H
#define MOWBITE_NTRIP_NTRIP_ACCESS_H

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_ntrip/ntrip_config.h"

namespace mowbite_ntrip {

// The access over RPC: ntrip.get shows it, ntrip.set and ntrip.delete change it. User and password go in, never out.
constexpr char kGetMethod[] = "ntrip.get";
constexpr char kSetMethod[] = "ntrip.set";
constexpr char kDeleteMethod[] = "ntrip.delete";

// The methods to offer: ntrip.set and ntrip.delete only where writing is allowed (access_over_rpc).
std::vector<std::string> AccessMethods(bool write);

// The answer of ntrip.get and ntrip.set: host, port, mountpoint and version, of user and password only whether they
// are set; for an existing file that cannot be used, why.
nlohmann::ordered_json AccessSummary(const AccessFile& file);

struct AccessChange {
  // nullopt when refused.
  std::optional<NtripConfig> config;
  // Why: the field and the problem, never a value.
  std::string error;
};

// The params of ntrip.set over the stored access, all or nothing. host and mountpoint are needed, port and version
// default to 2101 and 2. A user or password left out stays as stored, but only for the same host and port: a stored
// login never goes to another caster.
AccessChange ChangeAccess(const nlohmann::ordered_json& params, const std::optional<NtripConfig>& stored);

}  // namespace mowbite_ntrip

#endif  // MOWBITE_NTRIP_NTRIP_ACCESS_H
