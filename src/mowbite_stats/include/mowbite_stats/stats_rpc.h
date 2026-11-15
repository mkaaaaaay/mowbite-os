#ifndef MOWBITE_STATS_STATS_RPC_H
#define MOWBITE_STATS_STATS_RPC_H

#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mowbite_stats/counter.h"
#include "mowbite_stats/sensor_history.h"

namespace mowbite_stats {

constexpr int kInvalidParams = -32602;

struct RpcError {
  int code = kInvalidParams;
  std::string message;
  json data;  // {field, reason}
};

struct RpcOutcome {
  json result;
  std::optional<RpcError> error;
  bool parts_changed = false;  // stats/parts.json to be written
};

// An answer of invalid params: {field, reason}, message "field: message".
RpcOutcome BadParam(const std::string& field, const std::string& reason, const std::string& message);
// params an object of only these keys (null for none when none is required).
std::optional<RpcOutcome> CheckKeys(const json& params, const std::set<std::string>& keys, bool none_ok);
// The optional client of params matches kClientPattern.
std::optional<RpcOutcome> CheckClient(const json& params);

constexpr char kHistoryMethod[] = "sensors.history";

// All methods of mowbite_stats: stats.*, sensors.history, journal.status and incidents.* (Incidents::Methods).
const std::vector<std::string>& StatsMethods();

// The answer to one of StatsMethods but sensors.history; params as sent (null without any), now in unix s.
RpcOutcome HandleStats(Counter& counter, const std::string& method, const json& params, double now);
// The answer to sensors.history.
RpcOutcome HandleHistory(const SensorHistory& history, const json& params, double now);

}  // namespace mowbite_stats

#endif  // MOWBITE_STATS_STATS_RPC_H
