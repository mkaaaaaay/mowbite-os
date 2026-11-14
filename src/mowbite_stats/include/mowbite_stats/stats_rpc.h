#ifndef MOWBITE_STATS_STATS_RPC_H
#define MOWBITE_STATS_STATS_RPC_H

#include <optional>
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

constexpr char kHistoryMethod[] = "sensors.history";

// stats.get, stats.part_done, stats.part_due and sensors.history.
const std::vector<std::string>& StatsMethods();

// The answer to one of StatsMethods but sensors.history; params as sent (null without any), now in unix s.
RpcOutcome HandleStats(Counter& counter, const std::string& method, const json& params, double now);
// The answer to sensors.history.
RpcOutcome HandleHistory(const SensorHistory& history, const json& params, double now);

}  // namespace mowbite_stats

#endif  // MOWBITE_STATS_STATS_RPC_H
