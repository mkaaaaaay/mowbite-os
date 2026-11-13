#include "mowbite_stats/stats_rpc.h"

#include <set>

#include "mowbite_api/catalog.h"

namespace mowbite_stats {

namespace {

RpcOutcome Bad(const std::string& field, const std::string& reason, const std::string& message) {
  RpcOutcome out;
  out.error = RpcError{kInvalidParams, field + ": " + message, {{"field", field}, {"reason", reason}}};
  return out;
}

// params an object of only these keys (null for none when none is required).
std::optional<RpcOutcome> CheckKeys(const json& params, const std::set<std::string>& keys, bool none_ok) {
  if (params.is_null() && none_ok) return std::nullopt;
  if (!params.is_object()) return Bad("params", "type", "an object");
  for (const auto& item : params.items()) {
    if (keys.count(item.key()) == 0) return Bad(item.key(), "unknown", "not a key of this method");
  }
  return std::nullopt;
}

std::optional<RpcOutcome> CheckPart(const Counter& counter, const json& params) {
  if (!params.contains("part")) return Bad("part", "missing", "the key of a part");
  if (!params["part"].is_string()) return Bad("part", "type", "the key of a part");
  for (const Part& part : counter.parts()) {
    if (part.key == params["part"].get<std::string>()) return std::nullopt;
  }
  return Bad("part", "unknown", "not a part of stats.get");
}

}  // namespace

const std::vector<std::string>& StatsMethods() {
  static const std::vector<std::string> methods = {"stats.get", "stats.part_done", "stats.part_due"};
  return methods;
}

RpcOutcome HandleStats(Counter& counter, const std::string& method, const json& params, double now) {
  if (method == "stats.get") {
    if (auto bad = CheckKeys(params, {"from"}, true)) return *bad;
    std::optional<int64_t> from;
    if (params.is_object() && params.contains("from")) {
      if (!params["from"].is_number_integer() || params["from"].get<int64_t>() < 0) {
        return Bad("from", "type", "unix s, a whole number");
      }
      from = params["from"].get<int64_t>();
    }
    return {counter.Get(from, now), std::nullopt, false};
  }
  if (method == "stats.part_done") {
    if (auto bad = CheckKeys(params, {"part"}, false)) return *bad;
    if (auto bad = CheckPart(counter, params)) return *bad;
    counter.PartDone(params["part"].get<std::string>(), now);
    return {counter.Get(static_cast<int64_t>(now), now), std::nullopt, true};
  }
  if (auto bad = CheckKeys(params, {"part", "hours"}, false)) return *bad;
  if (auto bad = CheckPart(counter, params)) return *bad;
  const std::string range = "blade hours from 1 to " + std::to_string(mowbite_api::kStatsMaxDueHours);
  if (!params.contains("hours")) return Bad("hours", "missing", range);
  if (!params["hours"].is_number_integer()) return Bad("hours", "type", range);
  const int64_t hours = params["hours"].get<int64_t>();
  if (hours < 1 || hours > mowbite_api::kStatsMaxDueHours) return Bad("hours", "range", range);
  counter.PartDue(params["part"].get<std::string>(), static_cast<double>(hours));
  return {counter.Get(static_cast<int64_t>(now), now), std::nullopt, true};
}

}  // namespace mowbite_stats
