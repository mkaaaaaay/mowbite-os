#include "mowbite_stats/stats_rpc.h"

#include <iterator>
#include <regex>
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

// An optional unix time of params into out.
std::optional<RpcOutcome> UnixTime(const json& params, const char* key, std::optional<int64_t>* out) {
  if (!params.contains(key)) return std::nullopt;
  if (!params[key].is_number_integer() || params[key].get<int64_t>() < 0) {
    return Bad(key, "type", "unix s, a whole number");
  }
  *out = params[key].get<int64_t>();
  return std::nullopt;
}

}  // namespace

const std::vector<std::string>& StatsMethods() {
  static const std::vector<std::string> methods = {"stats.get", "stats.part_done", "stats.part_due", kHistoryMethod};
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

RpcOutcome HandleHistory(const SensorHistory& history, const json& params, double now) {
  if (auto bad = CheckKeys(params, {"client", "sensor_ids", "from", "to", "step"}, true)) return *bad;
  HistoryQuery query;
  if (params.is_null()) return {history.Query(query, now), std::nullopt, false};
  if (params.contains("client")) {
    static const std::regex client(mowbite_api::kClientPattern);
    if (!params["client"].is_string()) return Bad("client", "type", "a text");
    if (!std::regex_match(params["client"].get<std::string>(), client)) {
      return Bad("client", "format", mowbite_api::kClientPattern);
    }
  }
  if (params.contains("sensor_ids")) {
    const json& ids = params["sensor_ids"];
    if (!ids.is_array()) return Bad("sensor_ids", "type", "a list of sensor ids");
    query.ids.emplace();
    for (const json& id : ids) {
      if (!id.is_string()) return Bad("sensor_ids", "type", "a list of sensor ids");
      query.ids->push_back(id.get<std::string>());
    }
  }
  if (auto bad = UnixTime(params, "from", &query.from)) return *bad;
  if (auto bad = UnixTime(params, "to", &query.to)) return *bad;
  if (query.from && query.to && *query.from > *query.to) return Bad("from", "range", "not after to");
  if (params.contains("step")) {
    const int64_t largest = std::end(mowbite_api::kSensorsHistorySteps)[-1];
    const std::string range = "s from 1 to " + std::to_string(largest);
    if (!params["step"].is_number_integer()) return Bad("step", "type", range);
    query.step = params["step"].get<int64_t>();
    if (query.step < 1 || query.step > largest) return Bad("step", "range", range);
  }
  return {history.Query(query, now), std::nullopt, false};
}

}  // namespace mowbite_stats
