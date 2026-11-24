#include "mowbite_schedule/schedule_rpc.h"

#include <cmath>

#include "mowbite_time/sun.h"

namespace mowbite_schedule {

namespace {

constexpr int kSchema = 1;

RpcError Bad(const std::string& field, const std::string& reason, const std::string& message) {
  return {kInvalidParams, message, {{"field", field}, {"reason", reason}}};
}

json UnixOrNull(const std::optional<int64_t>& value) {
  return value ? json(*value) : json();
}

std::optional<int64_t> Rounded(const std::optional<double>& value) {
  if (!value) return std::nullopt;
  return static_cast<int64_t>(std::llround(*value));
}

}  // namespace

SunTimes SunOfDay(const TimeZone& zone, int64_t wall, std::optional<double> latitude, std::optional<double> longitude) {
  if (!latitude || !longitude) return {};
  const int64_t noon = zone.ToUnix(zone.ToLocal(wall).date, 12 * 60);
  const mowbite_time::SunDay day = mowbite_time::SunTimes(*latitude, *longitude, static_cast<double>(noon));
  return {Rounded(day.rise), Rounded(day.set)};
}

json ReadyToJson(const Ready& ready) {
  return {{"ok", ready.ok}, {"reason", ready.reason.empty() ? json() : json(ready.reason)}};
}

json GetAnswer(const Scheduler& scheduler, const ClockState& clock, const SunTimes& sun) {
  const ScheduleFile& file = scheduler.file();
  return {{"schema", kSchema},
          {"version", file.version},
          {"available", scheduler.available()},
          {"reason", scheduler.available() ? json() : json(scheduler.UnavailableReason())},
          {"clock_ok", clock.ok},
          {"clock_reason", clock.ok ? json() : json(clock.reason)},
          {"next_start", UnixOrNull(scheduler.NextStart())},
          {"schedule", ScheduleToJson(file.schedule)},
          {"sun", {{"rise", UnixOrNull(sun.rise)}, {"set", UnixOrNull(sun.set)}}}};
}

std::optional<RpcError> CheckSet(const json& params, int64_t stored_version, json* change) {
  if (!params.is_object()) return Bad("params", "type", "params are {schedule, version?}");
  for (const auto& item : params.items()) {
    if (item.key() != "schedule" && item.key() != "version") {
      return Bad(item.key(), "unknown", "not a key of schedule.set");
    }
  }
  if (!params.contains("schedule")) return Bad("schedule", "missing", "the keys of the schedule to change");
  if (params.contains("version")) {
    const json& version = params["version"];
    if (!version.is_number_integer()) return Bad("version", "type", "version of the last schedule.get");
    if (version.get<int64_t>() != stored_version) {
      return RpcError{
          kVersionChanged,
          "The schedule changed since version " + std::to_string(version.get<int64_t>()) + ", read it again",
          {{"version", stored_version}}};
    }
  }
  *change = params["schedule"];
  return std::nullopt;
}

RpcError InvalidError(const Invalid& invalid) {
  return Bad(invalid.field, invalid.reason, invalid.field + ": " + invalid.message);
}

std::optional<RpcError> CheckLog(const json& params, size_t* limit) {
  *limit = kLogEntries;
  if (params.is_null()) return std::nullopt;
  if (!params.is_object()) return Bad("params", "type", "params are {limit?}");
  for (const auto& item : params.items()) {
    if (item.key() != "limit") return Bad(item.key(), "unknown", "not a key of schedule.log");
  }
  if (!params.contains("limit")) return std::nullopt;
  const json& value = params["limit"];
  if (!value.is_number_integer()) return Bad("limit", "type", "a whole number from 1 to 100");
  const int64_t n = value.get<int64_t>();
  if (n < 1 || n > static_cast<int64_t>(kLogEntries)) return Bad("limit", "range", "a whole number from 1 to 100");
  *limit = static_cast<size_t>(n);
  return std::nullopt;
}

std::optional<RpcError> CheckLeftovers(const json& params, json* change) {
  if (params.is_null()) {
    *change = json::object();
    return std::nullopt;
  }
  if (!params.is_object()) return Bad("params", "type", "params are {enabled?, min_m2?}");
  *change = params;
  return std::nullopt;
}

}  // namespace mowbite_schedule
