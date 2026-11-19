#ifndef MOWBITE_SCHEDULE_SCHEDULE_RPC_H
#define MOWBITE_SCHEDULE_SCHEDULE_RPC_H

#include <cstdint>
#include <optional>
#include <string>

#include "mowbite_schedule/scheduler.h"

namespace mowbite_schedule {

// schedule.set named another version than the one stored.
constexpr int kVersionChanged = -32001;
constexpr int kInvalidParams = -32602;
// The change could not be stored: it holds until the OS restarts, then the stored one comes back.
constexpr int kInternalError = -32603;

// A refused call: its code, text and data (null for none).
struct RpcError {
  int code = 0;
  std::string message;
  json data;
};

struct ClockState {
  bool ok = false;
  std::string reason;
};

// Sunrise and sunset in unix s, none for either.
struct SunTimes {
  std::optional<int64_t> rise;
  std::optional<int64_t> set;
};

// Sunrise and sunset of the local day of wall after the zone, at the datum; none without a datum.
SunTimes SunOfDay(const TimeZone& zone, int64_t wall, std::optional<double> latitude, std::optional<double> longitude);

// The MQTT topic of ReadyNow, retained; empty once no start is to come.
constexpr char kReadyTopic[] = "mowbite/schedule/ready";
// {ok, reason}, reason null with ok.
json ReadyToJson(const Ready& ready);

// The answer of schedule.get and schedule.set.
json GetAnswer(const Scheduler& scheduler, const ClockState& clock, const SunTimes& sun);

// What schedule.set changes, or why it refuses before the change: params not {schedule, version?}, or version not
// stored_version.
std::optional<RpcError> CheckSet(const json& params, int64_t stored_version, json* change);
// The -32602 of a change that is not taken.
RpcError InvalidError(const Invalid& invalid);
// How many entries schedule.log answers with.
std::optional<RpcError> CheckLog(const json& params, size_t* limit);
// What schedule.leftovers changes: params an object, or none to read.
std::optional<RpcError> CheckLeftovers(const json& params, json* change);

}  // namespace mowbite_schedule

#endif  // MOWBITE_SCHEDULE_SCHEDULE_RPC_H
