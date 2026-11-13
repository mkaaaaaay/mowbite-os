#ifndef MOWBITE_SCHEDULE_MODEL_H
#define MOWBITE_SCHEDULE_MODEL_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "mowbite_schedule/tz.h"

namespace mowbite_schedule {

using json = nlohmann::json;

constexpr size_t kMaxPlans = 32;
constexpr size_t kMaxAreas = 64;
constexpr size_t kMaxAreaId = 64;

// One start time of the schedule. Its time names it: no two plans share a time on a day.
struct Plan {
  std::vector<int> days;           // 1 Monday to 7 Sunday
  int time = 0;                    // minutes of the local day
  std::vector<std::string> areas;  // in order; empty: every mowing area
  std::optional<int> end;          // minutes of the local day; not after time: on the next day
  bool dark = false;               // may start and mow in the dark
  bool sunset = false;             // with dark: home at sunset all the same
  bool fresh = false;              // begins a new job instead of carrying on the open one
  std::optional<double> angle_offset_deg;
};

// No starts up to and including until; for every area or the ones listed.
struct Pause {
  Date until;
  std::optional<std::vector<std::string>> areas;
};

struct Schedule {
  bool enabled = false;
  int min_battery = 80;
  bool skip_rain = true;
  bool skip_forecast = false;
  int forecast_hours = 1;
  bool skip_cold = false;
  int min_temp = 5;
  std::string tz = "UTC0";
  TimeZone zone = TimeZone::Utc();
  std::vector<Plan> plans;
  std::optional<Pause> pause;
};

struct Leftovers {
  bool enabled = false;
  double min_m2 = 0.5;
};

// Everything schedule.json keeps.
struct ScheduleFile {
  int64_t version = 0;
  int64_t changed_at = 0;  // unix s of the last change of plans, enabled or tz; a plan fires only after it
  Schedule schedule;
  Leftovers leftovers;
};

// Why a change is not taken (-32602): the field at fault, a reason word and a text.
struct Invalid {
  std::string field;
  std::string reason;
  std::string message;
};

// A change of schedule.set: every key replaced whole, all or nothing. enabled 1 only where the platform allows it.
// On success changed_at moves to now when plans, enabled or tz changed, and the version counts up.
std::optional<Invalid> ApplySchedule(const json& change, bool available, int64_t now, ScheduleFile* file);
// A change of schedule.leftovers. Leftovers on the OS wait for the planner (enabled 1 is refused until then).
std::optional<Invalid> ApplyLeftovers(const json& change, ScheduleFile* file);

json ScheduleToJson(const Schedule& schedule);
json LeftoversToJson(const Leftovers& leftovers);
json FileToJson(const ScheduleFile& file);
// The file as written by FileToJson; nothing and the reason for anything else.
std::optional<ScheduleFile> FileFromJson(const json& value, std::string* error);

// Whether the pause covers that local date, for all areas or for this one.
bool PausedAll(const Schedule& schedule, const Date& date);
bool PausedArea(const Schedule& schedule, const Date& date, const std::string& area);
// The plan with this time that runs on the weekday of date, if any.
const Plan* PlanAt(const Schedule& schedule, const Date& date, int time);

}  // namespace mowbite_schedule

#endif  // MOWBITE_SCHEDULE_MODEL_H
