#include "mowbite_schedule/model.h"

#include <algorithm>
#include <set>
#include <utility>

namespace mowbite_schedule {

namespace {

constexpr int kSchema = 1;

Invalid Bad(std::string field, std::string reason, const std::string& text) {
  Invalid invalid;
  invalid.message = field + ": " + text;
  invalid.field = std::move(field);
  invalid.reason = std::move(reason);
  return invalid;
}

std::optional<Invalid> ReadSwitch(const json& value, const std::string& field, bool* out) {
  if (!value.is_number_integer() || (value.get<int64_t>() != 0 && value.get<int64_t>() != 1)) {
    return Bad(field, "type", "0 or 1");
  }
  *out = value.get<int64_t>() == 1;
  return std::nullopt;
}

std::optional<Invalid> ReadChoice(const json& value, const std::string& field, std::initializer_list<int> choices,
                                  int* out) {
  if (!value.is_number_integer()) return Bad(field, "type", "a whole number");
  const int64_t number = value.get<int64_t>();
  if (std::find(choices.begin(), choices.end(), number) == choices.end()) return Bad(field, "range", "not allowed");
  *out = static_cast<int>(number);
  return std::nullopt;
}

std::optional<Invalid> ReadClock(const json& value, const std::string& field, int* out) {
  if (!value.is_string()) return Bad(field, "type", "a time \"HH:MM\"");
  const std::optional<int> minutes = ParseClockTime(value.get<std::string>());
  if (!minutes) return Bad(field, "format", "a time \"HH:MM\" from 00:00 to 23:59");
  *out = *minutes;
  return std::nullopt;
}

// A list of area ids: texts up to kMaxAreaId characters, at most kMaxAreas, none twice.
std::optional<Invalid> ReadAreas(const json& value, const std::string& field, std::vector<std::string>* out) {
  if (!value.is_array()) return Bad(field, "type", "a list of area ids");
  if (value.size() > kMaxAreas) return Bad(field, "too_many", "at most " + std::to_string(kMaxAreas) + " areas");
  std::set<std::string> seen;
  out->clear();
  for (const json& id : value) {
    if (!id.is_string() || id.get<std::string>().empty()) return Bad(field, "type", "area ids are texts");
    const std::string text = id.get<std::string>();
    if (text.size() > kMaxAreaId) return Bad(field, "range", "an area id has up to 64 characters");
    if (!seen.insert(text).second) return Bad(field, "duplicate", "the area " + text + " is in the list twice");
    out->push_back(text);
  }
  return std::nullopt;
}

std::optional<Invalid> ReadPlan(const json& value, const std::string& field, Plan* plan) {
  if (!value.is_object()) return Bad(field, "type", "a plan is an object");
  if (!value.contains("days")) return Bad(field + ".days", "missing", "required");
  if (!value.contains("time")) return Bad(field + ".time", "missing", "required");
  for (const auto& item : value.items()) {
    const std::string& key = item.key();
    const json& v = item.value();
    const std::string at = field + "." + key;
    std::optional<Invalid> bad;
    if (key == "days") {
      if (!v.is_array() || v.empty()) return Bad(at, "type", "a list of weekdays 1 to 7, not empty");
      std::set<int> seen;
      for (const json& day : v) {
        if (!day.is_number_integer()) return Bad(at, "type", "weekdays are whole numbers");
        const int64_t number = day.get<int64_t>();
        if (number < 1 || number > 7) return Bad(at, "range", "weekdays are 1 (Monday) to 7 (Sunday)");
        if (!seen.insert(static_cast<int>(number)).second) return Bad(at, "duplicate", "a weekday twice");
        plan->days.push_back(static_cast<int>(number));
      }
    } else if (key == "time") {
      bad = ReadClock(v, at, &plan->time);
    } else if (key == "end") {
      if (!v.is_null()) {
        int end = 0;
        bad = ReadClock(v, at, &end);
        plan->end = end;
      }
    } else if (key == "areas") {
      bad = ReadAreas(v, at, &plan->areas);
    } else if (key == "dark") {
      bad = ReadSwitch(v, at, &plan->dark);
    } else if (key == "sunset") {
      bad = ReadSwitch(v, at, &plan->sunset);
    } else if (key == "fresh") {
      bad = ReadSwitch(v, at, &plan->fresh);
    } else if (key == "angle_offset_deg") {
      if (!v.is_null()) {
        if (!v.is_number()) return Bad(at, "type", "a number of degrees");
        const double angle = v.get<double>();
        if (!(angle >= -180.0 && angle <= 180.0)) return Bad(at, "range", "from -180 to 180");
        plan->angle_offset_deg = angle;
      }
    } else {
      return Bad(at, "unknown", "not a key of a plan");
    }
    if (bad) return bad;
  }
  if (plan->end && *plan->end == plan->time) return Bad(field + ".end", "equal_time", "the same as time");
  return std::nullopt;
}

std::optional<Invalid> ReadPause(const json& value, std::optional<Pause>* out) {
  if (value.is_null()) {
    out->reset();
    return std::nullopt;
  }
  if (!value.is_object()) return Bad("pause", "type", "null or {until, areas}");
  Pause pause;
  if (!value.contains("until")) return Bad("pause.until", "missing", "required");
  for (const auto& item : value.items()) {
    const json& v = item.value();
    if (item.key() == "until") {
      const std::optional<Date> until = v.is_string() ? ParseDate(v.get<std::string>()) : std::nullopt;
      if (!until) return Bad("pause.until", "format", "a date \"YYYY-MM-DD\"");
      pause.until = *until;
    } else if (item.key() == "areas") {
      if (v.is_string() && v.get<std::string>() == "all") continue;
      std::vector<std::string> areas;
      if (const std::optional<Invalid> bad = ReadAreas(v, "pause.areas", &areas)) return bad;
      if (areas.empty()) return Bad("pause.areas", "empty", "\"all\" or a list of area ids");
      pause.areas = std::move(areas);
    } else {
      return Bad("pause." + item.key(), "unknown", "not a key of the pause");
    }
  }
  *out = std::move(pause);
  return std::nullopt;
}

bool ShareDay(const Plan& a, const Plan& b) {
  return std::any_of(a.days.begin(), a.days.end(),
                     [&b](int day) { return std::find(b.days.begin(), b.days.end(), day) != b.days.end(); });
}

json PlanToJson(const Plan& plan) {
  json out{{"days", plan.days}, {"time", FormatClockTime(plan.time)}, {"areas", plan.areas},
           {"dark", plan.dark ? 1 : 0}, {"sunset", plan.sunset ? 1 : 0}, {"fresh", plan.fresh ? 1 : 0}};
  if (plan.end) out["end"] = FormatClockTime(*plan.end);
  if (plan.angle_offset_deg) out["angle_offset_deg"] = *plan.angle_offset_deg;
  return out;
}

json PlansToJson(const std::vector<Plan>& plans) {
  json out = json::array();
  for (const Plan& plan : plans) out.push_back(PlanToJson(plan));
  return out;
}

}  // namespace

std::optional<Invalid> ApplySchedule(const json& change, bool available, int64_t now, ScheduleFile* file) {
  if (!change.is_object()) return Bad("schedule", "type", "an object of the keys to change");
  Schedule next = file->schedule;
  for (const auto& item : change.items()) {
    const std::string& key = item.key();
    const json& v = item.value();
    std::optional<Invalid> bad;
    if (key == "enabled") {
      bad = ReadSwitch(v, key, &next.enabled);
    } else if (key == "min_battery") {
      if (!v.is_number_integer() || v.get<int64_t>() < 0 || v.get<int64_t>() > 100) {
        return Bad(key, v.is_number_integer() ? "range" : "type", "a whole number from 0 to 100");
      }
      next.min_battery = static_cast<int>(v.get<int64_t>());
    } else if (key == "skip_rain") {
      bad = ReadSwitch(v, key, &next.skip_rain);
    } else if (key == "skip_forecast") {
      bad = ReadSwitch(v, key, &next.skip_forecast);
    } else if (key == "forecast_hours") {
      bad = ReadChoice(v, key, {1, 2, 3}, &next.forecast_hours);
    } else if (key == "skip_cold") {
      bad = ReadSwitch(v, key, &next.skip_cold);
    } else if (key == "min_temp") {
      bad = ReadChoice(v, key, {3, 5, 8, 10}, &next.min_temp);
    } else if (key == "tz") {
      if (!v.is_string()) return Bad(key, "type", "a POSIX TZ text");
      std::string reason;
      const std::optional<TimeZone> zone = TimeZone::Parse(v.get<std::string>(), &reason);
      if (!zone) return Bad(key, reason, "not a POSIX TZ text the schedule reads");
      next.tz = v.get<std::string>();
      next.zone = *zone;
    } else if (key == "plans") {
      if (!v.is_array()) return Bad(key, "type", "a list of plans");
      if (v.size() > kMaxPlans) return Bad(key, "too_many", "at most " + std::to_string(kMaxPlans) + " plans");
      next.plans.clear();
      for (size_t i = 0; i < v.size(); ++i) {
        Plan plan;
        if (const std::optional<Invalid> bad_plan = ReadPlan(v[i], "plans[" + std::to_string(i) + "]", &plan)) {
          return bad_plan;
        }
        next.plans.push_back(std::move(plan));
      }
    } else if (key == "pause") {
      bad = ReadPause(v, &next.pause);
    } else {
      return Bad(key, "unknown", "not a key of the schedule");
    }
    if (bad) return bad;
  }
  for (size_t i = 0; i < next.plans.size(); ++i) {
    for (size_t j = 0; j < i; ++j) {
      if (next.plans[i].time == next.plans[j].time && ShareDay(next.plans[i], next.plans[j])) {
        return Bad("plans[" + std::to_string(i) + "].time", "duplicate_time",
                   "another plan starts at the same time on a common day");
      }
    }
  }
  if (change.contains("enabled") && next.enabled && !available) {
    return Bad("enabled", "platform_off", "the schedule is not released for this platform");
  }
  const bool moved = next.enabled != file->schedule.enabled || next.tz != file->schedule.tz ||
                     PlansToJson(next.plans) != PlansToJson(file->schedule.plans);
  file->schedule = std::move(next);
  file->version += 1;
  if (moved) file->changed_at = now;
  return std::nullopt;
}

std::optional<Invalid> ApplyLeftovers(const json& change, ScheduleFile* file) {
  if (!change.is_object()) return Bad("leftovers", "type", "an object of the keys to change");
  Leftovers next = file->leftovers;
  for (const auto& item : change.items()) {
    const json& v = item.value();
    if (item.key() == "enabled") {
      if (const std::optional<Invalid> bad = ReadSwitch(v, "enabled", &next.enabled)) return bad;
      if (next.enabled) return Bad("enabled", "unsupported", "leftovers wait for the planner on MowBite OS");
    } else if (item.key() == "min_m2") {
      if (!v.is_number()) return Bad("min_m2", "type", "a number of square metres");
      const double m2 = v.get<double>();
      if (!(m2 >= 0.0 && m2 <= 10000.0)) return Bad("min_m2", "range", "from 0 to 10000");
      next.min_m2 = m2;
    } else {
      return Bad(item.key(), "unknown", "not a key of the leftovers");
    }
  }
  file->leftovers = next;
  file->version += 1;
  return std::nullopt;
}

json ScheduleToJson(const Schedule& schedule) {
  json pause = nullptr;
  if (schedule.pause) {
    pause = {{"until", FormatDate(schedule.pause->until)}};
    if (schedule.pause->areas) pause["areas"] = *schedule.pause->areas;
    else pause["areas"] = "all";
  }
  return {{"enabled", schedule.enabled ? 1 : 0},
          {"min_battery", schedule.min_battery},
          {"skip_rain", schedule.skip_rain ? 1 : 0},
          {"skip_forecast", schedule.skip_forecast ? 1 : 0},
          {"forecast_hours", schedule.forecast_hours},
          {"skip_cold", schedule.skip_cold ? 1 : 0},
          {"min_temp", schedule.min_temp},
          {"tz", schedule.tz},
          {"plans", PlansToJson(schedule.plans)},
          {"pause", pause}};
}

json LeftoversToJson(const Leftovers& leftovers) {
  return {{"enabled", leftovers.enabled ? 1 : 0}, {"min_m2", leftovers.min_m2}};
}

json FileToJson(const ScheduleFile& file) {
  return {{"schema", kSchema},
          {"version", file.version},
          {"changed_at", file.changed_at},
          {"schedule", ScheduleToJson(file.schedule)},
          {"leftovers", LeftoversToJson(file.leftovers)}};
}

std::optional<ScheduleFile> FileFromJson(const json& value, std::string* error) {
  if (!value.is_object() || value.value("schema", 0) != kSchema) {
    *error = "not schema " + std::to_string(kSchema);
    return std::nullopt;
  }
  if (!value.contains("version") || !value["version"].is_number_integer() || !value.contains("changed_at") ||
      !value["changed_at"].is_number_integer() || !value.contains("schedule") || !value.contains("leftovers")) {
    *error = "version, changed_at, schedule or leftovers missing";
    return std::nullopt;
  }
  ScheduleFile file;
  // Taken as stored: a schedule switched on before the platform was switched off stays so, the decision skips it.
  if (const std::optional<Invalid> bad = ApplySchedule(value["schedule"], true, 0, &file)) {
    *error = "schedule." + bad->message;
    return std::nullopt;
  }
  if (const std::optional<Invalid> bad = ApplyLeftovers(value["leftovers"], &file)) {
    *error = "leftovers." + bad->message;
    return std::nullopt;
  }
  file.version = value["version"].get<int64_t>();
  file.changed_at = value["changed_at"].get<int64_t>();
  return file;
}

bool PausedAll(const Schedule& schedule, const Date& date) {
  return schedule.pause && !(schedule.pause->until < date) && !schedule.pause->areas;
}

bool PausedArea(const Schedule& schedule, const Date& date, const std::string& area) {
  if (!schedule.pause || schedule.pause->until < date) return false;
  if (!schedule.pause->areas) return true;
  const std::vector<std::string>& areas = *schedule.pause->areas;
  return std::find(areas.begin(), areas.end(), area) != areas.end();
}

const Plan* PlanAt(const Schedule& schedule, const Date& date, int time) {
  const int weekday = Weekday(date);
  for (const Plan& plan : schedule.plans) {
    if (plan.time == time && std::find(plan.days.begin(), plan.days.end(), weekday) != plan.days.end()) return &plan;
  }
  return nullptr;
}

}  // namespace mowbite_schedule
