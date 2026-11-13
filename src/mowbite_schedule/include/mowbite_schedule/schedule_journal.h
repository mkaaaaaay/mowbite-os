#ifndef MOWBITE_SCHEDULE_SCHEDULE_JOURNAL_H
#define MOWBITE_SCHEDULE_SCHEDULE_JOURNAL_H

#include <mowbite_journal/journaled.h>

#include <stdexcept>
#include <string>

#include "mowbite_schedule/scheduler.h"

// The journal of the Scheduler (stream "schedule"). A new field of a struct below needs its place in its line, the size
// check makes the build say so.

namespace mowbite_schedule {

inline void to_json(json& j, const ScheduleFile& file) {
  j = FileToJson(file);
}

inline void from_json(const json& j, ScheduleFile& file) {
  std::string error;
  const std::optional<ScheduleFile> read = FileFromJson(j, &error);
  if (!read) throw std::invalid_argument("not a schedule file: " + error);
  file = *read;
}

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SchedulerStart, file, available, state, log, now)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Area, id, mowable)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Facts, wall, mono, clock_ok, clock_reason, logic_fresh, state, docked, emergency,
                                   battery, rain_sensor, rain_wet, rain_mode, hold, latitude, longitude, areas)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(WeatherRequest, id, latitude, longitude, hours)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Weather, id, ok, age_s, has_rain, rain_now_mm, rain_hour_mm, has_temp, temp_c)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RunRequest, run_id, area_ids, pass_over, restart, angle_offset_deg, dark_allowed,
                                   until_sunset, end_in_s)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DropRequest, run_id, area_ids)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RunStatus, run_id, phase, end_code, detail)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Output, weather, start, drop, state_changed, log_changed, file_changed)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Invalid, field, reason, message)

static_assert(sizeof(SchedulerStart) == 288);
static_assert(sizeof(Area) == 40);
static_assert(sizeof(Facts) == 176);
static_assert(sizeof(WeatherRequest) == 32);
static_assert(sizeof(Weather) == 80);
static_assert(sizeof(RunRequest) == 120);
static_assert(sizeof(DropRequest) == 56);
static_assert(sizeof(RunStatus) == 104);
static_assert(sizeof(Output) == 240);
static_assert(sizeof(Invalid) == 96);

}  // namespace mowbite_schedule

template <>
struct mowbite_journal::CoreTraits<mowbite_schedule::Scheduler> {
  using S = mowbite_schedule::Scheduler;
  static constexpr const char* kName = "schedule";
  using Config = mowbite_schedule::SchedulerStart;
  MOWBITE_JOURNAL_METHODS(&S::Tick, &S::OnWeather, &S::OnStartAnswer, &S::StartFailed, &S::OnRunStatus, &S::SetSchedule,
                          &S::SetLeftovers, &S::TakeOutput);
};

#endif  // MOWBITE_SCHEDULE_SCHEDULE_JOURNAL_H
