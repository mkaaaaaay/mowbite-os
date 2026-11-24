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
// data_rollback only from journals since it came; before it false.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Facts, wall, mono, clock_ok, clock_reason, logic_fresh, state, docked,
                                                emergency, battery, rain_sensor, rain_wet, rain_mode, hold, data_rollback,
                                                latitude, longitude, areas)
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
                          &S::SetLeftovers, &S::TakeOutput, &S::OnDropAnswer);

  // Every call with a place or the wall clock, for a copy of a journal without its place (mowbite_replay anonymize).
  template <typename A>
  static void Anonymize(A& a) {
    using namespace mowbite_schedule;
    a.Config([](const auto& o, SchedulerStart& start) {
      start.now += o.shift_s();
      if (start.file.changed_at > 0) start.file.changed_at += o.shift_s();
      if (start.file.schedule.pause) {
        Date& until = start.file.schedule.pause->until;
        o.ShiftDate(&until.year, &until.month, &until.day);
      }
      o.ShiftWallIn(&start.state);
      o.ShiftWallIn(&start.log, {"t"});
    });
    a.template Args<&S::Tick>([](const auto& o, Facts& facts) {
      facts.wall += o.shift_s();
      if (facts.latitude) facts.latitude = o.latitude;
      if (facts.longitude) facts.longitude = o.longitude;
    });
    a.template Args<&S::OnStartAnswer>([](const auto& o, std::string& run_id, bool&, std::string&, json& value) {
      run_id = o.ShiftDates(run_id);
      o.ShiftWallIn(&value);
    });
    a.template Args<&S::StartFailed>(
        [](const auto& o, std::string& run_id, std::string&) { run_id = o.ShiftDates(run_id); });
    a.template Args<&S::OnDropAnswer>([](const auto& o, std::string& run_id, bool&) { run_id = o.ShiftDates(run_id); });
    a.template Args<&S::OnRunStatus>([](const auto& o, RunStatus& status) {
      status.run_id = o.ShiftDates(status.run_id);
    });
    a.template Args<&S::SetSchedule>([](const auto& o, json& change, int64_t& now) {
      o.ShiftWallIn(&change);
      now += o.shift_s();
    });
    a.template Result<&S::TakeOutput>([](const auto& o, Output& out) {
      if (out.weather) {
        out.weather->latitude = o.latitude;
        out.weather->longitude = o.longitude;
      }
      if (out.start) out.start->run_id = o.ShiftDates(out.start->run_id);
      if (out.drop) out.drop->run_id = o.ShiftDates(out.drop->run_id);
    });
  }
};

#endif  // MOWBITE_SCHEDULE_SCHEDULE_JOURNAL_H
