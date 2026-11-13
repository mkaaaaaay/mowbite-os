#ifndef MOWBITE_SCHEDULE_SCHEDULER_H
#define MOWBITE_SCHEDULE_SCHEDULER_H

#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_schedule/model.h"

namespace mowbite_schedule {

// A plan fires in this many seconds after its time, later only from waiting.
constexpr int64_t kWindow = 60;
// How long a plan waits for the battery or the end of a hold, from its time.
constexpr int64_t kWaitLimit = 2 * 3600;
// How long the answer of the weather may take.
constexpr double kWeatherDeadline = 15.0;
// Weather older than this counts as none.
constexpr double kWeatherMaxAge = 30 * 60.0;
// A start that has not reached mowing fails after this long without word of the logic.
constexpr double kRunSilence = 120.0;
constexpr size_t kLogEntries = 100;
// Rain now from this (mm), or in one of the coming hours from this (mm), skips the start.
constexpr double kRainNowMm = 0.05;
constexpr double kRainHourMm = 0.2;

struct Area {
  std::string id;
  bool mowable = false;  // active and mowed
};

// What the scheduler sees once a second.
struct Facts {
  int64_t wall = 0;  // unix s
  double mono = 0;   // s
  bool clock_ok = false;
  std::string clock_reason;
  bool logic_fresh = false;  // robot_state came within the last 5 s
  std::string state;         // of the logic, e.g. IDLE
  bool docked = false;       // on the contacts
  bool emergency = false;    // emergency stop, its latch or the restart hold
  int battery = 0;           // %
  bool rain_sensor = false;
  bool rain_wet = false;
  int rain_mode = 0;
  bool hold = false;  // system.hold or a driver holds
  std::optional<double> latitude;
  std::optional<double> longitude;
  std::vector<Area> areas;  // of the map, in its order
};

struct WeatherRequest {
  int64_t id = 0;
  double latitude = 0;
  double longitude = 0;
  int hours = 1;
};

struct Weather {
  int64_t id = 0;
  bool ok = false;
  double age_s = 0;
  bool has_rain = false;
  double rain_now_mm = 0;
  std::vector<double> rain_hour_mm;
  bool has_temp = false;
  double temp_c = 0;
};

// A start for the logic (StartRun).
struct RunRequest {
  std::string run_id;
  std::vector<std::string> area_ids;   // the list of the plan, the job it names; empty: every mowing area
  std::vector<std::string> pass_over;  // in area_ids or on the map: paused or unknown, left out by this run only
  bool restart = false;
  std::optional<double> angle_offset_deg;
  bool dark_allowed = false;  // starts and mows in the dark
  bool until_sunset = true;   // docks at the next sunset
  int64_t end_in_s = 0;       // home after this long; 0: no end
};

// Areas the running plan leaves out from now on (DropTasks); empty: all of them.
struct DropRequest {
  std::string run_id;
  std::vector<std::string> area_ids;
};

enum class Phase { kNone, kAnnouncing, kChecking, kUndocking, kMowing, kEnded };

// What the logic says of a run it took (RunStatus).
struct RunStatus {
  std::string run_id;
  Phase phase = Phase::kNone;
  std::string end_code;  // with kEnded
  std::string detail;
};

struct LogEntry {
  int64_t t = 0;
  std::string what;
  json detail;  // text or null
  json more;    // text, number or null
  json at;      // "HH:MM" of the plan or null
};

// What the node has to do after a call.
struct Output {
  std::optional<WeatherRequest> weather;
  // Write the state first; if that fails, StartFailed("storage") instead of the start.
  std::optional<RunRequest> start;
  std::optional<DropRequest> drop;
  bool state_changed = false;
  bool log_changed = false;
  bool file_changed = false;
};

// The decision of the schedule without clock, files or ROS: the node gives it the time and the states, carries out
// what it asks for and stores its texts. A plan fires at most once per local date (S4), only from the station (S1)
// with a safe clock (S3); one missed while the OS was off is not made up (S5).
class Scheduler {
 public:
  // file and state as stored (state: schedule-state.json, null when missing), log: schedule-log.json entries.
  Scheduler(ScheduleFile file, bool available, const json& state, const json& log, int64_t now);

  void Tick(const Facts& facts);
  void OnWeather(const Weather& weather);
  // The answer of the logic to a start: taken, or refused with the reason (3.1a) and its value.
  void OnStartAnswer(const std::string& run_id, bool accepted, const std::string& reason, const json& value);
  // The start did not reach the logic (detail: storage, service).
  void StartFailed(const std::string& run_id, const std::string& detail);
  void OnRunStatus(const RunStatus& status);
  // schedule.set and schedule.leftovers; a pause during a run of the schedule leaves its areas out.
  std::optional<Invalid> SetSchedule(const json& change, int64_t now);
  std::optional<Invalid> SetLeftovers(const json& change);

  // What has to be done since the last call, then cleared.
  Output TakeOutput();

  const ScheduleFile& file() const { return file_; }
  bool available() const { return available_; }
  // The next start (unix s): now while a plan waits or starts; nothing without one, switched off or with an unsafe
  // clock.
  std::optional<int64_t> NextStart() const;
  json StateJson() const;
  json LogJson() const;
  json LogEntries(size_t limit) const;

 private:
  struct PlanKey {
    Date date;
    int time = 0;
    bool operator<(const PlanKey& other) const;
    bool operator==(const PlanKey& other) const { return date == other.date && time == other.time; }
  };
  enum class WaitKind { kBattery, kHold };
  struct Waiting {
    PlanKey key;
    WaitKind kind = WaitKind::kBattery;
    bool battery_logged = false;
    bool hold_logged = false;
  };
  struct Pending {
    PlanKey key;
    int64_t id = 0;
    double deadline = 0;
    std::optional<Weather> answer;
  };
  struct Run {
    PlanKey key;
    std::string run_id;
    std::vector<std::string> area_ids;
    bool asked = false;  // StartRun sent, no answer yet
    bool mowing = false;
    Phase phase = Phase::kNone;
    double last_word = 0;  // mono
  };
  enum class Verdict { kGo, kDecided, kWaitOffline };

  void Log(const std::string& what, json detail, json more, const std::optional<PlanKey>& key);
  // Logs the word that decides the plan and ends its wait.
  void Decide(const PlanKey& key, const std::string& what, json detail);
  void Mark(const PlanKey& key);
  bool Handled(const PlanKey& key) const;
  int64_t PlanUnix(const PlanKey& key) const;
  std::optional<PlanKey> DuePlan() const;
  void SweepMissed();
  void EndWait(const char* battery_word, const char* hold_word);
  void TickWaiting();
  // Rows T2 to T13 for a plan; kGo when it may start.
  Verdict Check(const PlanKey& key, const Plan& plan, bool from_wait);
  void Ask(const PlanKey& key, const Plan& plan, bool from_wait);
  void ResolveWeather();
  void Start(const PlanKey& key, const Plan& plan);
  // DropTasks for the areas the pause covers today while a run of the schedule goes on.
  void DropPaused();

  ScheduleFile file_;
  const bool available_;
  Facts facts_;
  bool ticked_ = false;
  std::map<Date, std::set<int>> handled_;
  // Due while the logic was silent, skip_offline once the window is over.
  std::set<PlanKey> seen_offline_;
  // Logged skip_clock: with an unsafe clock the date may be wrong, so these marks are not stored.
  std::set<PlanKey> clock_skipped_;
  std::optional<Waiting> waiting_;
  std::optional<Pending> pending_;
  std::optional<Run> run_;
  std::deque<LogEntry> log_;
  int64_t next_weather_id_ = 1;
  Output out_;
};

json EntryToJson(const LogEntry& entry);

}  // namespace mowbite_schedule

#endif  // MOWBITE_SCHEDULE_SCHEDULER_H
