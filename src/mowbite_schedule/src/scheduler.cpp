#include "mowbite_schedule/scheduler.h"

#include <algorithm>
#include <cstdio>
#include <iterator>

#include "mowbite_time/sun.h"

namespace mowbite_schedule {

namespace {

constexpr int kStateSchema = 1;
constexpr int kLogSchema = 1;
constexpr char kIdle[] = "IDLE";

const char* PhaseName(Phase phase) {
  switch (phase) {
    case Phase::kNone: return "NONE";
    case Phase::kAnnouncing: return "ANNOUNCING";
    case Phase::kChecking: return "CHECKING";
    case Phase::kUndocking: return "UNDOCKING";
    case Phase::kMowing: return "MOWING";
    case Phase::kEnded: return "ENDED";
  }
  return "NONE";
}

std::string OneDecimal(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.1f", value);
  return text;
}

// The list of a plan against the map: what it names, what this run leaves out and whether anything is left.
struct AreaSplit {
  std::vector<std::string> unknown;
  std::vector<std::string> paused;
  bool left = false;                // a known, mowable area that is not paused
  bool left_without_pause = false;  // the same without the pause
};

AreaSplit SplitAreas(const Schedule& schedule, const Plan& plan, const Date& date, const std::vector<Area>& map) {
  AreaSplit split;
  std::vector<std::string> list = plan.areas;
  if (list.empty()) {
    for (const Area& area : map) list.push_back(area.id);
  }
  for (const std::string& id : list) {
    const auto found = std::find_if(map.begin(), map.end(), [&id](const Area& area) { return area.id == id; });
    if (found == map.end()) {
      split.unknown.push_back(id);
      continue;
    }
    const bool paused = PausedArea(schedule, date, id);
    if (paused) split.paused.push_back(id);
    if (found->mowable) {
      split.left_without_pause = true;
      if (!paused) split.left = true;
    }
  }
  return split;
}

}  // namespace

bool Scheduler::PlanKey::operator<(const PlanKey& other) const {
  if (date != other.date) return date < other.date;
  return time < other.time;
}

json EntryToJson(const LogEntry& entry) {
  return {{"t", entry.t}, {"what", entry.what}, {"detail", entry.detail}, {"more", entry.more}, {"at", entry.at}};
}

Scheduler::Scheduler(ScheduleFile file, bool available, const json& state, const json& log, int64_t now)
    : file_(std::move(file)), available_(available) {
  facts_.wall = now;
  if (log.is_object() && log.value("schema", 0) == kLogSchema && log.contains("entries") && log["entries"].is_array()) {
    for (const json& item : log["entries"]) {
      if (!item.is_object() || !item.contains("t") || !item["t"].is_number_integer() || !item.contains("what") ||
          !item["what"].is_string()) {
        continue;
      }
      LogEntry entry;
      entry.t = item["t"].get<int64_t>();
      entry.what = item["what"].get<std::string>();
      entry.detail = item.value("detail", json());
      entry.more = item.value("more", json());
      entry.at = item.value("at", json());
      log_.push_back(std::move(entry));
    }
    while (log_.size() > kLogEntries) log_.pop_front();
  }
  if (!state.is_object() || state.value("schema", 0) != kStateSchema) return;
  if (state.contains("handled") && state["handled"].is_object()) {
    for (const auto& item : state["handled"].items()) {
      const std::optional<Date> date = ParseDate(item.key());
      if (!date || !item.value().is_array()) continue;
      std::set<int>& times = handled_[*date];
      for (const json& time : item.value()) {
        const std::optional<int> minutes = time.is_string() ? ParseClockTime(time.get<std::string>()) : std::nullopt;
        if (minutes) times.insert(*minutes);
      }
    }
  }
  // A plan that waited when the OS went down is not taken up again (S5).
  const json waiting = state.value("waiting", json());
  if (waiting.is_object() && waiting.contains("time") && waiting["time"].is_string()) {
    const std::optional<int> minutes = ParseClockTime(waiting["time"].get<std::string>());
    if (minutes) Log("skip_missed", nullptr, nullptr, PlanKey{Date(), *minutes});
    out_.state_changed = true;
  }
}

void Scheduler::Log(const std::string& what, json detail, json more, const std::optional<PlanKey>& key) {
  LogEntry entry;
  entry.t = facts_.wall;
  entry.what = what;
  entry.detail = std::move(detail);
  entry.more = std::move(more);
  entry.at = key ? json(FormatClockTime(key->time)) : json();
  log_.push_back(std::move(entry));
  while (log_.size() > kLogEntries) log_.pop_front();
  out_.log_changed = true;
}

void Scheduler::Mark(const PlanKey& key) {
  if (handled_[key.date].insert(key.time).second) out_.state_changed = true;
}

bool Scheduler::Handled(const PlanKey& key) const {
  const auto found = handled_.find(key.date);
  return found != handled_.end() && found->second.count(key.time) > 0;
}

int64_t Scheduler::PlanUnix(const PlanKey& key) const {
  return file_.schedule.zone.ToUnix(key.date, key.time);
}

std::optional<Scheduler::PlanKey> Scheduler::DuePlan() const {
  const Date today = file_.schedule.zone.ToLocal(facts_.wall).date;
  for (const Plan& plan : file_.schedule.plans) {
    const PlanKey key{today, plan.time};
    if (PlanAt(file_.schedule, today, plan.time) != &plan) continue;
    const int64_t at = PlanUnix(key);
    if (at <= facts_.wall && facts_.wall < at + kWindow && at > file_.changed_at && !Handled(key) &&
        clock_skipped_.count(key) == 0) {
      return key;
    }
  }
  return std::nullopt;
}

void Scheduler::SweepMissed() {
  const Date today = file_.schedule.zone.ToLocal(facts_.wall).date;
  for (const Date& date : {AddDays(today, -1), today}) {
    for (const Plan& plan : file_.schedule.plans) {
      const PlanKey key{date, plan.time};
      if (PlanAt(file_.schedule, date, plan.time) != &plan || Handled(key)) continue;
      const int64_t at = PlanUnix(key);
      if (at + kWindow > facts_.wall || at <= file_.changed_at) continue;
      // Logged as skip_clock while the clock was unsafe.
      if (clock_skipped_.count(key) == 0) {
        Log(seen_offline_.count(key) > 0 ? "skip_offline" : "skip_missed", nullptr, nullptr, key);
      }
      Mark(key);
    }
  }
}

void Scheduler::EndWait(const char* battery_word, const char* hold_word) {
  if (!waiting_) return;
  const Waiting wait = *waiting_;
  waiting_.reset();
  out_.state_changed = true;
  if (wait.kind == WaitKind::kBattery) Log(battery_word, std::to_string(facts_.battery), nullptr, wait.key);
  else Log(hold_word, nullptr, nullptr, wait.key);
}

void Scheduler::Decide(const PlanKey& key, const std::string& what, json detail) {
  Log(what, std::move(detail), nullptr, key);
  if (waiting_ && waiting_->key == key) {
    waiting_.reset();
    out_.state_changed = true;
  }
}

void Scheduler::Tick(const Facts& facts) {
  facts_ = facts;
  ticked_ = true;
  if (run_ && !run_->mowing && facts_.mono - run_->last_word > kRunSilence) {
    const PlanKey key = run_->key;
    run_.reset();
    Log("start_failed", "timeout", nullptr, key);
  }
  if (facts_.clock_ok) {
    // Only a safe clock prunes: the marks of other days than today and yesterday fall out.
    const Date today = file_.schedule.zone.ToLocal(facts_.wall).date;
    const Date yesterday = AddDays(today, -1);
    for (auto it = handled_.begin(); it != handled_.end();) {
      if (it->first == today || it->first == yesterday) {
        ++it;
      } else {
        it = handled_.erase(it);
        out_.state_changed = true;
      }
    }
  }
  {
    // Kept only for the last two days of the clock as it stands.
    const Date today = file_.schedule.zone.ToLocal(facts_.wall).date;
    const Date yesterday = AddDays(today, -1);
    const auto stale = [&](const PlanKey& key) { return key.date != today && key.date != yesterday; };
    for (std::set<PlanKey>* keys : {&seen_offline_, &clock_skipped_}) {
      for (auto it = keys->begin(); it != keys->end();) it = stale(*it) ? keys->erase(it) : std::next(it);
    }
  }
  // T0: not released for the platform, or switched off.
  if (!available_ || !file_.schedule.enabled) {
    if (waiting_) {
      waiting_.reset();
      out_.state_changed = true;
    }
    pending_.reset();
    return;
  }
  ResolveWeather();
  if (facts_.clock_ok) SweepMissed();
  const std::optional<PlanKey> due = DuePlan();
  if (due && waiting_ && !pending_) EndWait("skip_battery", "skip_hold");
  if (waiting_ && !pending_ && !run_) {
    const PlanKey key = waiting_->key;
    const Plan* plan = PlanAt(file_.schedule, key.date, key.time);
    if (plan) {
      Ask(key, *plan, true);
    } else {
      waiting_.reset();
      out_.state_changed = true;
      Log("skip_gone", nullptr, nullptr, key);
    }
  }
  if (!due || pending_) return;
  const Plan* plan = PlanAt(file_.schedule, due->date, due->time);
  if (run_) {
    Mark(*due);
    Log("skip_busy", PhaseName(run_->phase), nullptr, due);
    return;
  }
  Ask(*due, *plan, false);
}

void Scheduler::Ask(const PlanKey& key, const Plan& plan, bool from_wait) {
  // T2: with an unsafe clock the date may be wrong, so the mark stays in memory.
  if (!facts_.clock_ok) {
    if (from_wait) {
      Decide(key, "skip_clock", facts_.clock_reason);
    } else if (clock_skipped_.insert(key).second) {
      Log("skip_clock", facts_.clock_reason, nullptr, key);
    }
    return;
  }
  const Verdict verdict = Check(key, plan, from_wait);
  if (verdict == Verdict::kWaitOffline) {
    seen_offline_.insert(key);
    return;
  }
  Mark(key);
  if (verdict != Verdict::kGo) return;
  const Schedule& schedule = file_.schedule;
  if (schedule.skip_forecast || schedule.skip_cold) {
    if (facts_.latitude && facts_.longitude) {
      Pending pending;
      pending.key = key;
      pending.id = next_weather_id_++;
      pending.deadline = facts_.mono + kWeatherDeadline;
      out_.weather = WeatherRequest{pending.id, *facts_.latitude, *facts_.longitude, schedule.forecast_hours};
      pending_ = pending;
      return;
    }
    if (schedule.skip_forecast) Log("weather_unknown", "forecast", nullptr, key);
    if (schedule.skip_cold) Log("weather_unknown", "cold", nullptr, key);
  }
  Start(key, plan);
}

Scheduler::Verdict Scheduler::Check(const PlanKey& key, const Plan& plan, bool from_wait) {
  const Schedule& schedule = file_.schedule;
  const auto decided = [&](const char* what, json detail = nullptr) {
    Decide(key, what, std::move(detail));
    return Verdict::kDecided;
  };
  // T3, T4
  if (PausedAll(schedule, key.date)) return decided("skip_paused");
  const AreaSplit split = SplitAreas(schedule, plan, key.date, facts_.areas);
  if (!split.left) return decided(split.left_without_pause ? "skip_paused" : "skip_no_area");
  // T5
  if (from_wait && plan.end) {
    const Date end_date = *plan.end > plan.time ? key.date : AddDays(key.date, 1);
    if (facts_.wall >= schedule.zone.ToUnix(end_date, *plan.end)) return decided("skip_end");
  }
  // T6: within the window the logic may still come.
  if (!facts_.logic_fresh) return from_wait ? decided("skip_offline") : Verdict::kWaitOffline;
  // T7 to T11
  if (facts_.emergency) return decided("skip_emergency");
  if (facts_.state != kIdle) return decided("skip_busy", facts_.state);
  if (!facts_.docked) return decided("skip_not_docked");
  if (facts_.rain_sensor && facts_.rain_wet && (schedule.skip_rain || facts_.rain_mode != 0)) {
    return decided("skip_rain");
  }
  if (!plan.dark) {
    if (!facts_.latitude || !facts_.longitude) return decided("skip_no_position");
    if (mowbite_time::SunElevation(*facts_.latitude, *facts_.longitude, static_cast<double>(facts_.wall)) <
        mowbite_time::kSunsetElevation) {
      return decided("skip_dark");
    }
  }
  // T12 and T13: wait, up to kWaitLimit after the time of the plan.
  const bool low = facts_.battery < schedule.min_battery;
  if (low || facts_.hold) {
    const WaitKind kind = low ? WaitKind::kBattery : WaitKind::kHold;
    if (facts_.wall >= PlanUnix(key) + kWaitLimit) {
      return low ? decided("skip_battery", std::to_string(facts_.battery)) : decided("skip_hold");
    }
    if (!waiting_ || !(waiting_->key == key)) {
      waiting_ = Waiting{key, kind, false, false};
      out_.state_changed = true;
    }
    if (waiting_->kind != kind) {
      waiting_->kind = kind;
      out_.state_changed = true;
    }
    bool& logged = low ? waiting_->battery_logged : waiting_->hold_logged;
    if (!logged) {
      logged = true;
      if (low) Log("waiting_battery", std::to_string(facts_.battery), nullptr, key);
      else Log("waiting_hold", nullptr, nullptr, key);
    }
    return Verdict::kDecided;
  }
  if (waiting_ && waiting_->key == key) {
    waiting_.reset();
    out_.state_changed = true;
  }
  return Verdict::kGo;
}

void Scheduler::ResolveWeather() {
  if (!pending_ || (!pending_->answer && facts_.mono < pending_->deadline)) return;
  const Pending pending = *pending_;
  pending_.reset();
  const PlanKey key = pending.key;
  const Plan* plan = PlanAt(file_.schedule, key.date, key.time);
  if (!plan) {
    Log("skip_gone", nullptr, nullptr, key);
    return;
  }
  const Schedule& schedule = file_.schedule;
  const std::optional<Weather>& weather = pending.answer;
  const bool fresh = weather && weather->ok && weather->age_s <= kWeatherMaxAge;
  if (schedule.skip_forecast) {
    if (!fresh || !weather->has_rain) {
      Log("weather_unknown", "forecast", nullptr, key);
    } else {
      bool rain = weather->rain_now_mm >= kRainNowMm;
      const size_t hours = std::min(weather->rain_hour_mm.size(), static_cast<size_t>(schedule.forecast_hours));
      for (size_t i = 0; i < hours; ++i) rain = rain || weather->rain_hour_mm[i] >= kRainHourMm;
      if (rain) {
        Decide(key, "skip_forecast", nullptr);
        return;
      }
    }
  }
  if (schedule.skip_cold) {
    if (!fresh || !weather->has_temp) {
      Log("weather_unknown", "cold", nullptr, key);
    } else if (weather->temp_c < schedule.min_temp) {
      Decide(key, "skip_cold", OneDecimal(weather->temp_c));
      return;
    }
  }
  // T15: the rows once more, the answer took time.
  if (!facts_.clock_ok) {
    Decide(key, "skip_clock", facts_.clock_reason);
    return;
  }
  if (Check(key, *plan, true) == Verdict::kGo) Start(key, *plan);
}

void Scheduler::Start(const PlanKey& key, const Plan& plan) {
  const Schedule& schedule = file_.schedule;
  const AreaSplit split = SplitAreas(schedule, plan, key.date, facts_.areas);
  for (const std::string& id : split.paused) Log("paused_area", id, nullptr, key);
  for (const std::string& id : split.unknown) Log("unknown_area", id, nullptr, key);
  RunRequest request;
  request.run_id = FormatDate(key.date) + "T" + FormatClockTime(key.time);
  request.area_ids = plan.areas;
  request.pass_over = split.unknown;
  request.pass_over.insert(request.pass_over.end(), split.paused.begin(), split.paused.end());
  request.restart = plan.fresh;
  request.angle_offset_deg = plan.angle_offset_deg;
  request.dark_allowed = plan.dark;
  request.until_sunset = !plan.dark || plan.sunset;
  if (plan.end) {
    const Date end_date = *plan.end > plan.time ? key.date : AddDays(key.date, 1);
    request.end_in_s = schedule.zone.ToUnix(end_date, *plan.end) - facts_.wall;
  }
  Run run;
  run.key = key;
  run.run_id = request.run_id;
  run.area_ids = plan.areas;
  run.asked = true;
  run.last_word = facts_.mono;
  run_ = run;
  out_.start = std::move(request);
  out_.state_changed = true;
}

void Scheduler::OnWeather(const Weather& weather) {
  if (pending_ && weather.id == pending_->id && !pending_->answer) pending_->answer = weather;
}

void Scheduler::OnStartAnswer(const std::string& run_id, bool accepted, const std::string& reason,
                              const json& value) {
  if (!run_ || run_->run_id != run_id || !run_->asked) return;
  if (!accepted) {
    const PlanKey key = run_->key;
    run_.reset();
    Log("start_refused", reason, value, key);
    return;
  }
  run_->asked = false;
  run_->last_word = facts_.mono;
  DropPaused();
}

void Scheduler::StartFailed(const std::string& run_id, const std::string& detail) {
  if (!run_ || run_->run_id != run_id) return;
  const PlanKey key = run_->key;
  run_.reset();
  Log("start_failed", detail, nullptr, key);
}

void Scheduler::OnRunStatus(const RunStatus& status) {
  if (!run_ || status.run_id != run_->run_id) return;
  run_->asked = false;
  run_->last_word = facts_.mono;
  run_->phase = status.phase;
  const PlanKey key = run_->key;
  if (status.phase == Phase::kMowing && !run_->mowing) {
    run_->mowing = true;
    Log("started", std::to_string(facts_.battery), nullptr, key);
  }
  if (status.phase != Phase::kEnded) return;
  const bool mowed = run_->mowing;
  run_.reset();
  if (!mowed) {
    if (status.end_code == "REFUSED") Log("start_refused", status.detail, nullptr, key);
    else if (status.end_code == "CANCELED") Log("start_canceled", status.detail, nullptr, key);
    else Log("start_failed", status.end_code, nullptr, key);
    return;
  }
  if (status.end_code == "SCHEDULE" && status.detail == "end") Log("stopped_end", nullptr, nullptr, key);
  else if (status.end_code == "SCHEDULE" && status.detail == "pause") Log("stopped_paused", nullptr, nullptr, key);
  else if (status.end_code == "DARK") Log("stopped_dark", nullptr, nullptr, key);
  else if (status.end_code == "RAIN") Log("stopped_rain", nullptr, nullptr, key);
}

void Scheduler::DropPaused() {
  if (!run_ || run_->asked || !ticked_) return;
  const Schedule& schedule = file_.schedule;
  const Date today = schedule.zone.ToLocal(facts_.wall).date;
  if (PausedAll(schedule, today)) {
    out_.drop = DropRequest{run_->run_id, {}};
    return;
  }
  std::vector<std::string> list = run_->area_ids;
  if (list.empty()) {
    for (const Area& area : facts_.areas) list.push_back(area.id);
  }
  DropRequest drop{run_->run_id, {}};
  for (const std::string& id : list) {
    if (PausedArea(schedule, today, id)) drop.area_ids.push_back(id);
  }
  if (!drop.area_ids.empty()) out_.drop = std::move(drop);
}

std::optional<Invalid> Scheduler::SetSchedule(const json& change, int64_t now) {
  std::optional<Invalid> bad = ApplySchedule(change, available_, now, &file_);
  if (bad) return bad;
  out_.file_changed = true;
  DropPaused();
  return std::nullopt;
}

std::optional<Invalid> Scheduler::SetLeftovers(const json& change) {
  std::optional<Invalid> bad = ApplyLeftovers(change, &file_);
  if (!bad) out_.file_changed = true;
  return bad;
}

Output Scheduler::TakeOutput() {
  return std::exchange(out_, Output());
}

std::optional<int64_t> Scheduler::NextStart() const {
  const Schedule& schedule = file_.schedule;
  if (!available_ || !schedule.enabled || !ticked_ || !facts_.clock_ok) return std::nullopt;
  if (waiting_ || pending_ || (run_ && !run_->mowing && run_->phase != Phase::kEnded)) return facts_.wall;
  const Date today = schedule.zone.ToLocal(facts_.wall).date;
  std::optional<int64_t> next;
  for (int64_t day = 0; day <= 7; ++day) {
    const Date date = AddDays(today, day);
    if (PausedAll(schedule, date)) continue;
    for (const Plan& plan : schedule.plans) {
      const PlanKey key{date, plan.time};
      if (PlanAt(schedule, date, plan.time) != &plan || Handled(key) || clock_skipped_.count(key) > 0) continue;
      const int64_t at = PlanUnix(key);
      if (at + kWindow <= facts_.wall || at <= file_.changed_at) continue;
      if (!next || at < *next) next = at;
    }
    // Later days start later.
    if (next) break;
  }
  return next;
}

json Scheduler::StateJson() const {
  json handled = json::object();
  for (const auto& day : handled_) {
    json times = json::array();
    for (int time : day.second) times.push_back(FormatClockTime(time));
    handled[FormatDate(day.first)] = times;
  }
  json waiting = nullptr;
  if (waiting_) {
    waiting = {{"date", FormatDate(waiting_->key.date)},
               {"time", FormatClockTime(waiting_->key.time)},
               {"kind", waiting_->kind == WaitKind::kBattery ? "battery" : "hold"}};
  }
  return {{"schema", kStateSchema}, {"handled", handled}, {"waiting", waiting}};
}

json Scheduler::LogJson() const {
  return {{"schema", kLogSchema}, {"entries", LogEntries(kLogEntries)}};
}

json Scheduler::LogEntries(size_t limit) const {
  json entries = json::array();
  const size_t skip = log_.size() > limit ? log_.size() - limit : 0;
  for (auto it = std::next(log_.begin(), static_cast<std::ptrdiff_t>(skip)); it != log_.end(); ++it) {
    entries.push_back(EntryToJson(*it));
  }
  return entries;
}

}  // namespace mowbite_schedule
