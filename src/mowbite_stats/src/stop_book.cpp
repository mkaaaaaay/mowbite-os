#include "mowbite_stats/stop_book.h"

#include <time.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <set>
#include <utility>

#include "mowbite_api/catalog.h"

namespace mowbite_stats {

namespace {

constexpr int64_t kDay = 86400;
constexpr int64_t kWeek = 7 * kDay;
// Events kept at most to explain the ends still to decide.
constexpr size_t kMaxSeen = 500;

// What the mower does by the state and sub_state of STATE, empty for nothing the book follows.
std::string ActivityOf(const std::string& state, const std::string& sub_state) {
  if (state == "MOWING" || state == "UNDOCKING" || state == "DOCKING") return state;
  if (state == "IDLE" && (sub_state == "DRIVING" || sub_state == "CALIBRATING")) return sub_state;
  return "";
}

// The event an activity ends with when it is done.
bool Done(const std::string& activity, const std::string& type, bool arrived) {
  if (activity == "MOWING") return type == "JOB_COMPLETE";
  if (activity == "UNDOCKING") return type == "UNDOCKED";
  if (activity == "DOCKING") return type == "DOCKED";
  if (activity == "CALIBRATING") return type == "CALIBRATED";
  return type == "DRIVE" && arrived;
}

}  // namespace

int64_t WeekStart(double t) {
  // 1970-01-01 was a Thursday; Monday 00:00 UTC before it is -3 days.
  const auto since_monday = static_cast<int64_t>(std::floor(t)) + 3 * kDay;
  return (since_monday - (((since_monday % kWeek) + kWeek) % kWeek)) - 3 * kDay;
}

std::string WeekName(int64_t start) {
  // The ISO week belongs to the year of its Thursday and counts from the week of the first Thursday.
  const time_t thursday = static_cast<time_t>(start + 3 * kDay);
  struct tm day {};
  gmtime_r(&thursday, &day);
  char name[32];
  std::snprintf(name, sizeof(name), "%04d-W%02d", day.tm_year + 1900, day.tm_yday / 7 + 1);
  return name;
}

StopBook::StopBook(const json& stored) {
  if (!stored.is_object()) return;
  try {
    for (const json& week : stored.value("weeks", json::array())) {
      StopWeek row;
      row.start = week.at("start").get<int64_t>();
      row.stops = week.at("stops").get<int64_t>();
      row.unexplained = week.at("unexplained").get<int64_t>();
      row.standstills = week.at("standstills").get<int64_t>();
      for (const json& reason : week.at("reasons")) {
        row.reasons[reason.at("reason").get<std::string>()] = {reason.at("class").get<std::string>(),
                                                               reason.at("count").get<int64_t>()};
      }
      weeks_.push_back(std::move(row));
    }
    for (const json& item : stored.value("last", json::array())) {
      if (last_.size() >= static_cast<size_t>(mowbite_api::kStopBookLast)) break;
      last_.push_back({item.at("t").get<double>(), item.at("activity").get<std::string>(),
                       item.at("kind").get<std::string>(), item.at("reason").get<std::string>(),
                       item.at("class").get<std::string>(), item.at("explained").get<bool>(),
                       item.at("incident").get<std::string>()});
    }
  } catch (const std::exception&) {
    weeks_.clear();
    last_.clear();
  }
  std::sort(weeks_.begin(), weeks_.end(), [](const StopWeek& a, const StopWeek& b) { return a.start < b.start; });
}

void StopBook::OnEvent(const json& event, double mono, double wall) {
  if (!event.is_object()) return;
  Seen seen{mono, TextOf(event, "id"), TextOf(event, "type"), TextOf(event, "reason")};
  // INCIDENT only points at its trigger; an event sent again with its id counts once.
  if (seen.type.empty() || seen.type == "INCIDENT") return;
  const auto same = [&](const Seen& other) { return other.id == seen.id; };
  if (!seen.id.empty() && std::any_of(seen_.begin(), seen_.end(), same)) return;
  static const std::set<std::string> commands = {"MANUAL", "SCHEDULE", "TEST"};
  seen.commanded = !TextOf(event, "client").empty() || commands.count(seen.reason) != 0;
  // A field of another type is no true: value() would throw for it.
  const auto is_true = [&](const char* key) {
    const auto it = event.find(key);
    return it != event.end() && *it == true;
  };
  seen.arrived = TextOf(event, "phase") == "end" && is_true("arrived");
  const auto bits = event.find("reason");
  const bool began = is_true("emergency");
  if (seen.type == "EMERGENCY" && began && bits != event.end() && bits->is_number_integer()) {
    seen.bits = bits->get<int64_t>();
  }
  seen_.push_back(std::move(seen));
  if (seen_.size() > kMaxSeen) seen_.pop_front();
  const Seen& added = seen_.back();
  if (added.type == "STATE") {
    const std::string next = ActivityOf(TextOf(event, "state"), TextOf(event, "sub_state"));
    if (next == activity_) return;
    if (!activity_.empty()) ends_.push_back({activity_, mono, wall});
    activity_ = next;
    anchor_.reset();
  } else if (added.type == "UNEXPECTED_RESTART") {
    // state_before names a drive or calibration drive of IDLE by its sub_state.
    const std::string before = TextOf(event, "state_before");
    std::string was = ActivityOf(before, "");
    if (was.empty()) was = ActivityOf("IDLE", before);
    if (!was.empty()) ends_.push_back({was, mono, wall});
  }
}

bool StopBook::Tick(double mono) {
  bool added = false;
  while (!ends_.empty() && mono >= ends_.front().mono + kStopExplainS) {
    added = Decide(ends_.front()) || added;
    ends_.pop_front();
  }
  // Events before the window of the next end, or of an end now, explain nothing more.
  const double keep = (ends_.empty() ? mono : ends_.front().mono) - kStopExplainS;
  while (!seen_.empty() && seen_.front().mono < keep) seen_.pop_front();
  return added;
}

bool StopBook::Decide(const End& end) {
  // The events of the window, the closest to the end first.
  std::vector<const Seen*> window;
  for (const Seen& seen : seen_) {
    if (std::abs(seen.mono - end.mono) <= kStopExplainS) window.push_back(&seen);
  }
  std::stable_sort(window.begin(), window.end(), [&](const Seen* a, const Seen* b) {
    return std::abs(a->mono - end.mono) < std::abs(b->mono - end.mono);
  });
  for (const Seen* seen : window) {
    if (seen->commanded || Done(end.activity, seen->type, seen->arrived)) return false;
  }
  StopEntry entry{end.wall, end.activity, "stop", "", mowbite_api::kSymptom, false, ""};
  // A symptom as the best hint, a reason that only points at an emergency last.
  std::string hint, weak;
  for (const Seen* seen : window) {
    std::string reason, cls;
    if (seen->type == "EMERGENCY") {
      const mowbite_api::EmergencyBit* bit = mowbite_api::EmergencyReasonBit(static_cast<int>(seen->bits));
      if (!bit) continue;
      reason = "EMERGENCY/" + bit->name;
      cls = bit->cls;
    } else if (!seen->reason.empty()) {
      reason = seen->type + "/" + seen->reason;
      cls = mowbite_api::ReasonClass(seen->type, seen->reason);
    } else {
      continue;
    }
    if (cls == mowbite_api::kCause) {
      entry.reason = reason;
      entry.cls = cls;
      entry.explained = true;
      break;
    }
    std::string& slot = seen->reason == "EMERGENCY" ? weak : hint;
    if (slot.empty()) slot = reason;
  }
  if (!entry.explained) entry.reason = !hint.empty() ? hint : !weak.empty() ? weak : "UNKNOWN";
  Add(std::move(entry));
  return true;
}

bool StopBook::OnPose(bool paused, bool pose, double x, double y, double mono, double wall) {
  // A calibration drive stands on purpose.
  const bool busy = !activity_.empty() && activity_ != "CALIBRATING";
  if (!busy || paused || !pose) {
    anchor_.reset();
    return false;
  }
  if (!anchor_ || std::hypot(x - anchor_->x, y - anchor_->y) >= kStandstillM) {
    anchor_ = Anchor{mono, x, y};
    return false;
  }
  if (anchor_->booked || mono - anchor_->mono < kStandstillS) return false;
  anchor_->booked = true;
  Add({wall, activity_, "standstill", "STANDSTILL", mowbite_api::kSymptom, false, ""});
  return true;
}

bool StopBook::OnMark(const std::string& incident, double wall) {
  // A stop before it, or a mark that only grew the incident of an earlier one.
  const bool known = std::any_of(last_.begin(), last_.end(), [&](const StopEntry& entry) {
    const bool stop_before = entry.kind == "stop" && entry.t >= wall - kStopMarkS;
    return stop_before || (entry.kind == "mark" && entry.incident == incident);
  });
  if (known) return false;
  Add({wall, activity_.empty() ? "IDLE" : activity_, "mark", "MARK", mowbite_api::kSymptom, false, incident});
  return true;
}

void StopBook::Add(StopEntry entry) {
  if (entry.kind == "stop" && !entry.explained) {
    if (incidents_.open) entry.incident = incidents_.open(entry.reason);
  } else if (entry.kind != "mark" && incidents_.collecting) {
    entry.incident = incidents_.collecting();
  }
  const int64_t start = WeekStart(entry.t);
  auto week = std::find_if(weeks_.begin(), weeks_.end(), [&](const StopWeek& w) { return w.start >= start; });
  if (week == weeks_.end() || week->start != start) week = weeks_.insert(week, StopWeek{start, 0, 0, 0, {}});
  if (entry.kind == "standstill") {
    week->standstills++;
  } else {
    week->stops += entry.kind == "stop" ? 1 : 0;
    week->unexplained += entry.explained ? 0 : 1;
  }
  const bool room = week->reasons.size() < kStopBookMaxReasons || week->reasons.count(entry.reason) != 0;
  StopCount& count = week->reasons[room ? entry.reason : "OTHER"];
  if (count.cls.empty()) count.cls = room ? entry.cls : mowbite_api::kSymptom;
  count.count++;
  last_.push_front(std::move(entry));
  if (last_.size() > static_cast<size_t>(mowbite_api::kStopBookLast)) last_.pop_back();
  Prune(last_.front().t);
}

int64_t OldestWeek(double now) {
  return WeekStart(now) - (mowbite_api::kStopBookWeeks - 1) * kWeek;
}

void StopBook::Prune(double now) {
  const int64_t oldest = OldestWeek(now);
  while (!weeks_.empty() && weeks_.front().start < oldest) weeks_.pop_front();
}

json StopBook::Json(double now, const std::map<int64_t, double>* blade_h) const {
  const int64_t oldest = OldestWeek(now);
  json weeks = json::array();
  for (const StopWeek& week : weeks_) {
    if (week.start < oldest) continue;
    std::vector<std::pair<std::string, const StopCount*>> reasons;
    for (const auto& [reason, count] : week.reasons) reasons.emplace_back(reason, &count);
    std::stable_sort(reasons.begin(), reasons.end(),
                     [](const auto& a, const auto& b) { return a.second->count > b.second->count; });
    json rows = json::array();
    for (const auto& [reason, count] : reasons) {
      rows.push_back({{"reason", reason}, {"class", count->cls}, {"count", count->count}});
    }
    json row = {{"week", WeekName(week.start)},     {"start", week.start},
                {"stops", week.stops},              {"unexplained", week.unexplained},
                {"standstills", week.standstills}};
    if (blade_h) {
      const auto found = blade_h->find(week.start);
      row["blade_h"] = found == blade_h->end() ? 0.0 : found->second;
    }
    row["reasons"] = std::move(rows);
    weeks.push_back(std::move(row));
  }
  json last = json::array();
  for (const StopEntry& entry : last_) {
    last.push_back({{"t", std::llround(entry.t)},
                    {"activity", entry.activity},
                    {"kind", entry.kind},
                    {"reason", entry.reason},
                    {"class", entry.cls},
                    {"explained", entry.explained},
                    {"incident", entry.incident}});
  }
  json out = json::object();
  out["schema"] = 1;
  out["weeks"] = std::move(weeks);
  out["last"] = std::move(last);
  return out;
}

}  // namespace mowbite_stats
