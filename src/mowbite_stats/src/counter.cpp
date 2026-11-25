#include "mowbite_stats/counter.h"

#include <algorithm>
#include <cmath>
#include <ctime>

#include "mowbite_api/catalog.h"

namespace mowbite_stats {

namespace {

constexpr char kIdle[] = "IDLE";

int64_t HourOf(double wall) {
  return static_cast<int64_t>(std::floor(wall / 3600.0)) * 3600;
}

double Number(const json& j, const char* key) {
  const auto it = j.find(key);
  return it != j.end() && it->is_number() && std::isfinite(it->get<double>()) && it->get<double>() >= 0
             ? it->get<double>()
             : 0.0;
}

Amounts AmountsFrom(const json& j) {
  if (!j.is_object()) return {};
  return {Number(j, "blade_s"), Number(j, "mowed_m"), Number(j, "driven_m"),
          static_cast<int64_t>(Number(j, "runs"))};
}

double Decimetres(double metres) {
  return std::round(metres * 10.0) / 10.0;
}

// The amounts as stats.get and the files show them: whole seconds, decimetres.
json AmountsJson(const Amounts& a) {
  return {{"blade_s", std::llround(a.blade_s)},
          {"mowed_m", Decimetres(a.mowed_m)},
          {"driven_m", Decimetres(a.driven_m)},
          {"runs", a.runs}};
}

void AddUp(Amounts* to, const Amounts& a) {
  to->blade_s += a.blade_s;
  to->mowed_m += a.mowed_m;
  to->driven_m += a.driven_m;
  to->runs += a.runs;
}

// 1 January of the year of now, UTC, less a day: a local year begins within it in every time zone.
int64_t YearStart(double now) {
  const std::time_t t = static_cast<std::time_t>(now);
  std::tm utc{};
  gmtime_r(&t, &utc);
  std::tm first{};
  first.tm_year = utc.tm_year;
  first.tm_mday = 1;
  return static_cast<int64_t>(timegm(&first)) - 86400;
}

}  // namespace

const std::vector<Part>& DefaultParts() {
  static const std::vector<Part> parts = {{"blade", 80, 0, 0}, {"clean", 200, 0, 0}};
  return parts;
}

Counter::Counter(const json& state, const json& hours, const json& parts, double now) : parts_(DefaultParts()) {
  if (state.is_object() && state.value("schema", 0) == 1 && state.contains("since")) {
    since_ = Number(state, "since");
    total_ = AmountsFrom(state.value("total", json()));
    const json hour = state.value("hour", json());
    hour_.t = static_cast<int64_t>(Number(hour, "t"));
    hour_.amounts = AmountsFrom(hour);
  } else {
    since_ = std::floor(now);
    hour_.t = HourOf(now);
  }
  // Without a set clock since is 0 and the hour going on 0 until the first sample with one.
  if (!WallSet(since_)) since_ = WallSet(now) ? std::floor(now) : 0;
  if (!WallSet(hour_.t)) hour_.t = 0;
  if (hours.is_array()) {
    for (const json& row : hours) {
      if (!row.is_array() || row.size() != 5) continue;
      bool numbers = true;
      for (const json& v : row) numbers = numbers && v.is_number() && v.get<double>() >= 0;
      if (!numbers || !WallSet(row[0].get<double>())) continue;
      hours_.push_back({row[0].get<int64_t>(),
                        {row[1].get<double>(), row[2].get<double>(), row[3].get<double>(), row[4].get<int64_t>()}});
    }
    while (hours_.size() > kMaxHours) hours_.pop_front();
  }
  if (parts.is_array()) {
    for (const json& stored : parts) {
      if (!stored.is_object()) continue;
      for (Part& part : parts_) {
        if (stored.value("key", "") != part.key) continue;
        if (Number(stored, "due_h") >= 1) part.due_h = Number(stored, "due_h");
        part.base_blade_s = std::min(Number(stored, "base_blade_s"), total_.blade_s);
        part.done_at = static_cast<int64_t>(Number(stored, "done_at"));
      }
    }
  }
}

std::string TextOf(const json& object, const char* key) {
  const auto found = object.find(key);
  return found != object.end() && found->is_string() ? found->get<std::string>() : std::string();
}

Changed Counter::Add(const Sample& sample) {
  Changed changed;
  // What comes without a set clock goes into the hour 0, and from there into the first hour with a set clock.
  const int64_t hour = WallSet(sample.wall) ? HourOf(sample.wall) : 0;
  if (hour != hour_.t) {
    if (hour_.t != 0) {
      changed.hour_closed = CloseHour();
      hour_.amounts = {};
    }
    hour_.t = hour;
  }
  if (hour != 0 && since_ == 0) since_ = std::floor(sample.wall);
  if (last_) {
    const Sample& last = *last_;
    const double gap = sample.mono - last.mono;
    Amounts add;
    if (gap > 0 && gap <= kMaxGap) {
      if (last.blade) add.blade_s = gap;
      if (last.pose && sample.pose && last.state != kIdle) {
        const double step = std::hypot(sample.x - last.x, sample.y - last.y);
        if (step <= kMaxStep) (last.blade ? add.mowed_m : add.driven_m) = step;
      }
    }
    if (last.state == kIdle && (sample.state == "UNDOCKING" || sample.state == "MOWING")) add.runs = 1;
    AddUp(&hour_.amounts, add);
    AddUp(&total_, add);
  }
  last_ = sample;
  return changed;
}

bool Counter::CloseHour() {
  if (hour_.amounts.Empty()) return false;
  hours_.push_back(hour_);
  while (hours_.size() > kMaxHours) hours_.pop_front();
  return true;
}

json Counter::StateJson() const {
  json hour = AmountsJson(hour_.amounts);
  hour["t"] = hour_.t;
  return {{"schema", 1}, {"since", std::llround(since_)}, {"total", AmountsJson(total_)}, {"hour", hour}};
}

json Counter::HoursJson() const {
  json rows = json::array();
  for (const Hour& h : hours_) {
    const json a = AmountsJson(h.amounts);
    rows.push_back({h.t, a["blade_s"], a["mowed_m"], a["driven_m"], a["runs"]});
  }
  return rows;
}

json Counter::PartsJson() const {
  json out = json::array();
  for (const Part& part : parts_) {
    out.push_back({{"key", part.key},
                   {"due_h", part.due_h},
                   {"base_blade_s", std::llround(part.base_blade_s)},
                   {"done_at", part.done_at}});
  }
  return out;
}

std::vector<Hour> Counter::HoursSince(int64_t from) const {
  std::vector<Hour> picked;
  for (const Hour& h : hours_) {
    if (h.t >= from) picked.push_back(h);
  }
  if (!hour_.amounts.Empty() && hour_.t != 0 && hour_.t >= from) picked.push_back(hour_);
  return picked;
}

json Counter::Get(std::optional<int64_t> from, double now) const {
  const int64_t start = from.value_or(YearStart(now));
  std::vector<Hour> picked = HoursSince(start);
  // Cut to the newest; from then says where the hours begin.
  int64_t begins = start;
  // About 0.7 MB of JSON at the most, well below what MQTT carries.
  const size_t most = mowbite_api::kStatsMaxAnswerHours;
  if (picked.size() > most) {
    picked.erase(picked.begin(), picked.end() - static_cast<std::ptrdiff_t>(most));
    begins = picked.front().t;
  }
  json rows = json::array();
  for (const Hour& h : picked) {
    json row = {{"t", h.t}};
    row.update(AmountsJson(h.amounts));
    rows.push_back(row);
  }
  return {{"schema", 1},         {"since", std::llround(since_)}, {"from", begins}, {"total", AmountsJson(total_)},
          {"hours", rows},      {"parts", PartsJson()}};
}

bool Counter::PartDone(const std::string& key, double now) {
  for (Part& part : parts_) {
    if (part.key != key) continue;
    part.base_blade_s = total_.blade_s;
    part.done_at = static_cast<int64_t>(now);
    return true;
  }
  return false;
}

bool Counter::PartDue(const std::string& key, double hours) {
  for (Part& part : parts_) {
    if (part.key != key) continue;
    part.due_h = hours;
    return true;
  }
  return false;
}

double Counter::UsedHours(const Part& part) const {
  return std::max(0.0, total_.blade_s - part.base_blade_s) / 3600.0;
}

}  // namespace mowbite_stats
