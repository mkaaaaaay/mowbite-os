#include "mowbite_stats/incident_keeper.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "mowbite_api/catalog.h"

namespace mowbite_stats {

namespace {

constexpr int64_t kNsPerS = 1000000000;
constexpr int64_t kDayNs = 86400 * kNsPerS;

int64_t Ns(double s) {
  return static_cast<int64_t>(std::llround(s * 1e9));
}

std::string Text(const json& object, const char* key) {
  const auto it = object.find(key);
  return it != object.end() && it->is_string() ? it->get<std::string>() : std::string();
}

// The names of emergency bits as in x-emergency-bits.
std::string BitNames(uint64_t bits) {
  std::string names;
  for (const auto& bit : mowbite_api::EmergencyBits()) {
    if (bit.bit < 0 || bit.bit >= 64 || (bits & (uint64_t{1} << bit.bit)) == 0) continue;
    if (!names.empty()) names += ", ";
    names += bit.name;
  }
  return names;
}

bool EndsWith(const std::string& text, const std::string& end) {
  return text.size() >= end.size() && text.compare(text.size() - end.size(), end.size(), end) == 0;
}

}  // namespace

std::optional<Trigger> TriggerOfEvent(const json& event, int64_t t_ns, const std::string& utc) {
  if (!event.is_object()) return std::nullopt;
  const std::string type = Text(event, "type");
  const std::string reason = Text(event, "reason");
  Trigger trigger;
  trigger.t_ns = t_ns;
  trigger.utc = utc;
  trigger.reason = reason;
  trigger.detail = event;
  if (type == "UNEXPECTED_RESTART") {
    trigger.kind = type;
    trigger.safety = true;
  } else if (EndsWith(type, "_FAILED") || type == "POSITION_JUMP") {
    trigger.kind = type;
  } else if (reason == "FOLLOWER_MISSING" || reason == "BLADE_REFUSED") {
    trigger.kind = reason;
  } else {
    return std::nullopt;
  }
  return trigger;
}

std::optional<Trigger> EmergencyTrigger(uint32_t before, uint32_t now, int64_t t_ns, const std::string& utc) {
  const uint32_t rising = now & ~before;
  if (rising == 0) return std::nullopt;
  Trigger trigger;
  trigger.kind = "EMERGENCY";
  trigger.reason = BitNames(rising);
  trigger.safety = true;
  trigger.t_ns = t_ns;
  trigger.utc = utc;
  trigger.detail = json{{"bits", now}, {"rising", rising}};
  return trigger;
}

IncidentKeeper::IncidentKeeper(IncidentLimits limits) : limits_(limits) {}

void IncidentKeeper::Known(const std::string& id) {
  ids_.insert(id);
}

std::string IncidentKeeper::OnTrigger(const Trigger& trigger) {
  if (collecting_ && trigger.t_ns > collecting_->to_ns) ToSealing();
  if (collecting_) {
    Incident& incident = *collecting_;
    incident.to_ns = std::min(std::max(incident.to_ns, trigger.t_ns + Ns(limits_.after_s)),
                              incident.start_ns + Ns(limits_.max_s));
    incident.safety = incident.safety || trigger.safety;
    if (incident.triggers.size() < kMaxTriggers) {
      incident.triggers.push_back(trigger);
    } else {
      ++incident.more;
    }
    return incident.id;
  }
  std::deque<int64_t>& opened = opened_[trigger.kind];
  while (!opened.empty() && opened.front() <= trigger.t_ns - kDayNs) opened.pop_front();
  if (Throttled(trigger)) {
    ++throttled_[trigger.kind];
    return std::string();
  }
  opened.push_back(trigger.t_ns);
  Incident incident;
  incident.id = NewId(trigger.utc);
  incident.start_ns = trigger.t_ns;
  incident.from_ns = trigger.t_ns - Ns(trigger.before_s >= 0 ? trigger.before_s : limits_.before_s);
  incident.to_ns = trigger.t_ns + Ns(limits_.after_s);
  incident.safety = trigger.safety;
  incident.triggers.push_back(trigger);
  collecting_ = std::move(incident);
  return collecting_->id;
}

std::vector<std::string> IncidentKeeper::OnTick(int64_t now_ns) {
  if (collecting_ && now_ns > collecting_->to_ns) ToSealing();
  return std::exchange(to_seal_, {});
}

void IncidentKeeper::Sealed(const std::string& id) {
  sealing_.erase(id);
}

const Incident* IncidentKeeper::Find(const std::string& id) const {
  if (collecting_ && collecting_->id == id) return &*collecting_;
  const auto it = sealing_.find(id);
  return it == sealing_.end() ? nullptr : &it->second;
}

std::vector<std::string> IncidentKeeper::OpenIds() const {
  std::vector<std::string> ids;
  for (const auto& item : sealing_) ids.push_back(item.first);
  if (collecting_) ids.push_back(collecting_->id);
  return ids;
}

void IncidentKeeper::ToSealing() {
  collecting_->state = IncidentState::kSealing;
  to_seal_.push_back(collecting_->id);
  sealing_.emplace(collecting_->id, std::move(*collecting_));
  collecting_.reset();
}

bool IncidentKeeper::Throttled(const Trigger& trigger) const {
  if (trigger.safety) return false;
  const auto it = opened_.find(trigger.kind);
  if (it == opened_.end()) return false;
  // Only openings of the last 24 h are kept.
  const std::deque<int64_t>& times = it->second;
  if (static_cast<int64_t>(times.size()) >= limits_.kind_per_day) return true;
  return !times.empty() && trigger.t_ns - times.back() < Ns(limits_.kind_gap_s);
}

std::string IncidentKeeper::NewId(const std::string& utc) {
  // 2026-11-15T12:13:14Z → 20261115T121314Z
  std::string base;
  for (const char c : utc) {
    if ((c >= '0' && c <= '9') || c == 'T' || c == 'Z') base += c;
  }
  if (base.empty()) base = "incident";
  std::string id = base;
  for (int n = 2; ids_.count(id) != 0; ++n) id = base + "-" + std::to_string(n);
  ids_.insert(id);
  return id;
}

uint64_t UniqueBytes(const std::vector<StoredFile>& files) {
  std::map<uint64_t, uint64_t> inodes;
  for (const StoredFile& file : files) inodes[file.inode] = file.bytes;
  uint64_t bytes = 0;
  for (const auto& item : inodes) bytes += item.second;
  return bytes;
}

std::vector<std::string> IncidentsToRemove(const std::vector<StoredIncident>& stored, double now_s,
                                           const IncidentLimits& limits) {
  std::vector<std::string> removed;
  std::vector<const StoredIncident*> kept;
  for (const StoredIncident& incident : stored) {
    const double days = (now_s - incident.created_s) / 86400.0;
    if (!incident.open && days > (incident.safety ? limits.max_days_safety : limits.max_days)) {
      removed.push_back(incident.id);
    } else {
      kept.push_back(&incident);
    }
  }
  // Each inode counts once, as long as an incident still holds it.
  std::map<uint64_t, std::pair<uint64_t, int>> inodes;
  double bytes = 0;
  for (const StoredIncident* incident : kept) {
    for (const StoredFile& file : incident->files) {
      auto& [size, holders] = inodes[file.inode];
      if (holders++ == 0) {
        size = file.bytes;
        bytes += static_cast<double>(file.bytes);
      }
    }
  }
  std::vector<const StoredIncident*> candidates;
  for (const StoredIncident* incident : kept) {
    if (!incident->open) candidates.push_back(incident);
  }
  std::stable_sort(candidates.begin(), candidates.end(), [](const StoredIncident* a, const StoredIncident* b) {
    if (a->safety != b->safety) return !a->safety;
    return a->created_s < b->created_s;
  });
  size_t count = kept.size();
  for (const StoredIncident* incident : candidates) {
    if (count <= limits.max_count && bytes <= limits.max_mb * 1e6) break;
    removed.push_back(incident->id);
    --count;
    for (const StoredFile& file : incident->files) {
      auto& [size, holders] = inodes[file.inode];
      if (--holders == 0) bytes -= static_cast<double>(size);
    }
  }
  return removed;
}

}  // namespace mowbite_stats
