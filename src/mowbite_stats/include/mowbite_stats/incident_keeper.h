#ifndef MOWBITE_STATS_INCIDENT_KEEPER_H
#define MOWBITE_STATS_INCIDENT_KEEPER_H

#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mowbite_stats/counter.h"

namespace mowbite_stats {

// Limits of the incidents in journal_dir/incidents; sizes and days come from the platform config.
struct IncidentLimits {
  // Kept before the first trigger and after the last one.
  double before_s = 60;
  double after_s = 15;
  // Later triggers grow a window up to this long after its first.
  double max_s = 600;
  // A kind that is no safety opens at most one incident in kind_gap_s and kind_per_day in 24 h.
  double kind_gap_s = 60;
  int kind_per_day = 30;
  size_t max_count = 1000;
  double max_mb = 512;
  double max_days = 30;
  double max_days_safety = 90;
};

// What sets off an incident.
struct Trigger {
  // EMERGENCY, an event *_FAILED, POSITION_JUMP, UNEXPECTED_RESTART, FOLLOWER_MISSING, BLADE_REFUSED, CRASH (a run
  // of the journal without its end), MARK (incidents.mark).
  std::string kind;
  // The reason of the event, the emergency bits or the note of a mark.
  std::string reason;
  // Never throttled and removed last.
  bool safety = false;
  // Monotonic ns, and the wall clock as ISO UTC for the id and the manifest.
  int64_t t_ns = 0;
  std::string utc;
  // CRASH: the run that ended without its end.
  std::string run;
  // Kept before it instead of before_s when not negative (MARK: the last 5 min).
  double before_s = -1;
  json detail;
};

// The trigger an event of events/json sets off, if any; emergencies come from EmergencyTrigger.
std::optional<Trigger> TriggerOfEvent(const json& event, int64_t t_ns, const std::string& utc);
// Every bit of emergency_reason in hw/status that was not set before opens or grows an incident, also while others
// stay set and whether the logic tells of it or not.
std::optional<Trigger> EmergencyTrigger(uint32_t before, uint32_t now, int64_t t_ns, const std::string& utc);

enum class IncidentState { kCollecting, kSealing };

struct Incident {
  std::string id;
  IncidentState state = IncidentState::kCollecting;
  // The window on the monotonic clock; start_ns is the first trigger.
  int64_t start_ns = 0;
  int64_t from_ns = 0;
  int64_t to_ns = 0;
  bool safety = false;
  // The first kMaxTriggers triggers, and how many more came.
  std::vector<Trigger> triggers;
  uint64_t more = 0;
};

constexpr size_t kMaxTriggers = 100;

// Decides which trigger opens an incident or grows the open one, and when a window is over. The node links the
// segments, writes the manifest and seals; no clock, no files.
class IncidentKeeper {
 public:
  explicit IncidentKeeper(IncidentLimits limits);

  // An id already on the disk, so a new one differs.
  void Known(const std::string& id);
  // The id of the incident the trigger opened or grew, empty when throttled.
  std::string OnTrigger(const Trigger& trigger);
  // The incidents whose window is over at now_ns go to sealing: their ids.
  std::vector<std::string> OnTick(int64_t now_ns);
  // The node wrote the sealed incident; it is no longer kept here.
  void Sealed(const std::string& id);

  // The collecting incident and those waiting to be sealed.
  const Incident* Find(const std::string& id) const;
  std::vector<std::string> OpenIds() const;
  const std::optional<Incident>& collecting() const { return collecting_; }
  // Throttled triggers by kind since the start.
  const std::map<std::string, uint64_t>& throttled() const { return throttled_; }

 private:
  bool Throttled(const Trigger& trigger) const;
  std::string NewId(const std::string& utc);

  IncidentLimits limits_;
  void ToSealing();

  std::optional<Incident> collecting_;
  std::map<std::string, Incident> sealing_;
  // Went to sealing in OnTrigger, reported by the next OnTick.
  std::vector<std::string> to_seal_;
  std::set<std::string> ids_;
  // Openings of the last 24 h by kind (monotonic ns).
  std::map<std::string, std::deque<int64_t>> opened_;
  std::map<std::string, uint64_t> throttled_;
};

// A file of an incident on the disk: hard links of one segment share the inode.
struct StoredFile {
  uint64_t inode = 0;
  uint64_t bytes = 0;
};

struct StoredIncident {
  std::string id;
  // Unix s.
  double created_s = 0;
  bool safety = false;
  // Still collecting or sealing: never removed.
  bool open = false;
  std::vector<StoredFile> files;
};

// Expired incidents (max_days, max_days_safety), then while there are more than max_count or their files take more
// than max_mb (each inode counted once) the oldest that are no safety, then the oldest safety ones.
std::vector<std::string> IncidentsToRemove(const std::vector<StoredIncident>& stored, double now_s,
                                           const IncidentLimits& limits);

}  // namespace mowbite_stats

#endif  // MOWBITE_STATS_INCIDENT_KEEPER_H
