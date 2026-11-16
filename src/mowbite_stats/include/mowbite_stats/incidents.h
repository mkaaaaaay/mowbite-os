#ifndef MOWBITE_STATS_INCIDENTS_H
#define MOWBITE_STATS_INCIDENTS_H

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mowbite_journal/run_index.h"
#include "mowbite_stats/incident_keeper.h"
#include "mowbite_stats/incident_store.h"
#include "mowbite_stats/stats_rpc.h"

namespace mowbite_stats {

struct IncidentsConfig {
  // Empty: no journal, no incidents.
  std::string journal_dir;
  IncidentLimits limits;
  // The limits of the journal itself, for journal.status.
  double journal_max_mb = 2048;
  double journal_max_days = 14;
  double reserve_mb = 1024;
  // Of this boot; segments of runs with another one count only for CRASH.
  std::string boot_id;
  // The note of the launch about a driver that ended outside the shutdown (driver_exit.json in data_dir): each such
  // end is a CRASH, also of a driver without a journal of its own. Empty: none.
  std::string driver_note;
};

// The boot id of the kernel, as the journal writes it into the head of every segment.
std::string BootId();

// Seconds between the passes of Tick.
constexpr double kIncidentRunsEvery = 10.0;
constexpr double kIncidentRemoveEvery = 60.0;
// Runs found at the first pass without their end count as crashed only when written to this shortly before.
constexpr double kIncidentCrashBefore = 600.0;
// Log lines at WARN and above kept for the manifest: the last kIncidentLogSeconds, at most kIncidentLogLines.
constexpr double kIncidentLogSeconds = 120.0;
constexpr size_t kIncidentLogLines = 500;

// The incidents of the journal (FP23 B2a): triggers from events, emergency bits and crashed runs open windows
// (IncidentKeeper), their segments are linked while the window is open, sealed on a worker thread at the idle I/O
// class, kept within the limits and served over journal.status and incidents.*. All calls from one thread; the
// worker only seals and prepares downloads. Returns the INCIDENT events the node publishes, as details of the event.
class Incidents {
 public:
  using Notices = std::vector<json>;

  Incidents(IncidentsConfig config, std::function<int64_t(const std::string&)> free_bytes,
            std::function<void(const std::string&)> warn);
  ~Incidents();
  Incidents(const Incidents&) = delete;
  Incidents& operator=(const Incidents&) = delete;

  static const std::vector<std::string>& Methods();

  // An event of events/json, the emergency bits of hw/status, the state for the manifest, a log line.
  Notices OnEvent(const json& event, int64_t now_ns, double wall_s);
  Notices OnEmergency(uint32_t bits, int64_t now_ns, double wall_s);
  void OnRobot(json robot);
  // journal_enabled of the logic: journal_dir/disabled turns the journal off from the next start of each process.
  void SetJournalEnabled(bool enabled);
  void OnLog(int64_t now_ns, json line);
  // Once a second: windows that are over, links, crashed runs, sealing, the limits.
  Notices Tick(int64_t now_ns, double wall_s);
  // One of Methods.
  RpcOutcome Handle(const std::string& method, const json& params, int64_t now_ns, double wall_s, Notices* notices);

  // For tests: wait until the worker has nothing left.
  void Drain();

 private:
  class Worker;

  bool Enabled() const;
  // Why no incident is kept right now, empty while they are.
  std::string Problem() const;
  // opened: the id of the incident the trigger opened or grew, empty when throttled.
  Notices Open(const Trigger& trigger, double wall_s, std::string* opened = nullptr);
  void RefreshRuns(int64_t now_ns, double wall_s, Notices* notices);
  void LinkOpen();
  void FindCrashes(int64_t now_ns, double wall_s, Notices* notices);
  void RemoveOld(double wall_s);
  void TakeDone();
  json Status() const;
  json List() const;
  RpcOutcome Get(const json& params) const;
  RpcOutcome Download(const json& params);
  RpcOutcome Mark(const json& params, int64_t now_ns, double wall_s, Notices* notices);
  RpcOutcome Delete(const json& params);
  std::string StateOf(const std::string& id, const json& manifest) const;

  IncidentsConfig config_;
  std::function<int64_t(const std::string&)> free_bytes_;
  std::function<void(const std::string&)> warn_;
  IncidentKeeper keeper_;
  IncidentStore store_;
  mowbite_journal::RunIndex index_;
  std::vector<mowbite_journal::RunInfo> runs_;
  std::map<std::string, IncidentContext> contexts_;
  // Incidents handed to the worker for sealing.
  std::set<std::string> sealing_;
  // Downloads being prepared, and why one could not be.
  std::set<std::pair<std::string, bool>> preparing_;
  std::map<std::pair<std::string, bool>, std::string> prepare_failed_;
  json robot_ = json::object();
  std::deque<std::pair<int64_t, json>> log_;
  uint32_t bits_ = 0;
  // Runs whose end was looked at for a crash and the end of the driver in driver_note, kept in incidents/seen.json.
  std::set<std::string> seen_;
  bool first_pass_ = true;
  double last_runs_ = -1e18;
  double last_remove_ = -1e18;
  std::unique_ptr<Worker> worker_;
};

}  // namespace mowbite_stats

#endif  // MOWBITE_STATS_INCIDENTS_H
