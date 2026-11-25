#ifndef MOWBITE_STATS_STOP_BOOK_H
#define MOWBITE_STATS_STOP_BOOK_H

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_stats/counter.h"

namespace mowbite_stats {

// Events this long before and after the end of an activity explain it.
constexpr double kStopExplainS = 2.0;
// Busy, not paused and less than kStandstillM from where it stood kStandstillS ago: a standstill.
constexpr double kStandstillS = 60.0;
constexpr double kStandstillM = 0.05;
// A mark without a stop in this long before it counts as an unexplained stop.
constexpr double kStopMarkS = 300.0;
// Reasons kept per week; more go under OTHER.
constexpr size_t kStopBookMaxReasons = 500;

struct StopEntry {
  double t = 0;  // unix s
  // One of kStopActivities.
  std::string activity;
  // stop, standstill or mark.
  std::string kind;
  // TYPE/REASON, EMERGENCY/BIT, UNKNOWN, STANDSTILL or MARK.
  std::string reason;
  std::string cls;
  bool explained = false;
  std::string incident;
};

struct StopCount {
  std::string cls;
  int64_t count = 0;
};

struct StopWeek {
  int64_t start = 0;  // Monday 00:00 UTC, unix s
  int64_t stops = 0;
  int64_t unexplained = 0;
  int64_t standstills = 0;
  std::map<std::string, StopCount> reasons;
};

// Monday 00:00 UTC of the ISO week of t, and its name as 2026-W48.
int64_t WeekStart(double t);
std::string WeekName(int64_t start);
// The start of the oldest week the book keeps at now.
int64_t OldestWeek(double now);

// How the book reaches the incidents: open opens STOP with a reason, collecting names the one collecting now; each
// gives the id, empty for none.
struct StopIncidents {
  std::function<std::string(const std::string& reason)> open;
  std::function<std::string()> collecting;
};

// The stop book (FP23 B2c): an activity (MOWING, UNDOCKING, DOCKING, a drive, a calibration drive) that ends without
// a command and without its normal end is a stop. It is explained when an event of class cause comes within
// kStopExplainS of the end, or an emergency with a bit of class cause; every stop that is not opens an incident STOP,
// every other entry names the incident collecting. Standstills while busy are only observed. Counts by reason and ISO
// week for kStopBookWeeks, the last kStopBookLast entries. No clock, no files: the node hands in events, the state and
// the time.
class StopBook {
 public:
  // From stats/stops.json (null or broken: an empty book).
  explicit StopBook(const json& stored);

  // Without them the entries name no incident.
  void SetIncidents(StopIncidents incidents) { incidents_ = std::move(incidents); }

  // An event of events/json; mono and wall in s.
  void OnEvent(const json& event, double mono, double wall);
  // robot_state: whether it pauses and the pose, for standstills; true when an entry was added.
  bool OnPose(bool paused, bool pose, double x, double y, double mono, double wall);
  // Decides the ends whose window is over; true when an entry was added.
  bool Tick(double mono);
  // The incident incidents.mark opened or grew; true when it counts as a stop: no stop came in kStopMarkS before and
  // no mark of the same incident.
  bool OnMark(const std::string& incident, double wall);

  // The answer of stats.stops and stats/stops.json; blade_h per week start only in the answer.
  json Json(double now, const std::map<int64_t, double>* blade_h = nullptr) const;

  const std::string& activity() const { return activity_; }
  const std::deque<StopEntry>& last() const { return last_; }

 private:
  struct End {
    std::string activity;
    double mono = 0;
    double wall = 0;
  };
  // What deciding an end needs of an event.
  struct Seen {
    double mono = 0;
    std::string id;
    std::string type;
    std::string reason;
    // By a client, or with a reason that is the user's or the schedule's own.
    bool commanded = false;
    // DRIVE: an end that arrived.
    bool arrived = false;
    // EMERGENCY: the bits as it began, 0 at its end.
    int64_t bits = 0;
  };
  struct Anchor {
    double mono = 0;
    double x = 0;
    double y = 0;
    // The standstill is in the book.
    bool booked = false;
  };

  // true when it added an entry.
  bool Decide(const End& end);
  void Add(StopEntry entry);
  void Prune(double now);

  StopIncidents incidents_;
  std::string activity_;
  std::deque<End> ends_;
  std::deque<Seen> seen_;
  std::optional<Anchor> anchor_;
  std::deque<StopWeek> weeks_;
  std::deque<StopEntry> last_;
};

}  // namespace mowbite_stats

#endif  // MOWBITE_STATS_STOP_BOOK_H
