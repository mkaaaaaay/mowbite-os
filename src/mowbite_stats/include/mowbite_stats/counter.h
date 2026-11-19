#ifndef MOWBITE_STATS_COUNTER_H
#define MOWBITE_STATS_COUNTER_H

#include <cstdint>
#include <deque>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_journal/format.h"

namespace mowbite_stats {

using json = nlohmann::ordered_json;

// Hours with something in them that are kept, about three years of mowing.
constexpr size_t kMaxHours = 10000;
// Samples further apart (s) do not count: the stream had a gap.
constexpr double kMaxGap = 5.0;
// A step of the pose longer than this (m) between two samples is a jump of the position, not driven.
constexpr double kMaxStep = 2.0;
using mowbite_journal::kEarliestWall;
using mowbite_journal::WallSet;

struct Amounts {
  double blade_s = 0;   // the blade turned
  double mowed_m = 0;   // driven with the blade turning
  double driven_m = 0;  // driven without it, not in IDLE
  int64_t runs = 0;     // left IDLE to undock or to mow
  bool Empty() const { return blade_s == 0 && mowed_m == 0 && driven_m == 0 && runs == 0; }
};

struct Hour {
  int64_t t = 0;  // start, unix s, a whole UTC hour
  Amounts amounts;
};

// A part to see to after due_h blade hours.
struct Part {
  std::string key;
  double due_h = 0;
  double base_blade_s = 0;  // total blade seconds when it was last seen to
  int64_t done_at = 0;      // unix s, 0 not since counting started
};

// One reading, once a second: blade from hw/status, state and pose from robot_state.
struct Sample {
  double wall = 0;  // unix s
  double mono = 0;  // s of a clock that does not jump
  bool blade = false;
  std::string state;
  bool pose = false;  // x, y are recent
  double x = 0;
  double y = 0;
};

// What a sample changed: which files are to be written.
struct Changed {
  bool hour_closed = false;  // an hour with something in it went into the hours
};

// Counts blade time, distance and runs per hour from the samples, and the blade hours of the parts since they were
// last seen to. No clock, no files: the node hands in samples and stores the JSON.
class Counter {
 public:
  // From the stored files (null or broken: counting starts anew at now) and the parts with their defaults.
  Counter(const json& state, const json& hours, const json& parts, double now);

  Changed Add(const Sample& sample);

  // stats/state.json: since, the totals and the hour going on; since and its t are 0 until the clock is set.
  json StateJson() const;
  // stats/hours.json: [t, blade_s, mowed_m, driven_m, runs] per closed hour, oldest first.
  json HoursJson() const;
  // stats/parts.json.
  json PartsJson() const;

  // The answer of stats.get: hours from from on (unix s), at most kMaxAnswerHours of the newest.
  json Get(std::optional<int64_t> from, double now) const;
  // false for a part that does not exist.
  bool PartDone(const std::string& key, double now);
  bool PartDue(const std::string& key, double hours);

  // Blade hours of a part since it was last seen to.
  double UsedHours(const Part& part) const;
  const std::vector<Part>& parts() const { return parts_; }
  const Amounts& total() const { return total_; }

 private:
  // Keeps the hour going on when it has something in it.
  bool CloseHour();

  double since_ = 0;
  Amounts total_;
  Hour hour_;
  std::deque<Hour> hours_;
  std::vector<Part> parts_;
  std::optional<Sample> last_;
};

// The parts and how many blade hours they come due after, until stats.part_due changes them.
const std::vector<Part>& DefaultParts();

}  // namespace mowbite_stats

#endif  // MOWBITE_STATS_COUNTER_H
