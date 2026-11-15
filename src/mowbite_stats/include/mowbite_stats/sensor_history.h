#ifndef MOWBITE_STATS_SENSOR_HISTORY_H
#define MOWBITE_STATS_SENSOR_HISTORY_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_stats/counter.h"

namespace mowbite_stats {

// Width of a bucket (s), and how long buckets are kept (s).
constexpr int64_t kBucketSeconds = mowbite_api::kSensorsHistorySteps[0];
constexpr int64_t kKeepSeconds = int64_t{mowbite_api::kSensorsHistoryHours} * 3600;
// Number sensors recorded at once; about 20 on the Tron.
constexpr size_t kMaxSensors = 64;
// Longest id, name or unit (bytes); a sensor with a longer one is not recorded.
constexpr size_t kMaxSensorText = 100;
// All hour files together; at kMaxSensors full about 11.3 MB.
constexpr size_t kMaxHistoryBytes = size_t{16} * 1024 * 1024;
// Wall times before 2026-01-01 come from a clock that is not set: their values are not recorded.
constexpr double kEarliestWall = 1767225600;

// Values of one sensor in one minute; t its start in unix s.
struct Bucket {
  int64_t t = 0;
  double min = 0;
  double max = 0;
  double sum = 0;
  uint32_t n = 0;
};

// A file of one hour to write; empty text removes it.
struct HourFile {
  int64_t hour = 0;
  std::string text;
};

struct HistoryQuery {
  std::optional<std::vector<std::string>> ids;  // none: all
  std::optional<int64_t> from;                  // none: kKeepSeconds before now
  std::optional<int64_t> to;                    // none: now
  int64_t step = mowbite_api::kSensorsHistorySteps[0];
};

// Min, mean and max of the number sensors per minute over the last kKeepSeconds, kept in one file per UTC hour:
// a line {"schema": 1, "sensors": [[id, name, unit], ...]}, then [t, index, min, avg, max, n] per bucket. No clock,
// no files: the node hands in values and the texts of the files, and writes what TakeFiles gives.
class SensorHistory {
 public:
  // max_bytes: all hour files together, smaller only in tests.
  explicit SensorHistory(size_t max_bytes = kMaxHistoryBytes) : max_bytes_(max_bytes) {}

  // false when the sensor is not recorded: a text, a text field too long or no valid UTF-8, kMaxSensors reached.
  bool Describe(const std::string& id, const std::string& name, const std::string& unit, bool number);
  // Values of sensors not described as numbers, not finite or before kEarliestWall are left out.
  void Add(const std::string& id, double value, double wall);

  // The text of the file of hour, newest hour first; lines that do not parse are left out and the file is written
  // anew, a file beyond max_bytes is removed.
  void Load(int64_t hour, const std::string& text);
  // Drops buckets older than kKeepSeconds before now, then the files to write: hours changed since the last call,
  // and those to remove. Past max_bytes the oldest hours go.
  std::vector<HourFile> TakeFiles(double now);
  // These hours are written at the next TakeFiles again.
  void Unwritten(const std::vector<HourFile>& files);
  // Every hour is written at the next TakeFiles.
  void AllUnwritten();

  // The answer of sensors.history: the smallest step from query.step on whose answer has at most max_bytes; with
  // the largest step the oldest points go until it fits.
  json Query(const HistoryQuery& query, double now,
             size_t max_bytes = mowbite_api::kSensorsHistoryMaxAnswerBytes) const;

  size_t Bytes() const;
  size_t Buckets() const;

 private:
  struct Series {
    std::string name;
    std::string unit;
    bool recorded = false;       // described as a number now
    std::deque<Bucket> buckets;  // by t, oldest first
  };

  Series* SeriesFor(const std::string& id);
  void Merge(Series* series, const Bucket& bucket);
  std::string HourText(int64_t hour) const;
  void DropHour(int64_t hour);
  // The answer with this step for the points from from to to; ids null for all.
  json Answer(const std::set<std::string>* ids, int64_t step, int64_t from, int64_t to) const;

  const size_t max_bytes_;
  std::map<std::string, Series> series_;
  std::set<int64_t> dirty_;
  std::set<int64_t> remove_;
  // Hours with a file and its bytes.
  std::map<int64_t, size_t> files_;
};

}  // namespace mowbite_stats

#endif  // MOWBITE_STATS_SENSOR_HISTORY_H
