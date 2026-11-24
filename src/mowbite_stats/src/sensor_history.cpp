#include "mowbite_stats/sensor_history.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iterator>

#include "mowbite_api/catalog.h"

namespace mowbite_stats {

namespace {

constexpr int64_t kHour = 3600;
// Most values of a sensor in one minute a file may give, one every 60 us.
constexpr int64_t kMaxBucketValues = 1000000;

int64_t AlignDown(int64_t t, int64_t step) {
  return t - (((t % step) + step) % step);
}

int64_t HourOf(int64_t t) {
  return AlignDown(t, kHour);
}

// Six significant digits, as the files and the answers carry values.
double Rounded(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.6g", value);
  return std::strtod(text, nullptr);
}

// Kept within min and max, so that the rounded mean is never outside the rounded ends.
double Mean(double sum, uint64_t n, double min, double max) {
  return std::clamp(sum / static_cast<double>(n), min, max);
}

bool TextFits(const std::string& text) {
  if (text.size() > kMaxSensorText) return false;
  try {
    (void)json(text).dump();
  } catch (const json::type_error&) {
    return false;
  }
  return true;
}

bool AnswerFits(const json& value, size_t max_bytes) {
  return value.dump().size() <= max_bytes;
}

template <class Buckets>
auto FirstFrom(Buckets& buckets, int64_t t) {
  return std::lower_bound(buckets.begin(), buckets.end(), t, [](const Bucket& b, int64_t v) { return b.t < v; });
}

}  // namespace

bool SensorHistory::Describe(const std::string& id, const std::string& name, const std::string& unit, bool number) {
  const bool fits = !id.empty() && TextFits(id) && TextFits(name) && TextFits(unit);
  if (series_.count(id) == 0 && !(number && fits)) return false;
  Series* series = SeriesFor(id);
  if (!series) return false;
  series->recorded = number && fits;
  if (fits) {
    series->name = name;
    series->unit = unit;
  }
  return series->recorded;
}

void SensorHistory::Add(const std::string& id, double value, double wall) {
  if (!std::isfinite(value) || !WallSet(wall)) return;
  const auto it = series_.find(id);
  if (it == series_.end() || !it->second.recorded) return;
  const int64_t t = AlignDown(static_cast<int64_t>(std::floor(wall)), kBucketSeconds);
  Merge(&it->second, {t, value, value, value, 1});
  dirty_.insert(HourOf(t));
}

SensorHistory::Series* SensorHistory::SeriesFor(const std::string& id) {
  const auto it = series_.find(id);
  if (it != series_.end()) return &it->second;
  if (series_.size() >= kMaxSensors) return nullptr;
  return &series_.emplace(id, Series{}).first->second;
}

void SensorHistory::Merge(Series* series, const Bucket& bucket) {
  std::deque<Bucket>& buckets = series->buckets;
  // Mostly the newest bucket or a new one after it; after the clock went back, one in between.
  auto it = buckets.empty() || buckets.back().t < bucket.t ? buckets.end() : FirstFrom(buckets, bucket.t);
  if (it == buckets.end() || it->t != bucket.t) {
    buckets.insert(it, bucket);
    return;
  }
  it->min = std::min(it->min, bucket.min);
  it->max = std::max(it->max, bucket.max);
  it->sum += bucket.sum;
  it->n += bucket.n;
}

void SensorHistory::Load(int64_t hour, const std::string& text) {
  if (hour % kHour != 0) return;
  if (Bytes() + text.size() > max_bytes_) {
    remove_.insert(hour);
    return;
  }
  std::vector<Series*> sensors;
  bool clean = !text.empty() && text.back() == '\n';
  size_t loaded = 0;
  size_t start = 0;
  bool head = true;
  while (start < text.size()) {
    size_t end = text.find('\n', start);
    if (end == std::string::npos) end = text.size();
    const json line = json::parse(text.begin() + static_cast<std::ptrdiff_t>(start),
                                  text.begin() + static_cast<std::ptrdiff_t>(end), nullptr, false);
    start = end + 1;
    if (head) {
      head = false;
      if (!line.is_object() || line.value("schema", 0) != 1 || !line.contains("sensors") ||
          !line["sensors"].is_array()) {
        break;
      }
      for (const json& entry : line["sensors"]) {
        Series* series = nullptr;
        const bool texts = entry.is_array() && entry.size() == 3 && entry[0].is_string() && entry[1].is_string() &&
                           entry[2].is_string();
        std::string id = texts ? entry[0].get<std::string>() : "";
        std::string name = texts ? entry[1].get<std::string>() : "";
        // Files from before the own ids: the series goes on under the own id.
        if (const auto* own = mowbite_api::FindOpenMowerSensor(id)) {
          id = own->id;
          name = own->name;
        }
        const std::string unit = texts ? entry[2].get<std::string>() : "";
        if (!id.empty() && TextFits(id) && TextFits(name) && TextFits(unit)) {
          const bool known = series_.count(id) > 0;
          series = SeriesFor(id);
          // Files come newest first: the first name found is the newest.
          if (series && !known) {
            series->name = name;
            series->unit = unit;
          }
        }
        clean = clean && series;
        sensors.push_back(series);
      }
      continue;
    }
    const bool shape = line.is_array() && line.size() == 6 && line[0].is_number_integer() &&
                       line[1].is_number_unsigned() && line[5].is_number_integer() && line[2].is_number() &&
                       line[3].is_number() && line[4].is_number();
    if (!shape) {
      clean = false;
      continue;
    }
    const int64_t t = line[0].get<int64_t>();
    const uint64_t index = line[1].get<uint64_t>();
    const int64_t n = line[5].get<int64_t>();
    const double min = line[2].get<double>();
    const double avg = line[3].get<double>();
    const double max = line[4].get<double>();
    if (t % kBucketSeconds != 0 || HourOf(t) != hour || index >= sensors.size() || !sensors[index] || n < 1 ||
        n > kMaxBucketValues || !std::isfinite(min) || !std::isfinite(max) || !std::isfinite(avg) || min > avg ||
        avg > max) {
      clean = false;
      continue;
    }
    Merge(sensors[index], {t, min, max, avg * static_cast<double>(n), static_cast<uint32_t>(n)});
    ++loaded;
  }
  if (loaded == 0) {
    remove_.insert(hour);
    return;
  }
  files_[hour] = text.size();
  if (!clean) dirty_.insert(hour);
}

std::string SensorHistory::HourText(int64_t hour) const {
  json sensors = json::array();
  std::string lines;
  for (const auto& [id, series] : series_) {
    const auto begin = FirstFrom(series.buckets, hour);
    const auto end = FirstFrom(series.buckets, hour + kHour);
    if (begin == end) continue;
    const size_t index = sensors.size();
    sensors.push_back(json::array({id, series.name, series.unit}));
    for (auto it = begin; it != end; ++it) {
      lines += json::array({it->t, index, Rounded(it->min), Rounded(Mean(it->sum, it->n, it->min, it->max)),
                            Rounded(it->max), it->n})
                   .dump();
      lines += '\n';
    }
  }
  if (sensors.empty()) return "";
  return json{{"schema", 1}, {"sensors", sensors}}.dump() + "\n" + lines;
}

void SensorHistory::DropHour(int64_t hour) {
  for (auto& [id, series] : series_) {
    series.buckets.erase(FirstFrom(series.buckets, hour), FirstFrom(series.buckets, hour + kHour));
  }
  files_.erase(hour);
  dirty_.erase(hour);
  remove_.insert(hour);
}

std::vector<HourFile> SensorHistory::TakeFiles(double now) {
  if (WallSet(now)) {
    // Only what lies wholly before the window goes; after the clock went back nothing does.
    const double cutoff = now - static_cast<double>(kKeepSeconds);
    for (auto& [id, series] : series_) {
      while (!series.buckets.empty() && static_cast<double>(series.buckets.front().t + kBucketSeconds) <= cutoff) {
        series.buckets.pop_front();
      }
    }
    while (!files_.empty() && static_cast<double>(files_.begin()->first + kHour) <= cutoff) {
      remove_.insert(files_.begin()->first);
      files_.erase(files_.begin());
    }
  }
  for (auto it = series_.begin(); it != series_.end();) {
    it = !it->second.recorded && it->second.buckets.empty() ? series_.erase(it) : std::next(it);
  }
  std::map<int64_t, std::string> writes;
  for (const int64_t hour : dirty_) {
    std::string text = HourText(hour);
    if (text.empty()) {
      if (files_.erase(hour) > 0) remove_.insert(hour);
      continue;
    }
    files_[hour] = text.size();
    remove_.erase(hour);
    writes[hour] = std::move(text);
  }
  dirty_.clear();
  while (Bytes() > max_bytes_) {
    const int64_t oldest = files_.begin()->first;
    DropHour(oldest);
    writes.erase(oldest);
  }
  std::vector<HourFile> out;
  out.reserve(writes.size() + remove_.size());
  for (auto& [hour, text] : writes) out.push_back({hour, std::move(text)});
  for (const int64_t hour : remove_) out.push_back({hour, ""});
  remove_.clear();
  return out;
}

void SensorHistory::Unwritten(const std::vector<HourFile>& files) {
  for (const HourFile& file : files) (file.text.empty() ? remove_ : dirty_).insert(file.hour);
}

void SensorHistory::AllUnwritten() {
  for (const auto& [id, series] : series_) {
    for (const Bucket& bucket : series.buckets) dirty_.insert(HourOf(bucket.t));
  }
}

json SensorHistory::Answer(const std::set<std::string>* ids, int64_t step, int64_t from, int64_t to) const {
  json sensors = json::array();
  for (const auto& [id, series] : series_) {
    if (ids && ids->count(id) == 0) continue;
    json points = json::array();
    auto it = FirstFrom(series.buckets, from);
    const auto end = series.buckets.end();
    while (it != end && it->t <= to) {
      const int64_t t = AlignDown(it->t, step);
      double min = it->min;
      double max = it->max;
      double sum = 0;
      uint64_t n = 0;
      for (; it != end && it->t <= to && AlignDown(it->t, step) == t; ++it) {
        min = std::min(min, it->min);
        max = std::max(max, it->max);
        sum += it->sum;
        n += it->n;
      }
      points.push_back(json::array({t, Rounded(min), Rounded(Mean(sum, n, min, max)), Rounded(max)}));
    }
    sensors.push_back(
        {{"sensor_id", id}, {"key", id}, {"name", series.name}, {"unit", series.unit}, {"points", points}});
  }
  return {{"step", step}, {"from", from}, {"to", to}, {"truncated", false}, {"sensors", sensors}};
}

json SensorHistory::Query(const HistoryQuery& query, double now, size_t max_bytes) const {
  const int64_t to = query.to.value_or(static_cast<int64_t>(std::floor(now)));
  const int64_t from = std::max<int64_t>(0, query.from.value_or(to - kKeepSeconds));
  std::optional<std::set<std::string>> ids;
  if (query.ids) ids.emplace(query.ids->begin(), query.ids->end());
  const std::set<std::string>* only = ids ? &*ids : nullptr;
  const int64_t largest = mowbite_api::kSensorsHistoryMaxStep;
  for (const int64_t step : mowbite_api::kSensorsHistorySteps) {
    if (step < query.step && step != largest) continue;
    json answer = Answer(only, step, AlignDown(from, step), to);
    if (AnswerFits(answer, max_bytes)) return answer;
  }
  // Even the largest step is too much: the smallest number of its oldest points left out that makes it fit.
  const int64_t base = AlignDown(from, largest);
  int64_t low = 1;
  int64_t high = to < base ? 1 : (AlignDown(to, largest) - base) / largest + 1;
  while (low < high) {
    const int64_t mid = low + (high - low) / 2;
    if (AnswerFits(Answer(only, largest, base + mid * largest, to), max_bytes)) {
      high = mid;
    } else {
      low = mid + 1;
    }
  }
  json answer = Answer(only, largest, base + low * largest, to);
  answer["truncated"] = true;
  return answer;
}

size_t SensorHistory::Bytes() const {
  size_t bytes = 0;
  for (const auto& [hour, size] : files_) bytes += size;
  return bytes;
}

size_t SensorHistory::Buckets() const {
  size_t count = 0;
  for (const auto& [id, series] : series_) count += series.buckets.size();
  return count;
}

}  // namespace mowbite_stats
