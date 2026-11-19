#include "mowbite_bridge/event_log.h"

#include <algorithm>
#include <cctype>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "mowbite_bridge/text.h"

namespace fs = std::filesystem;

namespace mowbite_bridge {

namespace {

constexpr char kSuffix[] = ".jsonl";

bool IsDay(const std::string& text) {
  return text.size() == 8 && std::all_of(text.begin(), text.end(), [](unsigned char c) { return std::isdigit(c); });
}

}  // namespace

EventLog::EventLog(std::string dir, std::size_t max_days, std::uintmax_t max_bytes, std::size_t answer_bytes)
    : dir_(std::move(dir)), max_days_(max_days), max_bytes_(max_bytes), answer_bytes_(answer_bytes) {}

std::string EventLog::DayOf(double t) {
  const std::time_t seconds = static_cast<std::time_t>(t);
  std::tm local{};
  localtime_r(&seconds, &local);
  char text[16];
  std::strftime(text, sizeof(text), "%Y%m%d", &local);
  return text;
}

std::string EventLog::PathOf(const std::string& day) const {
  return (fs::path(dir_) / (day + kSuffix)).string();
}

std::vector<EventLog::DayFile> EventLog::Scan() const {
  std::vector<DayFile> days;
  std::error_code ec;
  if (dir_.empty() || !fs::is_directory(dir_, ec)) return days;
  for (const auto& entry : fs::directory_iterator(dir_, ec)) {
    const fs::path& path = entry.path();
    if (path.extension() != kSuffix || !IsDay(path.stem().string()) || !entry.is_regular_file(ec)) continue;
    days.push_back({path.stem().string(), entry.file_size(ec)});
  }
  std::sort(days.begin(), days.end(), [](const DayFile& a, const DayFile& b) { return a.day < b.day; });
  return days;
}

void EventLog::Trim(const std::string& today, std::uintmax_t incoming) {
  std::vector<DayFile> days = Scan();
  std::error_code ec;
  // Today counts even before its first line.
  const bool has_today = std::any_of(days.begin(), days.end(), [&](const DayFile& d) { return d.day == today; });
  std::size_t count = days.size() + (has_today ? 0 : 1);
  std::uintmax_t total = incoming;
  for (const auto& d : days) total += d.bytes;
  for (const auto& d : days) {
    if (d.day == today) continue;
    if (count <= max_days_ && total <= max_bytes_) break;
    if (fs::remove(PathOf(d.day), ec)) {
      --count;
      total -= d.bytes;
    }
  }
}

bool EventLog::Write(const Json& event, double now) {
  if (dir_.empty() || !event.is_object()) return true;
  std::error_code ec;
  fs::create_directories(dir_, ec);
  const std::string today = DayOf(now);
  const std::string line = event.dump(-1, ' ', false, Json::error_handler_t::replace) + "\n";
  // The day count only changes with a new day; the bytes are checked at every line.
  if (trimmed_for_ != today) {
    trimmed_for_ = today;
    Trim(today, 0);
  }
  std::uintmax_t total = line.size();
  for (const auto& d : Scan()) total += d.bytes;
  if (total > max_bytes_) {
    Trim(today, line.size());
    total = line.size();
    for (const auto& d : Scan()) total += d.bytes;
    if (total > max_bytes_) return true;
  }
  return AppendLines(PathOf(today), line);
}

Json EventLog::Read(const Json& params, double now) const {
  Json events = Json::array();
  std::string day = AskedName(params, "date", IsDay);
  if (day.empty()) day = DayOf(now);
  if (day == "-" || dir_.empty()) return events;
  std::ifstream in(PathOf(day));
  std::deque<std::string> lines;
  std::size_t bytes = 2;
  std::string line;
  while (std::getline(in, line)) {
    if (NestingDepth(line) > kMaxRequestDepth) continue;
    bytes += line.size() + 1;
    lines.push_back(std::move(line));
    while (bytes >= answer_bytes_ && !lines.empty()) {
      bytes -= lines.front().size() + 1;
      lines.pop_front();
    }
  }
  for (const auto& kept : lines) {
    Json event = Json::parse(kept, nullptr, false);
    if (event.is_object()) events.push_back(std::move(event));
  }
  return events;
}

Json EventLog::List() const {
  Json days = Json::array();
  const auto files = Scan();
  for (auto d = files.rbegin(); d != files.rend(); ++d) days.push_back(d->day);
  return days;
}

Json EventLog::Erase(const Json& params) {
  const std::string day = AskedName(params, "date", IsDay);
  if (day == "-") throw std::invalid_argument("params: none for every day, or {\"date\": \"YYYYMMDD\"}");
  int deleted = 0;
  std::error_code ec;
  for (const auto& d : Scan()) {
    if ((day.empty() || d.day == day) && fs::remove(PathOf(d.day), ec)) ++deleted;
  }
  return Json{{"deleted", deleted}};
}

}  // namespace mowbite_bridge
