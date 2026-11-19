#include "mowbite_calibration/recording.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <limits>
#include <system_error>

namespace mowbite_calibration {

namespace fs = std::filesystem;

namespace {

constexpr const char* kPrefix = "calib-";
constexpr const char* kSuffix = ".jsonl.gz";
constexpr const char* kPart = ".part";
constexpr double kDay = 86400.0;

gzFile Gz(void* file) {
  return static_cast<gzFile>(file);
}

bool EndsWith(const std::string& text, const std::string& end) {
  return text.size() >= end.size() && text.compare(text.size() - end.size(), end.size(), end) == 0;
}

bool IsRecording(const std::string& name) {
  return name.rfind(kPrefix, 0) == 0 && EndsWith(name, kSuffix);
}

// Asks the disk to keep the file, or a rename or removal in a folder.
void Sync(const std::string& path, bool folder) {
  const int fd = ::open(path.c_str(), folder ? O_RDONLY | O_DIRECTORY : O_RDONLY);
  if (fd < 0) return;
  ::fsync(fd);
  ::close(fd);
}

// Flat, and every number finite: what the reader of the planner takes.
bool Clean(const json& fields) {
  if (!fields.is_object()) return false;
  for (const auto& item : fields.items()) {
    const json& value = item.value();
    if (value.is_structured()) return false;
    if (value.is_number_float() && !std::isfinite(value.get<double>())) return false;
  }
  return true;
}

bool Finite(const json& value) {
  if (value.is_number_float()) return std::isfinite(value.get<double>());
  if (value.is_structured()) {
    for (const auto& item : value) {
      if (!Finite(item)) return false;
    }
  }
  return true;
}

std::string Stamp(double wall_t) {
  const std::time_t seconds = static_cast<std::time_t>(std::floor(wall_t));
  std::tm utc{};
  gmtime_r(&seconds, &utc);
  char text[32];
  std::strftime(text, sizeof(text), "%Y%m%dT%H%M%SZ", &utc);
  return text;
}

}  // namespace

int Depth(const json& value) {
  if (!value.is_structured()) return 0;
  int deepest = 0;
  for (const auto& item : value) deepest = std::max(deepest, Depth(item));
  return deepest + 1;
}

Recording::~Recording() {
  Close();
}

void Recording::Close() {
  if (file_) gzclose(Gz(file_));
  file_ = nullptr;
}

bool Recording::Open(const std::string& folder, json head, double wall_t, const RecordingLimits& limits,
                     std::string* why) {
  if (file_) {
    Close();
    std::error_code ignored;
    fs::remove(path_ + kPart, ignored);
  }
  dropped_ = 0;
  max_bytes_ = limits.max_bytes;
  std::error_code error;
  fs::create_directories(folder, error);
  if (error) {
    if (why) *why = "cannot create " + folder + ": " + error.message();
    return false;
  }
  // Room for this one within the limits.
  RecordingLimits before = limits;
  before.max_runs = limits.max_runs > 0 ? limits.max_runs - 1 : 0;
  Prune(folder, wall_t, before);

  json line = json::object();
  line["format"] = "mowbite-calib/1";
  line["source"] = "mowbite-os";
  for (const auto& item : head.items()) {
    if (item.key() != "format" && item.key() != "source") line[item.key()] = item.value();
  }
  if (Depth(line) > 3 || !Finite(line)) {
    if (why) *why = "the head is deeper than 3 levels or has a number that is not finite";
    return false;
  }
  const std::string text = line.dump(-1, ' ', false, json::error_handler_t::replace) + "\n";
  if (text.size() > kMaxLine) {
    if (why) *why = "the head is longer than 4096 bytes";
    return false;
  }

  const std::string base = folder + "/" + kPrefix + Stamp(wall_t);
  path_ = base + kSuffix;
  for (int n = 2; fs::exists(path_, error) || fs::exists(path_ + kPart, error); n++) {
    path_ = base + "-" + std::to_string(n) + kSuffix;
  }
  file_ = gzopen((path_ + kPart).c_str(), "wb6");
  if (!file_) {
    if (why) *why = "cannot write " + path_ + kPart;
    return false;
  }
  if (gzwrite(Gz(file_), text.data(), static_cast<unsigned>(text.size())) != static_cast<int>(text.size())) {
    Close();
    fs::remove(path_ + kPart, error);
    if (why) *why = "cannot write " + path_ + kPart;
    return false;
  }
  return true;
}

bool Recording::Line(double wall_t, const char* kind, const json& fields) {
  if (!file_) return false;
  if (!std::isfinite(wall_t) || !Clean(fields)) {
    dropped_++;
    return false;
  }
  json line = json::object();
  line["t"] = std::round(wall_t * 1000.0) / 1000.0;
  line["k"] = kind;
  for (const auto& item : fields.items()) line[item.key()] = item.value();
  const std::string text = line.dump(-1, ' ', false, json::error_handler_t::replace) + "\n";
  if (text.size() > kMaxLine) {
    dropped_++;
    return false;
  }
  if (gzwrite(Gz(file_), text.data(), static_cast<unsigned>(text.size())) != static_cast<int>(text.size())) {
    dropped_++;
    return false;
  }
  if (static_cast<uint64_t>(gzoffset(Gz(file_))) >= max_bytes_) End(wall_t, "size", "");
  return true;
}

std::string Recording::End(double wall_t, const std::string& why, const std::string& reason) {
  if (!file_) return "";
  json line = json::object();
  line["t"] = std::isfinite(wall_t) ? std::round(wall_t * 1000.0) / 1000.0 : 0.0;
  line["k"] = "end";
  line["why"] = why;
  if (!reason.empty()) line["reason"] = reason;
  const std::string text = line.dump(-1, ' ', false, json::error_handler_t::replace) + "\n";
  gzwrite(Gz(file_), text.data(), static_cast<unsigned>(text.size()));
  const bool closed = gzclose(Gz(file_)) == Z_OK;
  file_ = nullptr;
  const std::string part = path_ + kPart;
  if (!closed) return "";
  Sync(part, false);
  std::error_code error;
  fs::rename(part, path_, error);
  if (error) return "";
  Sync(fs::path(path_).parent_path().string(), true);
  finished_ = path_;
  return path_;
}

std::vector<std::string> Prune(const std::string& folder, double wall_t, const RecordingLimits& limits,
                               const std::string& keep) {
  struct Entry {
    std::string path;
    std::string name;
    uint64_t bytes;
    double age_s;
  };
  std::vector<std::string> removed;
  std::vector<Entry> entries;
  std::error_code error;
  for (const auto& item : fs::directory_iterator(folder, error)) {
    const std::string name = item.path().filename().string();
    const bool part = name.rfind(kPrefix, 0) == 0 && EndsWith(name, std::string(kSuffix) + kPart);
    if (!part && !IsRecording(name)) continue;
    std::error_code item_error;
    if (part) {
      if (fs::remove(item.path(), item_error)) removed.push_back(name);
      continue;
    }
    struct stat info{};
    if (::stat(item.path().c_str(), &info) != 0) continue;
    entries.push_back({item.path().string(), name, static_cast<uint64_t>(info.st_size),
                       wall_t - static_cast<double>(info.st_mtime)});
  }
  // A clock behind the newest recording is not set yet or was set back: no recording counts as old by it then.
  double newest = -std::numeric_limits<double>::infinity();
  for (const auto& entry : entries) newest = std::max(newest, wall_t - entry.age_s);
  const bool clock_behind = !entries.empty() && wall_t < newest;
  // Newest first: the names carry the UTC time of their start.
  std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) { return a.name > b.name; });
  size_t kept = 0;
  uint64_t bytes = 0;
  for (const auto& entry : entries) {
    const bool always = !keep.empty() && fs::path(entry.path) == fs::path(keep);
    const bool old = !clock_behind && entry.age_s > limits.max_days * kDay;
    if (always || (!old && kept < limits.max_runs && bytes + entry.bytes <= limits.max_bytes)) {
      kept++;
      bytes += entry.bytes;
      continue;
    }
    std::error_code item_error;
    if (fs::remove(entry.path, item_error)) removed.push_back(entry.name);
  }
  if (!removed.empty()) Sync(folder, true);
  return removed;
}

}  // namespace mowbite_calibration
