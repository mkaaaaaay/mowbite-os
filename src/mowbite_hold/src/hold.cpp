#include "mowbite_hold/hold.h"

#include <sys/stat.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <nlohmann/json.hpp>

#include "file_io.h"

namespace mowbite_hold {

namespace {

// A hold file is a few dozen bytes; anything much larger is not one.
constexpr size_t kMaxFileBytes = 4096;
// Beyond any wall clock this OS will see: larger numbers are no time.
constexpr int64_t kLatestTime = 100000000000;

std::optional<int64_t> Time(const nlohmann::json& value) {
  if (!value.is_number_integer()) return std::nullopt;
  if (value.is_number_unsigned() && value.get<uint64_t>() > static_cast<uint64_t>(kLatestTime)) return std::nullopt;
  const int64_t t = value.get<int64_t>();
  if (t < 0 || t > kLatestTime) return std::nullopt;
  return t;
}

}  // namespace

std::optional<HoldRecord> ParseHold(const std::string& text) {
  const nlohmann::json value = nlohmann::json::parse(text, nullptr, false);
  if (value.is_discarded() || !value.is_object()) return std::nullopt;
  const auto until = value.find("until");
  const auto set_at = value.find("set_at");
  if (until == value.end() || set_at == value.end()) return std::nullopt;
  HoldRecord record;
  const auto u = Time(*until);
  const auto s = Time(*set_at);
  if (!u || !s || *u == 0) return std::nullopt;
  record.until = *u;
  record.set_at = *s;
  return record;
}

std::string HoldText(const HoldRecord& record) {
  return nlohmann::json{{"until", record.until}, {"set_at", record.set_at}}.dump() + "\n";
}

int64_t HoldSeconds(const HoldRecord& record, int64_t now_wall) {
  return std::clamp<int64_t>(record.until - now_wall, 0, kMaxHoldSeconds);
}

HoldFile ReadHoldFile(const std::string& path) {
  HoldFile file;
  std::string text;
  const FileRead read = ReadSmallFile(path, kMaxFileBytes, &text, &file.error);
  if (read == FileRead::kMissing) return file;
  file.state = HoldFile::State::kUnreadable;
  if (read == FileRead::kTooLarge) file.error = path + " is larger than a hold";
  if (read != FileRead::kText) return file;
  const auto record = ParseHold(text);
  if (!record) {
    file.error = path + " is not a hold (until and set_at in unix seconds)";
    return file;
  }
  file.state = HoldFile::State::kRecord;
  file.record = *record;
  return file;
}

bool WriteHoldFile(const std::string& path, const HoldRecord& record, std::string* error) {
  return WriteFileAtomic(path, HoldText(record), error);
}

bool RemoveHoldFile(const std::string& path, std::string* error) {
  return RemoveFileSynced(path, error);
}

void HoldTimer::Start(const HoldRecord& record, double now_mono, int64_t now_wall) {
  record_ = record;
  end_mono_ = now_mono + static_cast<double>(HoldSeconds(record, now_wall));
}

void HoldTimer::Clear() {
  record_.reset();
}

bool HoldTimer::Active(double now_mono) const {
  return record_.has_value() && now_mono < end_mono_;
}

int64_t HoldTimer::Until(double now_mono) const {
  return Active(now_mono) ? record_->until : 0;
}

HoldWatch::HoldWatch(std::string path) : path_(std::move(path)) {
}

HoldView HoldWatch::Poll(double now_mono, int64_t now_wall) {
  if (path_.empty()) return {};
  struct stat st {};
  if (::stat(path_.c_str(), &st) != 0) {
    const int e = errno;
    seen_.reset();
    timer_.Clear();
    unreadable_ = e != ENOENT;
    if (!unreadable_) return {};
    error_ = "cannot stat " + path_ + ": " + std::strerror(e);
    return {true, true, 0, error_};
  }
  const Identity id{st.st_dev, st.st_ino, st.st_size, static_cast<int64_t>(st.st_mtim.tv_sec),
                    static_cast<int64_t>(st.st_mtim.tv_nsec)};
  if (!seen_ || !(*seen_ == id)) {
    seen_ = id;
    const HoldFile file = ReadHoldFile(path_);
    timer_.Clear();
    unreadable_ = file.state == HoldFile::State::kUnreadable;
    error_ = file.error;
    // Gone between stat and open: looked at again with the next poll.
    if (file.state == HoldFile::State::kMissing) seen_.reset();
    if (file.state == HoldFile::State::kRecord) timer_.Start(file.record, now_mono, now_wall);
  }
  if (unreadable_) return {true, true, 0, error_};
  HoldView view;
  view.held = timer_.Active(now_mono);
  view.until = timer_.Until(now_mono);
  return view;
}

}  // namespace mowbite_hold
