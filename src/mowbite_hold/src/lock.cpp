#include "mowbite_hold/lock.h"

#include <sys/stat.h>

#include <cerrno>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>

#include "mowbite_hold/file_io.h"

namespace mowbite_hold {

namespace {

// At most one cause per step of a step list (kSetupMaxSteps) and a reset; anything much larger is not a lock.
constexpr size_t kMaxFileBytes = 65536;
constexpr size_t kMaxCauses = 128;

bool OptionalNumber(const nlohmann::json& cause, const char* key, std::optional<double>* out) {
  const auto it = cause.find(key);
  if (it == cause.end() || it->is_null()) return true;
  if (!it->is_number() || !std::isfinite(it->get<double>())) return false;
  *out = it->get<double>();
  return true;
}

bool OptionalText(const nlohmann::json& cause, const char* key, std::string* out) {
  const auto it = cause.find(key);
  if (it == cause.end()) return true;
  if (!it->is_string()) return false;
  *out = it->get<std::string>();
  return true;
}

std::optional<LockCause> ParseCause(const nlohmann::json& cause) {
  if (!cause.is_object()) return std::nullopt;
  LockCause out;
  const auto reset = cause.find("reset");
  if (reset != cause.end()) {
    if (!reset->is_boolean() || !reset->get<bool>()) return std::nullopt;
    out.reset = true;
  }
  if (!OptionalText(cause, "step", &out.step) || !OptionalText(cause, "part", &out.part) ||
      !OptionalText(cause, "reason", &out.reason) || !OptionalText(cause, "firmware", &out.firmware) ||
      !OptionalNumber(cause, "value", &out.value) || !OptionalNumber(cause, "limit", &out.limit)) {
    return std::nullopt;
  }
  const auto t = cause.find("t");
  if (t != cause.end()) {
    if (!t->is_number_integer()) return std::nullopt;
    out.t = t->get<int64_t>();
  }
  if (out.reason.empty() || (!out.reset && out.step.empty())) return std::nullopt;
  return out;
}

}  // namespace

std::optional<LockRecord> ParseSafetyLock(const std::string& text) {
  const nlohmann::json value = nlohmann::json::parse(text, nullptr, false);
  if (value.is_discarded() || !value.is_object()) return std::nullopt;
  const auto failures = value.find("failures");
  const auto causes = value.find("causes");
  if (failures == value.end() || !failures->is_number_unsigned() ||
      failures->get<uint64_t>() > std::numeric_limits<uint32_t>::max()) {
    return std::nullopt;
  }
  if (causes == value.end() || !causes->is_array() || causes->empty() || causes->size() > kMaxCauses) {
    return std::nullopt;
  }
  LockRecord record;
  record.failures = static_cast<uint32_t>(failures->get<uint64_t>());
  for (const auto& item : *causes) {
    auto cause = ParseCause(item);
    if (!cause) return std::nullopt;
    record.causes.push_back(std::move(*cause));
  }
  return record;
}

std::string SafetyLockText(const LockRecord& record) {
  nlohmann::json causes = nlohmann::json::array();
  for (const LockCause& cause : record.causes) {
    if (cause.reset) {
      causes.push_back({{"reset", true}, {"reason", cause.reason}, {"t", cause.t}});
      continue;
    }
    causes.push_back({{"step", cause.step},
                      {"part", cause.part},
                      {"reason", cause.reason},
                      {"value", cause.value ? nlohmann::json(*cause.value) : nlohmann::json()},
                      {"limit", cause.limit ? nlohmann::json(*cause.limit) : nlohmann::json()},
                      {"t", cause.t},
                      {"firmware", cause.firmware}});
  }
  // Texts of a board (firmware) may hold bytes that are no UTF-8.
  return nlohmann::json{{"failures", record.failures}, {"causes", std::move(causes)}}.dump(
             2, ' ', false, nlohmann::json::error_handler_t::replace) +
         "\n";
}

SafetyLock ReadSafetyLock(const std::string& path) {
  SafetyLock lock;
  std::string text;
  std::string error;
  const FileRead read = ReadSmallFile(path, kMaxFileBytes, &text, &error);
  if (read == FileRead::kMissing) return lock;
  lock.closed = true;
  const auto record = read == FileRead::kText ? ParseSafetyLock(text) : std::nullopt;
  if (!record) {
    lock.unreadable = true;
    lock.text = read == FileRead::kTooLarge ? path + " is larger than a lock"
                : read == FileRead::kFailed ? error
                                            : path + " is not a lock of the setup program";
    return lock;
  }
  lock.failures = record->failures;
  const LockCause& first = record->causes.front();
  lock.text = first.reset ? "the setup program was reset (" + first.reason + ")"
                          : "the safety check " + first.step + " failed (" + first.reason + ")";
  return lock;
}

bool SafetyMeasured(const std::string& date, const SafetyLock& lock) {
  return !date.empty() && !lock.closed;
}

SafetyLockWatch::SafetyLockWatch(std::string path) : path_(std::move(path)) {
}

const SafetyLock& SafetyLockWatch::Poll() {
  if (path_.empty()) return lock_;
  struct stat st {};
  if (::stat(path_.c_str(), &st) != 0) {
    const int e = errno;
    seen_.reset();
    lock_ = SafetyLock();
    if (e == ENOENT) return lock_;
    lock_.closed = lock_.unreadable = true;
    lock_.text = "cannot stat " + path_ + ": " + ErrorText(e);
    return lock_;
  }
  const Identity id{st.st_dev, st.st_ino, st.st_size, static_cast<int64_t>(st.st_mtim.tv_sec),
                    static_cast<int64_t>(st.st_mtim.tv_nsec)};
  if (seen_ && *seen_ == id) return lock_;
  seen_ = id;
  lock_ = ReadSafetyLock(path_);
  // Gone between stat and open: looked at again with the next poll.
  if (!lock_.closed) seen_.reset();
  return lock_;
}

}  // namespace mowbite_hold
