#include "mowbite_hold/restart.h"

#include <sys/stat.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <nlohmann/json.hpp>

#include "mowbite_hold/file_io.h"

namespace mowbite_hold {

namespace {

// The files are a few hundred bytes; anything much larger is not one of them.
constexpr size_t kMaxFileBytes = 4096;
// A reason of the kernel record or an error text in an event: at most this long.
constexpr size_t kMaxDetail = 200;

std::string Clip(std::string text) {
  while (!text.empty() && (text.back() == '\n' || text.back() == '\r' || text.back() == ' ')) text.pop_back();
  if (text.size() > kMaxDetail) text.resize(kMaxDetail);
  return text;
}

std::string Tenths(double seconds) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.1f", seconds);
  return text;
}

}  // namespace

std::optional<SessionRecord> ParseSession(const std::string& text) {
  const nlohmann::json value = nlohmann::json::parse(text, nullptr, false);
  if (value.is_discarded() || !value.is_object()) return std::nullopt;
  const auto text_of = [&](const char* key) -> const nlohmann::json* {
    const auto it = value.find(key);
    return it != value.end() && it->is_string() ? &*it : nullptr;
  };
  const nlohmann::json* state = text_of("state");
  const nlohmann::json* job_id = text_of("job_id");
  const nlohmann::json* boot_id = text_of("boot_id");
  const auto since = value.find("since");
  const auto clean = value.find("clean");
  if (!state || state->get<std::string>().empty() || !job_id || !boot_id || since == value.end() ||
      !since->is_number_integer() || clean == value.end() || !clean->is_boolean()) {
    return std::nullopt;
  }
  return SessionRecord{state->get<std::string>(), job_id->get<std::string>(), boot_id->get<std::string>(),
                       since->get<int64_t>(), clean->get<bool>()};
}

std::string SessionText(const SessionRecord& record) {
  return nlohmann::json{{"state", record.state},
                        {"job_id", record.job_id},
                        {"boot_id", record.boot_id},
                        {"since", record.since},
                        {"clean", record.clean}}
             .dump() +
         "\n";
}

bool HoldsOnRestart(const SessionRecord& record) {
  return !record.clean && record.state != kIdle;
}

bool MustWaitForSession(bool written_holds, const SessionRecord& next) {
  return HoldsOnRestart(next) && !written_holds;
}

std::string SessionState(const std::string& state, const std::string& sub_state) {
  return state == kIdle && !sub_state.empty() ? sub_state : state;
}

const char* CauseName(Cause cause) {
  switch (cause) {
    case Cause::kKernelCrash: return "KERNEL_CRASH";
    case Cause::kOsCrash: return "OS_CRASH";
    case Cause::kPowerLoss: return "POWER_LOSS";
    case Cause::kNone:
    default: return "";
  }
}

SessionFile ReadSessionFile(const std::string& path) {
  SessionFile file;
  std::string text;
  const FileRead read = ReadSmallFile(path, kMaxFileBytes, &text, &file.error);
  if (read == FileRead::kMissing) return file;
  file.present = true;
  if (read == FileRead::kTooLarge) file.error = path + " is larger than a session";
  if (read != FileRead::kText) return file;
  file.record = ParseSession(text);
  if (!file.record) file.error = path + " is not a session";
  return file;
}

Verdict JudgeStart(const SessionFile& session, const std::string& boot_id, const std::optional<std::string>& kernel,
                   const std::optional<Stall>& stall) {
  Verdict verdict;
  if (session.record) {
    verdict.state_before = session.record->state;
    verdict.job_id_before = session.record->job_id;
  }
  // The kernel record of this boot was judged at the first start in it.
  if (session.record && !boot_id.empty() && session.record->boot_id == boot_id) {
    if (session.record->clean) return verdict;
    verdict.cause = Cause::kOsCrash;
    verdict.detail = "MowBite OS ended in this boot without its own stop (crash, killed, out of memory)";
    if (stall) {
      verdict.stalled = stall;
      verdict.detail = Clip("The watch ended MowBite OS: " + stall->node + " gave no sign for " +
                            Tenths(stall->seconds) + " s (limit " + Tenths(stall->limit) + " s)");
    }
    verdict.hold = HoldsOnRestart(*session.record);
    return verdict;
  }
  if (kernel) {
    verdict.cause = Cause::kKernelCrash;
    char* end = nullptr;
    const long long t = std::strtoll(kernel->c_str(), &end, 10);
    if (end != kernel->c_str() && t > 0 && *end == ' ') {
      verdict.crashed_at = t;
      verdict.detail = Clip(end + 1);
    } else {
      verdict.detail = Clip(*kernel);
    }
    if (verdict.detail.empty()) verdict.detail = "The kernel crashed";
    verdict.hold = true;
    return verdict;
  }
  if (!session.present || (session.record && session.record->clean)) return verdict;
  verdict.cause = Cause::kPowerLoss;
  if (!session.record) {
    verdict.detail = Clip("The state before the start is not known: " + session.error);
    verdict.hold = true;
    return verdict;
  }
  verdict.detail = "The mower started again while MowBite OS ran: power cut, hard reset, power key held or a hang";
  verdict.hold = HoldsOnRestart(*session.record);
  return verdict;
}

std::string RestartLine(const Verdict& verdict, bool held) {
  return "Unexpected restart " + std::string(CauseName(verdict.cause)) + " after " +
         (verdict.state_before.empty() ? "an unknown state" : verdict.state_before) + ": " + verdict.detail +
         (held ? "; held until emergency.reset" : "");
}

std::optional<Stall> ReadStall(const std::string& path) {
  std::string text;
  std::string error;
  if (ReadSmallFile(path, kMaxFileBytes, &text, &error) != FileRead::kText) return std::nullopt;
  const nlohmann::json note = nlohmann::json::parse(text, nullptr, false);
  if (!note.is_object() || !note.contains("stalled")) return std::nullopt;
  const nlohmann::json& stalled = note["stalled"];
  if (!stalled.is_object() || !stalled.contains("node") || !stalled["node"].is_string() ||
      !stalled.value("seconds", nlohmann::json()).is_number() || !stalled.value("limit", nlohmann::json()).is_number()) {
    return std::nullopt;
  }
  return Stall{Clip(stalled["node"].get<std::string>()), stalled["seconds"].get<double>(),
               stalled["limit"].get<double>()};
}

std::string RestartHoldText(const Verdict& verdict, int64_t since) {
  return nlohmann::json{{"cause", CauseName(verdict.cause)},
                        {"detail", verdict.detail},
                        {"crashed_at", verdict.crashed_at > 0 ? nlohmann::json(verdict.crashed_at) : nlohmann::json()},
                        {"state_before", verdict.state_before},
                        {"job_id_before", verdict.job_id_before},
                        {"since", since}}
             .dump() +
         "\n";
}

bool RestartHoldPresent(const std::string& path) {
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0 || errno != ENOENT;
}

std::string ReadBootId(const std::string& path) {
  std::string text;
  std::string error;
  if (ReadSmallFile(path, 64, &text, &error) != FileRead::kText) return "";
  while (!text.empty() && (text.back() == '\n' || text.back() == ' ')) text.pop_back();
  return text;
}

std::optional<std::string> ReadKernelRecord(const std::string& path) {
  if (path.empty()) return std::nullopt;
  std::string text;
  std::string error;
  switch (ReadSmallFile(path, kMaxFileBytes, &text, &error)) {
    case FileRead::kMissing: return std::nullopt;
    case FileRead::kText: return text;
    case FileRead::kTooLarge: return Clip(text);
    case FileRead::kFailed:
    default: return error;
  }
}

bool WriteRecordFile(const std::string& path, const std::string& text, std::string* error) {
  return WriteFileAtomic(path, text, error);
}

bool RemoveRecordFile(const std::string& path, std::string* error) {
  return RemoveFileSynced(path, error);
}

}  // namespace mowbite_hold
