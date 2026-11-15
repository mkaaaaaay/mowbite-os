#include "mowbite_stats/incidents.h"

#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <condition_variable>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <utility>

#include "mowbite_api/catalog.h"
#include "mowbite_hold/file_io.h"
#include "mowbite_journal/journal.h"

namespace mowbite_stats {

namespace fs = std::filesystem;

namespace {

constexpr int kInternalError = -32603;
constexpr char kSeenFile[] = "seen.json";
// incidents.mark keeps this much before the mark.
constexpr double kMarkBefore = 300.0;
// A failed seal is tried again this often before the incident is given up (it stays cut on the disk).
constexpr int kSealTries = 3;
constexpr size_t kMaxSeenBytes = size_t{4} * 1024 * 1024;

std::string IsoUtc(double unix_s) {
  const std::time_t t = static_cast<std::time_t>(unix_s);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char text[32];
  std::strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return text;
}

std::string Base64(const std::string& data) {
  static const char kDigits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((data.size() + 2) / 3 * 4);
  size_t i = 0;
  for (; i + 2 < data.size(); i += 3) {
    const uint32_t n = static_cast<uint8_t>(data[i]) << 16 | static_cast<uint8_t>(data[i + 1]) << 8 |
                       static_cast<uint8_t>(data[i + 2]);
    for (int shift = 18; shift >= 0; shift -= 6) out += kDigits[(n >> shift) & 63];
  }
  if (i < data.size()) {
    uint32_t n = static_cast<uint8_t>(data[i]) << 16;
    if (i + 1 < data.size()) n |= static_cast<uint8_t>(data[i + 1]) << 8;
    out += kDigits[(n >> 18) & 63];
    out += kDigits[(n >> 12) & 63];
    out += i + 1 < data.size() ? kDigits[(n >> 6) & 63] : '=';
    out += '=';
  }
  return out;
}

// Characters of a UTF-8 text.
size_t Characters(const std::string& text) {
  return static_cast<size_t>(
      std::count_if(text.begin(), text.end(), [](char c) { return (static_cast<uint8_t>(c) & 0xC0) != 0x80; }));
}

double FileMtime(const std::string& path) {
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0 ? static_cast<double>(st.st_mtime) : 0.0;
}

RpcOutcome Result(json result) {
  return {std::move(result), std::nullopt, false};
}

RpcOutcome Failed(const std::string& message) {
  RpcOutcome out;
  out.error = RpcError{kInternalError, message, nullptr};
  return out;
}

std::optional<RpcOutcome> RequiredId(const json& params, std::string* id) {
  if (!params.is_object() || !params.contains("id")) return BadParam("id", "missing", "the id of an incident");
  if (!params["id"].is_string()) return BadParam("id", "type", "the id of an incident");
  *id = params["id"].get<std::string>();
  return std::nullopt;
}

}  // namespace

// One thread at the idle I/O class for sealing and downloads; what follows a job runs on the thread of Tick.
class Incidents::Worker {
 public:
  using Job = std::function<std::function<void()>()>;

  Worker() : thread_([this] { Run(); }) {}
  ~Worker() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    wake_.notify_all();
    thread_.join();
  }

  void Post(Job job) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      jobs_.push_back(std::move(job));
    }
    wake_.notify_all();
  }

  std::vector<std::function<void()>> TakeDone() {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::exchange(done_, {});
  }

  void Drain() {
    std::unique_lock<std::mutex> lock(mutex_);
    idle_.wait(lock, [this] { return jobs_.empty() && !busy_; });
  }

 private:
  void Run() {
    // ioprio_set(IOPRIO_WHO_PROCESS, this thread, IOPRIO_CLASS_IDLE)
    syscall(SYS_ioprio_set, 1, 0, 3 << 13);
    std::unique_lock<std::mutex> lock(mutex_);
    for (;;) {
      wake_.wait(lock, [this] { return stop_ || !jobs_.empty(); });
      if (stop_) return;
      Job job = std::move(jobs_.front());
      jobs_.pop_front();
      busy_ = true;
      lock.unlock();
      std::function<void()> then = job();
      lock.lock();
      busy_ = false;
      if (then) done_.push_back(std::move(then));
      if (jobs_.empty()) idle_.notify_all();
    }
  }

  std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable idle_;
  std::deque<Job> jobs_;
  std::vector<std::function<void()>> done_;
  bool busy_ = false;
  bool stop_ = false;
  std::thread thread_;
};

std::string BootId() {
  std::ifstream in("/proc/sys/kernel/random/boot_id");
  std::string id;
  std::getline(in, id);
  return id;
}

const std::vector<std::string>& Incidents::Methods() {
  static const std::vector<std::string> methods = {"journal.status", "incidents.list", "incidents.get",
                                                   "incidents.download", "incidents.mark", "incidents.delete"};
  return methods;
}

Incidents::Incidents(IncidentsConfig config, std::function<int64_t(const std::string&)> free_bytes,
                     std::function<void(const std::string&)> warn)
    : config_(std::move(config)),
      free_bytes_(std::move(free_bytes)),
      warn_(std::move(warn)),
      keeper_(config_.limits),
      store_(config_.journal_dir, config_.limits),
      index_(config_.journal_dir),
      worker_(std::make_unique<Worker>()) {
  if (config_.journal_dir.empty()) return;
  for (const StoredIncident& incident : store_.Load()) keeper_.Known(incident.id);
  std::string text, error;
  if (mowbite_hold::ReadSmallFile((fs::path(store_.dir()) / kSeenFile).string(), kMaxSeenBytes, &text, &error) ==
      mowbite_hold::FileRead::kText) {
    const json seen = json::parse(text, nullptr, false);
    if (seen.is_array()) {
      for (const json& name : seen) {
        if (name.is_string()) seen_.insert(name.get<std::string>());
      }
    }
  }
}

Incidents::~Incidents() = default;

bool Incidents::Enabled() const {
  std::error_code ec;
  return !config_.journal_dir.empty() && !fs::exists(fs::path(config_.journal_dir) / mowbite_journal::kDisabledFile, ec);
}

std::string Incidents::Problem() const {
  if (config_.journal_dir.empty()) return "no journal_dir";
  if (!Enabled()) return "the journal is off (journal_enabled)";
  const int64_t free = free_bytes_(config_.journal_dir);
  if (free >= 0 && static_cast<double>(free) < config_.reserve_mb * 1e6) {
    return "less than journal_reserve_mb free on the disk of the journal";
  }
  return std::string();
}

Incidents::Notices Incidents::Open(const Trigger& trigger, double wall_s, std::string* opened) {
  if (!Problem().empty()) return {};
  const std::string id = keeper_.OnTrigger(trigger);
  if (opened) *opened = id;
  if (id.empty() || contexts_.count(id) != 0) return {};
  const Incident* incident = keeper_.Find(id);
  IncidentContext context;
  context.created_s = wall_s;
  context.boot_id = config_.boot_id;
  context.robot = robot_;
  for (const auto& [t_ns, line] : log_) {
    if (t_ns >= incident->from_ns) context.log.push_back(line);
  }
  contexts_[id] = std::move(context);
  // At once, so that the journal cannot remove a segment of the window before it is linked.
  runs_ = index_.Refresh(false);
  std::string error;
  if (!store_.Link(*incident, contexts_[id], runs_, "collecting", &error)) warn_("Incident " + id + ": " + error);
  return {json{{"reason", trigger.kind},
               {"message", "Incident " + id + ": " + trigger.kind},
               {"severity", "info"},
               {"resume", "auto"},
               {"id", id},
               {"detail", trigger.reason},
               {"safety", trigger.safety}}};
}

Incidents::Notices Incidents::OnEvent(const json& event, int64_t now_ns, double wall_s) {
  const std::optional<Trigger> trigger = TriggerOfEvent(event, now_ns, IsoUtc(wall_s));
  return trigger ? Open(*trigger, wall_s) : Notices();
}

Incidents::Notices Incidents::OnEmergency(uint32_t bits, int64_t now_ns, double wall_s) {
  const std::optional<Trigger> trigger = EmergencyTrigger(bits_, bits, now_ns, IsoUtc(wall_s));
  bits_ = bits;
  return trigger ? Open(*trigger, wall_s) : Notices();
}

void Incidents::SetJournalEnabled(bool enabled) {
  if (config_.journal_dir.empty() || enabled == Enabled()) return;
  const std::string path = (fs::path(config_.journal_dir) / mowbite_journal::kDisabledFile).string();
  std::string error;
  const bool ok = enabled ? mowbite_hold::RemoveFileSynced(path, &error)
                          : mowbite_hold::WriteFileSafely(path, "journal_enabled 0\n", &error);
  if (!ok) warn_("journal_enabled: " + error);
}

void Incidents::OnRobot(json robot) {
  robot_ = std::move(robot);
}

void Incidents::OnLog(int64_t now_ns, json line) {
  if (log_.size() >= kIncidentLogLines) log_.pop_front();
  log_.emplace_back(now_ns, line);
  if (const auto& collecting = keeper_.collecting()) {
    auto it = contexts_.find(collecting->id);
    if (it != contexts_.end() && it->second.log.size() < kIncidentLogLines && now_ns >= collecting->from_ns) {
      it->second.log.push_back(std::move(line));
    }
  }
}

Incidents::Notices Incidents::Tick(int64_t now_ns, double wall_s) {
  TakeDone();
  const int64_t keep_from = now_ns - static_cast<int64_t>(kIncidentLogSeconds * 1e9);
  while (!log_.empty() && log_.front().first < keep_from) log_.pop_front();
  Notices notices;
  if (config_.journal_dir.empty()) return notices;
  const bool over = !keeper_.OnTick(now_ns).empty();
  const double now_s = static_cast<double>(now_ns) / 1e9;
  if (over || now_s - last_runs_ >= kIncidentRunsEvery) {
    last_runs_ = now_s;
    RefreshRuns(now_ns, wall_s, &notices);
  }
  if (now_s - last_remove_ >= kIncidentRemoveEvery) {
    last_remove_ = now_s;
    RemoveOld(wall_s);
  }
  return notices;
}

void Incidents::RefreshRuns(int64_t now_ns, double wall_s, Notices* notices) {
  runs_ = index_.Refresh(false);
  if (Enabled()) FindCrashes(now_ns, wall_s, notices);
  LinkOpen();
}

void Incidents::LinkOpen() {
  for (const std::string& id : keeper_.OpenIds()) {
    if (sealing_.count(id) != 0) continue;
    const Incident* incident = keeper_.Find(id);
    IncidentContext& context = contexts_[id];
    const bool collecting = incident->state == IncidentState::kCollecting;
    std::string error;
    if (!store_.Link(*incident, context, runs_, collecting ? "collecting" : "sealing", &error)) {
      warn_("Incident " + id + ": " + error);
    }
    if (collecting || !store_.ReadyToSeal(*incident, runs_)) continue;
    sealing_.insert(id);
    worker_->Post([this, incident = *incident, context]() -> std::function<void()> {
      std::string why;
      const bool ok = store_.Seal(incident, context, &why);
      return [this, id = incident.id, ok, why] {
        sealing_.erase(id);
        IncidentContext& held = contexts_[id];
        if (!ok) {
          warn_("Incident " + id + " not sealed: " + why);
          if (++held.tries < kSealTries) return;
        }
        keeper_.Sealed(id);
        contexts_.erase(id);
      };
    });
  }
}

void Incidents::FindCrashes(int64_t now_ns, double wall_s, Notices* notices) {
  std::set<std::string> present;
  bool changed = false;
  for (const mowbite_journal::RunInfo& run : runs_) {
    present.insert(run.name);
    if (run.active || seen_.count(run.name) != 0) continue;
    seen_.insert(run.name);
    changed = true;
    // Runs that ended before this process came only when they were written to shortly before.
    if (first_pass_ && wall_s - FileMtime(run.segments.back().path) > kIncidentCrashBefore) continue;
    const mowbite_journal::RunInfo full = mowbite_journal::InspectRun(run.dir, true);
    if (full.ended) continue;
    Trigger trigger;
    trigger.kind = "CRASH";
    trigger.reason = full.process.empty() ? run.name : full.process;
    trigger.safety = true;
    trigger.t_ns = now_ns;
    trigger.utc = IsoUtc(wall_s);
    trigger.run = run.name;
    trigger.detail = json{{"end_reason", full.end_reason}, {"problem", full.problem}};
    for (json& notice : Open(trigger, wall_s)) notices->push_back(std::move(notice));
  }
  first_pass_ = false;
  for (auto it = seen_.begin(); it != seen_.end();) {
    if (present.count(*it) == 0) {
      it = seen_.erase(it);
      changed = true;
    } else {
      ++it;
    }
  }
  if (!changed) return;
  std::string error;
  if (!mowbite_hold::WriteFileSafely((fs::path(store_.dir()) / kSeenFile).string(), json(seen_).dump() + "\n",
                                     &error)) {
    warn_("Incidents: " + error);
  }
}

void Incidents::RemoveOld(double wall_s) {
  std::vector<StoredIncident> stored = store_.Stored();
  for (StoredIncident& incident : stored) {
    if (keeper_.Find(incident.id) != nullptr) incident.open = true;
  }
  for (const std::string& id : IncidentsToRemove(stored, wall_s, config_.limits)) {
    std::string error;
    if (!store_.Remove(id, &error)) warn_("Incident " + id + " not removed: " + error);
  }
}

void Incidents::TakeDone() {
  for (auto& then : worker_->TakeDone()) then();
}

void Incidents::Drain() {
  worker_->Drain();
  TakeDone();
}

std::string Incidents::StateOf(const std::string& id, const json& manifest) const {
  if (const Incident* incident = keeper_.Find(id)) {
    return incident->state == IncidentState::kCollecting ? "collecting" : "sealing";
  }
  return manifest.value("state", "") == "done" ? "done" : "sealing";
}

json Incidents::Status() const {
  uint64_t bytes = 0;
  uint64_t active = 0;
  for (const auto& run : runs_) {
    for (const auto& segment : run.segments) bytes += segment.bytes;
    if (run.active) ++active;
  }
  const int64_t free = config_.journal_dir.empty() ? -1 : free_bytes_(config_.journal_dir);
  std::map<uint64_t, uint64_t> inodes;
  const std::vector<StoredIncident> stored = config_.journal_dir.empty() ? std::vector<StoredIncident>()
                                                                          : store_.Stored();
  for (const StoredIncident& incident : stored) {
    for (const StoredFile& file : incident.files) inodes[file.inode] = file.bytes;
  }
  uint64_t incident_bytes = 0;
  for (const auto& item : inodes) incident_bytes += item.second;
  json throttled = json::array();
  for (const auto& [kind, count] : keeper_.throttled()) throttled.push_back({{"kind", kind}, {"count", count}});
  const auto& limits = config_.limits;
  return {{"enabled", Enabled()},
          {"problem", Problem()},
          {"bytes", bytes},
          {"max_bytes", static_cast<uint64_t>(config_.journal_max_mb * 1e6)},
          {"max_days", config_.journal_max_days},
          {"free", free < 0 ? 0 : free},
          {"reserve", static_cast<uint64_t>(config_.reserve_mb * 1e6)},
          {"runs", runs_.size()},
          {"active", active},
          {"incidents",
           {{"count", stored.size()},
            {"bytes", incident_bytes},
            {"max_bytes", static_cast<uint64_t>(limits.max_mb * 1e6)},
            {"max_days", limits.max_days},
            {"max_days_safety", limits.max_days_safety},
            {"throttled", throttled}}}};
}

json Incidents::List() const {
  std::vector<std::pair<double, json>> rows;
  if (config_.journal_dir.empty()) return json::array();
  for (const StoredIncident& incident : store_.Stored()) {
    const std::optional<json> manifest = store_.Manifest(incident.id);
    if (!manifest) continue;
    json kinds = json::array();
    std::string reason;
    for (const json& trigger : manifest->value("triggers", json::array())) {
      const std::string kind = trigger.value("kind", "");
      if (kinds.empty()) reason = trigger.value("reason", "");
      if (std::find(kinds.begin(), kinds.end(), kind) == kinds.end()) kinds.push_back(kind);
    }
    std::map<uint64_t, uint64_t> inodes;
    for (const StoredFile& file : incident.files) inodes[file.inode] = file.bytes;
    uint64_t bytes = 0;
    for (const auto& item : inodes) bytes += item.second;
    rows.emplace_back(incident.created_s, json{{"id", incident.id},
                                               {"created", static_cast<int64_t>(incident.created_s)},
                                               {"state", StateOf(incident.id, *manifest)},
                                               {"safety", incident.safety},
                                               {"triggers", kinds},
                                               {"reason", reason},
                                               {"bytes", bytes},
                                               {"replay", manifest->value("replay", "window")}});
  }
  std::stable_sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
  json list = json::array();
  for (auto& row : rows) list.push_back(std::move(row.second));
  return list;
}

RpcOutcome Incidents::Get(const json& params) const {
  if (auto bad = CheckKeys(params, {"client", "id"}, false)) return *bad;
  if (auto bad = CheckClient(params)) return *bad;
  std::string id;
  if (auto bad = RequiredId(params, &id)) return *bad;
  std::optional<json> manifest = store_.Manifest(id);
  if (!manifest) return BadParam("id", "unknown", "no incident with this id");
  (*manifest)["state"] = StateOf(id, *manifest);
  return Result({{"manifest", *manifest}, {"timeline", store_.Timeline(id)}});
}

RpcOutcome Incidents::Download(const json& params) {
  if (auto bad = CheckKeys(params, {"client", "id", "offset", "size", "location"}, false)) return *bad;
  if (auto bad = CheckClient(params)) return *bad;
  std::string id;
  if (auto bad = RequiredId(params, &id)) return *bad;
  uint64_t offset = 0;
  if (params.contains("offset")) {
    if (!params["offset"].is_number_unsigned()) return BadParam("offset", "type", "bytes, a whole number from 0");
    offset = params["offset"].get<uint64_t>();
  }
  int64_t size = mowbite_api::kIncidentChunkBytes;
  const std::string range = "bytes from 1 to " + std::to_string(mowbite_api::kIncidentChunkBytes);
  if (params.contains("size")) {
    if (!params["size"].is_number_integer()) return BadParam("size", "type", range);
    size = params["size"].get<int64_t>();
    if (size < 1 || size > mowbite_api::kIncidentChunkBytes) return BadParam("size", "range", range);
  }
  bool location = false;
  if (params.contains("location")) {
    if (!params["location"].is_boolean()) return BadParam("location", "type", "true or false");
    location = params["location"].get<bool>();
  }
  const std::optional<json> manifest = store_.Manifest(id);
  if (!manifest) return BadParam("id", "unknown", "no incident with this id");
  if (StateOf(id, *manifest) != "done") return BadParam("id", "state", "the incident is still collecting");
  const std::pair<std::string, bool> key{id, location};
  if (const auto failed = prepare_failed_.find(key); failed != prepare_failed_.end()) {
    const std::string why = failed->second;
    prepare_failed_.erase(failed);
    return Failed("The archive could not be made: " + why);
  }
  if (!store_.Prepared(id, location)) {
    if (preparing_.count(key) == 0) {
      const std::string problem = Problem();
      if (!problem.empty()) return Failed("The archive cannot be made: " + problem);
      preparing_.insert(key);
      worker_->Post([this, key]() -> std::function<void()> {
        std::string why;
        const bool ok = store_.Prepare(key.first, key.second, &why);
        return [this, key, ok, why] {
          preparing_.erase(key);
          if (!ok) prepare_failed_[key] = why;
        };
      });
    }
    return Result({{"ready", false}, {"size", 0}, {"offset", offset}, {"data", ""}, {"sha256", ""},
                   {"archive_sha256", ""}});
  }
  IncidentPiece piece;
  std::string error;
  if (!store_.Piece(id, offset, static_cast<size_t>(size), location, &piece, &error)) return Failed(error);
  return Result({{"ready", true},
                 {"size", piece.size},
                 {"offset", offset},
                 {"data", Base64(piece.data)},
                 {"sha256", Sha256Hex(piece.data)},
                 {"archive_sha256", piece.archive_sha256}});
}

RpcOutcome Incidents::Mark(const json& params, int64_t now_ns, double wall_s, Notices* notices) {
  if (auto bad = CheckKeys(params, {"client", "note"}, false)) return *bad;
  if (auto bad = CheckClient(params)) return *bad;
  const std::string limit = "a text of at most " + std::to_string(mowbite_api::kIncidentMaxNote) + " characters";
  if (!params.contains("note")) return BadParam("note", "missing", limit);
  if (!params["note"].is_string()) return BadParam("note", "type", limit);
  const std::string note = params["note"].get<std::string>();
  if (Characters(note) > static_cast<size_t>(mowbite_api::kIncidentMaxNote)) return BadParam("note", "range", limit);
  const std::string problem = Problem();
  if (!problem.empty()) return Failed("No incident: " + problem);
  Trigger trigger;
  trigger.kind = "MARK";
  trigger.reason = note;
  trigger.t_ns = now_ns;
  trigger.utc = IsoUtc(wall_s);
  trigger.before_s = kMarkBefore;
  std::string id;
  for (json& notice : Open(trigger, wall_s, &id)) notices->push_back(std::move(notice));
  if (id.empty()) {
    return BadParam("note", "throttled",
                    "a mark came less than " + std::to_string(static_cast<int>(config_.limits.kind_gap_s)) +
                        " s ago or " + std::to_string(config_.limits.kind_per_day) + " times today");
  }
  return Result({{"id", id}});
}

RpcOutcome Incidents::Delete(const json& params) {
  if (auto bad = CheckKeys(params, {"client", "id"}, true)) return *bad;
  if (auto bad = CheckClient(params)) return *bad;
  int64_t deleted = 0;
  std::string error;
  if (params.is_object() && params.contains("id")) {
    std::string id;
    if (auto bad = RequiredId(params, &id)) return *bad;
    if (!store_.Manifest(id)) return BadParam("id", "unknown", "no incident with this id");
    if (keeper_.Find(id) != nullptr) return BadParam("id", "state", "the incident is still collecting");
    if (!store_.Remove(id, &error)) return Failed(error);
    return Result({{"deleted", 1}});
  }
  for (const StoredIncident& incident : store_.Stored()) {
    if (keeper_.Find(incident.id) != nullptr) continue;
    if (store_.Remove(incident.id, &error)) {
      ++deleted;
    } else {
      warn_("Incident " + incident.id + " not deleted: " + error);
    }
  }
  return Result({{"deleted", deleted}});
}

RpcOutcome Incidents::Handle(const std::string& method, const json& params, int64_t now_ns, double wall_s,
                             Notices* notices) {
  TakeDone();
  if (method == "journal.status") {
    if (auto bad = CheckKeys(params, {}, true)) return *bad;
    return Result(Status());
  }
  if (method == "incidents.list") {
    if (auto bad = CheckKeys(params, {}, true)) return *bad;
    return Result(List());
  }
  if (config_.journal_dir.empty()) return Failed("No incidents: no journal_dir");
  if (method == "incidents.get") return Get(params);
  if (method == "incidents.download") return Download(params);
  if (method == "incidents.mark") return Mark(params, now_ns, wall_s, notices);
  return Delete(params);
}

}  // namespace mowbite_stats
