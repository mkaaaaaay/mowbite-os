#include "mowbite_stats/incident_store.h"

#include <fcntl.h>
#include <openssl/evp.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <utility>

#include "mowbite_api/catalog.h"
#include "mowbite_hold/file_io.h"
#include "mowbite_journal/tar.h"
#include "mowbite_journal/timeline.h"

extern char** environ;

namespace mowbite_stats {

namespace fs = std::filesystem;

namespace {

constexpr char kIncidentsDir[] = "incidents";
constexpr char kManifest[] = "manifest.json";
constexpr char kTimeline[] = "timeline.jsonl";
// The copy without location, inside the incident; made in kAnonWork and renamed when complete.
constexpr char kAnonDir[] = ".anon";
constexpr char kAnonWork[] = ".anon.work";
constexpr size_t kMaxManifestBytes = size_t{4} * 1024 * 1024;
// The tool that copies a run without its place, in its own process: it knows every core, the stats do not.
constexpr char kAnonymizeTool[] = "mowbite_replay";
constexpr char kToolErrors[] = ".errors";
constexpr size_t kMaxToolErrorBytes = 2000;
constexpr int kManifestFormat = 1;

std::string Hex(const unsigned char* data, size_t size) {
  static const char kDigits[] = "0123456789abcdef";
  std::string out;
  out.reserve(size * 2);
  for (size_t i = 0; i < size; ++i) {
    out += kDigits[data[i] >> 4];
    out += kDigits[data[i] & 15];
  }
  return out;
}

class Sha256 {
 public:
  Sha256() : ctx_(EVP_MD_CTX_new(), EVP_MD_CTX_free) { EVP_DigestInit_ex(ctx_.get(), EVP_sha256(), nullptr); }
  void Add(const void* data, size_t size) { EVP_DigestUpdate(ctx_.get(), data, size); }
  std::string Hex() {
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int size = 0;
    EVP_DigestFinal_ex(ctx_.get(), digest, &size);
    return mowbite_stats::Hex(digest, size);
  }

 private:
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx_;
};

bool FileSha256(const std::string& path, std::string* hex, std::string* error) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *error = "cannot read " + path;
    return false;
  }
  Sha256 sha;
  std::vector<char> buffer(1 << 20);
  while (in) {
    in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    sha.Add(buffer.data(), static_cast<size_t>(in.gcount()));
  }
  if (in.bad()) {
    *error = "cannot read " + path;
    return false;
  }
  *hex = sha.Hex();
  return true;
}

std::string IsoUtc(double unix_s) {
  const std::time_t t = static_cast<std::time_t>(unix_s);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char text[32];
  std::strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return text;
}

bool SafeId(const std::string& id) {
  return !id.empty() && id.find('/') == std::string::npos && id[0] != '.';
}

// The segment files of an incident below root, relative and sorted: in the folders of its runs.
std::vector<std::string> SegmentFiles(const fs::path& root) {
  std::vector<std::string> files;
  std::error_code error;
  for (const auto& run : fs::directory_iterator(root, error)) {
    const std::string name = run.path().filename().string();
    if (!run.is_directory() || name.empty() || name[0] == '.') continue;
    std::error_code inner;
    for (const auto& file : fs::directory_iterator(run.path(), inner)) {
      if (file.is_regular_file()) files.push_back(name + "/" + file.path().filename().string());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

std::vector<std::string> RunFolders(const fs::path& root) {
  std::vector<std::string> dirs;
  std::error_code error;
  for (const auto& entry : fs::directory_iterator(root, error)) {
    const std::string name = entry.path().filename().string();
    if (entry.is_directory() && !name.empty() && name[0] != '.') dirs.push_back(entry.path().string());
  }
  std::sort(dirs.begin(), dirs.end());
  return dirs;
}

// mowbite_replay anonymize of one run into out_dir; its errors go to errors_file.
bool AnonymizeRun(const std::string& run, const std::string& out_dir, const std::string& errors_file,
                  const mowbite_journal::AnonymizeOptions& options, std::string* error) {
  std::vector<std::string> args = {kAnonymizeTool, "anonymize", "--shift-days", std::to_string(options.shift_days)};
  if (options.keep_raw) args.push_back("--keep-raw");
  args.push_back(run);
  args.push_back(out_dir);
  std::vector<char*> argv;
  for (auto& arg : args) argv.push_back(arg.data());
  argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, "/dev/null", O_WRONLY, 0);
  posix_spawn_file_actions_addopen(&actions, STDERR_FILENO, errors_file.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  pid_t pid = 0;
  const int spawned = posix_spawnp(&pid, kAnonymizeTool, &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  if (spawned != 0) {
    *error = std::string("cannot start ") + kAnonymizeTool + ": " + std::strerror(spawned);
    return false;
  }
  int status = 0;
  while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
  if (WIFEXITED(status) && WEXITSTATUS(status) == 0) return true;
  std::string text, why;
  mowbite_hold::ReadSmallFile(errors_file, kMaxToolErrorBytes, &text, &why);
  *error = std::string(kAnonymizeTool) + " anonymize " + run + " failed (" +
           (WIFEXITED(status) ? "exit " + std::to_string(WEXITSTATUS(status)) : std::string("signal")) +
           "): " + text;
  return false;
}

// ended and end_reason come at sealing, from the copy in the incident.
json RunFacts(const mowbite_journal::RunInfo& run, bool from_start) {
  return json{{"name", run.name},       {"process", run.process},   {"pid", run.pid},
              {"boot_id", run.boot_id}, {"revision", run.revision}, {"image", run.image},
              {"platform", run.platform}, {"from_start", from_start}};
}

json TriggerJson(const Trigger& trigger) {
  json out = {{"kind", trigger.kind}, {"reason", trigger.reason}, {"safety", trigger.safety},
              {"t_ns", trigger.t_ns}, {"utc", trigger.utc}};
  if (!trigger.run.empty()) out["run"] = trigger.run;
  if (!trigger.detail.is_null()) out["detail"] = trigger.detail;
  return out;
}

// The manifest of the copy without location: no log lines, boot ids, wall clock of triggers or event details; dates
// in names and texts moved.
json AnonymousManifest(json manifest, const mowbite_journal::AnonymizeOptions& options) {
  manifest.erase("log");
  manifest["window"].erase("boot_id");
  for (auto& trigger : manifest["triggers"]) {
    trigger.erase("utc");
    trigger.erase("detail");
    if (trigger.contains("run")) trigger["run"] = ShiftNameDate(trigger["run"].get<std::string>(), options);
  }
  for (auto& run : manifest["runs"]) {
    run.erase("files");
    run.erase("boot_id");
    run["name"] = ShiftNameDate(run.value("name", ""), options);
  }
  manifest["id"] = ShiftNameDate(manifest.value("id", ""), options);
  options.ShiftWallIn(&manifest, {"created"});
  manifest["location"] = false;
  return manifest;
}

// The archive of an incident: its manifest first, then the segments of its runs, then (with location) its timeline.
mowbite_journal::VirtualTar Archive(const std::string& id, const fs::path& folder, const json& manifest,
                                    bool location) {
  const auto& options = IncidentStore::Anonymous();
  const fs::path root = location ? folder : folder / kAnonDir;
  const std::string name = location ? id : ShiftNameDate(id, options);
  const auto mtime =
      static_cast<int64_t>(manifest.value("created", 0.0) + (location ? 0.0 : static_cast<double>(options.shift_s())));
  std::vector<mowbite_journal::TarEntry> entries;
  const auto add = [&](const std::string& relative) {
    mowbite_journal::TarEntry entry;
    entry.name = name + "/" + relative;
    entry.path = (root / relative).string();
    std::error_code ec;
    entry.size = fs::file_size(entry.path, ec);
    entry.mtime = std::max<int64_t>(0, mtime);
    entries.push_back(std::move(entry));
  };
  add(kManifest);
  for (const std::string& file : SegmentFiles(root)) add(file);
  std::error_code ec;
  if (location && fs::exists(root / kTimeline, ec)) add(kTimeline);
  return mowbite_journal::VirtualTar(std::move(entries));
}

}  // namespace

std::string Sha256Hex(const std::string& data) {
  Sha256 sha;
  sha.Add(data.data(), data.size());
  return sha.Hex();
}

std::string ShiftNameDate(const std::string& name, const mowbite_journal::AnonymizeOptions& options) {
  if (name.size() < 8 || !std::all_of(name.begin(), name.begin() + 8, [](char c) { return c >= '0' && c <= '9'; })) {
    return name;
  }
  const std::string date = name.substr(0, 4) + "-" + name.substr(4, 2) + "-" + name.substr(6, 2);
  const std::string moved = options.ShiftDates(date);
  if (moved == date) return name;
  return moved.substr(0, 4) + moved.substr(5, 2) + moved.substr(8, 2) + name.substr(8);
}

const mowbite_journal::AnonymizeOptions& IncidentStore::Anonymous() {
  static const mowbite_journal::AnonymizeOptions options;
  return options;
}

IncidentStore::IncidentStore(std::string journal_dir, IncidentLimits limits)
    : journal_dir_(std::move(journal_dir)),
      dir_((fs::path(journal_dir_) / kIncidentsDir).string()),
      limits_(limits) {}

std::string IncidentStore::Folder(const std::string& id) const {
  return (fs::path(dir_) / id).string();
}

std::optional<json> IncidentStore::Manifest(const std::string& id) const {
  if (!SafeId(id)) return std::nullopt;
  std::string text, error;
  const auto read =
      mowbite_hold::ReadSmallFile((fs::path(Folder(id)) / kManifest).string(), kMaxManifestBytes, &text, &error);
  if (read != mowbite_hold::FileRead::kText) return std::nullopt;
  json manifest = json::parse(text, nullptr, false);
  if (manifest.is_discarded() || !manifest.is_object()) return std::nullopt;
  return manifest;
}

json IncidentStore::Timeline(const std::string& id) const {
  json lines = json::array();
  if (!SafeId(id)) return lines;
  std::string text, error;
  const auto read = mowbite_hold::ReadSmallFile((fs::path(Folder(id)) / kTimeline).string(),
                                                2 * mowbite_api::kIncidentTimelineBytes, &text, &error);
  if (read != mowbite_hold::FileRead::kText) return lines;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) {
    json value = json::parse(line, nullptr, false);
    if (!value.is_discarded()) lines.push_back(std::move(value));
  }
  return lines;
}

std::vector<StoredIncident> IncidentStore::Load() {
  std::error_code error;
  fs::create_directories(dir_, error);
  for (const auto& entry : fs::directory_iterator(dir_, error)) {
    if (!entry.is_directory()) continue;
    const std::string id = entry.path().filename().string();
    std::error_code ec;
    fs::remove_all(entry.path() / kAnonWork, ec);
    std::optional<json> manifest = Manifest(id);
    if (!manifest || manifest->value("state", "") == "done") continue;
    // Left open by an earlier process: what it linked stays, without the hashes it could not finish.
    (*manifest)["state"] = "done";
    (*manifest)["cut"] = true;
    std::string why;
    mowbite_hold::WriteFileSafely((fs::path(Folder(id)) / kManifest).string(), manifest->dump(1) + "\n", &why);
  }
  return Stored();
}

bool IncidentStore::WriteManifest(const Incident& incident, const IncidentContext& context, const std::string& state,
                                  const json& runs, std::string* error) const {
  json triggers = json::array();
  for (const Trigger& trigger : incident.triggers) triggers.push_back(TriggerJson(trigger));
  bool from_start = !runs.empty();
  for (const auto& run : runs) from_start = from_start && run.value("from_start", false);
  const json manifest = {
      {"format", kManifestFormat},
      {"id", incident.id},
      {"state", state},
      {"created", context.created_s},
      {"created_utc", IsoUtc(context.created_s)},
      {"safety", incident.safety},
      {"window", {{"from_ns", incident.from_ns}, {"to_ns", incident.to_ns}, {"boot_id", context.boot_id}}},
      {"triggers", triggers},
      {"more_triggers", incident.more},
      {"replay", from_start ? "from_start" : "window"},
      {"robot", context.robot},
      {"log", context.log},
      {"runs", runs}};
  return mowbite_hold::WriteFileSafely((fs::path(Folder(incident.id)) / kManifest).string(), manifest.dump(1) + "\n",
                                       error);
}

bool IncidentStore::Link(const Incident& incident, const IncidentContext& context,
                         const std::vector<mowbite_journal::RunInfo>& runs, const std::string& state,
                         std::string* error) {
  const fs::path folder = Folder(incident.id);
  std::error_code ec;
  fs::create_directories(folder, ec);
  if (ec) {
    *error = "cannot create " + folder.string() + ": " + ec.message();
    return false;
  }
  std::set<std::string> crashed;
  for (const Trigger& trigger : incident.triggers) {
    if (!trigger.run.empty()) crashed.insert(trigger.run);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  json& linked = runs_[incident.id];
  if (!linked.is_object()) linked = json::object();
  bool ok = true;
  for (const mowbite_journal::RunInfo& run : runs) {
    const auto& segments = run.segments;
    if (segments.empty()) continue;
    std::vector<size_t> chosen;
    if (crashed.count(run.name) != 0) {
      for (size_t i = segments.size() >= 2 ? segments.size() - 2 : 0; i < segments.size(); ++i) chosen.push_back(i);
    } else if (run.boot_id == context.boot_id) {
      // A segment lasts until the next begins; the last one up to now.
      for (size_t i = 0; i < segments.size(); ++i) {
        const bool last = i + 1 == segments.size();
        const int64_t end = last ? INT64_MAX : segments[i + 1].start_ns;
        if (segments[i].start_ns <= incident.to_ns && end >= incident.from_ns) chosen.push_back(i);
      }
    }
    if (chosen.empty()) continue;
    uint64_t bytes = 0;
    for (const auto& segment : segments) bytes += segment.bytes;
    const bool from_start = run.missing.empty() && static_cast<double>(bytes) <= limits_.max_mb * 1e6 / 4;
    if (from_start) {
      for (size_t i = 0; i < chosen.front(); ++i) chosen.push_back(i);
    }
    const fs::path into = folder / run.name;
    fs::create_directories(into, ec);
    for (const size_t i : chosen) {
      const fs::path target = into / fs::path(segments[i].path).filename();
      if (::link(segments[i].path.c_str(), target.c_str()) != 0 && errno != EEXIST) {
        *error = "cannot link " + segments[i].path + ": " + std::strerror(errno);
        ok = false;
      }
    }
    // Once from its start, always: the segments before stay linked.
    const bool before = linked.contains(run.name) && linked[run.name].value("from_start", false);
    linked[run.name] = RunFacts(run, from_start || before);
  }
  json facts = json::array();
  for (const auto& item : linked.items()) facts.push_back(item.value());
  return WriteManifest(incident, context, state, facts, error) && ok;
}

bool IncidentStore::ReadyToSeal(const Incident& incident,
                                const std::vector<mowbite_journal::RunInfo>& runs) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = runs_.find(incident.id);
  if (it == runs_.end()) return true;
  const fs::path folder = Folder(incident.id);
  for (const mowbite_journal::RunInfo& run : runs) {
    if (!it->second.contains(run.name) || !run.active) continue;
    // A linked segment of a running process that is still written.
    for (const auto& segment : run.segments) {
      if (segment.closed) continue;
      std::error_code ec;
      if (fs::exists(folder / run.name / fs::path(segment.path).filename(), ec)) return false;
    }
  }
  return true;
}

bool IncidentStore::Seal(const Incident& incident, const IncidentContext& context, std::string* error) {
  const fs::path folder = Folder(incident.id);
  json linked;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = runs_.find(incident.id);
    if (it != runs_.end()) linked = it->second;
  }
  std::string timeline_error;
  const std::string timeline = mowbite_journal::Timeline(RunFolders(folder), incident.from_ns, incident.to_ns,
                                                         mowbite_api::kIncidentTimelineBytes, &timeline_error);
  if (!mowbite_hold::WriteFileSafely((folder / kTimeline).string(), timeline, error)) return false;
  json facts = json::array();
  if (linked.is_object()) {
    for (const auto& item : linked.items()) {
      const std::string& name = item.key();
      json files = json::array();
      std::error_code ec;
      std::vector<std::string> names;
      for (const auto& entry : fs::directory_iterator(folder / name, ec)) {
        if (entry.is_regular_file()) names.push_back(entry.path().filename().string());
      }
      std::sort(names.begin(), names.end());
      for (const std::string& file : names) {
        std::string hex;
        if (!FileSha256((folder / name / file).string(), &hex, error)) return false;
        files.push_back({{"name", file}, {"bytes", fs::file_size(folder / name / file, ec)}, {"sha256", hex}});
      }
      json fact = item.value();
      const mowbite_journal::RunInfo copy = mowbite_journal::InspectRun((folder / name).string(), true);
      fact["ended"] = copy.ended;
      fact["end_reason"] = copy.end_reason;
      fact["files"] = files;
      facts.push_back(fact);
    }
  }
  IncidentContext sealed = context;
  if (!timeline_error.empty()) sealed.robot["timeline_error"] = timeline_error;
  if (!WriteManifest(incident, sealed, "done", facts, error)) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  runs_.erase(incident.id);
  return true;
}

std::vector<StoredIncident> IncidentStore::Stored() const {
  std::vector<StoredIncident> stored;
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(dir_, ec)) {
    if (!entry.is_directory()) continue;
    StoredIncident incident;
    incident.id = entry.path().filename().string();
    const std::optional<json> manifest = Manifest(incident.id);
    if (manifest) {
      incident.created_s = manifest->value("created", 0.0);
      incident.safety = manifest->value("safety", false);
      incident.open = manifest->value("state", "") != "done";
    }
    std::error_code walk;
    for (auto it = fs::recursive_directory_iterator(entry.path(), walk);
         !walk && it != fs::recursive_directory_iterator(); it.increment(walk)) {
      struct stat st {};
      if (it->is_regular_file() && ::stat(it->path().c_str(), &st) == 0) {
        incident.files.push_back({static_cast<uint64_t>(st.st_ino), static_cast<uint64_t>(st.st_size)});
      }
    }
    stored.push_back(std::move(incident));
  }
  return stored;
}

bool IncidentStore::Remove(const std::string& id, std::string* error) {
  if (!SafeId(id) || !fs::is_directory(Folder(id))) {
    *error = "no incident " + id;
    return false;
  }
  std::error_code ec;
  fs::remove_all(Folder(id), ec);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    runs_.erase(id);
    archive_sha_.erase({id, false});
    archive_sha_.erase({id, true});
  }
  if (ec) {
    *error = "cannot remove " + Folder(id) + ": " + ec.message();
    return false;
  }
  return true;
}

bool IncidentStore::Prepared(const std::string& id, bool location) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return archive_sha_.count({id, location}) != 0;
}

bool IncidentStore::Prepare(const std::string& id, bool location, std::string* error) {
  const std::optional<json> manifest = Manifest(id);
  if (!manifest) {
    *error = "no incident " + id;
    return false;
  }
  if (manifest->value("state", "") != "done") {
    *error = "the incident is still collecting";
    return false;
  }
  const fs::path folder = Folder(id);
  std::error_code ec;
  if (!location && !fs::exists(folder / kAnonDir, ec)) {
    const fs::path work = folder / kAnonWork;
    fs::remove_all(work, ec);
    fs::create_directories(work, ec);
    if (ec) {
      *error = "cannot create " + work.string() + ": " + ec.message();
      return false;
    }
    for (const std::string& run : RunFolders(folder)) {
      const std::string name = ShiftNameDate(fs::path(run).filename().string(), Anonymous());
      if (!AnonymizeRun(run, (work / name).string(), (work / kToolErrors).string(), Anonymous(), error)) {
        return false;
      }
    }
    const json anon = AnonymousManifest(*manifest, Anonymous());
    if (!mowbite_hold::WriteFileSafely((work / kManifest).string(), anon.dump(1) + "\n", error)) return false;
    fs::rename(work, folder / kAnonDir, ec);
    if (ec) {
      *error = "cannot finish " + work.string() + ": " + ec.message();
      return false;
    }
  }
  const mowbite_journal::VirtualTar tar = Archive(id, folder, *manifest, location);
  if (!tar.error().empty()) {
    *error = tar.error();
    return false;
  }
  Sha256 whole;
  std::string chunk;
  for (uint64_t at = 0; at < tar.size(); at += chunk.size()) {
    if (!tar.Read(at, size_t{1} << 20, &chunk, error)) return false;
    if (chunk.empty()) break;
    whole.Add(chunk.data(), chunk.size());
  }
  std::lock_guard<std::mutex> lock(mutex_);
  archive_sha_[{id, location}] = whole.Hex();
  return true;
}

bool IncidentStore::Piece(const std::string& id, uint64_t offset, size_t size, bool location, IncidentPiece* piece,
                          std::string* error) const {
  const std::optional<json> manifest = Manifest(id);
  if (!manifest) {
    *error = "no incident " + id;
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = archive_sha_.find({id, location});
    if (it == archive_sha_.end()) {
      *error = "the archive is not prepared";
      return false;
    }
    piece->archive_sha256 = it->second;
  }
  const mowbite_journal::VirtualTar tar = Archive(id, Folder(id), *manifest, location);
  piece->size = tar.size();
  piece->data.clear();
  if (offset >= tar.size()) return true;
  return tar.Read(offset, size, &piece->data, error);
}

}  // namespace mowbite_stats
