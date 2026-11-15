#include "mowbite_stats/incident_store.h"

#include <openssl/evp.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <utility>

#include "mowbite_hold/file_io.h"
#include "mowbite_journal/tar.h"
#include "mowbite_replay/anonymize.h"

namespace mowbite_stats {

namespace fs = std::filesystem;

namespace {

constexpr char kIncidentsDir[] = "incidents";
constexpr char kManifest[] = "manifest.json";
// The copy without location, inside the incident.
constexpr char kAnonDir[] = ".anon";
constexpr size_t kMaxManifestBytes = size_t{4} * 1024 * 1024;
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

// The files of an incident below its folder, relative and sorted; the manifest and the copy without location apart.
std::vector<std::string> Files(const fs::path& folder, bool anon) {
  std::vector<std::string> files;
  std::error_code error;
  const fs::path root = anon ? folder / kAnonDir : folder;
  for (auto it = fs::recursive_directory_iterator(root, error); !error && it != fs::recursive_directory_iterator();
       it.increment(error)) {
    if (!anon && it->is_directory() && it->path().filename() == kAnonDir) {
      it.disable_recursion_pending();
      continue;
    }
    if (!it->is_regular_file()) continue;
    const std::string relative = fs::relative(it->path(), root).string();
    if (relative == kManifest) continue;
    files.push_back(relative);
  }
  std::sort(files.begin(), files.end());
  return files;
}

json RunFacts(const mowbite_journal::RunInfo& run, bool from_start) {
  return json{{"name", run.name},         {"process", run.process},   {"pid", run.pid},
              {"boot_id", run.boot_id},   {"revision", run.revision}, {"image", run.image},
              {"platform", run.platform}, {"ended", run.ended},       {"end_reason", run.end_reason},
              {"from_start", from_start}};
}

json TriggerJson(const Trigger& trigger) {
  json out = {{"kind", trigger.kind}, {"reason", trigger.reason}, {"safety", trigger.safety},
              {"t_ns", trigger.t_ns}, {"utc", trigger.utc}};
  if (!trigger.run.empty()) out["run"] = trigger.run;
  if (!trigger.detail.is_null()) out["detail"] = trigger.detail;
  return out;
}

}  // namespace

IncidentStore::IncidentStore(std::string journal_dir, IncidentLimits limits)
    : journal_dir_(std::move(journal_dir)),
      dir_((fs::path(journal_dir_) / kIncidentsDir).string()),
      limits_(limits) {}

std::string IncidentStore::Folder(const std::string& id) const {
  return (fs::path(dir_) / id).string();
}

std::optional<json> IncidentStore::Manifest(const std::string& id) const {
  if (id.empty() || id.find('/') != std::string::npos || id[0] == '.') return std::nullopt;
  std::string text, error;
  const auto read =
      mowbite_hold::ReadSmallFile((fs::path(Folder(id)) / kManifest).string(), kMaxManifestBytes, &text, &error);
  if (read != mowbite_hold::FileRead::kText) return std::nullopt;
  json manifest = json::parse(text, nullptr, false);
  if (manifest.is_discarded() || !manifest.is_object()) return std::nullopt;
  return manifest;
}

std::vector<StoredIncident> IncidentStore::Load() {
  std::error_code error;
  fs::create_directories(dir_, error);
  for (const auto& entry : fs::directory_iterator(dir_, error)) {
    if (!entry.is_directory()) continue;
    const std::string id = entry.path().filename().string();
    std::optional<json> manifest = Manifest(id);
    if (!manifest || manifest->value("state", "") == "done") continue;
    // Left open by an earlier process: what it linked stays, without hashes it could not finish.
    (*manifest)["state"] = "done";
    (*manifest)["cut"] = true;
    std::string why;
    mowbite_hold::WriteFileSafely((fs::path(Folder(id)) / kManifest).string(), manifest->dump(1) + "\n", &why);
  }
  return Stored();
}

bool IncidentStore::WriteManifest(const Incident& incident, const IncidentContext& context, const std::string& state,
                                  const json& runs, std::string* error) {
  json triggers = json::array();
  for (const Trigger& trigger : incident.triggers) triggers.push_back(TriggerJson(trigger));
  bool from_start = !runs.empty();
  for (const auto& run : runs) from_start = from_start && run.value("from_start", false);
  const json manifest = {{"format", kManifestFormat},
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
  for (const auto& [name, run] : linked.items()) {
    (void)name;
    facts.push_back(run);
  }
  return WriteManifest(incident, context, state, facts, error) && ok;
}

bool IncidentStore::ReadyToSeal(const Incident& incident,
                                const std::vector<mowbite_journal::RunInfo>& runs) const {
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
  json facts = json::array();
  const auto it = runs_.find(incident.id);
  if (it != runs_.end()) {
    for (auto& [name, run] : it->second.items()) {
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
      json fact = run;
      fact["files"] = files;
      facts.push_back(fact);
    }
  }
  runs_.erase(incident.id);
  return WriteManifest(incident, context, "done", facts, error);
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

std::vector<std::string> IncidentStore::RunDirs(const std::string& id) const {
  std::vector<std::string> dirs;
  if (!Manifest(id)) return dirs;
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(Folder(id), ec)) {
    if (entry.is_directory() && entry.path().filename() != kAnonDir) dirs.push_back(entry.path().string());
  }
  std::sort(dirs.begin(), dirs.end());
  return dirs;
}

bool IncidentStore::Remove(const std::string& id, std::string* error) {
  if (!Manifest(id) && !fs::is_directory(Folder(id))) {
    *error = "no incident " + id;
    return false;
  }
  std::error_code ec;
  fs::remove_all(Folder(id), ec);
  runs_.erase(id);
  for (auto it = archive_sha_.begin(); it != archive_sha_.end();) {
    it = it->first.first == id ? archive_sha_.erase(it) : std::next(it);
  }
  if (ec) {
    *error = "cannot remove " + Folder(id) + ": " + ec.message();
    return false;
  }
  return true;
}

bool IncidentStore::Piece(const std::string& id, uint64_t offset, size_t size, bool location, IncidentPiece* piece,
                          std::string* error) {
  std::optional<json> manifest = Manifest(id);
  if (!manifest) {
    *error = "no incident " + id;
    return false;
  }
  if (manifest->value("state", "") != "done") {
    *error = "the incident is still collecting";
    return false;
  }
  const fs::path folder = Folder(id);
  if (!location) {
    // The copy without location, made once: the runs anonymized, the manifest without log lines and the date shifted.
    std::error_code ec;
    if (!fs::exists(folder / kAnonDir / kManifest, ec)) {
      for (const std::string& run : RunDirs(id)) {
        mowbite_replay::AnonymizeOptions options;
        options.shift_days = kIncidentShiftDays;
        if (!mowbite_replay::AnonymizeRun(run, (folder / kAnonDir / fs::path(run).filename()).string(), options,
                                          error)) {
          return false;
        }
      }
      json anon = *manifest;
      const double shift = kIncidentShiftDays * 86400.0;
      anon["created"] = anon.value("created", 0.0) + shift;
      anon["created_utc"] = IsoUtc(anon["created"].get<double>());
      anon.erase("log");
      for (auto& trigger : anon["triggers"]) {
        trigger.erase("utc");
        trigger.erase("detail");
      }
      for (auto& run : anon["runs"]) {
        run.erase("files");
        run.erase("boot_id");
      }
      anon["window"].erase("boot_id");
      anon["location"] = false;
      if (!mowbite_hold::WriteFileSafely((folder / kAnonDir / kManifest).string(), anon.dump(1) + "\n", error)) {
        return false;
      }
    }
  }
  const fs::path root = location ? folder : folder / kAnonDir;
  const auto mtime = static_cast<int64_t>(manifest->value("created", 0.0) + (location ? 0 : kIncidentShiftDays * 86400.0));
  std::vector<mowbite_journal::TarEntry> entries;
  mowbite_journal::TarEntry head;
  head.name = id + "/" + kManifest;
  head.path = (root / kManifest).string();
  std::error_code ec;
  head.size = fs::file_size(head.path, ec);
  head.mtime = mtime;
  entries.push_back(head);
  for (const std::string& file : Files(folder, !location)) {
    mowbite_journal::TarEntry entry;
    entry.name = id + "/" + file;
    entry.path = (root / file).string();
    entry.size = fs::file_size(entry.path, ec);
    entry.mtime = mtime;
    entries.push_back(entry);
  }
  const mowbite_journal::VirtualTar tar(std::move(entries));
  piece->size = tar.size();
  if (offset < tar.size() && !tar.Read(offset, size, &piece->data, error)) return false;
  if (offset >= tar.size()) piece->data.clear();
  std::string& sha = archive_sha_[{id, location}];
  if (sha.empty()) {
    Sha256 whole;
    std::string chunk;
    for (uint64_t at = 0; at < tar.size(); at += chunk.size()) {
      if (!tar.Read(at, 1 << 20, &chunk, error) || chunk.empty()) return false;
      whole.Add(chunk.data(), chunk.size());
    }
    sha = whole.Hex();
  }
  piece->archive_sha256 = sha;
  return true;
}

}  // namespace mowbite_stats
