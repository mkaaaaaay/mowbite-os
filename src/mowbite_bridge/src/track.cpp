#include "mowbite_bridge/track.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "mowbite_bridge/json_rpc.h"
#include "mowbite_bridge/text.h"

namespace fs = std::filesystem;

namespace mowbite_bridge {

namespace {

// A point is kept once the line to the newest position misses a point between by more than this.
constexpr double kTolerance = 0.05;
constexpr std::size_t kMaxOpen = 80;
// Kept points of one job in memory; beyond, the job records no more.
constexpr std::size_t kMaxKept = 200000;
constexpr char kSuffix[] = ".jsonl";

bool SafeJobId(const std::string& id) {
  return !id.empty() && id.size() <= 128 &&
         std::all_of(id.begin(), id.end(), [](unsigned char c) { return std::isalnum(c) || c == '_' || c == '-'; });
}

std::string TextField(const Json& j, const char* key) {
  const auto it = j.find(key);
  return it != j.end() && it->is_string() ? it->get<std::string>() : "";
}

template <typename P>
double Miss(const P& a, const P& b, const P& q) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double length2 = dx * dx + dy * dy;
  double s = length2 > 0.0 ? ((q.x - a.x) * dx + (q.y - a.y) * dy) / length2 : 0.0;
  s = std::clamp(s, 0.0, 1.0);
  return std::hypot(a.x + s * dx - q.x, a.y + s * dy - q.y);
}

template <typename Points>
Json PointsJson(const Points& points, std::size_t from = 0) {
  Json out = Json::array();
  for (std::size_t i = from; i < points.size(); ++i) out.push_back(Json::array({points[i].x, points[i].y}));
  return out;
}

// Every step-th point and the last one.
template <typename Points>
Json ThinnedJson(const Points& points, std::size_t step) {
  Json out = Json::array();
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i % step == 0 || i + 1 == points.size()) out.push_back(Json::array({points[i].x, points[i].y}));
  }
  return out;
}

Json AttributesJson(const PoseAttributes& a) {
  return Json{{"job_id", a.job_id}, {"session_id", a.session_id}, {"blades", a.blades}};
}

bool SameAttributes(const PoseAttributes& a, const PoseAttributes& b) {
  return a.job_id == b.job_id && a.session_id == b.session_id && a.blades == b.blades;
}

}  // namespace

MowTrack::MowTrack(std::string dir, double max_days, std::uintmax_t max_bytes, std::size_t answer_bytes)
    : dir_(std::move(dir)), max_days_(max_days), max_bytes_(max_bytes), answer_bytes_(answer_bytes) {}

MowTrack::Run MowTrack::NewRun(double now) const {
  Run run;
  run.attributes = Attributes();
  run.started_at = now;
  return run;
}

bool MowTrack::Recording() const {
  return !job_id_.empty() && !session_id_.empty();
}

PoseAttributes MowTrack::Attributes() const {
  return {job_id_, session_id_, blades_};
}

std::vector<MowTrack::JobFile> MowTrack::Scan() const {
  std::vector<JobFile> files;
  std::error_code ec;
  if (dir_.empty() || !fs::is_directory(dir_, ec)) return files;
  for (const auto& entry : fs::directory_iterator(dir_, ec)) {
    const fs::path& path = entry.path();
    const std::string stem = path.stem().string();
    const auto cut = stem.find('_');
    if (path.extension() != kSuffix || cut == std::string::npos || cut == 0 || !entry.is_regular_file(ec)) continue;
    const std::string epoch = stem.substr(0, cut), job = stem.substr(cut + 1);
    const bool digits = std::all_of(epoch.begin(), epoch.end(), [](unsigned char c) { return std::isdigit(c); });
    if (epoch.size() > 15 || !digits || !SafeJobId(job)) {
      continue;
    }
    files.push_back({path.string(), job, std::stoll(epoch), entry.file_size(ec)});
  }
  std::sort(files.begin(), files.end(), [](const JobFile& a, const JobFile& b) {
    return a.epoch != b.epoch ? a.epoch < b.epoch : a.path < b.path;
  });
  return files;
}

std::vector<MowTrack::Run> MowTrack::Load(const std::string& path) {
  std::vector<Run> runs;
  // As many points as a job keeps in memory, whatever the file holds.
  std::size_t loaded = 0;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (NestingDepth(line) > 8) continue;
    const Json j = Json::parse(line, nullptr, false);
    if (!j.is_object()) continue;
    const std::string type = TextField(j, "type");
    if (type == "new_segment") {
      Run run;
      const Json a = j.value("attributes", Json::object());
      if (a.is_object()) {
        run.attributes = {TextField(a, "job_id"), TextField(a, "session_id"), a.value("blades", Json(false)) == true};
      }
      if (j.contains("started_at") && j["started_at"].is_number()) run.started_at = j["started_at"].get<double>();
      runs.push_back(std::move(run));
    } else if (type != "append_points" || runs.empty()) {
      continue;
    }
    const auto points = j.find("points");
    if (points == j.end() || !points->is_array()) continue;
    for (const auto& p : *points) {
      if (loaded >= kMaxKept) break;
      if (p.is_array() && p.size() == 2 && p[0].is_number() && p[1].is_number()) {
        runs.back().points.push_back({p[0].get<double>(), p[1].get<double>()});
        ++loaded;
      }
    }
  }
  for (auto& run : runs) {
    run.saved = run.points.size();
    run.in_file = true;
  }
  return runs;
}

Json MowTrack::Render(const std::vector<Run>& runs, const std::vector<Point>& open) const {
  std::size_t longest = open.size();
  for (const auto& run : runs) longest = std::max(longest, run.points.size());
  for (std::size_t step = 1;; step *= 2) {
    Json segments = Json::array();
    for (const auto& run : runs) {
      segments.push_back(
          Json{{"attributes", AttributesJson(run.attributes)}, {"points", ThinnedJson(run.points, step)}});
    }
    Json answer{{"segments", std::move(segments)}, {"buffer", ThinnedJson(open, step)}};
    if (step > 1) answer["thinned"] = true;
    if (step >= longest || answer.dump().size() < answer_bytes_) return answer;
  }
}

void MowTrack::Open(double now) {
  const auto files = Scan();
  if (!files.empty()) {
    job_id_ = files.back().job_id;
    file_ = files.back().path;
  }
  // The newest file stays, whatever its age.
  Trim(now);
  if (files.empty()) return;
  session_id_.clear();
  blades_ = false;
  runs_ = Load(file_);
  for (const auto& run : runs_) kept_ += run.points.size();
  // For the points after this start; it goes to the file only once it has some.
  runs_.push_back(NewRun(now));
}

void MowTrack::OnEvent(const Json& event, double now) {
  if (!event.is_object()) return;
  const std::string type = TextField(event, "type");
  if (type == "STATE") {
    const std::string job = TextField(event, "job_id");
    if (SafeJobId(job) && job != job_id_) {
      EndRun();
      Save();
      runs_.clear();
      kept_ = 0;
      job_id_ = job;
      const std::string name = std::to_string(static_cast<long long>(now)) + "_" + job + kSuffix;
      file_ = dir_.empty() ? "" : (fs::path(dir_) / name).string();
      Trim(now);
    }
    session_id_ = TextField(event, "session_id");
  } else if (type == "BLADES") {
    const auto enabled = event.find("enabled");
    if (enabled == event.end() || !enabled->is_boolean()) return;
    blades_ = enabled->get<bool>();
  } else {
    return;
  }
  if (!Recording()) {
    EndRun();
    return;
  }
  if (!runs_.empty() && SameAttributes(runs_.back().attributes, Attributes())) return;
  EndRun();
  Run run = NewRun(now);
  // A new blade state within a session goes on from where the last run ended.
  if (!runs_.empty() && runs_.back().attributes.session_id == session_id_ && !runs_.back().points.empty()) {
    run.points.push_back(runs_.back().points.back());
  }
  if (!runs_.empty() && runs_.back().points.empty() && !runs_.back().in_file) {
    runs_.back() = std::move(run);
  } else {
    runs_.push_back(std::move(run));
  }
}

void MowTrack::Keep(const Point& point) {
  if (kept_ >= kMaxKept) return;
  runs_.back().points.push_back(point);
  ++kept_;
}

void MowTrack::Add(double x, double y) {
  if (!Recording() || runs_.empty() || !std::isfinite(x) || !std::isfinite(y)) return;
  const Point point{x, y};
  if (runs_.back().points.empty()) {
    Keep(point);
    return;
  }
  open_.push_back(point);
  const Point& from = runs_.back().points.back();
  for (std::size_t i = 0; i + 1 < open_.size(); ++i) {
    if (Miss(from, point, open_[i]) > kTolerance) {
      // The point before still had a line that fits; it is kept and the newest begins the next stretch.
      Keep(open_[open_.size() - 2]);
      open_.erase(open_.begin(), open_.end() - 1);
      break;
    }
  }
  if (open_.size() >= kMaxOpen) {
    Keep(open_.back());
    open_.clear();
  }
}

void MowTrack::EndRun() {
  if (!open_.empty() && !runs_.empty()) Keep(open_.back());
  open_.clear();
}

bool MowTrack::Save() {
  if (file_.empty()) return true;
  std::string lines;
  std::vector<Run*> written;
  for (auto& run : runs_) {
    if (!run.in_file) {
      // A run with nothing in it is written only when it began with a session (not the one after a start).
      if (run.points.empty() && run.attributes.session_id.empty()) continue;
      Json line = {{"type", "new_segment"}, {"attributes", AttributesJson(run.attributes)},
                   {"started_at", run.started_at}, {"points", PointsJson(run.points)}};
      lines += line.dump() + "\n";
    } else if (run.saved < run.points.size()) {
      lines += Json{{"type", "append_points"}, {"points", PointsJson(run.points, run.saved)}}.dump() + "\n";
    } else {
      continue;
    }
    written.push_back(&run);
  }
  if (lines.empty()) return true;
  std::error_code ec;
  fs::create_directories(dir_, ec);
  // Only what went into the file counts as saved.
  if (!AppendLines(file_, lines)) return false;
  for (Run* run : written) {
    run->in_file = true;
    run->saved = run->points.size();
  }
  return true;
}

void MowTrack::Close() {
  EndRun();
  Save();
}

void MowTrack::Trim(double now) {
  auto files = Scan();
  std::uintmax_t total = 0;
  for (const auto& f : files) total += f.bytes;
  const double oldest = now - max_days_ * 86400.0;
  std::error_code ec;
  for (const auto& f : files) {
    if (f.path == file_) continue;
    if (static_cast<double>(f.epoch) >= oldest && total <= max_bytes_) continue;
    if (fs::remove(f.path, ec)) total -= f.bytes;
  }
}

Json MowTrack::History(const Json& params) const {
  std::string job = AskedName(params, "job_id", SafeJobId);
  if (job.empty()) job = job_id_;
  if (job.empty()) return Render({}, {});
  if (job == job_id_ && !runs_.empty()) return Render(runs_, open_);
  const auto files = Scan();
  for (auto f = files.rbegin(); f != files.rend(); ++f) {
    if (f->job_id == job) return Render(Load(f->path), {});
  }
  if (job == job_id_) return Render({}, {});
  return Json{{"error", "not found"}};
}

Json MowTrack::List() const {
  Json jobs = Json::array();
  const auto files = Scan();
  for (auto f = files.rbegin(); f != files.rend(); ++f) {
    jobs.push_back(Json{{"job_id", f->job_id}, {"timestamp", f->epoch}});
  }
  return jobs;
}

Json MowTrack::Erase(const Json& params) {
  const std::string job = AskedName(params, "job_id", SafeJobId);
  if (job == "-") throw std::invalid_argument("params: none for all, or {\"job_id\": a job}");
  int deleted = 0;
  std::error_code ec;
  for (const auto& f : Scan()) {
    if (f.path == file_ || f.job_id == job_id_) continue;
    if ((job.empty() || f.job_id == job) && fs::remove(f.path, ec)) ++deleted;
  }
  return Json{{"deleted", deleted}};
}

}  // namespace mowbite_bridge
