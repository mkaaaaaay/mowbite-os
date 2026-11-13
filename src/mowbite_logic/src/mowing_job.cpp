#include "mowbite_logic/mowing_job.h"

#include <algorithm>
#include <cmath>

namespace mowbite_logic {

namespace {

constexpr int kCheckpointVersion = 1;

}  // namespace

bool IsMowKind(uint8_t kind) {
  return kind == kSegmentPerimeter || kind == kSegmentLane || kind == kSegmentTurn;
}

std::vector<JobPath> JobPaths(const CoverageAnswer& answer) {
  const size_t n = answer.poses.size();
  // The type of the stretch from each pose to the next; where two segments meet, the later one.
  std::vector<uint8_t> kinds(n, kSegmentTransit);
  const size_t count = std::min({answer.segment_start.size(), answer.segment_end.size(), answer.segment_type.size()});
  for (size_t s = 0; s < count; ++s) {
    const size_t first = answer.segment_start[s];
    const size_t last = std::min<size_t>(answer.segment_end[s], n);
    for (size_t i = first; i < last; ++i) kinds[i] = answer.segment_type[s];
  }
  const bool with_headings = answer.headings.size() == n;
  std::vector<JobPath> paths;
  for (const auto& piece : CutIntoPieces(answer)) {
    if (piece.last >= n) continue;
    JobPath path;
    path.is_outline = piece.is_outline;
    for (size_t i = piece.first; i <= piece.last; ++i) {
      JobPose pose;
      pose.x = answer.poses[i].x;
      pose.y = answer.poses[i].y;
      pose.kind = kinds[i];
      if (with_headings) {
        pose.heading = answer.headings[i];
      } else {
        const size_t a = i < piece.last ? i : i - 1;
        pose.heading = std::atan2(answer.poses[a + 1].y - answer.poses[a].y, answer.poses[a + 1].x - answer.poses[a].x);
      }
      path.poses.push_back(pose);
    }
    // The last pose ends the path, its kind is the one of the stretch before it.
    if (path.poses.size() > 1) path.poses.back().kind = path.poses[path.poses.size() - 2].kind;
    paths.push_back(std::move(path));
  }
  return paths;
}

std::string CheckpointText(const JobProgress& p) {
  json j = {{"version", kCheckpointVersion},
            {"job_id", p.job_id},
            {"area", p.area},
            {"area_id", p.area_id},
            {"path", p.path},
            {"index", p.index},
            {"fingerprint", p.fingerprint}};
  // A job over all areas keeps the text of before.
  if (!p.tasks.empty()) {
    j["tasks"] = p.tasks;
    j["task"] = p.task;
  }
  if (p.angle_offset_deg) j["angle_offset_deg"] = *p.angle_offset_deg;
  return j.dump();
}

std::optional<JobProgress> ParseCheckpoint(const std::string& text) {
  // Text that is no JSON object throws at the first key.
  const json j = json::parse(text, nullptr, false);
  JobProgress p;
  try {
    if (j.at("version").get<int>() != kCheckpointVersion) return std::nullopt;
    p.job_id = j.at("job_id").get<std::string>();
    p.area = j.at("area").get<int>();
    p.area_id = j.at("area_id").get<std::string>();
    p.path = j.at("path").get<int>();
    p.index = j.at("index").get<int>();
    p.fingerprint = j.at("fingerprint").get<std::string>();
    // Without tasks a job over all areas, as before them.
    if (j.contains("tasks")) {
      p.tasks = j.at("tasks").get<std::vector<std::string>>();
      p.task = j.at("task").get<int>();
      // Never all areas for a list that lost its ids.
      if (p.tasks.empty()) return std::nullopt;
    }
    if (j.contains("angle_offset_deg")) {
      p.angle_offset_deg = j.at("angle_offset_deg").get<double>();
      if (!std::isfinite(*p.angle_offset_deg)) return std::nullopt;
    }
  } catch (const json::exception&) {
    return std::nullopt;
  }
  if (p.area < 0 || p.path < 0 || p.index < 0) return std::nullopt;
  if (p.task < 0 || static_cast<size_t>(p.task) > p.tasks.size()) return std::nullopt;
  return p;
}

void MowingJob::SetAreas(std::vector<JobArea> areas) {
  areas_ = std::move(areas);
}

bool MowingJob::HasMowableArea(const std::vector<std::string>& tasks) const {
  return std::any_of(areas_.begin(), areas_.end(), [&tasks](const JobArea& a) {
    return a.mowable && (tasks.empty() || std::find(tasks.begin(), tasks.end(), a.id) != tasks.end());
  });
}

const JobArea* MowingJob::FindArea(const std::string& id) const {
  const auto area = std::find_if(areas_.begin(), areas_.end(), [&id](const JobArea& a) { return a.id == id; });
  return area == areas_.end() ? nullptr : &*area;
}

bool MowingJob::HasArea(const std::string& id) const {
  return FindArea(id) != nullptr;
}

std::vector<std::string> MowingJob::TaskIds(const std::vector<std::string>& tasks) const {
  if (!tasks.empty()) return tasks;
  std::vector<std::string> ids;
  for (const auto& area : areas_) {
    if (area.mowable) ids.push_back(area.id);
  }
  return ids;
}

void MowingJob::SetTasks(std::vector<std::string> tasks) {
  progress_.tasks = std::move(tasks);
  progress_.task = 0;
}

void MowingJob::SetAngle(std::optional<double> angle_offset_deg) {
  progress_.angle_offset_deg = angle_offset_deg;
}

int MowingJob::TaskIndex() const {
  if (!progress_.tasks.empty()) {
    return static_cast<size_t>(progress_.task) < progress_.tasks.size() ? progress_.task : -1;
  }
  const JobArea* area = CurrentArea();
  if (!area || !area->mowable) return -1;
  return static_cast<int>(std::count_if(areas_.begin(), areas_.begin() + progress_.area,
                                        [](const JobArea& a) { return a.mowable; }));
}

int MowingJob::TaskCount() const {
  return static_cast<int>(TaskIds(progress_.tasks).size());
}

void MowingJob::Restore(const JobProgress& progress) {
  progress_ = progress;
  paths_.clear();
  has_plan_ = false;
}

bool MowingJob::Unfinished() const {
  const int place = progress_.tasks.empty() ? progress_.area : progress_.task;
  return place > 0 || progress_.path > 0 || progress_.index > 0 || has_plan_;
}

void MowingJob::SetPassOver(std::vector<std::string> ids) {
  pass_over_ = std::move(ids);
}

void MowingJob::PassOver(const std::vector<std::string>& ids) {
  for (const auto& id : ids) {
    if (!PassedOver(id)) pass_over_.push_back(id);
  }
}

bool MowingJob::PassedOver(const std::string& id) const {
  return std::find(pass_over_.begin(), pass_over_.end(), id) != pass_over_.end();
}

bool MowingJob::AreaLeft() const {
  const auto to_mow = [this](const JobArea* area) { return area && area->mowable && !PassedOver(area->id); };
  if (progress_.tasks.empty()) {
    for (size_t at = static_cast<size_t>(progress_.area); at < areas_.size(); ++at) {
      if (to_mow(&areas_[at])) return true;
    }
    return false;
  }
  for (size_t at = static_cast<size_t>(progress_.task); at < progress_.tasks.size(); ++at) {
    if (to_mow(FindArea(progress_.tasks[at]))) return true;
  }
  return false;
}

bool MowingJob::SeekArea(std::string* notice, std::vector<JobArea>* passed) {
  if (!progress_.tasks.empty()) return SeekTask(notice, passed);
  const size_t at = static_cast<size_t>(progress_.area);
  if (!progress_.area_id.empty() && (at >= areas_.size() || areas_[at].id != progress_.area_id)) {
    if (notice) *notice = "The area " + progress_.area_id + " of the checkpoint is not where it was, starting over";
    progress_.area = 0;
    progress_.path = 0;
    progress_.index = 0;
    progress_.fingerprint.clear();
    paths_.clear();
    has_plan_ = false;
  }
  while (static_cast<size_t>(progress_.area) < areas_.size()) {
    const JobArea& area = areas_[progress_.area];
    if (area.mowable && !PassedOver(area.id)) break;
    if (area.mowable && passed) passed->push_back(area);
    NextArea();
  }
  if (static_cast<size_t>(progress_.area) >= areas_.size()) {
    progress_.area_id.clear();
    return false;
  }
  progress_.area_id = areas_[progress_.area].id;
  return true;
}

bool MowingJob::SeekTask(std::string* notice, std::vector<JobArea>* passed) {
  // Found by its id: the place in the map may have changed, the fingerprint of the plan tells about path and index.
  while (static_cast<size_t>(progress_.task) < progress_.tasks.size()) {
    const std::string& id = progress_.tasks[progress_.task];
    const JobArea* area = FindArea(id);
    if (area && area->mowable && PassedOver(id)) {
      if (passed) passed->push_back(*area);
      NextArea();
      continue;
    }
    if (area && area->mowable) {
      progress_.area = static_cast<int>(area - areas_.data());
      progress_.area_id = id;
      return true;
    }
    if (notice) {
      if (!notice->empty()) *notice += "; ";
      *notice += "The area " + id + " of the job is " + (area ? "not active" : "no longer on the map") +
                 ", passed over";
    }
    NextArea();
  }
  progress_.area_id.clear();
  return false;
}

const JobArea* MowingJob::CurrentArea() const {
  const size_t at = static_cast<size_t>(progress_.area);
  return at < areas_.size() ? &areas_[at] : nullptr;
}

void MowingJob::SetPlan(std::vector<JobPath> paths, const std::string& fingerprint) {
  if (fingerprint.empty() || fingerprint != progress_.fingerprint ||
      static_cast<size_t>(progress_.path) >= paths.size()) {
    progress_.path = 0;
    progress_.index = 0;
  }
  progress_.fingerprint = fingerprint;
  paths_ = std::move(paths);
  has_plan_ = true;
  if (paths_.empty()) NextArea();
}

const JobPath* MowingJob::Path(int number) const {
  if (!has_plan_ || number < 0) return nullptr;
  const size_t at = static_cast<size_t>(number);
  return at < paths_.size() ? &paths_[at] : nullptr;
}

const JobPath* MowingJob::CurrentPath() const {
  return Path(progress_.path);
}

const JobPose* MowingJob::CurrentPose() const {
  const JobPath* path = CurrentPath();
  if (!path) return nullptr;
  const size_t at = std::min(static_cast<size_t>(progress_.index), path->poses.size() - 1);
  return &path->poses[at];
}

void MowingJob::OnIndex(size_t index) {
  const JobPath* path = CurrentPath();
  if (!path) return;
  progress_.index = static_cast<int>(std::min(index, path->poses.size() - 1));
}

void MowingJob::PathDone() {
  if (!has_plan_) return;
  progress_.path++;
  progress_.index = 0;
  if (static_cast<size_t>(progress_.path) >= paths_.size()) NextArea();
}

void MowingJob::SkipPath() {
  PathDone();
}

void MowingJob::SkipArea() {
  NextArea();
}

void MowingJob::NextArea() {
  // With tasks the next area is the one of the next task.
  if (progress_.tasks.empty()) {
    progress_.area++;
  } else {
    progress_.task++;
  }
  progress_.area_id.clear();
  progress_.path = 0;
  progress_.index = 0;
  progress_.fingerprint.clear();
  paths_.clear();
  has_plan_ = false;
}

void MowingJob::Complete() {
  Reset();
}

void MowingJob::Reset() {
  progress_ = JobProgress();
  paths_.clear();
  has_plan_ = false;
}

}  // namespace mowbite_logic
