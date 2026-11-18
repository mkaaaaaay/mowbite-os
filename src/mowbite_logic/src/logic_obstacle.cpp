#include <algorithm>
#include <iterator>
#include <utility>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/obstacle_part.h"

namespace mowbite_logic {

namespace {

constexpr int kZoneSlow = 1;
constexpr int kZoneStop = 2;

// PERSON before ANIMAL before OBJECT before UNKNOWN.
int Weight(const std::string& kind) {
  if (kind == "PERSON") return 3;
  if (kind == "ANIMAL") return 2;
  if (kind == "OBJECT") return 1;
  return 0;
}

}  // namespace

void ObstaclePart::SetMode(int setting, int mode_max) {
  setting_ = setting;
  mode_max_ = mode_max;
  if (!Armed()) {
    hold_ = false;
    free_since_.reset();
    waived_.clear();
  }
}

int ObstaclePart::mode() const {
  return mowbite_api::EffectiveObstacleMode(setting_, mode_max_);
}

std::vector<const ObstacleTrackSample*> ObstaclePart::Holders(const ObstacleSample& sample) const {
  std::vector<const ObstacleTrackSample*> holders;
  for (const auto& track : sample.tracks) {
    if (!track.confirmed || track.zone != kZoneStop) continue;
    if (waived_.count(track.id) && !track.in_view) continue;
    holders.push_back(&track);
  }
  return holders;
}

void ObstaclePart::Note(uint32_t id, const std::string& action) {
  if (noted_.insert({id, action}).second) notes_.push_back({id, action});
}

void ObstaclePart::OnStatus(const ObstacleSample& sample, double now_s, bool driving) {
  last_ = sample;
  last_at_ = now_s;
  fresh_ = true;
  // A waived thing counts again once a sensor sees it; forgotten tracks are forgotten here too.
  for (auto it = waived_.begin(); it != waived_.end();) {
    const auto track = std::find_if(sample.tracks.begin(), sample.tracks.end(),
                                    [&it](const ObstacleTrackSample& t) { return t.id == *it; });
    it = track == sample.tracks.end() || track->in_view ? waived_.erase(it) : std::next(it);
  }
  for (auto it = noted_.begin(); it != noted_.end();) {
    const bool kept = std::any_of(sample.tracks.begin(), sample.tracks.end(),
                                  [&it](const ObstacleTrackSample& t) { return t.id == it->first; });
    it = kept ? std::next(it) : noted_.erase(it);
  }
  slow_ = std::any_of(sample.tracks.begin(), sample.tracks.end(),
                      [](const ObstacleTrackSample& t) { return t.confirmed && t.zone == kZoneSlow; });
  const auto holders = Holders(sample);
  if (driving && mode() >= 1) {
    for (const auto* track : holders) Note(track->id, Armed() ? "stopped" : "would_stop");
    if (Armed()) {
      for (const auto& track : sample.tracks) {
        if (track.confirmed && track.zone == kZoneSlow) Note(track.id, "slowed");
      }
    }
  }
  if (!Armed()) return;
  if (!holders.empty()) {
    if (!hold_) {
      hold_ = true;
      kind_.clear();
      near_ = false;
    }
    free_since_.reset();
    for (const auto* track : holders) {
      if (track->ahead < config_.near_ahead) near_ = true;
      if (kind_.empty() || Weight(track->kind) > Weight(kind_)) kind_ = track->kind;
    }
    return;
  }
  if (!hold_) return;
  if (!sample.sight) {
    free_since_.reset();
    return;
  }
  if (!free_since_) free_since_ = now_s;
  if (now_s - *free_since_ >= (near_ ? config_.near_wait : config_.free_wait)) {
    hold_ = false;
    free_since_.reset();
  }
}

void ObstaclePart::Tick(double now_s) {
  if (!last_at_ || now_s - *last_at_ <= config_.stale) return;
  fresh_ = false;
  free_since_.reset();
}

Reason ObstaclePart::Why() const {
  const std::string kind = kind_.empty() ? "UNKNOWN" : kind_;
  return Reason("OBSTACLE", "An obstacle is in the way (" + kind + ")").With("kind", kind);
}

bool ObstaclePart::Waive() {
  const auto holders = Holders(last_);
  if (fresh_ && std::any_of(holders.begin(), holders.end(), [](const auto* t) { return t->in_view; })) return false;
  for (const auto* track : holders) waived_.insert(track->id);
  hold_ = false;
  free_since_.reset();
  return true;
}

std::vector<ObstacleNote> ObstaclePart::TakeNotes() {
  return std::exchange(notes_, {});
}

}  // namespace mowbite_logic
