#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <utility>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/obstacle_part.h"

namespace mowbite_logic {

namespace {

constexpr int kZoneSlow = 1;
constexpr int kZoneStop = 2;
constexpr int kModeAvoid = 3;

// PERSON before ANIMAL before OBJECT before UNKNOWN.
int Weight(const std::string& kind) {
  if (kind == "PERSON") return 3;
  if (kind == "ANIMAL") return 2;
  if (kind == "OBJECT") return 1;
  return 0;
}

Reason ObstacleWhy(const std::string& worst_kind) {
  const std::string kind = worst_kind.empty() ? "UNKNOWN" : worst_kind;
  return Reason("OBSTACLE", "An obstacle is in the way (" + kind + ")").With("kind", kind);
}

// Corners of KeepOutRing.
constexpr int kKeepOutCorners = 24;

}  // namespace

Ring KeepOutRing(const AvoidCircle& circle, bool inner) {
  Ring ring;
  const double step = 2.0 * M_PI / kKeepOutCorners;
  const double reach = (inner ? circle.pass_r : circle.r) / std::cos(step / 2.0);
  for (int i = 0; i < kKeepOutCorners; ++i) {
    ring.push_back({circle.x + reach * std::cos(i * step), circle.y + reach * std::sin(i * step)});
  }
  return ring;
}

void ObstaclePart::SetMode(int setting, int mode_max) {
  setting_ = setting;
  mode_max_ = mode_max;
  if (!Armed()) {
    hold_ = false;
    free_since_.reset();
    waived_.clear();
    standing_.clear();
  }
  if (mode() < kModeAvoid) ClearCircles();
}

int ObstaclePart::mode() const {
  return mowbite_api::EffectiveObstacleMode(setting_, mode_max_);
}

std::vector<const ObstacleTrackSample*> ObstaclePart::Holders(const ObstacleSample& sample) const {
  std::vector<const ObstacleTrackSample*> holders;
  for (const auto& track : sample.tracks) {
    if (!track.confirmed || track.zone != kZoneStop) continue;
    if (waived_.count(track.id) && !track.in_view) continue;
    if (CircleOf(track)) continue;
    holders.push_back(&track);
  }
  return holders;
}

void ObstaclePart::Note(uint32_t id, const std::string& action) {
  if (noted_.insert({id, action}).second) notes_.push_back(ObstacleNote{id, action, 0.0, 0.0, "", ""});
}

bool ObstaclePart::Inside(const ObstacleTrackSample& track, const AvoidCircle& circle) const {
  const double reach = circle.r - config_.avoid_clearance + config_.avoid_still;
  return track.kind == "OBJECT" && track.confirmed &&
         std::hypot(track.x - circle.x, track.y - circle.y) + track.radius <= reach;
}

const AvoidCircle* ObstaclePart::CircleOf(const ObstacleTrackSample& track) const {
  for (const auto& circle : circles_) {
    if (Inside(track, circle)) return &circle;
  }
  return nullptr;
}

void ObstaclePart::DropCircle(size_t index) {
  // Its entry in the list loses the circle.
  notes_.push_back(ObstacleNote{circles_[index].track_id, "avoided", 0.0, 0.0, "", ""});
  circles_.erase(circles_.begin() + static_cast<std::ptrdiff_t>(index));
  circles_gone_ = true;
}

void ObstaclePart::ClearCircles() {
  while (!circles_.empty()) DropCircle(circles_.size() - 1);
}

void ObstaclePart::OnStatus(const ObstacleSample& sample, double now_s, bool driving) {
  // A circle goes once the track in it ended in view (its place seen free); out of view it stays without one. A
  // confirmed object inside a circle without a track is its track from then on.
  const auto find = [](const ObstacleSample& s, uint32_t id) {
    const auto it = std::find_if(s.tracks.begin(), s.tracks.end(), [id](const auto& t) { return t.id == id; });
    return it == s.tracks.end() ? nullptr : &*it;
  };
  for (size_t i = circles_.size(); i-- > 0;) {
    AvoidCircle& circle = circles_[i];
    if (circle.follows != 0 && !find(sample, circle.follows)) {
      const ObstacleTrackSample* before = find(last_, circle.follows);
      if (before && before->in_view) {
        DropCircle(i);
        continue;
      }
      circle.follows = 0;
    }
  }
  for (const auto& track : sample.tracks) {
    for (auto& circle : circles_) {
      if (circle.follows == 0 && Inside(track, circle)) circle.follows = track.id;
    }
  }
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
    std::map<uint32_t, Standing> standing;
    for (const auto* track : holders) {
      if (track->ahead < config_.near_ahead) near_ = true;
      if (kind_.empty() || Weight(track->kind) > Weight(kind_)) kind_ = track->kind;
      // A thing that moved more than avoid_still stands anew from where it is now.
      const auto was = standing_.find(track->id);
      const bool still = was != standing_.end() &&
                         std::hypot(track->x - was->second.x, track->y - was->second.y) <= config_.avoid_still;
      standing[track->id] = still ? was->second : Standing{track->x, track->y, now_s};
    }
    standing_ = std::move(standing);
    return;
  }
  if (!hold_) return;
  if (!sample.sight) {
    free_since_.reset();
    return;
  }
  if (!free_since_) free_since_ = now_s;
  if (now_s - *free_since_ >= (near_ ? config_.near_wait : config_.free_wait)) EndHold();
}

void ObstaclePart::EndHold() {
  hold_ = false;
  free_since_.reset();
  standing_.clear();
  avoid_refused_ = false;
}

void ObstaclePart::Tick(double now_s) {
  for (size_t i = circles_.size(); i-- > 0;) {
    if (now_s - circles_[i].since >= config_.avoid_keep) DropCircle(i);
  }
  if (!last_at_ || now_s - *last_at_ <= config_.stale) return;
  fresh_ = false;
  free_since_.reset();
}

Reason ObstaclePart::Why() const {
  return ObstacleWhy(kind_);
}

bool ObstaclePart::Waive() {
  const auto holders = Holders(last_);
  const bool seen = std::any_of(holders.begin(), holders.end(), [](const auto* t) { return t->in_view; });
  // A stale status tells nothing of where they are now.
  if (!holders.empty() && (!fresh_ || seen)) return false;
  for (const auto* track : holders) waived_.insert(track->id);
  EndHold();
  return true;
}

std::vector<const ObstacleTrackSample*> ObstaclePart::StartHolders(bool objects) const {
  std::vector<const ObstacleTrackSample*> counted;
  if (!hold_) return counted;
  for (const auto* track : Holders(last_)) {
    if (objects || Weight(track->kind) >= Weight("ANIMAL")) counted.push_back(track);
  }
  return counted;
}

bool ObstaclePart::WaiveForStart(bool objects) {
  const auto counted = StartHolders(objects);
  const bool seen = std::any_of(counted.begin(), counted.end(), [](const auto* t) { return t->in_view; });
  if (!counted.empty() && (!fresh_ || seen)) return false;
  for (const auto* track : counted) waived_.insert(track->id);
  if (hold_ && Holders(last_).empty()) EndHold();
  return true;
}

Reason ObstaclePart::StartProblem(bool objects) const {
  std::string kind;
  for (const auto* track : StartHolders(objects)) {
    if (kind.empty() || Weight(track->kind) > Weight(kind)) kind = track->kind;
  }
  return kind.empty() ? Reason() : ObstacleWhy(kind);
}

std::vector<AvoidCircle> ObstaclePart::AvoidDue(double now_s) const {
  if (mode() < kModeAvoid || !hold_ || !fresh_ || avoid_refused_) return {};
  const auto holders = Holders(last_);
  std::vector<AvoidCircle> circles;
  for (const auto* track : holders) {
    // Only objects; a person, an animal or a thing of no kind is waited for.
    if (track->kind != "OBJECT") return {};
    const auto standing = standing_.find(track->id);
    if (standing == standing_.end() || now_s - standing->second.since < config_.avoid_wait) return {};
    // The way around keeps the clearance beyond the spread, a lane passing it the pass clearance with the spread in.
    const double around = track->radius + config_.avoid_sigma * track->sigma + config_.avoid_clearance;
    const double pass = track->radius + config_.avoid_pass_clearance;
    circles.push_back({track->id, track->id, track->x, track->y, around, std::min(pass, around), now_s});
  }
  return circles;
}

bool ObstaclePart::StillDue(const std::vector<AvoidCircle>& circles) const {
  if (!last_at_) return false;
  const auto due = AvoidDue(*last_at_);
  if (due.size() != circles.size()) return false;
  return std::all_of(circles.begin(), circles.end(), [&](const AvoidCircle& circle) {
    return std::any_of(due.begin(), due.end(), [&](const AvoidCircle& now) {
      return now.track_id == circle.track_id &&
             std::hypot(now.x - circle.x, now.y - circle.y) <= config_.avoid_still;
    });
  });
}

void ObstaclePart::Avoid(const std::vector<AvoidCircle>& circles) {
  circles_.insert(circles_.end(), circles.begin(), circles.end());
  EndHold();
}

std::vector<ObstacleNote> ObstaclePart::TakeNotes() {
  return std::exchange(notes_, {});
}

}  // namespace mowbite_logic
