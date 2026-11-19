#include "mowbite_obstacles/tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mowbite_obstacles/ground.h"

namespace mowbite_obstacles {

namespace {

constexpr double kTwoPi = 2.0 * M_PI;

// An angle within [0, 2 pi).
double Turn(double angle) {
  angle = std::fmod(angle, kTwoPi);
  return angle < 0.0 ? angle + kTwoPi : angle;
}

// 1 sigma along the larger axis of a covariance.
double LargerSigma(double xx, double xy, double yy) {
  const double mean = 0.5 * (xx + yy);
  const double half = std::sqrt(0.25 * (xx - yy) * (xx - yy) + xy * xy);
  return std::sqrt(std::max(0.0, mean + half));
}

// A find of any kind at the place of a person or animal keeps that track: a model that sees only legs from close by
// must not end it.
bool Compatible(Kind track, Kind find) {
  return track == find || track == Kind::kUnknown || find == Kind::kUnknown || Living(track);
}

// PERSON before ANIMAL before OBJECT before UNKNOWN: a track takes the kind of a find above its own, never one below.
// A find cut at the lower edge of the image as a disc from its place down to where the view begins, along the ray of
// the sensor: how near it really is stays unknown.
Find Reaching(const Find& find, const ViewSector& view) {
  if (!find.cut_bottom) return find;
  const double dx = find.x - view.x, dy = find.y - view.y;
  const double far = std::hypot(dx, dy);
  if (!(far > view.range_min)) return find;
  const double nx = view.x + dx / far * view.range_min, ny = view.y + dy / far * view.range_min;
  Find reaching = find;
  reaching.x = 0.5 * (nx + find.x);
  reaching.y = 0.5 * (ny + find.y);
  reaching.radius = find.radius + 0.5 * (far - view.range_min);
  return reaching;
}

int Rank(Kind kind) {
  switch (kind) {
    case Kind::kPerson:
      return 3;
    case Kind::kAnimal:
      return 2;
    case Kind::kObject:
      return 1;
    case Kind::kUnknown:
      break;
  }
  return 0;
}

double Interpolate(double a, double b, double f) {
  return a + (b - a) * f;
}

double InterpolateAngle(double a, double b, double f) {
  return a + std::remainder(b - a, kTwoPi) * f;
}

}  // namespace

bool ParseKind(const std::string& name, Kind* kind) {
  if (name == "PERSON") *kind = Kind::kPerson;
  else if (name == "ANIMAL") *kind = Kind::kAnimal;
  else if (name == "OBJECT") *kind = Kind::kObject;
  else if (name == "UNKNOWN") *kind = Kind::kUnknown;
  else return false;
  return true;
}

const char* KindName(Kind kind) {
  switch (kind) {
    case Kind::kPerson:
      return "PERSON";
    case Kind::kAnimal:
      return "ANIMAL";
    case Kind::kObject:
      return "OBJECT";
    case Kind::kUnknown:
      break;
  }
  return "UNKNOWN";
}

bool Living(Kind kind) {
  return kind == Kind::kPerson || kind == Kind::kAnimal;
}

bool Inside(const ViewSector& view, double x, double y, double radius) {
  const double dx = x - view.x, dy = y - view.y;
  const double distance = std::hypot(dx, dy);
  if (distance - radius < view.range_min || distance + radius > view.range_max) return false;
  if (radius >= distance) return false;
  const double margin = std::asin(radius / distance);
  const double span = Turn(view.angle_left - view.angle_right);
  const double at = Turn(std::atan2(dy, dx) - view.angle_right);
  return at - margin >= 0.0 && at + margin <= span;
}

Tracker::Tracker(TrackerConfig config) : config_(config) {}

void Tracker::SetSources(const std::vector<std::string>& sources) {
  for (const std::string& source : sources) sensors_[source];
  for (auto it = sensors_.begin(); it != sensors_.end();) {
    it = std::find(sources.begin(), sources.end(), it->first) == sources.end() ? sensors_.erase(it) : ++it;
  }
  sources_ = sources;
}

void Tracker::OnPose(const Pose& pose) {
  if (!std::isfinite(pose.stamp) || !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.heading) ||
      !std::isfinite(pose.accuracy) || !std::isfinite(pose.heading_accuracy)) {
    return;
  }
  if (!poses_.empty() && pose.stamp <= poses_.back().stamp) {
    // A pose from the past (another clock, a restart of the localization) starts the history over.
    if (pose.stamp < poses_.back().stamp - config_.pose_keep) poses_.clear();
    else return;
  }
  poses_.push_back(pose);
  while (!poses_.empty() && poses_.front().stamp < pose.stamp - config_.pose_keep) poses_.pop_front();
}

bool Tracker::PoseAt(double stamp, Pose* pose) const {
  if (poses_.empty()) return false;
  if (stamp >= poses_.back().stamp) {
    if (stamp - poses_.back().stamp > config_.pose_gap) return false;
    *pose = poses_.back();
    return true;
  }
  if (stamp <= poses_.front().stamp) {
    if (poses_.front().stamp - stamp > config_.pose_gap) return false;
    *pose = poses_.front();
    return true;
  }
  const auto after = std::lower_bound(poses_.begin(), poses_.end(), stamp,
                                      [](const Pose& p, double t) { return p.stamp < t; });
  const Pose& b = *after;
  const Pose& a = *(after - 1);
  if (std::min(stamp - a.stamp, b.stamp - stamp) > config_.pose_gap) return false;
  const double f = (stamp - a.stamp) / (b.stamp - a.stamp);
  *pose = {stamp, Interpolate(a.x, b.x, f), Interpolate(a.y, b.y, f), InterpolateAngle(a.heading, b.heading, f),
           std::max(a.accuracy, b.accuracy), std::max(a.heading_accuracy, b.heading_accuracy)};
  return true;
}

double Tracker::MinScore(Kind kind) const {
  switch (kind) {
    case Kind::kPerson:
      return config_.min_score_person;
    case Kind::kAnimal:
      return config_.min_score_animal;
    case Kind::kObject:
      return config_.min_score_object;
    case Kind::kUnknown:
      break;
  }
  return 0.0;
}

void Tracker::Update(Track& track, const Find& find, const Frame& frame, double fx, double fy, double cxx, double cxy,
                     double cyy) {
  const double speed = Living(track.kind) || Living(find.kind) ? config_.living_speed : config_.object_speed;
  const double moved = speed * std::max(0.0, frame.stamp - track.last_seen);
  const Fused fused = Fuse({track.x, track.y, track.cov_xx + moved * moved, track.cov_xy, track.cov_yy + moved * moved},
                           {fx, fy, cxx, cxy, cyy}, std::numeric_limits<double>::infinity());
  track.x = fused.spot.x;
  track.y = fused.spot.y;
  track.cov_xx = fused.spot.cov_xx;
  track.cov_xy = fused.spot.cov_xy;
  track.cov_yy = fused.spot.cov_yy;
  const bool moving = Living(track.kind) || Living(find.kind);
  // A cut find never makes a thing smaller.
  track.radius = moving ? find.radius
                 : find.cut_bottom ? std::max(track.radius, find.radius)
                                   : 0.5 * (track.radius + find.radius);
  if (std::isfinite(find.height)) {
    track.height = moving || !std::isfinite(track.height) ? find.height : 0.5 * (track.height + find.height);
  }
  if (Rank(find.kind) > Rank(track.kind)) {
    track.kind = find.kind;
    track.class_name = find.class_name;
    track.score = find.score;
  } else if (find.kind == track.kind && find.score > track.score) {
    track.class_name = find.class_name;
    track.score = find.score;
  }
  track.last_seen = frame.stamp;
  track.free_since = -1.0;
  track.in_view = true;
  track.viewed_by = frame.source;
  track.window.push_back(true);
  while (static_cast<int>(track.window.size()) > config_.confirm_frames) track.window.pop_front();
  Shoot(track, find, frame);
}

void Tracker::Shoot(Track& track, const Find& find, const Frame& frame) const {
  track.source = frame.source;
  if (find.box_w <= 0.0 || find.box_h <= 0.0) return;
  track.shots.push_back({frame.source, frame.stamp, find.box_x, find.box_y, find.box_w, find.box_h});
  while (track.shots.size() > config_.shots) track.shots.pop_front();
}

FrameResult Tracker::OnFrame(const Frame& frame, double now) {
  FrameResult result;
  const auto sensor = sensors_.find(frame.source);
  if (sensor == sensors_.end()) return result;
  sensor->second.heard = true;
  sensor->second.last = now;
  sensor->second.sight = frame.sight;
  sensor->second.reason = frame.sight ? "" : frame.reason;
  if (!frame.sight) return result;
  Pose pose;
  if (!PoseAt(frame.stamp, &pose)) {
    // A frame that cannot be put into the map sees nothing.
    sensor->second.sight = false;
    sensor->second.reason = "NO_SIGNAL";
    result.no_pose = true;
    return result;
  }
  result.used = true;
  const double c = std::cos(pose.heading), s = std::sin(pose.heading);
  const double accuracy2 = pose.accuracy * pose.accuracy;

  // The finds that count, best first, in the map.
  struct Mapped {
    const Find* find;
    double x, y, cxx, cxy, cyy;
  };
  std::vector<Find> reaching;
  reaching.reserve(frame.finds.size());
  for (const Find& find : frame.finds) reaching.push_back(Reaching(find, frame.view));
  std::vector<Mapped> finds;
  for (const Find& find : reaching) {
    if (find.score < MinScore(find.kind)) continue;
    // The error of the heading moves a find by its distance from base_link times that angle.
    const double turned = pose.heading_accuracy * std::hypot(find.x, find.y);
    const double spread2 = accuracy2 + turned * turned;
    finds.push_back({&find, pose.x + c * find.x - s * find.y, pose.y + s * find.x + c * find.y,
                     c * c * find.cov_xx - 2.0 * c * s * find.cov_xy + s * s * find.cov_yy + spread2,
                     c * s * (find.cov_xx - find.cov_yy) + (c * c - s * s) * find.cov_xy,
                     s * s * find.cov_xx + 2.0 * c * s * find.cov_xy + c * c * find.cov_yy + spread2});
  }
  std::stable_sort(finds.begin(), finds.end(),
                   [](const Mapped& a, const Mapped& b) { return a.find->score > b.find->score; });

  std::vector<uint32_t> matched;
  for (const Mapped& mapped : finds) {
    Track* best = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();
    for (auto& [id, track] : tracks_) {
      if (!Compatible(track.kind, mapped.find->kind)) continue;
      if (std::find(matched.begin(), matched.end(), id) != matched.end()) continue;
      const double speed = Living(track.kind) || Living(mapped.find->kind) ? config_.living_speed : config_.object_speed;
      const double moved = speed * std::max(0.0, frame.stamp - track.last_seen);
      const double spread = LargerSigma(track.cov_xx, track.cov_xy, track.cov_yy) + moved +
                            LargerSigma(mapped.cxx, mapped.cxy, mapped.cyy);
      const double gate = std::min(config_.match_max, config_.match_distance + 2.0 * spread);
      const double distance = std::hypot(track.x - mapped.x, track.y - mapped.y);
      if (distance <= gate && distance < best_distance) {
        best = &track;
        best_distance = distance;
      }
    }
    if (!best) {
      Track track;
      track.id = next_id_++;
      track.kind = mapped.find->kind;
      track.first_seen = frame.stamp;
      track.last_seen = frame.stamp;
      track.x = mapped.x;
      track.y = mapped.y;
      track.cov_xx = mapped.cxx;
      track.cov_xy = mapped.cxy;
      track.cov_yy = mapped.cyy;
      track.radius = mapped.find->radius;
      track.height = mapped.find->height;
      track.class_name = mapped.find->class_name;
      track.score = mapped.find->score;
      track.in_view = true;
      track.viewed_by = frame.source;
      track.window.push_back(true);
      Shoot(track, *mapped.find, frame);
      if (config_.confirm_hits <= 1) {
        track.confirmed = true;
        result.confirmed.push_back(track.id);
      }
      matched.push_back(track.id);
      tracks_.emplace(track.id, std::move(track));
      continue;
    }
    const bool was_confirmed = best->confirmed;
    Update(*best, *mapped.find, frame, mapped.x, mapped.y, mapped.cxx, mapped.cxy, mapped.cyy);
    matched.push_back(best->id);
    if (!was_confirmed && std::count(best->window.begin(), best->window.end(), true) >= config_.confirm_hits) {
      best->confirmed = true;
      result.confirmed.push_back(best->id);
    }
  }

  // The tracks this frame did not find: a miss where their place was in view.
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    Track& track = it->second;
    if (std::find(matched.begin(), matched.end(), track.id) != matched.end()) {
      ++it;
      continue;
    }
    const double dx = track.x - pose.x, dy = track.y - pose.y;
    const double spare = config_.view_margin + 2.0 * LargerSigma(track.cov_xx, track.cov_xy, track.cov_yy);
    if (!Inside(frame.view, c * dx + s * dy, -s * dx + c * dy, track.radius + spare)) {
      if (track.viewed_by.empty() || track.viewed_by == frame.source) {
        track.in_view = false;
        track.free_since = -1.0;
        track.viewed_by.clear();
      }
      ++it;
      continue;
    }
    track.in_view = true;
    track.viewed_by = frame.source;
    track.window.push_back(false);
    while (static_cast<int>(track.window.size()) > config_.confirm_frames) track.window.pop_front();
    if (track.free_since < 0.0) track.free_since = frame.stamp;
    const auto misses = std::count(track.window.begin(), track.window.end(), false);
    const bool hopeless = !track.confirmed && misses > config_.confirm_frames - config_.confirm_hits;
    const bool seen_free = track.confirmed && misses == static_cast<long>(track.window.size()) &&
                           frame.stamp - track.free_since >= config_.free_seen;
    if (hopeless || seen_free) {
      result.ended.push_back(track.id);
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
  return result;
}

std::vector<uint32_t> Tracker::Tick(double now) {
  std::vector<uint32_t> ended;
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    Track& track = it->second;
    const double limit = track.confirmed ? config_.track_timeout : config_.candidate_timeout;
    track.stopping = track.confirmed && !poses_.empty() && ViewFrom(track, poses_.back()).zone == Zone::kStop;
    if (now - track.last_seen > limit && !track.stopping) {
      ended.push_back(it->first);
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
  return ended;
}

TrackView Tracker::ViewFrom(const Track& track, const Pose& pose) const {
  const double c = std::cos(pose.heading), s = std::sin(pose.heading);
  const double dx = track.x - pose.x, dy = track.y - pose.y;
  const double bx = c * dx + s * dy, by = -s * dx + c * dy;
  TrackView view;
  view.track = &track;
  view.sigma = LargerSigma(track.cov_xx, track.cov_xy, track.cov_yy);
  const double reach = track.radius + config_.zone_sigma * view.sigma;
  view.ahead = bx - reach - body_.front;
  view.aside = by;
  if (!track.confirmed) return view;
  const double release = track.stopping ? config_.zone_release : 0.0;
  // How far the thing stays off the body sideways, and whether any of it is level with the body or ahead.
  const double beside = std::abs(by) - reach - body_.half_width;
  const bool not_behind = bx + reach >= -body_.rear;
  if (Living(track.kind)) {
    const double off_x = std::max({-body_.rear - bx, 0.0, bx - body_.front});
    const double off_y = std::max(std::abs(by) - body_.half_width, 0.0);
    const bool around = std::hypot(off_x, off_y) - reach < config_.stop_living_around + release;
    const bool corridor = not_behind && beside <= config_.stop_living_margin;
    if (around || (not_behind && beside <= config_.stop_living_margin + release &&
                   view.ahead <= config_.stop_living_ahead + release)) {
      view.zone = Zone::kStop;
    } else if (corridor && view.ahead <= config_.slow_ahead) {
      view.zone = Zone::kSlow;
    }
  } else if (track.kind == Kind::kObject || config_.unknown_stops) {
    if (not_behind && beside <= config_.stop_object_margin + release &&
        view.ahead <= config_.stop_object_ahead + release) {
      view.zone = Zone::kStop;
    }
  }
  return view;
}

const Track* Tracker::TrackOf(uint32_t id) const {
  const auto it = tracks_.find(id);
  return it == tracks_.end() ? nullptr : &it->second;
}

Status Tracker::StatusAt(double now) const {
  Status status;
  for (const std::string& source : sources_) {
    const auto sensor = sensors_.find(source);
    const bool fresh = sensor != sensors_.end() && sensor->second.heard && now - sensor->second.last <= config_.stale;
    if (fresh && sensor->second.sight) continue;
    status.sight = false;
    status.source = source;
    status.reason = !fresh ? "NO_SIGNAL" : sensor->second.reason;
    break;
  }
  if (status.sight && !sources_.empty() && !body_known_) {
    status.sight = false;
    status.reason = "NOT_SET_UP";
  }
  // The zones need a recent pose.
  if (status.sight && !sources_.empty() && (poses_.empty() || now - poses_.back().stamp > config_.stale)) {
    status.sight = false;
    status.reason = "NO_SIGNAL";
  }
  if (poses_.empty()) return status;
  double stop_ahead = std::numeric_limits<double>::infinity();
  for (const auto& [id, track] : tracks_) {
    const TrackView view = ViewFrom(track, poses_.back());
    status.tracks.push_back(view);
    if (view.zone == Zone::kSlow) status.slow = true;
    if (view.zone == Zone::kStop && view.ahead < stop_ahead) {
      stop_ahead = view.ahead;
      status.stop = true;
      status.stop_id = id;
      status.stop_kind = track.kind;
    }
  }
  return status;
}

}  // namespace mowbite_obstacles
