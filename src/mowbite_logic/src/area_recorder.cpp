#include "mowbite_logic/area_recorder.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mowbite_logic {

namespace {

// Indices into ActionNames(), the order OpenMower registers them in.
enum ActionIndex {
  kStartRecording,
  kStopRecording,
  kFinishNavigationArea,
  kFinishMowingArea,
  kExitRecordingMode,
  kFinishDiscard,
  kRecordDock,
  kAutoPointsEnable,
  kAutoPointsDisable,
  kCollectPoint,
  kStartManualMowing,
  kStopManualMowing,
  kActionCount
};

std::string Metres(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.2f", value);
  return text;
}

// > 0: c lies left of the line from a to b.
double Cross(const Point2& a, const Point2& b, const Point2& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// c lies on the segment from a to b, given the three are on one line.
bool Within(const Point2& a, const Point2& b, const Point2& c) {
  return std::min(a.x, b.x) <= c.x && c.x <= std::max(a.x, b.x) && std::min(a.y, b.y) <= c.y &&
         c.y <= std::max(a.y, b.y);
}

int Side(double cross) {
  return (cross > 0) - (cross < 0);
}

bool SegmentsTouch(const Point2& a, const Point2& b, const Point2& c, const Point2& d) {
  const int d1 = Side(Cross(a, b, c));
  const int d2 = Side(Cross(a, b, d));
  const int d3 = Side(Cross(c, d, a));
  const int d4 = Side(Cross(c, d, b));
  if (d1 * d2 < 0 && d3 * d4 < 0) return true;
  // An end on the other segment counts as touching.
  if (d1 == 0 && Within(a, b, c)) return true;
  if (d2 == 0 && Within(a, b, d)) return true;
  if (d3 == 0 && Within(c, d, a)) return true;
  if (d4 == 0 && Within(c, d, b)) return true;
  return false;
}

}  // namespace

double RingArea(const Ring& ring) {
  double twice = 0;
  for (size_t i = 0; i < ring.size(); i++) {
    const Point2& a = ring[i];
    const Point2& b = ring[(i + 1) % ring.size()];
    twice += a.x * b.y - b.x * a.y;
  }
  return std::fabs(twice) / 2;
}

bool SelfIntersects(const Ring& ring) {
  const size_t n = ring.size();
  if (n < 4) return false;
  for (size_t i = 0; i < n; i++) {
    for (size_t j = i + 2; j < n; j++) {
      // The last segment ends where the first begins.
      if (i == 0 && j == n - 1) continue;
      if (SegmentsTouch(ring[i], ring[i + 1], ring[j], ring[(j + 1) % n])) return true;
    }
  }
  return false;
}

std::string RingProblem(const Ring& ring, double min_area_m2) {
  if (ring.size() < 3) return "a ring needs at least 3 points, it has " + std::to_string(ring.size());
  if (SelfIntersects(ring)) return "the line crosses itself";
  const double area = RingArea(ring);
  if (area < min_area_m2) return "it encloses only " + Metres(area) + " m², at least " + Metres(min_area_m2) + " needed";
  return "";
}

AreaRecorder::AreaRecorder(const RecorderConfig& config) : config_(config) {}

const std::vector<std::string>& AreaRecorder::ActionNames() {
  static const std::vector<std::string> names = {"start_recording",
                                                 "stop_recording",
                                                 "finish_navigation_area",
                                                 "finish_mowing_area",
                                                 "exit_recording_mode",
                                                 "finish_discard",
                                                 "record_dock",
                                                 "auto_point_collecting_enable",
                                                 "auto_point_collecting_disable",
                                                 "collect_point",
                                                 "start_manual_mowing",
                                                 "stop_manual_mowing"};
  return names;
}

void AreaRecorder::Reset() {
  recording_ = false;
  current_.clear();
  outline_.reset();
  obstacles_.clear();
  collect_requested_ = false;
  accuracy_blocked_ = false;
  full_reported_ = false;
  actions_changed_ = true;
}

AreaRecorder::Result AreaRecorder::Refuse(const std::string& action, const std::string& reason) {
  events_.push_back({"RECORDING_REFUSED", {{"action", action}, {"reason", reason}}});
  return Result::kRefused;
}

AreaRecorder::Result AreaRecorder::OnAction(const std::string& name) {
  const auto& names = ActionNames();
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end() || name == "exit_recording_mode") return Result::kUnknown;
  const size_t index = static_cast<size_t>(it - names.begin());
  // The edit on its way decides what comes next; the app shows no buttons until then.
  if (pending_) return Refuse(name, "The last area is still being saved");
  if (!Enabled()[index]) return Result::kRefused;
  actions_changed_ = true;
  switch (index) {
    case kStartRecording:
      recording_ = true;
      current_.clear();
      collect_requested_ = false;
      notices_.push_back(outline_ ? "Recording an obstacle" : "Recording an outline");
      return Result::kDone;
    case kStopRecording: return CloseRing(name) ? Result::kDone : Result::kRefused;
    case kFinishNavigationArea: return Finish(name, "nav");
    case kFinishMowingArea: return Finish(name, "mow");
    case kFinishDiscard:
      Reset();
      notices_.push_back("Discarded the area being recorded");
      return Result::kDone;
    case kAutoPointsEnable:
    case kAutoPointsDisable:
      auto_points_ = index == kAutoPointsEnable;
      collect_requested_ = false;
      return Result::kDone;
    case kCollectPoint:
      if (!pose_) return Refuse(name, "No position");
      // The next pose, so the point is where the mower is now.
      collect_requested_ = true;
      return Result::kDone;
    default: return Result::kRefused;
  }
}

bool AreaRecorder::CloseRing(const std::string& action) {
  const std::string problem = RingProblem(current_, config_.min_area);
  if (!problem.empty()) {
    // The ring stays open: the mower can drive on and close it better.
    Refuse(action, "Cannot close the ring: " + problem);
    return false;
  }
  if (outline_) {
    obstacles_.push_back(current_);
  } else {
    outline_ = current_;
  }
  notices_.push_back("Closed the " + std::string(obstacles_.empty() ? "outline" : "obstacle") + " with " +
                     std::to_string(current_.size()) + " points");
  recording_ = false;
  current_.clear();
  collect_requested_ = false;
  return true;
}

AreaRecorder::Result AreaRecorder::Finish(const std::string& action, const std::string& type) {
  if (!map_writable_) return Refuse(action, map_reason_);
  if (recording_ && !CloseRing(action)) return Result::kRefused;
  if (!outline_) return Refuse(action, "No outline recorded");
  MapEdit edit;
  edit.type = type;
  edit.outline = *outline_;
  edit.obstacles = obstacles_;
  pending_ = std::move(edit);
  edit_untaken_ = true;
  return Result::kDone;
}

std::optional<MapEdit> AreaRecorder::TakeMapEdit() {
  if (!edit_untaken_) return std::nullopt;
  edit_untaken_ = false;
  return pending_;
}

void AreaRecorder::OnMapEditResult(bool ok, const std::string& id, const std::string& message) {
  if (!pending_) return;
  const MapEdit edit = std::move(*pending_);
  pending_.reset();
  edit_untaken_ = false;
  actions_changed_ = true;
  const std::string action = edit.type == "nav" ? "finish_navigation_area" : "finish_mowing_area";
  if (!ok) {
    // Outline and obstacles stay, the user can try again or discard them.
    Refuse(action, message);
    return;
  }
  size_t points = edit.outline.size();
  for (const auto& ring : edit.obstacles) points += ring.size();
  // Not AREA: the app reads that as the start of mowing an area.
  events_.push_back({"AREA_RECORDED",
                     {{"area_id", id},
                      {"area_type", edit.type},
                      {"obstacles", edit.obstacles.size()},
                      {"points", points},
                      {"area_m2", std::round(RingArea(edit.outline) * 10) / 10}}});
  notices_.push_back("Saved the " + std::string(edit.type == "nav" ? "navigation" : "mowing") + " area " + id);
  outline_.reset();
  obstacles_.clear();
}

size_t AreaRecorder::PointCount() const {
  size_t count = current_.size() + (outline_ ? outline_->size() : 0);
  for (const auto& ring : obstacles_) count += ring.size();
  return count;
}

void AreaRecorder::AddPoint(double x, double y) {
  if (PointCount() >= config_.max_points) {
    if (!full_reported_) {
      full_reported_ = true;
      Refuse("point", "The recording has reached " + std::to_string(config_.max_points) + " points, close the ring");
    }
    return;
  }
  current_.push_back({x, y});
}

void AreaRecorder::OnPose(double x, double y, float position_accuracy) {
  pose_ = Pose{x, y, position_accuracy};
  if (!recording_) return;
  if (!(position_accuracy <= config_.max_position_accuracy)) {
    const std::string reason = "Position accuracy " + Metres(position_accuracy) + " m, points need " +
                               Metres(config_.max_position_accuracy) + " m or better";
    if (collect_requested_) {
      collect_requested_ = false;
      Refuse("collect_point", reason);
    } else if (!accuracy_blocked_ && auto_points_) {
      Refuse("point", reason);
    }
    accuracy_blocked_ = true;
    return;
  }
  if (accuracy_blocked_) {
    accuracy_blocked_ = false;
    notices_.push_back("Position accurate again, points are recorded");
  }
  const double distance = current_.empty() ? 0 : std::hypot(x - current_.back().x, y - current_.back().y);
  const bool far_enough = current_.empty() || distance > config_.min_point_distance;
  if (auto_points_) {
    if (far_enough) AddPoint(x, y);
    return;
  }
  if (!collect_requested_) return;
  collect_requested_ = false;
  if (far_enough) {
    AddPoint(x, y);
  } else {
    Refuse("collect_point", "Only " + Metres(distance) + " m from the last point, more than " +
                                Metres(config_.min_point_distance) + " m needed");
  }
}

void AreaRecorder::OnPoseLost() {
  pose_.reset();
  if (collect_requested_) {
    collect_requested_ = false;
    Refuse("collect_point", "No position");
  }
}

void AreaRecorder::SetMapWritable(bool writable, const std::string& reason) {
  map_writable_ = writable;
  map_reason_ = writable ? "" : reason;
}

std::string AreaRecorder::SubState() const {
  if (!recording_) return "";
  return outline_ ? "RECORD_OBSTACLE" : "RECORD_OUTLINE";
}

// Like OpenMower; the app tells its steps apart by these flags.
std::vector<bool> AreaRecorder::Enabled() const {
  std::vector<bool> on(kActionCount, false);
  on[kExitRecordingMode] = true;
  if (pending_) return on;
  if (recording_) {
    on[kStopRecording] = on[kFinishNavigationArea] = on[kFinishMowingArea] = on[kFinishDiscard] = true;
    on[kAutoPointsEnable] = !auto_points_;
    on[kAutoPointsDisable] = auto_points_;
    on[kCollectPoint] = !auto_points_;
  } else if (outline_) {
    on[kStartRecording] = on[kFinishNavigationArea] = on[kFinishMowingArea] = on[kFinishDiscard] = true;
  } else {
    on[kStartRecording] = true;
  }
  // start_manual_mowing and stop_manual_mowing stay off: the blade comes after phase 7.
  return on;
}

std::vector<Event> AreaRecorder::TakeEvents() {
  std::vector<Event> out;
  out.swap(events_);
  return out;
}

std::vector<std::string> AreaRecorder::TakeNotices() {
  std::vector<std::string> out;
  out.swap(notices_);
  return out;
}

bool AreaRecorder::TakeActionsChanged() {
  const bool changed = actions_changed_;
  actions_changed_ = false;
  return changed;
}

}  // namespace mowbite_logic
