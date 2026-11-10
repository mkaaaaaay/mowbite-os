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
  kUndoPoints,
  kActionCount
};

std::string Metres(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.2f", value);
  return text;
}

Reason NoPosition() {
  return Reason("NO_POSITION", "No position");
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

// The same point within a micrometre: a crossing at a corner of the ring.
bool Same(const Point2& a, const Point2& b) {
  return Distance(a, b) < 1e-6;
}

// Where the segments from a to b and from c to d meet, given they touch; on a common line an end inside the other.
Point2 Meeting(const Point2& a, const Point2& b, const Point2& c, const Point2& d) {
  const Point2 along{b.x - a.x, b.y - a.y};
  const double denominator = along.x * (d.y - c.y) - along.y * (d.x - c.x);
  if (denominator != 0) {
    const double share = Cross(c, d, a) / denominator;
    return {a.x + share * along.x, a.y + share * along.y};
  }
  if (Within(a, b, c)) return c;
  if (Within(a, b, d)) return d;
  return a;
}

// Path along the ring from point first to point last.
double PathLength(const Ring& ring, size_t first, size_t last) {
  double length = 0;
  for (size_t k = first; k < last; k++) length += Distance(ring[k], ring[k + 1]);
  return length;
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

std::optional<CutRing> CutOverhang(const Ring& ring, double max_cut) {
  const size_t n = ring.size();
  if (n < 4) return std::nullopt;
  // Only segments that begin within max_cut of the start (i) or end within max_cut of the end (j) can be cut at.
  size_t last_i = 0;
  double path = 0;
  while (last_i + 2 < n && (path += Distance(ring[last_i], ring[last_i + 1])) <= max_cut) last_i++;
  size_t first_j = n - 2;
  path = 0;
  while (first_j > 0 && (path += Distance(ring[first_j], ring[first_j + 1])) <= max_cut) first_j--;
  // Segment j runs from point j to j + 1; the first that meets an earlier one ends the loop, the latest of those
  // begins it.
  for (size_t j = std::max<size_t>(first_j, 2); j + 1 < n; j++) {
    for (size_t i = std::min(j - 1, last_i + 1); i-- > 0;) {
      if (!SegmentsTouch(ring[i], ring[i + 1], ring[j], ring[j + 1])) continue;
      const Point2 at = Meeting(ring[i], ring[i + 1], ring[j], ring[j + 1]);
      const double cut = PathLength(ring, 0, i) + Distance(ring[i], at) + Distance(at, ring[j + 1]) +
                         PathLength(ring, j + 1, n - 1);
      if (cut > max_cut) continue;
      CutRing out;
      if (!Same(at, ring[i + 1]) && !Same(at, ring[j])) out.ring.push_back(at);
      out.ring.insert(out.ring.end(), ring.begin() + static_cast<std::ptrdiff_t>(i + 1),
                      ring.begin() + static_cast<std::ptrdiff_t>(j + 1));
      // A crossing on point i or j + 1 keeps that point as where the loop begins.
      const bool kept_end = Same(at, ring[i]) || Same(at, ring[j + 1]);
      out.dropped = n - (j - i) - (kept_end ? 1 : 0);
      return out;
    }
  }
  return std::nullopt;
}

Reason RingProblem(const Ring& ring, double min_area_m2) {
  if (ring.size() < 3) {
    return Reason("RING_FEW_POINTS", "Cannot close the ring: it needs at least 3 points, it has " +
                                         std::to_string(ring.size()))
        .With("points", ring.size())
        .With("limit", 3);
  }
  if (SelfIntersects(ring)) return Reason("RING_CROSSES_ITSELF", "Cannot close the ring: the line crosses itself");
  const double area = RingArea(ring);
  if (area < min_area_m2) {
    return Reason("RING_TOO_SMALL", "Cannot close the ring: it encloses only " + Metres(area) + " m², at least " +
                                        Metres(min_area_m2) + " needed")
        .With("area", area)
        .With("limit", min_area_m2);
  }
  return {};
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
                                                 "stop_manual_mowing",
                                                 "undo_points"};
  return names;
}

void AreaRecorder::Reset() {
  recording_ = false;
  current_.clear();
  outline_.reset();
  obstacles_.clear();
  collect_requested_ = false;
  dock_first_.reset();
  accuracy_blocked_ = false;
  full_reported_ = false;
  actions_changed_ = true;
  overlay_changed_ = true;
}

AreaRecorder::Result AreaRecorder::Refuse(const std::string& action, const Reason& reason) {
  json details = {{"action", action}};
  PutReason(&details, "RECORDING_REFUSED", reason);
  events_.push_back({"RECORDING_REFUSED", std::move(details), ""});
  refusal_ = reason;
  return Result::kRefused;
}

AreaRecorder::Result AreaRecorder::OnAction(const std::string& name, Reason* why, double undo_meters) {
  refusal_ = Reason();
  trimmed_points_ = 0;
  const Result result = Act(name, undo_meters);
  if (why) *why = result == Result::kRefused ? refusal_ : Reason();
  return result;
}

AreaRecorder::Result AreaRecorder::Act(const std::string& name, double undo_meters) {
  const auto& names = ActionNames();
  const auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end() || name == "exit_recording_mode") return Result::kUnknown;
  const size_t index = static_cast<size_t>(it - names.begin());
  // The edit on its way decides what comes next; the app shows no buttons until then.
  if (pending_) return Refuse(name, Reason("STILL_SAVING", "The last change of the map is still being saved"));
  if (!Enabled()[index]) return Result::kRefused;
  actions_changed_ = true;
  switch (index) {
    case kStartRecording:
      recording_ = true;
      current_.clear();
      collect_requested_ = false;
      // Every ring that meets the limit says so once.
      full_reported_ = false;
      notices_.push_back(outline_ ? "Recording an obstacle" : "Recording an outline");
      return Result::kDone;
    case kStopRecording: return CloseRing(name) ? Result::kDone : Result::kRefused;
    case kFinishNavigationArea: return Finish(name, "nav");
    case kFinishMowingArea: return Finish(name, "mow");
    case kFinishDiscard:
      notices_.push_back(dock_first_ ? "Discarded the first point of the docking station"
                                     : "Discarded the area being recorded");
      Reset();
      return Result::kDone;
    case kRecordDock: return RecordDock();
    case kAutoPointsEnable:
    case kAutoPointsDisable:
      auto_points_ = index == kAutoPointsEnable;
      collect_requested_ = false;
      return Result::kDone;
    case kCollectPoint:
      if (!pose_) return Refuse(name, NoPosition());
      // The next pose, so the point is where the mower is now.
      collect_requested_ = true;
      return Result::kDone;
    case kUndoPoints:
      Undo(undo_meters);
      return Result::kDone;
    default: return Result::kRefused;
  }
}

void AreaRecorder::Undo(double meters) {
  const size_t before = current_.size();
  double taken = 0;
  while (!current_.empty() && taken < meters) {
    const Point2 last = current_.back();
    current_.pop_back();
    if (!current_.empty()) taken += Distance(current_.back(), last);
  }
  // Else the next pose puts a point where the mower stands; the user drives to the new end first.
  auto_points_ = false;
  collect_requested_ = false;
  full_reported_ = false;
  overlay_changed_ = true;
  notices_.push_back("Took back " + std::to_string(before - current_.size()) + " points (" + Metres(taken) +
                     " m), automatic points are off");
}

bool AreaRecorder::CloseRing(const std::string& action) {
  if (current_.empty()) {
    notices_.push_back("Ended the ring without points");
  } else {
    Reason problem = RingProblem(current_, config_.min_area);
    std::optional<CutRing> cut;
    if (problem.code == "RING_CROSSES_ITSELF") cut = CutOverhang(current_, config_.max_overhang);
    if (cut && RingProblem(cut->ring, config_.min_area).empty()) {
      notices_.push_back("Cut off " + std::to_string(cut->dropped) + " points where the ring ran past its start");
      trimmed_points_ = cut->dropped;
      current_ = std::move(cut->ring);
    } else if (!problem.empty()) {
      // The ring stays open: the mower can drive on, or take points back.
      Refuse(action, problem);
      return false;
    }
    const size_t points = current_.size();
    if (outline_) {
      obstacles_.push_back(std::move(current_));
    } else {
      outline_ = std::move(current_);
    }
    notices_.push_back("Closed the " + std::string(obstacles_.empty() ? "outline" : "obstacle") + " with " +
                       std::to_string(points) + " points");
  }
  recording_ = false;
  current_.clear();
  collect_requested_ = false;
  overlay_changed_ = true;
  return true;
}

AreaRecorder::Result AreaRecorder::Finish(const std::string& action, const std::string& type) {
  if (!map_writable_) return Refuse(action, map_reason_);
  if (recording_ && !CloseRing(action)) return Result::kRefused;
  if (!outline_) return Refuse(action, Reason("NO_OUTLINE", "No outline recorded"));
  MapEdit edit;
  edit.type = type;
  edit.outline = *outline_;
  edit.obstacles = obstacles_;
  pending_ = std::move(edit);
  edit_untaken_ = true;
  return Result::kDone;
}

// Like OpenMower: a point in front of the station, then one on the contacts; the way between them is the heading.
// Both are base_link, which is only where the mower is with a valid heading; the second one only with the contact.
AreaRecorder::Result AreaRecorder::RecordDock() {
  const std::string action = "record_dock";
  if (!map_writable_) return Refuse(action, map_reason_);
  if (!pose_) return Refuse(action, NoPosition());
  if (!(pose_->accuracy <= config_.max_position_accuracy)) {
    return Refuse(action, Reason("POSITION_ACCURACY", "Position accuracy " + Metres(pose_->accuracy) +
                                                          " m, the docking station needs " +
                                                          Metres(config_.max_position_accuracy) + " m or better")
                              .With("accuracy", pose_->accuracy)
                              .With("limit", config_.max_position_accuracy));
  }
  if (!pose_->heading_valid) {
    return Refuse(action, Reason("NO_HEADING",
                                 "The heading is not aligned yet, drive straight for a few metres first: without it "
                                 "the point may be off by the antenna offset"));
  }
  const std::string onto = config_.dock_backwards ? "backwards onto the contacts" : "onto the contacts";
  const Point2 here{pose_->x, pose_->y};
  if (!dock_first_) {
    dock_first_ = here;
    notices_.push_back("Recorded the point in front of the docking station, now drive " + onto);
    return Result::kDone;
  }
  if (!dock_contact_) {
    // The first point stays, driving on is enough.
    return Refuse(action, Reason("NO_DOCK_CONTACT", "No contact with the docking station: drive " + onto +
                                                        " until the mower reports it"));
  }
  const double distance = Distance(here, *dock_first_);
  if (distance < config_.min_dock_distance) {
    // The first point stays, driving on is enough.
    return Refuse(action, Reason("DOCK_TOO_CLOSE", "Only " + Metres(distance) + " m from the first point, at least " +
                                                       Metres(config_.min_dock_distance) + " m needed for the heading")
                              .With("distance", distance)
                              .With("limit", config_.min_dock_distance));
  }
  MapEdit edit;
  edit.kind = MapEdit::Kind::kDock;
  edit.x = here.x;
  edit.y = here.y;
  edit.heading = std::atan2(here.y - dock_first_->y, here.x - dock_first_->x);
  pending_ = edit;
  edit_untaken_ = true;
  return Result::kDone;
}

std::optional<MapEdit> AreaRecorder::TakeMapEdit() {
  if (!edit_untaken_) return std::nullopt;
  edit_untaken_ = false;
  return pending_;
}

void AreaRecorder::OnMapEditResult(bool ok, const std::string& id, const std::string& code,
                                   const std::string& message) {
  const Reason failed = Reason("SAVE_FAILED", message).With("code", code);
  if (!pending_) return;
  const MapEdit edit = std::move(*pending_);
  pending_.reset();
  edit_untaken_ = false;
  actions_changed_ = true;
  if (edit.kind == MapEdit::Kind::kDock) {
    if (!ok) {
      // The first point stays, the second can be taken again.
      Refuse("record_dock", failed);
      return;
    }
    events_.push_back({"DOCK_RECORDED", {{"dock_id", id}, {"heading", edit.heading}}, ""});
    notices_.push_back("Saved the docking station " + id);
    dock_first_.reset();
    return;
  }
  const std::string action = edit.type == "nav" ? "finish_navigation_area" : "finish_mowing_area";
  if (!ok) {
    // Outline and obstacles stay, the user can try again or discard them.
    Refuse(action, failed);
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
                      {"area_m2", std::round(RingArea(edit.outline) * 10) / 10}},
                     ""});
  notices_.push_back("Saved the " + std::string(edit.type == "nav" ? "navigation" : "mowing") + " area " + id);
  outline_.reset();
  obstacles_.clear();
  overlay_changed_ = true;
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
      Refuse("point", Reason("TOO_MANY_POINTS", "The recording has reached " + std::to_string(config_.max_points) +
                                                    " points, close the ring")
                          .With("limit", config_.max_points));
    }
    return;
  }
  // undo_points comes with the first point.
  if (current_.empty()) actions_changed_ = true;
  current_.push_back({x, y});
  overlay_changed_ = true;
}

void AreaRecorder::OnPose(double x, double y, float position_accuracy, bool heading_valid) {
  pose_ = Pose{x, y, position_accuracy, heading_valid};
  if (!recording_) return;
  if (!(position_accuracy <= config_.max_position_accuracy)) {
    const Reason reason = Reason("POSITION_ACCURACY", "Position accuracy " + Metres(position_accuracy) +
                                                          " m, points need " + Metres(config_.max_position_accuracy) +
                                                          " m or better")
                              .With("accuracy", position_accuracy)
                              .With("limit", config_.max_position_accuracy);
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
  const double distance = current_.empty() ? 0 : Distance(current_.back(), {x, y});
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
    Refuse("collect_point", Reason("POINT_TOO_CLOSE", "Only " + Metres(distance) + " m from the last point, more than " +
                                                          Metres(config_.min_point_distance) + " m needed")
                                .With("distance", distance)
                                .With("limit", config_.min_point_distance));
  }
}

void AreaRecorder::OnPoseLost() {
  pose_.reset();
  if (collect_requested_) {
    collect_requested_ = false;
    Refuse("collect_point", NoPosition());
  }
}

void AreaRecorder::SetMapWritable(bool writable, const Reason& reason) {
  map_writable_ = writable;
  map_reason_ = writable ? Reason() : reason;
}

std::string AreaRecorder::SubState() const {
  if (dock_first_) return "RECORD_DOCKING_POSITION";
  if (!recording_) return "";
  return outline_ ? "RECORD_OBSTACLE" : "RECORD_OUTLINE";
}

// Like OpenMower; the app tells its steps apart by these flags.
std::vector<bool> AreaRecorder::Enabled() const {
  std::vector<bool> on(kActionCount, false);
  on[kExitRecordingMode] = true;
  if (pending_) return on;
  if (dock_first_) {
    // The app shows only record_dock here; discard drops the first point.
    on[kRecordDock] = on[kFinishDiscard] = true;
  } else if (recording_) {
    on[kStopRecording] = on[kFinishNavigationArea] = on[kFinishMowingArea] = on[kFinishDiscard] = true;
    on[kAutoPointsEnable] = !auto_points_;
    on[kAutoPointsDisable] = auto_points_;
    on[kCollectPoint] = !auto_points_;
    on[kUndoPoints] = !current_.empty();
  } else if (outline_) {
    on[kStartRecording] = on[kFinishNavigationArea] = on[kFinishMowingArea] = on[kFinishDiscard] = true;
  } else {
    on[kStartRecording] = on[kRecordDock] = true;
  }
  // start_manual_mowing and stop_manual_mowing stay off: the blade comes after phase 7.
  return on;
}

json AreaRecorder::Overlay() const {
  json polygons = json::array();
  const auto add = [&polygons](const Ring& ring, bool closed, double line_width, const char* color) {
    // Like OpenMower: a single point is no line.
    if (ring.size() < 2) return;
    json points = json::array();
    for (const auto& p : ring) points.push_back({{"x", Millimetres(p.x)}, {"y", Millimetres(p.y)}});
    polygons.push_back(
        {{"poly", std::move(points)}, {"is_closed", closed ? 1 : 0}, {"line_width", line_width}, {"color", color}});
  };
  if (outline_) add(*outline_, true, 0.05, "green");
  for (const auto& ring : obstacles_) add(ring, true, 0.05, "red");
  if (recording_) add(current_, false, 0.1, "blue");
  return {{"polygons", std::move(polygons)}};
}

bool AreaRecorder::TakeOverlayChanged() {
  const bool changed = overlay_changed_;
  overlay_changed_ = false;
  return changed;
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
