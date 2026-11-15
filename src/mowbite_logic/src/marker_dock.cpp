#include "mowbite_logic/marker_dock.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mowbite_logic {

namespace {

// The states of DockMarker.msg.
constexpr uint8_t kOff = 0;
constexpr uint8_t kMeasuring = 2;
constexpr uint8_t kFault = 3;
// m along the line within which base_link stands at the approach point, and the heading off the line there within
// which the line starts without turning on the spot (as DockingSequence::FirstLeg).
constexpr double kAtApproach = 0.1;
constexpr double kAtApproachHeading = 20.0 * M_PI / 180.0;
// s between two poses beyond which the way of the wheels is not counted.
constexpr double kMaxPoseGap = 0.5;

double Normalize(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

// The marker in the station frame, whose origin is base_link of the docked mower.
Point2 MarkerAt(const DockingSequence& docking, const MarkerConfig& marker) {
  const double c = std::cos(docking.facing());
  const double s = std::sin(docking.facing());
  return {c * marker.marker_x - s * marker.marker_y, s * marker.marker_x + c * marker.marker_y};
}

}  // namespace

DockingSequence MarkerDocking(DockConfig config, const UndockConfig& out, bool contacts_back,
                              const MarkerConfig& marker) {
  config.approach_distance = std::min(config.approach_distance, marker.range);
  return DockingSequence(config, out, DockPose{0.0, 0.0, 0.0}, contacts_back,
                         StationLegs{marker.cross_track, marker.blind_distance});
}

DockPose StationByMarker(double x, double y, double heading, const DockPoseSample& pose) {
  const double station = Normalize(heading - pose.heading);
  const double c = std::cos(station);
  const double s = std::sin(station);
  return DockPose{x - (c * pose.x - s * pose.y), y - (s * pose.x + c * pose.y), station};
}

std::string MarkerMapProblem(const DockPose& by_marker, const DockPose& map, double limit, double heading_limit) {
  const double off = std::hypot(by_marker.x - map.x, by_marker.y - map.y);
  const double turned = std::abs(Normalize(by_marker.heading - map.heading));
  char text[200];
  if (off > limit) {
    std::snprintf(text, sizeof(text), "by the marker the docking station stands %.2f m from the recorded one, at most "
                  "%.2f m allowed", off, limit);
    return text;
  }
  if (heading_limit > 0 && turned > heading_limit) {
    std::snprintf(text, sizeof(text), "by the marker the docking station faces %.0f deg off the recorded one, at most "
                  "%.0f deg allowed", turned * 180.0 / M_PI, heading_limit * 180.0 / M_PI);
    return text;
  }
  return "";
}

void MarkerWatch::OnMarker(const DockMarkerSample& sample) {
  last_ = sample;
  heard_ = sample.time;
  if (sample.state != kMeasuring) return;
  measuring_ = true;
  if (sample.valid) {
    valid_ = true;
    yaw_ = yaw_ || sample.yaw_valid;
  } else if (!sample.reason.empty()) {
    image_reason_ = sample.reason;
  }
}

void MarkerWatch::OnPose(const DockPoseSample& pose, bool on_line) {
  if (on_line && !on_line_) {
    line_x_ = pose.x;
    wheel_way_ = 0;
  } else if (on_line && pose_) {
    const double gap = pose.time - pose_->time;
    if (gap > 0 && gap <= kMaxPoseGap) wheel_way_ += std::abs(pose.speed) * gap;
  }
  on_line_ = on_line;
  pose_ = pose;
}

void MarkerWatch::BeginWait(double now, bool sight) {
  since_ = now;
  heard_ = std::max(heard_, now);
  sight_ = sight;
  measuring_ = valid_ = yaw_ = false;
  image_reason_.clear();
}

MarkerWatch::Verdict MarkerWatch::Check(double now) const {
  const bool cannot = last_ && last_->time >= since_ && SourceFault();
  if (cannot || now - heard_ >= kMarkerSilentS) return Verdict::kUnavailable;
  if (now - since_ < config_.wait) return Verdict::kWaiting;
  if (!sight_) return Verdict::kLost;
  if (!measuring_) return Verdict::kUnavailable;
  return valid_ && !yaw_ ? Verdict::kNoDirection : Verdict::kNotSeen;
}

std::string MarkerWatch::Detail() const {
  if (last_ && last_->state != kMeasuring && !last_->reason.empty()) return last_->reason;
  return image_reason_;
}

std::string MarkerWatch::ScaleProblem() const {
  if (wheel_way_ < kScaleMinWay || !pose_) return "";
  const double marker_way = std::abs(pose_->x - line_x_);
  if (std::abs(marker_way - wheel_way_) <= config_.scale_limit * wheel_way_) return "";
  char text[200];
  std::snprintf(text, sizeof(text),
                "on the line the marker measured %.2f m, the wheels %.2f m, more than %.0f %% apart: is the marker of "
                "the right size?",
                marker_way, wheel_way_, config_.scale_limit * 100.0);
  return text;
}

bool MarkerWatch::SourceFault() const {
  return last_ && (last_->state == kFault || (last_->state == kOff && !last_->reason.empty()));
}

bool MarkerWatch::PoseGood(double now, double limit) const {
  return !SourceFault() && pose_ && pose_->heading_valid && pose_->position_accuracy <= limit && now - pose_->time <= kDockPoseMaxAgeS;
}

void MarkerWatch::Expire(double now) {
  if (pose_ && now - pose_->time > kDockPoseMaxAgeS) pose_.reset();
}

void MarkerWatch::Reset() {
  *this = MarkerWatch(config_);
}

double SideOffset(const DockingSequence& docking, const MarkerConfig& marker, double x, double y, double heading) {
  const Point2 at = MarkerAt(docking, marker);
  return -std::sin(heading) * (at.x - x) + std::cos(heading) * (at.y - y) - marker.marker_y;
}

std::string TestStartProblem(const DockingSequence& docking, const MarkerConfig& marker, const DockPoseSample& pose,
                             double lateral) {
  const double along = docking.Along(pose.x, pose.y);
  const double across = docking.Across(pose.x, pose.y);
  const double off = Normalize(pose.heading - docking.facing());
  const double side = SideOffset(docking, marker, pose.x, pose.y, pose.heading);
  char text[240];
  if (std::abs(off) >= kAtApproachHeading) {
    std::snprintf(text, sizeof(text),
                  "the mower faces %.0f deg off the line of the docking station, less than %.0f deg needed: turn it "
                  "towards the line, the contacts towards the station",
                  std::abs(off) * 180.0 / M_PI, kAtApproachHeading * 180.0 / M_PI);
    return text;
  }
  // On the line it drives straight out first: the marker has to lie close to its axis. At the approach point the line
  // begins where it stands.
  const bool at_approach = std::abs(along) <= kAtApproach;
  if (!at_approach && std::abs(side) > lateral) {
    std::snprintf(text, sizeof(text),
                  "the dock marker lies %.2f m beside the axis of the mower, at most %.2f m allowed: place it straight "
                  "in front of the station",
                  std::abs(side), lateral);
    return text;
  }
  if (!at_approach && !docking.OnLine(pose.x, pose.y, pose.heading)) {
    std::snprintf(text, sizeof(text),
                  "the mower stands %.2f m from the approach point, neither on the line nor at its start: place it "
                  "on the line in front of the docking station",
                  std::abs(along));
    return text;
  }
  // Where the line begins: there, or after the straight way out along the heading, against the way in, as far as the
  // approach point lies along the line. An error of the heading moves it beside the line by the way to the marker.
  const double begin = at_approach ? across : across - along * std::sin(off);
  const Point2 start = docking.ApproachPoint();
  const Point2 at = MarkerAt(docking, marker);
  const double to_marker = std::hypot(at.x - start.x, at.y - start.y);
  // A spread that is unknown (999, or a journal from before it) widens nothing.
  const double spread = pose.heading_accuracy >= 0 && pose.heading_accuracy <= M_PI ? pose.heading_accuracy : 0.0;
  const double allowed = std::min(lateral + to_marker * spread, marker.cross_track);
  if (std::abs(begin) > allowed) {
    std::snprintf(text, sizeof(text),
                  "the line of the docking station would begin %.2f m beside it, at most %.2f m allowed: place the "
                  "mower straighter in front of the station",
                  std::abs(begin), allowed);
    return text;
  }
  return "";
}

}  // namespace mowbite_logic
