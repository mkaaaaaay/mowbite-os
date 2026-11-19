#include "tron_gnss/gnss_tracker.h"

#include <algorithm>
#include <cmath>

namespace tron {

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kDayS = 86400.0;

// current_s - previous_s across midnight as well.
double Elapsed(double previous_s, double current_s) {
  double d = current_s - previous_s;
  if (d < -kDayS / 2) d += kDayS;
  return d;
}

// Later within half a day, across midnight as well. A step back counts as not advancing once, the next GGA is
// compared with it.
bool Advances(double previous_s, double current_s) {
  const double d = Elapsed(previous_s, current_s);
  return d > 0.0 && d < kDayS / 2;
}

double Way(const Enu& a, const Enu& b) {
  return std::hypot(b.east - a.east, b.north - a.north);
}

}  // namespace

void GnssTracker::SetMotion(bool valid, const std::optional<double>& speed, const std::optional<double>& course,
                            double now_s) {
  if (valid && speed && course) {
    motion_ = Motion{*speed, *course, now_s};
  } else {
    motion_.reset();
  }
}

std::optional<GnssPose> GnssTracker::Update(const NmeaMessage& message, double now_s) {
  switch (message.type) {
    case NmeaMessage::Type::kRmc:
      SetMotion(message.rmc.valid, message.rmc.speed_mps, message.rmc.course_deg, now_s);
      return std::nullopt;
    case NmeaMessage::Type::kVtg:
      SetMotion(message.vtg.valid, message.vtg.speed_mps, message.vtg.course_deg, now_s);
      return std::nullopt;
    case NmeaMessage::Type::kBoardRtk: return std::nullopt;
    case NmeaMessage::Type::kGga: break;
  }

  const GgaData& gga = message.gga;
  const bool advanced = !last_gga_utc_s_ || Advances(*last_gga_utc_s_, gga.utc_s);
  last_gga_utc_s_ = gga.utc_s;
  if (!advanced) {
    stale_gga_++;
    return std::nullopt;
  }
  // An epoch that gives no pose is not RTK fixed either: the run of fixed positions ends with it.
  if (!gga.lat_deg || !gga.lon_deg) {
    fixes_.clear();
    return std::nullopt;
  }
  GnssPose pose;
  switch (gga.quality) {
    case static_cast<int>(FixQuality::kSingle):
      pose.solution = Solution::kSingle;
      pose.accuracy_m = config_.accuracy_single_m;
      break;
    case static_cast<int>(FixQuality::kDgps):
      pose.solution = Solution::kDgps;
      pose.accuracy_m = config_.accuracy_dgps_m;
      break;
    case static_cast<int>(FixQuality::kRtkFixed): {
      std::optional<double> age = gga.correction_age_s;
      // The host counts only while NTRIP is set up: corrections it passed before stay out of a fix over LoRa.
      if (age && host_required_ && host_correction_age_s_) age = std::max(*age, *host_correction_age_s_);
      const bool before_host = host_required_ && !host_correction_age_s_;
      if (!before_host && age && *age <= config_.max_correction_age_s) {
        pose.solution = Solution::kRtkFixed;
        pose.accuracy_m = config_.accuracy_rtk_fixed_m;
        break;
      }
      // Without fresh corrections from the expected source the fix may rest on old or other data: worth no more than
      // float.
      if (before_host) {
        fixed_before_host_++;
      } else {
        old_corrections_++;
        last_old_correction_age_ = age;
      }
      [[fallthrough]];
    }
    case static_cast<int>(FixQuality::kRtkFloat):
      pose.solution = Solution::kRtkFloat;
      pose.accuracy_m = config_.accuracy_rtk_float_m;
      break;
    case static_cast<int>(FixQuality::kDeadReckoning):
      pose.solution = Solution::kDeadReckoning;
      pose.accuracy_m = config_.accuracy_dead_reckoning_m;
      break;
    default:
      fixes_.clear();
      return std::nullopt;
  }
  pose.position = frame_.ToEnu(*gga.lat_deg, *gga.lon_deg, gga.altitude_m.value_or(0.0));
  pose.sensor_stamp_ms = static_cast<uint32_t>(std::lround(gga.utc_s * 1000.0));
  pose.satellites = gga.satellites;

  if (config_.motion_from_fixes) {
    if (pose.solution == Solution::kRtkFixed) {
      fixes_.push_back(Fix{gga.utc_s, pose.position});
    } else {
      fixes_.clear();
    }
    MotionFromFixes(pose);
  } else if (motion_ && now_s - motion_->time_s <= config_.motion_max_age_s) {
    double course = motion_->course_deg * kDegToRad;
    pose.motion_valid = true;
    pose.motion_east_mps = motion_->speed_mps * std::sin(course);
    pose.motion_north_mps = motion_->speed_mps * std::cos(course);
    pose.motion_heading = std::atan2(pose.motion_north_mps, pose.motion_east_mps);
  }
  return pose;
}

void GnssTracker::MotionFromFixes(GnssPose& pose) {
  if (fixes_.empty()) return;
  const Fix& newest = fixes_.back();
  while (Elapsed(fixes_.front().utc_s, newest.utc_s) > config_.motion_max_window_s + 1e-6) fixes_.pop_front();
  // The latest position far enough away: the shortest window with the way.
  size_t start = fixes_.size();
  for (size_t i = fixes_.size() - 1; i-- > 0;) {
    if (Way(fixes_[i].position, newest.position) >= config_.motion_min_way_m) {
      start = i;
      break;
    }
  }
  if (start == fixes_.size()) return;
  const Fix& oldest = fixes_[start];
  const double dt = Elapsed(oldest.utc_s, newest.utc_s);
  if (!(dt > 0.0)) return;
  // The position nearest the middle in time splits the way in two halves of one direction and about the same length:
  // starting or stopping within the window gives none, so the vector is never one of a motion already over.
  const double middle_s = dt / 2.0;
  size_t mid = start;
  for (size_t i = start; i < fixes_.size(); i++) {
    if (std::fabs(Elapsed(oldest.utc_s, fixes_[i].utc_s) - middle_s) <
        std::fabs(Elapsed(oldest.utc_s, fixes_[mid].utc_s) - middle_s)) {
      mid = i;
    }
  }
  const Enu& a = oldest.position;
  const Enu& m = fixes_[mid].position;
  const Enu& b = newest.position;
  const double whole = Way(a, b);
  if (Way(a, m) < whole / 3.0 || Way(m, b) < whole / 3.0) return;
  const double first = std::atan2(m.north - a.north, m.east - a.east);
  const double second = std::atan2(b.north - m.north, b.east - m.east);
  const double bend = std::fabs(std::remainder(second - first, 2.0 * M_PI));
  if (bend > config_.motion_straight_deg * kDegToRad) return;
  pose.motion_valid = true;
  pose.motion_east_mps = (b.east - a.east) / dt;
  pose.motion_north_mps = (b.north - a.north) / dt;
  pose.motion_heading = std::atan2(pose.motion_north_mps, pose.motion_east_mps);
  pose.motion_window_s = dt;
}

}  // namespace tron
