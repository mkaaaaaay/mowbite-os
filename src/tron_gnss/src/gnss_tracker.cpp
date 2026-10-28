#include "tron_gnss/gnss_tracker.h"

#include <cmath>

namespace tron {

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kDayS = 86400.0;

// Later within half a day, across midnight as well. A step back counts as not advancing once, the next GGA is
// compared with it.
bool Advances(double previous_s, double current_s) {
  double d = current_s - previous_s;
  if (d < -kDayS / 2) d += kDayS;
  return d > 0.0 && d < kDayS / 2;
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
    case NmeaMessage::Type::kGga: break;
  }

  const GgaData& gga = message.gga;
  const bool advanced = !last_gga_utc_s_ || Advances(*last_gga_utc_s_, gga.utc_s);
  last_gga_utc_s_ = gga.utc_s;
  if (!advanced) {
    stale_gga_++;
    return std::nullopt;
  }
  if (!gga.lat_deg || !gga.lon_deg) return std::nullopt;
  GnssPose pose;
  switch (gga.quality) {
    case static_cast<int>(FixQuality::kSingle): pose.accuracy_m = config_.accuracy_single_m; break;
    case static_cast<int>(FixQuality::kDgps): pose.accuracy_m = config_.accuracy_dgps_m; break;
    case static_cast<int>(FixQuality::kRtkFixed):
      pose.flags = kFlagRtk | kFlagRtkFixed;
      pose.accuracy_m = config_.accuracy_rtk_fixed_m;
      break;
    case static_cast<int>(FixQuality::kRtkFloat):
      pose.flags = kFlagRtk | kFlagRtkFloat;
      pose.accuracy_m = config_.accuracy_rtk_float_m;
      break;
    case static_cast<int>(FixQuality::kDeadReckoning):
      pose.flags = kFlagDeadReckoning;
      pose.accuracy_m = config_.accuracy_dead_reckoning_m;
      break;
    default: return std::nullopt;
  }
  pose.position = frame_.ToEnu(*gga.lat_deg, *gga.lon_deg, gga.altitude_m.value_or(0.0));
  pose.sensor_stamp_ms = static_cast<uint32_t>(std::lround(gga.utc_s * 1000.0));
  pose.satellites = gga.satellites;

  if (motion_ && now_s - motion_->time_s <= config_.motion_max_age_s) {
    double course = motion_->course_deg * kDegToRad;
    pose.motion_valid = true;
    pose.motion_east_mps = motion_->speed_mps * std::sin(course);
    pose.motion_north_mps = motion_->speed_mps * std::cos(course);
    pose.motion_heading = std::atan2(pose.motion_north_mps, pose.motion_east_mps);
  }
  return pose;
}

}  // namespace tron
