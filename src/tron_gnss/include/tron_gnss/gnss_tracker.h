#ifndef TRON_GNSS_GNSS_TRACKER_H
#define TRON_GNSS_GNSS_TRACKER_H

#include <cstdint>
#include <deque>
#include <optional>

#include "mowbite_gnss/geodesy.h"
#include "mowbite_gnss/solution.h"
#include "tron_gnss/nmea.h"

namespace tron {

using mowbite_gnss::Enu;
using mowbite_gnss::LocalFrame;
using mowbite_gnss::Solution;

struct GnssConfig {
  // Accuracy assumed per fix type until BESTNAVA sigmas are logged on the device.
  double accuracy_rtk_fixed_m = 0.02;
  double accuracy_rtk_float_m = 0.3;
  double accuracy_dgps_m = 1.0;
  double accuracy_single_m = 3.0;
  double accuracy_dead_reckoning_m = 5.0;
  // Where the motion vector comes from: the speed and course of the receiver (RMC, VTG), or the RTK fixed positions
  // themselves. The RTK board of the Tron (LG290P, firmware 0.9.50) holds its course at one value at mowing speed
  // (285.91 in every RMC while driving, measured 2026-11-15), so there it is the positions.
  bool motion_from_fixes = true;
  // Receiver: speed and course older than this are not used as motion vector.
  double motion_max_age_s = 1.0;
  // Positions: from the newest RTK fixed position back to the latest one at least motion_min_way_m away, within
  // motion_max_window_s, with only RTK fixed positions between; the way through the middle position has to bend by no
  // more than motion_straight_deg, so that a turn gives no course late by half the window, and each half has to be at
  // least a third of it, so that a start or a stop gives none.
  double motion_min_way_m = 0.10;
  double motion_max_window_s = 1.5;
  double motion_straight_deg = 5.0;
  // A receiver can hold RTK fixed for a while after its corrections stop. Fixed counts as float when the corrections are
  // older than this or of unknown age.
  double max_correction_age_s = 30.0;
};

struct GnssPose {
  uint32_t sensor_stamp_ms = 0;  // UTC time of day of the fix
  Solution solution = Solution::kNone;
  double accuracy_m = 0;
  Enu position;
  bool motion_valid = false;
  double motion_east_mps = 0;
  double motion_north_mps = 0;
  double motion_heading = 0;  // yaw, counter-clockwise from east
  int satellites = 0;
};

// Combines position from GGA with a motion vector, from the speed and course of RMC or VTG or from the RTK fixed
// positions (motion_from_fixes). One pose per usable GGA whose UTC time is later than the one of the GGA before, so a
// receiver that hangs and repeats its last sentence gives no pose.
class GnssTracker {
 public:
  GnssTracker(const GnssConfig& config, const LocalFrame& frame) : config_(config), frame_(frame) {
  }

  // now_s is a monotonic clock, only differences are used.
  std::optional<GnssPose> Update(const NmeaMessage& message, double now_s);

  // The host's own age of the corrections (s since it last passed RTCM to the receiver), nullopt if it never did. While
  // set, the larger of it and the receiver's age counts against max_correction_age.
  void SetHostCorrectionAge(const std::optional<double>& age_s) {
    host_correction_age_s_ = age_s;
  }
  // While set, RTK fixed counts as float until the host has passed on corrections: with NTRIP set up a fix before
  // that may rest on a LoRa base, which has a reference of its own (G24).
  void SetHostCorrectionsRequired(bool required) {
    host_required_ = required;
  }

  // GGA sentences dropped because their time did not advance.
  uint64_t stale_gga() const {
    return stale_gga_;
  }

  // Fixed solutions counted as float for old or missing corrections, and the age of the last one (empty if missing).
  uint64_t old_corrections() const {
    return old_corrections_;
  }
  std::optional<double> last_old_correction_age() const {
    return last_old_correction_age_;
  }
  // Fixed solutions counted as float because the host had not passed on corrections yet.
  uint64_t fixed_before_host() const {
    return fixed_before_host_;
  }

 private:
  struct Motion {
    double speed_mps;
    double course_deg;
    double time_s;
  };
  struct Fix {
    double utc_s;
    Enu position;
  };
  void SetMotion(bool valid, const std::optional<double>& speed, const std::optional<double>& course, double now_s);
  // The motion of the recent RTK fixed positions into pose, or nothing.
  void MotionFromFixes(GnssPose& pose);

  GnssConfig config_;
  LocalFrame frame_;
  std::optional<Motion> motion_;
  // RTK fixed positions within motion_max_window_s, oldest first.
  std::deque<Fix> fixes_;
  std::optional<double> last_gga_utc_s_;
  uint64_t stale_gga_ = 0;
  uint64_t old_corrections_ = 0;
  std::optional<double> last_old_correction_age_;
  std::optional<double> host_correction_age_s_;
  bool host_required_ = false;
  uint64_t fixed_before_host_ = 0;
};

}  // namespace tron

#endif  // TRON_GNSS_GNSS_TRACKER_H
