#ifndef TRON_GNSS_GNSS_TRACKER_H
#define TRON_GNSS_GNSS_TRACKER_H

#include <cstdint>
#include <optional>

#include "tron_gnss/geodesy.h"
#include "tron_gnss/nmea.h"

namespace tron {

// Flags and source as in mowbite_msgs/AbsolutePose.
constexpr uint16_t kFlagRtk = 1;
constexpr uint16_t kFlagRtkFixed = 2;
constexpr uint16_t kFlagRtkFloat = 4;
constexpr uint16_t kFlagDeadReckoning = 8;

struct GnssConfig {
  // Accuracy assumed per fix type until BESTNAVA sigmas are logged on the device.
  double accuracy_rtk_fixed_m = 0.02;
  double accuracy_rtk_float_m = 0.3;
  double accuracy_dgps_m = 1.0;
  double accuracy_single_m = 3.0;
  double accuracy_dead_reckoning_m = 5.0;
  // Speed and course older than this are not used as motion vector.
  double motion_max_age_s = 1.0;
  // A receiver can hold RTK fixed for a while after its corrections stop. Fixed counts as float when the corrections are
  // older than this or of unknown age.
  double max_correction_age_s = 30.0;
};

struct GnssPose {
  uint32_t sensor_stamp_ms = 0;  // UTC time of day of the fix
  uint16_t flags = 0;
  double accuracy_m = 0;
  Enu position;
  bool motion_valid = false;
  double motion_east_mps = 0;
  double motion_north_mps = 0;
  double motion_heading = 0;  // yaw, counter-clockwise from east
  int satellites = 0;
};

// Combines position from GGA with speed and course from RMC or VTG. One pose per usable GGA whose UTC time is later
// than the one of the GGA before, so a receiver that hangs and repeats its last sentence gives no pose.
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

 private:
  struct Motion {
    double speed_mps;
    double course_deg;
    double time_s;
  };
  void SetMotion(bool valid, const std::optional<double>& speed, const std::optional<double>& course, double now_s);

  GnssConfig config_;
  LocalFrame frame_;
  std::optional<Motion> motion_;
  std::optional<double> last_gga_utc_s_;
  uint64_t stale_gga_ = 0;
  uint64_t old_corrections_ = 0;
  std::optional<double> last_old_correction_age_;
  std::optional<double> host_correction_age_s_;
};

}  // namespace tron

#endif  // TRON_GNSS_GNSS_TRACKER_H
