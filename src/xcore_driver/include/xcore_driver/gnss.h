#ifndef XCORE_DRIVER_GNSS_H
#define XCORE_DRIVER_GNSS_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "mowbite_gnss/geodesy.h"
#include "mowbite_gnss/gga.h"
#include "mowbite_gnss/solution.h"
#include "xcore_driver/link.h"
#include "xcore_driver/wire.h"

namespace xcore {

// Fix type as the board's GPS service reports it.
enum class BoardFix { kNone, kFloat, kFixed };

// One solution of the receiver as the GPS service of the board reports it.
struct GnssSolution {
  double lat_deg = 0;
  double lon_deg = 0;
  double height_m = 0;  // above mean sea level
  std::optional<double> horizontal_accuracy_m;
  BoardFix fix = BoardFix::kNone;
  std::optional<std::array<double, 3>> motion_enu;  // m/s
  uint64_t board_time_us = 0;
};

// The board sends each solution as one transaction, so the GPS outputs of one datagram belong together. nullopt
// without a position, or with one that cannot be (not finite, out of range, or exactly 0/0).
std::optional<GnssSolution> DecodeGnss(const std::vector<Output>& outputs);

struct GnssConfig {
  // Accuracy per kind of solution where the board reports none that can be used.
  double accuracy_rtk_fixed_m = 0.02;
  double accuracy_rtk_float_m = 0.3;
  double accuracy_single_m = 3.0;
  // The board's accuracy counts as no better than this.
  double min_accuracy_m = 0.01;
  // The board does not report the age of its corrections: RTK fixed counts only while the host passed corrections to
  // it within this many s, otherwise as float.
  double max_correction_age_s = 30.0;
};

struct GnssPose {
  uint32_t sensor_stamp_ms = 0;  // board time, wraps after about 71 min
  mowbite_gnss::Solution solution = mowbite_gnss::Solution::kNone;
  double accuracy_m = 0;
  mowbite_gnss::Enu position;
  bool motion_valid = false;
  double motion_east_mps = 0;
  double motion_north_mps = 0;
  double motion_heading = 0;  // yaw, counter-clockwise from east
};

// Board solutions to poses in the local frame of the datum, with kind and accuracy as mowbite_msgs/GnssFix has them.
class GnssConverter {
 public:
  GnssConverter(const GnssConfig& config, const mowbite_gnss::LocalFrame& frame) : config_(config), frame_(frame) {
  }

  // host_correction_age_s: s since the host last passed RTCM to the board, nullopt if it never did. nullopt for a
  // solution with the board time of the one before (the same datagram again).
  std::optional<GnssPose> Convert(const GnssSolution& solution, const std::optional<double>& host_correction_age_s);

  // Fixed solutions counted as float for old or missing corrections, and the host's age at the last one.
  uint64_t fixed_without_corrections() const {
    return fixed_without_corrections_;
  }
  std::optional<double> last_downgrade_age() const {
    return last_downgrade_age_;
  }
  uint64_t repeated() const {
    return repeated_;
  }

 private:
  GnssConfig config_;
  mowbite_gnss::LocalFrame frame_;
  std::optional<uint64_t> last_board_time_us_;
  uint64_t fixed_without_corrections_ = 0;
  std::optional<double> last_downgrade_age_;
  uint64_t repeated_ = 0;
};

// The GGA a caster gets for a solution of the board, which reports neither satellites nor HDOP.
mowbite_gnss::GgaFields GgaOf(const GnssSolution& solution, double utc_s);

// RTCM on its way to the GPS service: bytes in arrival order, taken out in pieces that fit one datagram. The board
// passes them on to the receiver as a stream, so a piece may end anywhere.
class RtcmQueue {
 public:
  explicit RtcmQueue(size_t max_bytes = 32 * 1024, size_t max_piece = kMaxPayload)
      : max_bytes_(max_bytes), max_piece_(max_piece) {
  }

  // False when there is no room: the bytes are dropped, as the receiver wants fresh corrections, not all of them.
  bool Push(const uint8_t* data, size_t size);
  std::vector<Bytes> Take();
  // Drops what is queued, counted as dropped.
  void Clear();

  size_t size() const {
    return bytes_.size();
  }
  uint64_t dropped_bytes() const {
    return dropped_;
  }

 private:
  size_t max_bytes_;
  size_t max_piece_;
  std::deque<uint8_t> bytes_;
  uint64_t dropped_ = 0;
};

}  // namespace xcore

#endif  // XCORE_DRIVER_GNSS_H
