#ifndef XCORE_DRIVER_ODOMETRY_H
#define XCORE_DRIVER_ODOMETRY_H

#include <cstdint>
#include <optional>

namespace xcore {

struct OdometryConfig {
  double ticks_per_meter = 2025.0;
  // Between the wheel contact points, in m.
  double wheel_distance = 0.3556;
  // +1 if the counter grows while the wheel turns forward, -1 if it falls. The right motor is mounted mirrored, its
  // counter falls going forward (from the firmware; to be checked on the device).
  int left_sign = 1;
  int right_sign = -1;
  // A pair of samples further apart than this, or not after each other, gives no speed; the next pair counts again.
  double max_gap_s = 0.5;
};

struct WheelTwist {
  double linear = 0;
  double angular = 0;
  double left = 0;
  double right = 0;
  double dt = 0;
};

// Wheel speeds from the free running counters of the board. The board's time stamp (µs since its start, 32 bit)
// gives the interval, so that delays on the network do not show up as speed.
class TickOdometry {
 public:
  explicit TickOdometry(const OdometryConfig& config) : config_(config) {
  }

  // nullopt for the first sample, after a gap or when the board restarted.
  std::optional<WheelTwist> OnTicks(uint32_t left, uint32_t right, uint64_t board_time_us);
  void Reset() {
    last_.reset();
  }

 private:
  struct Sample {
    uint32_t left;
    uint32_t right;
    uint32_t time_us;
  };

  OdometryConfig config_;
  std::optional<Sample> last_;
};

}  // namespace xcore

#endif  // XCORE_DRIVER_ODOMETRY_H
