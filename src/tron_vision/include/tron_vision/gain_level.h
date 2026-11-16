#ifndef TRON_VISION_GAIN_LEVEL_H
#define TRON_VISION_GAIN_LEVEL_H

#include <cstdint>
#include <string>

namespace tron_vision {

// Gain level of the control inside the stereo camera. The node only wishes for one; the setup of the platform sets
// it (tron-stereo-ae watch).
enum class GainLevel { kDay, kDusk, kNight };

const char* GainLevelName(GainLevel level);
bool GainLevelFromName(const std::string& name, GainLevel& level);

// Mean luma of the left half of a YUYV frame, from every `step`-th pixel of every `step`-th line. `yuyv` holds
// `height` lines of `line_bytes` bytes. 0 for a frame without pixels.
float MeanLumaLeft(const uint8_t* yuyv, uint32_t width, uint32_t height, uint32_t line_bytes, uint32_t step = 8);

struct GainWishLimits {
  // The control holds the mean luma near 145 while it has reserve. Below this for dark_s it is at its limit.
  float dark_below = 95.0F;
  double dark_s = 20.0;
  // Above this for bright_s the picture is burnt out: one level down without a wait.
  float bright_above = 200.0F;
  double bright_s = 5.0;
  // After a change the luma counts again after this time.
  double settle_s = 5.0;
  // On a fixed level the luma does not show the reserve, so the next lower level is tried: after this time on the
  // level, with the luma at least reserve_above.
  double try_down_s = 600.0;
  float reserve_above = 130.0F;
  // A try that went dark within try_hold_s doubles the wait, up to try_down_max_s; one that held sets it back.
  double try_hold_s = 60.0;
  double try_down_max_s = 3600.0;
  // A longer gap between two values (no stream) starts the times anew.
  double gap_s = 3.0;
};

struct GainWishState {
  GainLevel level = GainLevel::kDay;
  bool started = false;
  double last_at = 0.0;
  double changed_at = 0.0;
  // Negative: not dark, not bright.
  double dark_since = -1.0;
  double bright_since = -1.0;
  // 0: the first wait, try_down_s.
  double try_down_wait = 0.0;
  bool trying_down = false;
};

// State after the mean luma `luma` of about a second of frames at `now_s`.
GainWishState NextGainWish(const GainWishState& state, double now_s, float luma, const GainWishLimits& limits);

// The line of the wish file: level, monotonic clock and wall clock in seconds.
std::string GainWishLine(GainLevel level, double monotonic_s, double unix_s);

// Writes that line to `path` through a temporary file beside it; makes the directory of the file if it is missing.
bool WriteGainWish(const std::string& path, GainLevel level, double monotonic_s, double unix_s, std::string& error);

// The level in the first field of such a line; false for anything else.
bool GainLevelFromLine(const std::string& line, GainLevel& level);

// An exclusive flock on a file that another program makes, tried for at most `wait_s` and held until the object goes.
// Without a path or without the file it counts as held: then nobody else holds it either.
class FileLock {
 public:
  FileLock(const std::string& path, double wait_s);
  ~FileLock();
  FileLock(const FileLock&) = delete;
  FileLock& operator=(const FileLock&) = delete;

  bool held() const {
    return held_;
  }

 private:
  int fd_ = -1;
  bool held_ = false;
};

}  // namespace tron_vision

#endif  // TRON_VISION_GAIN_LEVEL_H
