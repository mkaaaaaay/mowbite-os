#ifndef TRON_VISION_OBSTACLE_SIGHT_H
#define TRON_VISION_OBSTACLE_SIGHT_H

#include <cstdint>
#include <string>

#include "tron_vision/stereo_depth.h"

namespace tron_vision {

// Why the stereo camera does not see the ground; the names of Obstacles.msg map to these one by one.
enum class SightReason {
  kNone,
  kNotSetUp,
  kTooDark,
  kTooBright,
  kCovered,
  kNoSignal,
  kTooHot,
  kSelfTest,
  kLockedAfterCrash,
};

const char* SightReasonName(SightReason reason);

// Start values, not measured yet.
struct SightLimits {
  // No frame for this long, or this many analysed frames in a row with a bit-equal check field.
  double no_frame_s = 0.5;
  int still_frames = 5;
  // Too dark: mean luma of the lower half below dark_luma for dark_s at the highest gain, for dark_any_s at any gain
  // (the gain follows the whole picture, a bright sky keeps it low over a dark ground); back from dark_back.
  double dark_luma = 35.0;
  double dark_back = 45.0;
  double dark_s = 2.0;
  double dark_any_s = 10.0;
  // Too bright: of the pixels at bright_value or more, bright_low of the lower half or bright_all of the image, for
  // bright_s; back once both are below bright_back.
  int bright_value = 245;
  double bright_low = 0.20;
  double bright_all = 0.40;
  double bright_back = 0.10;
  double bright_s = 1.0;
  // Covered, for covered_s: the mean luma of the eyes differs by more than covered_luma of the brighter one, the edges
  // of one eye are below covered_edges of the other, or while moving less than covered_matched of the check points
  // find a depth.
  double covered_luma = 0.35;
  double covered_edges = 0.40;
  double covered_matched = 0.30;
  double covered_s = 2.0;
};

// What one analysed frame shows.
struct SightSample {
  // When it was taken (s).
  double taken = 0.0;
  // Of the left eye: mean luma of the lower half, the share of bright pixels there and in the whole eye.
  double luma_low = 0.0;
  double bright_low = 0.0;
  double bright_all = 0.0;
  // Of each eye: mean luma and mean difference of neighbouring pixels.
  double luma_left = 0.0;
  double luma_right = 0.0;
  double edges_left = 0.0;
  double edges_right = 0.0;
  // A hash of a field in the middle of the lower half of the left eye.
  uint64_t field = 0;
  // The camera runs at its highest gain; true where that is not known.
  bool at_max_gain = true;
  // Share of the check points with a depth; below 0 while it was not measured (the mower stands).
  double matched = -1.0;
};

// Fills all but taken, at_max_gain and matched from every fourth pixel of every fourth line.
SightSample MeasureSight(const StereoPair& pair, int bright_value);

// The sight of the camera from the analysed frames, each reason held for its time and let go with its margin.
class SightCheck {
 public:
  explicit SightCheck(const SightLimits& limits) : limits_(limits) {}

  void Add(const SightSample& sample);
  // Any frame of the camera, analysed or not.
  void Frame(double now) {
    last_frame_ = now;
  }
  // Forgets all, as after the camera stopped.
  void Reset();
  // The first reason that holds now, in the order of SightReason, with a few words for the log.
  SightReason Reason(double now, std::string& detail) const;

 private:
  // Whether a condition held long enough: `since` is when it began, below 0 while it does not hold.
  static bool Held(bool holds, double taken, double hold_s, double& since);

  SightLimits limits_;
  double last_frame_ = -1.0;
  bool have_field_ = false;
  uint64_t field_ = 0;
  int still_ = 0;
  bool dark_ = false;
  double dark_since_ = -1.0;
  bool bright_ = false;
  double bright_since_ = -1.0;
  bool covered_ = false;
  double covered_since_ = -1.0;
  SightSample last_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_OBSTACLE_SIGHT_H
