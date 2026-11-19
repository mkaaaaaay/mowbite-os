#include "tron_vision/obstacle_sight.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace tron_vision {

namespace {

constexpr int kStep = 4;
// The check field: this many pixels wide and lines high, every pixel.
constexpr int kFieldWidth = 128;
constexpr int kFieldHeight = 32;

// Extra arguments the format does not name are left alone.
std::string Text(const char* format, double a, double b = 0.0) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), format, a, b);
  return buf;
}

double Ratio(double small, double large) {
  return large > 0.0 ? small / large : 1.0;
}

}  // namespace

const char* SightReasonName(SightReason reason) {
  switch (reason) {
    case SightReason::kNone: return "";
    case SightReason::kNotSetUp: return "NOT_SET_UP";
    case SightReason::kTooDark: return "TOO_DARK";
    case SightReason::kTooBright: return "TOO_BRIGHT";
    case SightReason::kCovered: return "COVERED";
    case SightReason::kNoSignal: return "NO_SIGNAL";
    case SightReason::kTooHot: return "TOO_HOT";
    case SightReason::kSelfTest: return "SELF_TEST";
    case SightReason::kLockedAfterCrash: return "LOCKED_AFTER_CRASH";
  }
  return "NO_SIGNAL";
}

SightSample MeasureSight(const StereoPair& pair, int bright_value) {
  SightSample s;
  const EyeLuma& left = pair.left;
  const EyeLuma& right = pair.right;
  const int width = static_cast<int>(left.width);
  const int height = static_cast<int>(left.height);
  if (width < 2 * kStep || height < 2 * kStep) return s;
  double sum_l = 0.0;
  double sum_r = 0.0;
  double sum_low = 0.0;
  double edges_l = 0.0;
  double edges_r = 0.0;
  int all = 0;
  int low = 0;
  int bright_all = 0;
  int bright_low = 0;
  for (int y = 0; y + kStep < height; y += kStep) {
    for (int x = 0; x + kStep < width; x += kStep) {
      const int l = left.At(x, y);
      const int r = right.At(x, y);
      sum_l += l;
      sum_r += r;
      edges_l += std::abs(left.At(x + kStep, y) - l) + std::abs(left.At(x, y + kStep) - l);
      edges_r += std::abs(right.At(x + kStep, y) - r) + std::abs(right.At(x, y + kStep) - r);
      all++;
      const bool bright = l >= bright_value;
      bright_all += bright;
      if (2 * y >= height) {
        sum_low += l;
        low++;
        bright_low += bright;
      }
    }
  }
  s.luma_left = sum_l / all;
  s.luma_right = sum_r / all;
  s.edges_left = edges_l / all;
  s.edges_right = edges_r / all;
  s.bright_all = static_cast<double>(bright_all) / all;
  s.luma_low = low > 0 ? sum_low / low : 0.0;
  s.bright_low = low > 0 ? static_cast<double>(bright_low) / low : 0.0;
  // FNV-1a over the check field.
  uint64_t hash = 1469598103934665603ULL;
  const int x0 = std::max(0, (width - kFieldWidth) / 2);
  const int y0 = std::max(0, (3 * height) / 4 - kFieldHeight / 2);
  for (int y = y0; y < std::min(height, y0 + kFieldHeight); y++) {
    for (int x = x0; x < std::min(width, x0 + kFieldWidth); x++) {
      hash = (hash ^ left.At(x, y)) * 1099511628211ULL;
    }
  }
  s.field = hash;
  return s;
}

bool SightCheck::Held(bool holds, double taken, double hold_s, double& since) {
  if (!holds) {
    since = -1.0;
    return false;
  }
  if (since < 0.0) since = taken;
  return taken - since >= hold_s;
}

void SightCheck::Add(const SightSample& s) {
  last_ = s;
  if (have_field_ && s.field == field_) {
    still_++;
  } else {
    still_ = 1;
  }
  have_field_ = true;
  field_ = s.field;

  if (dark_) {
    dark_ = s.luma_low < limits_.dark_back;
    if (!dark_) dark_since_ = -1.0;
  } else {
    dark_ = Held(s.luma_low < limits_.dark_luma, s.taken, s.at_max_gain ? limits_.dark_s : limits_.dark_any_s,
                 dark_since_);
  }

  if (bright_) {
    bright_ = s.bright_low >= limits_.bright_back || s.bright_all >= limits_.bright_back;
    if (!bright_) bright_since_ = -1.0;
  } else {
    bright_ = Held(s.bright_low >= limits_.bright_low || s.bright_all >= limits_.bright_all, s.taken,
                   limits_.bright_s, bright_since_);
  }

  const double brighter = std::max(s.luma_left, s.luma_right);
  const bool luma_apart = brighter >= 1.0 && std::abs(s.luma_left - s.luma_right) > limits_.covered_luma * brighter;
  const double sharper = std::max(s.edges_left, s.edges_right);
  const bool edges_apart =
      sharper >= 1.0 && Ratio(std::min(s.edges_left, s.edges_right), sharper) < limits_.covered_edges;
  const bool unmatched = s.matched >= 0.0 && s.matched < limits_.covered_matched;
  covered_ = Held(luma_apart || edges_apart || unmatched, s.taken, limits_.covered_s, covered_since_);
}

void SightCheck::Reset() {
  *this = SightCheck(limits_);
}

SightReason SightCheck::Reason(double now, std::string& detail) const {
  if (last_frame_ < 0.0 || now - last_frame_ > limits_.no_frame_s) {
    detail = last_frame_ < 0.0 ? "no frame yet" : Text("no frame for %.1f s", now - last_frame_);
    return SightReason::kNoSignal;
  }
  if (still_ >= limits_.still_frames) {
    detail = Text("%.0f frames alike", still_);
    return SightReason::kNoSignal;
  }
  if (dark_) {
    detail = Text("luma %.0f below %.0f", last_.luma_low, limits_.dark_luma);
    return SightReason::kTooDark;
  }
  if (bright_) {
    detail = Text("%.0f %% bright below, %.0f %% in all", 100.0 * last_.bright_low, 100.0 * last_.bright_all);
    return SightReason::kTooBright;
  }
  if (covered_) {
    detail = Text("luma %.0f/%.0f", last_.luma_left, last_.luma_right) +
             Text(", edges %.1f/%.1f", last_.edges_left, last_.edges_right) +
             (last_.matched >= 0.0 ? Text(", %.0f %% matched", 100.0 * last_.matched) : std::string());
    return SightReason::kCovered;
  }
  detail.clear();
  return SightReason::kNone;
}

}  // namespace tron_vision
