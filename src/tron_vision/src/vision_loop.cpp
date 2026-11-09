#include "tron_vision/vision_loop.h"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace tron_vision {

namespace {

constexpr float kGainSmoothing = 0.2F;
constexpr size_t kLastShown = 3;

bool Exists(const std::string& path) {
  struct stat st {};
  return stat(path.c_str(), &st) == 0;
}

std::string Score(float v) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%.2f", static_cast<double>(v));
  return buf;
}

}  // namespace

const char* StateName(VisionState state) {
  switch (state) {
    case VisionState::kOff: return "off";
    case VisionState::kModelMissing: return "model_missing";
    case VisionState::kModelMismatch: return "model_mismatch";
    case VisionState::kModelError: return "model_error";
    case VisionState::kLocked: return "locked";
    case VisionState::kStarting: return "starting";
    case VisionState::kRunning: return "running";
    case VisionState::kPausedHot: return "paused_hot";
    case VisionState::kNoFrames: return "no_frames";
    case VisionState::kCameraError: return "camera_error";
    case VisionState::kFailed: return "failed";
  }
  return "unknown";
}

VisionLoop::VisionLoop(VisionConfig config, Camera& camera, CrashGuard& guard, std::function<double()> now)
    : config_(std::move(config)),
      camera_(camera),
      guard_(guard),
      now_(std::move(now)),
      thermal_(config_.pause_at_c, config_.resume_below_c) {
  config_.infer_rate = std::clamp(config_.infer_rate, 0.5, 5.0);
  status_.exposure.exposure = config_.exposure.exposure_max;
  status_.exposure.gain = config_.exposure.gain_min;
}

VisionLoop::~VisionLoop() {
  Shutdown();
}

void VisionLoop::Set(VisionState state, const std::string& detail) {
  status_.state = state;
  status_.detail = detail;
}

bool VisionLoop::Load() {
  loaded_ = false;
  for (const std::string* path : {&config_.library_path, &config_.model_path, &config_.labels_path}) {
    if (!Exists(*path)) {
      Set(VisionState::kModelMissing, path->empty() ? "path not set" : *path + " missing");
      return false;
    }
  }
  std::string error;
  if (!ReadClassList(config_.labels_path, names_, error)) {
    Set(VisionState::kModelMissing, error);
    return false;
  }
  if (!model_.Open(config_.library_path, config_.model_path, error)) {
    Set(VisionState::kModelError, error);
    return false;
  }
  if (!CheckModel(model_.Inputs(), model_.Outputs(), names_.size(), layout_, error)) {
    model_.Close();
    Set(VisionState::kModelMismatch, error);
    return false;
  }
  loaded_ = true;
  Set(VisionState::kOff, "model ok, " + std::to_string(layout_.classes) + " classes, runtime " + model_.SdkVersion());
  return true;
}

bool VisionLoop::StartStream(double now) {
  std::string detail;
  if (!guard_.MayStream(config_.camera_id, detail)) {
    Set(VisionState::kLocked, detail);
    return false;
  }
  next_retry_ = now + config_.retry_s;
  std::string error;
  if (!guard_.Arm(config_.camera_id, error)) {
    attempts_++;
    Set(VisionState::kCameraError, error);
    return false;
  }
  if (!camera_.SetExposure(status_.exposure, error) || !camera_.Start(error)) {
    camera_.Stop();
    guard_.Disarm(config_.camera_id);
    attempts_++;
    Set(VisionState::kCameraError, error);
    return false;
  }
  streaming_ = true;
  stream_start_ = now;
  last_frame_ = -1.0;
  frames_since_exposure_ = 0;
  next_infer_ = now;
  Set(VisionState::kStarting, "");
  return true;
}

void VisionLoop::StopStream(VisionState state, const std::string& detail) {
  if (streaming_) {
    camera_.Stop();
    guard_.Disarm(config_.camera_id);
    streaming_ = false;
  }
  if (state == VisionState::kNoFrames || state == VisionState::kCameraError) attempts_++;
  next_retry_ = now_() + config_.retry_s;
  Set(state, detail);
}

bool VisionLoop::CheckBudget(double now) {
  if (now < next_check_) return true;
  next_check_ = now + config_.check_s;
  const size_t rss = ResidentBytes(config_.statm);
  if (rss > config_.max_rss_bytes) {
    StopStream(VisionState::kFailed, "resident memory " + std::to_string(rss >> 20) + " MB over the limit");
    status_.exit_code = kExitMemory;
    return false;
  }
  // Only a camera that streams or may start again pauses; other states keep their reason.
  const bool may_stream = loaded_ && status_.state != VisionState::kLocked && attempts_ < config_.max_attempts;
  double celsius = 0.0;
  if (!may_stream || config_.temp_file.empty() || !ReadCelsius(config_.temp_file, celsius)) return true;
  const bool hot = thermal_.Paused(celsius);
  if (hot && status_.state != VisionState::kPausedHot) {
    StopStream(VisionState::kPausedHot, "SoC " + std::to_string(static_cast<int>(celsius)) + " C");
  } else if (!hot && status_.state == VisionState::kPausedHot) {
    Set(VisionState::kOff, "cooled down");
    next_retry_ = now;
  }
  return true;
}

bool VisionLoop::Step() {
  if (status_.state == VisionState::kFailed) return false;
  double now = now_();
  if (!CheckBudget(now)) return false;
  if (!streaming_) {
    if (loaded_ && status_.state != VisionState::kLocked && status_.state != VisionState::kPausedHot &&
        attempts_ < config_.max_attempts && now >= next_retry_) {
      StartStream(now);
    }
    return true;
  }
  const double deadline =
      last_frame_ < 0.0 ? stream_start_ + config_.first_frame_s : last_frame_ + config_.frame_timeout_s;
  if (now >= deadline) {
    StopStream(VisionState::kNoFrames, "no frame within " +
                                           Score(static_cast<float>(last_frame_ < 0.0 ? config_.first_frame_s
                                                                                       : config_.frame_timeout_s)) +
                                           " s, attempt " + std::to_string(attempts_ + 1) + " of " +
                                           std::to_string(config_.max_attempts));
    return true;
  }
  Nv12View frame;
  std::string error;
  const int got = camera_.Next(static_cast<int>(std::ceil((deadline - now) * 1000.0)), frame, error);
  if (got < 0) {
    StopStream(VisionState::kCameraError, error);
    return true;
  }
  if (got == 0) return true;
  OnFrame(frame, now_());
  return status_.state != VisionState::kFailed;
}

void VisionLoop::OnFrame(const Nv12View& frame, double now) {
  if (status_.state == VisionState::kStarting) Set(VisionState::kRunning, "");
  last_frame_ = now;
  const Placement place = PlaceFrame(frame.width, frame.height, config_.fit, kModelWidth, kModelHeight);
  const FrameStats stats = MeasureFrame(frame, place);

  const ColorGains target = GrayWorld(stats);
  gain_r_ += kGainSmoothing * (static_cast<float>(target.r) / 256.0F - gain_r_);
  gain_b_ += kGainSmoothing * (static_cast<float>(target.b) / 256.0F - gain_b_);
  status_.display_gains.r = static_cast<uint16_t>(std::lround(gain_r_ * 256.0F));
  status_.display_gains.b = static_cast<uint16_t>(std::lround(gain_b_ * 256.0F));

  if (++frames_since_exposure_ >= config_.settle_frames) {
    const ExposureSetting next = NextExposure(status_.exposure, stats.y, config_.exposure);
    if (next.exposure != status_.exposure.exposure || next.gain != status_.exposure.gain) {
      std::string error;
      if (camera_.SetExposure(next, error)) {
        status_.exposure = next;
      } else {
        status_.detail = error;
      }
      frames_since_exposure_ = 0;
    }
  }
  if (now >= next_infer_) Infer(frame, now);
}

bool VisionLoop::Infer(const Nv12View& frame, double now) {
  const Placement place = PlaceFrame(frame.width, frame.height, config_.fit, kModelWidth, kModelHeight);
  const ColorGains gains = config_.white_balance_inference ? status_.display_gains : ColorGains{};
  ConvertFrame(frame, place, gains, config_.swap_rb, kModelWidth, kModelHeight, rgb_);
  const double period = 1.0 / config_.infer_rate;
  next_infer_ = std::max(next_infer_ + period, now);

  const double start = now_();
  std::string error;
  if (!model_.Run(rgb_.data(), rgb_.size(), outputs_, error)) {
    StopStream(VisionState::kFailed, error);
    status_.exit_code = kExitSlowNpu;
    return false;
  }
  const double end = now_();
  status_.infer_ms = (end - start) * 1000.0;
  if (end - start > config_.slow_run_s && ++slow_runs_ >= config_.max_slow_runs) {
    StopStream(VisionState::kFailed, "inference over " + Score(static_cast<float>(config_.slow_run_s)) + " s " +
                                         std::to_string(slow_runs_) + " times");
    status_.exit_code = kExitSlowNpu;
    return false;
  }

  detections_ = DecodeDetections(layout_, model_.Outputs(), outputs_, config_.decode);
  std::string text;
  for (size_t i = 0; i < detections_.size(); i++) {
    if (i < kLastShown) {
      const uint32_t id = detections_[i].class_id;
      text += (i > 0 ? ", " : "") + (id < names_.size() ? names_[id] : std::to_string(id)) + " " +
              Score(detections_[i].score);
    }
    detections_[i] = ToSource(detections_[i], place);
  }
  if (detections_.size() > kLastShown) text += " (+" + std::to_string(detections_.size() - kLastShown) + ")";
  status_.last = text.empty() ? "none" : text;
  status_.inferences++;
  status_.frame_width = frame.width;
  status_.frame_height = frame.height;
  if (last_infer_ >= 0.0 && start > last_infer_) {
    const double rate = 1.0 / (start - last_infer_);
    status_.rate = status_.rate == 0.0 ? rate : status_.rate + 0.3 * (rate - status_.rate);
  }
  last_infer_ = start;
  return true;
}

void VisionLoop::Shutdown() {
  if (streaming_) {
    camera_.Stop();
    guard_.Disarm(config_.camera_id);
    streaming_ = false;
  }
  model_.Close();
  loaded_ = false;
}

}  // namespace tron_vision
