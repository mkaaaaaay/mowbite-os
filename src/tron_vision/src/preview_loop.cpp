#include "tron_vision/preview_loop.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace tron_vision {

namespace {

std::string Seconds(double v) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%.2f", v);
  return buf;
}

}  // namespace

const char* StateName(PreviewState state) {
  switch (state) {
    case PreviewState::kOff: return "off";
    case PreviewState::kLocked: return "locked";
    case PreviewState::kStarting: return "starting";
    case PreviewState::kRunning: return "running";
    case PreviewState::kPausedHot: return "paused_hot";
    case PreviewState::kNoFrames: return "no_frames";
    case PreviewState::kCameraError: return "camera_error";
    case PreviewState::kFailed: return "failed";
  }
  return "unknown";
}

PreviewLoop::PreviewLoop(PreviewLoopConfig config, Camera& camera, CrashGuard& guard, std::function<double()> now,
                         H264Stream* stream)
    : config_(std::move(config)),
      camera_(camera),
      guard_(guard),
      now_(std::move(now)),
      thermal_(config_.pause_at_c, config_.resume_below_c),
      preview_(config_.preview) {
  status_.exposure.exposure = config_.exposure.exposure_max;
  status_.exposure.gain = config_.exposure.gain_min;
  if (stream != nullptr) video_ = std::make_unique<VideoFeed>(config_.video, *stream, now_);
}

PreviewLoop::~PreviewLoop() {
  Shutdown();
}

void PreviewLoop::Set(PreviewState state, const std::string& detail) {
  status_.state = state;
  status_.detail = detail;
}

bool PreviewLoop::StartStream(double now) {
  std::string detail;
  if (!guard_.MayStream(config_.camera_id, detail)) {
    Set(PreviewState::kLocked, detail);
    return false;
  }
  next_retry_ = now + config_.retry_s;
  std::string error;
  if (!guard_.Arm(config_.camera_id, error)) {
    attempts_++;
    Set(PreviewState::kCameraError, error);
    return false;
  }
  if (!camera_.SetExposure(status_.exposure, error) || !camera_.Start(error)) {
    camera_.Stop();
    guard_.Disarm(config_.camera_id);
    attempts_++;
    Set(PreviewState::kCameraError, error);
    return false;
  }
  streaming_ = true;
  stream_start_ = now;
  last_frame_ = -1.0;
  frames_since_exposure_ = 0;
  Set(PreviewState::kStarting, "");
  return true;
}

void PreviewLoop::StopStream(PreviewState state, const std::string& detail) {
  Shutdown();
  const bool failed = state == PreviewState::kNoFrames || state == PreviewState::kCameraError;
  if (failed) attempts_++;
  // Only a failure waits before the next start; a viewer who comes back gets the camera at once.
  next_retry_ = now_() + (failed ? config_.retry_s : 0.0);
  Set(state, detail);
}

void PreviewLoop::EndDemand() {
  attempts_ = 0;
  if (streaming_) {
    StopStream(PreviewState::kOff, "");
  } else if (status_.state == PreviewState::kNoFrames || status_.state == PreviewState::kCameraError) {
    Set(PreviewState::kOff, "");
  }
}

bool PreviewLoop::CheckBudget(double now) {
  if (now < next_check_) return true;
  next_check_ = now + config_.check_s;
  const size_t rss = ResidentBytes(config_.statm);
  if (rss > config_.max_rss_bytes) {
    StopStream(PreviewState::kFailed, "resident memory " + std::to_string(rss >> 20) + " MB over the limit");
    status_.exit_code = kExitMemory;
    return false;
  }
  // Only a camera that streams or may start again pauses; other states keep their reason.
  const bool may_stream = status_.state != PreviewState::kLocked && attempts_ < config_.max_attempts;
  double celsius = 0.0;
  if (!may_stream || config_.temp_file.empty() || !ReadCelsius(config_.temp_file, celsius)) return true;
  const bool hot = thermal_.Paused(celsius);
  if (hot && status_.state != PreviewState::kPausedHot) {
    StopStream(PreviewState::kPausedHot, "SoC " + std::to_string(static_cast<int>(celsius)) + " C");
  } else if (!hot && status_.state == PreviewState::kPausedHot) {
    Set(PreviewState::kOff, "cooled down");
    next_retry_ = now;
  }
  return true;
}

bool PreviewLoop::Step() {
  if (status_.state == PreviewState::kFailed) return false;
  const double now = now_();
  // Known before the first viewer, so that the list of cameras shows it.
  if (!lock_checked_) {
    lock_checked_ = true;
    std::string detail;
    if (!guard_.MayStream(config_.camera_id, detail)) Set(PreviewState::kLocked, detail);
  }
  if (!CheckBudget(now)) return false;
  if (video_) {
    // Offered while the camera may start.
    const bool may_start = status_.state != PreviewState::kLocked && status_.state != PreviewState::kPausedHot &&
                           attempts_ < config_.max_attempts;
    const std::string detail = video_->Step(may_start, streaming_);
    if (!detail.empty()) status_.detail = detail;
    status_.video = video_->Active();
    status_.fps = video_->Fps();
    status_.kbps = video_->Kbps();
  }
  if (wanted_) last_jpeg_wanted_ = now;
  if (wanted_ || (video_ && video_->Playing())) last_wanted_ = now;
  if (last_wanted_ < 0.0 || now >= last_wanted_ + config_.hold_s) {
    EndDemand();
    return true;
  }
  if (!streaming_) {
    if (status_.state != PreviewState::kLocked && status_.state != PreviewState::kPausedHot &&
        attempts_ < config_.max_attempts && now >= next_retry_) {
      StartStream(now);
    }
    return true;
  }
  const double deadline =
      last_frame_ < 0.0 ? stream_start_ + config_.first_frame_s : last_frame_ + config_.frame_timeout_s;
  if (now >= deadline) {
    StopStream(PreviewState::kNoFrames,
               "no frame within " + Seconds(last_frame_ < 0.0 ? config_.first_frame_s : config_.frame_timeout_s) +
                   " s, attempt " + std::to_string(attempts_ + 1) + " of " + std::to_string(config_.max_attempts));
    return true;
  }
  Nv12View frame;
  std::string error;
  const int got = camera_.Next(static_cast<int>(std::ceil((deadline - now) * 1000.0)), frame, error);
  if (got < 0) {
    StopStream(PreviewState::kCameraError, error);
    return true;
  }
  if (got == 1) {
    OnFrame(frame, now_());
    SendVideo();
  }
  return true;
}

void PreviewLoop::SendVideo() {
  if (!video_ || !video_->Playing()) return;
  const int du = white_.ShiftU();
  const int dv = white_.ShiftV();
  const auto shift = [du, dv](const Nv12Buffer& f) { ShiftChroma(f.uv, f.width, f.height, f.stride, du, dv); };
  const std::string detail = video_->Send(camera_, shift);
  if (!detail.empty()) status_.detail = detail;
  status_.video = video_->Active();
  status_.fps = video_->Fps();
  status_.kbps = video_->Kbps();
  status_.encode_ms = video_->EncodeMs();
}

void PreviewLoop::OnFrame(const Nv12View& frame, double now) {
  if (status_.state == PreviewState::kStarting) Set(PreviewState::kRunning, "");
  last_frame_ = now;
  const FrameStats stats =
      MeasureFrame(frame, PlaceFrame(frame.width, frame.height, Fit::kPad, frame.width, frame.height));

  white_.Update(stats);
  status_.display_gains = white_.Gains();

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
  const bool jpeg_wanted = last_jpeg_wanted_ >= 0.0 && now < last_jpeg_wanted_ + config_.hold_s;
  if (jpeg_wanted && preview_.Due(now)) {
    std::string error;
    if (!preview_.Make(frame, status_.display_gains, {}, now, error)) status_.detail = error;
  }
}

void PreviewLoop::Shutdown() {
  // The encoder lets go of the buffers before the camera frees them.
  if (video_) video_->End();
  if (streaming_) {
    camera_.Stop();
    guard_.Disarm(config_.camera_id);
    streaming_ = false;
  }
}

}  // namespace tron_vision
