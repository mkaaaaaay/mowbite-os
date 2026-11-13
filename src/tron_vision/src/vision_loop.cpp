#include "tron_vision/vision_loop.h"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace tron_vision {

namespace {

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

VisionLoop::VisionLoop(VisionConfig config, Camera& camera, CrashGuard& guard, std::function<double()> now,
                       H264Stream* stream)
    : config_(std::move(config)),
      camera_(camera),
      guard_(guard),
      now_(std::move(now)),
      thermal_(config_.pause_at_c, config_.resume_below_c),
      preview_(config_.preview) {
  config_.infer_rate = std::clamp(config_.infer_rate, 0.5, 5.0);
  status_.exposure.exposure = config_.exposure.exposure_max;
  status_.exposure.gain = config_.exposure.gain_min;
  if (stream != nullptr) video_ = std::make_unique<VideoFeed>(config_.video, *stream, now_);
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
  StopWorker();
  auto worker = std::make_shared<InferenceWorker>();
  NpuModel& model = worker->Model();
  if (!model.Open(config_.library_path, config_.model_path, config_.npu_core, error)) {
    Set(VisionState::kModelError, error);
    return false;
  }
  if (!CheckModel(model.Inputs(), model.Outputs(), names_.size(), layout_, error)) {
    model.Close();
    Set(VisionState::kModelMismatch, error);
    return false;
  }
  const std::string detail =
      "model ok, " + std::to_string(layout_.classes) + " classes, runtime " + model.SdkVersion();
  worker->Start(layout_, config_.decode);
  worker_ = std::move(worker);
  loaded_ = true;
  Set(VisionState::kOff, detail);
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
  detections_.clear();
  streams_++;
  Set(VisionState::kStarting, "");
  return true;
}

void VisionLoop::StopStream(VisionState state, const std::string& detail) {
  if (streaming_) {
    // The encoder lets go of the buffers before the camera frees them.
    if (video_) video_->End();
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
  if (!CheckBudget(now) || !CheckInference(now)) return false;
  StepVideo();
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
  SendVideo();
  return status_.state != VisionState::kFailed;
}

void VisionLoop::OnFrame(const Nv12View& frame, double now) {
  if (status_.state == VisionState::kStarting) Set(VisionState::kRunning, "");
  last_frame_ = now;
  const Placement place = PlaceFrame(frame.width, frame.height, config_.fit, kModelWidth, kModelHeight);
  const FrameStats stats = MeasureFrame(frame, place);

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
  if (now >= next_infer_ && worker_ && worker_->Idle()) Submit(frame, now);
  if (preview_wanted_ && status_.state == VisionState::kRunning && preview_.Due(now)) {
    std::string error;
    if (!preview_.Make(frame, status_.display_gains, detections_, now, error)) status_.detail = error;
  }
}

void VisionLoop::StepVideo() {
  if (!video_) return;
  // Offered while the camera may start.
  const bool may_start = loaded_ && status_.state != VisionState::kLocked &&
                         status_.state != VisionState::kPausedHot && attempts_ < config_.max_attempts;
  const std::string detail = video_->Step(may_start, streaming_);
  if (!detail.empty()) status_.detail = detail;
  status_.video = video_->Active();
  status_.fps = video_->Fps();
  status_.kbps = video_->Kbps();
}

void VisionLoop::SendVideo() {
  if (!video_ || !video_->Playing()) return;
  const int du = white_.ShiftU();
  const int dv = white_.ShiftV();
  const std::vector<Detection>& boxes = detections_;
  const auto edit = [du, dv, &boxes](const Nv12Buffer& f) {
    ShiftChroma(f.uv, f.width, f.height, f.stride, du, dv);
    // Outlines a pixel wide per 320 of the picture width, 4 at 1280.
    DrawBoxes(f, boxes, std::max(2U, f.width / 320));
  };
  const std::string detail = video_->Send(camera_, edit);
  if (!detail.empty()) status_.detail = detail;
  status_.video = video_->Active();
  status_.fps = video_->Fps();
  status_.kbps = video_->Kbps();
  status_.encode_ms = video_->EncodeMs();
}

void VisionLoop::Submit(const Nv12View& frame, double now) {
  job_place_ = PlaceFrame(frame.width, frame.height, config_.fit, kModelWidth, kModelHeight);
  const ColorGains gains = config_.white_balance_inference ? status_.display_gains : ColorGains{};
  ConvertFrame(frame, job_place_, gains, config_.swap_rb, kModelWidth, kModelHeight, worker_->Input());
  job_width_ = frame.width;
  job_height_ = frame.height;
  job_submitted_ = now;
  job_stream_ = streams_;
  next_infer_ = std::max(next_infer_ + 1.0 / config_.infer_rate, now);
  worker_->Submit();
}

bool VisionLoop::CheckInference(double now) {
  if (!worker_) return true;
  InferenceWorker::Result result;
  if (worker_->Take(result)) Adopt(result);
  if (status_.state == VisionState::kFailed) return false;
  if (worker_->Busy() && now - job_submitted_ > config_.hang_s) {
    StopStream(VisionState::kFailed,
               "inference has not ended after " + Score(static_cast<float>(config_.hang_s)) + " s");
    status_.exit_code = kExitSlowNpu;
    return false;
  }
  return true;
}

void VisionLoop::Adopt(const InferenceWorker::Result& result) {
  if (!result.ok) {
    StopStream(VisionState::kFailed, result.error);
    status_.exit_code = kExitSlowNpu;
    return;
  }
  status_.infer_ms = result.run_s * 1000.0;
  if (result.run_s > config_.slow_run_s && ++slow_runs_ >= config_.max_slow_runs) {
    StopStream(VisionState::kFailed, "inference over " + Score(static_cast<float>(config_.slow_run_s)) + " s " +
                                         std::to_string(slow_runs_) + " times");
    status_.exit_code = kExitSlowNpu;
    return;
  }
  // Boxes of a stream that has ended no longer fit the picture.
  if (!streaming_ || job_stream_ != streams_) return;

  detections_.clear();
  std::string text;
  for (size_t i = 0; i < result.detections.size(); i++) {
    const Detection& d = result.detections[i];
    if (i < kLastShown) {
      text += (i > 0 ? ", " : "") + (d.class_id < names_.size() ? names_[d.class_id] : std::to_string(d.class_id)) +
              " " + Score(d.score);
    }
    detections_.push_back(ToSource(d, job_place_));
  }
  if (result.detections.size() > kLastShown) {
    text += " (+" + std::to_string(result.detections.size() - kLastShown) + ")";
  }
  status_.last = text.empty() ? "none" : text;
  status_.inferences++;
  status_.frame_width = job_width_;
  status_.frame_height = job_height_;
  if (last_infer_ >= 0.0 && job_submitted_ > last_infer_) {
    const double rate = 1.0 / (job_submitted_ - last_infer_);
    status_.rate = status_.rate == 0.0 ? rate : status_.rate + 0.3 * (rate - status_.rate);
  }
  last_infer_ = job_submitted_;
}

void VisionLoop::StopWorker() {
  if (!worker_) return;
  // A run that hangs keeps its thread; the process ends anyway.
  worker_->Stop(config_.slow_run_s);
  worker_.reset();
}

void VisionLoop::Shutdown() {
  if (video_) video_->End();
  if (streaming_) {
    camera_.Stop();
    guard_.Disarm(config_.camera_id);
    streaming_ = false;
  }
  StopWorker();
  loaded_ = false;
}

}  // namespace tron_vision
