#include "tron_vision/stereo_loop.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

#include "tron_vision/text_file.h"

namespace tron_vision {

namespace {

constexpr char kConfigured[] = "configured";
// Longest wait for a frame per step, so that the blade and the viewer are looked at often enough.
constexpr double kStreamStepS = 0.1;
// Frame rates outside are no camera of this kind but a fault in the frame times.
constexpr double kMinFps = 1.0;
constexpr double kMaxFps = 120.0;

std::string Fixed(double value, const char* format = "%.1f") {
  char buf[32];
  std::snprintf(buf, sizeof(buf), format, value);
  return buf;
}

double UnixSeconds() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

}  // namespace

MowerGate CheckMower(const MowerView& mower, double now, double max_age_s, bool for_selftest) {
  if (!mower.received || now - mower.received_at > max_age_s) return MowerGate::kUnknown;
  if (mower.blade_on) return MowerGate::kBlade;
  if (for_selftest && mower.moving) return MowerGate::kMoving;
  return MowerGate::kOpen;
}

const char* StateName(StereoState state) {
  switch (state) {
    case StereoState::kOff: return "off";
    case StereoState::kWaitingSetup: return "waiting_setup";
    case StereoState::kNotConfigured: return "not_configured";
    case StereoState::kLocked: return "locked";
    case StereoState::kWaitingIdle: return "waiting_idle";
    case StereoState::kSelftest: return "selftest";
    case StereoState::kReady: return "ready";
    case StereoState::kStreaming: return "streaming";
    case StereoState::kPausedBlade: return "paused_blade";
    case StereoState::kPausedDriver: return "paused_driver";
    case StereoState::kPausedHot: return "paused_hot";
    case StereoState::kNoFrames: return "no_frames";
    case StereoState::kStalled: return "stalled";
    case StereoState::kEncoderMissing: return "encoder_missing";
    case StereoState::kEncoderError: return "encoder_error";
    case StereoState::kFailed: return "failed";
  }
  return "unknown";
}

StereoLoop::StereoLoop(StereoConfig config, FrameSource& camera, H264Encoder& encoder, RtspServer& server,
                       CrashGuard& guard, std::function<double()> now, FrameSink* sink)
    : config_(std::move(config)),
      camera_(camera),
      encoder_(encoder),
      server_(server),
      guard_(guard),
      now_(std::move(now)),
      sink_(sink),
      thermal_(config_.pause_at_c, config_.resume_below_c),
      started_(now_()),
      stream_(encoder_, server_, config_.slow_encode_s, now_),
      recorder_(config_.record) {}

StereoLoop::~StereoLoop() {
  Shutdown();
}

void StereoLoop::Set(StereoState state, const std::string& detail) {
  status_.state = state;
  status_.detail = detail;
}

void StereoLoop::End(StereoState state, const std::string& detail) {
  server_.SetUnavailable();
  Set(state, detail);
  phase_ = Phase::kEnd;
}

std::string StereoLoop::Marker(const char* kind) const {
  return config_.marker_dir + "/" + kind + "-" + config_.camera_id;
}

void StereoLoop::SetMowerStatus(const MowerView& mower) {
  mower_ = mower;
}

StereoLoop::Gate StereoLoop::CheckGate(double now, bool for_selftest, std::string& why) const {
  if (hot_) {
    why = "SoC too hot";
    return Gate::kHot;
  }
  if (config_.stream_while_mowing && !for_selftest) return Gate::kOpen;
  return CheckMowerGate(now, for_selftest, why);
}

StereoLoop::Gate StereoLoop::CheckMowerGate(double now, bool for_selftest, std::string& why) const {
  switch (CheckMower(mower_, now, config_.mower_status_max_age_s, for_selftest)) {
    case MowerGate::kOpen: return Gate::kOpen;
    case MowerGate::kUnknown:
      why = mower_.received ? "hw/status older than " + Fixed(config_.mower_status_max_age_s) + " s"
                            : "no hw/status yet";
      return Gate::kDriver;
    case MowerGate::kBlade: why = "blade running"; return Gate::kBlade;
    case MowerGate::kMoving: why = "mower moving"; return Gate::kMoving;
  }
  why = "state of the mower unknown";
  return Gate::kDriver;
}

void StereoLoop::Pause(Gate gate, const std::string& why) {
  server_.SetUnavailable();
  switch (gate) {
    case Gate::kHot: Set(StereoState::kPausedHot, why); return;
    case Gate::kDriver: Set(StereoState::kPausedDriver, why); return;
    case Gate::kBlade: Set(StereoState::kPausedBlade, why); return;
    case Gate::kMoving:
    case Gate::kOpen: Set(StereoState::kWaitingIdle, why); return;
  }
}

bool StereoLoop::CheckBudget(double now) {
  if (now < next_check_) return true;
  next_check_ = now + config_.check_s;
  const size_t rss = ResidentBytes(config_.statm);
  if (rss > config_.max_rss_bytes) {
    StopStream();
    End(StereoState::kFailed, "resident memory " + std::to_string(rss >> 20) + " MB over the limit");
    status_.exit_code = kExitMemory;
    return false;
  }
  double celsius = 0.0;
  if (!config_.temp_file.empty() && ReadCelsius(config_.temp_file, celsius)) hot_ = thermal_.Paused(celsius);
  return true;
}

bool StereoLoop::Step() {
  if (status_.exit_code != kExitNone) return false;
  const double now = now_();
  if (!CheckBudget(now)) return false;
  StepGainWish(now);
  switch (phase_) {
    case Phase::kInit: {
      std::string text;
      if (!guard_.MayStream(config_.camera_id, text)) {
        End(StereoState::kLocked, text);
      } else if (!encoder_.Probe(text)) {
        if (!Obstacles()) {
          End(StereoState::kEncoderMissing, text);
          return true;
        }
        // Without the stream the camera still serves the obstacles.
        video_off_ = true;
        video_missing_ = true;
        video_error_ = text;
        phase_ = Phase::kSetup;
      } else {
        phase_ = Phase::kSetup;
      }
      return true;
    }
    case Phase::kSetup: CheckSetup(now); break;
    case Phase::kMarkers: CheckMarkers(); return true;
    case Phase::kSelftestWait: StartSelftest(now); break;
    case Phase::kSelftest: StepSelftest(now); return true;
    case Phase::kPrepare: Prepare(now); break;
    case Phase::kIdle: StepIdle(now); return true;
    case Phase::kStream: StepStream(now); return true;
    case Phase::kCapture: StepCapture(now); return true;
    case Phase::kEnd: break;
  }
  // Nothing streams: a client gets its answer, and the wait for one paces the loop.
  if (phase_ != Phase::kSelftest) server_.Poll(config_.idle_wait_ms);
  return true;
}

void StereoLoop::CheckSetup(double now) {
  const std::string result = FirstLine(config_.setup_file);
  if (result == kConfigured) {
    phase_ = Phase::kMarkers;
  } else if (result.empty() && now - started_ < config_.setup_wait_s) {
    Set(StereoState::kWaitingSetup, "no result of the camera setup yet");
  } else {
    Set(StereoState::kNotConfigured, result.empty() ? "no result of the camera setup" : result);
  }
}

void StereoLoop::CheckMarkers() {
  const std::string boot = FirstLine(config_.boot_id_file);
  if (boot.empty()) {
    End(StereoState::kFailed, "no boot id in " + config_.boot_id_file);
    return;
  }
  // A marker is "<boot id> <text>" and counts for that boot only.
  const auto of_this_boot = [&boot](const std::string& line, std::string& text) {
    if (line.compare(0, boot.size() + 1, boot + " ") != 0) return false;
    text = line.substr(boot.size() + 1);
    return true;
  };
  std::string text;
  if (of_this_boot(FirstLine(Marker("failed")), text)) {
    End(StereoState::kFailed, "no picture earlier in this boot: " + text);
    return;
  }
  // The passed test left the frame rate it measured.
  const double fps = of_this_boot(FirstLine(Marker("verified")), text) ? std::strtod(text.c_str(), nullptr) : 0.0;
  if (fps >= kMinFps && fps <= kMaxFps) {
    status_.camera_fps = fps;
    phase_ = Phase::kPrepare;
  } else {
    phase_ = Phase::kSelftestWait;
  }
}

bool StereoLoop::CountFailure(int& count, double now) {
  next_retry_ = now + config_.retry_s;
  return ++count >= config_.max_attempts;
}

void StereoLoop::FailForThisBoot(StereoState state, const std::string& detail) {
  StopStream();
  const std::string boot = FirstLine(config_.boot_id_file);
  std::string error;
  RemoveDurably(Marker("verified"));
  if (!WriteDurably(Marker("failed"), boot + " " + detail + "\n", error)) {
    End(state, detail + "; " + error);
    return;
  }
  End(state, detail);
}

void StereoLoop::StartSelftest(double now) {
  std::string text;
  const Gate gate = CheckGate(now, true, text);
  if (gate != Gate::kOpen) {
    Pause(gate, text);
    return;
  }
  if (now < next_retry_) {
    if (status_.state != StereoState::kWaitingIdle) Set(StereoState::kWaitingIdle, "next try of the camera");
    return;
  }
  if (!guard_.MayStream(config_.camera_id, text)) {
    End(StereoState::kLocked, text);
    return;
  }
  const FileLock lock(GainLock(), config_.gain_lock_wait_s);
  if (!lock.held()) {
    if (status_.state != StereoState::kWaitingIdle) Set(StereoState::kWaitingIdle, "the gain level is being set");
    return;
  }
  if (!camera_.Open(text)) {
    camera_.Close();
    if (CountFailure(camera_failures_, now)) {
      End(StereoState::kFailed, text);
    } else {
      Set(StereoState::kWaitingIdle, text);
    }
    return;
  }
  if (!guard_.Arm(config_.camera_id, text)) {
    camera_.Close();
    End(StereoState::kFailed, text);
    return;
  }
  streaming_ = true;
  if (!camera_.Start(text)) {
    FailForThisBoot(StereoState::kNoFrames, "self-test: " + text);
    return;
  }
  stream_start_ = now;
  test_frames_ = 0;
  Set(StereoState::kSelftest, "");
  phase_ = Phase::kSelftest;
}

void StereoLoop::StepSelftest(double now) {
  // The blade never runs next to a stream, also not for the rest of a self-test, nor does a stream go on while the
  // state of the mower is unknown; the wheels may start meanwhile.
  std::string why;
  const Gate gate = config_.stream_while_mowing ? Gate::kOpen : CheckMowerGate(now, false, why);
  if (gate != Gate::kOpen) {
    if (test_frames_ == 0) {
      FailForThisBoot(StereoState::kNoFrames, "self-test cut off before its first frame: " + why);
      return;
    }
    StopStream();
    Pause(gate, "self-test cut off: " + why);
    phase_ = Phase::kSelftestWait;
    return;
  }
  const double deadline = stream_start_ + config_.selftest_s;
  if (now >= deadline) {
    FailForThisBoot(StereoState::kNoFrames, "self-test: " + std::to_string(test_frames_) + " frames in " +
                                                Fixed(config_.selftest_s) + " s");
    return;
  }
  CapturedFrame frame;
  std::string error;
  const int got =
      camera_.Next(static_cast<int>(std::ceil(std::min(deadline - now, kStreamStepS) * 1000.0)), frame, error);
  if (got < 0) {
    FailForThisBoot(StereoState::kNoFrames, "self-test: " + error);
    return;
  }
  if (got == 0) return;
  if (test_frames_++ == 0) test_first_us_ = frame.timestamp_us;
  test_last_us_ = frame.timestamp_us;
  if (test_frames_ < config_.selftest_frames) return;
  StopStream();
  const double span_s = static_cast<double>(test_last_us_ - test_first_us_) / 1e6;
  const double fps = span_s > 0.0 ? (test_frames_ - 1) / span_s : 0.0;
  if (fps < kMinFps || fps > kMaxFps) {
    FailForThisBoot(StereoState::kNoFrames, "self-test: frame times give " + Fixed(fps) + " fps");
    return;
  }
  const std::string boot = FirstLine(config_.boot_id_file);
  RemoveDurably(Marker("failed"));
  if (!WriteDurably(Marker("verified"), boot + " " + Fixed(fps, "%.2f") + "\n", error)) {
    End(StereoState::kFailed, error);
    return;
  }
  status_.camera_fps = fps;
  phase_ = Phase::kPrepare;
}

void StereoLoop::Prepare(double now) {
  if (Obstacles()) {
    // The stream is prepared beside the running camera.
    phase_ = Phase::kIdle;
    return;
  }
  std::string text;
  const Gate gate = CheckGate(now, false, text);
  if (gate != Gate::kOpen) {
    Pause(gate, text);
    return;
  }
  if (now < next_retry_) return;
  config_.encoder.fps = static_cast<int>(std::lround(status_.camera_fps));
  // The parameter sets for the session description; the encoder is opened again for each stream.
  if (!stream_.Prepare(config_.encoder, text)) {
    if (CountFailure(encoder_failures_, now)) {
      End(StereoState::kEncoderError, text);
    } else {
      Set(StereoState::kEncoderError, text);
    }
    return;
  }
  stream_.Offer();
  Set(StereoState::kReady, "");
  phase_ = Phase::kIdle;
}

void StereoLoop::StepIdle(double now) {
  if (Obstacles()) {
    StepIdleCapture(now);
    return;
  }
  std::string text;
  const Gate gate = CheckGate(now, false, text);
  if (gate != Gate::kOpen) {
    if (status_.state == StereoState::kReady || status_.detail != text) Pause(gate, text);
    server_.Poll(config_.idle_wait_ms);
    return;
  }
  if (status_.state != StereoState::kReady) {
    stream_.Offer();
    Set(StereoState::kReady, "");
  }
  server_.Poll(config_.idle_wait_ms);
  if (!server_.Playing()) return;
  if (now < next_retry_) {
    // After a failure the next try waits; the viewer comes again on its own.
    server_.DropClient();
    return;
  }
  StartStream(now_());
}

void StereoLoop::StartStream(double now) {
  std::string text;
  if (!guard_.MayStream(config_.camera_id, text)) {
    End(StereoState::kLocked, text);
    return;
  }
  const FileLock lock(GainLock(), config_.gain_lock_wait_s);
  if (!lock.held()) {
    // The viewer comes again on its own.
    server_.DropClient();
    Set(StereoState::kReady, "the gain level is being set");
    return;
  }
  if (!stream_.Begin(text)) {
    server_.DropClient();
    if (CountFailure(encoder_failures_, now)) {
      End(StereoState::kEncoderError, text);
    } else {
      Set(StereoState::kReady, "encoder: " + text);
    }
    return;
  }
  if (!camera_.Open(text)) {
    camera_.Close();
    stream_.End();
    server_.DropClient();
    if (CountFailure(camera_failures_, now)) {
      End(StereoState::kFailed, text);
    } else {
      Set(StereoState::kReady, "camera: " + text);
    }
    return;
  }
  if (!guard_.Arm(config_.camera_id, text)) {
    stream_.End();
    camera_.Close();
    End(StereoState::kFailed, text);
    return;
  }
  streaming_ = true;
  if (!camera_.Start(text)) {
    FailForThisBoot(StereoState::kNoFrames, text);
    return;
  }
  stream_start_ = now;
  last_frame_ = -1.0;
  have_sequence_ = false;
  luma_frames_ = 0;
  Set(StereoState::kStreaming, "");
  phase_ = Phase::kStream;
}

void StereoLoop::StopStream() {
  // Without the lock in time the stream stops all the same.
  const FileLock lock(streaming_ ? GainLock() : std::string(), config_.gain_lock_wait_s);
  // The stream ends first; the encoder lets go of the buffers before the camera frees them.
  camera_.Stop();
  stream_.End();
  camera_.Close();
  if (streaming_) guard_.Disarm(config_.camera_id);
  streaming_ = false;
  status_.fps = 0.0;
  status_.kbps = 0.0;
  if (status_.capturing) sink_->OnStop();
  status_.capturing = false;
}

void StereoLoop::StepIdleCapture(double now) {
  std::string text;
  const Gate gate = CheckGate(now, false, text);
  if (gate != Gate::kOpen) {
    if (status_.detail != text) Pause(gate, text);
  } else if (now >= next_retry_) {
    StartCapture(now);
    if (phase_ == Phase::kCapture) return;
  } else if (status_.state != StereoState::kWaitingIdle) {
    Set(StereoState::kWaitingIdle, "next try of the camera");
  }
  server_.Poll(config_.idle_wait_ms);
}

void StereoLoop::StartCapture(double now) {
  std::string text;
  if (!guard_.MayStream(config_.camera_id, text)) {
    End(StereoState::kLocked, text);
    return;
  }
  const FileLock lock(GainLock(), config_.gain_lock_wait_s);
  if (!lock.held()) {
    Set(StereoState::kWaitingIdle, "the gain level is being set");
    return;
  }
  if (!camera_.Open(text)) {
    camera_.Close();
    if (CountFailure(camera_failures_, now)) {
      End(StereoState::kFailed, text);
    } else {
      Set(StereoState::kWaitingIdle, "camera: " + text);
    }
    return;
  }
  if (!guard_.Arm(config_.camera_id, text)) {
    camera_.Close();
    End(StereoState::kFailed, text);
    return;
  }
  streaming_ = true;
  if (!camera_.Start(text)) {
    FailForThisBoot(StereoState::kNoFrames, text);
    return;
  }
  stream_start_ = now;
  last_frame_ = -1.0;
  have_sequence_ = false;
  luma_frames_ = 0;
  status_.capturing = true;
  if (!video_prepared_ && !video_off_ && now >= next_video_retry_) {
    PrepareVideo(now);
  } else if (video_prepared_ && !video_off_) {
    stream_.Offer();
  }
  SetCaptureState();
  phase_ = Phase::kCapture;
}

void StereoLoop::SetCaptureState() {
  if (stream_.Running()) {
    Set(StereoState::kStreaming, "");
  } else if (video_off_) {
    Set(video_missing_ ? StereoState::kEncoderMissing : StereoState::kEncoderError, video_error_);
  } else if (!video_prepared_) {
    Set(StereoState::kEncoderError, video_error_.empty() ? "stream not prepared yet" : video_error_);
  } else {
    Set(StereoState::kReady, video_error_);
  }
}

void StereoLoop::StepCapture(double now) {
  server_.Poll(0);
  std::string text;
  const Gate gate = CheckGate(now, false, text);
  if (gate != Gate::kOpen) {
    StopStream();
    Pause(gate, text);
    phase_ = Phase::kIdle;
    return;
  }
  StepVideo(now);
  const bool first = last_frame_ < 0.0;
  const double deadline = first ? stream_start_ + config_.first_frame_s : last_frame_ + config_.frame_gap_s;
  if (now >= deadline) {
    FailForThisBoot(first ? StereoState::kNoFrames : StereoState::kStalled,
                    "no frame within " + Fixed(first ? config_.first_frame_s : config_.frame_gap_s) + " s");
    return;
  }
  CapturedFrame frame;
  const int got =
      camera_.Next(static_cast<int>(std::ceil(std::min(deadline - now, kStreamStepS) * 1000.0)), frame, text);
  if (got < 0) {
    FailForThisBoot(StereoState::kStalled, text);
    return;
  }
  if (got == 0) return;
  last_frame_ = now_();
  if (have_sequence_ && frame.sequence > last_sequence_ + 1) status_.lost += frame.sequence - last_sequence_ - 1;
  have_sequence_ = true;
  last_sequence_ = frame.sequence;
  AddLuma(frame, last_frame_);
  sink_->OnFrame(frame, last_frame_, {mower_.moving, AtMaxGain()});
  if (stream_.Running()) SendFrame(frame);
}

void StereoLoop::StepVideo(double now) {
  if (stream_.Running()) {
    if (server_.Playing()) return;
    EndVideo();
    SetCaptureState();
    return;
  }
  if (!video_prepared_ && !video_off_ && now >= next_video_retry_) PrepareVideo(now);
  if (!server_.Playing()) return;
  std::string text;
  if (!video_prepared_ || video_off_ || now < next_video_retry_) {
    server_.DropClient();
  } else if (!stream_.Begin(text)) {
    server_.DropClient();
    VideoFailed(now, "encoder: " + text);
  } else {
    video_error_.clear();
    SetCaptureState();
  }
}

void StereoLoop::PrepareVideo(double now) {
  config_.encoder.fps = static_cast<int>(std::lround(status_.camera_fps));
  std::string text;
  if (!stream_.Prepare(config_.encoder, text)) {
    VideoFailed(now, "encoder: " + text);
    return;
  }
  video_prepared_ = true;
  video_error_.clear();
  stream_.Offer();
  SetCaptureState();
}

void StereoLoop::VideoFailed(double now, const std::string& error) {
  video_error_ = error;
  next_video_retry_ = now + config_.retry_s;
  if (++encoder_failures_ >= config_.max_attempts) {
    video_off_ = true;
    server_.SetUnavailable();
  }
  SetCaptureState();
}

void StereoLoop::EndVideo() {
  stream_.End();
  status_.fps = 0.0;
  status_.kbps = 0.0;
}

bool StereoLoop::AtMaxGain() const {
  return config_.gain_wish_file.empty() || gain_.level == GainLevel::kNight;
}

void StereoLoop::StepStream(double now) {
  server_.Poll(0);
  if (!server_.Playing()) {
    StopStream();
    Set(StereoState::kReady, "");
    phase_ = Phase::kIdle;
    return;
  }
  std::string text;
  const Gate gate = CheckGate(now, false, text);
  if (gate != Gate::kOpen) {
    StopStream();
    Pause(gate, text);
    phase_ = Phase::kIdle;
    return;
  }
  const bool first = last_frame_ < 0.0;
  const double deadline = first ? stream_start_ + config_.first_frame_s : last_frame_ + config_.frame_gap_s;
  if (now >= deadline) {
    FailForThisBoot(first ? StereoState::kNoFrames : StereoState::kStalled,
                    "no frame within " + Fixed(first ? config_.first_frame_s : config_.frame_gap_s) + " s");
    return;
  }
  CapturedFrame frame;
  const int got =
      camera_.Next(static_cast<int>(std::ceil(std::min(deadline - now, kStreamStepS) * 1000.0)), frame, text);
  if (got < 0) {
    FailForThisBoot(StereoState::kStalled, text);
    return;
  }
  if (got == 0) return;
  last_frame_ = now_();
  if (have_sequence_ && frame.sequence > last_sequence_ + 1) status_.lost += frame.sequence - last_sequence_ - 1;
  have_sequence_ = true;
  last_sequence_ = frame.sequence;
  AddLuma(frame, last_frame_);
  SendFrame(frame);
}

std::string StereoLoop::GainLock() const {
  return config_.gain_wish_file.empty() ? std::string() : config_.gain_lock_file;
}

void StereoLoop::StepGainWish(double now) {
  if (config_.gain_wish_file.empty()) return;
  if (!gain_started_) {
    gain_started_ = true;
    // The level the setup holds from before this process; day without one.
    GainLevelFromLine(FirstLine(config_.gain_level_file), gain_.level);
    status_.gain_level = GainLevelName(gain_.level);
  }
  if (now < next_wish_) return;
  next_wish_ = now + config_.gain_wish_s;
  status_.gain_error.clear();
  WriteGainWish(config_.gain_wish_file, gain_.level, now, UnixSeconds(), status_.gain_error);
}

void StereoLoop::AddLuma(const CapturedFrame& frame, double now) {
  const uint32_t line_bytes = config_.encoder.line_bytes;
  const uint32_t lines = config_.encoder.lines;
  if (config_.gain_wish_file.empty() || frame.data == nullptr ||
      frame.bytes < static_cast<size_t>(line_bytes) * lines) {
    return;
  }
  if (luma_frames_++ == 0) {
    luma_from_ = now;
    luma_sum_ = 0.0;
  }
  luma_sum_ += MeanLumaLeft(frame.data, line_bytes / 2, lines, line_bytes);
  if (now - luma_from_ < 1.0) return;
  status_.luma = static_cast<float>(luma_sum_ / luma_frames_);
  luma_frames_ = 0;
  const GainLevel before = gain_.level;
  gain_ = NextGainWish(gain_, now, status_.luma, config_.gain);
  if (gain_.level == before) return;
  status_.gain_level = GainLevelName(gain_.level);
  // Written with the next step.
  next_wish_ = now;
}

void StereoLoop::SendFrame(const CapturedFrame& frame) {
  std::string text;
  const H264Stream::Sent sent = stream_.Send(frame, text);
  if (Obstacles() && sent != H264Stream::Sent::kOk) {
    // Only the stream ends; the camera runs on.
    EndVideo();
    if (sent == H264Stream::Sent::kEncoderFailed) {
      VideoFailed(now_(), "encoder: " + text);
    } else {
      SetCaptureState();
    }
    return;
  }
  if (sent == H264Stream::Sent::kEncoderFailed) {
    StopStream();
    if (CountFailure(encoder_failures_, now_())) {
      End(StereoState::kEncoderError, text);
    } else {
      Set(StereoState::kReady, "encoder: " + text);
      phase_ = Phase::kIdle;
    }
    return;
  }
  if (sent == H264Stream::Sent::kClientGone) {
    StopStream();
    Set(StereoState::kReady, "");
    phase_ = Phase::kIdle;
    return;
  }
  status_.frames = stream_.Frames();
  status_.fps = stream_.Fps();
  status_.kbps = stream_.Kbps();
  status_.encode_ms = stream_.EncodeMs();
  Record(frame);
}

void StereoLoop::Record(const CapturedFrame& frame) {
  if (!status_.record_error.empty()) return;
  const double taken = static_cast<double>(frame.timestamp_us) / 1e6;
  if (!recorder_.Due(taken)) return;
  const uint32_t line_bytes = config_.encoder.line_bytes;
  const uint32_t lines = config_.encoder.lines;
  std::string error;
  if (frame.data == nullptr || frame.bytes < static_cast<size_t>(line_bytes) * lines) {
    error = "frame of " + std::to_string(frame.bytes) + " bytes without its picture";
  } else if (record_jpeg_.Encode(frame.data, line_bytes / 2, lines, line_bytes, config_.record_quality, record_image_,
                                 error)) {
    // One "name value" per line; taken on the clock of the driver, unix the same moment on the clock of the system.
    const std::string text = "taken " + Fixed(taken, "%.6f") + "\nunix " +
                             Fixed(UnixSeconds() - (now_() - taken), "%.6f") + "\nsequence " +
                             std::to_string(frame.sequence) + "\nwidth " + std::to_string(line_bytes / 2) +
                             "\nheight " + std::to_string(lines) + "\nblade_on " + (mower_.blade_on ? "1" : "0") +
                             "\nmoving " + (mower_.moving ? "1" : "0") + "\n";
    if (recorder_.Write(taken, record_image_.data(), record_image_.size(), text, error)) return;
  }
  status_.record_error = "recording ended: " + error;
}

void StereoLoop::Shutdown() {
  StopStream();
  server_.SetUnavailable();
}

}  // namespace tron_vision
