#include "tron_vision/stereo_loop.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
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

uint32_t Random() {
  return std::random_device{}();
}

}  // namespace

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
                       CrashGuard& guard, std::function<double()> now)
    : config_(std::move(config)),
      camera_(camera),
      encoder_(encoder),
      server_(server),
      guard_(guard),
      now_(std::move(now)),
      thermal_(config_.pause_at_c, config_.resume_below_c),
      started_(now_()),
      packetizer_(Random(), static_cast<uint16_t>(Random())),
      clock_(0) {}

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

void StereoLoop::SetMowerStatus(bool blade_on, bool moving) {
  mower_known_ = true;
  blade_on_ = blade_on;
  moving_ = moving;
  mower_at_ = now_();
}

StereoLoop::Gate StereoLoop::CheckGate(double now, bool for_selftest, std::string& why) const {
  if (hot_) {
    why = "SoC too hot";
    return Gate::kHot;
  }
  if (config_.stream_while_mowing && !for_selftest) return Gate::kOpen;
  if (!mower_known_ || now - mower_at_ > config_.mower_status_max_age_s) {
    why = "state of the mower unknown";
    return Gate::kUnknown;
  }
  if (blade_on_) {
    why = "blade running";
    return Gate::kBlade;
  }
  if (for_selftest && moving_) {
    why = "mower moving";
    return Gate::kBlade;
  }
  return Gate::kOpen;
}

void StereoLoop::Pause(Gate gate, const std::string& why) {
  server_.SetUnavailable();
  Set(gate == Gate::kHot ? StereoState::kPausedHot : StereoState::kPausedBlade, why);
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
  switch (phase_) {
    case Phase::kInit: {
      std::string text;
      if (!guard_.MayStream(config_.camera_id, text)) {
        End(StereoState::kLocked, text);
      } else if (!encoder_.Probe(text)) {
        End(StereoState::kEncoderMissing, text);
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
  if (CheckGate(now, true, text) != Gate::kOpen) {
    Set(StereoState::kWaitingIdle, text);
    return;
  }
  if (now < next_retry_) return;
  if (!guard_.MayStream(config_.camera_id, text)) {
    End(StereoState::kLocked, text);
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
  // The blade never runs next to a stream, also not for the rest of a self-test.
  if (!config_.stream_while_mowing && mower_known_ && blade_on_) {
    if (test_frames_ == 0) {
      FailForThisBoot(StereoState::kNoFrames, "self-test cut off by the blade before its first frame");
      return;
    }
    StopStream();
    Set(StereoState::kWaitingIdle, "self-test cut off by the blade");
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
  const int got = camera_.Next(static_cast<int>(std::ceil(std::min(deadline - now, kStreamStepS) * 1000.0)), frame, error);
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
  std::string text;
  const Gate gate = CheckGate(now, false, text);
  if (gate != Gate::kOpen) {
    Pause(gate, text);
    return;
  }
  if (now < next_retry_) return;
  config_.encoder.fps = static_cast<int>(std::lround(status_.camera_fps));
  // The parameter sets for the session description; the encoder is opened again for each stream.
  std::vector<uint8_t> header;
  bool ok = encoder_.Open(config_.encoder, text) && encoder_.Header(header, text);
  encoder_.Close();
  if (ok && !FindParameterSets(header.data(), header.size(), sets_)) {
    ok = false;
    text = "no SPS and PPS from the encoder";
  }
  if (!ok) {
    if (CountFailure(encoder_failures_, now)) {
      End(StereoState::kEncoderError, text);
    } else {
      Set(StereoState::kEncoderError, text);
    }
    return;
  }
  server_.SetAvailable(sets_);
  Set(StereoState::kReady, "");
  phase_ = Phase::kIdle;
}

void StereoLoop::StepIdle(double now) {
  std::string text;
  const Gate gate = CheckGate(now, false, text);
  if (gate != Gate::kOpen) {
    if (status_.state == StereoState::kReady || status_.detail != text) Pause(gate, text);
    server_.Poll(config_.idle_wait_ms);
    return;
  }
  if (status_.state != StereoState::kReady) {
    server_.SetAvailable(sets_);
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
  if (!encoder_.Open(config_.encoder, text)) {
    encoder_.Close();
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
    encoder_.Close();
    server_.DropClient();
    if (CountFailure(camera_failures_, now)) {
      End(StereoState::kFailed, text);
    } else {
      Set(StereoState::kReady, "camera: " + text);
    }
    return;
  }
  if (!guard_.Arm(config_.camera_id, text)) {
    encoder_.Close();
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
  clock_ = RtpClock(Random());
  window_start_ = now;
  window_frames_ = 0;
  window_bytes_ = 0;
  Set(StereoState::kStreaming, "");
  phase_ = Phase::kStream;
}

void StereoLoop::StopStream() {
  // The stream ends first; the encoder lets go of the buffers before the camera frees them.
  camera_.Stop();
  encoder_.Close();
  camera_.Close();
  if (streaming_) guard_.Disarm(config_.camera_id);
  streaming_ = false;
  status_.fps = 0.0;
  status_.kbps = 0.0;
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
  const int got = camera_.Next(static_cast<int>(std::ceil(std::min(deadline - now, kStreamStepS) * 1000.0)), frame, text);
  if (got < 0) {
    FailForThisBoot(StereoState::kStalled, text);
    return;
  }
  if (got == 0) return;
  last_frame_ = now_();
  if (have_sequence_ && frame.sequence > last_sequence_ + 1) status_.lost += frame.sequence - last_sequence_ - 1;
  have_sequence_ = true;
  last_sequence_ = frame.sequence;
  if (!SendFrame(frame)) return;
  if (last_frame_ - window_start_ >= 1.0) {
    const double span = last_frame_ - window_start_;
    status_.fps = window_frames_ / span;
    status_.kbps = static_cast<double>(window_bytes_) * 8.0 / 1000.0 / span;
    window_start_ = last_frame_;
    window_frames_ = 0;
    window_bytes_ = 0;
  }
}

bool StereoLoop::SendFrame(const CapturedFrame& frame) {
  const uint8_t* data = nullptr;
  size_t size = 0;
  std::string text;
  const double before = now_();
  bool ok = encoder_.Encode(frame, data, size, text);
  const double took = now_() - before;
  if (ok && took > config_.slow_encode_s) {
    ok = false;
    text = "encode took " + Fixed(took) + " s";
  }
  if (!ok) {
    StopStream();
    server_.DropClient();
    if (CountFailure(encoder_failures_, now_())) {
      End(StereoState::kEncoderError, text);
    } else {
      Set(StereoState::kReady, "encoder: " + text);
      phase_ = Phase::kIdle;
    }
    return false;
  }
  status_.encode_ms = took * 1000.0;
  packetizer_.Packetize(SplitAnnexB(data, size), clock_.Next(frame.timestamp_us), packets_);
  if (!server_.SendRtp(packets_)) {
    // The viewer is gone or reads too slowly; the next one starts a new stream.
    StopStream();
    Set(StereoState::kReady, "");
    phase_ = Phase::kIdle;
    return false;
  }
  status_.frames++;
  window_frames_++;
  window_bytes_ += size;
  return true;
}

void StereoLoop::Shutdown() {
  StopStream();
  server_.SetUnavailable();
}

}  // namespace tron_vision
