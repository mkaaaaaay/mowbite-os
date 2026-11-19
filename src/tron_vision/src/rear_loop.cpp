#include "tron_vision/rear_loop.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace tron_vision {

namespace {

// Longest wait for a frame per step, so that a request that ran out is seen soon.
constexpr double kFrameStepS = 0.1;
// Unit of the UVC exposure time.
constexpr double kExposureUnitS = 1e-4;
// Two frames closer than this share of the evaluation period: the second is dropped.
constexpr double kDropShare = 0.75;
// Reports of the state alone come at this share of the period, so that a late step never stretches the gap beyond it.
constexpr double kReportShare = 0.8;
// A step that opens or closes the camera can take long: the state goes out before it unless the last report is younger
// than this share of the period, which leaves the rest of the period to the camera.
constexpr double kFreshShare = 0.3;
constexpr char kHot[] = "SoC too hot";

std::string Fixed(double value, const char* format = "%.1f") {
  char buf[32];
  std::snprintf(buf, sizeof(buf), format, value);
  return buf;
}

}  // namespace

const char* DockStateName(DockState state) {
  switch (state) {
    case DockState::kOff: return "off";
    case DockState::kStarting: return "starting";
    case DockState::kMeasuring: return "measuring";
    case DockState::kFault: return "fault";
  }
  return "unknown";
}

DockReason ReasonOf(MarkerResult result) {
  switch (result) {
    case MarkerResult::kFound: return DockReason::kNone;
    case MarkerResult::kNoMarker: return DockReason::kNoMarker;
    case MarkerResult::kClipped: return DockReason::kMarkerClipped;
    case MarkerResult::kAmbiguous: return DockReason::kMarkerAmbiguous;
    case MarkerResult::kTooDark: return DockReason::kTooDark;
    case MarkerResult::kTooBright: return DockReason::kTooBright;
    case MarkerResult::kBlurred: return DockReason::kBlurred;
  }
  return DockReason::kNoMarker;
}

RearLoop::RearLoop(RearConfig config, DockCamera& camera, CrashGuard& guard, std::function<double()> now,
                   LastRequest last_request, Wait wait, Publish publish, H264Stream* stream, DmaFrames* frames)
    : config_(std::move(config)),
      camera_(camera),
      guard_(guard),
      now_(std::move(now)),
      last_request_(std::move(last_request)),
      wait_(std::move(wait)),
      publish_(std::move(publish)),
      thermal_(config_.pause_at_c, config_.resume_below_c),
      picture_(config_.view),
      frames_(frames),
      recorder_(config_.record) {
  if (stream != nullptr && frames != nullptr) video_ = std::make_unique<VideoFeed>(config_.video, *stream, now_);
}

RearLoop::~RearLoop() {
  Shutdown();
}

bool RearLoop::Asked(double now) const {
  const double at = last_request_();
  return at >= 0.0 && now < at + config_.request_hold_s;
}

bool RearLoop::CheckBudget(double now) {
  if (now < next_check_) return true;
  next_check_ = now + config_.check_s;
  const size_t rss = ResidentBytes(config_.statm);
  if (rss > config_.max_rss_bytes) {
    Fault(DockReason::kNoSignal, "resident memory " + std::to_string(rss >> 20) + " MB over the limit");
    exit_code_ = kExitMemory;
    return false;
  }
  double celsius = 0.0;
  if (!config_.temp_file.empty() && ReadCelsius(config_.temp_file, celsius)) hot_ = thermal_.Paused(celsius);
  return true;
}

DockReason RearLoop::CameraBlocked(std::string& detail) {
  // The lock is read only while the camera is closed: MayStream takes away the marker of a stream of this boot.
  if (!armed_) locked_ = !guard_.MayStream(config_.camera_id, locked_detail_);
  missing_ = !camera_.Present(missing_detail_);
  if (locked_) {
    detail = locked_detail_;
    return DockReason::kLockedAfterCrash;
  }
  if (hot_) {
    detail = kHot;
    return DockReason::kTooHot;
  }
  if (missing_) {
    detail = missing_detail_;
    return DockReason::kNoSignal;
  }
  detail.clear();
  return DockReason::kNone;
}

DockReason RearLoop::WouldFail(std::string& detail) {
  const DockReason camera = CameraBlocked(detail);
  if (!config_.not_set_up.empty()) {
    detail = config_.not_set_up;
    return DockReason::kNotSetUp;
  }
  return camera;
}

ViewAvailability RearLoop::Viewable() const {
  if (locked_) return {false, DockReason::kLockedAfterCrash, locked_detail_};
  if (hot_) return {false, DockReason::kTooHot, kHot};
  if (missing_) return {false, DockReason::kNoSignal, missing_detail_};
  if (!view_error_.empty()) return {false, DockReason::kNoSignal, view_error_};
  return {true, DockReason::kNone, ""};
}

bool RearLoop::Step() {
  if (exit_code_ != kExitNone) return false;
  const double now = now_();
  if (view_wanted_) last_view_wanted_ = now;
  if (video_ && video_->Playing()) last_video_wanted_ = now;
  if (!CheckBudget(now)) return false;
  const bool asked = Asked(now);
  const bool viewer = Held(last_view_wanted_, now) || Held(last_video_wanted_, now);
  if (!viewer) EndView();
  switch (state_) {
    case DockState::kOff: StepOff(now, asked, viewer); break;
    case DockState::kFault: StepFault(now, asked); break;
    case DockState::kStarting:
    case DockState::kMeasuring: StepCamera(now, asked, viewer); break;
  }
  if (video_) StepVideo();
  return true;
}

void RearLoop::StepVideo() {
  const bool may_start = !locked_ && !hot_ && !missing_ && view_attempts_ < config_.view_attempts;
  // The loop waits itself, for a request or a frame.
  const std::string detail = video_->Step(may_start, true);
  if (!detail.empty()) video_error_ = detail;
}

void RearLoop::StepOff(double now, bool asked, bool viewer) {
  if (asked) {
    // The request goes first: the viewer's stream ends and the camera opens anew for the marker.
    ReportOffIfStale(now);
    if (viewing_) StopCamera();
    Begin(now);
    return;
  }
  if (ReportDue(now)) {
    std::string detail;
    const DockReason reason = WouldFail(detail);
    ReportIfDue(now, reason, detail);
  }
  if (viewer) {
    StepView(now);
  } else {
    wait_(config_.idle_wait_s);
  }
}

void RearLoop::StepView(double now) {
  // A pause when hot is no failed attempt.
  if (viewing_ && hot_) {
    ReportOffIfStale(now);
    StopCamera();
  }
  if (!viewing_ && (view_attempts_ >= config_.view_attempts || now < view_next_try_ || !StartView(now))) {
    wait_(config_.idle_wait_s);
    return;
  }
  const bool first = view_last_frame_ < 0.0;
  const double limit = first ? config_.first_frame_s : config_.view_gap_s;
  const double deadline = (first ? view_started_ : view_last_frame_) + limit;
  if (now >= deadline) {
    ViewFailed("no image within " + Fixed(limit) + " s");
    return;
  }
  CapturedFrame frame;
  std::string error;
  // As long as a step of the marker at most, so that a request is seen as soon.
  const auto wait_ms = static_cast<int>(std::ceil(std::min(deadline - now, kFrameStepS) * 1000.0));
  const int got = camera_.Next(wait_ms, frame, error);
  if (got < 0) {
    ViewFailed(error);
    return;
  }
  if (got == 0) return;
  view_last_frame_ = now_();
  if (ShowFrame(frame, view_last_frame_, error)) {
    view_broken_ = 0;
  } else if (++view_broken_ >= config_.view_broken) {
    ViewFailed("broken images: " + error);
  }
}

bool RearLoop::StartView(double now) {
  // Locked, hot or missing is no failed attempt, only looked at again later; Viewable() says why.
  std::string detail;
  if (CameraBlocked(detail) != DockReason::kNone) {
    view_next_try_ = now + config_.view_retry_s;
    return false;
  }
  ReportOffIfStale(now);
  if (!camera_.Open(detail)) {
    ViewFailed(detail);
    return false;
  }
  if (!guard_.Arm(config_.camera_id, detail)) {
    ViewFailed(detail);
    return false;
  }
  armed_ = true;
  // No ManualExposure: the camera keeps its own exposure for the viewer.
  if (!camera_.Start(detail)) {
    ViewFailed(detail);
    return false;
  }
  viewing_ = true;
  view_started_ = now;
  view_last_frame_ = -1.0;
  view_broken_ = 0;
  view_error_.clear();
  return true;
}

void RearLoop::ViewFailed(const std::string& detail) {
  ReportOffIfStale(now_());
  StopCamera();
  view_attempts_++;
  view_next_try_ = now_() + config_.view_retry_s;
  view_error_ = detail;
}

void RearLoop::EndView() {
  if (viewing_) {
    ReportOffIfStale(now_());
    StopCamera();
  }
  view_attempts_ = 0;
  view_error_.clear();
}

bool RearLoop::ShowFrame(const CapturedFrame& frame, double now, std::string& error) {
  bool shown = !Held(last_view_wanted_, now) || MakePicture(frame, now, error);
  if (video_ && video_->Playing()) shown = SendVideo(frame, error) && shown;
  return shown;
}

bool RearLoop::SendVideo(const CapturedFrame& jpeg, std::string& error) {
  const auto decode = [this, &jpeg](const Nv12Buffer& out, std::string& decode_error) {
    return view_decoder_.DecodeNv12(jpeg.data, jpeg.bytes, config_.width, config_.height, config_.video_scale, out,
                                    config_.video.encoder.lines, decode_error);
  };
  CapturedFrame frame;
  if (!frames_->Write(decode, frame, error)) return false;
  frame.sequence = jpeg.sequence;
  frame.timestamp_us = jpeg.timestamp_us;
  const std::string detail = video_->Send(frame);
  if (!detail.empty()) video_error_ = detail;
  return true;
}

bool RearLoop::MakePicture(const CapturedFrame& frame, double now, std::string& error) {
  if (!picture_.Due(now)) return true;
  return view_decoder_.Decode(frame.data, frame.bytes, config_.width, config_.height, config_.view_scale, view_rgb_,
                              error) &&
         picture_.Make(view_rgb_, ScaledSize(config_.width, config_.view_scale),
                       ScaledSize(config_.height, config_.view_scale), now, error);
}

void RearLoop::StepFault(double now, bool asked) {
  if (!asked) {
    Off(now);
    return;
  }
  ReportIfDue(now, fault_reason_, fault_detail_);
  wait_(config_.idle_wait_s);
}

void RearLoop::Begin(double now) {
  std::string detail;
  const DockReason reason = WouldFail(detail);
  if (reason != DockReason::kNone) {
    Fault(reason, detail);
    return;
  }
  if (!camera_.Open(detail)) {
    Fault(DockReason::kNoSignal, detail);
    return;
  }
  // The marker goes down before the first frame, a kernel crash while streaming then locks the camera.
  if (!guard_.Arm(config_.camera_id, detail)) {
    Fault(DockReason::kNotSetUp, detail);
    return;
  }
  armed_ = true;
  if (!SetUpExposure(detail)) {
    Fault(DockReason::kNoSignal, "exposure: " + detail);
    return;
  }
  if (!camera_.Start(detail)) {
    Fault(DockReason::kNoSignal, detail);
    return;
  }
  started_ = now;
  last_good_ = -1.0;
  last_evaluated_ = -1e9;
  state_ = DockState::kStarting;
  DockReport report;
  report.state = state_;
  report.stamp = now;
  if (!manual_) report.detail = "automatic exposure: " + auto_reason_;
  Send(report);
}

bool RearLoop::SetUpExposure(std::string& error) {
  ExposureRange controls;
  ExposureSetting now;
  // A camera without manual exposure keeps its own.
  manual_ = camera_.ManualExposure(controls, now, auto_reason_);
  if (!manual_) return true;
  gain_offset_ = 1 - controls.gain_min;
  ExposureRange& range = config_.exposure.range;
  range.exposure_min = std::max(controls.exposure_min, 1);
  const auto cap = static_cast<int>(std::floor(config_.max_exposure_s / kExposureUnitS + 1e-9));
  range.exposure_max = std::max(range.exposure_min, std::min(controls.exposure_max, cap));
  range.gain_min = 1;
  range.gain_max = controls.gain_max + gain_offset_;
  exposure_ = DockExposureState{};
  exposure_.setting = {std::clamp(now.exposure, range.exposure_min, range.exposure_max),
                       std::clamp(now.gain + gain_offset_, range.gain_min, range.gain_max)};
  return camera_.SetExposure(Controls(exposure_.setting), error);
}

ExposureSetting RearLoop::Controls(const ExposureSetting& rule) const {
  return {rule.exposure, rule.gain - gain_offset_};
}

void RearLoop::StepCamera(double now, bool asked, bool viewer) {
  if (!asked) {
    Off(now);
    return;
  }
  if (hot_) {
    Fault(DockReason::kTooHot, kHot);
    return;
  }
  if (state_ == DockState::kStarting) ReportIfDue(now);
  // Frames the driver holds after a long step still count: the gap is judged only after asking for them.
  CapturedFrame frame;
  std::string error;
  const auto wait_ms = static_cast<int>(std::ceil(std::clamp(GoodBy() - now, 0.0, kFrameStepS) * 1000.0));
  const int got = camera_.Next(wait_ms, frame, error);
  if (got < 0) {
    Fault(DockReason::kNoSignal, error);
    return;
  }
  if (got == 1) Evaluate(frame, now_());
  if (state_ == DockState::kFault) return;
  if (now_() >= GoodBy()) {
    const double limit = last_good_ < 0.0 ? config_.first_frame_s : config_.frame_gap_s;
    Fault(DockReason::kNoSignal, "no usable image within " + Fixed(limit) + " s");
    return;
  }
  // The picture comes after the report of its frame, so it never delays a measurement; a broken image gives none.
  if (got == 1 && viewer) ShowFrame(frame, now_(), error);
}

// By when the next usable image has to be there.
double RearLoop::GoodBy() const {
  return last_good_ < 0.0 ? started_ + config_.first_frame_s : last_good_ + config_.frame_gap_s;
}

void RearLoop::Evaluate(const CapturedFrame& frame, double now) {
  const double taken = static_cast<double>(frame.timestamp_us) / 1e6;
  if (taken >= last_evaluated_ && taken - last_evaluated_ < kDropShare / config_.eval_rate) return;
  last_evaluated_ = taken;
  state_ = DockState::kMeasuring;
  DockReport report;
  report.state = state_;
  report.image = true;
  report.manual_exposure = manual_;
  report.exposure = manual_ ? Controls(exposure_.setting) : ExposureSetting{};
  // Without a trusted capture time the image says nothing about now.
  if (taken > now || now - taken > config_.max_frame_age_s) {
    report.stamp = now;
    report.reason = DockReason::kNoSignal;
    report.detail = "image time off by " + Fixed(now - taken, "%.3f") + " s";
    Send(report);
    return;
  }
  report.stamp = taken;
  GrayView luma;
  std::string error;
  const double before_decode = now_();
  const bool decoded = decoder_.Decode(frame.data, frame.bytes, config_.width, config_.height, luma, error);
  const double before_detect = now_();
  report.decode_ms = (before_detect - before_decode) * 1000.0;
  if (!decoded) {
    report.reason = DockReason::kNoSignal;
    report.detail = "broken image: " + error;
    Send(report);
    Record(frame, report);
    return;
  }
  last_good_ = now;
  report.detection = DetectMarker(luma, config_.lens, config_.marker);
  report.reason = ReasonOf(report.detection.result);
  if (report.detection.result == MarkerResult::kFound) {
    report.pose = FitMarkerPose(report.detection.corners, report.detection.corner_covariance, config_.lens.fx,
                                config_.mount, config_.marker_side, config_.pose);
    if (!report.pose.valid) {
      report.reason = DockReason::kFitRejected;
      report.detail = "fit error " + Fixed(report.pose.fit_error_px, "%.2f") + " px";
    }
  }
  report.detect_ms = (now_() - before_detect) * 1000.0;
  if (!manual_) {
    Send(report);
    Record(frame, report);
    return;
  }
  const DockExposureState next = NextDockExposure(exposure_, report.detection, config_.exposure);
  const bool changed =
      next.setting.exposure != exposure_.setting.exposure || next.setting.gain != exposure_.setting.gain;
  exposure_ = next;
  Send(report);
  if (changed && !camera_.SetExposure(Controls(next.setting), error)) {
    Fault(DockReason::kNoSignal, "exposure: " + error);
    return;
  }
  Record(frame, report);
}

void RearLoop::Record(const CapturedFrame& frame, const DockReport& report) {
  if (!recorder_.Due(report.stamp)) return;
  const MarkerDetection& d = report.detection;
  // One "name value" per line; reason is the number of DockReason, search_down and without_marker as after this image.
  std::string text = "taken " + Fixed(report.stamp, "%.3f") + "\nreason " +
                     std::to_string(static_cast<int>(report.reason)) + "\ndetail " + report.detail + "\nmanual " +
                     (report.manual_exposure ? "1" : "0") + "\nexposure " + std::to_string(report.exposure.exposure) +
                     "\ngain " + std::to_string(report.exposure.gain) + "\nimage_mean " + Fixed(d.image_mean, "%.1f") +
                     "\nimage_saturated " + Fixed(d.image_saturated, "%.5f") + "\nwhite " + Fixed(d.white, "%.1f") +
                     "\nblack " + Fixed(d.black, "%.1f") + "\nsaturated " + Fixed(d.saturated, "%.3f") + "\nside_px " +
                     Fixed(d.side_px, "%.1f") + "\nedge_px " + Fixed(d.edge_px, "%.2f") + "\ncorner_sigma_px " +
                     Fixed(d.corner_sigma_px, "%.3f") + "\nfit_error_px " + Fixed(report.pose.fit_error_px, "%.3f") +
                     "\nsearch_down " + (exposure_.search_down ? "1" : "0") + "\nwithout_marker " +
                     std::to_string(exposure_.without_marker) + "\n";
  std::string error;
  if (!recorder_.Write(report.stamp, frame.data, frame.bytes, text, error)) record_error_ = "recording ended: " + error;
}

void RearLoop::Fault(DockReason reason, const std::string& detail) {
  StopCamera();
  state_ = DockState::kFault;
  fault_reason_ = reason;
  fault_detail_ = detail;
  DockReport report;
  report.state = state_;
  report.stamp = now_();
  report.reason = reason;
  report.detail = detail;
  Send(report);
}

void RearLoop::Off(double now) {
  StopCamera();
  state_ = DockState::kOff;
  DockReport report;
  report.state = state_;
  report.stamp = now;
  report.reason = WouldFail(report.detail);
  Send(report);
}

void RearLoop::StopCamera() {
  // Close stops the stream and gives the camera its exposure back; the marker goes after STREAMOFF.
  camera_.Close();
  if (armed_) guard_.Disarm(config_.camera_id);
  armed_ = false;
  manual_ = false;
  viewing_ = false;
}

void RearLoop::Send(const DockReport& report) {
  last_report_ = now_();
  publish_(report);
}

bool RearLoop::ReportDue(double now) const {
  return now - last_report_ >= kReportShare * config_.report_period_s;
}

void RearLoop::ReportIfDue(double now, DockReason reason, const std::string& detail) {
  if (!ReportDue(now)) return;
  DockReport report;
  report.state = state_;
  report.stamp = now;
  report.reason = reason;
  report.detail = detail;
  Send(report);
}

void RearLoop::ReportOffIfStale(double now) {
  if (now - last_report_ < kFreshShare * config_.report_period_s) return;
  DockReport report;
  report.state = state_;
  report.stamp = now;
  report.reason = WouldFail(report.detail);
  Send(report);
}

void RearLoop::Shutdown() {
  if (video_) video_->End();
  StopCamera();
}

}  // namespace tron_vision
