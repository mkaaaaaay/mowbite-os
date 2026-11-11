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
                   LastRequest last_request, Wait wait, Publish publish)
    : config_(std::move(config)),
      camera_(camera),
      guard_(guard),
      now_(std::move(now)),
      last_request_(std::move(last_request)),
      wait_(std::move(wait)),
      publish_(std::move(publish)),
      thermal_(config_.pause_at_c, config_.resume_below_c) {}

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

DockReason RearLoop::WouldFail(std::string& detail) {
  if (!config_.not_set_up.empty()) {
    detail = config_.not_set_up;
    return DockReason::kNotSetUp;
  }
  if (!guard_.MayStream(config_.camera_id, detail)) return DockReason::kLockedAfterCrash;
  if (hot_) {
    detail = "SoC too hot";
    return DockReason::kTooHot;
  }
  if (!camera_.Present(detail)) return DockReason::kNoSignal;
  detail.clear();
  return DockReason::kNone;
}

bool RearLoop::Step() {
  if (exit_code_ != kExitNone) return false;
  const double now = now_();
  if (!CheckBudget(now)) return false;
  const bool asked = Asked(now);
  switch (state_) {
    case DockState::kOff: StepOff(now, asked); break;
    case DockState::kFault: StepFault(now, asked); break;
    case DockState::kStarting:
    case DockState::kMeasuring: StepCamera(now, asked); break;
  }
  return true;
}

void RearLoop::StepOff(double now, bool asked) {
  if (asked) {
    Begin(now);
    return;
  }
  if (ReportDue(now)) {
    std::string detail;
    const DockReason reason = WouldFail(detail);
    ReportIfDue(now, reason, detail);
  }
  wait_(config_.idle_wait_s);
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

void RearLoop::StepCamera(double now, bool asked) {
  if (!asked) {
    Off(now);
    return;
  }
  if (hot_) {
    Fault(DockReason::kTooHot, "SoC too hot");
    return;
  }
  const bool first = last_good_ < 0.0;
  const double limit = first ? config_.first_frame_s : config_.frame_gap_s;
  const double deadline = (first ? started_ : last_good_) + limit;
  if (now >= deadline) {
    Fault(DockReason::kNoSignal, "no usable image within " + Fixed(limit) + " s");
    return;
  }
  if (state_ == DockState::kStarting) ReportIfDue(now);
  CapturedFrame frame;
  std::string error;
  const auto wait_ms = static_cast<int>(std::ceil(std::min(deadline - now, kFrameStepS) * 1000.0));
  const int got = camera_.Next(wait_ms, frame, error);
  if (got < 0) {
    Fault(DockReason::kNoSignal, error);
    return;
  }
  if (got == 1) Evaluate(frame, now_());
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
    return;
  }
  const DockExposureState next = NextDockExposure(exposure_, report.detection, config_.exposure);
  const bool changed =
      next.setting.exposure != exposure_.setting.exposure || next.setting.gain != exposure_.setting.gain;
  exposure_ = next;
  Send(report);
  if (changed && !camera_.SetExposure(Controls(next.setting), error)) {
    Fault(DockReason::kNoSignal, "exposure: " + error);
  }
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

void RearLoop::Shutdown() {
  StopCamera();
}

}  // namespace tron_vision
