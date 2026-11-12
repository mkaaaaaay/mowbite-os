#ifndef TRON_VISION_REAR_LOOP_H
#define TRON_VISION_REAR_LOOP_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tron_vision/budget.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/dock_exposure.h"
#include "tron_vision/dock_marker.h"
#include "tron_vision/dock_pose.h"
#include "tron_vision/jpeg_luma.h"
#include "tron_vision/lens_model.h"
#include "tron_vision/preview.h"
#include "tron_vision/rear_camera.h"

namespace tron_vision {

// The states of hw/dock_marker.
enum class DockState {
  kOff,
  kStarting,
  kMeasuring,
  kFault,
};

// Why a report carries no measurement; the first seven are about the image, the rest about the camera.
enum class DockReason {
  kNone,
  kNoMarker,
  kMarkerClipped,
  kMarkerAmbiguous,
  kTooDark,
  kTooBright,
  kBlurred,
  kFitRejected,
  kNoSignal,
  kNotSetUp,
  kLockedAfterCrash,
  kTooHot,
};

const char* DockStateName(DockState state);

// What the detector found, as a reason; kFound gives kNone.
DockReason ReasonOf(MarkerResult result);

struct RearConfig {
  std::string camera_id = "rear";
  // Empty when lens, mount, marker side and pattern are given; else what is missing, and the loop never measures.
  std::string not_set_up;
  LensModel lens;
  CameraMount mount;
  double marker_side = 0.0;
  MarkerParams marker;
  PoseLimits pose;
  // The range is read from the camera; the exposure time is held at or below max_exposure_s against motion blur.
  DockExposureLimits exposure;
  double max_exposure_s = 0.005;
  uint32_t width = 1920;
  uint32_t height = 1080;
  // Images evaluated per second; the frames between are dropped.
  double eval_rate = 15.0;
  // From hw/dock_marker: how long a request holds, and the longest silence in any state.
  double request_hold_s = 0.0;
  double report_period_s = 0.0;
  // No image after the start within first_frame_s, or no usable one within frame_gap_s after the last: FAULT.
  double first_frame_s = 3.0;
  double frame_gap_s = 0.5;
  // An image taken longer ago than this when it comes, or in the future, has a broken time and is not measured.
  double max_frame_age_s = 0.5;
  // Where the markers of the crash guard are kept.
  std::string marker_dir = "/var/lib/mowbite-vision";
  double pause_at_c = 80.0;
  double resume_below_c = 75.0;
  // Empty: no temperature check.
  std::string temp_file;
  size_t max_rss_bytes = 100UL * 1024 * 1024;
  std::string statm = "/proc/self/statm";
  double check_s = 5.0;
  // Longest wait per step while the camera is off.
  double idle_wait_s = 0.1;
  // The live picture for a viewer: the image scaled in the decoder, as JPEG within the limits.
  JpegScale view_scale{3, 8};
  JpegLimits view;
  // How long the demand of a viewer holds after the last sign of one.
  double view_hold_s = kViewHoldS;
  // While the camera is open for a viewer alone, an attempt fails without an image within first_frame_s after the
  // start or view_gap_s after the last one, or with view_broken broken images in a row; view_attempts per viewing,
  // view_retry_s apart.
  double view_gap_s = 1.0;
  int view_broken = 5;
  int view_attempts = 3;
  double view_retry_s = 5.0;
};

// One message of hw/dock_marker, with the pixel values of its image for the measurements.
struct DockReport {
  DockState state = DockState::kOff;
  // When the image was taken, on the clock of the loop; without an image when the report was made.
  double stamp = 0.0;
  bool image = false;
  // Valid with a measurement; its yaw entries count only with yaw_valid.
  MarkerPose pose;
  DockReason reason = DockReason::kNone;
  std::string detail;
  // Only with an image.
  MarkerDetection detection;
  bool manual_exposure = false;
  // In the units of the camera controls.
  ExposureSetting exposure;
  double decode_ms = 0.0;
  double detect_ms = 0.0;
};

// Whether a viewer can get pictures of the camera, else why: kLockedAfterCrash, kTooHot or kNoSignal.
struct ViewAvailability {
  bool available = false;
  DockReason reason = DockReason::kNone;
  std::string detail;
};

// The dock marker without ROS: the camera is open while asked, a request holds for request_hold_s from its arrival.
// Every evaluated image gives one report, and every state at least one per report_period_s. FAULT stays until the
// requests run out. One thread calls Step() in a loop; it blocks at most until the next frame deadline or the idle
// wait and returns false once the process has to end.
// A viewer gets a live picture from the same camera: without a request the camera opens for the viewer alone with
// its own exposure. A request goes first and takes the camera with the manual exposure of the marker; its pictures
// are made after the report of their frame. Failures while only a viewer has the camera never reach hw/dock_marker.
class RearLoop {
 public:
  // Arrival of the last request on the clock of the loop, a negative value without one.
  using LastRequest = std::function<double()>;
  // Waits up to this long; it may return early when a request comes.
  using Wait = std::function<void(double)>;
  using Publish = std::function<void(const DockReport&)>;

  RearLoop(RearConfig config, DockCamera& camera, CrashGuard& guard, std::function<double()> now,
           LastRequest last_request, Wait wait, Publish publish);
  ~RearLoop();
  RearLoop(const RearLoop&) = delete;
  RearLoop& operator=(const RearLoop&) = delete;

  bool Step();
  void Shutdown();

  // Whether a viewer wants pictures; the demand holds view_hold_s after the last call with true.
  void SetViewWanted(bool wanted) {
    view_wanted_ = wanted;
  }
  const BoundedJpeg& Picture() const {
    return picture_;
  }
  ViewAvailability Viewable() const;

  int ExitCode() const {
    return exit_code_;
  }
  DockState State() const {
    return state_;
  }

 private:
  bool Asked(double now) const;
  bool CheckBudget(double now);
  // Why the camera cannot stream now, kNone if it can; the result is kept for Viewable().
  DockReason CameraBlocked(std::string& detail);
  // Why a request would fail now; kNone if it would not.
  DockReason WouldFail(std::string& detail);
  void StepOff(double now, bool asked, bool viewer);
  void StepFault(double now, bool asked);
  void StepCamera(double now, bool asked, bool viewer);
  void StepView(double now);
  // Opens the camera for the viewer alone; false if it cannot now.
  bool StartView(double now);
  // An attempt of the viewer failed: the camera closes and the next attempt waits.
  void ViewFailed(const std::string& detail);
  void EndView();
  // A picture of the frame when one is due; false with the error if the image does not decode or encode.
  bool MakePicture(const CapturedFrame& frame, double now, std::string& error);
  void Begin(double now);
  // Manual exposure with the range of the camera; true also for a camera that keeps its own.
  bool SetUpExposure(std::string& error);
  // A setting of the rule in the units of the camera controls.
  ExposureSetting Controls(const ExposureSetting& rule) const;
  void Evaluate(const CapturedFrame& frame, double now);
  void Fault(DockReason reason, const std::string& detail);
  void Off(double now);
  void StopCamera();
  void Send(const DockReport& report);
  bool ReportDue(double now) const;
  // A report of the state alone, once per period.
  void ReportIfDue(double now, DockReason reason = DockReason::kNone, const std::string& detail = "");

  RearConfig config_;
  DockCamera& camera_;
  CrashGuard& guard_;
  std::function<double()> now_;
  LastRequest last_request_;
  Wait wait_;
  Publish publish_;
  ThermalGate thermal_;
  JpegLuma decoder_;
  DockState state_ = DockState::kOff;
  DockReason fault_reason_ = DockReason::kNone;
  std::string fault_detail_;
  int exit_code_ = kExitNone;
  bool hot_ = false;
  bool armed_ = false;
  double next_check_ = 0.0;
  double last_report_ = -1e9;
  double started_ = 0.0;
  // Of the last usable image on the clock of the loop, and of the last evaluated one by its capture time.
  double last_good_ = -1.0;
  double last_evaluated_ = -1.0;
  bool manual_ = false;
  // Why the camera keeps its own exposure.
  std::string auto_reason_;
  // The gain as the rule counts it is the control value plus this, so that it starts at 1.
  int gain_offset_ = 0;
  DockExposureState exposure_;
  // Of the last check while the camera was closed, and why.
  bool locked_ = false;
  bool missing_ = false;
  std::string locked_detail_;
  std::string missing_detail_;
  JpegDecoder view_decoder_;
  std::vector<uint8_t> view_rgb_;
  BoundedJpeg picture_;
  bool view_wanted_ = false;
  double last_view_wanted_ = -1.0;
  // The camera is open for the viewer alone.
  bool viewing_ = false;
  int view_attempts_ = 0;
  int view_broken_ = 0;
  double view_started_ = 0.0;
  double view_last_frame_ = -1.0;
  double view_next_try_ = 0.0;
  // Why the last attempt of this viewing failed; empty after a good start.
  std::string view_error_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_REAR_LOOP_H
