#ifndef TRON_VISION_REAR_LOOP_H
#define TRON_VISION_REAR_LOOP_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "tron_vision/budget.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/dock_exposure.h"
#include "tron_vision/dock_marker.h"
#include "tron_vision/dock_pose.h"
#include "tron_vision/jpeg_luma.h"
#include "tron_vision/lens_model.h"
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

// The dock marker without ROS: the camera is open only while asked, a request holds for request_hold_s from its
// arrival. Every evaluated image gives one report, and every state at least one per report_period_s. FAULT stays
// until the requests run out. One thread calls Step() in a loop; it blocks at most until the next frame deadline or
// the idle wait and returns false once the process has to end.
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

  int ExitCode() const {
    return exit_code_;
  }
  DockState State() const {
    return state_;
  }

 private:
  bool Asked(double now) const;
  bool CheckBudget(double now);
  // Why a request would fail now; kNone if it would not.
  DockReason WouldFail(std::string& detail);
  void StepOff(double now, bool asked);
  void StepFault(double now, bool asked);
  void StepCamera(double now, bool asked);
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
};

}  // namespace tron_vision

#endif  // TRON_VISION_REAR_LOOP_H
