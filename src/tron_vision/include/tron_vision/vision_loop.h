#ifndef TRON_VISION_VISION_LOOP_H
#define TRON_VISION_VISION_LOOP_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tron_vision/budget.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/exposure.h"
#include "tron_vision/frame_convert.h"
#include "tron_vision/model_check.h"
#include "tron_vision/npu_model.h"
#include "tron_vision/yolo_decode.h"

namespace tron_vision {

// A camera that streams NV12 and takes exposure settings.
class Camera {
 public:
  virtual ~Camera() = default;
  virtual bool Start(std::string& error) = 0;
  virtual void Stop() = 0;
  // As V4l2Capture::Next; the frame stays valid until the next call or Stop().
  virtual int Next(int timeout_ms, Nv12View& frame, std::string& error) = 0;
  virtual bool SetExposure(const ExposureSetting& setting, std::string& error) = 0;
};

struct VisionConfig {
  std::string camera_id = "oa_left";
  std::string library_path;
  std::string model_path;
  std::string labels_path;
  // Inferences per second, held within 0.5..5.
  double infer_rate = 2.0;
  Fit fit = Fit::kCrop;
  bool swap_rb = false;
  // Gray world for the inference too; for the shown image it is always on.
  bool white_balance_inference = false;
  DecodeParams decode;
  ExposureLimits exposure;
  // Frames between two exposure steps, the sensor takes a few to apply one.
  int settle_frames = 3;
  double first_frame_s = 2.0;
  double frame_timeout_s = 1.0;
  int max_attempts = 3;
  double retry_s = 5.0;
  double slow_run_s = 2.0;
  int max_slow_runs = 3;
  double pause_at_c = 80.0;
  double resume_below_c = 75.0;
  // Empty: no temperature check.
  std::string temp_file;
  size_t max_rss_bytes = 300UL * 1024 * 1024;
  std::string statm = "/proc/self/statm";
  double check_s = 10.0;
};

enum class VisionState {
  kOff,
  kModelMissing,
  kModelMismatch,
  kModelError,
  kLocked,
  kStarting,
  kRunning,
  kPausedHot,
  kNoFrames,
  kCameraError,
  kFailed,
};

const char* StateName(VisionState state);

enum ExitCode : int {
  kExitNone = 0,
  kExitMemory = 3,
  kExitSlowNpu = 4,
};

struct VisionStatus {
  VisionState state = VisionState::kOff;
  std::string detail;
  // Detections of the last inference, short text for the app ("person 0.81, dog 0.40" or "none").
  std::string last;
  double rate = 0.0;
  double infer_ms = 0.0;
  ExposureSetting exposure;
  ColorGains display_gains;
  int exit_code = kExitNone;
};

// Camera, exposure, inference and limits without ROS. The caller calls Step() in a loop; it blocks at most until the
// next frame deadline and returns false once the process has to end (status().exit_code).
class VisionLoop {
 public:
  VisionLoop(VisionConfig config, Camera& camera, CrashGuard& guard, std::function<double()> now);
  ~VisionLoop();
  VisionLoop(const VisionLoop&) = delete;
  VisionLoop& operator=(const VisionLoop&) = delete;

  // Loads class list and model and checks them; on failure the state says why and nothing streams.
  bool Load();
  bool Step();
  void Shutdown();

  const VisionStatus& Status() const {
    return status_;
  }
  // Boxes of the last inference in source pixels.
  const std::vector<Detection>& Detections() const {
    return detections_;
  }
  const std::vector<std::string>& ClassNames() const {
    return names_;
  }

 private:
  bool StartStream(double now);
  void StopStream(VisionState state, const std::string& detail);
  void OnFrame(const Nv12View& frame, double now);
  bool Infer(const Nv12View& frame, double now);
  bool CheckBudget(double now);
  void Set(VisionState state, const std::string& detail);

  VisionConfig config_;
  Camera& camera_;
  CrashGuard& guard_;
  std::function<double()> now_;
  NpuModel model_;
  ModelLayout layout_;
  std::vector<std::string> names_;
  ThermalGate thermal_;
  VisionStatus status_;
  std::vector<Detection> detections_;
  std::vector<uint8_t> rgb_;
  std::vector<std::vector<int8_t>> outputs_;
  bool loaded_ = false;
  bool streaming_ = false;
  int attempts_ = 0;
  double stream_start_ = 0.0;
  double last_frame_ = 0.0;
  double next_retry_ = 0.0;
  double next_infer_ = 0.0;
  double next_check_ = 0.0;
  double last_infer_ = -1.0;
  int frames_since_exposure_ = 0;
  int slow_runs_ = 0;
  float gain_r_ = 1.0F;
  float gain_b_ = 1.0F;
};

}  // namespace tron_vision

#endif  // TRON_VISION_VISION_LOOP_H
