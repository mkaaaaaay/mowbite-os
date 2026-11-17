#ifndef TRON_VISION_VISION_LOOP_H
#define TRON_VISION_VISION_LOOP_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "tron_vision/budget.h"
#include "tron_vision/camera.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/exposure.h"
#include "tron_vision/frame_convert.h"
#include "tron_vision/inference_worker.h"
#include "tron_vision/model_check.h"
#include "tron_vision/preview.h"
#include "tron_vision/v4l2_camera.h"
#include "tron_vision/video_feed.h"
#include "tron_vision/yolo_decode.h"

namespace tron_vision {

struct VisionConfig {
  std::string camera_id = "oa_left";
  std::string library_path;
  std::string model_path;
  std::string labels_path;
  // Inferences per second, held within 0.5..5.
  double infer_rate = 2.0;
  // The NPU core the model runs on alone, one per camera process.
  int npu_core = 0;
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
  // A run that has not ended after this long ends the process.
  double hang_s = 6.0;
  double pause_at_c = 80.0;
  double resume_below_c = 75.0;
  // Empty: no temperature check.
  std::string temp_file;
  size_t max_rss_bytes = 300UL * 1024 * 1024;
  std::string statm = "/proc/self/statm";
  double check_s = 10.0;
  PreviewConfig preview;
  // H.264 with the boxes for the client of the RTSP server, and without them for the client of the plain stream; the
  // camera exports its buffers then.
  VideoFeedConfig video;
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

struct VisionStatus {
  VisionState state = VisionState::kOff;
  std::string detail;
  // Detections of the last inference, short text for the app ("person 0.81, dog 0.40" or "none").
  std::string last;
  double rate = 0.0;
  double infer_ms = 0.0;
  // Inferences since the start, and the size of the frame of the last one.
  uint64_t inferences = 0;
  uint32_t frame_width = 0;
  uint32_t frame_height = 0;
  ExposureSetting exposure;
  ColorGains display_gains;
  // H.264: prepared and not given up, and the stream of the last second.
  bool video = false;
  double fps = 0.0;
  double kbps = 0.0;
  double encode_ms = 0.0;
  int exit_code = kExitNone;
};

// Camera, exposure, inference and limits without ROS. The caller calls Step() in a loop; it blocks at most until the
// next frame deadline and returns false once the process has to end (status().exit_code).
class VisionLoop {
 public:
  // `stream` may be null: JPEG only. `plain_stream`, on the server of `stream`, carries the frames without the boxes;
  // may be null.
  VisionLoop(VisionConfig config, Camera& camera, CrashGuard& guard, std::function<double()> now,
             H264Stream* stream = nullptr, H264Stream* plain_stream = nullptr);
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
  // Pictures are made only while someone watches, each kind on its own.
  void SetPreviewWanted(bool wanted) {
    preview_wanted_ = wanted;
  }
  void SetPlainWanted(bool wanted) {
    plain_wanted_ = wanted;
  }
  const Preview& GetPreview() const {
    return preview_;
  }
  // A run of the model is submitted and has not ended.
  bool InferenceBusy() const {
    return worker_ && worker_->Busy();
  }

 private:
  bool StartStream(double now);
  void StopStream(VisionState state, const std::string& detail);
  void OnFrame(const Nv12View& frame, double now);
  void StepVideo();
  void SendVideo();
  void Submit(const Nv12View& frame, double now);
  // Takes the result of a run that ended and watches the one in progress; false once the process has to end.
  bool CheckInference(double now);
  void Adopt(const InferenceWorker::Result& result);
  void StopWorker();
  bool CheckBudget(double now);
  void Set(VisionState state, const std::string& detail);

  VisionConfig config_;
  Camera& camera_;
  CrashGuard& guard_;
  std::function<double()> now_;
  std::unique_ptr<VideoFeed> video_;
  std::unique_ptr<VideoFeed> plain_video_;
  std::shared_ptr<InferenceWorker> worker_;
  ModelLayout layout_;
  std::vector<std::string> names_;
  ThermalGate thermal_;
  VisionStatus status_;
  std::vector<Detection> detections_;
  // The run in progress: its frame, when it was submitted and in which stream.
  Placement job_place_;
  uint32_t job_width_ = 0;
  uint32_t job_height_ = 0;
  double job_submitted_ = 0.0;
  uint64_t job_stream_ = 0;
  uint64_t streams_ = 0;
  Preview preview_;
  bool preview_wanted_ = false;
  bool plain_wanted_ = false;
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
  WhiteBalance white_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_VISION_LOOP_H
