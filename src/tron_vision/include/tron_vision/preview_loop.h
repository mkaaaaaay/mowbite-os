#ifndef TRON_VISION_PREVIEW_LOOP_H
#define TRON_VISION_PREVIEW_LOOP_H

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "tron_vision/budget.h"
#include "tron_vision/camera.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/exposure.h"
#include "tron_vision/frame_convert.h"
#include "tron_vision/h264_stream.h"
#include "tron_vision/preview.h"
#include "tron_vision/video_feed.h"

namespace tron_vision {

struct PreviewLoopConfig {
  std::string camera_id = "oa_right";
  ExposureLimits exposure;
  // Frames between two exposure steps, the sensor takes a few to apply one.
  int settle_frames = 3;
  double first_frame_s = 2.0;
  double frame_timeout_s = 1.0;
  // Attempts per viewing; a new viewing starts with none.
  int max_attempts = 3;
  double retry_s = 5.0;
  // How long the demand of a viewer holds after the last sign of one.
  double hold_s = kViewHoldS;
  double pause_at_c = 80.0;
  double resume_below_c = 75.0;
  // Empty: no temperature check.
  std::string temp_file;
  size_t max_rss_bytes = 100UL * 1024 * 1024;
  std::string statm = "/proc/self/statm";
  double check_s = 10.0;
  PreviewConfig preview;
  // H.264 for the client of the RTSP server; the camera exports its buffers then.
  VideoFeedConfig video;
};

enum class PreviewState {
  kOff,
  kLocked,
  kStarting,
  kRunning,
  kPausedHot,
  kNoFrames,
  kCameraError,
  kFailed,
};

const char* StateName(PreviewState state);

struct PreviewStatus {
  PreviewState state = PreviewState::kOff;
  std::string detail;
  ExposureSetting exposure;
  ColorGains display_gains;
  // H.264: prepared and not given up, and the stream of the last second.
  bool video = false;
  double fps = 0.0;
  double kbps = 0.0;
  double encode_ms = 0.0;
  // Chroma shift of the gray world for the H.264 picture.
  int shift_u = 0;
  int shift_v = 0;
  int exit_code = kExitNone;
};

// A camera that only shows a live picture, without ROS: it streams only while a viewer wants pictures and for
// hold_s after, with its own exposure and white balance. Pictures go out as JPEG and, with a stream, as H.264 to the
// client of its RTSP server, which is a viewer too. The caller calls SetWanted() and Step() in a loop; Step() blocks
// at most until the next frame deadline (or the idle wait of the stream) and returns false once the process has to
// end (Status().exit_code).
class PreviewLoop {
 public:
  // `stream` may be null: JPEG only.
  PreviewLoop(PreviewLoopConfig config, Camera& camera, CrashGuard& guard, std::function<double()> now,
              H264Stream* stream = nullptr);
  ~PreviewLoop();
  PreviewLoop(const PreviewLoop&) = delete;
  PreviewLoop& operator=(const PreviewLoop&) = delete;

  void SetWanted(bool wanted) {
    wanted_ = wanted;
  }
  bool Step();
  // Closes the camera if it streams; the state stays.
  void Shutdown();

  const PreviewStatus& Status() const {
    return status_;
  }
  const Preview& GetPreview() const {
    return preview_;
  }

 private:
  bool StartStream(double now);
  void StopStream(PreviewState state, const std::string& detail);
  void EndDemand();
  void OnFrame(const Nv12View& frame, double now);
  void SendVideo();
  bool CheckBudget(double now);
  void Set(PreviewState state, const std::string& detail);

  PreviewLoopConfig config_;
  Camera& camera_;
  CrashGuard& guard_;
  std::function<double()> now_;
  std::unique_ptr<VideoFeed> video_;
  ThermalGate thermal_;
  PreviewStatus status_;
  Preview preview_;
  bool wanted_ = false;
  bool lock_checked_ = false;
  bool streaming_ = false;
  int attempts_ = 0;
  double last_wanted_ = -1.0;
  // Of the JPEG pictures alone; the RTSP client wants none.
  double last_jpeg_wanted_ = -1.0;
  double stream_start_ = 0.0;
  double last_frame_ = 0.0;
  double next_retry_ = 0.0;
  double next_check_ = 0.0;
  int frames_since_exposure_ = 0;
  WhiteBalance white_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_PREVIEW_LOOP_H
