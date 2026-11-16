#ifndef TRON_VISION_STEREO_LOOP_H
#define TRON_VISION_STEREO_LOOP_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tron_vision/budget.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/frame_recorder.h"
#include "tron_vision/gain_level.h"
#include "tron_vision/h264_encoder.h"
#include "tron_vision/h264_stream.h"
#include "tron_vision/jpeg_luma.h"
#include "tron_vision/rtsp_server.h"
#include "tron_vision/v4l2_camera.h"

namespace tron_vision {

struct StereoConfig {
  std::string camera_id = "stereo";
  // Result of the camera setup at boot; only the line "configured" lets the camera stream.
  std::string setup_file = "/run/mowbite/stereo";
  // How long after the start of the process a missing result counts as not there yet.
  double setup_wait_s = 120.0;
  // Where the result of the self-test of this boot is kept, next to the markers of the crash guard.
  std::string marker_dir = "/var/lib/mowbite-vision";
  std::string boot_id_file = "/proc/sys/kernel/random/boot_id";
  // The self-test passes with this many full frames within this time.
  int selftest_frames = 10;
  double selftest_s = 2.0;
  double first_frame_s = 2.0;
  // A longer gap between two frames ends the stream for this boot.
  double frame_gap_s = 1.0;
  // False: no stream while the blade runs or while the state of the mower is unknown.
  bool stream_while_mowing = false;
  // hw/status older than this counts as unknown.
  double mower_status_max_age_s = 1.0;
  EncoderSettings encoder;
  // An encode that takes longer counts as a failure of the encoder.
  double slow_encode_s = 0.5;
  // Failures of the encoder, and of opening the camera, before the camera gives up until the process restarts.
  int max_attempts = 3;
  double retry_s = 5.0;
  double pause_at_c = 80.0;
  double resume_below_c = 75.0;
  // Empty: no temperature check.
  std::string temp_file;
  size_t max_rss_bytes = 150UL * 1024 * 1024;
  std::string statm = "/proc/self/statm";
  double check_s = 5.0;
  // Wait for a viewer per step while nothing streams.
  int idle_wait_ms = 200;
  // Keeps both eyes of the frames of a stream as gray JPEG, for work on the pictures off the mower; off with
  // max_bytes 0. It records only while a viewer holds the stream.
  RecordLimits record = {"", 0, 3600.0, 0.07};
  int record_quality = 92;
  // Where the gain level is wished for by the luma of the frames of a stream, written anew every gain_wish_s with
  // the clock of the loop, which must be CLOCK_MONOTONIC; empty: no wish and no lock. The setup of the platform sets
  // the level (tron-stereo-ae watch), names it in gain_level_file and holds gain_lock_file while it writes
  // registers; the loop holds that lock while a stream starts or stops, waiting for it at most gain_lock_wait_s.
  std::string gain_wish_file;
  std::string gain_level_file = "/run/mowbite/stereo-level";
  std::string gain_lock_file = "/run/mowbite/stereo.lock";
  double gain_wish_s = 10.0;
  double gain_lock_wait_s = 0.5;
  GainWishLimits gain;
};

// The mower as the last message on hw/status gave it.
struct MowerView {
  bool received = false;
  // When the message came, on the clock of the loop, not its stamp.
  double received_at = 0.0;
  bool blade_on = false;
  bool moving = false;
};

// What the mower allows the stereo camera; checked in this order, the first hit counts.
enum class MowerGate {
  kOpen,
  // No message yet, or the last one older than max_age_s.
  kUnknown,
  kBlade,
  // The wheels are commanded to move; asked for the self-test only.
  kMoving,
};

MowerGate CheckMower(const MowerView& mower, double now, double max_age_s, bool for_selftest);

enum class StereoState {
  kOff,
  kWaitingSetup,
  kNotConfigured,
  kLocked,
  kWaitingIdle,
  kSelftest,
  kReady,
  kStreaming,
  kPausedBlade,
  kPausedDriver,
  kPausedHot,
  kNoFrames,
  kStalled,
  kEncoderMissing,
  kEncoderError,
  kFailed,
};

const char* StateName(StereoState state);

struct StereoStatus {
  StereoState state = StereoState::kOff;
  std::string detail;
  // Frame rate of the camera as measured in the self-test.
  double camera_fps = 0.0;
  // Of the running stream, over the last second.
  double fps = 0.0;
  double kbps = 0.0;
  double encode_ms = 0.0;
  // Since the start of the process: frames sent, and frames the driver dropped during a stream.
  uint64_t frames = 0;
  uint64_t lost = 0;
  // Why the recording ended; empty while it runs or is off.
  std::string record_error;
  // The gain level wished for and the mean luma of the last second of frames it came from; empty without a wish.
  std::string gain_level;
  float luma = 0.0F;
  // Why the last wish could not be written; empty after one that could.
  std::string gain_error;
  int exit_code = kExitNone;
};

// Live picture of the stereo camera without ROS: the left eye as H.264 for one RTSP client, which is the viewer
// signal. The camera streams only after one self-test per boot, taken while the mower stands with the blade off; a
// stream without frames or with a gap ends streaming for that boot. One thread calls Step() in a loop; it blocks at
// most until the next frame deadline or the idle wait and returns false once the process has to end.
class StereoLoop {
 public:
  StereoLoop(StereoConfig config, FrameSource& camera, H264Encoder& encoder, RtspServer& server, CrashGuard& guard,
             std::function<double()> now);
  ~StereoLoop();
  StereoLoop(const StereoLoop&) = delete;
  StereoLoop& operator=(const StereoLoop&) = delete;

  // The mower as last seen on hw/status, with the time the message came.
  void SetMowerStatus(const MowerView& mower);
  bool Step();
  void Shutdown();

  const StereoStatus& Status() const {
    return status_;
  }

 private:
  enum class Phase {
    kInit,
    kSetup,
    kMarkers,
    kSelftestWait,
    kSelftest,
    kPrepare,
    kIdle,
    kStream,
    kEnd,
  };
  enum class Gate {
    kOpen,
    kDriver,
    kBlade,
    kMoving,
    kHot,
  };

  void Set(StereoState state, const std::string& detail);
  // Ends in a state that holds until the process restarts.
  void End(StereoState state, const std::string& detail);
  // Whether a stream may run now; `for_selftest` also asks for a mower that stands.
  Gate CheckGate(double now, bool for_selftest, std::string& why) const;
  Gate CheckMowerGate(double now, bool for_selftest, std::string& why) const;
  void Pause(Gate gate, const std::string& why);
  bool CheckBudget(double now);
  void CheckSetup(double now);
  void CheckMarkers();
  void StartSelftest(double now);
  void StepSelftest(double now);
  void Prepare(double now);
  void StepIdle(double now);
  void StartStream(double now);
  void StepStream(double now);
  void SendFrame(const CapturedFrame& frame);
  // Keeps the frame when one is due, after it went to the viewer.
  void Record(const CapturedFrame& frame);
  // Writes the wish when it is due.
  void StepGainWish(double now);
  // Takes the luma of a frame of the stream; once a second of frames is full the wish follows it.
  void AddLuma(const CapturedFrame& frame, double now);
  // The lock file while a wish is on, else none.
  std::string GainLock() const;
  void StopStream();
  // Notes that the camera gave no picture, which holds for this boot.
  void FailForThisBoot(StereoState state, const std::string& detail);
  // Counts a failure that may pass; true once it has happened too often.
  bool CountFailure(int& count, double now);
  std::string Marker(const char* kind) const;

  StereoConfig config_;
  FrameSource& camera_;
  H264Encoder& encoder_;
  RtspServer& server_;
  CrashGuard& guard_;
  std::function<double()> now_;
  ThermalGate thermal_;
  StereoStatus status_;
  Phase phase_ = Phase::kInit;
  double started_;
  bool streaming_ = false;
  bool hot_ = false;
  MowerView mower_;
  double next_check_ = 0.0;
  double next_retry_ = 0.0;
  int camera_failures_ = 0;
  int encoder_failures_ = 0;
  // Self-test and stream.
  double stream_start_ = 0.0;
  double last_frame_ = -1.0;
  int test_frames_ = 0;
  int64_t test_first_us_ = 0;
  int64_t test_last_us_ = 0;
  bool have_sequence_ = false;
  uint32_t last_sequence_ = 0;
  H264Stream stream_;
  FrameRecorder recorder_;
  YuyvGrayJpeg record_jpeg_;
  std::vector<uint8_t> record_image_;
  GainWishState gain_;
  bool gain_started_ = false;
  double next_wish_ = 0.0;
  double luma_sum_ = 0.0;
  int luma_frames_ = 0;
  double luma_from_ = 0.0;
};

}  // namespace tron_vision

#endif  // TRON_VISION_STEREO_LOOP_H
