#ifndef TRON_VISION_VIDEO_FEED_H
#define TRON_VISION_VIDEO_FEED_H

#include <functional>
#include <string>

#include "tron_vision/camera.h"
#include "tron_vision/frame_convert.h"
#include "tron_vision/h264_stream.h"

namespace tron_vision {

struct VideoFeedConfig {
  EncoderSettings encoder;
  // Failures of the encoder before the camera stays with JPEG until the process restarts.
  int max_failures = 3;
  double retry_s = 5.0;
  // Wait for an RTSP client per step while the camera does not stream.
  int idle_wait_ms = 200;
};

// The H.264 side of a camera loop: prepares the stream, offers it while the camera may start, ends it without a
// client and gives up after max_failures failures of the encoder.
class VideoFeed {
 public:
  VideoFeed(VideoFeedConfig config, H264Stream& stream, std::function<double()> now);

  // Once per step. `may_start`: the camera may stream, only then the stream is prepared and offered; `streaming`: it
  // does, then the step does not wait.
  // Returns what went wrong, empty if nothing.
  std::string Step(bool may_start, bool streaming);
  // One frame of the camera to the client, changed by `edit` before the encoder takes it. Returns as Step().
  std::string Send(Camera& camera, const std::function<void(const Nv12Buffer&)>& edit);
  // One frame in a dma-buf of the caller to the client. Returns as Step().
  std::string Send(const CapturedFrame& frame);
  // Prepared and not given up.
  bool Active() const {
    return active_;
  }
  bool Playing() const {
    return active_ && stream_.Playing();
  }
  // Closes the encoder; it comes before the camera stops.
  void End() {
    stream_.End();
  }
  double Fps() const;
  double Kbps() const;
  double EncodeMs() const {
    return stream_.EncodeMs();
  }

 private:
  // A client plays and the encoder is open; else false, with what went wrong if anything did.
  bool Begun(std::string& detail);
  std::string Encode(const CapturedFrame& frame);
  std::string Failed(const std::string& detail);

  VideoFeedConfig config_;
  H264Stream& stream_;
  std::function<double()> now_;
  bool active_ = false;
  bool offered_ = false;
  int failures_ = 0;
  double next_prepare_ = 0.0;
};

}  // namespace tron_vision

#endif  // TRON_VISION_VIDEO_FEED_H
