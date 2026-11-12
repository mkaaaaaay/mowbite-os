#ifndef TRON_VISION_H264_STREAM_H
#define TRON_VISION_H264_STREAM_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tron_vision/h264_encoder.h"
#include "tron_vision/h264_rtp.h"
#include "tron_vision/rtsp_server.h"
#include "tron_vision/v4l2_camera.h"

namespace tron_vision {

// H.264 of camera frames for the one client of an RTSP server: the parameter sets ahead for its session description,
// the encoder open only between Begin() and End(), frame and bit rate over the last second.
class H264Stream {
 public:
  enum class Sent {
    kOk,
    kEncoderFailed,
    // The client is gone or reads too slowly; the next one starts a new stream.
    kClientGone,
  };

  // An encode that takes longer than slow_encode_s counts as a failure of the encoder.
  H264Stream(H264Encoder& encoder, RtspServer& server, double slow_encode_s, std::function<double()> now);
  H264Stream(const H264Stream&) = delete;
  H264Stream& operator=(const H264Stream&) = delete;
  ~H264Stream();

  // Opens the encoder once for SPS and PPS and closes it again; Offer() then gives them to the server.
  bool Prepare(const EncoderSettings& settings, std::string& error);
  bool Prepared() const {
    return prepared_;
  }
  // DESCRIBE offers the prepared stream, or answers 503 and drops the client.
  void Offer();
  void Withdraw();
  void Poll(int timeout_ms) {
    server_.Poll(timeout_ms);
  }
  bool Playing() const {
    return server_.Playing();
  }
  void DropClient() {
    server_.DropClient();
  }

  // Opens the encoder with the prepared settings for a new stream.
  bool Begin(std::string& error);
  bool Running() const {
    return running_;
  }
  // One frame to the client; on kEncoderFailed the encoder is closed and the client dropped.
  Sent Send(const CapturedFrame& frame, std::string& error);
  // Closes the encoder; it lets go of the buffers of the camera, so it comes before the camera stops.
  void End();

  double Fps() const {
    return fps_;
  }
  double Kbps() const {
    return kbps_;
  }
  double EncodeMs() const {
    return encode_ms_;
  }
  uint64_t Frames() const {
    return frames_;
  }
  const ParameterSets& Sets() const {
    return sets_;
  }

 private:
  H264Encoder& encoder_;
  RtspServer& server_;
  double slow_encode_s_;
  std::function<double()> now_;
  EncoderSettings settings_;
  ParameterSets sets_;
  bool prepared_ = false;
  bool running_ = false;
  RtpPacketizer packetizer_;
  RtpClock clock_;
  std::vector<std::vector<uint8_t>> packets_;
  double fps_ = 0.0;
  double kbps_ = 0.0;
  double encode_ms_ = 0.0;
  uint64_t frames_ = 0;
  double window_start_ = 0.0;
  int window_frames_ = 0;
  size_t window_bytes_ = 0;
};

}  // namespace tron_vision

#endif  // TRON_VISION_H264_STREAM_H
