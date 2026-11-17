#ifndef TRON_VISION_VIDEO_OUT_H
#define TRON_VISION_VIDEO_OUT_H

#include <cstdint>
#include <memory>
#include <mowbite_msgs/msg/camera.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "tron_vision/h264_encoder.h"
#include "tron_vision/h264_stream.h"
#include "tron_vision/rtsp_server.h"
#include "tron_vision/tron_camera.h"
#include "tron_vision/video_feed.h"

namespace tron_vision {

// Suffix of the RTSP path and of the picture topic of a camera with detections whose pictures show no boxes.
constexpr char kPlainSuffix[] = "_plain";

// The H.264 stream of a camera process of the Tron: its parameters, the encoder and the RTSP server on the loopback
// address that MediaMTX reads. The ISP scales the sensor picture to the stream size, or the process decodes the
// camera's JPEG to it; the frame rate is that of the camera.
class VideoOut {
 public:
  // Declares video, stream_width, stream_height, stream_fps, bitrate_kbps, rtsp_port and encoder_library; throws
  // std::invalid_argument on a bad value.
  VideoOut(rclcpp::Node& node, int64_t default_port);
  // For frames of this size in buffers of the process: the same without stream_width and stream_height.
  VideoOut(rclcpp::Node& node, int64_t default_port, uint32_t width, uint32_t height);
  ~VideoOut();
  VideoOut(const VideoOut&) = delete;
  VideoOut& operator=(const VideoOut&) = delete;

  // With video: the camera at the stream size with exported buffers, and the settings of the feed.
  void Configure(TronCameraConfig& camera, VideoFeedConfig& feed) const;
  // With video: the settings of the feed for frames in buffers of the process, whose line bytes and lines are those
  // of whole macroblocks.
  void Configure(VideoFeedConfig& feed) const;
  // Listens under `path`, with `plain` also under path + kPlainSuffix with a stream of its own; false with the error
  // if the server cannot, then there is no stream.
  bool Open(const std::string& path, std::string& error, bool plain = false);
  // Null without video or before Open().
  H264Stream* Stream() const {
    return stream_.get();
  }
  // Null without video, before Open() or without `plain`.
  H264Stream* PlainStream() const {
    return plain_stream_.get();
  }
  // Ends the stream before the loop that feeds it goes.
  void Close();
  // The size of the JPEG pictures of the camera.
  void SetJpegSize(uint32_t width, uint32_t height) {
    jpeg_width_ = width;
    jpeg_height_ = height;
  }
  // Protocols, source and size of the camera entry: WebRTC before MJPEG while the feed is active, else MJPEG at the
  // size of the JPEG pictures. True when one of them changed.
  bool Describe(bool active, mowbite_msgs::msg::Camera& entry) const;

 private:
  // Declares video, stream_fps, bitrate_kbps, rtsp_port and encoder_library.
  void Declare(rclcpp::Node& node, int64_t default_port);

  bool video_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t jpeg_width_ = 0;
  uint32_t jpeg_height_ = 0;
  int fps_ = 0;
  int bitrate_bps_ = 0;
  uint16_t port_ = 0;
  std::string library_;
  std::string source_;
  std::unique_ptr<MppEncoder> encoder_;
  std::unique_ptr<MppEncoder> plain_encoder_;
  std::unique_ptr<RtspServer> server_;
  std::unique_ptr<H264Stream> stream_;
  std::unique_ptr<H264Stream> plain_stream_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_VIDEO_OUT_H
