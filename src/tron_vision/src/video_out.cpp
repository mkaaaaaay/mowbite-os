#include "tron_vision/video_out.h"

#include <stdexcept>

#include "tron_vision/camera_node.h"
#include "tron_vision/encoder_library.h"

namespace tron_vision {

namespace {

// An encode that takes longer counts as a failure of the encoder.
constexpr double kSlowEncodeS = 0.5;

}  // namespace

VideoOut::VideoOut(rclcpp::Node& node, int64_t default_port) {
  video_ = node.declare_parameter("video", true);
  const int64_t width = node.declare_parameter("stream_width", int64_t{1280});
  const int64_t height = node.declare_parameter("stream_height", int64_t{720});
  const int64_t fps = node.declare_parameter("stream_fps", int64_t{30});
  const int64_t kbps = node.declare_parameter("bitrate_kbps", int64_t{2500});
  const int64_t port = node.declare_parameter("rtsp_port", default_port);
  library_ = node.declare_parameter("encoder_library", DefaultEncoderLibrary());
  if (width < 16 || width > 1920 || height < 16 || height > 1080 || width % 2 != 0 || height % 2 != 0) {
    throw std::invalid_argument("stream size " + std::to_string(width) + "x" + std::to_string(height));
  }
  // 0 takes any free port.
  if (port < 0 || port > 65535) throw std::invalid_argument("rtsp_port " + std::to_string(port) + " is no port");
  width_ = static_cast<uint32_t>(width);
  height_ = static_cast<uint32_t>(height);
  fps_ = static_cast<int>(fps);
  bitrate_bps_ = static_cast<int>(kbps) * 1000;
  port_ = static_cast<uint16_t>(port);
}

VideoOut::~VideoOut() {
  Close();
}

void VideoOut::Configure(TronCameraConfig& camera, VideoFeedConfig& feed) const {
  if (!video_) return;
  camera.width = width_;
  camera.height = height_;
  camera.export_buffers = true;
  feed.encoder.format = PixelFormat::kNv12;
  feed.encoder.width = feed.encoder.line_bytes = width_;
  feed.encoder.height = feed.encoder.lines = height_;
  feed.encoder.fps = fps_;
  feed.encoder.bitrate_bps = bitrate_bps_;
}

bool VideoOut::Open(const std::string& path, std::string& error) {
  if (!video_) return true;
  RtspConfig rtsp;
  rtsp.port = port_;
  rtsp.path = path;
  encoder_ = std::make_unique<MppEncoder>(library_);
  server_ = std::make_unique<RtspServer>(rtsp, MonotonicSeconds);
  if (!server_->Listen(error)) return false;
  stream_ = std::make_unique<H264Stream>(*encoder_, *server_, kSlowEncodeS, MonotonicSeconds);
  source_ = "rtsp://127.0.0.1:" + std::to_string(server_->Port()) + "/" + path;
  return true;
}

void VideoOut::Close() {
  stream_.reset();
  server_.reset();
  encoder_.reset();
}

void VideoOut::Describe(bool active, uint32_t jpeg_width, uint32_t jpeg_height,
                        mowbite_msgs::msg::Camera& entry) const {
  using mowbite_msgs::msg::Camera;
  if (active && !source_.empty()) {
    entry.protocols = {Camera::PROTOCOL_WEBRTC, Camera::PROTOCOL_MJPEG};
    entry.source = source_;
    entry.width = width_;
    entry.height = height_;
  } else {
    entry.protocols = {Camera::PROTOCOL_MJPEG};
    entry.source.clear();
    entry.width = jpeg_width;
    entry.height = jpeg_height;
  }
}

}  // namespace tron_vision
