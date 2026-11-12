#include "tron_vision/side_node.h"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "tron_mcu/serial_port.h"
#include "tron_vision/encoder_library.h"

namespace tron_vision {

namespace {

// Step interval while the camera does not stream: demand, retries and the heat pause count in seconds.
constexpr auto kIdleWait = std::chrono::milliseconds(200);
// An encode that takes longer counts as a failure of the encoder.
constexpr double kSlowEncodeS = 0.5;

bool Streams(PreviewState state) {
  return state == PreviewState::kStarting || state == PreviewState::kRunning;
}

std::string StateText(const PreviewStatus& status) {
  std::string text = StateName(status.state);
  if (!status.detail.empty()) text += ": " + status.detail;
  return text;
}

}  // namespace

Availability CameraAvailability(PreviewState state) {
  using mowbite_msgs::msg::Camera;
  switch (state) {
    case PreviewState::kOff:
    case PreviewState::kStarting:
    case PreviewState::kRunning: return {true, ""};
    case PreviewState::kLocked: return {false, Camera::REASON_LOCKED_AFTER_CRASH};
    case PreviewState::kPausedHot: return {false, Camera::REASON_TOO_HOT};
    case PreviewState::kNoFrames:
    case PreviewState::kCameraError:
    case PreviewState::kFailed: break;
  }
  return {false, Camera::REASON_NO_SIGNAL};
}

SideNode::SideNode(const rclcpp::NodeOptions& options) : rclcpp::Node("tron_side", options) {
  PreviewLoopConfig config;
  camera_id_ = config.camera_id;
  config.max_rss_bytes =
      static_cast<size_t>(declare_parameter("max_rss_mb", static_cast<int64_t>(config.max_rss_bytes >> 20))) << 20;
  config.preview.max_rate = declare_parameter("jpeg_max_rate", config.preview.max_rate);
  const auto max_kb = static_cast<int64_t>(config.preview.max_bytes / 1024);
  config.preview.max_bytes = static_cast<size_t>(declare_parameter("jpeg_max_kb", max_kb)) * 1024;
  config.temp_file = FindThermalZone("/sys/class/thermal", "soc-thermal");
  if (config.temp_file.empty()) RCLCPP_WARN(get_logger(), "No soc-thermal zone: no pause when hot");
  const std::string crash_dir = declare_parameter("crash_dir", std::string("/var/lib/mowbite-vision"));

  TronCameraConfig camera;
  camera.sensor_name = declare_parameter("sensor_name", std::string(kRightSensorName));
  camera.video_device = declare_parameter("video_device", std::string(kRightVideoDevice));
  camera.vertical_blanking = static_cast<int>(declare_parameter("vertical_blanking", camera.vertical_blanking));
  camera.processes_using = tron::ProcessesUsing;

  // H.264: the ISP scales the sensor picture to the stream size; the frame rate is that of the sensor.
  const bool video = declare_parameter("video", true);
  const int64_t width = declare_parameter("stream_width", int64_t{1280});
  const int64_t height = declare_parameter("stream_height", int64_t{720});
  const int64_t fps = declare_parameter("stream_fps", int64_t{30});
  const int64_t kbps = declare_parameter("bitrate_kbps", int64_t{2500});
  const int64_t port = declare_parameter("rtsp_port", int64_t{8557});
  const std::string library = declare_parameter("encoder_library", DefaultEncoderLibrary());
  if (width < 16 || width > 1920 || height < 16 || height > 1080 || width % 2 != 0 || height % 2 != 0) {
    throw std::invalid_argument("stream size " + std::to_string(width) + "x" + std::to_string(height));
  }
  // 0 takes any free port.
  if (port < 0 || port > 65535) throw std::invalid_argument("rtsp_port " + std::to_string(port) + " is no port");
  if (video) {
    camera.width = static_cast<uint32_t>(width);
    camera.height = static_cast<uint32_t>(height);
    camera.export_buffers = true;
    config.encoder.format = PixelFormat::kNv12;
    config.encoder.width = config.encoder.line_bytes = camera.width;
    config.encoder.height = config.encoder.lines = camera.height;
    config.encoder.fps = static_cast<int>(fps);
    config.encoder.bitrate_bps = static_cast<int>(kbps) * 1000;
  }
  camera_entry_.id = camera_id_;
  camera_entry_.name = "Right camera";
  // The name the start file gives stack.own_process, so the OS can tie an exit note to this camera.
  camera_entry_.process = declare_parameter("process", std::string("right"));
  camera_entry_.kind = mowbite_msgs::msg::Camera::KIND_OBSTACLE;
  camera_entry_.position = mowbite_msgs::msg::Camera::POSITION_RIGHT;
  PreviewSize(camera.width, camera.height, config.preview.width, jpeg_width_, jpeg_height_);
  video_width_ = camera.width;
  video_height_ = camera.height;
  camera_entry_.detections = false;
  camera_entry_.protocols = {mowbite_msgs::msg::Camera::PROTOCOL_MJPEG};
  camera_ = std::make_unique<TronCamera>(std::move(camera));
  guard_ = std::make_unique<CrashGuard>(crash_dir, kCrashFlag, kBootId);
  if (video) {
    RtspConfig rtsp;
    rtsp.port = static_cast<uint16_t>(port);
    rtsp.path = camera_id_;
    encoder_ = std::make_unique<MppEncoder>(library);
    server_ = std::make_unique<RtspServer>(rtsp, MonotonicSeconds);
    std::string error;
    if (server_->Listen(error)) {
      stream_ = std::make_unique<H264Stream>(*encoder_, *server_, kSlowEncodeS, MonotonicSeconds);
      video_source_ = "rtsp://127.0.0.1:" + std::to_string(server_->Port()) + "/" + rtsp.path;
    } else {
      RCLCPP_WARN(get_logger(), "No H.264, only JPEG: %s", error.c_str());
    }
  }
  loop_ = std::make_unique<PreviewLoop>(std::move(config), *camera_, *guard_, MonotonicSeconds, stream_.get());

  // The only publishers of this process; test_side_node checks the list.
  cameras_pub_ = create_publisher<mowbite_msgs::msg::Camera>(
      "hw/cameras", rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local());
  picture_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("camera/" + camera_id_ + "/jpeg",
                                                                     rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

  thread_ = std::thread([this] { Run(); });
}

SideNode::~SideNode() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  wake_.notify_all();
  if (thread_.joinable()) thread_.join();
  loop_->Shutdown();
  loop_.reset();
  stream_.reset();
}

void SideNode::Run() {
  for (;;) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) return;
    }
    // The OS subscribes to the pictures only while a viewer watches.
    loop_->SetWanted(picture_pub_->get_subscription_count() > 0);
    const bool go_on = loop_->Step();
    const PreviewStatus& status = loop_->Status();
    if (loop_->GetPreview().Count() != published_pictures_) PublishPicture();
    PublishCamera(status);
    Log(MonotonicSeconds());
    if (!go_on) {
      exit_code_ = status.exit_code;
      RCLCPP_ERROR(get_logger(), "Ending with code %d: %s", status.exit_code, status.detail.c_str());
      rclcpp::shutdown();
      return;
    }
    // With a stream the loop waits for its client itself.
    if (!Streams(status.state) && stream_ == nullptr) {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_for(lock, kIdleWait, [this] { return !running_; });
    }
  }
}

void SideNode::Log(double now) {
  const PreviewStatus& status = loop_->Status();
  const std::string state = StateText(status);
  if (state == logged_state_) return;
  logged_state_ = state;
  int suppressed = 0;
  if (!log_gate_.Allow(StateName(status.state), now, suppressed)) return;
  const std::string more = suppressed > 0 ? " (" + std::to_string(suppressed) + " more since)" : "";
  RCLCPP_INFO(get_logger(), "%s%s", state.c_str(), more.c_str());
}

void SideNode::PublishCamera(const PreviewStatus& status) {
  using mowbite_msgs::msg::Camera;
  const Availability now = CameraAvailability(status.state);
  const bool video = status.video && !video_source_.empty();
  if (camera_published_ && now.available == camera_entry_.available && now.reason == camera_entry_.reason &&
      video == !camera_entry_.source.empty()) {
    return;
  }
  camera_published_ = true;
  if (video) {
    camera_entry_.protocols = {Camera::PROTOCOL_WEBRTC, Camera::PROTOCOL_MJPEG};
    camera_entry_.source = video_source_;
    camera_entry_.width = video_width_;
    camera_entry_.height = video_height_;
  } else {
    camera_entry_.protocols = {Camera::PROTOCOL_MJPEG};
    camera_entry_.source.clear();
    camera_entry_.width = jpeg_width_;
    camera_entry_.height = jpeg_height_;
  }
  camera_entry_.available = now.available;
  camera_entry_.reason = now.reason;
  camera_entry_.detail = now.available ? "" : StateText(status);
  cameras_pub_->publish(camera_entry_);
}

void SideNode::PublishPicture() {
  const Preview& preview = loop_->GetPreview();
  published_pictures_ = preview.Count();
  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = now();
  msg.header.frame_id = camera_id_;
  msg.format = "jpeg";
  msg.data = preview.Jpeg();
  picture_pub_->publish(msg);
}

}  // namespace tron_vision
