#include "tron_vision/side_node.h"

#include <chrono>
#include <utility>

#include "mowbite_serial/serial_port.h"
#include "mowbite_watch/watch.h"

namespace tron_vision {

namespace {

// Step interval while the camera does not stream: demand, retries and the heat pause count in seconds.
constexpr auto kIdleWait = std::chrono::milliseconds(200);
// The loop thread and the ROS thread sign within this.
constexpr double kWatchLimit = 2.0;

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

SideNode::SideNode(const rclcpp::NodeOptions& options) : SideNode(options, nullptr) {}

SideNode::SideNode(const rclcpp::NodeOptions& options, std::unique_ptr<Camera> camera)
    : rclcpp::Node("tron_side", options) {
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

  TronCameraConfig hardware;
  hardware.sensor_name = declare_parameter("sensor_name", std::string(kRightSensorName));
  hardware.video_device = declare_parameter("video_device", std::string(kRightVideoDevice));
  hardware.vertical_blanking = static_cast<int>(declare_parameter("vertical_blanking", hardware.vertical_blanking));
  hardware.processes_using = mowbite_serial::ProcessesUsing;

  video_ = std::make_unique<VideoOut>(*this, int64_t{8557});
  video_->Configure(hardware, config.video);
  camera_entry_.id = camera_id_;
  camera_entry_.name = "Right camera";
  // The name the start file gives stack.own_process, so the OS can tie an exit note to this camera.
  camera_entry_.process = declare_parameter("process", std::string("right"));
  camera_entry_.kind = mowbite_msgs::msg::Camera::KIND_OBSTACLE;
  camera_entry_.position = mowbite_msgs::msg::Camera::POSITION_RIGHT;
  uint32_t jpeg_width = 0;
  uint32_t jpeg_height = 0;
  PreviewSize(hardware.width, hardware.height, config.preview.width, jpeg_width, jpeg_height);
  video_->SetJpegSize(jpeg_width, jpeg_height);
  camera_entry_.detections = false;
  camera_entry_.protocols = {mowbite_msgs::msg::Camera::PROTOCOL_MJPEG};
  camera_ = camera ? std::move(camera) : std::make_unique<TronCamera>(std::move(hardware));
  guard_ = std::make_unique<CrashGuard>(crash_dir, kCrashFlag, kBootId);
  std::string error;
  if (!video_->Open(camera_id_, error)) RCLCPP_WARN(get_logger(), "No H.264, only JPEG: %s", error.c_str());
  loop_ = std::make_unique<PreviewLoop>(std::move(config), *camera_, *guard_, MonotonicSeconds, video_->Stream());

  // The only publishers of this process; test_side_node checks the list.
  cameras_pub_ = create_publisher<mowbite_msgs::msg::Camera>(
      "hw/cameras", rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local());
  picture_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("camera/" + camera_id_ + "/jpeg",
                                                                     rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

  thread_ = std::thread([this] { Run(); });
  watch_ = mowbite_watch::Attach(*this, kWatchLimit);
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
  video_->Close();
}

void SideNode::Run() {
  // Signed after each step: a step that hangs in the camera or the encoder ends the process.
  mowbite_watch::Watch watch("tron_side_loop", kWatchLimit);
  for (;;) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) return;
    }
    // The OS subscribes to the pictures only while a viewer watches.
    loop_->SetWanted(picture_pub_->get_subscription_count() > 0);
    const bool go_on = loop_->Step();
    watch.Sign();
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
    if (!Streams(status.state) && video_->Stream() == nullptr) {
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
  const bool described = video_->Describe(status.video, camera_entry_);
  if (!UpdateAvailability(camera_entry_, camera_published_, CameraAvailability(status.state), described)) return;
  camera_entry_.detail = camera_entry_.available ? "" : StateText(status);
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
