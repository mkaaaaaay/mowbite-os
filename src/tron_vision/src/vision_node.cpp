#include "tron_vision/vision_node.h"

#include <algorithm>
#include <chrono>
#include <regex>
#include <stdexcept>
#include <utility>

#include "mowbite_serial/serial_port.h"

namespace tron_vision {

namespace {

constexpr double kReportPeriod = 1.0;
// Step interval while no camera streams: retries and the heat pause count in seconds.
constexpr auto kIdleWait = std::chrono::milliseconds(200);

bool Streams(VisionState state) {
  return state == VisionState::kStarting || state == VisionState::kRunning;
}

// Name and position in hw/cameras and the hardware of the side camera with this id.
void SideCamera(const std::string& id, mowbite_msgs::msg::Camera& entry, TronCameraConfig& camera) {
  using mowbite_msgs::msg::Camera;
  if (id == "oa_left") {
    entry.name = "Left camera";
    entry.position = Camera::POSITION_LEFT;
  } else if (id == "oa_right") {
    entry.name = "Right camera";
    entry.position = Camera::POSITION_RIGHT;
    camera.sensor_name = kRightSensorName;
    camera.video_device = kRightVideoDevice;
  } else {
    throw std::invalid_argument("camera_id is '" + id + "', not oa_left or oa_right");
  }
}

// The words after "Camera AI" in the sensor names: "_right" gives " right", "" nothing.
std::string SuffixWords(const std::string& suffix) {
  if (!std::regex_match(suffix, std::regex("(_[a-z0-9]+)*"))) {
    throw std::invalid_argument("sensor_suffix is '" + suffix + "', not empty or _<word>");
  }
  std::string words = suffix;
  std::replace(words.begin(), words.end(), '_', ' ');
  return words;
}

std::string StateText(const VisionStatus& status) {
  std::string text = StateName(status.state);
  if (!status.detail.empty()) text += ": " + status.detail;
  return text;
}

}  // namespace

Availability CameraAvailability(VisionState state) {
  using mowbite_msgs::msg::Camera;
  switch (state) {
    case VisionState::kStarting:
    case VisionState::kRunning: return {true, ""};
    case VisionState::kLocked: return {false, Camera::REASON_LOCKED_AFTER_CRASH};
    case VisionState::kModelMissing:
    case VisionState::kModelMismatch:
    case VisionState::kModelError: return {false, Camera::REASON_NOT_SET_UP};
    case VisionState::kPausedHot: return {false, Camera::REASON_TOO_HOT};
    case VisionState::kOff:
    case VisionState::kNoFrames:
    case VisionState::kCameraError:
    case VisionState::kFailed: break;
  }
  return {false, Camera::REASON_NO_SIGNAL};
}

VisionNode::VisionNode(const rclcpp::NodeOptions& options) : VisionNode(options, nullptr) {}

VisionNode::VisionNode(const rclcpp::NodeOptions& options, std::unique_ptr<Camera> camera)
    : rclcpp::Node("tron_vision", options) {
  VisionConfig config;
  config.camera_id = declare_parameter("camera_id", config.camera_id);
  camera_id_ = config.camera_id;
  config.npu_core = static_cast<int>(declare_parameter("npu_core", int64_t{config.npu_core}));
  const std::string suffix = declare_parameter("sensor_suffix", std::string());
  const std::string words = SuffixWords(suffix);
  state_sensor_ = "vision" + suffix + "_state";
  last_sensor_ = "vision" + suffix + "_last";
  rate_sensor_ = "vision" + suffix + "_rate";
  config.library_path = declare_parameter("library_path", std::string());
  config.model_path = declare_parameter("model_path", std::string());
  config.labels_path = declare_parameter("labels_path", std::string());
  config.infer_rate = declare_parameter("infer_rate", config.infer_rate);
  const std::string fit = declare_parameter("fit", std::string("crop"));
  if (fit != "crop" && fit != "pad") throw std::invalid_argument("fit is '" + fit + "', not crop or pad");
  config.fit = fit == "pad" ? Fit::kPad : Fit::kCrop;
  config.white_balance_inference = declare_parameter("white_balance_inference", config.white_balance_inference);
  config.decode.score_threshold =
      static_cast<float>(declare_parameter("score_threshold", static_cast<double>(config.decode.score_threshold)));
  config.max_rss_bytes = static_cast<size_t>(declare_parameter("max_rss_mb", 300)) << 20;
  config.preview.max_rate = declare_parameter("jpeg_max_rate", config.preview.max_rate);
  const auto max_kb = static_cast<int64_t>(config.preview.max_bytes / 1024);
  config.preview.max_bytes = static_cast<size_t>(declare_parameter("jpeg_max_kb", max_kb)) * 1024;
  config.temp_file = FindThermalZone("/sys/class/thermal", "soc-thermal");
  if (config.temp_file.empty()) RCLCPP_WARN(get_logger(), "No soc-thermal zone: no pause when hot");
  const std::string crash_dir = declare_parameter("crash_dir", std::string("/var/lib/mowbite-vision"));

  TronCameraConfig hardware;
  camera_entry_.id = camera_id_;
  SideCamera(camera_id_, camera_entry_, hardware);
  // The name the start file gives stack.own_process, so the OS can tie an exit note to this camera.
  camera_entry_.process = declare_parameter("process", std::string("vision"));
  camera_entry_.kind = mowbite_msgs::msg::Camera::KIND_OBSTACLE;
  hardware.sensor_name = declare_parameter("sensor_name", hardware.sensor_name);
  hardware.video_device = declare_parameter("video_device", hardware.video_device);
  hardware.vertical_blanking = static_cast<int>(declare_parameter("vertical_blanking", hardware.vertical_blanking));
  hardware.processes_using = mowbite_serial::ProcessesUsing;
  video_ = std::make_unique<VideoOut>(*this, int64_t{8556});
  video_->Configure(hardware, config.video);
  uint32_t jpeg_width = 0;
  uint32_t jpeg_height = 0;
  PreviewSize(hardware.width, hardware.height, config.preview.width, jpeg_width, jpeg_height);
  video_->SetJpegSize(jpeg_width, jpeg_height);
  camera_entry_.detections = true;
  camera_entry_.protocols = {mowbite_msgs::msg::Camera::PROTOCOL_MJPEG};
  camera_ = camera ? std::move(camera) : std::make_unique<TronCamera>(std::move(hardware));
  guard_ = std::make_unique<CrashGuard>(crash_dir, kCrashFlag, kBootId);
  std::string error;
  if (!video_->Open(camera_id_, error, true)) RCLCPP_WARN(get_logger(), "No H.264, only JPEG: %s", error.c_str());
  loop_ = std::make_unique<VisionLoop>(std::move(config), *camera_, *guard_, MonotonicSeconds, video_->Stream(),
                                       video_->PlainStream());

  // The only publishers of this process; test_vision_node checks the list. Absolute names, so that an instance in a
  // namespace of its own publishes on the same topics.
  sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorSpec>(
      "/bridge/sensor_info", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("/bridge/sensor_value", 100);
  detections_pub_ = create_publisher<mowbite_msgs::msg::Detections>("/vision/detections", 5);
  cameras_pub_ = create_publisher<mowbite_msgs::msg::Camera>(
      "/hw/cameras", rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local());
  picture_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/camera/" + camera_id_ + "/jpeg",
                                                                     rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  plain_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/" + camera_id_ + "/jpeg" + kPlainSuffix, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  using mowbite_msgs::msg::SensorSpec;
  const std::string name = "Camera AI" + words;
  sensor_info_pub_->publish(PlainSensor(state_sensor_, name, SensorSpec::KIND_TEXT, ""));
  sensor_info_pub_->publish(PlainSensor(last_sensor_, name + " Detections", SensorSpec::KIND_TEXT, ""));
  sensor_info_pub_->publish(PlainSensor(rate_sensor_, name + " Rate", SensorSpec::KIND_NUMBER, "1/s"));

  thread_ = std::thread([this] { Run(); });
}

VisionNode::~VisionNode() {
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

void VisionNode::Run() {
  loop_->Load();
  for (;;) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) return;
    }
    loop_->SetPreviewWanted(picture_pub_->get_subscription_count() > 0);
    loop_->SetPlainWanted(plain_pub_->get_subscription_count() > 0);
    const bool go_on = loop_->Step();
    const VisionStatus& status = loop_->Status();
    if (status.inferences != published_inferences_) PublishDetections(status);
    const Preview& preview = loop_->GetPreview();
    if (preview.Count() != published_pictures_) {
      published_pictures_ = preview.Count();
      PublishPicture(preview.Jpeg(), picture_pub_);
    }
    if (preview.Plain().Count() != published_plain_) {
      published_plain_ = preview.Plain().Count();
      PublishPicture(preview.Plain().Jpeg(), plain_pub_);
    }
    PublishCamera(status);
    Report(MonotonicSeconds());
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

void VisionNode::Report(double now) {
  const VisionStatus& status = loop_->Status();
  const std::string state = StateText(status);
  if (state != reported_state_) {
    int suppressed = 0;
    if (log_gate_.Allow(StateName(status.state), now, suppressed)) {
      const std::string more = suppressed > 0 ? " (" + std::to_string(suppressed) + " more since)" : "";
      RCLCPP_INFO(get_logger(), "%s%s", state.c_str(), more.c_str());
    }
    reported_state_ = state;
    next_report_ = now;
  }
  if (now < next_report_) return;
  next_report_ = now + kReportPeriod;
  mowbite_msgs::msg::SensorValue value;
  value.sensor_id = state_sensor_;
  value.text = state;
  sensor_value_pub_->publish(value);
  value.sensor_id = last_sensor_;
  value.text = status.last;
  sensor_value_pub_->publish(value);
  value.sensor_id = rate_sensor_;
  value.text.clear();
  value.value = status.state == VisionState::kRunning ? status.rate : 0.0;
  sensor_value_pub_->publish(value);
}

void VisionNode::PublishDetections(const VisionStatus& status) {
  published_inferences_ = status.inferences;
  mowbite_msgs::msg::Detections msg;
  msg.header.stamp = now();
  msg.header.frame_id = camera_id_;
  msg.camera_id = camera_id_;
  msg.width = status.frame_width;
  msg.height = status.frame_height;
  msg.infer_ms = static_cast<float>(status.infer_ms);
  const auto& names = loop_->ClassNames();
  for (const Detection& d : loop_->Detections()) {
    mowbite_msgs::msg::Detection out;
    out.class_name = d.class_id < names.size() ? names[d.class_id] : std::to_string(d.class_id);
    out.score = d.score;
    out.x = d.x1;
    out.y = d.y1;
    out.w = d.x2 - d.x1;
    out.h = d.y2 - d.y1;
    msg.detections.push_back(std::move(out));
  }
  detections_pub_->publish(msg);
}

void VisionNode::PublishCamera(const VisionStatus& status) {
  const bool described = video_->Describe(status.video, camera_entry_);
  if (!UpdateAvailability(camera_entry_, camera_published_, CameraAvailability(status.state), described)) return;
  camera_entry_.detail = camera_entry_.available ? "" : StateText(status);
  cameras_pub_->publish(camera_entry_);
}

void VisionNode::PublishPicture(const std::vector<uint8_t>& jpeg,
                                const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& publisher) {
  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = now();
  msg.header.frame_id = camera_id_;
  msg.format = "jpeg";
  msg.data = jpeg;
  publisher->publish(msg);
}

}  // namespace tron_vision
