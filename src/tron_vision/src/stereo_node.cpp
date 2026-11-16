#include "tron_vision/stereo_node.h"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "tron_mcu/serial_port.h"
#include "tron_vision/encoder_library.h"

namespace tron_vision {

namespace {

const char kStateSensor[] = "stereo_state";
constexpr char kRecordDir[] = "/run/mowbite-os/stereo-record";
const char kFpsSensor[] = "stereo_fps";
constexpr double kReportPeriod = 1.0;
// Silence of the loop thread or of the ROS thread that ends the process.
constexpr double kWatchLimit = 2.0;
constexpr auto kFailedWait = std::chrono::milliseconds(200);

std::string StateText(const StereoStatus& status) {
  std::string text = StateName(status.state);
  if (!status.detail.empty()) text += ": " + status.detail;
  return text;
}

}  // namespace

Availability StereoAvailability(StereoState state) {
  using mowbite_msgs::msg::Camera;
  switch (state) {
    case StereoState::kReady:
    case StereoState::kStreaming: return {true, ""};
    case StereoState::kPausedBlade: return {false, Camera::REASON_BLADE_RUNNING};
    case StereoState::kPausedDriver: return {false, Camera::REASON_DRIVER_MISSING};
    case StereoState::kWaitingIdle:
    case StereoState::kSelftest: return {false, Camera::REASON_SELF_TEST};
    case StereoState::kWaitingSetup:
    case StereoState::kNotConfigured:
    case StereoState::kEncoderMissing: return {false, Camera::REASON_NOT_SET_UP};
    case StereoState::kLocked: return {false, Camera::REASON_LOCKED_AFTER_CRASH};
    case StereoState::kPausedHot: return {false, Camera::REASON_TOO_HOT};
    case StereoState::kOff:
    case StereoState::kNoFrames:
    case StereoState::kStalled:
    case StereoState::kEncoderError:
    case StereoState::kFailed: break;
  }
  return {false, Camera::REASON_NO_SIGNAL};
}

StereoNode::StereoNode(const rclcpp::NodeOptions& options, const StereoNodeSeams& seams)
    : rclcpp::Node("tron_stereo", options) {
  StereoConfig config;
  config.marker_dir = declare_parameter("crash_dir", config.marker_dir);
  config.encoder.bitrate_bps =
      static_cast<int>(declare_parameter("bitrate_kbps", config.encoder.bitrate_bps / 1000)) * 1000;
  config.temp_file = FindThermalZone("/sys/class/thermal", "soc-thermal");
  if (config.temp_file.empty()) RCLCPP_WARN(get_logger(), "No soc-thermal zone: no pause when hot");
  // The platform decides it; the self-test waits for a standing mower with the blade off either way.
  config.stream_while_mowing = declare_parameter("stream_while_mowing", config.stream_while_mowing);
  if (config.stream_while_mowing) RCLCPP_WARN(get_logger(), "stream_while_mowing: the stream runs next to the blade");
  const std::string library = declare_parameter("encoder_library", DefaultEncoderLibrary());
  ReadRecordLimits(*this, kRecordDir, config.record);
  // A wrong value, also one of the wrong type, leaves the wish off and the camera goes on.
  const rclcpp::ParameterValue wish = AnyType(*this, "gain_wish_file", rclcpp::ParameterValue(std::string()));
  if (wish.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    RCLCPP_WARN(get_logger(), "No gain wish: gain_wish_file is not a string");
  } else if (const std::string& file = wish.get<std::string>(); file.empty()) {
    // Off.
  } else if (file.rfind("/run/", 0) != 0 || file.find("..") != std::string::npos) {
    RCLCPP_WARN(get_logger(), "No gain wish: gain_wish_file %s is not below /run", file.c_str());
  } else {
    config.gain_wish_file = file;
    RCLCPP_INFO(get_logger(), "Gain level wished for in %s", file.c_str());
  }

  StereoCameraConfig camera;
  camera.capture_device = declare_parameter("capture_device", camera.capture_device);
  camera.stream_name = declare_parameter("stream_name", camera.stream_name);
  camera.processes_using = tron::ProcessesUsing;
  if (seams.adjust) seams.adjust(config, camera);

  RtspConfig rtsp;
  const int64_t port = declare_parameter("rtsp_port", static_cast<int64_t>(rtsp.port));
  if (port < 1 || port > 65535) throw std::invalid_argument("rtsp_port " + std::to_string(port) + " is no port");
  rtsp.port = static_cast<uint16_t>(port);
  rtsp.path = config.camera_id;

  using mowbite_msgs::msg::Camera;
  camera_entry_.id = config.camera_id;
  camera_entry_.name = "Stereo camera";
  // The name the start file gives stack.own_process, so the OS can tie an exit note to this camera.
  camera_entry_.process = declare_parameter("process", std::string("stereo"));
  camera_entry_.kind = Camera::KIND_STEREO;
  camera_entry_.position = Camera::POSITION_FRONT;
  camera_entry_.width = config.encoder.width;
  camera_entry_.height = config.encoder.height;
  camera_entry_.detections = false;
  camera_entry_.protocols = {Camera::PROTOCOL_WEBRTC};
  camera_entry_.source = "rtsp://127.0.0.1:" + std::to_string(rtsp.port) + "/" + rtsp.path;

  // The only publishers and the only subscription of this process; test_stereo_node checks the lists.
  sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorSpec>(
      "bridge/sensor_info", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("bridge/sensor_value", 100);
  cameras_pub_ = create_publisher<Camera>("hw/cameras", rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local());
  using mowbite_msgs::msg::SensorSpec;
  sensor_info_pub_->publish(PlainSensor(kStateSensor, "Stereo Camera", SensorSpec::KIND_TEXT, ""));
  sensor_info_pub_->publish(PlainSensor(kFpsSensor, "Stereo Camera Rate", SensorSpec::KIND_NUMBER, "1/s"));

  camera_ = std::make_unique<TronStereoCamera>(std::move(camera), seams.device);
  encoder_ = std::make_unique<MppEncoder>(library);
  server_ = std::make_unique<RtspServer>(rtsp, MonotonicSeconds);
  std::string error;
  if (server_->Listen(error)) {
    guard_ = std::make_unique<CrashGuard>(config.marker_dir, seams.crash_flag, config.boot_id_file);
    loop_ = std::make_unique<StereoLoop>(std::move(config), *camera_, *encoder_, *server_, *guard_, MonotonicSeconds);
  } else {
    failed_.state = StereoState::kFailed;
    failed_.detail = "RTSP port " + std::to_string(rtsp.port) + ": " + error;
  }

  // No feedback to the driver: a stalled process cannot slow tron_mcu down, and 10 Hz against 1 s bear losses.
  status_sub_ = create_subscription<mowbite_msgs::msg::HardwareStatus>(
      "hw/status", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const mowbite_msgs::msg::HardwareStatus& msg) { OnStatus(msg); });

  thread_ = std::thread([this] { Run(); });
  watch_ = mowbite_watch::Attach(*this, kWatchLimit);
}

StereoNode::~StereoNode() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
}

void StereoNode::OnStatus(const mowbite_msgs::msg::HardwareStatus& msg) {
  const MowerView mower{true, MonotonicSeconds(), msg.blade_on, msg.moving};
  std::lock_guard<std::mutex> lock(mower_mutex_);
  mower_ = mower;
}

void StereoNode::Run() {
  // Signed after each step: a step that hangs in the camera or the encoder ends the process.
  mowbite_watch::Watch watch("tron_stereo_loop", kWatchLimit);
  while (running_) {
    bool go_on = true;
    if (loop_) {
      MowerView mower;
      {
        std::lock_guard<std::mutex> lock(mower_mutex_);
        mower = mower_;
      }
      loop_->SetMowerStatus(mower);
      go_on = loop_->Step();
    } else {
      std::this_thread::sleep_for(kFailedWait);
    }
    watch.Sign();
    const StereoStatus& status = loop_ ? loop_->Status() : failed_;
    if (status.record_error != logged_record_) {
      logged_record_ = status.record_error;
      RCLCPP_WARN(get_logger(), "%s", logged_record_.c_str());
    }
    if (status.gain_level != logged_gain_) {
      logged_gain_ = status.gain_level;
      RCLCPP_INFO(get_logger(), "Gain level %s wished for, mean luma %.0f", logged_gain_.c_str(), status.luma);
    }
    if (status.gain_error != logged_gain_error_) {
      logged_gain_error_ = status.gain_error;
      if (!logged_gain_error_.empty()) RCLCPP_WARN(get_logger(), "Gain wish: %s", logged_gain_error_.c_str());
    }
    PublishCamera(status);
    Report(status, MonotonicSeconds());
    if (!go_on) {
      exit_code_ = status.exit_code;
      RCLCPP_ERROR(get_logger(), "Ending with code %d: %s", status.exit_code, status.detail.c_str());
      rclcpp::shutdown();
      return;
    }
  }
}

void StereoNode::Report(const StereoStatus& status, double now) {
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
  value.sensor_id = kStateSensor;
  value.text = state;
  sensor_value_pub_->publish(value);
  value.sensor_id = kFpsSensor;
  value.text.clear();
  value.value = status.state == StereoState::kStreaming ? status.fps : 0.0;
  sensor_value_pub_->publish(value);
}

void StereoNode::PublishCamera(const StereoStatus& status) {
  if (!UpdateAvailability(camera_entry_, camera_published_, StereoAvailability(status.state), false)) return;
  camera_entry_.detail = camera_entry_.available ? "" : StateText(status);
  cameras_pub_->publish(camera_entry_);
}

}  // namespace tron_vision
