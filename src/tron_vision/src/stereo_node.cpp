#include "tron_vision/stereo_node.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <utility>

#include <mowbite_msgs/blade.hpp>

#include "mowbite_serial/serial_port.h"
#include "tron_vision/encoder_library.h"

namespace tron_vision {

namespace {

const char kStateSensor[] = "stereo_state";
constexpr char kRecordDir[] = "/run/mowbite-os/stereo-record";
const char kFpsSensor[] = "stereo_fps";
const char kObstaclesSensor[] = "stereo_obstacles";
// Loading the model takes at most this long.
constexpr double kLoadLimit = 20.0;
constexpr double kReportPeriod = 1.0;
// Silence of the loop thread or of the ROS thread that ends the process.
constexpr double kWatchLimit = 2.0;
constexpr auto kFailedWait = std::chrono::milliseconds(200);
constexpr double kListenRetryS = 5.0;

std::string StateText(const StereoStatus& status) {
  std::string text = StateName(status.state);
  if (!status.detail.empty()) text += ": " + status.detail;
  return text;
}

std::string Joined(const std::string& a, const std::string& b) {
  return a.empty() ? b : a + ", " + b;
}

// The parameters of the obstacles; what is missing goes to not_set_up.
void ReadObstacleConfig(rclcpp::Node& node, ObstacleConfig& config) {
  config.library_path = node.declare_parameter("library_path", config.library_path);
  config.model_path = node.declare_parameter("model_path", config.model_path);
  config.labels_path = node.declare_parameter("labels_path", config.labels_path);
  config.npu_core = static_cast<int>(node.declare_parameter("npu_core", static_cast<int64_t>(config.npu_core)));
  config.swap_rb = node.declare_parameter("swap_rb", config.swap_rb);
  config.decode.score_threshold = static_cast<float>(
      node.declare_parameter("score_threshold", static_cast<double>(config.decode.score_threshold)));
  config.rate = node.declare_parameter("obstacle_rate", config.rate);
  config.range_max = node.declare_parameter("range_max", config.range_max);
  config.person = node.declare_parameter("kind_person", std::vector<std::string>{});
  config.animal = node.declare_parameter("kind_animal", std::vector<std::string>{});
  config.ignore = node.declare_parameter("kind_ignore", std::vector<std::string>{});
  for (const std::string& entry : node.declare_parameter("known_heights", std::vector<std::string>{})) {
    const size_t eq = entry.find('=');
    char* end = nullptr;
    const double height = eq == std::string::npos ? 0.0 : std::strtod(entry.c_str() + eq + 1, &end);
    if (eq == std::string::npos || eq == 0 || end == nullptr || *end != '\0' || !(height > 0.0 && height < 3.0)) {
      config.not_set_up = Joined(config.not_set_up, "known_heights needs class=metres, not " + entry);
    } else {
      config.known_height[entry.substr(0, eq)] = height;
    }
  }
  config.calibration_path = node.declare_parameter("calibration_path", config.calibration_path);
  config.wheel_radius = node.declare_parameter("wheel_radius", 0.0);
  if (!(config.wheel_radius > 0.0 && config.wheel_radius < 0.5)) {
    config.not_set_up = Joined(config.not_set_up, "no wheel_radius");
  }
  // x, y, z (m) and yaw, pitch, roll (rad) of the left eye in base_link.
  const std::vector<double> mount = node.declare_parameter("mount", std::vector<double>{});
  bool finite = true;
  for (const double v : mount) finite = finite && std::isfinite(v);
  if (mount.size() == 6 && finite) {
    config.mount = {mount[0], mount[1], mount[2], mount[5], mount[4], mount[3]};
    config.have_mount = true;
  } else {
    config.not_set_up = Joined(config.not_set_up, mount.empty() ? "no camera mount" : "mount needs 6 numbers");
  }
}

}  // namespace

std::string ObstacleReasonName(SightReason reason) {
  using mowbite_msgs::msg::Obstacles;
  switch (reason) {
    case SightReason::kNone: return "";
    case SightReason::kNotSetUp: return Obstacles::REASON_NOT_SET_UP;
    case SightReason::kTooDark: return Obstacles::REASON_TOO_DARK;
    case SightReason::kTooBright: return Obstacles::REASON_TOO_BRIGHT;
    case SightReason::kCovered: return Obstacles::REASON_COVERED;
    case SightReason::kNoSignal: return Obstacles::REASON_NO_SIGNAL;
    case SightReason::kTooHot: return Obstacles::REASON_TOO_HOT;
    case SightReason::kSelfTest: return Obstacles::REASON_SELF_TEST;
    case SightReason::kLockedAfterCrash: return Obstacles::REASON_LOCKED_AFTER_CRASH;
  }
  return Obstacles::REASON_NO_SIGNAL;
}

mowbite_msgs::msg::Obstacles ObstaclesMessage(const ObstacleReport& report, const std::string& source,
                                               const builtin_interfaces::msg::Time& stamp) {
  using mowbite_msgs::msg::Obstacle;
  mowbite_msgs::msg::Obstacles msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "base_link";
  msg.source = source;
  msg.sight = report.reason == SightReason::kNone;
  msg.reason = ObstacleReasonName(report.reason);
  msg.detail = report.detail;
  msg.view_x = static_cast<float>(report.view.x);
  msg.view_y = static_cast<float>(report.view.y);
  msg.view_angle_left = static_cast<float>(report.view.angle_left);
  msg.view_angle_right = static_cast<float>(report.view.angle_right);
  msg.view_range_min = static_cast<float>(report.view.range_min);
  msg.view_range_max = static_cast<float>(report.view.range_max);
  if (!msg.sight) return msg;
  for (const FoundObstacle& found : report.obstacles) {
    Obstacle o;
    switch (found.kind) {
      case ObstacleKind::kPerson: o.kind = Obstacle::KIND_PERSON; break;
      case ObstacleKind::kAnimal: o.kind = Obstacle::KIND_ANIMAL; break;
      case ObstacleKind::kObject:
      case ObstacleKind::kIgnore: o.kind = Obstacle::KIND_OBJECT; break;
    }
    o.class_name = found.class_name;
    o.score = static_cast<float>(found.score);
    o.x = static_cast<float>(found.spot.x);
    o.y = static_cast<float>(found.spot.y);
    o.radius = static_cast<float>(found.radius);
    o.height = static_cast<float>(found.height);
    o.cov_xx = static_cast<float>(found.spot.cov_xx);
    o.cov_xy = static_cast<float>(found.spot.cov_xy);
    o.cov_yy = static_cast<float>(found.spot.cov_yy);
    o.cut_bottom = found.cut_bottom;
    switch (found.from) {
      case PositionFrom::kGround: o.position_from = Obstacle::FROM_GROUND; break;
      case PositionFrom::kStereo: o.position_from = Obstacle::FROM_STEREO; break;
      case PositionFrom::kBoth: o.position_from = Obstacle::FROM_BOTH; break;
    }
    o.agrees = found.agrees;
    o.box_x = static_cast<float>(found.box.x1);
    o.box_y = static_cast<float>(found.box.y1);
    o.box_w = static_cast<float>(found.box.x2 - found.box.x1);
    o.box_h = static_cast<float>(found.box.y2 - found.box.y1);
    msg.obstacles.push_back(o);
  }
  return msg;
}

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
  config.camera_id = declare_parameter("camera_id", config.camera_id);
  camera_id_ = config.camera_id;
  config.obstacles = declare_parameter("obstacles", config.obstacles);
  config.marker_dir = declare_parameter("crash_dir", config.marker_dir);
  config.max_rss_bytes =
      static_cast<size_t>(declare_parameter("max_rss_mb", static_cast<int64_t>(config.max_rss_bytes >> 20))) << 20;
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
  } else if (const std::string& file = wish.get<std::string>(); !file.empty() && !BelowRun(file)) {
    RCLCPP_WARN(get_logger(), "No gain wish: gain_wish_file %s is not below /run", file.c_str());
  } else if (!file.empty()) {
    config.gain_wish_file = file;
    RCLCPP_INFO(get_logger(), "Gain level wished for in %s", file.c_str());
  }

  StereoCameraConfig camera;
  camera.capture_device = declare_parameter("capture_device", camera.capture_device);
  camera.stream_name = declare_parameter("stream_name", camera.stream_name);
  camera.processes_using = mowbite_serial::ProcessesUsing;
  if (seams.adjust) seams.adjust(config, camera);
  if (config.obstacles) {
    ObstacleConfig obstacles;
    ReadObstacleConfig(*this, obstacles);
    obstacles.eye_width = config.encoder.width;
    obstacles.eye_height = config.encoder.height;
    if (seams.adjust_obstacles) seams.adjust_obstacles(obstacles);
    if (!config.stream_while_mowing) {
      RCLCPP_WARN(get_logger(), "Obstacles without stream_while_mowing: no sight while the blade runs");
    }
    obstacles_ = std::make_unique<ObstacleCore>(std::move(obstacles), MonotonicSeconds);
  }

  RtspConfig rtsp;
  const int64_t port = declare_parameter("rtsp_port", static_cast<int64_t>(rtsp.port));
  if (port < 1 || port > 65535) throw std::invalid_argument("rtsp_port " + std::to_string(port) + " is no port");
  rtsp.port = static_cast<uint16_t>(port);
  rtsp.paths = {config.camera_id};

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
  camera_entry_.source = "rtsp://127.0.0.1:" + std::to_string(rtsp.port) + "/" + config.camera_id;

  // The only publishers and the only subscription of this process; test_stereo_node checks the lists.
  sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorSpec>(
      "bridge/sensor_info", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("bridge/sensor_value", 100);
  cameras_pub_ = create_publisher<Camera>("hw/cameras", rclcpp::QoS(rclcpp::KeepLast(4)).reliable().transient_local());
  using mowbite_msgs::msg::SensorSpec;
  sensor_info_pub_->publish(PlainSensor(kStateSensor, "Stereo Camera", SensorSpec::KIND_TEXT, ""));
  sensor_info_pub_->publish(PlainSensor(kFpsSensor, "Stereo Camera Rate", SensorSpec::KIND_NUMBER, "1/s"));
  if (obstacles_) {
    sensor_info_pub_->publish(PlainSensor(kObstaclesSensor, "Stereo Camera Obstacles", SensorSpec::KIND_TEXT, ""));
    obstacles_pub_ = create_publisher<mowbite_msgs::msg::Obstacles>("hw/obstacles", 10);
    images_pub_ = create_publisher<mowbite_msgs::msg::ObstacleImage>("hw/obstacle_images", 2);
    // Latched by the OS; without a message no photo leaves the process.
    photos_sub_ = create_subscription<std_msgs::msg::Bool>(
        "obstacles/photos_wanted", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](const std_msgs::msg::Bool& msg) { obstacles_->SetPhotosWanted(msg.data); });
  }

  camera_ = std::make_unique<TronStereoCamera>(std::move(camera), seams.device);
  encoder_ = std::make_unique<MppEncoder>(library);
  server_ = std::make_unique<RtspServer>(rtsp, MonotonicSeconds);
  guard_ = std::make_unique<CrashGuard>(config.marker_dir, seams.crash_flag, config.boot_id_file);
  waiting_config_ = std::move(config);
  rtsp_port_ = rtsp.port;
  TryListen();
  // Answered by the loop thread once the loop runs.
  unlock_srv_ = OfferUnlock(*this, camera_entry_.id, unlock_requests_, nullptr);

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
  const MowerView mower{true, MonotonicSeconds(), mowbite_msgs::BladeMayTurn(msg), msg.moving};
  std::lock_guard<std::mutex> lock(mower_mutex_);
  mower_ = mower;
}

// The loop starts once the RTSP port is had; until then the camera reports itself failed and the next try follows.
void StereoNode::TryListen() {
  const double now = MonotonicSeconds();
  if (now < next_listen_) return;
  next_listen_ = now + kListenRetryS;
  std::string error;
  if (!server_->Listen(error)) {
    failed_.state = StereoState::kFailed;
    failed_.detail = "RTSP port " + std::to_string(rtsp_port_) + ": " + error;
    return;
  }
  loop_ = std::make_unique<StereoLoop>(std::move(*waiting_config_), *camera_, *encoder_, *server_, *guard_,
                                       MonotonicSeconds, obstacles_.get());
  waiting_config_.reset();
}

void StereoNode::Run() {
  if (obstacles_) {
    {
      mowbite_watch::Watch load("tron_stereo_load", kLoadLimit);
      obstacles_->Load();
    }
  }
  // Signed after each step: a step that hangs in the camera, the encoder or the model ends the process.
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
      unlock_requests_.Serve([this](std::string& text) { return loop_->Unlock(text); });
      go_on = loop_->Step();
    } else {
      std::this_thread::sleep_for(kFailedWait);
      TryListen();
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
    const double now = MonotonicSeconds();
    if (obstacles_) {
      ObstacleReport report;
      if (obstacles_->Poll(now, status, report)) PublishObstacles(report, now);
      if (obstacles_->Status().exit_code != kExitNone && go_on) {
        exit_code_ = obstacles_->Status().exit_code;
        RCLCPP_ERROR(get_logger(), "Ending with code %d: %s", exit_code_.load(), obstacles_->Status().last.c_str());
        rclcpp::shutdown();
        return;
      }
    }
    Report(status, now);
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
  if (!obstacles_) return;
  value.sensor_id = kObstaclesSensor;
  value.value = 0.0;
  value.text = reported_obstacles_;
  sensor_value_pub_->publish(value);
}

void StereoNode::PublishObstacles(const ObstacleReport& report, double now) {
  // The frame was taken this long before now on the monotonic clock of the driver.
  const rclcpp::Time stamp = this->now() - rclcpp::Duration::from_seconds(std::max(0.0, now - report.taken));
  obstacles_pub_->publish(ObstaclesMessage(report, camera_id_, stamp));
  const ObstacleStatus& core = obstacles_->Status();
  if (report.reason == SightReason::kNone) {
    char rate[48];
    std::snprintf(rate, sizeof(rate), " (%.1f/s, %.0f ms)", core.rate, core.infer_ms);
    reported_obstacles_ = core.last + rate;
  } else {
    reported_obstacles_ = ObstacleReasonName(report.reason) + ": " + report.detail;
  }
  if (report.jpeg.empty()) return;
  mowbite_msgs::msg::ObstacleImage image;
  image.header.stamp = stamp;
  image.header.frame_id = "base_link";
  image.source = camera_id_;
  image.width = report.width;
  image.height = report.height;
  image.jpeg = report.jpeg;
  images_pub_->publish(image);
}

void StereoNode::PublishCamera(const StereoStatus& status) {
  if (!UpdateAvailability(camera_entry_, camera_published_, StereoAvailability(status.state), false)) return;
  camera_entry_.detail = camera_entry_.available ? "" : StateText(status);
  cameras_pub_->publish(camera_entry_);
}

}  // namespace tron_vision
