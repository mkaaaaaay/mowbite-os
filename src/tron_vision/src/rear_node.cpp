#include "tron_vision/rear_node.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mowbite_msgs/msg/camera.hpp>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tron_mcu/serial_port.h"

namespace tron_vision {

namespace {

using mowbite_msgs::msg::DockMarker;

// Silence of the loop thread or of the ROS thread that ends the process.
constexpr double kWatchLimit = 2.0;
constexpr int kMaxPattern = 0xFFFF;

std::string Fixed(double value, const char* format = "%.2f") {
  char buf[32];
  std::snprintf(buf, sizeof(buf), format, value);
  return buf;
}

std::string Joined(const std::string& a, const std::string& b) {
  return a.empty() ? b : a + ", " + b;
}

diagnostic_msgs::msg::KeyValue Value(const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

const char* ResultName(MarkerResult result) {
  switch (result) {
    case MarkerResult::kFound: return "found";
    case MarkerResult::kNoMarker: return "no_marker";
    case MarkerResult::kClipped: return "clipped";
    case MarkerResult::kAmbiguous: return "ambiguous";
    case MarkerResult::kTooDark: return "too_dark";
    case MarkerResult::kTooBright: return "too_bright";
    case MarkerResult::kBlurred: return "blurred";
  }
  return "unknown";
}

// Calibration and marker from the parameters; what is missing goes to config.not_set_up. None of them has a default.
void ReadCalibration(rclcpp::Node& node, RearConfig& config) {
  const double none = std::numeric_limits<double>::quiet_NaN();
  // fx, fy, cx, cy in pixels of the full image, then k1, k2, p1, p2, k3 of plumb_bob.
  const std::vector<double> lens = node.declare_parameter("lens", std::vector<double>{});
  if (lens.size() == 9 && lens[0] > 0.0 && lens[1] > 0.0) {
    config.lens = {lens[0], lens[1], lens[2], lens[3], lens[4], lens[5], lens[6], lens[7], lens[8]};
  } else {
    config.not_set_up = Joined(config.not_set_up, lens.empty() ? "no lens" : "lens needs 9 numbers, fx and fy > 0");
  }
  // x, y, z in m and yaw, pitch, roll in rad, as CameraMount.
  const std::vector<double> mount = node.declare_parameter("mount", std::vector<double>{});
  if (mount.size() == 6) {
    config.mount = {mount[0], mount[1], mount[2], mount[3], mount[4], mount[5]};
  } else {
    config.not_set_up = Joined(config.not_set_up, mount.empty() ? "no camera mount" : "mount needs 6 numbers");
  }
  config.marker_side = node.declare_parameter("marker_side", none);
  if (!(config.marker_side > 0.0)) config.not_set_up = Joined(config.not_set_up, "no marker side");
  const int64_t pattern = node.declare_parameter("marker_pattern", static_cast<int64_t>(-1));
  if (pattern >= 0 && pattern <= kMaxPattern) {
    config.marker.pattern = static_cast<uint16_t>(pattern);
  } else {
    config.not_set_up = Joined(config.not_set_up, "no marker pattern");
  }
}

}  // namespace

std::string DockReasonText(DockReason reason) {
  using mowbite_msgs::msg::Camera;
  switch (reason) {
    case DockReason::kNone: return "";
    case DockReason::kNoMarker: return DockMarker::REASON_NO_MARKER;
    case DockReason::kMarkerClipped: return DockMarker::REASON_MARKER_CLIPPED;
    case DockReason::kMarkerAmbiguous: return DockMarker::REASON_MARKER_AMBIGUOUS;
    case DockReason::kTooDark: return DockMarker::REASON_TOO_DARK;
    case DockReason::kTooBright: return DockMarker::REASON_TOO_BRIGHT;
    case DockReason::kBlurred: return DockMarker::REASON_BLURRED;
    case DockReason::kFitRejected: return DockMarker::REASON_FIT_REJECTED;
    case DockReason::kNoSignal: return Camera::REASON_NO_SIGNAL;
    case DockReason::kNotSetUp: return Camera::REASON_NOT_SET_UP;
    case DockReason::kLockedAfterCrash: return Camera::REASON_LOCKED_AFTER_CRASH;
    case DockReason::kTooHot: return Camera::REASON_TOO_HOT;
  }
  return Camera::REASON_NO_SIGNAL;
}

RearNode::RearNode(const rclcpp::NodeOptions& options, const RearNodeSeams& seams)
    : rclcpp::Node("tron_rear", options) {
  RearConfig config;
  ReadCalibration(*this, config);
  config.request_hold_s = DockMarker::REQUEST_HOLD_S;
  config.report_period_s = DockMarker::REPORT_PERIOD_S;
  config.eval_rate = declare_parameter("eval_rate", config.eval_rate);
  if (!(config.eval_rate > 0.0 && config.eval_rate <= 30.0)) {
    throw std::invalid_argument("eval_rate " + std::to_string(config.eval_rate) + " is not within 0..30");
  }
  config.max_exposure_s = declare_parameter("max_exposure_ms", config.max_exposure_s * 1000.0) / 1000.0;
  config.first_frame_s = declare_parameter("first_frame_s", config.first_frame_s);
  config.frame_gap_s = declare_parameter("frame_gap_s", config.frame_gap_s);
  // A longer gap would leave hw/dock_marker silent for longer than its period while measuring.
  if (!(config.frame_gap_s > 0.0 && config.frame_gap_s < config.report_period_s)) {
    throw std::invalid_argument("frame_gap_s " + std::to_string(config.frame_gap_s) + " is not within 0..1 s");
  }
  config.marker_dir = declare_parameter("crash_dir", config.marker_dir);
  config.max_rss_bytes =
      static_cast<size_t>(declare_parameter("max_rss_mb", static_cast<int64_t>(config.max_rss_bytes >> 20))) << 20;
  config.temp_file = FindThermalZone("/sys/class/thermal", "soc-thermal");
  if (config.temp_file.empty()) RCLCPP_WARN(get_logger(), "No soc-thermal zone: no pause when hot");

  RearCameraConfig camera;
  camera.device = declare_parameter("device", camera.device);
  camera.processes_using = tron::ProcessesUsing;
  if (seams.adjust) seams.adjust(config, camera);
  camera.width = config.width;
  camera.height = config.height;
  if (!config.not_set_up.empty()) RCLCPP_WARN(get_logger(), "Not set up: %s", config.not_set_up.c_str());

  // The only publishers and the only subscription of this process; test_rear_node checks the lists.
  marker_pub_ =
      create_publisher<DockMarker>("hw/dock_marker", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  pixels_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "tron_rear/marker_pixels", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
  request_sub_ = create_subscription<std_msgs::msg::Empty>(
      "hw/dock_marker_request", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      [this](const std_msgs::msg::Empty&) { OnRequest(); });

  camera_ = std::make_unique<UvcRearCamera>(std::move(camera), seams.device);
  guard_ = std::make_unique<CrashGuard>(config.marker_dir, seams.crash_flag, kBootId);
  loop_ = std::make_unique<RearLoop>(
      std::move(config), *camera_, *guard_, MonotonicSeconds, [this] { return LastRequest(); },
      [this](double seconds) { WaitForRequest(seconds); }, [this](const DockReport& r) { Publish(r); });
  thread_ = std::thread([this] { Run(); });
  watch_ = mowbite_watch::Attach(*this, kWatchLimit);
}

RearNode::~RearNode() {
  running_ = false;
  request_cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void RearNode::OnRequest() {
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    last_request_ = MonotonicSeconds();
    requests_++;
  }
  request_cv_.notify_all();
}

double RearNode::LastRequest() {
  std::lock_guard<std::mutex> lock(request_mutex_);
  return last_request_;
}

void RearNode::WaitForRequest(double seconds) {
  std::unique_lock<std::mutex> lock(request_mutex_);
  const uint64_t seen = requests_;
  request_cv_.wait_for(lock, std::chrono::duration<double>(seconds),
                       [this, seen] { return requests_ != seen || !running_; });
}

void RearNode::Run() {
  // Signed after each step: a step that hangs in the camera ends the process.
  mowbite_watch::Watch watch("tron_rear_loop", kWatchLimit);
  while (running_) {
    const bool go_on = loop_->Step();
    watch.Sign();
    if (!go_on) {
      exit_code_ = loop_->ExitCode();
      RCLCPP_ERROR(get_logger(), "Ending with code %d", loop_->ExitCode());
      rclcpp::shutdown();
      return;
    }
  }
}

void RearNode::Publish(const DockReport& report) {
  // The stamp on the ROS clock: the age on the monotonic clock taken from now.
  const double age = MonotonicSeconds() - report.stamp;
  const rclcpp::Time stamp = get_clock()->now() - rclcpp::Duration::from_seconds(age);
  DockMarker msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "base_link";
  switch (report.state) {
    case DockState::kOff: msg.state = DockMarker::STATE_OFF; break;
    case DockState::kStarting: msg.state = DockMarker::STATE_STARTING; break;
    case DockState::kMeasuring: msg.state = DockMarker::STATE_MEASURING; break;
    case DockState::kFault: msg.state = DockMarker::STATE_FAULT; break;
  }
  msg.valid = report.pose.valid && report.reason == DockReason::kNone;
  if (msg.valid) {
    msg.x = report.pose.x;
    msg.y = report.pose.y;
    msg.yaw_valid = report.pose.yaw_valid;
    msg.yaw = report.pose.yaw_valid ? report.pose.yaw : 0.0;
    for (size_t i = 0; i < msg.covariance.size(); i++) msg.covariance[i] = report.pose.covariance[i];
  }
  msg.reason = DockReasonText(report.reason);
  msg.detail = report.detail;
  marker_pub_->publish(msg);
  if (report.image) PublishPixels(report, msg.header.stamp);
  Log(report, report.stamp);
}

void RearNode::PublishPixels(const DockReport& report, const builtin_interfaces::msg::Time& stamp) {
  const MarkerDetection& d = report.detection;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "tron_rear";
  status.message = ResultName(d.result);
  std::string corners;
  for (const Point2& c : d.corners_px) corners += (corners.empty() ? "" : " ") + Fixed(c.x) + " " + Fixed(c.y);
  status.values = {Value("side_px", Fixed(d.side_px)),
                   Value("corners_px", corners),
                   Value("corner_sigma_px", Fixed(d.corner_sigma_px, "%.3f")),
                   Value("fit_error_px", Fixed(report.pose.fit_error_px, "%.3f")),
                   Value("edge_px", Fixed(d.edge_px)),
                   Value("white", Fixed(d.white, "%.1f")),
                   Value("black", Fixed(d.black, "%.1f")),
                   Value("saturated", Fixed(d.saturated, "%.3f")),
                   Value("image_mean", Fixed(d.image_mean, "%.1f")),
                   Value("image_saturated", Fixed(d.image_saturated, "%.4f")),
                   Value("manual_exposure", report.manual_exposure ? "true" : "false"),
                   Value("exposure", std::to_string(report.exposure.exposure)),
                   Value("gain", std::to_string(report.exposure.gain)),
                   Value("decode_ms", Fixed(report.decode_ms, "%.1f")),
                   Value("detect_ms", Fixed(report.detect_ms, "%.1f"))};
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  array.header.frame_id = "rear_camera";
  array.status.push_back(std::move(status));
  pixels_pub_->publish(array);
}

void RearNode::Log(const DockReport& report, double now) {
  // State and reason only: a measurement every image would flood the log.
  std::string line = DockStateName(report.state);
  const std::string reason = DockReasonText(report.reason);
  if (!reason.empty()) line += " " + reason;
  if (line == logged_) return;
  logged_ = line;
  int suppressed = 0;
  if (!log_gate_.Allow(line, now, suppressed)) return;
  const std::string more = suppressed > 0 ? " (" + std::to_string(suppressed) + " more since)" : "";
  const std::string detail = report.detail.empty() ? "" : ": " + report.detail;
  RCLCPP_INFO(get_logger(), "%s%s%s", line.c_str(), detail.c_str(), more.c_str());
}

}  // namespace tron_vision
