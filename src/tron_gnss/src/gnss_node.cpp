#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <memory>
#include <mowbite_component/node_options.h>
#include <mowbite_msgs/msg/gnss_fix.hpp>
#include <mowbite_msgs/msg/rtcm.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <vector>

#include "mowbite_watch/attach.h"
#include "tron_gnss/board_link.h"
#include "tron_gnss/gnss_tracker.h"
#include "tron_gnss/nmea.h"
#include "tron_mcu/serial_port.h"

namespace tron_gnss {

namespace {

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

const char kAgeSensor[] = "os_correction_age";

using mowbite_msgs::msg::GnssFix;
static_assert(static_cast<uint8_t>(tron::Solution::kNone) == GnssFix::NONE);
static_assert(static_cast<uint8_t>(tron::Solution::kSingle) == GnssFix::SINGLE);
static_assert(static_cast<uint8_t>(tron::Solution::kDgps) == GnssFix::DGPS);
static_assert(static_cast<uint8_t>(tron::Solution::kRtkFloat) == GnssFix::RTK_FLOAT);
static_assert(static_cast<uint8_t>(tron::Solution::kRtkFixed) == GnssFix::RTK_FIXED);
static_assert(static_cast<uint8_t>(tron::Solution::kDeadReckoning) == GnssFix::DEAD_RECKONING);
// RTCM waiting for the port; beyond this the newest is dropped (the receiver wants fresh corrections, not all of them).
constexpr size_t kMaxQueue = size_t{32} * 1024;

}  // namespace

// Reads NMEA from the receiver and publishes gps/absolute_pose and gnss/gga. Writes to the port only the RTK board's
// handshake and the RTCM from gnss/rtcm, as the port's only writer.
class GnssNode : public rclcpp::Node {
 public:
  explicit GnssNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("tron_gnss", mowbite_component::StartupOnly(options)), board_(tron::BoardConfig()) {
    port_name_ = declare_parameter("port", "/dev/ttyS4");
    baud_ = static_cast<int>(declare_parameter("baud", 115200));
    frame_id_ = declare_parameter("frame_id", "map");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double lat = declare_parameter("datum_lat", nan);
    const double lon = declare_parameter("datum_long", nan);
    const double height = declare_parameter("datum_height", nan);

    tron::GnssConfig config;
    config.accuracy_rtk_fixed_m = declare_parameter("accuracy_rtk_fixed", config.accuracy_rtk_fixed_m);
    config.accuracy_rtk_float_m = declare_parameter("accuracy_rtk_float", config.accuracy_rtk_float_m);
    config.accuracy_dgps_m = declare_parameter("accuracy_dgps", config.accuracy_dgps_m);
    config.accuracy_single_m = declare_parameter("accuracy_single", config.accuracy_single_m);
    config.motion_from_fixes = declare_parameter("motion_from_fixes", config.motion_from_fixes);
    config.motion_max_age_s = declare_parameter("motion_max_age", config.motion_max_age_s);
    config.motion_min_way_m = declare_parameter("motion_min_way", config.motion_min_way_m);
    config.motion_max_window_s = declare_parameter("motion_max_window", config.motion_max_window_s);
    config.motion_straight_deg = declare_parameter("motion_straight_deg", config.motion_straight_deg);
    config.max_correction_age_s = declare_parameter("max_correction_age", config.max_correction_age_s);
    max_correction_age_s_ = config.max_correction_age_s;
    tron::BoardConfig board;
    board.keepalive_s = declare_parameter("board_keepalive", board.keepalive_s);
    board.active_window_s = declare_parameter("board_active_window", board.active_window_s);
    board.reset_on_off = declare_parameter("board_reset_on_off", board.reset_on_off);
    board.gate = declare_parameter("board_gate", board.gate);
    board_ = tron::BoardLink(board);
    // With <data_dir>/ntrip.yaml (the access of mowbite_ntrip) RTK fixed counts only once NTRIP corrections went to
    // the receiver (G24). Empty: no such rule.
    const std::string data_dir = declare_parameter("data_dir", std::string());
    if (!data_dir.empty()) access_file_ = (std::filesystem::path(data_dir) / "ntrip.yaml").string();

    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(height) || std::abs(lat) > 90.0 ||
        std::abs(lon) > 180.0) {
      RCLCPP_ERROR(get_logger(), "datum_lat, datum_long and datum_height are not set, no pose is published");
    } else {
      tracker_.emplace(config, tron::LocalFrame(lat, lon, height));
    }

    pose_pub_ = create_publisher<GnssFix>("gps/absolute_pose", 10);
    // The app reads the datum from params/json, the bridge collects it from this latched topic.
    if (std::isfinite(lat) && std::isfinite(lon) && std::abs(lat) <= 90.0 && std::abs(lon) <= 180.0) {
      // The datum for the map, which belongs to it: the origin of the poses above, not a measured fix.
      datum_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
          "gps/datum", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
      sensor_msgs::msg::NavSatFix datum;
      datum.header.stamp = now();
      datum.header.frame_id = frame_id_;
      datum.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      datum.latitude = lat;
      datum.longitude = lon;
      datum.altitude = height;
      datum.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      datum_pub_->publish(datum);
      params_pub_ = create_publisher<std_msgs::msg::String>(
          "bridge/params", rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local());
      char text[160];
      std::snprintf(text, sizeof(text), "{\"node\":\"/tron_gnss\",\"params\":{\"datum_lat\":%.15g,\"datum_long\":%.15g}}",
                    lat, lon);
      std_msgs::msg::String msg;
      msg.data = text;
      params_pub_->publish(msg);
    }
    gga_pub_ = create_publisher<std_msgs::msg::String>("gnss/gga", 10);
    rtcm_sub_ = create_subscription<mowbite_msgs::msg::Rtcm>(
        "gnss/rtcm", rclcpp::QoS(50).reliable(),
        [this](const mowbite_msgs::msg::Rtcm::ConstSharedPtr& msg) { OnRtcm(*msg); });
    sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorSpec>(
        "bridge/sensor_info", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("bridge/sensor_value", 100);
    const double none = std::numeric_limits<double>::quiet_NaN();
    mowbite_msgs::msg::SensorSpec spec;
    spec.id = kAgeSensor;
    spec.name = "RTK Correction Age";
    spec.kind = mowbite_msgs::msg::SensorSpec::KIND_NUMBER;
    spec.meaning = mowbite_msgs::msg::SensorSpec::MEANING_UNKNOWN;
    spec.unit = "s";
    spec.min = none;
    spec.max = none;
    spec.critical_low = none;
    spec.critical_high = max_correction_age_s_;
    sensor_info_pub_->publish(spec);
    watch_ = mowbite_watch::Attach(*this);
    io_thread_ = std::thread([this] { IoLoop(); });
  }

  ~GnssNode() override {
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
  }

 private:
  void IoLoop() {
    uint8_t buffer[512];
    double next_open = 0;
    double next_access_check = 0;
    double last_data = MonotonicSeconds();
    while (running_ && rclcpp::ok()) {
      double now = MonotonicSeconds();
      if (now >= next_access_check) {
        next_access_check = now + 1.0;
        CheckAccessFile();
      }
      if (!port_.IsOpen()) {
        if (now >= next_open) {
          next_open = now + 1.0;
          TryOpen();
          last_data = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      ssize_t n = port_.Read(buffer, sizeof(buffer), 50);
      if (n < 0) {
        RCLCPP_ERROR(get_logger(), "Read error on %s, closing", port_name_.c_str());
        ClosePort();
        continue;
      }
      now = MonotonicSeconds();
      if (!WritePending(now)) {
        RCLCPP_ERROR(get_logger(), "Write error on %s, closing", port_name_.c_str());
        ClosePort();
        continue;
      }
      if (n > 0) {
        last_data = now;
        HandleBytes(buffer, static_cast<size_t>(n), now);
      } else if (now - last_data > 5.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "No data from %s for %.0f s", port_name_.c_str(),
                             now - last_data);
      }
    }
  }

  // Whether NTRIP is set up decides which RTK fixed counts; looked at once a second from the port thread, which alone
  // uses the tracker.
  void CheckAccessFile() {
    if (access_file_.empty() || !tracker_) return;
    std::error_code ec;
    const bool required = std::filesystem::exists(access_file_, ec);
    tracker_->SetHostCorrectionsRequired(required);
    if (host_required_ && *host_required_ == required) return;
    host_required_ = required;
    if (required) {
      RCLCPP_INFO(get_logger(), "%s is there: RTK fixed counts once corrections from NTRIP went to the receiver",
                  access_file_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "No %s: RTK fixed counts with corrections from any source", access_file_.c_str());
    }
  }

  void TryOpen() {
    auto users = tron::ProcessesUsing(port_name_);
    if (!users.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "%s is open in another process (pid %d), not opening it",
                            port_name_.c_str(), users.front());
      return;
    }
    std::string error;
    if (!port_.Open(port_name_, baud_, error)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Cannot open %s: %s", port_name_.c_str(), error.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Opened %s at %d baud", port_name_.c_str(), baud_);
    std::lock_guard<std::mutex> lock(board_mutex_);
    board_.OnPortOpened();
    queue_.clear();
    port_open_ = true;
  }

  void ClosePort() {
    port_.Close();
    std::lock_guard<std::mutex> lock(board_mutex_);
    port_open_ = false;
    queue_.clear();
  }

  void OnRtcm(const mowbite_msgs::msg::Rtcm& msg) {
    const double now = MonotonicSeconds();
    std::lock_guard<std::mutex> lock(board_mutex_);
    if (!port_open_) return;
    if (!board_.OnRtcm(msg.data.size(), now)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "RTCM held back until the RTK board reports network RTK ON (%lu bytes so far)",
                           static_cast<unsigned long>(board_.dropped_bytes()));
      return;
    }
    if (queue_.size() + msg.data.size() > kMaxQueue) {
      queue_dropped_ += msg.data.size();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "RTCM queue full, %lu bytes dropped so far",
                           static_cast<unsigned long>(queue_dropped_));
      return;
    }
    queue_.insert(queue_.end(), msg.data.begin(), msg.data.end());
  }

  // Board commands first, then the queued RTCM. False on a write error.
  bool WritePending(double now) {
    std::vector<uint8_t> out;
    {
      std::lock_guard<std::mutex> lock(board_mutex_);
      const std::string commands = board_.Poll(now);
      if (commands.find(tron::kBoardReset) != std::string::npos) {
        RCLCPP_WARN(get_logger(), "RTK board fell back from network RTK: RTKRESET and ON");
      }
      out.assign(commands.begin(), commands.end());
      out.insert(out.end(), queue_.begin(), queue_.end());
      queue_.clear();
    }
    return out.empty() || port_.Write(out);
  }

  void HandleBytes(const uint8_t* data, size_t size, double now) {
    std::vector<tron::NmeaMessage> messages;
    parser_.Feed(data, size, messages);
    for (; reported_kinds_ < parser_.ignored_kinds().size(); ++reported_kinds_) {
      RCLCPP_INFO(get_logger(), "Not decoded: %s", parser_.ignored_kinds()[reported_kinds_].c_str());
    }
    std::optional<double> host_age;
    for (const auto& message : messages) {
      if (message.type == tron::NmeaMessage::Type::kBoardRtk) {
        OnBoardReport(message.board.on, now);
        continue;
      }
      if (message.type == tron::NmeaMessage::Type::kGga) OnGga(message, now);
    }
    {
      std::lock_guard<std::mutex> lock(board_mutex_);
      host_age = board_.host_correction_age(now);
    }
    if (!tracker_) return;
    tracker_->SetHostCorrectionAge(host_age);
    for (const auto& message : messages) {
      auto pose = tracker_->Update(message, now);
      if (pose) Publish(*pose);
    }
    if (tracker_->stale_gga() != reported_stale_) {
      reported_stale_ = tracker_->stale_gga();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "GGA time does not advance, %lu sentences dropped so far; receiver hanging?",
                           static_cast<unsigned long>(reported_stale_));
    }
    if (tracker_->old_corrections() != reported_old_corrections_) {
      reported_old_corrections_ = tracker_->old_corrections();
      if (auto age = tracker_->last_old_correction_age()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "RTK fixed on corrections %.1f s old (limit %.0f s): counted as float", *age,
                             max_correction_age_s_);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "RTK fixed without the age of its corrections (GGA field 13): counted as float");
      }
    }
    if (tracker_->fixed_before_host() != reported_before_host_) {
      reported_before_host_ = tracker_->fixed_before_host();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                           "RTK fixed before any NTRIP corrections went to the receiver (another source such as a LoRa "
                           "base may give it): counted as float");
    }
    if (parser_.checksum_errors() + parser_.malformed() != reported_errors_) {
      reported_errors_ = parser_.checksum_errors() + parser_.malformed();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "NMEA errors so far: %lu checksum, %lu malformed",
                           static_cast<unsigned long>(parser_.checksum_errors()),
                           static_cast<unsigned long>(parser_.malformed()));
    }
  }

  void OnBoardReport(bool on, double now) {
    std::lock_guard<std::mutex> lock(board_mutex_);
    const tron::BoardState before = board_.state();
    board_.OnBoardReport(on, now);
    if (board_.state() != before) {
      RCLCPP_INFO(get_logger(), "RTK board reports network RTK %s", on ? "ON" : "OFF (LoRa)");
    }
  }

  // GGA with a position goes to the NTRIP client (VRS needs it) once a second; the age of the corrections to the sensor.
  void OnGga(const tron::NmeaMessage& message, double now) {
    const tron::GgaData& gga = message.gga;
    if (gga.quality <= 0 || !gga.lat_deg || !gga.lon_deg || now - last_gga_published_ < 1.0) return;
    last_gga_published_ = now;
    std_msgs::msg::String text;
    text.data = message.sentence;
    gga_pub_->publish(text);
    // The larger of the receiver's and the host's age, as the tracker counts it.
    std::optional<double> age = gga.correction_age_s;
    {
      std::lock_guard<std::mutex> lock(board_mutex_);
      if (auto host = board_.host_correction_age(now)) age = age ? std::max(*age, *host) : host;
    }
    if (age) {
      mowbite_msgs::msg::SensorValue value;
      value.sensor_id = kAgeSensor;
      value.value = *age;
      sensor_value_pub_->publish(value);
    }
  }

  void Publish(const tron::GnssPose& pose) {
    GnssFix msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.receiver_time_ms = pose.sensor_stamp_ms;
    msg.solution = static_cast<uint8_t>(pose.solution);
    msg.x = pose.position.east;
    msg.y = pose.position.north;
    msg.z = pose.position.up;
    msg.accuracy = static_cast<float>(pose.accuracy_m);
    msg.velocity_valid = pose.motion_valid;
    msg.velocity_east = pose.motion_east_mps;
    msg.velocity_north = pose.motion_north_mps;
    pose_pub_->publish(msg);
  }

  std::string port_name_;
  int baud_;
  std::string frame_id_;
  tron::SerialPort port_;
  tron::NmeaParser parser_;
  std::optional<tron::GnssTracker> tracker_;
  // Guards board_, queue_ and port_open_ between the RTCM callback and the port thread.
  std::mutex board_mutex_;
  tron::BoardLink board_;
  std::vector<uint8_t> queue_;
  bool port_open_ = false;
  uint64_t queue_dropped_ = 0;
  double last_gga_published_ = -1e9;
  uint64_t reported_errors_ = 0;
  size_t reported_kinds_ = 0;
  uint64_t reported_stale_ = 0;
  uint64_t reported_old_corrections_ = 0;
  uint64_t reported_before_host_ = 0;
  double max_correction_age_s_ = 0;
  std::string access_file_;
  // Whether access_file_ was there at the last look, unknown before the first.
  std::optional<bool> host_required_;
  rclcpp::Publisher<GnssFix>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr datum_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gga_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorSpec>::SharedPtr sensor_info_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::Rtcm>::SharedPtr rtcm_sub_;
  std::atomic<bool> running_{true};
  std::thread io_thread_;
  std::unique_ptr<mowbite_watch::Attached> watch_;
};

}  // namespace tron_gnss

RCLCPP_COMPONENTS_REGISTER_NODE(tron_gnss::GnssNode)
