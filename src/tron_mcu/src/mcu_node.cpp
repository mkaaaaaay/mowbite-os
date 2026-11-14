#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <mowbite_component/node_options.h>
#include <mowbite_msgs/msg/button_event.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/srv/set_blade.hpp>
#include <mowbite_msgs/srv/set_charge.hpp>
#include <mowbite_msgs/srv/set_emergency.hpp>
#include <mowbite_hold/hold.h>
#include <mowbite_journal/node_options.h>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>

#include "tron_mcu/mcu_codec.h"
#include "tron_mcu/mcu_control.h"
#include "tron_mcu/mcu_journal.h"
#include "tron_mcu/serial_port.h"

namespace tron_mcu {

namespace {

using mowbite_msgs::msg::HardwareStatus;
using tron::Module;
using C = tron::McuControl;

// The core numbers its reasons like the contract.
static_assert(tron::kReasonLatch == HardwareStatus::REASON_LATCH);
static_assert(tron::kReasonTimeoutInputs == HardwareStatus::REASON_TIMEOUT_INPUTS);
static_assert(tron::kReasonStop == HardwareStatus::REASON_STOP);
static_assert(tron::kReasonLift == HardwareStatus::REASON_LIFT);
static_assert(tron::kReasonCollision == HardwareStatus::REASON_COLLISION);
static_assert(tron::kReasonHighLevel == HardwareStatus::REASON_HIGH_LEVEL);
static_assert(tron::kReasonFirmwareIncompatible == HardwareStatus::REASON_FIRMWARE_INCOMPATIBLE);
static_assert(tron::kReasonTilt == HardwareStatus::REASON_TILT);
static_assert(tron::kReasonFence == HardwareStatus::REASON_FENCE);
static_assert(tron::kReasonStuck == HardwareStatus::REASON_STUCK);
static_assert(mowbite_hold::kReasonUnexpectedRestart == HardwareStatus::REASON_UNEXPECTED_RESTART);
static_assert(tron::kReasonImuTimeout == HardwareStatus::REASON_IMU_TIMEOUT);
static_assert(tron::kReasonMowerNotStopped == HardwareStatus::REASON_MOWER_NOT_STOPPED);
static_assert(static_cast<uint8_t>(tron::StopPress::kHeld) == mowbite_msgs::msg::ButtonEvent::PRESS_HELD);
static_assert(static_cast<uint8_t>(tron::StopPress::kLong) == mowbite_msgs::msg::ButtonEvent::PRESS_LONG);
static_assert(static_cast<uint8_t>(tron::StopPress::kCanceled) == mowbite_msgs::msg::ButtonEvent::PRESS_CANCELED);
static_assert(static_cast<uint8_t>(tron::BladeOffReason::kLeaseExpired) == HardwareStatus::BLADE_OFF_LEASE_EXPIRED);
static_assert(static_cast<uint8_t>(tron::BladeOffReason::kReconnect) == HardwareStatus::BLADE_OFF_RECONNECT);

const char* BladeOffText(tron::BladeOffReason reason) {
  switch (reason) {
    case tron::BladeOffReason::kNone: return "none";
    case tron::BladeOffReason::kRequest: return "request";
    case tron::BladeOffReason::kLeaseExpired: return "lease expired";
    case tron::BladeOffReason::kBlocked: return "blocked";
    case tron::BladeOffReason::kDock: return "dock contact";
    case tron::BladeOffReason::kReconnect: return "port reopened";
  }
  return "unknown";
}

rclcpp::QoS LatchedQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kStandardGravity = 9.80665;
// Battery and motor reports not refreshed within this time are treated as absent.
constexpr double kReportMaxAge = 3.0;
// hold.json is looked at this often.
constexpr double kHoldPollS = 0.5;

double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// The part of hw/status that goes out at once when it changes: emergency, latch, reason bits and the hold.
uint64_t BlockOf(const tron::McuControlStatus& state) {
  return state.emergency_reason | (uint64_t{state.blocked} << 32) | (uint64_t{state.latched} << 33) |
         (uint64_t{state.held} << 34);
}

int64_t WallSeconds() {
  using namespace std::chrono;
  return duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
}

tron::WallClock LocalWallClock() {
  using namespace std::chrono;
  auto now = system_clock::now();
  std::time_t seconds = system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&seconds, &local);
  tron::WallClock wall;
  wall.year = static_cast<uint16_t>(local.tm_year + 1900);
  wall.month = static_cast<uint8_t>(local.tm_mon + 1);
  wall.day = static_cast<uint8_t>(local.tm_mday);
  wall.hour = static_cast<uint8_t>(local.tm_hour);
  wall.minute = static_cast<uint8_t>(local.tm_min);
  wall.second = static_cast<uint8_t>(local.tm_sec);
  wall.millisecond = static_cast<uint16_t>(duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000);
  return wall;
}

}  // namespace

// Driver of the Tron behind the hardware contract hw/* of MowBite OS.
class McuNode : public rclcpp::Node {
 public:
  explicit McuNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("tron_mcu", mowbite_component::StartupOnly(options)),
        journal_(mowbite_journal::OpenFromParameters(*this)),
        control_(journal_, LoadConfig()),
        journal_watch_(journal_) {
    // The bytes on the port as they were read and written, beside the calls of the core.
    if (journal_) {
      rx_stream_ = journal_->Define({{"name", "mcu.rx"}, {"kind", "raw"}});
      tx_stream_ = journal_->Define({{"name", "mcu.tx"}, {"kind", "raw"}});
    }
    port_name_ = declare_parameter("port", "/dev/ttyS9");
    read_only_ = get_parameter("read_only").as_bool();
    if (read_only_) {
      RCLCPP_WARN(get_logger(), "read_only: %s is only read, no byte goes to the MCU: no motion, blade or charging",
                  port_name_.c_str());
    }
    blade_lease_max_s_ = get_parameter("blade_lease_max").as_double();
    mow_height_mm_ = static_cast<uint16_t>(std::clamp<int64_t>(get_parameter("mow_height_mm").as_int(),
                                                               tron::kMinCutterHeightMm, tron::kMaxCutterHeightMm));
    stop_release_ = get_parameter("stop_release").as_bool() && !read_only_;
    imu_timeout_s_ = get_parameter("imu_timeout").as_double();
    measured_linear_scale_ = declare_parameter("measured_linear_scale", 1.0);
    measured_angular_scale_ = declare_parameter("measured_angular_scale", 1.0);
    drive_current_scale_ = declare_parameter("drive_current_scale", 0.001);
    imu_roll_offset_deg_ = declare_parameter("imu_roll_offset_deg", 0.0);
    imu_pitch_offset_deg_ = declare_parameter("imu_pitch_offset_deg", 0.0);
    failsafe_without_host_s_ = declare_parameter("failsafe_without_host", -1.0);
    safety_measurement_date_ = read_only_ ? "" : declare_parameter("safety_measurement_date", "");
    if (safety_measurement_date_.empty()) {
      RCLCPP_WARN(get_logger(), "No safety measurement of this Tron yet: remote control only");
    }
    // <data_dir>/hold.json of the logic; empty: no hold.
    hold_watch_.emplace(declare_parameter("hold_file", ""));
    if (hold_watch_->path().empty()) RCLCPP_WARN(get_logger(), "No hold_file: an update cannot hold this driver");
    PollHold();

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("hw/wheel_odom", 10);
    status_pub_ = create_publisher<HardwareStatus>("hw/status", 10);
    capabilities_pub_ = create_publisher<mowbite_msgs::msg::HardwareCapabilities>("hw/capabilities", LatchedQos());
    // Never latched: a late subscriber must not act on an old press.
    button_pub_ = create_publisher<mowbite_msgs::msg::ButtonEvent>("hw/button", rclcpp::QoS(10).reliable());
    PublishCapabilities();

    if (!read_only_) {
      cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          "hw/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) { OnCmdVel(*msg); });
    }
    if (get_parameter("send_imu").as_bool() && !read_only_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          "imu/data", rclcpp::SensorDataQoS(),
          [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) { OnImu(*msg); });
    }
    emergency_srv_ = create_service<mowbite_msgs::srv::SetEmergency>(
        "hw/emergency", [this](const mowbite_msgs::srv::SetEmergency::Request::SharedPtr& req,
                               const mowbite_msgs::srv::SetEmergency::Response::SharedPtr& res) {
          OnEmergency(*req, *res);
        });
    blade_srv_ = create_service<mowbite_msgs::srv::SetBlade>(
        "hw/blade", [this](const mowbite_msgs::srv::SetBlade::Request::SharedPtr& req,
                           const mowbite_msgs::srv::SetBlade::Response::SharedPtr& res) { OnBlade(*req, *res); });
    charge_srv_ = create_service<mowbite_msgs::srv::SetCharge>(
        "hw/charge", [this](const mowbite_msgs::srv::SetCharge::Request::SharedPtr& req,
                            const mowbite_msgs::srv::SetCharge::Response::SharedPtr& res) { OnCharge(*req, *res); });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { PublishStatus(); });
    journal_timer_ = create_wall_timer(std::chrono::seconds(10), [this] { journal_watch_.Check(get_logger()); });
    LogTimeZone();

    io_thread_ = std::thread([this] { IoLoop(); });
  }

  ~McuNode() override {
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
  }

 private:
  tron::McuControlConfig LoadConfig() {
    tron::McuControlConfig config;
    // Only listens on the port: the core runs, its frames are dropped.
    const bool read_only = declare_parameter("read_only", false);
    config.cmd_vel_timeout_s = declare_parameter("cmd_vel_timeout", config.cmd_vel_timeout_s);
    config.speed_repeat_window_s = declare_parameter("speed_repeat_window", config.speed_repeat_window_s);
    config.telemetry_timeout_s = declare_parameter("telemetry_timeout", config.telemetry_timeout_s);
    config.allow_blade = declare_parameter("allow_blade", false) && !read_only;
    config.blade_speed_percent = static_cast<uint16_t>(declare_parameter("blade_speed_percent", 100));
    config.mow_height_mm = static_cast<uint16_t>(std::clamp<int64_t>(declare_parameter("mow_height_mm", 60),
                                                                      tron::kMinCutterHeightMm,
                                                                      tron::kMaxCutterHeightMm));
    config.blade_lease_max_s = declare_parameter("blade_lease_max", config.blade_lease_max_s);
    config.bumper_latches_emergency = declare_parameter("bumper_latches_emergency", true);
    const bool send_imu = declare_parameter("send_imu", true) && !read_only;
    const double imu_timeout = declare_parameter("imu_timeout", 0.5);
    const double imu_start_timeout = declare_parameter("imu_start_timeout", 2.0);
    // Without forwarding the MCU gets no IMU from here, so there is nothing to wait for.
    config.imu_timeout_s = send_imu ? imu_timeout : 0.0;
    config.imu_start_timeout_s = send_imu ? imu_start_timeout : 0.0;
    config.blade_stop_time_s = declare_parameter("blade_stop_time", config.blade_stop_time_s);
    config.blade_still_rpm = static_cast<int>(declare_parameter("blade_still_rpm", config.blade_still_rpm));
    config.expected_cutter_version = declare_parameter("expected_cutter_version", "");
    config.expected_chassis_version = declare_parameter("expected_chassis_version", "");
    config.stop_release = declare_parameter("stop_release", false) && !read_only;
    if (config.stop_release) {
      RCLCPP_INFO(get_logger(), "stop_release: STOP free %.1f s, then held %.1f to %.1f s, reports a release on hw/button",
                  tron::kStopReleaseFreeS, tron::kStopReleaseHoldS, tron::kStopReleaseMaxHoldS);
    }
    RCLCPP_INFO(get_logger(), "allow_blade: %d, cmd_vel_timeout: %.2f s, speed_repeat_window: %.2f s, mow_height: %u "
                "mm", config.allow_blade, config.cmd_vel_timeout_s, config.speed_repeat_window_s, config.mow_height_mm);
    if (config.expected_cutter_version.empty() && config.expected_chassis_version.empty()) {
      RCLCPP_WARN(get_logger(), "Firmware check is off: expected_cutter_version and expected_chassis_version are empty");
    } else {
      RCLCPP_INFO(get_logger(), "Expecting MCU firmware cutter '%s', chassis '%s'; blocked until the MCU reports it",
                  config.expected_cutter_version.c_str(), config.expected_chassis_version.c_str());
    }
    return config;
  }

  // The heartbeat carries local time; without TZ that is UTC in a container.
  void LogTimeZone() {
    const char* tz = std::getenv("TZ");  // NOLINT(concurrency-mt-unsafe): read once at start, nothing sets it
    std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char text[64];
    std::strftime(text, sizeof(text), "%H:%M %Z (UTC%z)", &local);
    if (tz == nullptr || *tz == '\0') {
      RCLCPP_WARN(get_logger(), "TZ is not set, the heartbeat sends %s", text);
    } else {
      RCLCPP_INFO(get_logger(), "Heartbeat in local time, TZ=%s: %s", tz, text);
    }
  }

  // Owns the port: reads telemetry, runs the control tick every 10 ms and writes all frames.
  void IoLoop() {
    uint8_t buffer[512];
    double next_tick = 0;
    double next_open = 0;
    double next_hold = 0;
    while (running_ && rclcpp::ok()) {
      double now = MonotonicSeconds();
      if (now >= next_hold) {
        next_hold = now + kHoldPollS;
        PollHold();
      }
      if (!port_.IsOpen() && now >= next_open) {
        next_open = now + 1.0;
        TryOpen();
      }
      if (port_.IsOpen()) {
        int wait_ms = std::clamp(static_cast<int>((next_tick - now) * 1000.0), 0, 5);
        ssize_t n = port_.Read(buffer, sizeof(buffer), wait_ms);
        if (n < 0) {
          RCLCPP_ERROR(get_logger(), "Read error on %s, closing", port_name_.c_str());
          port_.Close();
        } else if (n > 0) {
          HandleBytes(buffer, static_cast<size_t>(n));
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }

      now = MonotonicSeconds();
      // Nothing is computed while the port is closed, so nothing is lost either.
      if (port_.IsOpen() && now >= next_tick) {
        next_tick = now + 0.01;
        std::vector<tron::SubPacket> packets;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          packets = control_.Call<&C::Tick>(now, LocalWallClock());
        }
        if (!read_only_) Send(packets);
        PublishButton();
      }
      // So that the logic learns of a block before a blade request it refuses, not only at the next status tick.
      if (BlockChanged()) PublishStatus();
    }
    SendShutdownFrames();
  }

  // The hold of an update, from hold.json itself: it stands without the logic and right after a restart.
  void PollHold() {
    if (!hold_watch_) return;
    const mowbite_hold::HoldView view = hold_watch_->Poll(MonotonicSeconds(), WallSeconds());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      control_.Call<&C::SetHold>(view.held);
    }
    if (view.held == hold_logged_.held && view.unreadable == hold_logged_.unreadable &&
        view.until == hold_logged_.until) {
      return;
    }
    if (view.unreadable) {
      RCLCPP_ERROR(get_logger(), "Held: %s; motion and blade stay blocked until it is readable or gone",
                   view.error.c_str());
    } else if (view.held) {
      RCLCPP_INFO(get_logger(), "Held for an update until %lld (unix s): motion and blade blocked, charging goes on",
                  static_cast<long long>(view.until));
    } else if (hold_logged_.held) {
      RCLCPP_INFO(get_logger(), "Hold ended: motion and blade allowed again");
    }
    hold_logged_ = view;
  }

  bool BlockChanged() {
    std::lock_guard<std::mutex> lock(mutex_);
    return BlockOf(control_.View().Status(MonotonicSeconds())) != published_block_;
  }

  void TryOpen() {
    auto users = tron::ProcessesUsing(port_name_);
    if (!users.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "%s is open in another process (pid %d), not opening it",
                            port_name_.c_str(), users.front());
      return;
    }
    std::string error;
    if (!port_.Open(port_name_, 115200, error, read_only_)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "Cannot open %s: %s", port_name_.c_str(), error.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      control_.Call<&C::OnConnected>(MonotonicSeconds());
    }
    RCLCPP_INFO(get_logger(), "Opened %s", port_name_.c_str());
  }

  void Send(const std::vector<tron::SubPacket>& packets) {
    for (const auto& packet : packets) {
      const std::vector<uint8_t> frame = tron::EncodeFrame({packet});
      if (journal_) journal_->Append(tx_stream_, mowbite_journal::kRawBytes, frame.data(), frame.size());
      if (!port_.Write(frame)) {
        RCLCPP_ERROR(get_logger(), "Write error on %s, closing", port_name_.c_str());
        port_.Close();
        return;
      }
    }
  }

  void SendShutdownFrames() {
    if (!port_.IsOpen()) return;
    if (read_only_) {
      port_.Close();
      return;
    }
    // Stop wheels if they were commanded, switch the blade off and lift the deck.
    std::vector<tron::SubPacket> packets;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (control_.View().Status(MonotonicSeconds()).moving) packets.push_back(tron::ZeroSpeedPacket());
    }
    packets.push_back(tron::CutterPacket(tron::CutterCommand{}));
    Send(packets);
    port_.Close();
  }

  void HandleBytes(const uint8_t* data, size_t length) {
    if (journal_) journal_->Append(rx_stream_, mowbite_journal::kRawBytes, data, length);
    std::vector<tron::MeasuredSpeed> speeds;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      double now = MonotonicSeconds();
      for (const auto& frame : parser_.Push(data, length)) {
        for (const auto& packet : frame) {
          if (packet.type != static_cast<uint8_t>(tron::PacketType::kMcuToHost)) continue;
          if (packet.module == static_cast<uint8_t>(Module::kSpeed)) {
            if (auto speed = tron::DecodeSpeed(packet.payload)) speeds.push_back(*speed);
          } else {
            HandleTelemetry(packet, now);
          }
        }
      }
    }
    for (const auto& speed : speeds) PublishTwist(speed);
  }

  // Called with mutex_ held.
  void HandleTelemetry(const tron::SubPacket& packet, double now) {
    switch (static_cast<Module>(packet.module)) {
      case Module::kSensor:
        if (auto info = tron::DecodeSensorInfo(packet.payload)) {
          control_.Call<&C::OnSensorInfo>(*info, now);
          if (info->cutter_size != sensor_.cutter_size) {
            RCLCPP_INFO(get_logger(), "Mowing deck: %u (1 small, 2 large)", info->cutter_size);
          }
          sensor_ = *info;
        }
        break;
      case Module::kBattery:
        if (auto battery = tron::DecodeBattery(packet.payload)) {
          control_.Call<&C::OnDockContactSample>(battery->dock_ok == 1);
          battery_ = *battery;
          battery_s_ = now;
        }
        break;
      case Module::kMotors:
        if (auto motors = tron::DecodeMotors(packet.payload)) {
          motors_ = *motors;
          motors_s_ = now;
          control_.Call<&C::OnBladeRpm>(motors->cutter.rpm, now);
        }
        break;
      case Module::kVersion:
        if (auto versions = tron::DecodeVersionInfo(packet.payload)) CheckVersions(*versions);
        break;
      case Module::kBmsVersion:
        if (auto bms = tron::DecodeBmsVersion(packet.payload)) {
          std::string version = VersionString(*bms);
          if (version != bms_version_) {
            bms_version_ = version;
            PublishCapabilities();
          }
        }
        break;
      case Module::kLog: RCLCPP_INFO(get_logger(), "MCU log: %s", tron::DecodeLog(packet.payload).c_str()); break;
      case Module::kUptime:
      case Module::kCalibration: break;
      default:
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 10000, "Unhandled MCU module %d with %zu bytes",
                              packet.module, packet.payload.size());
    }
  }

  // Called with mutex_ held.
  void CheckVersions(const tron::VersionInfo& versions) {
    std::string cutter = tron::VersionString(versions.cutter_board);
    std::string chassis = tron::VersionString(versions.chassis_board);
    if (cutter != cutter_version_ || chassis != chassis_version_) {
      RCLCPP_INFO(get_logger(), "MCU versions: cutter %s, chassis %s", cutter.c_str(), chassis.c_str());
      cutter_version_ = cutter;
      chassis_version_ = chassis;
      PublishCapabilities();
    }
    if (!control_.Call<&C::OnVersions>(versions)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                            "Unexpected MCU firmware: cutter %s, chassis %s; motion and blade stay blocked",
                            cutter.c_str(), chassis.c_str());
    }
  }

  void PublishTwist(const tron::MeasuredSpeed& speed) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = speed.linear * measured_linear_scale_;
    msg.twist.angular.z = speed.angular * measured_angular_scale_;
    twist_pub_->publish(msg);
  }

  // The STOP gesture the last tick reported, once.
  void PublishButton() {
    tron::StopEvent event;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      event = control_.View().Status(MonotonicSeconds()).stop_event;
    }
    if (event.seq == published_stop_seq_) return;
    published_stop_seq_ = event.seq;
    mowbite_msgs::msg::ButtonEvent msg;
    msg.stamp = now();
    msg.button = mowbite_msgs::msg::ButtonEvent::BUTTON_STOP;
    msg.press = static_cast<uint8_t>(event.press);
    msg.latch_count = event.latch_count;
    switch (event.press) {
      case tron::StopPress::kHeld: RCLCPP_INFO(get_logger(), "STOP held: let go to release"); break;
      case tron::StopPress::kLong: RCLCPP_INFO(get_logger(), "STOP let go: release (latch_count %u)", msg.latch_count); break;
      case tron::StopPress::kCanceled: RCLCPP_INFO(get_logger(), "STOP press canceled, no release"); break;
      case tron::StopPress::kNone: return;
    }
    button_pub_->publish(msg);
  }

  // Called with mutex_ held, or before the I/O thread starts.
  void PublishCapabilities() {
    mowbite_msgs::msg::HardwareCapabilities msg;
    msg.platform = "airseekers_tron";
    auto add_firmware = [&msg](const char* board, const std::string& version) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = board;
      entry.value = version;
      msg.firmware.push_back(entry);
    };
    add_firmware("cutter", cutter_version_);
    add_firmware("chassis", chassis_version_);
    add_firmware("bms", bms_version_);
    // The MCU holds about 3000 rpm whatever percent the cutter frame names (30 and 100 measured).
    msg.blade_speed_control = false;
    msg.blade_height_control = true;
    msg.blade_height_min_mm = tron::kMinCutterHeightMm;
    msg.blade_height_max_mm = tron::kMaxCutterHeightMm;
    msg.blade_height_default_mm = mow_height_mm_;
    msg.blade_lease_max_s = static_cast<float>(blade_lease_max_s_);
    // The driver sends Charge itself after a debounced dock contact.
    msg.host_charge_control = false;
    msg.charge_stop = !read_only_;
    msg.rain_sensor = true;
    msg.separate_bumpers = true;
    msg.wheel_ticks = false;
    msg.wheel_ticks_per_m = tron::kMcuTicksPerMeter;
    msg.wheel_distance = tron::kMcuWheelDistance;
    msg.imu_source = "tron_imu";
    msg.gnss_source = "tron_gnss";
    msg.gnss_receiver_time = true;
    // 0 lets nothing move.
    msg.max_linear_speed = read_only_ ? 0.0f : tron::kMaxLinearSpeed;
    msg.max_angular_speed = read_only_ ? 0.0f : tron::kMaxAngularSpeed;
    if (stop_release_) {
      msg.stop_release_free_s = static_cast<float>(tron::kStopReleaseFreeS);
      msg.stop_release_hold_s = static_cast<float>(tron::kStopReleaseHoldS);
      msg.stop_release_max_hold_s = static_cast<float>(tron::kStopReleaseMaxHoldS);
    }
    msg.failsafe_without_host_s = static_cast<float>(failsafe_without_host_s_);
    msg.safety_measurement_date = safety_measurement_date_;
    capabilities_pub_->publish(msg);
  }

  mowbite_msgs::msg::MotorState MotorMessage(const tron::MotorInfo& motor, bool fresh, double current_scale) const {
    mowbite_msgs::msg::MotorState msg;
    msg.ok = fresh;
    if (!fresh) return msg;
    msg.rpm = motor.rpm;
    msg.current = static_cast<float>(motor.current_raw * current_scale);
    msg.temperature = motor.temperature_c;
    msg.status = motor.status;
    return msg;
  }

  // Called by the timer and by the I/O thread, one at a time, so that the messages go out in the order they were made.
  void PublishStatus() {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    HardwareStatus msg;
    msg.stamp = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      double now_s = MonotonicSeconds();
      const auto state = control_.View().Status(now_s);
      published_block_ = BlockOf(state);
      msg.emergency = state.blocked;
      msg.emergency_latched = state.latched;
      msg.emergency_reason = state.emergency_reason;
      const uint32_t added = state.emergency_reason & ~logged_reason_;
      logged_reason_ = state.emergency_reason;
      // Before its first sample the IMU only blocks, that is no gap.
      if ((added & tron::kReasonImuTimeout) && imu_seen_) {
        RCLCPP_ERROR(get_logger(), "No IMU sample for %.1f s: motion and blade blocked until released", imu_timeout_s_);
      }
      if (added & tron::kReasonMowerNotStopped) {
        RCLCPP_ERROR(get_logger(), "Blade still at %d rpm after it was switched off: blocked, off sent again",
                     motors_.cutter.rpm);
      }
      msg.telemetry_ok = !state.telemetry_stale;
      msg.hold = state.held;
      msg.lift = state.lift;
      msg.stop = state.stop;
      msg.bumper = state.bumper;
      msg.bumper_left = msg.telemetry_ok && sensor_.bumper_left;
      msg.bumper_right = msg.telemetry_ok && sensor_.bumper_right;
      msg.rain = msg.telemetry_ok && sensor_.rain;
      msg.dock_contact = state.dock_contact;
      msg.moving = state.moving;
      msg.blade_on = state.blade_on;
      msg.blade_off_reason = static_cast<uint8_t>(state.blade_off_reason);
      if (state.blade_on != blade_on_logged_) {
        blade_on_logged_ = state.blade_on;
        if (state.blade_on) {
          RCLCPP_INFO(get_logger(), "Blade on");
        } else if (state.blade_off_reason == tron::BladeOffReason::kLeaseExpired) {
          RCLCPP_WARN(get_logger(), "Blade off: lease expired without renewal");
        } else {
          RCLCPP_INFO(get_logger(), "Blade off: %s", BladeOffText(state.blade_off_reason));
        }
      }

      if (battery_s_ && now_s - *battery_s_ < kReportMaxAge) {
        // The battery report carries the contact.
        msg.dock_contact_raw = state.dock_contact_raw;
        msg.battery_source = HardwareStatus::BATTERY_SOURCE_BMS;
        msg.battery_voltage = static_cast<float>(battery_.voltage_v);
        msg.battery_current = static_cast<float>(battery_.current_a);
        msg.battery_percentage = battery_.percentage;
        msg.battery_temperature = battery_.temperature_c;
        msg.battery_error_bits = battery_.error_bits;
      }

      bool motors_ok = motors_s_ && now_s - *motors_s_ < kReportMaxAge;
      // Cutter current comes in 10 mA steps.
      msg.blade_motor = MotorMessage(motors_.cutter, motors_ok, 0.01);
      msg.left_motor = MotorMessage(motors_.left, motors_ok, drive_current_scale_);
      msg.right_motor = MotorMessage(motors_.right, motors_ok, drive_current_scale_);

      const auto& stats = parser_.Stats();
      msg.link_messages_ok = stats.frames;
      msg.link_errors = stats.checksum_errors + stats.length_errors + stats.escape_errors + stats.overflows;
    }
    status_pub_->publish(msg);
  }

  void OnCmdVel(const geometry_msgs::msg::Twist& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    control_.Call<&C::OnCmdVel>(static_cast<float>(msg.linear.x), static_cast<float>(msg.angular.z),
                                MonotonicSeconds());
  }

  void OnImu(const sensor_msgs::msg::Imu& msg) {
    double roll, pitch, yaw = 0;
    const auto& a = msg.linear_acceleration;
    if (msg.orientation_covariance[0] >= 0) {
      const auto& q = msg.orientation;
      roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
      pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
      yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    } else {
      // No orientation in the message: tilt from gravity.
      roll = std::atan2(a.y, a.z);
      pitch = std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z));
    }
    // Like the vendor, the rate fields stay 0.
    auto counts = tron::ImuCountsFromSi(
        pitch * kRadToDeg + imu_pitch_offset_deg_, roll * kRadToDeg + imu_roll_offset_deg_, yaw * kRadToDeg,
        {a.x / kStandardGravity, a.y / kStandardGravity, a.z / kStandardGravity}, {0.0, 0.0, 0.0});
    std::lock_guard<std::mutex> lock(mutex_);
    control_.Call<&C::OnImu>(counts, MonotonicSeconds());
    imu_seen_ = true;
  }

  void OnEmergency(const mowbite_msgs::srv::SetEmergency::Request& req,
                   mowbite_msgs::srv::SetEmergency::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (req.latch) {
      const uint32_t bits = req.bits ? req.bits : tron::kReasonHighLevel;
      if (bits & ~tron::kHostLatchReasons) {
        RCLCPP_ERROR(get_logger(), "Refused to latch with reason bits 0x%x: only HIGH_LEVEL, TILT, FENCE, STUCK and UNEXPECTED_RESTART",
                     bits);
        res.success = false;
        res.message = "only HIGH_LEVEL, TILT, FENCE, STUCK and UNEXPECTED_RESTART can be latched by request";
        return;
      }
      RCLCPP_WARN(get_logger(), "Emergency latched by request (bits 0x%x): %s", bits, req.reason.c_str());
      control_.Call<&C::LatchEmergency>(bits);
      res.message = "latched";
    } else {
      if (!control_.Call<&C::ReleaseEmergency>(MonotonicSeconds(), req.latch_count)) {
        RCLCPP_WARN(get_logger(), "Emergency release refused (%s): another reason latched since", req.reason.c_str());
        res.success = false;
        res.message = "another reason latched since the press";
        res.code = "LATCH_CHANGED";
        return;
      }
      RCLCPP_INFO(get_logger(), "Emergency release requested: %s", req.reason.c_str());
      res.message = "release requested";
    }
    res.success = true;
  }

  void OnBlade(const mowbite_msgs::srv::SetBlade::Request& req, mowbite_msgs::srv::SetBlade::Response& res) {
    tron::BladeRequest request;
    request.on = req.on;
    request.speed_percent = req.speed_percent;
    request.height_mm = req.height_mm;
    request.lease_s = req.lease_s;
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = control_.Call<&C::SetBlade>(request, MonotonicSeconds(), &res.emergency_reason);
    res.success = result == tron::BladeResult::kOk;
    res.message = tron::BladeResultText(result);
    if (!res.success) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Blade request refused: %s", res.message.c_str());
    }
  }

  // Charging follows the contact; the host only stops it, before leaving the station, or allows it again.
  void OnCharge(const mowbite_msgs::srv::SetCharge::Request& req, mowbite_msgs::srv::SetCharge::Response& res) {
    if (read_only_) {
      res.success = false;
      res.message = "read only: nothing goes to the MCU";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      control_.Call<&C::SetCharging>(req.on, MonotonicSeconds());
    }
    if (req.on) {
      RCLCPP_INFO(get_logger(), "Charging allowed again: Charge 1 while there is dock contact");
      res.message = "charging follows the dock contact";
    } else {
      RCLCPP_INFO(get_logger(), "Charging off on request: Charge 0, no Charge 1 until the contact ends");
      res.message = "charging off";
    }
    res.success = true;
  }

  // One PublishStatus at a time; taken before mutex_, never while mutex_ is held.
  std::mutex publish_mutex_;
  std::mutex mutex_;
  // Null without journal_dir.
  mowbite_journal::Journal* journal_;
  uint16_t rx_stream_ = 0;
  uint16_t tx_stream_ = 0;
  mowbite_journal::Journaled<tron::McuControl> control_;
  mowbite_journal::JournalWatch journal_watch_;
  tron::FrameParser parser_;
  tron::SensorInfo sensor_;
  tron::BatteryInfo battery_;
  std::optional<double> battery_s_;
  tron::MotorsInfo motors_;
  std::optional<double> motors_s_;
  std::string cutter_version_;
  std::string chassis_version_;
  std::string bms_version_;

  std::optional<mowbite_hold::HoldWatch> hold_watch_;
  // What PollHold logged last, only for the I/O thread after the start.
  mowbite_hold::HoldView hold_logged_;

  tron::SerialPort port_;
  std::string port_name_;
  std::atomic<bool> running_{true};
  std::thread io_thread_;

  bool read_only_;
  bool stop_release_;
  // seq of the last StopEvent published, only for the I/O thread.
  uint32_t published_stop_seq_ = 0;
  double blade_lease_max_s_;
  // mow_height_mm as the codec sends it.
  uint16_t mow_height_mm_;
  double failsafe_without_host_s_;
  std::string safety_measurement_date_;
  bool blade_on_logged_ = false;
  uint32_t logged_reason_ = 0;
  // BlockOf the last hw/status, under mutex_.
  uint64_t published_block_ = 0;
  double imu_timeout_s_;
  bool imu_seen_ = false;
  double measured_linear_scale_;
  double measured_angular_scale_;
  double drive_current_scale_;
  double imu_roll_offset_deg_;
  double imu_pitch_offset_deg_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<HardwareStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr capabilities_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::ButtonEvent>::SharedPtr button_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<mowbite_msgs::srv::SetEmergency>::SharedPtr emergency_srv_;
  rclcpp::Service<mowbite_msgs::srv::SetBlade>::SharedPtr blade_srv_;
  rclcpp::Service<mowbite_msgs::srv::SetCharge>::SharedPtr charge_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr journal_timer_;
};

}  // namespace tron_mcu

RCLCPP_COMPONENTS_REGISTER_NODE(tron_mcu::McuNode)
