#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>

#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cstdio>
#include <limits>
#include <mowbite_msgs/msg/gnss_fix.hpp>
#include <mowbite_msgs/msg/hardware_capabilities.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/rtcm.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mowbite_msgs/srv/set_blade.hpp>
#include <mowbite_msgs/srv/set_charge.hpp>
#include <mowbite_msgs/srv/set_emergency.hpp>
#include <mowbite_hold/hold.h>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <thread>

#include "mowbite_gnss/gga.h"
#include "xcore_driver/gnss.h"
#include "xcore_driver/input_config.h"
#include "xcore_driver/link.h"
#include "xcore_driver/services.h"
#include "xcore_driver/udp.h"
#include "xcore_driver/xcore_control.h"
#include "xcore_driver/xcore_io.h"

#ifndef XCORE_MEASUREMENT_BUILD
#define XCORE_MEASUREMENT_BUILD 0
#endif

namespace xcore_driver {

namespace {

using mowbite_msgs::msg::GnssFix;
using mowbite_msgs::msg::HardwareStatus;
using xcore::Bytes;
using xcore::Section;

// The core numbers its reasons like the contract.
static_assert(xcore::kReasonLatch == HardwareStatus::REASON_LATCH);
static_assert(xcore::kReasonTimeoutInputs == HardwareStatus::REASON_TIMEOUT_INPUTS);
static_assert(xcore::kReasonStop == HardwareStatus::REASON_STOP);
static_assert(xcore::kReasonLift == HardwareStatus::REASON_LIFT);
static_assert(xcore::kReasonCollision == HardwareStatus::REASON_COLLISION);
static_assert(xcore::kReasonHighLevel == HardwareStatus::REASON_HIGH_LEVEL);
static_assert(xcore::kReasonFirmwareIncompatible == HardwareStatus::REASON_FIRMWARE_INCOMPATIBLE);
static_assert(xcore::kReasonTilt == HardwareStatus::REASON_TILT);
static_assert(xcore::kReasonFence == HardwareStatus::REASON_FENCE);
static_assert(xcore::kReasonStuck == HardwareStatus::REASON_STUCK);
static_assert(mowbite_hold::kReasonUnexpectedRestart == HardwareStatus::REASON_UNEXPECTED_RESTART);
static_assert(xcore::kReasonMowerNotStopped == HardwareStatus::REASON_MOWER_NOT_STOPPED);
static_assert(xcore::kReasonWheelRunaway == HardwareStatus::REASON_WHEEL_RUNAWAY);
static_assert(xcore::kReasonForeignHost == HardwareStatus::REASON_FOREIGN_HOST);
static_assert(xcore::kReasonDriverStall == HardwareStatus::REASON_DRIVER_STALL);
static_assert(static_cast<uint8_t>(xcore::BladeOffReason::kLeaseExpired) == HardwareStatus::BLADE_OFF_LEASE_EXPIRED);
static_assert(static_cast<uint8_t>(xcore::BladeOffReason::kReconnect) == HardwareStatus::BLADE_OFF_RECONNECT);
static_assert(static_cast<uint8_t>(mowbite_gnss::Solution::kSingle) == GnssFix::SINGLE);
static_assert(static_cast<uint8_t>(mowbite_gnss::Solution::kRtkFloat) == GnssFix::RTK_FLOAT);
static_assert(static_cast<uint8_t>(mowbite_gnss::Solution::kRtkFixed) == GnssFix::RTK_FIXED);

// Built for the safety measurement only (CMake option XCORE_MEASUREMENT_BUILD): takes measurement_speed_limit.
constexpr bool kMeasurementBuild = XCORE_MEASUREMENT_BUILD != 0;
// While the board is claimed, the output thread checks the sockets of other programs this often.
constexpr double kForeignCheckS = 5.0;
// hold.json is looked at this often.
constexpr double kHoldPollS = 0.5;
// hw/status goes out at least this often, at once when a block changes.
constexpr double kStatusPeriodS = 0.1;
// Counts of frequent events become one line per this long at most.
constexpr double kCountLogS = 10.0;
const char kAgeSensor[] = "os_correction_age";

rclcpp::QoS LatchedQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

using xcore::MonotonicSeconds;

int64_t WallSeconds() {
  using namespace std::chrono;
  return duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
}

double UtcSecondsOfDay(int64_t wall_ns) {
  return std::fmod(static_cast<double>(wall_ns) * 1e-9, 86400.0);
}

builtin_interfaces::msg::Time Stamp(int64_t wall_ns) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(wall_ns / 1000000000);
  stamp.nanosec = static_cast<uint32_t>(wall_ns % 1000000000);
  return stamp;
}

const char* BladeOffText(xcore::BladeOffReason reason) {
  switch (reason) {
    case xcore::BladeOffReason::kNone: return "none";
    case xcore::BladeOffReason::kRequest: return "request";
    case xcore::BladeOffReason::kLeaseExpired: return "lease expired";
    case xcore::BladeOffReason::kBlocked: return "blocked";
    case xcore::BladeOffReason::kDock: return "dock contact";
    case xcore::BladeOffReason::kReconnect: return "service restarted";
  }
  return "unknown";
}

Bytes F32(double value) {
  xcore::Writer w;
  w.F32(static_cast<float>(value));
  return w.Take();
}

Bytes F64(double value) {
  xcore::Writer w;
  w.F64(value);
  return w.Take();
}

}  // namespace

// Driver of an OpenMower v2 mower with xCore board behind the hardware contract hw/* of MowBite OS: claims the
// services of the board over UDP, configures them from the platform parameters and passes their values on. The I/O
// thread (xcore::XcoreIo) keeps the board's period and only fills the outbox; the output thread of this node
// publishes, logs and makes the slow checks, and may block without holding up the board.
class XcoreNode : public rclcpp::Node {
 public:
  explicit XcoreNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("xcore_driver", options), inputs_(LoadInputs()) {
    const xcore::ControlConfig control = LoadControl();
    blade_lease_max_s_ = get_parameter("blade_lease_max").as_double();
    max_linear_speed_ = control.max_linear_speed;
    max_angular_speed_ = control.max_angular_speed;
    platform_ = declare_parameter("platform", "openmower_v2");
    xcore::IoConfig io;
    io.interface = declare_parameter("interface", "eth0");
    const std::string bind_ip = declare_parameter("bind_ip", "172.16.78.1");
    io.group = declare_parameter("announcement_group", std::string(xcore::kAnnouncementGroup));
    io.group_port = static_cast<uint16_t>(declare_parameter("announcement_port", int{xcore::kAnnouncementPort}));
    io.expected_major = static_cast<uint16_t>(declare_parameter("expected_major_version", 1));
    imu_frame_id_ = declare_parameter("imu_frame_id", "base_link");
    gnss_source_ = declare_parameter("gnss_source", "xcore_driver");
    io.contest_losses = static_cast<int>(declare_parameter("contest_losses", int64_t{2}));
    io.contest_window_s = declare_parameter("contest_window", 10.0);
    if (io.contest_losses < 2 || !(io.contest_window_s > 0)) {
      throw std::invalid_argument("contest parameters out of range");
    }
    failsafe_without_host_s_ = declare_parameter("failsafe_without_host", -1.0);
    safety_measurement_date_ = declare_parameter("safety_measurement_date", "");
    if (!xcore::ParseIpv4(bind_ip, io.bind_ip)) throw std::invalid_argument("bind_ip is no IPv4 address");
    bind_ip_ = io.bind_ip;
    group_port_ = io.group_port;
    io.services = LoadServices();
    if (safety_measurement_date_.empty()) {
      RCLCPP_WARN(get_logger(), "No safety measurement of this mower yet: remote control only");
    }

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("hw/wheel_odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<HardwareStatus>("hw/status", 10);
    capabilities_pub_ = create_publisher<mowbite_msgs::msg::HardwareCapabilities>("hw/capabilities", LatchedQos());
    PublishCapabilities("");
    if (gnss_source_ == get_name()) {
      SetUpGnss();
    } else {
      RCLCPP_INFO(get_logger(), "GNSS from %s, the board's GPS service is not passed on", gnss_source_.c_str());
    }
    io.gnss = gnss_enabled_;
    io_.emplace(io, control, xcore::InputMapOf(inputs_));
    // Started again by the launch after the last driver ended: it took the board without a latch (M4), so this one
    // holds it from the first frame until a release by the app or a key.
    if (declare_parameter("start_latched", false)) {
      io_->LatchEmergency(xcore::kReasonHighLevel);
      RCLCPP_WARN(get_logger(), "Started again after the last driver ended: emergency latched from the first frame on, "
                  "motion and blade wait for a release");
    }

    // <data_dir>/hold.json of the logic; empty: no hold.
    hold_watch_.emplace(declare_parameter("hold_file", ""));
    if (hold_watch_->path().empty()) RCLCPP_WARN(get_logger(), "No hold_file: an update cannot hold this driver");
    PollHold();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "hw/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          io_->OnCmdVel(msg->linear.x, msg->angular.z);
        });
    emergency_srv_ = create_service<mowbite_msgs::srv::SetEmergency>(
        "hw/emergency", [this](const mowbite_msgs::srv::SetEmergency::Request::SharedPtr req,
                               mowbite_msgs::srv::SetEmergency::Response::SharedPtr res) { OnEmergency(*req, *res); });
    blade_srv_ = create_service<mowbite_msgs::srv::SetBlade>(
        "hw/blade", [this](const mowbite_msgs::srv::SetBlade::Request::SharedPtr req,
                           mowbite_msgs::srv::SetBlade::Response::SharedPtr res) { OnBlade(*req, *res); });
    charge_srv_ = create_service<mowbite_msgs::srv::SetCharge>(
        "hw/charge", [this](const mowbite_msgs::srv::SetCharge::Request::SharedPtr req,
                            mowbite_msgs::srv::SetCharge::Response::SharedPtr res) { OnCharge(*req, *res); });

    // At the end of the process the loop sends its stop inputs before the node goes away.
    context_ = get_node_base_interface()->get_context();
    on_shutdown_ = context_->add_on_shutdown_callback([this] { io_->RequestStop(); });
    io_->Start();
    output_thread_ = std::thread([this] { OutputLoop(); });
  }

  ~XcoreNode() override {
    context_->remove_on_shutdown_callback(on_shutdown_);
    io_->Stop();
    io_->outbox().wakeup.Stop();
    if (output_thread_.joinable()) output_thread_.join();
  }

 private:
  // Poses from the board's GPS service in the frame of the datum, RTCM from gnss/rtcm to it, and a GGA for the caster.
  void SetUpGnss() {
    const double none = std::numeric_limits<double>::quiet_NaN();
    const double lat = declare_parameter("datum_lat", none);
    const double lon = declare_parameter("datum_long", none);
    const double height = declare_parameter("datum_height", none);
    xcore::GnssConfig config;
    config.accuracy_rtk_fixed_m = declare_parameter("accuracy_rtk_fixed", config.accuracy_rtk_fixed_m);
    config.accuracy_rtk_float_m = declare_parameter("accuracy_rtk_float", config.accuracy_rtk_float_m);
    config.accuracy_single_m = declare_parameter("accuracy_single", config.accuracy_single_m);
    config.min_accuracy_m = declare_parameter("min_accuracy", config.min_accuracy_m);
    config.max_correction_age_s = declare_parameter("max_correction_age", config.max_correction_age_s);
    max_correction_age_s_ = config.max_correction_age_s;
    gnss_frame_id_ = declare_parameter("gnss_frame_id", gnss_frame_id_);

    gga_pub_ = create_publisher<std_msgs::msg::String>("gnss/gga", 10);
    rtcm_sub_ = create_subscription<mowbite_msgs::msg::Rtcm>(
        "gnss/rtcm", rclcpp::QoS(50).reliable(), [this](mowbite_msgs::msg::Rtcm::ConstSharedPtr msg) { OnRtcm(*msg); });
    sensor_info_pub_ = create_publisher<mowbite_msgs::msg::SensorSpec>(
        "bridge/sensor_info", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    sensor_value_pub_ = create_publisher<mowbite_msgs::msg::SensorValue>("bridge/sensor_value", 100);
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
    gnss_enabled_ = true;

    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(height) || std::abs(lat) > 90.0 ||
        std::abs(lon) > 180.0) {
      RCLCPP_ERROR(get_logger(), "datum_lat, datum_long and datum_height are not set, no pose is published");
      return;
    }
    gnss_.emplace(config, mowbite_gnss::LocalFrame(lat, lon, height));
    pose_pub_ = create_publisher<GnssFix>("gps/absolute_pose", 10);
    // The datum for the map, which belongs to it: the origin of the poses above, not a measured fix.
    datum_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps/datum", LatchedQos());
    sensor_msgs::msg::NavSatFix datum;
    datum.header.stamp = now();
    datum.header.frame_id = gnss_frame_id_;
    datum.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    datum.latitude = lat;
    datum.longitude = lon;
    datum.altitude = height;
    datum.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    datum_pub_->publish(datum);
    // The app reads the datum from params/json, the bridge collects it from this latched topic.
    params_pub_ = create_publisher<std_msgs::msg::String>(
        "bridge/params", rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local());
    char text[160];
    std::snprintf(text, sizeof(text), "{\"node\":\"%s\",\"params\":{\"datum_lat\":%.15g,\"datum_long\":%.15g}}",
                  get_fully_qualified_name(), lat, lon);
    std_msgs::msg::String msg;
    msg.data = text;
    params_pub_->publish(msg);
  }

  xcore::InputConfig LoadInputs() {
    auto lines = declare_parameter("inputs.gpio_lines", std::vector<std::string>{});
    auto active_high = declare_parameter("inputs.gpio_active_high", std::vector<bool>{});
    auto reasons = declare_parameter("inputs.gpio_reasons", std::vector<std::string>{});
    auto delays = declare_parameter("inputs.gpio_delays_ms", std::vector<int64_t>{});
    auto roles = declare_parameter("inputs.gpio_roles", std::vector<std::string>{});
    auto cover = declare_parameter("inputs.cover", std::vector<std::string>{});
    const size_t n = lines.size();
    if (active_high.size() != n || reasons.size() != n || delays.size() != n || roles.size() != n) {
      throw std::invalid_argument("inputs.gpio_* need one entry per line");
    }
    xcore::InputConfig config;
    for (size_t i = 0; i < n; i++) {
      if (delays[i] < 0 || delays[i] > 65535) throw std::invalid_argument("inputs.gpio_delays_ms out of range");
      // "none" stands for an input without emergency reason or role, as a list cannot hold empty strings everywhere.
      auto none = [](const std::string& text) { return text == "none" ? std::string() : text; };
      config.gpio.push_back(xcore::GpioInput{lines[i], active_high[i], none(reasons[i]),
                                             static_cast<uint16_t>(delays[i]), none(roles[i])});
    }
    for (const auto& entry : cover) {
      auto equals = entry.find('=');
      if (equals == std::string::npos) throw std::invalid_argument("inputs.cover entries are id=value");
      config.cover.push_back(xcore::CoverSetting{entry.substr(0, equals), entry.substr(equals + 1)});
    }
    if (auto problem = xcore::CheckInputConfig(config)) throw std::invalid_argument("inputs: " + *problem);
    return config;
  }

  // The cap for driving: 0.3 m/s; only the measurement build takes measurement_speed_limit, read-only, up to 0.6 m/s.
  // Any other build does not start with it: the raised start of the measurement would otherwise run half raised.
  double LinearCap() {
    const char* name = "measurement_speed_limit";
    if (!kMeasurementBuild) {
      if (get_node_parameters_interface()->get_parameter_overrides().count(name)) {
        throw std::invalid_argument(std::string(name) + " is for the measurement image only: this build drives at "
                                    "most 0.3 m/s and does not start with it");
      }
      return xcore::kLinearSpeedCap;
    }
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    fixed.description = "cap for driving in m/s (0 to 0.6), only for the fast runs of M8";
    const double cap = xcore::MeasurementSpeedCap(declare_parameter(name, xcore::kLinearSpeedCap, fixed));
    if (cap > xcore::kLinearSpeedCap) {
      RCLCPP_WARN(get_logger(), "Measurement build: driving capped at %.2f m/s by %s, only for the fast runs of M8",
                  cap, name);
    } else {
      RCLCPP_INFO(get_logger(), "Measurement build: driving capped at %.2f m/s", cap);
    }
    return cap;
  }

  xcore::ControlConfig LoadControl() {
    xcore::ControlConfig c;
    c.cmd_vel_timeout_s = declare_parameter("cmd_vel_timeout", c.cmd_vel_timeout_s);
    c.max_linear_speed =
        xcore::CappedLimit("max_linear_speed", declare_parameter("max_linear_speed", c.max_linear_speed), LinearCap());
    c.max_angular_speed = xcore::CappedLimit(
        "max_angular_speed", declare_parameter("max_angular_speed", c.max_angular_speed), xcore::kAngularSpeedCap);
    c.allow_blade = declare_parameter("allow_blade", false);
    c.blade_speed_percent = static_cast<uint16_t>(std::clamp<int64_t>(declare_parameter("blade_speed_percent", 100), 1, 100));
    c.blade_lease_max_s = declare_parameter("blade_lease_max", c.blade_lease_max_s);
    c.blade_stop_time_s = declare_parameter("blade_stop_time", c.blade_stop_time_s);
    c.blade_still_rpm = declare_parameter("blade_still_rpm", c.blade_still_rpm);
    c.dock_voltage = declare_parameter("dock_voltage", c.dock_voltage);
    c.tilt_limit_deg = declare_parameter("tilt_limit", c.tilt_limit_deg);
    c.tilt_time_s = declare_parameter("tilt_time", c.tilt_time_s);
    c.tilt_release_deg = std::min(declare_parameter("tilt_release", c.tilt_release_deg), c.tilt_limit_deg);
    c.imu_fresh_s = declare_parameter("imu_fresh", c.imu_fresh_s);
    c.uncommanded_speed = declare_parameter("uncommanded_speed", c.uncommanded_speed);
    c.uncommanded_after_s = declare_parameter("uncommanded_after", c.uncommanded_after_s);
    c.uncommanded_window_s = declare_parameter("uncommanded_window", c.uncommanded_window_s);
    c.uncommanded_time_s = declare_parameter("uncommanded_time", c.uncommanded_time_s);
    c.overspeed_factor = declare_parameter("overspeed_factor", c.overspeed_factor);
    c.overspeed_time_s = declare_parameter("overspeed_time", c.overspeed_time_s);
    c.io_stall_s = declare_parameter("io_stall", c.io_stall_s);
    // The board stops after 1.0 s without the host; a stall must count as lost before that.
    if (!(c.tilt_limit_deg > 0) || !(c.tilt_time_s >= 0) || !(c.uncommanded_speed > 0) || !(c.overspeed_factor >= 1) ||
        !(c.io_stall_s > 0) || c.io_stall_s >= 1.0) {
      throw std::invalid_argument("tilt, wheel or stall parameters out of range");
    }
    c.odometry.ticks_per_meter = declare_parameter("ticks_per_meter", c.odometry.ticks_per_meter);
    c.odometry.wheel_distance = declare_parameter("wheel_distance", c.odometry.wheel_distance);
    c.odometry.left_sign = declare_parameter("left_ticks_sign", c.odometry.left_sign) < 0 ? -1 : 1;
    c.odometry.right_sign = declare_parameter("right_ticks_sign", c.odometry.right_sign) < 0 ? -1 : 1;
    c.regulator.wheel_distance = c.odometry.wheel_distance;
    // A file of the old speed model would otherwise drive with the defaults of the new one.
    if (get_node_parameters_interface()->get_parameter_overrides().count("full_duty_speed")) {
      throw std::invalid_argument("full_duty_speed is gone: give duty_slope and duty_deadband instead");
    }
    c.regulator.duty_slope = declare_parameter("duty_slope", c.regulator.duty_slope);
    c.regulator.duty_deadband = declare_parameter("duty_deadband", c.regulator.duty_deadband);
    c.regulator.max_duty = xcore::CappedLimit("max_duty", declare_parameter("max_duty", c.regulator.max_duty), 1.0);
    c.regulator.feedback = declare_parameter("regulator_feedback", c.regulator.feedback);
    if (!(c.odometry.ticks_per_meter > 0) || !(c.odometry.wheel_distance > 0)) {
      throw std::invalid_argument("ticks_per_meter and wheel_distance must be positive");
    }
    if (auto problem = xcore::CheckSpeedModel(c.regulator)) throw std::invalid_argument(*problem);
    const double parameter_linear = c.max_linear_speed;
    const double parameter_angular = c.max_angular_speed;
    xcore::ApplyReachableSpeeds(c);
    if (c.max_linear_speed < parameter_linear || c.max_angular_speed < parameter_angular) {
      RCLCPP_INFO(get_logger(), "max_duty %.2f reaches %.3f m/s per wheel: driving at most %.3f m/s instead of %.2f, "
                  "turning at most %.3f rad/s instead of %.2f", c.regulator.max_duty,
                  xcore::ReachableWheelSpeed(c.regulator), c.max_linear_speed, parameter_linear, c.max_angular_speed,
                  parameter_angular);
    }
    RCLCPP_INFO(get_logger(), "allow_blade: %d, cmd_vel_timeout: %.2f s, at most %.3f m/s and %.3f rad/s, duty = v / "
                "%.3f + %.3f up to max_duty %.2f, feedback: %d", c.allow_blade, c.cmd_vel_timeout_s,
                c.max_linear_speed, c.max_angular_speed, c.regulator.duty_slope, c.regulator.duty_deadband,
                c.regulator.max_duty, c.regulator.feedback);
    RCLCPP_INFO(get_logger(), "tilt %.0f deg for %.1f s, release below %.0f deg; wheels %.2f m/s %.1f-%.1f s after a "
                "setpoint, %.1fx the fastest wheel; I/O stall %.2f s", c.tilt_limit_deg, c.tilt_time_s,
                c.tilt_release_deg, c.uncommanded_speed, c.uncommanded_after_s, c.uncommanded_window_s,
                c.overspeed_factor, c.io_stall_s);
    return c;
  }

  // The registers of every service the driver claims, in the order of the announced descriptions.
  std::map<uint16_t, std::vector<Section>> LoadServices() {
    std::map<uint16_t, std::vector<Section>> services;
    using namespace xcore;

    const std::string firmware = declare_parameter("robot_firmware", "YardForce");
    if (firmware.empty() || firmware.size() > 50) throw std::invalid_argument("robot_firmware takes 1 to 50 characters");
    // Without a null byte: the board compares the name with exactly the bytes it gets.
    services[kMetaService] = {{meta::kRegRobotFirmware, Bytes(firmware.begin(), firmware.end())}};

    services[kEmergencyService] = {};
    services[kMowerService] = {};
    services[kDiffDriveService] = {{diff_drive::kRegTicksPerMeter, F64(get_parameter("ticks_per_meter").as_double())},
                                   {diff_drive::kRegWheelDistance, F64(get_parameter("wheel_distance").as_double())}};

    auto remap = declare_parameter("imu_axis_remap", std::vector<int64_t>{1, -2, -3});
    if (remap.size() != 3) throw std::invalid_argument("imu_axis_remap takes three axes");
    Bytes axes;
    for (int64_t axis : remap) {
      if (axis == 0 || axis < -3 || axis > 3) throw std::invalid_argument("imu_axis_remap takes ±1, ±2 or ±3");
      axes.push_back(static_cast<uint8_t>(static_cast<int8_t>(axis)));
    }
    services[kImuService] = {{imu::kRegAxisRemap, axes}};

    const double empty = declare_parameter("battery_empty_voltage", 0.0);
    const double full = declare_parameter("battery_full_voltage", 0.0);
    // The first three as the logic has them (nx60i.launch.py hands them over), the last the board's alone.
    const double low = declare_parameter("battery_critical_voltage", 0.0);
    const double high = declare_parameter("battery_critical_high_voltage", 0.0);
    if (!(low > 0) || !(empty > low) || !(full > empty) || !(high > full)) {
      throw std::invalid_argument(
          "battery voltages need 0 < battery_critical_voltage < battery_empty_voltage < battery_full_voltage < "
          "battery_critical_high_voltage");
    }
    std::vector<Section> power = {{power::kRegEmptyVoltage, F32(empty)},
                                  {power::kRegFullVoltage, F32(full)},
                                  {power::kRegCriticalLowVoltage, F32(low)},
                                  {power::kRegCriticalHighVoltage, F32(high)}};
    // Optional; left out at 0 so that the board keeps its own value.
    const double charge_voltage = declare_parameter("charge_voltage", 0.0);
    const double termination_current = declare_parameter("charge_termination_current", 0.0);
    if (charge_voltage > 0) power.push_back({power::kRegChargeVoltage, F32(charge_voltage)});
    if (termination_current > 0) power.push_back({power::kRegTerminationCurrent, F32(termination_current)});
    services[kPowerService] = power;

    // A value other than the one the board started with restarts it: these must be what the mower always had.
    const int64_t uart = declare_parameter("gps_uart", int64_t{-1});
    const int64_t baudrate = declare_parameter("gps_baudrate", int64_t{0});
    const std::string protocol = declare_parameter("gps_protocol", "");
    if (uart < 0 || uart > 255 || baudrate <= 0 || baudrate > 4000000 || (protocol != "nmea" && protocol != "ubx")) {
      throw std::invalid_argument("gps_uart, gps_baudrate and gps_protocol (nmea or ubx) must be set");
    }
    Writer baud;
    baud.U32(static_cast<uint32_t>(baudrate));
    services[kGpsService] = {{gps::kRegBaudrate, baud.Take()},
                             {gps::kRegProtocol, Bytes{static_cast<uint8_t>(protocol == "nmea" ? 1 : 0)}},
                             {gps::kRegUart, Bytes{static_cast<uint8_t>(uart)}}};

    services[kInputService] = {{input::kRegConfigs, HeatshrinkLiterals(InputConfigJson(inputs_))}};
    RCLCPP_INFO(get_logger(), "Robot '%s', GPS on UART %ld at %ld baud (%s), inputs %s", firmware.c_str(), uart,
                baudrate, protocol.c_str(), InputConfigJson(inputs_).c_str());
    return services;
  }

  // Publishes, logs and makes the slow checks, in this order of each round: a changed block first, then the
  // capabilities, the readings in their order, the log lines, the counts, hold.json and the sockets of other programs.
  void OutputLoop() {
    auto& out = io_->outbox();
    double now = MonotonicSeconds();
    double next_status = now;
    double next_hold = now + kHoldPollS;
    double next_foreign = now + kForeignCheckS;
    while (!out.wakeup.stopped()) {
      const double wait = std::max(0.0, std::min(next_status, next_hold) - MonotonicSeconds());
      out.wakeup.WaitFor(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(wait)));
      now = MonotonicSeconds();
      if (out.status_due.exchange(false) || now >= next_status) {
        next_status = now + kStatusPeriodS;
        PublishStatus(io_->Snapshot());
      }
      if (out.capabilities_due.exchange(false)) PublishCapabilities(io_->Snapshot().firmware_version);
      for (const auto& reading : out.twists.TakeAll()) PublishTwist(reading);
      for (const auto& reading : out.imu.TakeAll()) PublishImu(reading);
      for (const auto& reading : out.gps.TakeAll()) OnGnss(reading);
      for (const auto& line : out.log.TakeAll()) Log(line);
      ReportCounts(now);
      if (now >= next_hold) {
        next_hold = now + kHoldPollS;
        PollHold();
      }
      // While the board is claimed, a second host beside this driver latches (FOREIGN_HOST).
      if (!io_->open()) {
        next_foreign = now + kForeignCheckS;
      } else if (now >= next_foreign) {
        next_foreign = now + kForeignCheckS;
        if (auto foreign = xcore::ForeignHosts(bind_ip_, group_port_)) io_->LatchForeignHost(*foreign);
      }
    }
  }

  void Log(const mowbite_outbox::LogLine& line) {
    using Level = mowbite_outbox::LogLine::Level;
    switch (line.level) {
      case Level::kDebug: RCLCPP_DEBUG(get_logger(), "%s", line.text.c_str()); break;
      case Level::kInfo: RCLCPP_INFO(get_logger(), "%s", line.text.c_str()); break;
      case Level::kWarn: RCLCPP_WARN(get_logger(), "%s", line.text.c_str()); break;
      case Level::kError: RCLCPP_ERROR(get_logger(), "%s", line.text.c_str()); break;
    }
  }

  // Counts of the I/O thread and of what the outbox gave up, as throttled lines.
  void ReportCounts(double now) {
    auto& out = io_->outbox();
    send_failed_ += out.send_failed.Take();
    if (send_failed_ > 0 && send_failed_log_.Allow(now)) {
      RCLCPP_ERROR(get_logger(), "Cannot send to the board: %lu datagrams not sent",
                   static_cast<unsigned long>(std::exchange(send_failed_, 0)));
    }
    const auto late = out.late_ticks.Take();
    late_ticks_ += late.count;
    late_max_s_ = std::max(late_max_s_, late.max);
    if (late_ticks_ > 0 && late_tick_log_.Allow(now)) {
      RCLCPP_WARN(get_logger(), "I/O thread: %lu ticks more than %.0f ms late, the latest %.3f s",
                  static_cast<unsigned long>(std::exchange(late_ticks_, 0)), xcore::XcoreIo::kLateTickS * 1000,
                  std::exchange(late_max_s_, 0.0));
    }
    const uint64_t rtcm_closed = out.rtcm_dropped_closed.Take();
    rtcm_closed_total_ += rtcm_closed;
    if (rtcm_closed > 0) rtcm_closed_pending_ = true;
    if (rtcm_closed_pending_ && rtcm_closed_log_.Allow(now)) {
      rtcm_closed_pending_ = false;
      RCLCPP_WARN(get_logger(), "RTCM dropped while the GPS service does not run, %lu bytes so far",
                  static_cast<unsigned long>(rtcm_closed_total_));
    }
    if (rtcm_full_.exchange(false)) rtcm_full_pending_ = true;
    if (rtcm_full_pending_ && rtcm_full_log_.Allow(now)) {
      rtcm_full_pending_ = false;
      RCLCPP_WARN(get_logger(), "RTCM queue full, %lu bytes dropped so far",
                  static_cast<unsigned long>(io_->rtcm_dropped_bytes()));
    }
    // Only after a hang of this thread: the I/O thread went on, what did not fit is gone.
    dropped_twists_ += out.twists.TakeDropped();
    dropped_imu_ += out.imu.TakeDropped();
    dropped_gps_ += out.gps.TakeDropped();
    dropped_lines_ += out.log.TakeDropped();
    if (dropped_twists_ + dropped_imu_ + dropped_gps_ + dropped_lines_ > 0 && dropped_log_.Allow(now)) {
      RCLCPP_WARN(get_logger(), "Output fell behind: %lu odometry, %lu IMU and %lu GPS readings and %lu log lines "
                  "dropped", static_cast<unsigned long>(std::exchange(dropped_twists_, 0)),
                  static_cast<unsigned long>(std::exchange(dropped_imu_, 0)),
                  static_cast<unsigned long>(std::exchange(dropped_gps_, 0)),
                  static_cast<unsigned long>(std::exchange(dropped_lines_, 0)));
    }
  }

  // The hold of an update, from hold.json itself: it stands without the logic and right after a restart.
  void PollHold() {
    const mowbite_hold::HoldView view = hold_watch_->Poll(MonotonicSeconds(), WallSeconds());
    io_->SetHold(view.held);
    if (view.held == hold_logged_.held && view.unreadable == hold_logged_.unreadable &&
        view.until == hold_logged_.until) {
      return;
    }
    if (view.unreadable) {
      RCLCPP_ERROR(get_logger(), "Held: %s; motion and blade stay blocked until it is readable or gone",
                   view.error.c_str());
    } else if (view.held) {
      RCLCPP_INFO(get_logger(), "Held for an update until %lld (unix s): motion and blade blocked",
                  static_cast<long long>(view.until));
    } else if (hold_logged_.held) {
      RCLCPP_INFO(get_logger(), "Hold ended: motion and blade allowed again");
    }
    hold_logged_ = view;
  }

  // In the output thread, which alone uses the converter.
  void OnGnss(const xcore::GpsReading& reading) {
    auto solution = xcore::DecodeGnss(reading.outputs);
    if (!solution) return;
    const double now = MonotonicSeconds();
    if (now - last_gga_s_ >= 1.0) {
      last_gga_s_ = now;
      std_msgs::msg::String gga;
      gga.data = mowbite_gnss::BuildGga(xcore::GgaOf(*solution, UtcSecondsOfDay(reading.stamp_ns)));
      gga_pub_->publish(gga);
      if (reading.correction_age_s) {
        mowbite_msgs::msg::SensorValue value;
        value.sensor_id = kAgeSensor;
        value.value = *reading.correction_age_s;
        sensor_value_pub_->publish(value);
      }
    }
    if (!gnss_) return;
    auto pose = gnss_->Convert(*solution, reading.correction_age_s);
    if (gnss_->fixed_without_corrections() != reported_downgrades_) {
      reported_downgrades_ = gnss_->fixed_without_corrections();
      if (auto age = gnss_->last_downgrade_age()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "RTK fixed on corrections passed %.1f s ago (limit %.0f s): counted as float", *age,
                             max_correction_age_s_);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "RTK fixed before the host passed any corrections to the board: counted as float");
      }
    }
    if (gnss_->repeated() != reported_repeats_) {
      reported_repeats_ = gnss_->repeated();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "GPS solution with the board time of the one before, "
                           "%lu dropped so far", static_cast<unsigned long>(reported_repeats_));
    }
    if (pose) PublishPose(*pose, reading.stamp_ns);
  }

  void PublishPose(const xcore::GnssPose& pose, int64_t stamp_ns) {
    GnssFix msg;
    msg.header.stamp = Stamp(stamp_ns);
    msg.header.frame_id = gnss_frame_id_;
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

  void PublishTwist(const xcore::TwistReading& reading) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = Stamp(reading.stamp_ns);
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = reading.twist.linear;
    msg.twist.angular.z = reading.twist.angular;
    twist_pub_->publish(msg);
  }

  void PublishImu(const xcore::ImuReading& reading) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = Stamp(reading.stamp_ns);
    msg.header.frame_id = imu_frame_id_;
    // No orientation: the board has no fusion, users take the tilt from gravity.
    msg.orientation_covariance[0] = -1.0;
    msg.linear_acceleration.x = reading.axes[0];
    msg.linear_acceleration.y = reading.axes[1];
    msg.linear_acceleration.z = reading.axes[2];
    msg.angular_velocity.x = reading.axes[3];
    msg.angular_velocity.y = reading.axes[4];
    msg.angular_velocity.z = reading.axes[5];
    imu_pub_->publish(msg);
  }

  // Before the threads start, then only in the output thread.
  void PublishCapabilities(const std::string& firmware_version) {
    mowbite_msgs::msg::HardwareCapabilities msg;
    msg.platform = platform_;
    diagnostic_msgs::msg::KeyValue firmware;
    firmware.key = "xcore";
    firmware.value = firmware_version;
    msg.firmware.push_back(firmware);
    msg.blade_speed_control = true;
    msg.blade_height_control = false;
    msg.blade_height_min_mm = 0;
    msg.blade_height_max_mm = 0;
    msg.blade_lease_max_s = static_cast<float>(blade_lease_max_s_);
    // The board charges by itself and ignores the host: there is nothing to start or stop.
    msg.host_charge_control = false;
    msg.charge_stop = false;
    msg.rain_sensor = false;
    msg.separate_bumpers = true;
    msg.wheel_ticks = true;
    msg.wheel_ticks_per_m = get_parameter("ticks_per_meter").as_double();
    msg.wheel_distance = get_parameter("wheel_distance").as_double();
    msg.imu_source = "xcore_driver";
    msg.gnss_source = gnss_source_;
    msg.max_linear_speed = static_cast<float>(max_linear_speed_);
    msg.max_angular_speed = static_cast<float>(max_angular_speed_);
    msg.failsafe_without_host_s = static_cast<float>(failsafe_without_host_s_);
    msg.safety_measurement_date = safety_measurement_date_;
    capabilities_pub_->publish(msg);
  }

  static mowbite_msgs::msg::MotorState MotorMessage(const xcore::MotorReport& motor) {
    mowbite_msgs::msg::MotorState msg;
    msg.ok = motor.known && motor.status != xcore::kEscDisconnected;
    if (!motor.known) return msg;
    const double rpm = std::isfinite(motor.rpm) ? motor.rpm : 0.0;
    msg.rpm = static_cast<int16_t>(std::clamp<double>(rpm, std::numeric_limits<int16_t>::min(),
                                                      std::numeric_limits<int16_t>::max()));
    msg.current = std::isfinite(motor.current) ? motor.current : 0.0f;
    const double temperature = std::isfinite(motor.temperature) ? motor.temperature : 0.0;
    msg.temperature = static_cast<int8_t>(std::clamp(std::lround(temperature), -128L, 127L));
    msg.status = xcore::MotorStatusCode(motor.status);
    return msg;
  }

  // Only in the output thread, so that the messages go out in the order they were made.
  void PublishStatus(const xcore::IoSnapshot& snapshot) {
    const auto& s = snapshot.status;
    HardwareStatus msg;
    msg.stamp = now();
    msg.emergency = s.blocked;
    msg.emergency_latched = s.latched;
    msg.emergency_reason = s.emergency_reason;
    msg.telemetry_ok = s.telemetry_ok;
    msg.hold = s.held;
    msg.lift = s.lift;
    msg.stop = s.stop;
    msg.bumper_left = s.bumper_left;
    msg.bumper_right = s.bumper_right;
    msg.bumper = s.bumper_left || s.bumper_right;
    msg.rain = s.rain;
    msg.dock_contact = s.dock_contact;
    msg.dock_contact_raw = s.dock_contact_raw;
    msg.moving = s.moving;
    msg.blade_on = s.blade_on;
    msg.blade_off_reason = static_cast<uint8_t>(s.blade_off_reason);
    if (s.battery.known) {
      // The percentage follows the voltage linearly between empty and full.
      msg.battery_source = HardwareStatus::BATTERY_SOURCE_ESTIMATE;
      msg.battery_voltage = static_cast<float>(s.battery.voltage);
      // Only the charge current is known: negative while charging, 0 otherwise.
      msg.battery_current = s.battery.charge_current > 0 ? static_cast<float>(-s.battery.charge_current) : 0.0f;
      msg.battery_percentage = static_cast<uint8_t>(std::lround(s.battery.fraction * 100.0));
    }
    msg.blade_motor = MotorMessage(s.blade_motor);
    msg.left_motor = MotorMessage(s.left_motor);
    msg.right_motor = MotorMessage(s.right_motor);
    if (snapshot.linked) {
      msg.link_messages_ok = snapshot.link_received;
      msg.link_errors = snapshot.link_dropped;
    }
    status_pub_->publish(msg);
    if (s.blade_on != blade_on_logged_) {
      blade_on_logged_ = s.blade_on;
      if (s.blade_on) {
        RCLCPP_INFO(get_logger(), "Blade on");
      } else {
        RCLCPP_INFO(get_logger(), "Blade off: %s", BladeOffText(s.blade_off_reason));
      }
    }
    if (s.emergency_reason != logged_reason_) {
      logged_reason_ = s.emergency_reason;
      RCLCPP_INFO(get_logger(), "Emergency reason 0x%x", s.emergency_reason);
    }
  }

  // Executor callbacks: computing under the lock of the I/O core, their lines through the outbox.
  void OnEmergency(const mowbite_msgs::srv::SetEmergency::Request& req,
                   mowbite_msgs::srv::SetEmergency::Response& res) {
    auto& log = io_->outbox().log;
    if (req.latch) {
      const uint32_t bits = req.bits ? req.bits : xcore::kReasonHighLevel;
      if (!io_->LatchEmergency(bits)) {
        log.Error(mowbite_outbox::Format(
            "Refused to latch with reason bits 0x%x: only HIGH_LEVEL, TILT, FENCE, STUCK and UNEXPECTED_RESTART", bits));
        res.success = false;
        res.message = "only HIGH_LEVEL, TILT, FENCE, STUCK and UNEXPECTED_RESTART can be latched by request";
        return;
      }
      log.Warn(mowbite_outbox::Format("Emergency latched by request (bits 0x%x): %s", bits, req.reason.c_str()));
      res.message = "latched";
    } else {
      log.Info("Emergency release requested: " + req.reason);
      std::optional<xcore::ReleaseRefusal> refused;
      // /proc before the lock: the I/O thread does not wait for it.
      if (auto foreign = xcore::ForeignHosts(bind_ip_, group_port_)) {
        refused = xcore::ReleaseRefusal{"FOREIGN_HOST", *foreign};
      } else {
        refused = io_->ReleaseEmergency();
      }
      if (refused) {
        log.Error("Emergency release refused: " + refused->message);
        res.success = false;
        res.message = "release refused: " + refused->message;
        res.code = refused->code;
        res.value = static_cast<float>(refused->value);
        res.limit = static_cast<float>(refused->limit);
        return;
      }
      res.message = "release requested";
    }
    res.success = true;
  }

  void OnBlade(const mowbite_msgs::srv::SetBlade::Request& req, mowbite_msgs::srv::SetBlade::Response& res) {
    xcore::BladeRequest request;
    request.on = req.on;
    request.speed_percent = req.speed_percent;
    request.height_mm = req.height_mm;
    request.lease_s = req.lease_s;
    auto result = io_->SetBlade(request, &res.emergency_reason);
    res.success = result == xcore::BladeResult::kOk;
    res.message = xcore::BladeResultText(result);
    if (!res.success && blade_refused_log_.Allow(MonotonicSeconds())) {
      io_->outbox().log.Warn("Blade request refused: " + res.message);
    }
  }

  // The board charges whenever the station delivers and has no input to stop it (charge_stop false).
  void OnCharge(const mowbite_msgs::srv::SetCharge::Request& req, mowbite_msgs::srv::SetCharge::Response& res) {
    io_->outbox().log.Warn(std::string("hw/charge ") + (req.on ? "on" : "off") +
                           " refused: the board controls charging");
    res.success = false;
    res.message = "charging is controlled by the board";
  }

  void OnRtcm(const mowbite_msgs::msg::Rtcm& msg) {
    if (!io_->PushRtcm(msg.data.data(), msg.data.size())) {
      rtcm_full_ = true;
      io_->outbox().wakeup.Notify();
    }
  }

  std::optional<mowbite_hold::HoldWatch> hold_watch_;
  // What PollHold logged last, only for the output thread after the start.
  mowbite_hold::HoldView hold_logged_;
  xcore::InputConfig inputs_;
  std::optional<xcore::XcoreIo> io_;
  std::thread output_thread_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::OnShutdownCallbackHandle on_shutdown_;

  std::string platform_;
  uint32_t bind_ip_ = 0;
  uint16_t group_port_ = 0;
  std::string imu_frame_id_;
  std::string gnss_source_;
  std::string gnss_frame_id_ = "map";
  // GNSS: the converter and what was reported of it belong to the output thread.
  bool gnss_enabled_ = false;
  std::optional<xcore::GnssConverter> gnss_;
  double max_correction_age_s_ = 0;
  double last_gga_s_ = -1e9;
  uint64_t reported_downgrades_ = 0;
  uint64_t reported_repeats_ = 0;
  double blade_lease_max_s_ = 0;
  double max_linear_speed_ = 0;
  double max_angular_speed_ = 0;
  double failsafe_without_host_s_ = -1;
  std::string safety_measurement_date_;
  // Only the output thread.
  bool blade_on_logged_ = false;
  uint32_t logged_reason_ = 0;
  uint64_t send_failed_ = 0;
  uint64_t late_ticks_ = 0;
  double late_max_s_ = 0;
  uint64_t rtcm_closed_total_ = 0;
  bool rtcm_closed_pending_ = false;
  bool rtcm_full_pending_ = false;
  uint64_t dropped_twists_ = 0;
  uint64_t dropped_imu_ = 0;
  uint64_t dropped_gps_ = 0;
  uint64_t dropped_lines_ = 0;
  mowbite_outbox::Throttle send_failed_log_{kCountLogS};
  mowbite_outbox::Throttle late_tick_log_{kCountLogS};
  mowbite_outbox::Throttle rtcm_closed_log_{kCountLogS};
  mowbite_outbox::Throttle rtcm_full_log_{kCountLogS};
  mowbite_outbox::Throttle dropped_log_{kCountLogS};
  // Set by the executor, read by the output thread.
  std::atomic<bool> rtcm_full_{false};
  // Only the executor, whose callbacks run one at a time.
  mowbite_outbox::Throttle blade_refused_log_{5.0};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<HardwareStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::HardwareCapabilities>::SharedPtr capabilities_pub_;
  rclcpp::Publisher<GnssFix>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr datum_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr params_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gga_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorSpec>::SharedPtr sensor_info_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::Rtcm>::SharedPtr rtcm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<mowbite_msgs::srv::SetEmergency>::SharedPtr emergency_srv_;
  rclcpp::Service<mowbite_msgs::srv::SetBlade>::SharedPtr blade_srv_;
  rclcpp::Service<mowbite_msgs::srv::SetCharge>::SharedPtr charge_srv_;
};

}  // namespace xcore_driver

RCLCPP_COMPONENTS_REGISTER_NODE(xcore_driver::XcoreNode)
