#ifndef TRON_VISION_CAMERA_NODE_H
#define TRON_VISION_CAMERA_NODE_H

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <mowbite_msgs/msg/camera.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "tron_vision/crash_guard.h"
#include "tron_vision/frame_recorder.h"

// Parts the camera processes share.
namespace tron_vision {

// Set by the system after a kernel crash (mowbite-crash.service), and the id of this boot.
constexpr char kCrashFlag[] = "/run/mowbite/crash";
constexpr char kBootId[] = "/proc/sys/kernel/random/boot_id";

// Whether a camera can stream in its state, else the reason of hw/cameras (mowbite_msgs/Camera REASON_*).
struct Availability {
  bool available = false;
  std::string reason;
};

// Sets availability and reason of the hw/cameras entry. True when it goes out: the first time, with another
// availability or reason, or when the offer changed; a new detail alone waits for the next of these.
inline bool UpdateAvailability(mowbite_msgs::msg::Camera& entry, bool& published, const Availability& now,
                               bool offer_changed) {
  if (published && now.available == entry.available && now.reason == entry.reason && !offer_changed) return false;
  published = true;
  entry.available = now.available;
  entry.reason = now.reason;
  return true;
}

// The clock of the loops, also for the receive time of hw/status.
inline double MonotonicSeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// A sensor of bridge/sensor_info without range or limits.
inline mowbite_msgs::msg::SensorSpec PlainSensor(const std::string& id, const std::string& name, uint8_t kind,
                                                 const std::string& unit) {
  const double none = std::numeric_limits<double>::quiet_NaN();
  mowbite_msgs::msg::SensorSpec spec;
  spec.id = id;
  spec.name = name;
  spec.kind = kind;
  spec.meaning = mowbite_msgs::msg::SensorSpec::MEANING_UNKNOWN;
  spec.unit = unit;
  spec.min = none;
  spec.max = none;
  spec.critical_low = none;
  spec.critical_high = none;
  return spec;
}

// Offers /camera/<id>/unlock (std_srvs/Trigger, see CrashGuard::Unlock), absolute like /camera/<id>/jpeg. The
// loop thread answers through `requests`; the executor waits at most kUnlockWait, about one idle wait of the loop
// (200 ms) in practice.
constexpr auto kUnlockWait = std::chrono::seconds(2);
inline rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr OfferUnlock(rclcpp::Node& node, const std::string& id,
                                                                     UnlockRequests& requests,
                                                                     std::function<void()> wake) {
  return node.create_service<std_srvs::srv::Trigger>(
      "/camera/" + id + "/unlock",
      [&requests, wake](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (wake) wake();
        response->success = requests.Ask(kUnlockWait, response->message);
      });
}

// Most a camera may record. /run is RAM; the Tron has 4 GB and /run 781 MB, for all recordings together.
constexpr int64_t kMaxRecordMb = 300;

// Declares a value that may come with any type: one of the wrong type in the device file must not end the node.
inline rclcpp::ParameterValue AnyType(rclcpp::Node& node, const std::string& name,
                                      const rclcpp::ParameterValue& preset) {
  rcl_interfaces::msg::ParameterDescriptor any;
  any.dynamic_typing = true;
  return node.declare_parameter(name, preset, any);
}

// A whole or a decimal number; false for any other type.
inline bool Number(const rclcpp::ParameterValue& value, double& number) {
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    number = value.get<double>();
  } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    number = static_cast<double>(value.get<int64_t>());
  } else {
    return false;
  }
  return true;
}

// True for a path below /run without a step up.
inline bool BelowRun(const std::string& path) {
  return path.rfind("/run/", 0) == 0 && path.find("..") == std::string::npos;
}

// record_dir, record_max_mb, record_keep_min and record_interval_s of a camera. The recording lives in RAM under /run
// only, within kMaxRecordMb; off with record_max_mb 0. A wrong value, also one of the wrong type, leaves it off and
// the camera goes on.
inline void ReadRecordLimits(rclcpp::Node& node, const std::string& preset_dir, RecordLimits& record) {
  const rclcpp::ParameterValue dir = AnyType(node, "record_dir", rclcpp::ParameterValue(preset_dir));
  const rclcpp::ParameterValue max_value = AnyType(node, "record_max_mb", rclcpp::ParameterValue(int64_t{0}));
  const rclcpp::ParameterValue keep_value =
      AnyType(node, "record_keep_min", rclcpp::ParameterValue(record.keep_s / 60.0));
  const rclcpp::ParameterValue interval_value =
      AnyType(node, "record_interval_s", rclcpp::ParameterValue(record.interval_s));
  double max_mb = 0.0;
  double keep_min = 0.0;
  double interval_s = 0.0;
  std::string wrong;
  if (dir.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    wrong = "record_dir is not a string";
  } else if (!Number(max_value, max_mb) || !Number(keep_value, keep_min) || !Number(interval_value, interval_s)) {
    wrong = "record_max_mb, record_keep_min and record_interval_s are not all numbers";
  } else if (!(max_mb >= 0.0 && max_mb <= static_cast<double>(kMaxRecordMb)) || max_mb != std::floor(max_mb)) {
    wrong = "record_max_mb is not a whole number within 0.." + std::to_string(kMaxRecordMb);
  } else if (!BelowRun(dir.get<std::string>())) {
    wrong = "record_dir " + dir.get<std::string>() + " is not below /run";
  } else if (!(keep_min > 0.0) || !(interval_s > 0.0)) {
    wrong = "record_keep_min and record_interval_s are not above 0";
  }
  if (!wrong.empty()) {
    RCLCPP_WARN(node.get_logger(), "No recording: %s", wrong.c_str());
    return;
  }
  record.dir = dir.get<std::string>();
  record.keep_s = keep_min * 60.0;
  record.interval_s = interval_s;
  record.max_bytes = static_cast<size_t>(max_mb) << 20;
  if (record.max_bytes > 0) {
    RCLCPP_INFO(node.get_logger(), "Recording to %s, at most %.0f MB, %.0f min, one image per %.2f s",
                record.dir.c_str(), max_mb, record.keep_s / 60.0, record.interval_s);
  }
}

}  // namespace tron_vision

#endif  // TRON_VISION_CAMERA_NODE_H
