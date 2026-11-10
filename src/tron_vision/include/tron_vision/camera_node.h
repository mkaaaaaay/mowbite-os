#ifndef TRON_VISION_CAMERA_NODE_H
#define TRON_VISION_CAMERA_NODE_H

#include <chrono>
#include <cstdint>
#include <limits>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <string>

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

}  // namespace tron_vision

#endif  // TRON_VISION_CAMERA_NODE_H
