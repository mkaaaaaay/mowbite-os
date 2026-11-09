#ifndef TRON_VISION_VISION_NODE_H
#define TRON_VISION_VISION_NODE_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mowbite_msgs/msg/camera.hpp>
#include <mowbite_msgs/msg/detections.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>
#include <thread>

#include "tron_vision/budget.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/tron_camera.h"
#include "tron_vision/vision_loop.h"

namespace tron_vision {

// Whether the camera can stream in this state, else the reason of hw/cameras (mowbite_msgs/Camera REASON_*).
struct Availability {
  bool available = false;
  std::string reason;
};
Availability CameraAvailability(VisionState state);

// The camera AI of the Tron in a process of its own: it reports state, detections and the camera with its pictures and
// nothing else. It subscribes to no topic and calls no service, so it cannot move the mower. A loop thread runs
// VisionLoop; it ends the process with the exit code of the loop when the limits are broken.
class VisionNode : public rclcpp::Node {
 public:
  explicit VisionNode(const rclcpp::NodeOptions& options);
  ~VisionNode() override;
  VisionNode(const VisionNode&) = delete;
  VisionNode& operator=(const VisionNode&) = delete;

  int ExitCode() const {
    return exit_code_;
  }

 private:
  void Run();
  void Report(double now);
  void PublishDetections(const VisionStatus& status);
  void PublishCamera(const VisionStatus& status);
  void PublishPicture();
  void Sensor(const std::string& id, const std::string& name, uint8_t kind, const std::string& unit);

  std::unique_ptr<TronCamera> camera_;
  std::unique_ptr<CrashGuard> guard_;
  std::unique_ptr<VisionLoop> loop_;
  std::string camera_id_;
  mowbite_msgs::msg::Camera camera_entry_;
  bool camera_published_ = false;
  rclcpp::Publisher<mowbite_msgs::msg::SensorSpec>::SharedPtr sensor_info_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::Detections>::SharedPtr detections_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::Camera>::SharedPtr cameras_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr picture_pub_;
  LogGate log_gate_{60.0};
  std::string reported_state_;
  double next_report_ = 0.0;
  uint64_t published_inferences_ = 0;
  uint64_t published_pictures_ = 0;
  std::atomic<int> exit_code_{0};
  bool running_ = true;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::thread thread_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_VISION_NODE_H
