#ifndef TRON_VISION_STEREO_NODE_H
#define TRON_VISION_STEREO_NODE_H

#include <atomic>
#include <functional>
#include <memory>
#include <mowbite_msgs/msg/camera.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "mowbite_watch/attach.h"
#include "tron_vision/budget.h"
#include "tron_vision/camera_node.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/h264_encoder.h"
#include "tron_vision/rtsp_server.h"
#include "tron_vision/stereo_camera.h"
#include "tron_vision/stereo_loop.h"

namespace tron_vision {

Availability StereoAvailability(StereoState state);

// Stand-ins for the tests; left empty, the node uses the system.
struct StereoNodeSeams {
  VideoDevice* device = nullptr;
  // Applied to the configuration before the loop is made.
  std::function<void(StereoConfig&, StereoCameraConfig&)> adjust;
  std::string crash_flag = kCrashFlag;
};

// The live picture of the stereo camera in a process of its own. It reads hw/status and nothing else of the stack, and
// publishes only its state and its camera entry, so it cannot move the mower. A loop thread runs StereoLoop with the
// last hw/status and the time it came; the watch of the process ends it when that thread or the ROS thread stays
// silent for 2 s (exit code 86), the loop ends it over its memory limit (3).
class StereoNode : public rclcpp::Node {
 public:
  explicit StereoNode(const rclcpp::NodeOptions& options, const StereoNodeSeams& seams = {});
  ~StereoNode() override;
  StereoNode(const StereoNode&) = delete;
  StereoNode& operator=(const StereoNode&) = delete;

  int ExitCode() const {
    return exit_code_;
  }

 private:
  void OnStatus(const mowbite_msgs::msg::HardwareStatus& msg);
  void Run();
  void Report(const StereoStatus& status, double now);
  void PublishCamera(const StereoStatus& status);

  std::unique_ptr<TronStereoCamera> camera_;
  std::unique_ptr<MppEncoder> encoder_;
  std::unique_ptr<RtspServer> server_;
  std::unique_ptr<CrashGuard> guard_;
  std::unique_ptr<StereoLoop> loop_;
  // Set instead of the loop when the RTSP port cannot be had.
  StereoStatus failed_;
  mowbite_msgs::msg::Camera camera_entry_;
  bool camera_published_ = false;
  rclcpp::Publisher<mowbite_msgs::msg::SensorSpec>::SharedPtr sensor_info_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::Camera>::SharedPtr cameras_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr status_sub_;
  // From the ROS thread to the loop thread.
  std::mutex mower_mutex_;
  MowerView mower_;
  LogGate log_gate_{60.0};
  std::string reported_state_;
  std::string logged_record_;
  std::string logged_gain_;
  std::string logged_gain_error_;
  double next_report_ = 0.0;
  std::atomic<int> exit_code_{0};
  std::atomic<bool> running_{true};
  std::unique_ptr<mowbite_watch::Attached> watch_;
  std::thread thread_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_STEREO_NODE_H
