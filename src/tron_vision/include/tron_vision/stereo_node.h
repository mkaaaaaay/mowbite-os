#ifndef TRON_VISION_STEREO_NODE_H
#define TRON_VISION_STEREO_NODE_H

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <mowbite_msgs/msg/camera.hpp>
#include <mowbite_msgs/msg/hardware_status.hpp>
#include <mowbite_msgs/msg/obstacle_image.hpp>
#include <mowbite_msgs/msg/obstacles.hpp>
#include <mowbite_msgs/msg/sensor_spec.hpp>
#include <mowbite_msgs/msg/sensor_value.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <thread>

#include "mowbite_watch/attach.h"
#include "tron_vision/budget.h"
#include "tron_vision/camera_node.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/h264_encoder.h"
#include "tron_vision/obstacle_core.h"
#include "tron_vision/rtsp_server.h"
#include "tron_vision/stereo_camera.h"
#include "tron_vision/stereo_loop.h"

namespace tron_vision {

Availability StereoAvailability(StereoState state);
std::string ObstacleReasonName(SightReason reason);
// One report of the core as the message of hw/obstacles; `stamp` is when its frame was taken, by the host clock.
mowbite_msgs::msg::Obstacles ObstaclesMessage(const ObstacleReport& report, const std::string& source,
                                               const builtin_interfaces::msg::Time& stamp);

// Stand-ins for the tests; left empty, the node uses the system.
struct StereoNodeSeams {
  VideoDevice* device = nullptr;
  // Applied to the configuration before the loop is made.
  std::function<void(StereoConfig&, StereoCameraConfig&)> adjust;
  std::function<void(ObstacleConfig&)> adjust_obstacles;
  std::string crash_flag = kCrashFlag;
};

// The live picture of the stereo camera in a process of its own. It reads hw/status and nothing else of the stack, and
// publishes only its state, its camera entry and, with obstacles on, what it sees on the ground (hw/obstacles,
// hw/obstacle_images), so it cannot move the mower. A loop thread runs StereoLoop with the last hw/status and the time
// it came, and the obstacles beside it; the watch of the process ends it when that thread or the ROS thread stays
// silent for 2 s (exit code 86), or when loading the model takes over 20 s; the loop ends it over its memory limit (3),
// the model when it hangs or keeps running slow (4).
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
  void TryListen();
  void Report(const StereoStatus& status, double now);
  void PublishCamera(const StereoStatus& status);
  void PublishObstacles(const ObstacleReport& report, double now);

  std::unique_ptr<TronStereoCamera> camera_;
  std::unique_ptr<MppEncoder> encoder_;
  std::unique_ptr<RtspServer> server_;
  std::unique_ptr<CrashGuard> guard_;
  std::unique_ptr<ObstacleCore> obstacles_;
  std::unique_ptr<StereoLoop> loop_;
  UnlockRequests unlock_requests_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unlock_srv_;
  std::string camera_id_;
  // Set instead of the loop while the RTSP port cannot be had; the loop gets the config once it is.
  StereoStatus failed_;
  std::optional<StereoConfig> waiting_config_;
  int rtsp_port_ = 0;
  double next_listen_ = 0.0;
  mowbite_msgs::msg::Camera camera_entry_;
  bool camera_published_ = false;
  rclcpp::Publisher<mowbite_msgs::msg::SensorSpec>::SharedPtr sensor_info_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::SensorValue>::SharedPtr sensor_value_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::Camera>::SharedPtr cameras_pub_;
  rclcpp::Subscription<mowbite_msgs::msg::HardwareStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<mowbite_msgs::msg::Obstacles>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::ObstacleImage>::SharedPtr images_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr photos_sub_;
  std::string reported_obstacles_;
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
