#ifndef TRON_VISION_SIDE_NODE_H
#define TRON_VISION_SIDE_NODE_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mowbite_msgs/msg/camera.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>
#include <thread>

#include "tron_vision/budget.h"
#include "tron_vision/camera_node.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/preview_loop.h"
#include "tron_vision/tron_camera.h"

namespace tron_vision {

Availability CameraAvailability(PreviewState state);

// The live picture of the right side camera of the Tron in a process of its own: it publishes the camera and its
// pictures and nothing else, and it subscribes to no topic. A loop thread runs PreviewLoop with the wish of a viewer
// (a subscriber of the pictures); it ends the process with the exit code of the loop when the limits are broken.
class SideNode : public rclcpp::Node {
 public:
  explicit SideNode(const rclcpp::NodeOptions& options);
  ~SideNode() override;
  SideNode(const SideNode&) = delete;
  SideNode& operator=(const SideNode&) = delete;

  int ExitCode() const {
    return exit_code_;
  }

 private:
  void Run();
  void Log(double now);
  void PublishCamera(const PreviewStatus& status);
  void PublishPicture();

  std::unique_ptr<TronCamera> camera_;
  std::unique_ptr<CrashGuard> guard_;
  std::unique_ptr<PreviewLoop> loop_;
  std::string camera_id_;
  mowbite_msgs::msg::Camera camera_entry_;
  bool camera_published_ = false;
  rclcpp::Publisher<mowbite_msgs::msg::Camera>::SharedPtr cameras_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr picture_pub_;
  LogGate log_gate_{60.0};
  std::string logged_state_;
  uint64_t published_pictures_ = 0;
  std::atomic<int> exit_code_{0};
  bool running_ = true;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::thread thread_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_SIDE_NODE_H
