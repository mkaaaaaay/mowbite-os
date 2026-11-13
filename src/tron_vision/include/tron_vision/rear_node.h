#ifndef TRON_VISION_REAR_NODE_H
#define TRON_VISION_REAR_NODE_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <functional>
#include <memory>
#include <mowbite_msgs/msg/camera.hpp>
#include <mowbite_msgs/msg/dock_marker.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <thread>

#include "mowbite_watch/attach.h"
#include "tron_vision/budget.h"
#include "tron_vision/camera_node.h"
#include "tron_vision/crash_guard.h"
#include "tron_vision/dma_frames.h"
#include "tron_vision/rear_camera.h"
#include "tron_vision/rear_loop.h"
#include "tron_vision/video_out.h"

namespace tron_vision {

// The reason of a report as hw/dock_marker names it; empty for kNone.
std::string DockReasonText(DockReason reason);

// Stand-ins for the tests; left empty, the node uses the system.
struct RearNodeSeams {
  VideoDevice* device = nullptr;
  // The dma-buf heap of the H.264 frames.
  VideoDevice* heap = nullptr;
  // Applied to the configuration before the loop is made.
  std::function<void(RearConfig&, RearCameraConfig&)> adjust;
  std::string crash_flag = kCrashFlag;
};

// The dock marker seen by the rear camera, in a process of its own. It subscribes to hw/dock_marker_request alone and
// publishes hw/dock_marker and, for the measurements, the pixel values of each image on tron_rear/marker_pixels; it
// cannot move the mower. Beside it the camera on hw/cameras and its live picture for a viewer (a subscriber of the
// pictures, or the client of its RTSP server for H.264), which never delays a measurement. A loop thread runs
// RearLoop; the watch of the process ends it when that thread or the ROS thread stays silent for 2 s (exit code 86),
// the loop ends it over its memory limit (3).
class RearNode : public rclcpp::Node {
 public:
  explicit RearNode(const rclcpp::NodeOptions& options, const RearNodeSeams& seams = {});
  ~RearNode() override;
  RearNode(const RearNode&) = delete;
  RearNode& operator=(const RearNode&) = delete;

  int ExitCode() const {
    return exit_code_;
  }

 private:
  void OnRequest();
  // Up to `seconds`, less when a request comes.
  void WaitForRequest(double seconds);
  double LastRequest();
  void Run();
  void Publish(const DockReport& report);
  void PublishPixels(const DockReport& report, const builtin_interfaces::msg::Time& stamp);
  void Log(const DockReport& report, double now);
  // A control the camera took only on a later try, or not at all.
  void LogControl(const std::string& error, int tries, bool taken);
  void PublishCamera();
  void PublishPicture();
  void LogVideo();

  std::unique_ptr<UvcRearCamera> camera_;
  std::unique_ptr<CrashGuard> guard_;
  std::unique_ptr<DmaFrames> frames_;
  std::unique_ptr<VideoOut> video_;
  std::unique_ptr<RearLoop> loop_;
  rclcpp::Publisher<mowbite_msgs::msg::DockMarker>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pixels_pub_;
  rclcpp::Publisher<mowbite_msgs::msg::Camera>::SharedPtr cameras_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr picture_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr request_sub_;
  mowbite_msgs::msg::Camera camera_entry_;
  bool camera_published_ = false;
  uint64_t published_pictures_ = 0;
  // From the ROS thread to the loop thread: arrival of the last request on the monotonic clock, and their count.
  std::mutex request_mutex_;
  std::condition_variable request_cv_;
  double last_request_ = -1.0;
  uint64_t requests_ = 0;
  LogGate log_gate_{60.0};
  std::string logged_;
  std::string logged_video_;
  std::atomic<int> exit_code_{0};
  std::atomic<bool> running_{true};
  std::unique_ptr<mowbite_watch::Attached> watch_;
  std::thread thread_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_REAR_NODE_H
