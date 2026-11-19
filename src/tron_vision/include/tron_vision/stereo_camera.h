#ifndef TRON_VISION_STEREO_CAMERA_H
#define TRON_VISION_STEREO_CAMERA_H

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "tron_vision/stereo_loop.h"
#include "tron_vision/v4l2_camera.h"

namespace tron_vision {

// Stereo camera of the Tron: both eyes side by side as YUYV from the capture interface behind the bridge chip. Only
// the video node is opened, never a subdev: bridge and sensor drivers crash the kernel on enumerations, and a format
// set on the bridge undoes its setup.
struct StereoCameraConfig {
  std::string sysfs = "/sys/class/video4linux";
  // Platform device of the capture interface and name of its main stream; the number of the node is not fixed.
  std::string capture_device = "rkcif-mipi-lvds3";
  std::string stream_name = "stream_cif_mipi_id0";
  uint32_t width = 1280;
  uint32_t height = 480;
  uint32_t buffers = 4;
  // Other processes that have a device open; Open refuses the node while one does. Empty: no check.
  ProcessesUsing processes_using;
};

class TronStereoCamera : public FrameSource {
 public:
  // Without a device the real system calls are used.
  explicit TronStereoCamera(StereoCameraConfig config, VideoDevice* device = nullptr)
      : config_(std::move(config)), capture_(device) {}

  bool Open(std::string& error) override;
  bool Start(std::string& error) override;
  void Stop() override;
  void Close() override;
  int Next(int timeout_ms, CapturedFrame& frame, std::string& error) override;

 private:
  StereoCameraConfig config_;
  V4l2Capture capture_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_STEREO_CAMERA_H
