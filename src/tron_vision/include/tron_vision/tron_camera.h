#ifndef TRON_VISION_TRON_CAMERA_H
#define TRON_VISION_TRON_CAMERA_H

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "tron_vision/v4l2_camera.h"
#include "tron_vision/vision_loop.h"

namespace tron_vision {

// A side obstacle camera of the Tron, by default the left one: gc2093 behind the ISP, its subdev found by name. No
// other subdev of the graph is opened: the stereo bridge driver crashes the kernel on a frame interval enumeration.
struct TronCameraConfig {
  std::string sysfs = "/sys/class/video4linux";
  std::string sensor_name = "m01_b_gc2093";
  std::string video_device = "/dev/video53";
  std::string video_name = "rkisp_mainpath";
  uint32_t width = 1920;
  uint32_t height = 1080;
  uint32_t buffers = 4;
  // Also hands out the buffers as dma-buf, for the encoder.
  bool export_buffers = false;
  // Vertical blanking in lines while streaming; 0 leaves the sensor as it is.
  int vertical_blanking = 0;
  // Other processes that have a device open; Start refuses sensor and video node while one does. Empty: no check.
  ProcessesUsing processes_using;
};

// Sensor and video node of the right side camera, the same hardware as the left one.
constexpr char kRightSensorName[] = "m00_b_gc2093";
constexpr char kRightVideoDevice[] = "/dev/video44";

class TronCamera : public Camera {
 public:
  explicit TronCamera(TronCameraConfig config) : config_(std::move(config)) {}

  bool Start(std::string& error) override;
  // Also writes back the sensor values found at Start.
  void Stop() override;
  int Next(int timeout_ms, Nv12View& frame, std::string& error) override;
  // Before Start the setting is kept and applied there.
  bool SetExposure(const ExposureSetting& setting, std::string& error) override;
  const CapturedFrame* Captured() const override;
  bool ShiftChroma(int du, int dv, std::string& error) override;

 private:
  TronCameraConfig config_;
  SensorControls sensor_;
  V4l2Capture capture_;
  ExposureSetting pending_;
  CapturedFrame last_;
  bool have_last_ = false;
};

}  // namespace tron_vision

#endif  // TRON_VISION_TRON_CAMERA_H
