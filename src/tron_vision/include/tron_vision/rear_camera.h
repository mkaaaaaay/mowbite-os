#ifndef TRON_VISION_REAR_CAMERA_H
#define TRON_VISION_REAR_CAMERA_H

#include <cstdint>
#include <string>
#include <utility>

#include "tron_vision/exposure.h"
#include "tron_vision/v4l2_camera.h"

namespace tron_vision {

// A camera for the dock marker: frames, and the manual exposure in the units of its controls.
class DockCamera : public FrameSource {
 public:
  // False with the reason while the device is not there.
  virtual bool Present(std::string& error) = 0;
  // Switches the open camera to manual exposure and gives the range and the values now of the exposure time (100 us)
  // and the gain; false if it has no manual exposure, it then keeps its own.
  virtual bool ManualExposure(ExposureRange& range, ExposureSetting& now, std::string& error) = 0;
  virtual bool SetExposure(const ExposureSetting& setting, std::string& error) = 0;
};

// Rear camera of the Tron, a USB camera with UVC: MJPEG from its capture node, which a udev rule names by the USB id.
struct RearCameraConfig {
  std::string device = "/dev/mowbite-rear";
  uint32_t width = 1920;
  uint32_t height = 1080;
  uint32_t buffers = 4;
  // Other processes that have a device open; Open refuses the node while one does. Empty: no check.
  ProcessesUsing processes_using;
};

// The exposure controls of UVC; Close() gives them back the values they had before ManualExposure().
class UvcRearCamera : public DockCamera {
 public:
  // Without a device the real system calls are used.
  explicit UvcRearCamera(RearCameraConfig config, VideoDevice* device = nullptr)
      : config_(std::move(config)), capture_(device) {}

  bool Present(std::string& error) override;
  bool Open(std::string& error) override;
  bool Start(std::string& error) override;
  void Stop() override;
  void Close() override;
  int Next(int timeout_ms, CapturedFrame& frame, std::string& error) override;
  bool ManualExposure(ExposureRange& range, ExposureSetting& now, std::string& error) override;
  bool SetExposure(const ExposureSetting& setting, std::string& error) override;

 private:
  RearCameraConfig config_;
  V4l2Capture capture_;
  // Values before the switch to manual, written back on Close().
  bool manual_ = false;
  bool has_gain_ = false;
  int saved_mode_ = 0;
  ExposureSetting saved_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_REAR_CAMERA_H
