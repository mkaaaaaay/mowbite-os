#ifndef TRON_VISION_CAMERA_H
#define TRON_VISION_CAMERA_H

#include <functional>
#include <string>

#include "tron_vision/exposure.h"
#include "tron_vision/frame_convert.h"
#include "tron_vision/v4l2_camera.h"

namespace tron_vision {

// Longest wait for a frame in one step of a camera loop, so that its thread signs the watch of the process often.
constexpr double kFrameWaitStepS = 0.1;

// A camera that streams NV12 and takes exposure settings.
class Camera {
 public:
  virtual ~Camera() = default;
  virtual bool Start(std::string& error) = 0;
  virtual void Stop() = 0;
  // As V4l2Capture::Next; the frame stays valid until the next call or Stop().
  virtual int Next(int timeout_ms, Nv12View& frame, std::string& error) = 0;
  virtual bool SetExposure(const ExposureSetting& setting, std::string& error) = 0;
  // The buffer of the frame of the last Next() for a reader of its dma-buf; nullptr unless the camera exports them.
  virtual const CapturedFrame* Captured() const {
    return nullptr;
  }
  // Changes that frame in place, between the cache syncs of its dma-buf.
  virtual bool EditFrame(const std::function<void(const Nv12Buffer&)>& edit, std::string& error) {
    (void)edit;
    error = "the camera does not export its frames";
    return false;
  }
};

}  // namespace tron_vision

#endif  // TRON_VISION_CAMERA_H
