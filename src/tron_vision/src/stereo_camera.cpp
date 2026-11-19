#include "tron_vision/stereo_camera.h"

namespace tron_vision {

bool TronStereoCamera::Open(std::string& error) {
  Close();
  const std::string node = FindVideoNode(config_.sysfs, config_.capture_device, config_.stream_name);
  if (node.empty()) {
    error = "no single video node " + config_.stream_name + " of " + config_.capture_device;
    return false;
  }
  return DeviceFree(config_.processes_using, node, error) &&
         capture_.Open(node, {config_.width, config_.height, PixelFormat::kYuyv, config_.buffers, true}, error);
}

bool TronStereoCamera::Start(std::string& error) {
  return capture_.Start(error);
}

void TronStereoCamera::Stop() {
  capture_.Stop();
}

void TronStereoCamera::Close() {
  capture_.Close();
}

int TronStereoCamera::Next(int timeout_ms, CapturedFrame& frame, std::string& error) {
  return capture_.Next(timeout_ms, frame, error);
}

}  // namespace tron_vision
