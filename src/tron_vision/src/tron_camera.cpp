#include "tron_vision/tron_camera.h"

namespace tron_vision {

bool TronCamera::Start(std::string& error) {
  Stop();
  const std::string subdev = FindSubdev(config_.sysfs, config_.sensor_name);
  if (subdev.empty()) {
    error = "no single subdev named " + config_.sensor_name;
    return false;
  }
  const std::string name = NodeName(config_.sysfs, config_.video_device);
  if (name != config_.video_name) {
    error = config_.video_device + " is \"" + name + "\", not " + config_.video_name;
    return false;
  }
  for (const std::string& device : {subdev, config_.video_device}) {
    const std::vector<int> users = config_.processes_using ? config_.processes_using(device) : std::vector<int>{};
    if (!users.empty()) {
      error = device + " is open in process " + std::to_string(users.front());
      return false;
    }
  }
  if (!sensor_.Open(subdev, error)) return false;
  if (!sensor_.SetVerticalBlanking(config_.vertical_blanking, error) || !sensor_.Set(pending_, error) ||
      !capture_.Open(config_.video_device, {config_.width, config_.height, PixelFormat::kNv12, config_.buffers, false},
                     error) ||
      !capture_.Start(error)) {
    Stop();
    return false;
  }
  return true;
}

void TronCamera::Stop() {
  capture_.Close();
  sensor_.Close();
}

int TronCamera::Next(int timeout_ms, Nv12View& frame, std::string& error) {
  CapturedFrame raw;
  const int got = capture_.Next(timeout_ms, raw, error);
  if (got <= 0) return got;
  frame.y = raw.data;
  frame.uv = raw.data + static_cast<size_t>(raw.stride) * config_.height;
  frame.width = config_.width;
  frame.height = config_.height;
  frame.stride = raw.stride;
  return 1;
}

bool TronCamera::SetExposure(const ExposureSetting& setting, std::string& error) {
  pending_ = setting;
  return !sensor_.IsOpen() || sensor_.Set(setting, error);
}

}  // namespace tron_vision
