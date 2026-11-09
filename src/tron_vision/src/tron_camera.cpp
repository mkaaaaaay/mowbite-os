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
      !capture_.Open(config_.video_device, config_.width, config_.height, config_.buffers, error) ||
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
  return capture_.Next(timeout_ms, frame, error);
}

bool TronCamera::SetExposure(const ExposureSetting& setting, std::string& error) {
  pending_ = setting;
  return !sensor_.IsOpen() || sensor_.Set(setting, error);
}

}  // namespace tron_vision
