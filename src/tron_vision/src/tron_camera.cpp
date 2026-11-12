#include "tron_vision/tron_camera.h"

#include "tron_vision/frame_convert.h"

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
    if (!DeviceFree(config_.processes_using, device, error)) return false;
  }
  if (!sensor_.Open(subdev, error)) return false;
  if (!sensor_.SetVerticalBlanking(config_.vertical_blanking, error) || !sensor_.Set(pending_, error) ||
      !capture_.Open(config_.video_device,
                     {config_.width, config_.height, PixelFormat::kNv12, config_.buffers, config_.export_buffers},
                     error) ||
      !capture_.Start(error)) {
    Stop();
    return false;
  }
  return true;
}

void TronCamera::Stop() {
  have_last_ = false;
  capture_.Close();
  sensor_.Close();
}

int TronCamera::Next(int timeout_ms, Nv12View& frame, std::string& error) {
  have_last_ = false;
  CapturedFrame raw;
  const int got = capture_.Next(timeout_ms, raw, error);
  if (got <= 0) return got;
  last_ = raw;
  have_last_ = true;
  frame.y = raw.data;
  frame.uv = raw.data + static_cast<size_t>(raw.stride) * config_.height;
  frame.width = config_.width;
  frame.height = config_.height;
  frame.stride = raw.stride;
  return 1;
}

const CapturedFrame* TronCamera::Captured() const {
  return have_last_ && last_.dmabuf >= 0 ? &last_ : nullptr;
}

bool TronCamera::ShiftChroma(int du, int dv, std::string& error) {
  const CapturedFrame* frame = Captured();
  if (frame == nullptr) {
    error = "no exported frame";
    return false;
  }
  if (!capture_.SyncForCpu(frame->index, true, error)) return false;
  // The buffers are mapped writable; nothing else reads this one until the next frame.
  auto* uv = const_cast<uint8_t*>(frame->data) + static_cast<size_t>(frame->stride) * config_.height;
  tron_vision::ShiftChroma(uv, config_.width, config_.height, frame->stride, du, dv);
  return capture_.SyncForCpu(frame->index, false, error);
}

bool TronCamera::SetExposure(const ExposureSetting& setting, std::string& error) {
  pending_ = setting;
  return !sensor_.IsOpen() || sensor_.Set(setting, error);
}

}  // namespace tron_vision
