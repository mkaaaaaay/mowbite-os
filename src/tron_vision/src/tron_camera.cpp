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
  const std::string node = FindVideoNode(config_.sysfs, config_.capture_device, config_.video_name);
  if (node.empty()) {
    error = "no single video node " + config_.video_name + " of " + config_.capture_device;
    return false;
  }
  for (const std::string& device : {subdev, node}) {
    if (!DeviceFree(config_.processes_using, device, error)) return false;
  }
  if (!sensor_.Open(subdev, error)) return false;
  if (!sensor_.SetVerticalBlanking(config_.vertical_blanking, error) || !sensor_.Set(pending_, error) ||
      !capture_.Open(node,
                     {config_.width, config_.height, PixelFormat::kNv12, config_.buffers, config_.export_buffers},
                     error) ||
      !capture_.Start(error)) {
    Stop();
    return false;
  }
  return true;
}

void TronCamera::Stop() {
  last_ = CapturedFrame{};
  capture_.Close();
  sensor_.Close();
}

int TronCamera::Next(int timeout_ms, Nv12View& frame, std::string& error) {
  last_ = CapturedFrame{};
  CapturedFrame raw;
  const int got = capture_.Next(timeout_ms, raw, error);
  if (got <= 0) return got;
  last_ = raw;
  frame.y = raw.data;
  frame.uv = raw.data + static_cast<size_t>(raw.stride) * config_.height;
  frame.width = config_.width;
  frame.height = config_.height;
  frame.stride = raw.stride;
  return 1;
}

const CapturedFrame* TronCamera::Captured() const {
  return last_.dmabuf >= 0 ? &last_ : nullptr;
}

bool TronCamera::EditFrame(const std::function<void(const Nv12Buffer&)>& edit, std::string& error) {
  const CapturedFrame* frame = Captured();
  if (frame == nullptr) {
    error = "no exported frame";
    return false;
  }
  if (!capture_.SyncForCpu(frame->index, true, error)) return false;
  // The buffers are mapped writable; nothing else reads this one until the next frame.
  Nv12Buffer buffer;
  buffer.y = const_cast<uint8_t*>(frame->data);
  buffer.uv = buffer.y + static_cast<size_t>(frame->stride) * config_.height;
  buffer.width = config_.width;
  buffer.height = config_.height;
  buffer.stride = frame->stride;
  edit(buffer);
  return capture_.SyncForCpu(frame->index, false, error);
}

bool TronCamera::SetExposure(const ExposureSetting& setting, std::string& error) {
  pending_ = setting;
  return !sensor_.IsOpen() || sensor_.Set(setting, error);
}

}  // namespace tron_vision
