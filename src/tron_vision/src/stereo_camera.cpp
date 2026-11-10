#include "tron_vision/stereo_camera.h"

#include <filesystem>

#include "tron_vision/text_file.h"

namespace tron_vision {

std::string FindVideoNode(const std::string& sysfs_root, const std::string& device, const std::string& name) {
  return UniqueNode(sysfs_root, [&device, &name](const std::filesystem::path& dir) {
    if (dir.filename().string().rfind("video", 0) != 0 || FirstLine((dir / "name").string()) != name) return false;
    std::error_code link_error;
    const std::filesystem::path owner = std::filesystem::read_symlink(dir / "device", link_error);
    return !link_error && owner.filename() == device;
  });
}

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
