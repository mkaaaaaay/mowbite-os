#include "tron_vision/stereo_camera.h"

#include <filesystem>

#include "tron_vision/text_file.h"

namespace tron_vision {

std::string FindVideoNode(const std::string& sysfs_root, const std::string& device, const std::string& name) {
  std::string found;
  int matches = 0;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(sysfs_root, ec)) {
    const std::string node = entry.path().filename().string();
    if (node.rfind("video", 0) != 0 || FirstLine(sysfs_root + "/" + node + "/name") != name) continue;
    std::error_code link_error;
    const std::filesystem::path owner = std::filesystem::read_symlink(entry.path() / "device", link_error);
    if (!link_error && owner.filename() == device) {
      found = "/dev/" + node;
      matches++;
    }
  }
  return matches == 1 ? found : "";
}

bool TronStereoCamera::Open(std::string& error) {
  Close();
  const std::string node = FindVideoNode(config_.sysfs, config_.capture_device, config_.stream_name);
  if (node.empty()) {
    error = "no single video node " + config_.stream_name + " of " + config_.capture_device;
    return false;
  }
  const std::vector<int> users = config_.processes_using ? config_.processes_using(node) : std::vector<int>{};
  if (!users.empty()) {
    error = node + " is open in process " + std::to_string(users.front());
    return false;
  }
  return capture_.Open(node, {config_.width, config_.height, PixelFormat::kYuyv, config_.buffers, true}, error);
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
