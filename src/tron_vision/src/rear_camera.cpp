#include "tron_vision/rear_camera.h"

#include <linux/videodev2.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <filesystem>
#include <thread>

namespace tron_vision {

namespace {

// Pauses before the further tries of a control write the camera refused.
constexpr std::array<int, 2> kRetryAfterMs = {20, 40};

}  // namespace

bool UvcRearCamera::Present(std::string& error) {
  std::error_code ec;
  if (std::filesystem::exists(config_.device, ec)) return true;
  error = "no " + config_.device;
  return false;
}

bool UvcRearCamera::Open(std::string& error) {
  Close();
  return Present(error) && DeviceFree(config_.processes_using, config_.device, error) &&
         capture_.Open(config_.device, {config_.width, config_.height, PixelFormat::kMjpeg, config_.buffers, false},
                       error);
}

bool UvcRearCamera::Start(std::string& error) {
  return capture_.Start(error);
}

void UvcRearCamera::Stop() {
  capture_.Stop();
}

void UvcRearCamera::Close() {
  capture_.Stop();
  if (manual_) {
    // Automatic last: a camera in automatic mode may refuse the manual values.
    std::string ignored;
    Write(V4L2_CID_EXPOSURE_ABSOLUTE, saved_.exposure, ignored);
    if (has_gain_) Write(V4L2_CID_GAIN, saved_.gain, ignored);
    Write(V4L2_CID_EXPOSURE_AUTO, saved_mode_, ignored);
    manual_ = false;
  }
  capture_.Close();
}

int UvcRearCamera::Next(int timeout_ms, CapturedFrame& frame, std::string& error) {
  return capture_.Next(timeout_ms, frame, error);
}

bool UvcRearCamera::ManualExposure(ExposureRange& range, ExposureSetting& now, std::string& error) {
  ControlRange mode;
  ControlRange exposure;
  if (!capture_.QueryControl(V4L2_CID_EXPOSURE_AUTO, mode, error) ||
      !capture_.QueryControl(V4L2_CID_EXPOSURE_ABSOLUTE, exposure, error)) {
    return false;
  }
  // Without a gain control the gain is a range of one value.
  ControlRange gain{1, 1, 1};
  std::string no_gain;
  has_gain_ = capture_.QueryControl(V4L2_CID_GAIN, gain, no_gain);
  if (!Write(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, error)) return false;
  manual_ = true;
  saved_mode_ = mode.value;
  saved_ = {exposure.value, gain.value};
  range = {exposure.min, exposure.max, gain.min, gain.max};
  now = saved_;
  return true;
}

bool UvcRearCamera::SetExposure(const ExposureSetting& setting, std::string& error) {
  if (!manual_) {
    error = "exposure not manual";
    return false;
  }
  return Write(V4L2_CID_EXPOSURE_ABSOLUTE, setting.exposure, error) &&
         (!has_gain_ || Write(V4L2_CID_GAIN, setting.gain, error));
}

bool UvcRearCamera::Write(uint32_t id, int value, std::string& error) {
  std::string failure;
  for (size_t attempt = 1;; attempt++) {
    int code = 0;
    const bool written = capture_.SetControl(id, value, failure, &code);
    if (!written && code == EPIPE && attempt <= kRetryAfterMs.size()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kRetryAfterMs[attempt - 1]));
      continue;
    }
    if (attempt > 1 && config_.control_retried) config_.control_retried(failure, static_cast<int>(attempt), written);
    if (!written) error = failure;
    return written;
  }
}

}  // namespace tron_vision
