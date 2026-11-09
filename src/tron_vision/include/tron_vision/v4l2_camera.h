#ifndef TRON_VISION_V4L2_CAMERA_H
#define TRON_VISION_V4L2_CAMERA_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "tron_vision/exposure.h"
#include "tron_vision/frame_convert.h"

namespace tron_vision {

// /dev/v4l-subdevN whose sysfs name starts with `name_prefix`, found by reading the names under `sysfs_root`
// (normally /sys/class/video4linux) only; empty if none or more than one match. No device is opened.
std::string FindSubdev(const std::string& sysfs_root, const std::string& name_prefix);

// Name of a video node from sysfs ("rkisp_mainpath"), empty if unreadable.
std::string NodeName(const std::string& sysfs_root, const std::string& device_path);

// Exposure, gain and vertical blanking of the image sensor through its subdev. Only VIDIOC_QUERYCTRL, G_CTRL and
// S_CTRL with these three controls.
class SensorControls {
 public:
  SensorControls() = default;
  SensorControls(const SensorControls&) = delete;
  SensorControls& operator=(const SensorControls&) = delete;
  ~SensorControls();

  // Reads the ranges and the current values, which Close() writes back.
  bool Open(const std::string& subdev, std::string& error);
  void Close();

  bool Set(const ExposureSetting& setting, std::string& error);
  // 0 leaves the blanking as it is.
  bool SetVerticalBlanking(int lines, std::string& error);
  int ExposureMax() const {
    return exposure_max_;
  }
  int GainMin() const {
    return gain_min_;
  }
  int GainMax() const {
    return gain_max_;
  }

 private:
  int fd_ = -1;
  int exposure_max_ = 0;
  int gain_min_ = 0;
  int gain_max_ = 0;
  int vblank_min_ = 0;
  int vblank_max_ = 0;
  ExposureSetting initial_;
  int initial_vblank_ = 0;
};

// Multi-planar NV12 capture with memory-mapped buffers on one video node. Nothing but QUERYCAP, S_FMT, REQBUFS,
// QUERYBUF, QBUF, DQBUF, STREAMON and STREAMOFF on that node.
class V4l2Capture {
 public:
  V4l2Capture() = default;
  V4l2Capture(const V4l2Capture&) = delete;
  V4l2Capture& operator=(const V4l2Capture&) = delete;
  ~V4l2Capture();

  bool Open(const std::string& device, uint32_t width, uint32_t height, uint32_t buffers, std::string& error);
  void Close();
  bool Start(std::string& error);
  void Stop();

  // Waits up to timeout_ms for a frame: 1 with `frame` valid until Release(), 0 on timeout, -1 on error.
  int Next(int timeout_ms, Nv12View& frame, std::string& error);
  void Release();

 private:
  struct Buffer {
    void* data = nullptr;
    size_t length = 0;
  };

  int fd_ = -1;
  bool streaming_ = false;
  int held_ = -1;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t stride_ = 0;
  std::vector<Buffer> buffers_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_V4L2_CAMERA_H
