#ifndef TRON_VISION_V4L2_CAMERA_H
#define TRON_VISION_V4L2_CAMERA_H

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "tron_vision/exposure.h"

namespace tron_vision {

// The PIDs of other processes that have a device open.
using ProcessesUsing = std::function<std::vector<int>(const std::string&)>;

// /dev/<node> of the one directory under `sysfs_root` that `match` accepts; empty if none or more than one does.
std::string UniqueNode(const std::string& sysfs_root, const std::function<bool(const std::filesystem::path&)>& match);

// False with the error while another process has `device` open; an empty `processes_using` checks nothing.
bool DeviceFree(const ProcessesUsing& processes_using, const std::string& device, std::string& error);

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
  bool IsOpen() const {
    return fd_ >= 0;
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

// System calls of a video node. Tests put a model of the driver in their place.
class VideoDevice {
 public:
  virtual ~VideoDevice() = default;
  virtual int Open(const std::string& path);
  virtual void Close(int fd);
  virtual int Ioctl(int fd, unsigned long request, void* arg);
  // nullptr if the mapping fails.
  virtual void* Map(int fd, size_t length, int64_t offset);
  virtual void Unmap(void* data, size_t length);
  // 1 if a buffer is ready, 0 on timeout or a signal, -1 on error.
  virtual int WaitReadable(int fd, int timeout_ms);
};

enum class PixelFormat {
  kNv12,
  kYuyv,
  // Compressed: a frame is as long as the driver says, the stride is 0.
  kMjpeg,
};

struct CaptureFormat {
  uint32_t width = 0;
  uint32_t height = 0;
  PixelFormat pixel = PixelFormat::kNv12;
  uint32_t buffers = 4;
  // Also hands out every buffer as a dma-buf, for a consumer that reads it without a copy.
  bool export_buffers = false;
};

// A filled buffer, valid until Release().
struct CapturedFrame {
  uint32_t index = 0;
  const uint8_t* data = nullptr;
  size_t bytes = 0;
  // Bytes per line.
  uint32_t stride = 0;
  uint32_t sequence = 0;
  // Time of capture on the monotonic clock of the driver.
  int64_t timestamp_us = 0;
  // -1 unless the buffers are exported.
  int dmabuf = -1;
};

// A control of a video node with its range and its value when read.
struct ControlRange {
  int min = 0;
  int max = 0;
  int value = 0;
};

// Capture of NV12, YUYV or MJPEG with memory-mapped buffers on one video node, single- or multi-planar as the node
// reports. Nothing but QUERYCAP, S_FMT, REQBUFS, QUERYBUF, EXPBUF, QBUF, DQBUF, STREAMON and STREAMOFF on that node,
// and QUERYCTRL, G_CTRL and S_CTRL for a caller that asks for a control.
class V4l2Capture {
 public:
  // Without a device the real system calls are used.
  explicit V4l2Capture(VideoDevice* device = nullptr);
  V4l2Capture(const V4l2Capture&) = delete;
  V4l2Capture& operator=(const V4l2Capture&) = delete;
  ~V4l2Capture();

  bool Open(const std::string& device, const CaptureFormat& format, std::string& error);
  void Close();
  bool Start(std::string& error);
  void Stop();

  // Waits up to timeout_ms for a full frame: 1 with `frame` valid until Release(), 0 on timeout, -1 on error.
  int Next(int timeout_ms, CapturedFrame& frame, std::string& error);
  void Release();

  // dma-buf of each buffer by index; empty unless exported. They stay open until Close().
  const std::vector<int>& ExportedBuffers() const {
    return exported_;
  }
  uint32_t Stride() const {
    return stride_;
  }
  // 0 for MJPEG.
  size_t FrameBytes() const;

  // Range and value of a control of the open node; false if it has none or it is disabled.
  bool QueryControl(uint32_t id, ControlRange& range, std::string& error);
  // `code` takes the errno of a write the driver refused, 0 otherwise.
  bool SetControl(uint32_t id, int value, std::string& error, int* code = nullptr);

 private:
  bool Fail(std::string message, std::string& error);
  // QBUF of a buffer the driver gave back or that was never queued.
  bool Queue(uint32_t index);

  struct Buffer {
    void* data = nullptr;
    size_t length = 0;
  };

  VideoDevice* device_;
  int fd_ = -1;
  bool mplane_ = true;
  // Buffers were requested and have to be freed.
  bool streaming_ = false;
  int held_ = -1;
  CaptureFormat format_;
  uint32_t stride_ = 0;
  std::vector<Buffer> buffers_;
  std::vector<int> exported_;
};

// A camera on a video node, for the loops.
class FrameSource {
 public:
  virtual ~FrameSource() = default;
  // Opens the video node and asks for its buffers; nothing streams yet.
  virtual bool Open(std::string& error) = 0;
  // The STREAMON of the node.
  virtual bool Start(std::string& error) = 0;
  // The STREAMOFF of the node; its buffers stay.
  virtual void Stop() = 0;
  // Stops, frees the buffers and closes the node.
  virtual void Close() = 0;
  // As V4l2Capture::Next.
  virtual int Next(int timeout_ms, CapturedFrame& frame, std::string& error) = 0;
};

}  // namespace tron_vision

#endif  // TRON_VISION_V4L2_CAMERA_H
