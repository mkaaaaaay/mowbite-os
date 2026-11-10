#include "tron_vision/v4l2_camera.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <filesystem>
#include <utility>

#include "tron_vision/errno_text.h"
#include "tron_vision/text_file.h"

namespace tron_vision {

namespace {

int RetryIoctl(int fd, unsigned long request, void* arg) {
  int ret = 0;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret < 0 && errno == EINTR);
  return ret;
}

struct Range {
  int min = 0;
  int max = 0;
  int value = 0;
};

bool ReadControl(int fd, uint32_t id, const char* name, Range& range, std::string& error) {
  v4l2_queryctrl query{};
  query.id = id;
  if (RetryIoctl(fd, VIDIOC_QUERYCTRL, &query) < 0) {
    error = ErrnoText(std::string("query ") + name);
    return false;
  }
  v4l2_control control{};
  control.id = id;
  if (RetryIoctl(fd, VIDIOC_G_CTRL, &control) < 0) {
    error = ErrnoText(std::string("read ") + name);
    return false;
  }
  range.min = query.minimum;
  range.max = query.maximum;
  range.value = control.value;
  return true;
}

uint32_t BufferType(bool mplane) {
  return mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

// A memory-mapped buffer of either buffer type with its single plane; not copyable, the buffer points at the plane.
struct BufferArg {
  BufferArg(bool mplane, uint32_t index) : mplane(mplane) {
    buf.type = BufferType(mplane);
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    if (mplane) {
      buf.length = 1;
      buf.m.planes = &plane;
    }
  }
  BufferArg(const BufferArg&) = delete;
  BufferArg& operator=(const BufferArg&) = delete;

  uint32_t Length() const {
    return mplane ? plane.length : buf.length;
  }
  uint32_t Offset() const {
    return mplane ? plane.m.mem_offset : buf.m.offset;
  }
  uint32_t Used() const {
    return mplane ? plane.bytesused : buf.bytesused;
  }

  bool mplane;
  v4l2_plane plane{};
  v4l2_buffer buf{};
};

uint32_t FourCc(PixelFormat pixel) {
  return pixel == PixelFormat::kYuyv ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_NV12;
}

VideoDevice& SystemDevice() {
  static VideoDevice device;
  return device;
}

bool WriteControl(int fd, uint32_t id, int value, const char* name, std::string& error) {
  v4l2_control control{};
  control.id = id;
  control.value = value;
  if (RetryIoctl(fd, VIDIOC_S_CTRL, &control) < 0) {
    error = ErrnoText(std::string("set ") + name);
    return false;
  }
  return true;
}

}  // namespace

std::string UniqueNode(const std::string& sysfs_root, const std::function<bool(const std::filesystem::path&)>& match) {
  std::string found;
  int matches = 0;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(sysfs_root, ec)) {
    if (!match(entry.path())) continue;
    found = "/dev/" + entry.path().filename().string();
    matches++;
  }
  return matches == 1 ? found : "";
}

bool DeviceFree(const ProcessesUsing& processes_using, const std::string& device, std::string& error) {
  const std::vector<int> users = processes_using ? processes_using(device) : std::vector<int>{};
  if (users.empty()) return true;
  error = device + " is open in process " + std::to_string(users.front());
  return false;
}

std::string FindSubdev(const std::string& sysfs_root, const std::string& name_prefix) {
  return UniqueNode(sysfs_root, [&name_prefix](const std::filesystem::path& dir) {
    return dir.filename().string().rfind("v4l-subdev", 0) == 0 &&
           FirstLine((dir / "name").string()).rfind(name_prefix, 0) == 0;
  });
}

std::string NodeName(const std::string& sysfs_root, const std::string& device_path) {
  const size_t slash = device_path.rfind('/');
  return FirstLine(sysfs_root + "/" + device_path.substr(slash + 1) + "/name");
}

SensorControls::~SensorControls() {
  Close();
}

bool SensorControls::Open(const std::string& subdev, std::string& error) {
  Close();
  fd_ = open(subdev.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    error = ErrnoText("open " + subdev);
    return false;
  }
  Range exposure;
  Range gain;
  Range vblank;
  if (!ReadControl(fd_, V4L2_CID_EXPOSURE, "exposure", exposure, error) ||
      !ReadControl(fd_, V4L2_CID_ANALOGUE_GAIN, "analogue_gain", gain, error) ||
      !ReadControl(fd_, V4L2_CID_VBLANK, "vertical_blanking", vblank, error)) {
    close(fd_);
    fd_ = -1;
    return false;
  }
  exposure_max_ = exposure.max;
  gain_min_ = gain.min;
  gain_max_ = gain.max;
  vblank_min_ = vblank.min;
  vblank_max_ = vblank.max;
  initial_.exposure = exposure.value;
  initial_.gain = gain.value;
  initial_vblank_ = vblank.value;
  return true;
}

void SensorControls::Close() {
  if (fd_ < 0) return;
  // The sensor keeps its values across streams; the blanking first, it bounds the exposure.
  std::string ignored;
  WriteControl(fd_, V4L2_CID_VBLANK, initial_vblank_, "vertical_blanking", ignored);
  WriteControl(fd_, V4L2_CID_EXPOSURE, initial_.exposure, "exposure", ignored);
  WriteControl(fd_, V4L2_CID_ANALOGUE_GAIN, initial_.gain, "analogue_gain", ignored);
  close(fd_);
  fd_ = -1;
}

bool SensorControls::Set(const ExposureSetting& setting, std::string& error) {
  if (fd_ < 0) {
    error = "sensor not open";
    return false;
  }
  return WriteControl(fd_, V4L2_CID_EXPOSURE, std::clamp(setting.exposure, 1, exposure_max_), "exposure", error) &&
         WriteControl(fd_, V4L2_CID_ANALOGUE_GAIN, std::clamp(setting.gain, gain_min_, gain_max_), "analogue_gain",
                      error);
}

bool SensorControls::SetVerticalBlanking(int lines, std::string& error) {
  if (lines == 0) return true;
  if (fd_ < 0) {
    error = "sensor not open";
    return false;
  }
  return WriteControl(fd_, V4L2_CID_VBLANK, std::clamp(lines, vblank_min_, vblank_max_), "vertical_blanking", error);
}

int VideoDevice::Open(const std::string& path) {
  return open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
}

void VideoDevice::Close(int fd) {
  close(fd);
}

int VideoDevice::Ioctl(int fd, unsigned long request, void* arg) {
  return RetryIoctl(fd, request, arg);
}

void* VideoDevice::Map(int fd, size_t length, int64_t offset) {
  void* data = mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, static_cast<off_t>(offset));
  return data == MAP_FAILED ? nullptr : data;
}

void VideoDevice::Unmap(void* data, size_t length) {
  munmap(data, length);
}

int VideoDevice::WaitReadable(int fd, int timeout_ms) {
  pollfd pfd{fd, POLLIN, 0};
  const int ready = poll(&pfd, 1, timeout_ms);
  if (ready < 0) return errno == EINTR ? 0 : -1;
  if (ready == 0) return 0;
  if ((pfd.revents & POLLERR) != 0) {
    errno = EIO;
    return -1;
  }
  return 1;
}

V4l2Capture::V4l2Capture(VideoDevice* device) : device_(device != nullptr ? device : &SystemDevice()) {}

V4l2Capture::~V4l2Capture() {
  Close();
}

bool V4l2Capture::Fail(std::string message, std::string& error) {
  error = std::move(message);
  Close();
  return false;
}

size_t V4l2Capture::FrameBytes() const {
  const size_t luma = static_cast<size_t>(stride_) * format_.height;
  return format_.pixel == PixelFormat::kYuyv ? luma : luma * 3 / 2;
}

bool V4l2Capture::Queue(uint32_t index) {
  BufferArg b(mplane_, index);
  return device_->Ioctl(fd_, VIDIOC_QBUF, &b.buf) == 0;
}

bool V4l2Capture::Open(const std::string& device, const CaptureFormat& format, std::string& error) {
  Close();
  fd_ = device_->Open(device);
  if (fd_ < 0) {
    error = ErrnoText("open " + device);
    return false;
  }
  format_ = format;
  v4l2_capability cap{};
  if (device_->Ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) return Fail(ErrnoText("querycap"), error);
  const uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) != 0 ? cap.device_caps : cap.capabilities;
  mplane_ = (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0;
  if ((!mplane_ && (caps & V4L2_CAP_VIDEO_CAPTURE) == 0) || (caps & V4L2_CAP_STREAMING) == 0) {
    return Fail(device + " is no streaming capture node", error);
  }
  // Bytes per pixel in a line of the first plane.
  const uint32_t pixel_bytes = format.pixel == PixelFormat::kYuyv ? 2 : 1;
  v4l2_format fmt{};
  fmt.type = BufferType(mplane_);
  // What the driver made of the request.
  v4l2_pix_format got{};
  uint32_t planes = 1;
  if (mplane_) {
    v4l2_pix_format_mplane& pix = fmt.fmt.pix_mp;
    pix.width = format.width;
    pix.height = format.height;
    pix.pixelformat = FourCc(format.pixel);
    pix.field = V4L2_FIELD_NONE;
    pix.num_planes = 1;
    if (device_->Ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) return Fail(ErrnoText("set format"), error);
    planes = pix.num_planes;
    got.width = pix.width;
    got.height = pix.height;
    got.pixelformat = pix.pixelformat;
    got.bytesperline = pix.plane_fmt[0].bytesperline;
    got.sizeimage = pix.plane_fmt[0].sizeimage;
  } else {
    v4l2_pix_format& pix = fmt.fmt.pix;
    pix.width = format.width;
    pix.height = format.height;
    pix.pixelformat = FourCc(format.pixel);
    pix.field = V4L2_FIELD_NONE;
    if (device_->Ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) return Fail(ErrnoText("set format"), error);
    got = pix;
  }
  stride_ = got.bytesperline;
  if (got.pixelformat != FourCc(format.pixel) || got.width != format.width || got.height != format.height ||
      planes != 1 || stride_ < format.width * pixel_bytes || got.sizeimage < FrameBytes()) {
    return Fail("format not accepted: " + std::to_string(got.width) + "x" + std::to_string(got.height) + " planes " +
                    std::to_string(planes) + " stride " + std::to_string(stride_),
                error);
  }

  v4l2_requestbuffers req{};
  req.count = format.buffers;
  req.type = BufferType(mplane_);
  req.memory = V4L2_MEMORY_MMAP;
  if (device_->Ioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count == 0) return Fail(ErrnoText("request buffers"), error);
  buffers_.resize(req.count);
  for (uint32_t i = 0; i < req.count; i++) {
    BufferArg b(mplane_, i);
    if (device_->Ioctl(fd_, VIDIOC_QUERYBUF, &b.buf) < 0) return Fail(ErrnoText("query buffer"), error);
    void* data = device_->Map(fd_, b.Length(), b.Offset());
    if (data == nullptr) return Fail(ErrnoText("map buffer"), error);
    buffers_[i].data = data;
    buffers_[i].length = b.Length();
    if (b.Length() < FrameBytes()) return Fail("buffer smaller than a frame", error);
    if (format.export_buffers) {
      v4l2_exportbuffer exp{};
      exp.type = BufferType(mplane_);
      exp.index = i;
      exp.flags = O_CLOEXEC;
      if (device_->Ioctl(fd_, VIDIOC_EXPBUF, &exp) < 0) return Fail(ErrnoText("export buffer"), error);
      exported_.push_back(exp.fd);
    }
  }
  return true;
}

void V4l2Capture::Close() {
  Stop();
  // Buffers come only from a successful request.
  const bool requested = !buffers_.empty();
  for (Buffer& b : buffers_) {
    if (b.data != nullptr) device_->Unmap(b.data, b.length);
  }
  buffers_.clear();
  for (int fd : exported_) device_->Close(fd);
  exported_.clear();
  if (requested) {
    v4l2_requestbuffers req{};
    req.count = 0;
    req.type = BufferType(mplane_);
    req.memory = V4L2_MEMORY_MMAP;
    device_->Ioctl(fd_, VIDIOC_REQBUFS, &req);
  }
  if (fd_ >= 0) {
    device_->Close(fd_);
    fd_ = -1;
  }
}

bool V4l2Capture::Start(std::string& error) {
  if (fd_ < 0 || buffers_.empty()) {
    error = "camera not open";
    return false;
  }
  if (streaming_) return true;
  for (uint32_t i = 0; i < buffers_.size(); i++) {
    if (!Queue(i)) {
      error = ErrnoText("queue buffer");
      return false;
    }
  }
  int type = static_cast<int>(BufferType(mplane_));
  if (device_->Ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    error = ErrnoText("stream on");
    return false;
  }
  streaming_ = true;
  return true;
}

void V4l2Capture::Stop() {
  held_ = -1;
  if (!streaming_) return;
  int type = static_cast<int>(BufferType(mplane_));
  device_->Ioctl(fd_, VIDIOC_STREAMOFF, &type);
  streaming_ = false;
}

int V4l2Capture::Next(int timeout_ms, CapturedFrame& frame, std::string& error) {
  Release();
  if (!streaming_) {
    error = "not streaming";
    return -1;
  }
  const int ready = device_->WaitReadable(fd_, timeout_ms);
  if (ready < 0) {
    error = ErrnoText("wait for a frame");
    return -1;
  }
  if (ready == 0) return 0;
  BufferArg b(mplane_, 0);
  if (device_->Ioctl(fd_, VIDIOC_DQBUF, &b.buf) < 0) {
    if (errno == EAGAIN) return 0;
    error = ErrnoText("dequeue buffer");
    return -1;
  }
  held_ = static_cast<int>(b.buf.index);
  if (b.buf.index >= buffers_.size() || (b.buf.flags & V4L2_BUF_FLAG_ERROR) != 0 ||
      (b.Used() != 0 && b.Used() < FrameBytes())) {
    Release();
    return 0;
  }
  frame.index = b.buf.index;
  frame.data = static_cast<const uint8_t*>(buffers_[b.buf.index].data);
  frame.bytes = b.Used() != 0 ? b.Used() : FrameBytes();
  frame.stride = stride_;
  frame.sequence = b.buf.sequence;
  frame.timestamp_us = static_cast<int64_t>(b.buf.timestamp.tv_sec) * 1000000 + b.buf.timestamp.tv_usec;
  frame.dmabuf = exported_.empty() ? -1 : exported_[b.buf.index];
  return 1;
}

void V4l2Capture::Release() {
  if (held_ < 0 || !streaming_) {
    held_ = -1;
    return;
  }
  Queue(static_cast<uint32_t>(held_));
  held_ = -1;
}

}  // namespace tron_vision
