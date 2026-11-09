#include "tron_vision/v4l2_camera.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <utility>

#include "tron_vision/text_file.h"

namespace tron_vision {

namespace {

int Ioctl(int fd, unsigned long request, void* arg) {
  int ret = 0;
  do {
    ret = ioctl(fd, request, arg);
  } while (ret < 0 && errno == EINTR);
  return ret;
}

std::string Errno(const std::string& what) {
  char buf[128];
  return what + ": " + strerror_r(errno, buf, sizeof(buf));
}

struct Range {
  int min = 0;
  int max = 0;
  int value = 0;
};

bool ReadControl(int fd, uint32_t id, const char* name, Range& range, std::string& error) {
  v4l2_queryctrl query{};
  query.id = id;
  if (Ioctl(fd, VIDIOC_QUERYCTRL, &query) < 0) {
    error = Errno(std::string("query ") + name);
    return false;
  }
  v4l2_control control{};
  control.id = id;
  if (Ioctl(fd, VIDIOC_G_CTRL, &control) < 0) {
    error = Errno(std::string("read ") + name);
    return false;
  }
  range.min = query.minimum;
  range.max = query.maximum;
  range.value = control.value;
  return true;
}

// One multi-planar memory-mapped buffer with its single plane; not copyable, the buffer points at the plane.
struct PlaneBuffer {
  explicit PlaneBuffer(uint32_t index) {
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    buf.length = 1;
    buf.m.planes = &plane;
  }
  PlaneBuffer(const PlaneBuffer&) = delete;
  PlaneBuffer& operator=(const PlaneBuffer&) = delete;

  v4l2_plane plane{};
  v4l2_buffer buf{};
};

bool WriteControl(int fd, uint32_t id, int value, const char* name, std::string& error) {
  v4l2_control control{};
  control.id = id;
  control.value = value;
  if (Ioctl(fd, VIDIOC_S_CTRL, &control) < 0) {
    error = Errno(std::string("set ") + name);
    return false;
  }
  return true;
}

}  // namespace

std::string FindSubdev(const std::string& sysfs_root, const std::string& name_prefix) {
  std::string found;
  int matches = 0;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(sysfs_root, ec)) {
    const std::string node = entry.path().filename().string();
    if (node.rfind("v4l-subdev", 0) == 0 &&
        FirstLine(sysfs_root + "/" + node + "/name").rfind(name_prefix, 0) == 0) {
      found = "/dev/" + node;
      matches++;
    }
  }
  return matches == 1 ? found : "";
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
    error = Errno("open " + subdev);
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

V4l2Capture::~V4l2Capture() {
  Close();
}

bool V4l2Capture::Fail(std::string message, std::string& error) {
  error = std::move(message);
  Close();
  return false;
}

size_t V4l2Capture::FrameBytes() const {
  return static_cast<size_t>(stride_) * height_ * 3 / 2;
}

bool V4l2Capture::Queue(uint32_t index) {
  PlaneBuffer b(index);
  return Ioctl(fd_, VIDIOC_QBUF, &b.buf) == 0;
}

bool V4l2Capture::Open(const std::string& device, uint32_t width, uint32_t height, uint32_t buffers,
                       std::string& error) {
  Close();
  fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    error = Errno("open " + device);
    return false;
  }
  v4l2_capability cap{};
  if (Ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) return Fail(Errno("querycap"), error);
  const uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) != 0 ? cap.device_caps : cap.capabilities;
  if ((caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) == 0 || (caps & V4L2_CAP_STREAMING) == 0) {
    return Fail(device + " is no multi-planar streaming capture node", error);
  }
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  fmt.fmt.pix_mp.width = width;
  fmt.fmt.pix_mp.height = height;
  fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
  fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
  fmt.fmt.pix_mp.num_planes = 1;
  if (Ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) return Fail(Errno("set format"), error);
  const v4l2_pix_format_mplane& pix = fmt.fmt.pix_mp;
  width_ = width;
  height_ = height;
  stride_ = pix.plane_fmt[0].bytesperline;
  if (pix.pixelformat != V4L2_PIX_FMT_NV12 || pix.width != width || pix.height != height || pix.num_planes != 1 ||
      stride_ < width || pix.plane_fmt[0].sizeimage < FrameBytes()) {
    return Fail("format not accepted: " + std::to_string(pix.width) + "x" + std::to_string(pix.height) + " planes " +
                    std::to_string(pix.num_planes) + " stride " + std::to_string(stride_),
                error);
  }

  v4l2_requestbuffers req{};
  req.count = buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  req.memory = V4L2_MEMORY_MMAP;
  if (Ioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count == 0) return Fail(Errno("request buffers"), error);
  buffers_.resize(req.count);
  for (uint32_t i = 0; i < req.count; i++) {
    PlaneBuffer b(i);
    if (Ioctl(fd_, VIDIOC_QUERYBUF, &b.buf) < 0) return Fail(Errno("query buffer"), error);
    void* data = mmap(nullptr, b.plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, b.plane.m.mem_offset);
    if (data == MAP_FAILED) return Fail(Errno("map buffer"), error);
    buffers_[i].data = data;
    buffers_[i].length = b.plane.length;
    if (b.plane.length < FrameBytes()) return Fail("buffer smaller than a frame", error);
  }
  return true;
}

void V4l2Capture::Close() {
  Stop();
  for (Buffer& b : buffers_) {
    if (b.data != nullptr) munmap(b.data, b.length);
  }
  buffers_.clear();
  if (fd_ >= 0) {
    v4l2_requestbuffers req{};
    req.count = 0;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    Ioctl(fd_, VIDIOC_REQBUFS, &req);
    close(fd_);
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
      error = Errno("queue buffer");
      return false;
    }
  }
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (Ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    error = Errno("stream on");
    return false;
  }
  streaming_ = true;
  return true;
}

void V4l2Capture::Stop() {
  held_ = -1;
  if (!streaming_) return;
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  Ioctl(fd_, VIDIOC_STREAMOFF, &type);
  streaming_ = false;
}

int V4l2Capture::Next(int timeout_ms, Nv12View& frame, std::string& error) {
  Release();
  if (!streaming_) {
    error = "not streaming";
    return -1;
  }
  pollfd pfd{fd_, POLLIN, 0};
  const int ready = poll(&pfd, 1, timeout_ms);
  if (ready < 0) {
    if (errno == EINTR) return 0;
    error = Errno("poll");
    return -1;
  }
  if (ready == 0) return 0;
  if ((pfd.revents & POLLERR) != 0) {
    error = "poll error on the video node";
    return -1;
  }
  PlaneBuffer b(0);
  if (Ioctl(fd_, VIDIOC_DQBUF, &b.buf) < 0) {
    if (errno == EAGAIN) return 0;
    error = Errno("dequeue buffer");
    return -1;
  }
  held_ = static_cast<int>(b.buf.index);
  if (b.buf.index >= buffers_.size() || (b.buf.flags & V4L2_BUF_FLAG_ERROR) != 0 ||
      (b.plane.bytesused != 0 && b.plane.bytesused < FrameBytes())) {
    Release();
    return 0;
  }
  const auto* base = static_cast<const uint8_t*>(buffers_[b.buf.index].data);
  frame.y = base;
  frame.uv = base + static_cast<size_t>(stride_) * height_;
  frame.width = width_;
  frame.height = height_;
  frame.stride = stride_;
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
