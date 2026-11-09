#ifndef TRON_VISION_FAKE_VIDEO_DEVICE_H
#define TRON_VISION_FAKE_VIDEO_DEVICE_H

#include <fcntl.h>
#include <linux/videodev2.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <set>
#include <string>
#include <vector>

#include "tron_vision/v4l2_camera.h"

namespace tron_vision::test {

// Model of a capture driver with one node and memory-mapped buffers. The test makes frames ready with Push().
class FakeVideoDevice : public VideoDevice {
 public:
  static constexpr int kNodeFd = 100;
  static constexpr int kFirstExportFd = 200;

  int Open(const std::string& path) override {
    opened.push_back(path);
    if (open_errno != 0) {
      errno = open_errno;
      return -1;
    }
    fds.insert(kNodeFd);
    return kNodeFd;
  }

  void Close(int fd) override {
    fds.erase(fd);
    if (fd == kNodeFd) streaming = false;
  }

  int Ioctl(int fd, unsigned long request, void* arg) override {
    requests.push_back(request);
    if (fd != kNodeFd) return Error(EBADF);
    switch (request) {
      case VIDIOC_QUERYCAP: {
        auto* cap = static_cast<v4l2_capability*>(arg);
        cap->capabilities = V4L2_CAP_DEVICE_CAPS;
        cap->device_caps = caps;
        return 0;
      }
      case VIDIOC_S_FMT: return SetFormat(static_cast<v4l2_format*>(arg));
      case VIDIOC_REQBUFS: {
        auto* req = static_cast<v4l2_requestbuffers*>(arg);
        if (req->type != Type()) return Error(EINVAL);
        if (req->count > max_buffers) req->count = max_buffers;
        memory.assign(req->count, std::vector<uint8_t>(buffer_bytes, 0));
        queued.clear();
        ready.clear();
        return 0;
      }
      case VIDIOC_QUERYBUF: {
        auto* buf = static_cast<v4l2_buffer*>(arg);
        if (buf->type != Type() || buf->index >= memory.size()) return Error(EINVAL);
        if (Mplane()) {
          buf->m.planes[0].length = buffer_bytes;
          buf->m.planes[0].m.mem_offset = buf->index * 0x1000000U;
        } else {
          buf->length = buffer_bytes;
          buf->m.offset = buf->index * 0x1000000U;
        }
        return 0;
      }
      case VIDIOC_EXPBUF: {
        auto* exp = static_cast<v4l2_exportbuffer*>(arg);
        if (exp->type != Type() || exp->index >= memory.size() || (exp->flags & O_CLOEXEC) == 0) return Error(EINVAL);
        exp->fd = kFirstExportFd + static_cast<int>(exp->index);
        fds.insert(exp->fd);
        return 0;
      }
      case VIDIOC_QBUF: {
        auto* buf = static_cast<v4l2_buffer*>(arg);
        if (buf->type != Type() || buf->index >= memory.size()) return Error(EINVAL);
        queued.push_back(buf->index);
        return 0;
      }
      case VIDIOC_DQBUF: {
        auto* buf = static_cast<v4l2_buffer*>(arg);
        if (!streaming || ready.empty()) return Error(EAGAIN);
        const Ready r = ready.front();
        ready.pop_front();
        buf->index = r.index;
        buf->flags = r.flags;
        buf->sequence = r.sequence;
        buf->timestamp.tv_sec = r.timestamp_us / 1000000;
        buf->timestamp.tv_usec = r.timestamp_us % 1000000;
        if (Mplane()) {
          buf->m.planes[0].bytesused = r.used;
        } else {
          buf->bytesused = r.used;
        }
        return 0;
      }
      case VIDIOC_STREAMON:
        if (*static_cast<int*>(arg) != static_cast<int>(Type())) return Error(EINVAL);
        stream_ons++;
        if (streamon_errno != 0) return Error(streamon_errno);
        streaming = true;
        return 0;
      case VIDIOC_STREAMOFF:
        stream_offs++;
        streaming = false;
        queued.clear();
        ready.clear();
        return 0;
      default: return Error(ENOTTY);
    }
  }

  void* Map(int fd, size_t length, int64_t offset) override {
    const size_t index = static_cast<size_t>(offset) / 0x1000000U;
    if (fd != kNodeFd || index >= memory.size() || length != buffer_bytes) return nullptr;
    maps++;
    return memory[index].data();
  }

  void Unmap(void*, size_t) override {
    maps--;
  }

  int WaitReadable(int, int timeout_ms) override {
    if (on_wait) on_wait(timeout_ms);
    return streaming && !ready.empty() ? 1 : 0;
  }

  // Fills the oldest queued buffer and makes it ready; without a queued buffer the frame is lost.
  bool Push(uint8_t value, uint32_t used, uint32_t flags = 0) {
    sequence++;
    timestamp_us += frame_period_us;
    if (!streaming || queued.empty()) return false;
    const uint32_t index = queued.front();
    queued.pop_front();
    std::memset(memory[index].data(), value, memory[index].size());
    ready.push_back({index, used, flags, sequence, timestamp_us});
    return true;
  }

  // Every request seen is one of `allowed`.
  bool OnlyRequests(const std::set<unsigned long>& allowed) const {
    for (unsigned long r : requests) {
      if (allowed.count(r) == 0) return false;
    }
    return true;
  }

  uint32_t caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING;
  // Bytes per line and per buffer the driver answers with.
  uint32_t stride = 2560;
  uint32_t buffer_bytes = 2560 * 480;
  // Not 0: the driver answers S_FMT with this pixel format.
  uint32_t forced_fourcc = 0;
  uint32_t max_buffers = 4;
  int open_errno = 0;
  int streamon_errno = 0;
  int64_t frame_period_us = 42000;
  std::function<void(int)> on_wait;

  std::vector<std::string> opened;
  std::vector<unsigned long> requests;
  std::set<int> fds;
  std::vector<std::vector<uint8_t>> memory;
  std::deque<uint32_t> queued;
  bool streaming = false;
  int stream_ons = 0;
  int stream_offs = 0;
  int maps = 0;
  uint32_t sequence = 0;
  int64_t timestamp_us = 1000000;

 private:
  struct Ready {
    uint32_t index;
    uint32_t used;
    uint32_t flags;
    uint32_t sequence;
    int64_t timestamp_us;
  };

  static int Error(int code) {
    errno = code;
    return -1;
  }
  bool Mplane() const {
    return (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0;
  }
  uint32_t Type() const {
    return Mplane() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
  }
  int SetFormat(v4l2_format* fmt) {
    if (fmt->type != Type()) return Error(EINVAL);
    if (Mplane()) {
      v4l2_pix_format_mplane& pix = fmt->fmt.pix_mp;
      if (forced_fourcc != 0) pix.pixelformat = forced_fourcc;
      pix.num_planes = 1;
      pix.plane_fmt[0].bytesperline = stride;
      pix.plane_fmt[0].sizeimage = buffer_bytes;
    } else {
      v4l2_pix_format& pix = fmt->fmt.pix;
      if (forced_fourcc != 0) pix.pixelformat = forced_fourcc;
      pix.bytesperline = stride;
      pix.sizeimage = buffer_bytes;
    }
    return 0;
  }

  std::deque<Ready> ready;
};

}  // namespace tron_vision::test

#endif  // TRON_VISION_FAKE_VIDEO_DEVICE_H
