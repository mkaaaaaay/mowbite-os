#include "tron_vision/dma_frames.h"

#include <fcntl.h>
#include <linux/dma-heap.h>

#include <utility>

#include "tron_vision/errno_text.h"

namespace tron_vision {

DmaFrames::DmaFrames(std::string heap, VideoDevice* device)
    : heap_(std::move(heap)), device_(device != nullptr ? device : &own_device_) {}

DmaFrames::~DmaFrames() {
  Close();
}

bool DmaFrames::Open(const EncoderSettings& settings, uint32_t count, std::string& error) {
  Close();
  if (settings.format != PixelFormat::kNv12 || settings.line_bytes < settings.width ||
      settings.lines < settings.height || settings.width == 0 || settings.height == 0 || count == 0) {
    error = "frames need NV12 within line bytes and lines";
    return false;
  }
  settings_ = settings;
  bytes_ = static_cast<size_t>(settings.line_bytes) * settings.lines / 2 * 3;
  const int heap = device_->Open(heap_);
  if (heap < 0) {
    error = ErrnoText("open " + heap_);
    return false;
  }
  for (uint32_t i = 0; i < count; i++) {
    dma_heap_allocation_data alloc{};
    alloc.len = bytes_;
    alloc.fd_flags = O_RDWR | O_CLOEXEC;
    if (device_->Ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &alloc) < 0) {
      error = ErrnoText("allocate " + std::to_string(bytes_) + " bytes from " + heap_);
      break;
    }
    Buffer buffer;
    buffer.fd = static_cast<int>(alloc.fd);
    buffer.data = static_cast<uint8_t*>(device_->Map(buffer.fd, bytes_, 0));
    buffers_.push_back(buffer);
    if (buffer.data == nullptr) {
      error = ErrnoText("map dma-buf");
      break;
    }
  }
  device_->Close(heap);
  if (buffers_.size() < count || buffers_.back().data == nullptr) {
    Close();
    return false;
  }
  return true;
}

void DmaFrames::Close() {
  for (const Buffer& b : buffers_) {
    if (b.data != nullptr) device_->Unmap(b.data, bytes_);
    device_->Close(b.fd);
  }
  buffers_.clear();
  next_ = 0;
}

bool DmaFrames::Sync(const Buffer& buffer, bool start, std::string& error) {
  return SyncDmaBuf(*device_, buffer.fd, start, "dma-buf", error);
}

bool DmaFrames::Write(const Fill& fill, CapturedFrame& frame, std::string& error) {
  if (buffers_.empty()) {
    error = "no frames open";
    return false;
  }
  const uint32_t index = static_cast<uint32_t>(next_);
  const Buffer& buffer = buffers_[next_];
  next_ = (next_ + 1) % buffers_.size();
  if (!Sync(buffer, true, error)) return false;
  Nv12Buffer out;
  out.y = buffer.data;
  out.uv = buffer.data + static_cast<size_t>(settings_.line_bytes) * settings_.lines;
  out.width = settings_.width;
  out.height = settings_.height;
  out.stride = settings_.line_bytes;
  const bool filled = fill(out, error);
  // The end of the access also after a failed fill.
  std::string sync_error;
  if (!Sync(buffer, false, sync_error) && filled) {
    error = sync_error;
    return false;
  }
  if (!filled) return false;
  frame = CapturedFrame{};
  frame.index = index;
  frame.data = buffer.data;
  frame.bytes = bytes_;
  frame.stride = settings_.line_bytes;
  frame.dmabuf = buffer.fd;
  return true;
}

}  // namespace tron_vision
