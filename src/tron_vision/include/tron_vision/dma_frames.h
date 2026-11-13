#ifndef TRON_VISION_DMA_FRAMES_H
#define TRON_VISION_DMA_FRAMES_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tron_vision/frame_convert.h"
#include "tron_vision/h264_encoder.h"
#include "tron_vision/v4l2_camera.h"

namespace tron_vision {

// Cached memory of the system; DMA_BUF_IOCTL_SYNC makes a write of the CPU visible to the encoder.
constexpr char kSystemHeap[] = "/dev/dma_heap/system";

// NV12 frames in dma-bufs of a dma-buf heap, which the CPU writes and the encoder reads: a few buffers taken in turn,
// each mapped from Open() until Close(). The system calls go through a VideoDevice, a model of the heap in the tests.
class DmaFrames {
 public:
  using Fill = std::function<bool(const Nv12Buffer& frame, std::string& error)>;

  // Without a device the real system calls are used.
  explicit DmaFrames(std::string heap = kSystemHeap, VideoDevice* device = nullptr);
  ~DmaFrames();
  DmaFrames(const DmaFrames&) = delete;
  DmaFrames& operator=(const DmaFrames&) = delete;

  // `count` NV12 frames with the size, line bytes and lines of the encoder; false with the error, then none stay.
  bool Open(const EncoderSettings& settings, uint32_t count, std::string& error);
  void Close();
  bool IsOpen() const {
    return !buffers_.empty();
  }
  // Lets `fill` write the next frame in turn between the cache syncs of its dma-buf; `frame` then names that buffer
  // for the encoder, without time and sequence. False with the error of `fill` or of a sync.
  bool Write(const Fill& fill, CapturedFrame& frame, std::string& error);

 private:
  struct Buffer {
    int fd = -1;
    uint8_t* data = nullptr;
  };

  bool Sync(const Buffer& buffer, bool start, std::string& error);

  std::string heap_;
  VideoDevice own_device_;
  VideoDevice* device_;
  EncoderSettings settings_;
  size_t bytes_ = 0;
  std::vector<Buffer> buffers_;
  size_t next_ = 0;
};

}  // namespace tron_vision

#endif  // TRON_VISION_DMA_FRAMES_H
