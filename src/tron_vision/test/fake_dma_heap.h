#ifndef TRON_VISION_FAKE_DMA_HEAP_H
#define TRON_VISION_FAKE_DMA_HEAP_H

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "tron_vision/v4l2_camera.h"

namespace tron_vision::test {

// A dma-buf heap in memory: each allocation is a new fd with zeroed memory behind its mapping; syncs are noted.
class FakeDmaHeap : public VideoDevice {
 public:
  int Open(const std::string& path) override {
    opened.push_back(path);
    if (fail_open) {
      errno = ENOENT;
      return -1;
    }
    return kHeapFd;
  }
  void Close(int fd) override {
    closed.push_back(fd);
  }
  int Ioctl(int fd, unsigned long request, void* arg) override {
    if (request == DMA_HEAP_IOCTL_ALLOC && fd == kHeapFd) {
      if (allocations >= fail_alloc_from) {
        errno = ENOMEM;
        return -1;
      }
      auto* alloc = static_cast<dma_heap_allocation_data*>(arg);
      alloc->fd = static_cast<uint32_t>(kFirstBufferFd + allocations++);
      memory[static_cast<int>(alloc->fd)].assign(alloc->len, 0);
      return 0;
    }
    if (request == DMA_BUF_IOCTL_SYNC && memory.count(fd) != 0) {
      syncs.push_back(static_cast<dma_buf_sync*>(arg)->flags);
      if (fail_sync) {
        errno = EIO;
        return -1;
      }
      return 0;
    }
    errno = ENOTTY;
    return -1;
  }
  void* Map(int fd, size_t length, int64_t) override {
    auto it = memory.find(fd);
    if (fail_map || it == memory.end() || it->second.size() < length) return nullptr;
    maps++;
    return it->second.data();
  }
  void Unmap(void*, size_t) override {
    unmaps++;
  }

  static constexpr int kHeapFd = 70;
  static constexpr int kFirstBufferFd = 80;
  bool fail_open = false;
  bool fail_map = false;
  bool fail_sync = false;
  int fail_alloc_from = 1 << 30;
  int allocations = 0;
  int maps = 0;
  int unmaps = 0;
  std::vector<std::string> opened;
  std::vector<int> closed;
  std::vector<uint64_t> syncs;
  std::map<int, std::vector<uint8_t>> memory;
};

}  // namespace tron_vision::test

#endif  // TRON_VISION_FAKE_DMA_HEAP_H
