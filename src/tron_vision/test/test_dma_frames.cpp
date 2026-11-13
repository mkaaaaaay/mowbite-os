#include <gtest/gtest.h>
#include <linux/dma-buf.h>

#include <cstdint>
#include <string>
#include <vector>

#include "fake_dma_heap.h"
#include "tron_vision/dma_frames.h"

namespace tron_vision {
namespace {

EncoderSettings Settings() {
  EncoderSettings s;
  s.format = PixelFormat::kNv12;
  s.width = 960;
  s.height = 540;
  s.line_bytes = 960;
  s.lines = 544;
  return s;
}

constexpr size_t kBytes = 960U * 544U * 3U / 2U;
constexpr uint64_t kStart = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
constexpr uint64_t kEnd = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;

// Three frames from the heap in turn, each written between the syncs of its dma-buf; the heap itself closes at once.
TEST(DmaFrames, FramesInTurnBetweenTheSyncs) {
  test::FakeDmaHeap heap;
  DmaFrames frames(kSystemHeap, &heap);
  std::string error;
  ASSERT_TRUE(frames.Open(Settings(), 3, error)) << error;
  EXPECT_TRUE(frames.IsOpen());
  EXPECT_EQ(heap.opened, std::vector<std::string>{"/dev/dma_heap/system"});
  EXPECT_EQ(heap.closed, std::vector<int>{test::FakeDmaHeap::kHeapFd});
  ASSERT_EQ(heap.memory.size(), 3U);
  EXPECT_EQ(heap.memory.begin()->second.size(), kBytes);
  std::vector<uint32_t> indices;
  for (int i = 0; i < 4; i++) {
    CapturedFrame frame;
    const size_t syncs_before = heap.syncs.size();
    const auto fill = [&](const Nv12Buffer& out, std::string&) {
      EXPECT_EQ(heap.syncs.size(), syncs_before + 1);
      EXPECT_EQ(out.width, 960U);
      EXPECT_EQ(out.height, 540U);
      EXPECT_EQ(out.stride, 960U);
      EXPECT_EQ(out.uv, out.y + 960 * 544);
      out.y[0] = static_cast<uint8_t>(10 + i);
      out.uv[0] = static_cast<uint8_t>(20 + i);
      return true;
    };
    ASSERT_TRUE(frames.Write(fill, frame, error)) << error;
    indices.push_back(frame.index);
    EXPECT_EQ(frame.stride, 960U);
    EXPECT_EQ(frame.bytes, kBytes);
    EXPECT_EQ(frame.dmabuf, test::FakeDmaHeap::kFirstBufferFd + static_cast<int>(frame.index));
    const std::vector<uint8_t>& memory = heap.memory[frame.dmabuf];
    EXPECT_EQ(frame.data, memory.data());
    EXPECT_EQ(memory[0], 10 + i);
    EXPECT_EQ(memory[960 * 544], 20 + i);
    ASSERT_EQ(heap.syncs.size(), syncs_before + 2);
    EXPECT_EQ(heap.syncs[syncs_before], kStart);
    EXPECT_EQ(heap.syncs[syncs_before + 1], kEnd);
  }
  EXPECT_EQ(indices, (std::vector<uint32_t>{0, 1, 2, 0}));
  frames.Close();
  EXPECT_FALSE(frames.IsOpen());
  EXPECT_EQ(heap.unmaps, 3);
  EXPECT_EQ(heap.closed.size(), 4U);
}

// A failure on the way leaves nothing open.
TEST(DmaFrames, NoneStayAfterAFailure) {
  std::string error;
  {
    test::FakeDmaHeap heap;
    heap.fail_open = true;
    DmaFrames frames(kSystemHeap, &heap);
    EXPECT_FALSE(frames.Open(Settings(), 3, error));
    EXPECT_EQ(error, "open /dev/dma_heap/system: No such file or directory");
  }
  {
    test::FakeDmaHeap heap;
    heap.fail_alloc_from = 2;
    DmaFrames frames(kSystemHeap, &heap);
    EXPECT_FALSE(frames.Open(Settings(), 3, error));
    EXPECT_EQ(error.rfind("allocate 783360 bytes from /dev/dma_heap/system: ", 0), 0U) << error;
    EXPECT_FALSE(frames.IsOpen());
    EXPECT_EQ(heap.unmaps, 2);
    EXPECT_EQ(heap.closed.size(), 3U);
  }
  {
    test::FakeDmaHeap heap;
    heap.fail_map = true;
    DmaFrames frames(kSystemHeap, &heap);
    EXPECT_FALSE(frames.Open(Settings(), 3, error));
    EXPECT_EQ(error.rfind("map dma-buf", 0), 0U) << error;
    EXPECT_EQ(heap.unmaps, 0);
    EXPECT_EQ(heap.closed.size(), 2U);
  }
  test::FakeDmaHeap heap;
  DmaFrames frames(kSystemHeap, &heap);
  EncoderSettings yuyv = Settings();
  yuyv.format = PixelFormat::kYuyv;
  EXPECT_FALSE(frames.Open(yuyv, 3, error));
  EncoderSettings short_lines = Settings();
  short_lines.lines = 536;
  EXPECT_FALSE(frames.Open(short_lines, 3, error));
  EXPECT_EQ(error, "frames need NV12 within line bytes and lines");
  EXPECT_TRUE(heap.opened.empty());
  CapturedFrame frame;
  EXPECT_FALSE(frames.Write([](const Nv12Buffer&, std::string&) { return true; }, frame, error));
  EXPECT_EQ(error, "no frames open");
}

// A fill that fails still ends the access; a failed sync gives no frame.
TEST(DmaFrames, FailedFillOrSync) {
  test::FakeDmaHeap heap;
  DmaFrames frames(kSystemHeap, &heap);
  std::string error;
  ASSERT_TRUE(frames.Open(Settings(), 2, error)) << error;
  CapturedFrame frame;
  EXPECT_FALSE(frames.Write(
      [](const Nv12Buffer&, std::string& e) {
        e = "JPEG: broken";
        return false;
      },
      frame, error));
  EXPECT_EQ(error, "JPEG: broken");
  ASSERT_EQ(heap.syncs.size(), 2U);
  EXPECT_EQ(heap.syncs[1], kEnd);
  heap.fail_sync = true;
  bool filled = false;
  EXPECT_FALSE(frames.Write(
      [&filled](const Nv12Buffer&, std::string&) {
        filled = true;
        return true;
      },
      frame, error));
  EXPECT_FALSE(filled);
  EXPECT_EQ(error, "sync dma-buf: Input/output error");
}

}  // namespace
}  // namespace tron_vision
