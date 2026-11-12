#ifndef TRON_VISION_MPP_ABI_H
#define TRON_VISION_MPP_ABI_H

#include <cstddef>
#include <cstdint>

// The few values, types and calls of the Rockchip media library (librockchip_mpp 1.1.0) that the H.264 encoder uses,
// declared here from its public interface; the library itself is loaded at run time and never part of the build.
// test/check_mpp_abi.cpp compares every line with the headers of the library.
namespace tron_vision::mpp {

// Context, encoder settings, frame, packet and buffer are opaque to the caller.
using Handle = void*;

constexpr int kContextEncoder = 1;
constexpr int kCodingAvc = 7;
// Planar 4:2:0, the luma plane followed by one plane of interleaved U and V (NV12).
constexpr int kFormatNv12 = 0;
// Packed 4:2:2, two bytes per pixel in the order Y U Y V.
constexpr int kFormatYuyv = 8;
// A dma-buf that the caller owns.
constexpr int kBufferExternalDma = 2;
constexpr int kRateControlCbr = 1;
// SPS and PPS in front of every IDR picture.
constexpr int kHeaderEachIdr = 1;
constexpr int kSeiOff = 0;
// Output wait that blocks until the packet is there.
constexpr int64_t kWaitForever = -1;

enum Command : int {
  kSetOutputTimeout = 0x200007,
  kEncSetConfig = 0x320001,
  kEncGetConfig = 0x320002,
  kEncGetHeader = 0x32000D,
  kEncSetSei = 0x32000F,
  kEncSetHeaderMode = 0x328001,
};

// Call table that comes with a context; only the three named calls are used.
struct Api {
  uint32_t size;
  uint32_t version;
  void* decode_calls[4];
  int (*encode_put_frame)(Handle context, Handle frame);
  int (*encode_get_packet)(Handle context, Handle* packet);
  void* other_calls[7];
  int (*control)(Handle context, int command, void* param);
  uint32_t reserved[16];
};

struct BufferInfo {
  int type;
  size_t size;
  void* address;
  void* handle;
  int fd;
  int index;
};

using CreateFn = int (*)(Handle* context, Api** api);
using InitFn = int (*)(Handle context, int type, int coding);
using DestroyFn = int (*)(Handle context);
using ConfigInitFn = int (*)(Handle* config);
using ConfigDeinitFn = int (*)(Handle config);
using ConfigSetS32Fn = int (*)(Handle config, const char* name, int32_t value);
using ConfigSetU32Fn = int (*)(Handle config, const char* name, uint32_t value);
using FrameInitFn = int (*)(Handle* frame);
using FrameDeinitFn = int (*)(Handle* frame);
using FrameSetU32Fn = void (*)(Handle frame, uint32_t value);
using FrameSetFormatFn = void (*)(Handle frame, int format);
using FrameSetBufferFn = void (*)(Handle frame, Handle buffer);
// The library takes the time as long long.
using FrameSetPtsFn = void (*)(Handle frame, long long pts);
using BufferImportFn = int (*)(Handle group, BufferInfo* info, Handle* buffer, const char* tag, const char* caller);
using BufferPutFn = int (*)(Handle buffer, const char* caller);
using PacketInitFn = int (*)(Handle* packet, void* data, size_t size);
using PacketDeinitFn = int (*)(Handle* packet);
using PacketSetLengthFn = void (*)(Handle packet, size_t length);
using PacketGetPosFn = void* (*)(Handle packet);
using PacketGetLengthFn = size_t (*)(Handle packet);

}  // namespace tron_vision::mpp

#endif  // TRON_VISION_MPP_ABI_H
