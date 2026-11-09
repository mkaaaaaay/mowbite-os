#ifndef TRON_VISION_NPU_ABI_H
#define TRON_VISION_NPU_ABI_H

#include <cstdint>

// The few types and calls of the NPU runtime library (librknnrt 2.2.0 on aarch64) that this package uses, declared
// here from their binary layout; the library itself is loaded at run time and never part of the build.
namespace tron_vision::abi {

using Context = uint64_t;

constexpr uint32_t kMaxDims = 16;
constexpr uint32_t kMaxName = 256;

enum Query : int {
  kQueryInOutNum = 0,
  kQueryInputAttr = 1,
  kQueryOutputAttr = 2,
  kQuerySdkVersion = 5,
};

enum TensorType : int {
  kFloat32 = 0,
  kFloat16 = 1,
  kInt8 = 2,
  kUint8 = 3,
};

enum TensorFormat : int {
  kNchw = 0,
  kNhwc = 1,
};

// Quantization: real value = (q - zp) * scale.
enum QuantType : int {
  kQuantNone = 0,
  kQuantDfp = 1,
  kQuantAffine = 2,
};

// Core mask value for core 0 only.
constexpr int kCore0 = 1;

struct InOutNum {
  uint32_t inputs;
  uint32_t outputs;
};

struct TensorAttr {
  uint32_t index;
  uint32_t dim_count;
  uint32_t dims[kMaxDims];
  char name[kMaxName];
  uint32_t elements;
  uint32_t bytes;
  int format;
  int type;
  int quant;
  int8_t fixed_point;
  int32_t zero_point;
  float scale;
  uint32_t width_stride;
  uint32_t bytes_with_stride;
  uint8_t raw;
  uint32_t height_stride;
};

struct SdkVersion {
  char api[256];
  char driver[256];
};

// raw = 1 hands the buffer to the NPU as it is, 0 lets the runtime convert type and layout.
struct Input {
  uint32_t index;
  void* data;
  uint32_t bytes;
  uint8_t raw;
  int type;
  int format;
};

struct Output {
  uint8_t as_float;
  uint8_t caller_buffer;
  uint32_t index;
  void* data;
  uint32_t bytes;
};

using InitFn = int (*)(Context*, void* model, uint32_t size, uint32_t flag, void* extend);
using QueryFn = int (*)(Context, int query, void* info, uint32_t size);
using SetCoreMaskFn = int (*)(Context, int mask);
using InputsSetFn = int (*)(Context, uint32_t n, Input* inputs);
using RunFn = int (*)(Context, void* extend);
using OutputsGetFn = int (*)(Context, uint32_t n, Output* outputs, void* extend);
using OutputsReleaseFn = int (*)(Context, uint32_t n, Output* outputs);
using DestroyFn = int (*)(Context);

}  // namespace tron_vision::abi

#endif  // TRON_VISION_NPU_ABI_H
