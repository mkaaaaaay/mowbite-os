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
  uint32_t n_input;
  uint32_t n_output;
};

struct TensorAttr {
  uint32_t index;
  uint32_t n_dims;
  uint32_t dims[kMaxDims];
  char name[kMaxName];
  uint32_t n_elems;
  uint32_t size;
  int fmt;
  int type;
  int qnt_type;
  int8_t fl;
  int32_t zp;
  float scale;
  uint32_t w_stride;
  uint32_t size_with_stride;
  uint8_t pass_through;
  uint32_t h_stride;
};

struct SdkVersion {
  char api[256];
  char driver[256];
};

struct Input {
  uint32_t index;
  void* buf;
  uint32_t size;
  uint8_t pass_through;
  int type;
  int fmt;
};

struct Output {
  uint8_t want_float;
  uint8_t is_prealloc;
  uint32_t index;
  void* buf;
  uint32_t size;
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
