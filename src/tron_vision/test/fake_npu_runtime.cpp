// Stand-in for the NPU runtime library in tests, loaded with dlopen like the real one. The "model" is a text file:
//   classes <n>             class channels per scale (default 22)
//   input <h> <w> <c>       input dims (default 640 480 3)
//   outputs <n>             number of outputs reported (default 9)
//   hit <class> <score> <stride> <col> <row> <l> <t> <r> <b>
//                           one detection: cell of that scale, class score, side distances in cells
//   fail_init               rknn_init fails
//   fail_run                rknn_run fails
//   run_ms <ms>             rknn_run sleeps that long
// The last input is kept for checks through fake_last_input().
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "tron_vision/npu_abi.h"

using tron_vision::abi::Context;
using tron_vision::abi::Input;
using tron_vision::abi::InOutNum;
using tron_vision::abi::Output;
using tron_vision::abi::SdkVersion;
using tron_vision::abi::TensorAttr;

namespace {

struct Hit {
  uint32_t cls;
  float score;
  uint32_t stride;
  uint32_t col;
  uint32_t row;
  float side[4];
};

struct Fake {
  uint32_t classes = 22;
  uint32_t in_dims[3] = {640, 480, 3};
  uint32_t outputs = 9;
  bool fail_run = false;
  int run_ms = 0;
  std::vector<Hit> hits;
  std::vector<std::vector<int8_t>> data;
  std::vector<uint8_t> last_input;
  int core_mask = -1;
};

std::map<Context, Fake> fakes;
Context next_context = 1;

constexpr uint32_t kStrides[] = {8, 16, 32};
constexpr int32_t kBoxZp = 0;
constexpr float kBoxScale = 0.125F;
constexpr int32_t kScoreZp = -128;
constexpr float kScoreScale = 1.0F / 255.0F;

TensorAttr Attr(const Fake& f, uint32_t index) {
  TensorAttr a{};
  a.index = index;
  a.n_dims = 4;
  const uint32_t branch = index / 3;
  const uint32_t kind = index % 3;
  const uint32_t stride = kStrides[branch % 3];
  const uint32_t channels = kind == 0 ? 64 : kind == 1 ? f.classes : 1;
  const uint32_t dims[4] = {1, channels, f.in_dims[0] / stride, f.in_dims[1] / stride};
  std::memcpy(a.dims, dims, sizeof(dims));
  std::snprintf(a.name, sizeof(a.name), "out%u", index);
  a.n_elems = dims[1] * dims[2] * dims[3];
  a.size = a.n_elems;
  a.fmt = tron_vision::abi::kNchw;
  a.type = tron_vision::abi::kInt8;
  a.qnt_type = tron_vision::abi::kQuantAffine;
  a.zp = kind == 0 ? kBoxZp : kScoreZp;
  a.scale = kind == 0 ? kBoxScale : kScoreScale;
  return a;
}

int8_t Quant(float v, int32_t zp, float scale) {
  const long q = std::lround(v / scale) + zp;
  return static_cast<int8_t>(q < -128 ? -128 : q > 127 ? 127 : q);
}

// Box channels hold a distribution that peaks sharply at the side distance (whole cells).
void Fill(Fake& f) {
  f.data.clear();
  for (uint32_t i = 0; i < f.outputs; i++) {
    const TensorAttr a = Attr(f, i);
    f.data.emplace_back(a.n_elems, static_cast<int8_t>(i % 3 == 0 ? Quant(-8.0F, kBoxZp, kBoxScale) : -128));
  }
  for (const Hit& h : f.hits) {
    uint32_t b = 0;
    while (b < 3 && kStrides[b] != h.stride) b++;
    if (b == 3 || 3 * b + 2 >= f.outputs) continue;
    const uint32_t width = f.in_dims[1] / h.stride;
    const uint32_t plane = width * (f.in_dims[0] / h.stride);
    const uint32_t cell = h.row * width + h.col;
    f.data[3 * b + 1][h.cls * plane + cell] = Quant(h.score, kScoreZp, kScoreScale);
    f.data[3 * b + 2][cell] = Quant(h.score, kScoreZp, kScoreScale);
    for (uint32_t s = 0; s < 4; s++) {
      const auto bin = static_cast<uint32_t>(std::lround(h.side[s]));
      f.data[3 * b][(s * 16 + bin) * plane + cell] = Quant(8.0F, kBoxZp, kBoxScale);
    }
  }
}

}  // namespace

extern "C" {

int rknn_init(Context* context, void* model, uint32_t size, uint32_t, void*) {
  Fake f;
  std::istringstream in(std::string(static_cast<const char*>(model), size));
  std::string word;
  while (in >> word) {
    if (word == "classes") {
      in >> f.classes;
    } else if (word == "input") {
      in >> f.in_dims[0] >> f.in_dims[1] >> f.in_dims[2];
    } else if (word == "outputs") {
      in >> f.outputs;
    } else if (word == "hit") {
      Hit h{};
      in >> h.cls >> h.score >> h.stride >> h.col >> h.row >> h.side[0] >> h.side[1] >> h.side[2] >> h.side[3];
      f.hits.push_back(h);
    } else if (word == "fail_init") {
      return -1;
    } else if (word == "fail_run") {
      f.fail_run = true;
    } else if (word == "run_ms") {
      in >> f.run_ms;
    }
  }
  Fill(f);
  *context = next_context++;
  fakes[*context] = std::move(f);
  return 0;
}

int rknn_query(Context context, int query, void* info, uint32_t size) {
  auto it = fakes.find(context);
  if (it == fakes.end()) return -1;
  Fake& f = it->second;
  switch (query) {
    case tron_vision::abi::kQueryInOutNum: {
      if (size < sizeof(InOutNum)) return -1;
      InOutNum n{1, f.outputs};
      std::memcpy(info, &n, sizeof(n));
      return 0;
    }
    case tron_vision::abi::kQueryInputAttr: {
      if (size < sizeof(TensorAttr)) return -1;
      TensorAttr a{};
      a.n_dims = 4;
      const uint32_t dims[4] = {1, f.in_dims[0], f.in_dims[1], f.in_dims[2]};
      std::memcpy(a.dims, dims, sizeof(dims));
      std::snprintf(a.name, sizeof(a.name), "images");
      a.n_elems = f.in_dims[0] * f.in_dims[1] * f.in_dims[2];
      a.size = a.n_elems;
      a.fmt = tron_vision::abi::kNhwc;
      a.type = tron_vision::abi::kInt8;
      a.qnt_type = tron_vision::abi::kQuantAffine;
      a.zp = -128;
      a.scale = 1.0F / 255.0F;
      std::memcpy(info, &a, sizeof(a));
      return 0;
    }
    case tron_vision::abi::kQueryOutputAttr: {
      if (size < sizeof(TensorAttr)) return -1;
      const uint32_t index = static_cast<TensorAttr*>(info)->index;
      if (index >= f.outputs) return -1;
      const TensorAttr a = Attr(f, index);
      std::memcpy(info, &a, sizeof(a));
      return 0;
    }
    case tron_vision::abi::kQuerySdkVersion: {
      if (size < sizeof(SdkVersion)) return -1;
      SdkVersion v{};
      std::snprintf(v.api, sizeof(v.api), "fake 0");
      std::memcpy(info, &v, sizeof(v));
      return 0;
    }
    default:
      return -1;
  }
}

int rknn_set_core_mask(Context context, int mask) {
  auto it = fakes.find(context);
  if (it == fakes.end()) return -1;
  it->second.core_mask = mask;
  return 0;
}

int rknn_inputs_set(Context context, uint32_t n, Input* inputs) {
  auto it = fakes.find(context);
  if (it == fakes.end() || n != 1) return -1;
  const Input& in = inputs[0];
  if (in.type != tron_vision::abi::kUint8 || in.fmt != tron_vision::abi::kNhwc || in.pass_through != 0) return -1;
  const auto* p = static_cast<const uint8_t*>(in.buf);
  it->second.last_input.assign(p, p + in.size);
  return 0;
}

int rknn_run(Context context, void*) {
  auto it = fakes.find(context);
  if (it == fakes.end() || it->second.fail_run) return -1;
  if (it->second.run_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(it->second.run_ms));
  return 0;
}

int rknn_outputs_get(Context context, uint32_t n, Output* outputs, void*) {
  auto it = fakes.find(context);
  if (it == fakes.end() || n != it->second.outputs) return -1;
  for (uint32_t i = 0; i < n; i++) {
    if (outputs[i].want_float != 0 || outputs[i].is_prealloc != 0) return -1;
    outputs[i].buf = it->second.data[outputs[i].index].data();
    outputs[i].size = static_cast<uint32_t>(it->second.data[outputs[i].index].size());
  }
  return 0;
}

int rknn_outputs_release(Context context, uint32_t, Output*) {
  return fakes.count(context) != 0 ? 0 : -1;
}

int rknn_destroy(Context context) {
  return fakes.erase(context) != 0 ? 0 : -1;
}

// Test hooks, not part of the real library.
int fake_core_mask(Context context) {
  auto it = fakes.find(context);
  return it == fakes.end() ? -1 : it->second.core_mask;
}

size_t fake_open_contexts() {
  return fakes.size();
}

// Input of the last rknn_inputs_set of any context.
const uint8_t* fake_last_input(size_t* size) {
  static std::vector<uint8_t> last;
  for (const auto& entry : fakes) {
    if (!entry.second.last_input.empty()) last = entry.second.last_input;
  }
  *size = last.size();
  return last.data();
}

}  // extern "C"
