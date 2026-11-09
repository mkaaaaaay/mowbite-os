#include "tron_vision/npu_model.h"

#include <dlfcn.h>

#include <cstring>
#include <fstream>
#include <iterator>

namespace tron_vision {

namespace {

// Larger files are not one of the expected models (best_large_0208 has 28.8 MB).
constexpr std::streamoff kMaxModelBytes = 64LL * 1024 * 1024;

bool ReadFile(const std::string& path, std::vector<char>& data, std::string& error) {
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in) {
    error = "cannot open " + path;
    return false;
  }
  const std::streamoff size = in.tellg();
  if (size <= 0 || size > kMaxModelBytes) {
    error = path + ": size " + std::to_string(size) + " out of range";
    return false;
  }
  data.resize(static_cast<size_t>(size));
  in.seekg(0);
  if (!in.read(data.data(), size)) {
    error = "cannot read " + path;
    return false;
  }
  return true;
}

TensorInfo ToInfo(const abi::TensorAttr& attr) {
  TensorInfo info;
  info.name.assign(attr.name, strnlen(attr.name, sizeof(attr.name)));
  const uint32_t n = attr.n_dims < abi::kMaxDims ? attr.n_dims : abi::kMaxDims;
  info.dims.assign(attr.dims, attr.dims + n);
  info.fmt = attr.fmt;
  info.type = attr.type;
  info.qnt_type = attr.qnt_type;
  info.zp = attr.zp;
  info.scale = attr.scale;
  info.n_elems = attr.n_elems;
  return info;
}

template <typename Fn>
bool Symbol(void* library, const char* name, Fn& fn, std::string& error) {
  fn = reinterpret_cast<Fn>(dlsym(library, name));
  if (fn == nullptr) {
    error = std::string("symbol ") + name + " missing";
    return false;
  }
  return true;
}

}  // namespace

NpuModel::~NpuModel() {
  Close();
}

bool NpuModel::Resolve(std::string& error) {
  return Symbol(library_, "rknn_init", init_, error) && Symbol(library_, "rknn_query", query_, error) &&
         Symbol(library_, "rknn_set_core_mask", set_core_mask_, error) &&
         Symbol(library_, "rknn_inputs_set", inputs_set_, error) && Symbol(library_, "rknn_run", run_, error) &&
         Symbol(library_, "rknn_outputs_get", outputs_get_, error) &&
         Symbol(library_, "rknn_outputs_release", outputs_release_, error) &&
         Symbol(library_, "rknn_destroy", destroy_, error);
}

bool NpuModel::Open(const std::string& library_path, const std::string& model_path, std::string& error) {
  Close();
  library_ = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (library_ == nullptr) {
    const char* e = dlerror();  // NOLINT(concurrency-mt-unsafe): models are loaded on one thread
    error = "cannot load " + library_path + (e != nullptr ? std::string(": ") + e : std::string());
    return false;
  }
  if (!Resolve(error)) {
    Close();
    return false;
  }
  {
    std::vector<char> data;
    if (!ReadFile(model_path, data, error)) {
      Close();
      return false;
    }
    abi::Context context = 0;
    const int ret = init_(&context, data.data(), static_cast<uint32_t>(data.size()), 0, nullptr);
    if (ret != 0 || context == 0) {
      error = "rknn_init " + std::to_string(ret);
      Close();
      return false;
    }
    context_ = context;
  }
  abi::SdkVersion version{};
  if (query_(context_, abi::kQuerySdkVersion, &version, sizeof(version)) == 0) {
    sdk_version_.assign(version.api, strnlen(version.api, sizeof(version.api)));
  }
  if (!QueryTensors(error)) {
    Close();
    return false;
  }
  const int ret = set_core_mask_(context_, abi::kCore0);
  if (ret != 0) {
    error = "rknn_set_core_mask " + std::to_string(ret);
    Close();
    return false;
  }
  return true;
}

bool NpuModel::QueryTensors(std::string& error) {
  abi::InOutNum num{};
  int ret = query_(context_, abi::kQueryInOutNum, &num, sizeof(num));
  if (ret != 0) {
    error = "query in_out_num " + std::to_string(ret);
    return false;
  }
  // The expected models have 1 input and 9 outputs; anything far beyond is not a detection model.
  if (num.n_input > 4 || num.n_output > 32) {
    error = "model has " + std::to_string(num.n_input) + " inputs and " + std::to_string(num.n_output) + " outputs";
    return false;
  }
  inputs_.clear();
  outputs_.clear();
  for (uint32_t i = 0; i < num.n_input + num.n_output; i++) {
    const bool is_input = i < num.n_input;
    abi::TensorAttr attr{};
    attr.index = is_input ? i : i - num.n_input;
    ret = query_(context_, is_input ? abi::kQueryInputAttr : abi::kQueryOutputAttr, &attr, sizeof(attr));
    if (ret != 0) {
      error = std::string("query ") + (is_input ? "input " : "output ") + std::to_string(attr.index) + " " +
              std::to_string(ret);
      return false;
    }
    (is_input ? inputs_ : outputs_).push_back(ToInfo(attr));
  }
  return true;
}

bool NpuModel::Run(const uint8_t* image, size_t size, std::vector<std::vector<int8_t>>& outputs, std::string& error) {
  if (context_ == 0 || inputs_.size() != 1) {
    error = "model not open";
    return false;
  }
  if (size != inputs_[0].n_elems) {
    error = "input size " + std::to_string(size) + " instead of " + std::to_string(inputs_[0].n_elems);
    return false;
  }
  abi::Input input{};
  input.index = 0;
  input.buf = const_cast<uint8_t*>(image);  // the runtime only reads the input
  input.size = static_cast<uint32_t>(size);
  input.pass_through = 0;
  input.type = abi::kUint8;
  input.fmt = abi::kNhwc;
  int ret = inputs_set_(context_, 1, &input);
  if (ret != 0) {
    error = "rknn_inputs_set " + std::to_string(ret);
    return false;
  }
  ret = run_(context_, nullptr);
  if (ret != 0) {
    error = "rknn_run " + std::to_string(ret);
    return false;
  }
  std::vector<abi::Output> raw(outputs_.size());
  for (size_t i = 0; i < raw.size(); i++) {
    raw[i].index = static_cast<uint32_t>(i);
  }
  ret = outputs_get_(context_, static_cast<uint32_t>(raw.size()), raw.data(), nullptr);
  if (ret != 0) {
    error = "rknn_outputs_get " + std::to_string(ret);
    return false;
  }
  outputs.resize(raw.size());
  bool sizes_ok = true;
  for (size_t i = 0; i < raw.size(); i++) {
    if (raw[i].buf == nullptr || raw[i].size != outputs_[i].n_elems) {
      sizes_ok = false;
      break;
    }
    const auto* data = static_cast<const int8_t*>(raw[i].buf);
    outputs[i].assign(data, data + raw[i].size);
  }
  outputs_release_(context_, static_cast<uint32_t>(raw.size()), raw.data());
  if (!sizes_ok) {
    error = "output size differs from its attributes";
    return false;
  }
  return true;
}

void NpuModel::Close() {
  if (context_ != 0 && destroy_ != nullptr) {
    destroy_(context_);
  }
  context_ = 0;
  if (library_ != nullptr) {
    dlclose(library_);
    library_ = nullptr;
  }
  init_ = nullptr;
  query_ = nullptr;
  set_core_mask_ = nullptr;
  inputs_set_ = nullptr;
  run_ = nullptr;
  outputs_get_ = nullptr;
  outputs_release_ = nullptr;
  destroy_ = nullptr;
  inputs_.clear();
  outputs_.clear();
  sdk_version_.clear();
}

}  // namespace tron_vision
