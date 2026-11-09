#include "tron_vision/model_check.h"

#include <fstream>

namespace tron_vision {

namespace {

constexpr size_t kMaxClassLines = 1000;
constexpr uint32_t kStrides[] = {8, 16, 32};

std::string Dims(const std::vector<uint32_t>& dims) {
  std::string s = "[";
  for (size_t i = 0; i < dims.size(); i++) {
    s += (i > 0 ? "," : "") + std::to_string(dims[i]);
  }
  return s + "]";
}

bool QuantizedNchw(const TensorInfo& t, uint32_t channels, uint32_t height, uint32_t width, std::string& reason) {
  if (t.dims != std::vector<uint32_t>{1, channels, height, width}) {
    reason = "output " + t.name + " has dims " + Dims(t.dims) + ", expected " +
             Dims({1, channels, height, width});
    return false;
  }
  if (t.format != abi::kNchw || t.type != abi::kInt8 || t.quant != abi::kQuantAffine || !(t.scale > 0.0F)) {
    reason = "output " + t.name + " is not NCHW int8 with affine quantization";
    return false;
  }
  return true;
}

}  // namespace

bool CheckModel(const std::vector<TensorInfo>& inputs, const std::vector<TensorInfo>& outputs, size_t class_names,
                ModelLayout& layout, std::string& reason) {
  layout = ModelLayout{};
  if (inputs.size() != 1) {
    reason = std::to_string(inputs.size()) + " inputs, expected 1";
    return false;
  }
  const TensorInfo& in = inputs[0];
  if (in.dims != std::vector<uint32_t>{1, kModelHeight, kModelWidth, 3} || in.format != abi::kNhwc ||
      (in.type != abi::kInt8 && in.type != abi::kUint8)) {
    reason = "input " + in.name + " has dims " + Dims(in.dims) + ", expected NHWC 8 bit [1,640,480,3]";
    return false;
  }
  if (outputs.size() != 3 * std::size(kStrides)) {
    reason = std::to_string(outputs.size()) + " outputs, expected 9";
    return false;
  }
  const uint32_t classes = outputs[1].dims.size() == 4 ? outputs[1].dims[1] : 0;
  if (classes == 0) {
    reason = "output " + outputs[1].name + " has dims " + Dims(outputs[1].dims);
    return false;
  }
  for (size_t b = 0; b < std::size(kStrides); b++) {
    Branch branch;
    branch.box = 3 * b;
    branch.cls = 3 * b + 1;
    branch.sum = 3 * b + 2;
    branch.stride = kStrides[b];
    branch.height = kModelHeight / branch.stride;
    branch.width = kModelWidth / branch.stride;
    if (!QuantizedNchw(outputs[branch.box], 4 * kBoxBins, branch.height, branch.width, reason) ||
        !QuantizedNchw(outputs[branch.cls], classes, branch.height, branch.width, reason) ||
        !QuantizedNchw(outputs[branch.sum], 1, branch.height, branch.width, reason)) {
      return false;
    }
    layout.branches.push_back(branch);
  }
  if (classes != class_names) {
    reason = "model has " + std::to_string(classes) + " classes, the class list " + std::to_string(class_names) +
             " lines";
    layout = ModelLayout{};
    return false;
  }
  layout.classes = classes;
  return true;
}

bool ReadClassList(const std::string& path, std::vector<std::string>& names, std::string& error) {
  names.clear();
  std::ifstream in(path);
  if (!in) {
    error = "cannot open " + path;
    return false;
  }
  std::string line;
  size_t lines = 0;
  while (std::getline(in, line)) {
    if (++lines > kMaxClassLines) {
      error = path + " has more than " + std::to_string(kMaxClassLines) + " lines";
      names.clear();
      return false;
    }
    const size_t first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos) continue;
    const size_t last = line.find_last_not_of(" \t\r");
    names.push_back(line.substr(first, last - first + 1));
  }
  return true;
}

}  // namespace tron_vision
