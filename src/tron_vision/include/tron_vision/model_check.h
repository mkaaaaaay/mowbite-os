#ifndef TRON_VISION_MODEL_CHECK_H
#define TRON_VISION_MODEL_CHECK_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "tron_vision/npu_model.h"

namespace tron_vision {

// Input of the detection models of the Tron: portrait, NHWC.
constexpr uint32_t kModelWidth = 480;
constexpr uint32_t kModelHeight = 640;
// Box side as a distribution over this many bins per side, 4 sides.
constexpr uint32_t kBoxBins = 16;

// One output scale: indices of its box, class and score sum tensors.
struct Branch {
  size_t box = 0;
  size_t cls = 0;
  size_t sum = 0;
  uint32_t height = 0;
  uint32_t width = 0;
  uint32_t stride = 0;
};

struct ModelLayout {
  uint32_t classes = 0;
  std::vector<Branch> branches;
};

// Accepts only the layout of the YOLOv8 export for the NPU: 1 input [1,640,480,3] NHWC 8 bit, 3 scales (stride 8, 16,
// 32) of box [1,64,h,w], classes [1,n,h,w] and sum [1,1,h,w], NCHW int8 quantized, and n equal to the class list.
// The reason names the first mismatch.
bool CheckModel(const std::vector<TensorInfo>& inputs, const std::vector<TensorInfo>& outputs, size_t class_names,
                ModelLayout& layout, std::string& reason);

// One class name per line, trimmed; empty lines are skipped. Fails on a missing file or more than 1000 lines.
bool ReadClassList(const std::string& path, std::vector<std::string>& names, std::string& error);

}  // namespace tron_vision

#endif  // TRON_VISION_MODEL_CHECK_H
