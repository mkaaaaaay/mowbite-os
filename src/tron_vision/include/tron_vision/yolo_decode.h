#ifndef TRON_VISION_YOLO_DECODE_H
#define TRON_VISION_YOLO_DECODE_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "tron_vision/model_check.h"

namespace tron_vision {

// Box corners in pixels of the model input.
struct Detection {
  uint32_t class_id = 0;
  float score = 0.0F;
  float x1 = 0.0F;
  float y1 = 0.0F;
  float x2 = 0.0F;
  float y2 = 0.0F;
};

struct DecodeParams {
  float score_threshold = 0.25F;
  float nms_iou = 0.45F;
  // Bounds the work of the suppression; the best candidates are kept.
  size_t max_candidates = 1000;
  size_t max_detections = 50;
};

// Cells whose score sum is below the threshold are skipped; the best class of the others counts if it reaches the
// threshold, its box comes from the expected value of each side's bin distribution. Suppression per class, result
// sorted by score.
std::vector<Detection> DecodeDetections(const ModelLayout& layout, const std::vector<TensorInfo>& outputs,
                                        const std::vector<std::vector<int8_t>>& data, const DecodeParams& params);

float Iou(const Detection& a, const Detection& b);

}  // namespace tron_vision

#endif  // TRON_VISION_YOLO_DECODE_H
