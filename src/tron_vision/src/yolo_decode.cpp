#include "tron_vision/yolo_decode.h"

#include <algorithm>
#include <cmath>

namespace tron_vision {

namespace {

float Dequant(int8_t q, const TensorInfo& t) {
  return (static_cast<float>(q) - static_cast<float>(t.zero_point)) * t.scale;
}

// Expected bin of one side: softmax over its bins, at cell index `cell` of a plane of `plane` values.
float SideDistance(const std::vector<int8_t>& box, const TensorInfo& t, uint32_t side, size_t plane, size_t cell) {
  float values[kBoxBins];
  float top = -INFINITY;
  for (uint32_t i = 0; i < kBoxBins; i++) {
    values[i] = Dequant(box[(side * kBoxBins + i) * plane + cell], t);
    top = std::max(top, values[i]);
  }
  float sum = 0.0F;
  float weighted = 0.0F;
  for (uint32_t i = 0; i < kBoxBins; i++) {
    const float e = std::exp(values[i] - top);
    sum += e;
    weighted += e * static_cast<float>(i);
  }
  return weighted / sum;
}

bool ByScore(const Detection& a, const Detection& b) {
  return a.score > b.score;
}

}  // namespace

float Iou(const Detection& a, const Detection& b) {
  const float w = std::min(a.x2, b.x2) - std::max(a.x1, b.x1);
  const float h = std::min(a.y2, b.y2) - std::max(a.y1, b.y1);
  if (w <= 0.0F || h <= 0.0F) return 0.0F;
  const float inter = w * h;
  const float uni = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - inter;
  return uni > 0.0F ? inter / uni : 0.0F;
}

std::vector<Detection> DecodeDetections(const ModelLayout& layout, const std::vector<TensorInfo>& outputs,
                                        const std::vector<std::vector<int8_t>>& data, const DecodeParams& params) {
  std::vector<Detection> candidates;
  if (data.size() != outputs.size()) return candidates;
  for (const Branch& br : layout.branches) {
    const TensorInfo& box_t = outputs[br.box];
    const TensorInfo& cls_t = outputs[br.cls];
    const TensorInfo& sum_t = outputs[br.sum];
    const std::vector<int8_t>& box = data[br.box];
    const std::vector<int8_t>& cls = data[br.cls];
    const std::vector<int8_t>& sum = data[br.sum];
    const size_t plane = static_cast<size_t>(br.height) * br.width;
    if (box.size() != plane * 4 * kBoxBins || cls.size() != layout.classes * plane || sum.size() != plane) {
      return {};
    }
    for (size_t cell = 0; cell < plane; cell++) {
      // The sum over the classes bounds each class score from above.
      if (Dequant(sum[cell], sum_t) < params.score_threshold) continue;
      uint32_t best = 0;
      int8_t best_q = cls[cell];
      for (uint32_t c = 1; c < layout.classes; c++) {
        const int8_t q = cls[c * plane + cell];
        if (q > best_q) {
          best_q = q;
          best = c;
        }
      }
      const float score = Dequant(best_q, cls_t);
      if (score < params.score_threshold) continue;
      const size_t col = cell % br.width;
      const size_t row = cell / br.width;
      const float cx = static_cast<float>(col) + 0.5F;
      const float cy = static_cast<float>(row) + 0.5F;
      const auto stride = static_cast<float>(br.stride);
      Detection d;
      d.class_id = best;
      d.score = score;
      d.x1 = (cx - SideDistance(box, box_t, 0, plane, cell)) * stride;
      d.y1 = (cy - SideDistance(box, box_t, 1, plane, cell)) * stride;
      d.x2 = (cx + SideDistance(box, box_t, 2, plane, cell)) * stride;
      d.y2 = (cy + SideDistance(box, box_t, 3, plane, cell)) * stride;
      candidates.push_back(d);
    }
  }
  std::sort(candidates.begin(), candidates.end(), ByScore);
  if (candidates.size() > params.max_candidates) candidates.resize(params.max_candidates);

  std::vector<Detection> kept;
  std::vector<bool> removed(candidates.size(), false);
  for (size_t i = 0; i < candidates.size() && kept.size() < params.max_detections; i++) {
    if (removed[i]) continue;
    kept.push_back(candidates[i]);
    for (size_t j = i + 1; j < candidates.size(); j++) {
      if (!removed[j] && candidates[j].class_id == candidates[i].class_id &&
          Iou(candidates[i], candidates[j]) > params.nms_iou) {
        removed[j] = true;
      }
    }
  }
  return kept;
}

}  // namespace tron_vision
