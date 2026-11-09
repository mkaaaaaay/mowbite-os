#include <gtest/gtest.h>

#include <cmath>

#include "tron_vision/yolo_decode.h"

namespace tron_vision {
namespace {

constexpr int32_t kBoxZp = 51;
constexpr float kBoxScale = 0.166F;
constexpr int32_t kScoreZp = -128;
constexpr float kScoreScale = 0.0034F;

int8_t Q(float v, int32_t zp, float scale) {
  const long q = std::lround(v / scale) + zp;
  return static_cast<int8_t>(std::max(-128L, std::min(127L, q)));
}

// Outputs of a model with `classes` classes, all cells empty.
struct Model {
  explicit Model(uint32_t classes) {
    layout.classes = classes;
    for (uint32_t b = 0; b < 3; b++) {
      Branch br;
      br.box = 3 * b;
      br.cls = 3 * b + 1;
      br.sum = 3 * b + 2;
      br.stride = 8U << b;
      br.height = 640 / br.stride;
      br.width = 480 / br.stride;
      layout.branches.push_back(br);
      const size_t plane = static_cast<size_t>(br.height) * br.width;
      for (uint32_t k = 0; k < 3; k++) {
        TensorInfo t;
        t.zp = k == 0 ? kBoxZp : kScoreZp;
        t.scale = k == 0 ? kBoxScale : kScoreScale;
        info.push_back(t);
        const size_t n = (k == 0 ? 64 : k == 1 ? classes : 1) * plane;
        data.emplace_back(n, k == 0 ? Q(-10.0F, kBoxZp, kBoxScale) : static_cast<int8_t>(-128));
      }
    }
  }

  // Detection at cell (col,row) of scale b; sides in cells, peaked so the expected bin is the side.
  void Hit(uint32_t b, uint32_t cls, float score, uint32_t col, uint32_t row, const uint32_t side[4],
           float sum = -1.0F) {
    const Branch& br = layout.branches[b];
    const size_t plane = static_cast<size_t>(br.height) * br.width;
    const size_t cell = static_cast<size_t>(row) * br.width + col;
    data[br.cls][cls * plane + cell] = Q(score, kScoreZp, kScoreScale);
    data[br.sum][cell] = Q(sum < 0.0F ? score : sum, kScoreZp, kScoreScale);
    for (uint32_t s = 0; s < 4; s++) data[br.box][(s * 16 + side[s]) * plane + cell] = Q(10.0F, kBoxZp, kBoxScale);
  }

  ModelLayout layout;
  std::vector<TensorInfo> info;
  std::vector<std::vector<int8_t>> data;
};

TEST(YoloDecode, DecodesOneBox) {
  Model m(22);
  const uint32_t side[4] = {2, 3, 4, 5};
  m.Hit(1, 7, 0.8F, 10, 20, side);  // stride 16
  const auto dets = DecodeDetections(m.layout, m.info, m.data, DecodeParams{});
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_EQ(dets[0].class_id, 7U);
  EXPECT_NEAR(dets[0].score, 0.8F, 0.01F);
  // The other bins sit 20 below the peak, so the expected value is the side within a millionth of a cell.
  EXPECT_NEAR(dets[0].x1, (10.5F - 2.0F) * 16.0F, 0.01F);
  EXPECT_NEAR(dets[0].y1, (20.5F - 3.0F) * 16.0F, 0.01F);
  EXPECT_NEAR(dets[0].x2, (10.5F + 4.0F) * 16.0F, 0.01F);
  EXPECT_NEAR(dets[0].y2, (20.5F + 5.0F) * 16.0F, 0.01F);
}

TEST(YoloDecode, ExpectedValueBetweenBins) {
  Model m(1);
  const uint32_t side[4] = {3, 3, 3, 3};
  m.Hit(0, 0, 0.8F, 5, 5, side);
  const Branch& br = m.layout.branches[0];
  const size_t plane = static_cast<size_t>(br.height) * br.width;
  const size_t cell = 5 * br.width + 5;
  // Left side: equal peaks at bins 3 and 4 give 3.5 cells.
  m.data[br.box][4 * plane + cell] = Q(10.0F, kBoxZp, kBoxScale);
  const auto dets = DecodeDetections(m.layout, m.info, m.data, DecodeParams{});
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_NEAR(dets[0].x1, (5.5F - 3.5F) * 8.0F, 0.01F);
}

TEST(YoloDecode, ThresholdsOnSumAndClass) {
  Model m(3);
  const uint32_t side[4] = {1, 1, 1, 1};
  m.Hit(0, 0, 0.2F, 1, 1, side);         // class below the threshold
  m.Hit(0, 1, 0.6F, 30, 30, side, 0.1F);  // sum below the threshold skips the cell
  m.Hit(2, 2, 0.5F, 3, 3, side);
  const auto dets = DecodeDetections(m.layout, m.info, m.data, DecodeParams{});
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_EQ(dets[0].class_id, 2U);
}

TEST(YoloDecode, SuppressesOverlapsOfOneClassOnly) {
  Model m(2);
  const uint32_t side[4] = {4, 4, 4, 4};
  // Scores stay below 0.867, the largest the measured quantization of the class outputs holds.
  m.Hit(0, 0, 0.85F, 20, 20, side);
  m.Hit(0, 0, 0.7F, 21, 20, side);  // same class, IoU about 0.78
  m.Hit(0, 1, 0.6F, 21, 21, side);  // other class at the same place stays
  m.Hit(0, 0, 0.5F, 50, 70, side);  // same class far away stays
  const auto dets = DecodeDetections(m.layout, m.info, m.data, DecodeParams{});
  ASSERT_EQ(dets.size(), 3U);
  EXPECT_NEAR(dets[0].score, 0.85F, 0.01F);
  EXPECT_EQ(dets[1].class_id, 1U);
  EXPECT_NEAR(dets[2].score, 0.5F, 0.01F);
}

TEST(YoloDecode, LimitsTheResult) {
  Model m(1);
  const uint32_t side[4] = {0, 0, 1, 1};
  for (uint32_t i = 0; i < 40; i++) m.Hit(0, 0, 0.5F, i, 0, side);
  DecodeParams p;
  p.max_detections = 5;
  EXPECT_EQ(DecodeDetections(m.layout, m.info, m.data, p).size(), 5U);
  p.max_detections = 50;
  p.max_candidates = 10;
  EXPECT_EQ(DecodeDetections(m.layout, m.info, m.data, p).size(), 10U);
}

TEST(YoloDecode, EmptyOnWrongSizes) {
  Model m(2);
  const uint32_t side[4] = {1, 1, 1, 1};
  m.Hit(0, 0, 0.8F, 1, 1, side);
  m.data[4].pop_back();
  EXPECT_TRUE(DecodeDetections(m.layout, m.info, m.data, DecodeParams{}).empty());
  m.data.pop_back();
  EXPECT_TRUE(DecodeDetections(m.layout, m.info, m.data, DecodeParams{}).empty());
}

TEST(YoloDecode, Iou) {
  Detection a{0, 1.0F, 0, 0, 10, 10};
  Detection b{0, 1.0F, 5, 0, 15, 10};
  EXPECT_NEAR(Iou(a, b), 50.0F / 150.0F, 1e-5F);
  Detection c{0, 1.0F, 20, 20, 30, 30};
  EXPECT_EQ(Iou(a, c), 0.0F);
}

}  // namespace
}  // namespace tron_vision
