#include <gtest/gtest.h>

#include "test_util.h"
#include "tron_vision/model_check.h"

namespace tron_vision {
namespace {

TensorInfo Tensor(std::vector<uint32_t> dims, int fmt, int type, int32_t zp, float scale) {
  TensorInfo t;
  t.name = "t";
  t.dims = std::move(dims);
  t.fmt = fmt;
  t.type = type;
  t.qnt_type = abi::kQuantAffine;
  t.zp = zp;
  t.scale = scale;
  t.n_elems = 1;
  for (uint32_t d : t.dims) t.n_elems *= d;
  return t;
}

// Layout as measured on best_large_0208 (M1, 09.11.), with `classes` class channels.
void Measured(uint32_t classes, std::vector<TensorInfo>& in, std::vector<TensorInfo>& out) {
  in = {Tensor({1, 640, 480, 3}, abi::kNhwc, abi::kInt8, -128, 1.0F / 255)};
  out.clear();
  for (uint32_t stride : {8U, 16U, 32U}) {
    const uint32_t h = 640 / stride;
    const uint32_t w = 480 / stride;
    out.push_back(Tensor({1, 64, h, w}, abi::kNchw, abi::kInt8, 51, 0.166F));
    out.push_back(Tensor({1, classes, h, w}, abi::kNchw, abi::kInt8, -128, 0.0034F));
    out.push_back(Tensor({1, 1, h, w}, abi::kNchw, abi::kInt8, -128, 0.0034F));
  }
}

TEST(ModelCheck, AcceptsTheLargeModelWithItsList) {
  std::vector<TensorInfo> in;
  std::vector<TensorInfo> out;
  Measured(22, in, out);
  ModelLayout layout;
  std::string reason;
  ASSERT_TRUE(CheckModel(in, out, 22, layout, reason)) << reason;
  EXPECT_EQ(layout.classes, 22U);
  ASSERT_EQ(layout.branches.size(), 3U);
  EXPECT_EQ(layout.branches[0].stride, 8U);
  EXPECT_EQ(layout.branches[0].height, 80U);
  EXPECT_EQ(layout.branches[0].width, 60U);
  EXPECT_EQ(layout.branches[2].stride, 32U);
  EXPECT_EQ(layout.branches[2].box, 6U);
  EXPECT_EQ(layout.branches[2].cls, 7U);
  EXPECT_EQ(layout.branches[2].sum, 8U);
}

// best_small_0208 has 1 class channel but a list of 2 names: it must be refused with a reason, never run.
TEST(ModelCheck, RefusesTheSmallModelWhoseListDoesNotFit) {
  std::vector<TensorInfo> in;
  std::vector<TensorInfo> out;
  Measured(1, in, out);
  ModelLayout layout;
  std::string reason;
  EXPECT_FALSE(CheckModel(in, out, 2, layout, reason));
  EXPECT_NE(reason.find("1 classes"), std::string::npos) << reason;
  EXPECT_NE(reason.find("2 lines"), std::string::npos) << reason;
  EXPECT_TRUE(layout.branches.empty());
  EXPECT_EQ(layout.classes, 0U);
}

TEST(ModelCheck, RefusesOtherInputs) {
  std::vector<TensorInfo> in;
  std::vector<TensorInfo> out;
  Measured(22, in, out);
  ModelLayout layout;
  std::string reason;
  in[0].dims = {1, 480, 640, 3};
  EXPECT_FALSE(CheckModel(in, out, 22, layout, reason));
  Measured(22, in, out);
  in[0].fmt = abi::kNchw;
  EXPECT_FALSE(CheckModel(in, out, 22, layout, reason));
  Measured(22, in, out);
  in.push_back(in[0]);
  EXPECT_FALSE(CheckModel(in, out, 22, layout, reason));
}

TEST(ModelCheck, RefusesOtherOutputs) {
  std::vector<TensorInfo> in;
  std::vector<TensorInfo> out;
  ModelLayout layout;
  std::string reason;
  Measured(22, in, out);
  out.pop_back();
  EXPECT_FALSE(CheckModel(in, out, 22, layout, reason));
  Measured(22, in, out);
  out[4].dims[1] = 21;  // classes differ between scales
  EXPECT_FALSE(CheckModel(in, out, 22, layout, reason));
  Measured(22, in, out);
  out[3].dims[1] = 4;  // box without bins
  EXPECT_FALSE(CheckModel(in, out, 22, layout, reason));
  Measured(22, in, out);
  out[0].type = abi::kFloat16;
  EXPECT_FALSE(CheckModel(in, out, 22, layout, reason));
  Measured(22, in, out);
  out[8].scale = 0.0F;
  EXPECT_FALSE(CheckModel(in, out, 22, layout, reason));
  Measured(22, in, out);
  std::swap(out[0], out[3]);  // scales out of order
  EXPECT_FALSE(CheckModel(in, out, 22, layout, reason));
}

TEST(ClassList, TrimsAndSkipsEmptyLines) {
  test::TempDir dir;
  const std::string file = dir.Write("list.txt", "person\r\n  dog \n\n\tsports ball\n");
  std::vector<std::string> names;
  std::string error;
  ASSERT_TRUE(ReadClassList(file, names, error)) << error;
  EXPECT_EQ(names, (std::vector<std::string>{"person", "dog", "sports ball"}));
}

TEST(ClassList, FailsOnMissingOrHugeFile) {
  test::TempDir dir;
  std::vector<std::string> names;
  std::string error;
  EXPECT_FALSE(ReadClassList(dir.Path() + "/none.txt", names, error));
  EXPECT_FALSE(ReadClassList(dir.Write("big.txt", test::Lines(1001)), names, error));
  EXPECT_TRUE(names.empty());
}

}  // namespace
}  // namespace tron_vision
