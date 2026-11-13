#include <dlfcn.h>
#include <gtest/gtest.h>

#include "test_util.h"
#include "tron_vision/model_check.h"
#include "tron_vision/npu_model.h"
#include "tron_vision/yolo_decode.h"

namespace tron_vision {
namespace {

using CoreMaskFn = int (*)(abi::Context);
using OpenFn = size_t (*)();

// Test hooks of the stand-in library, from the same handle the model uses.
template <typename Fn>
Fn Hook(const char* name) {
  void* lib = dlopen(FAKE_NPU_LIB, RTLD_NOW | RTLD_NOLOAD);
  Fn fn = lib != nullptr ? reinterpret_cast<Fn>(dlsym(lib, name)) : nullptr;
  if (lib != nullptr) dlclose(lib);
  return fn;
}

TEST(NpuModel, OpensQueriesAndRuns) {
  test::TempDir dir;
  const std::string model = dir.Write("m.rknn", "classes 22\nhit 0 0.8 16 10 20 2 3 4 5\n");
  NpuModel m;
  std::string error;
  ASSERT_TRUE(m.Open(FAKE_NPU_LIB, model, 0, error)) << error;
  EXPECT_TRUE(m.IsOpen());
  EXPECT_EQ(m.SdkVersion(), "fake 0");
  ASSERT_EQ(m.Inputs().size(), 1U);
  EXPECT_EQ(m.Inputs()[0].dims, (std::vector<uint32_t>{1, 640, 480, 3}));
  EXPECT_EQ(m.Inputs()[0].name, "images");
  ASSERT_EQ(m.Outputs().size(), 9U);
  EXPECT_EQ(m.Outputs()[4].dims, (std::vector<uint32_t>{1, 22, 40, 30}));

  ModelLayout layout;
  ASSERT_TRUE(CheckModel(m.Inputs(), m.Outputs(), 22, layout, error)) << error;

  auto open_count = Hook<OpenFn>("fake_open_contexts");
  ASSERT_NE(open_count, nullptr);
  EXPECT_EQ(open_count(), 1U);

  std::vector<uint8_t> image(640 * 480 * 3, 7);
  std::vector<std::vector<int8_t>> outputs;
  ASSERT_TRUE(m.Run(image.data(), image.size(), outputs, error)) << error;
  ASSERT_EQ(outputs.size(), 9U);
  EXPECT_EQ(outputs[1].size(), 22U * 80U * 60U);
  const auto dets = DecodeDetections(layout, m.Outputs(), outputs, DecodeParams{});
  ASSERT_EQ(dets.size(), 1U);
  EXPECT_NEAR(dets[0].x1, (10.5F - 2.0F) * 16.0F, 0.1F);
  EXPECT_NEAR(dets[0].y2, (20.5F + 5.0F) * 16.0F, 0.1F);

  EXPECT_FALSE(m.Run(image.data(), image.size() - 1, outputs, error));
  m.Close();
  EXPECT_FALSE(m.IsOpen());
}

// The mask of the open context, -1 without one.
int OpenCoreMask() {
  auto core_mask = Hook<CoreMaskFn>("fake_core_mask");
  if (core_mask == nullptr) return -1;
  // Contexts count from 1 per process; the one just opened is the highest still open.
  int mask = -1;
  for (abi::Context c = 1; c < 100; c++) {
    if (core_mask(c) >= 0) mask = core_mask(c);
  }
  return mask;
}

// The runtime takes one bit per core: 1 for core 0, 2 for core 1.
TEST(NpuModel, BindsTheGivenCore) {
  test::TempDir dir;
  NpuModel m;
  std::string error;
  ASSERT_TRUE(m.Open(FAKE_NPU_LIB, dir.Write("m.rknn", "classes 1\n"), 0, error)) << error;
  EXPECT_EQ(OpenCoreMask(), 1);
  ASSERT_TRUE(m.Open(FAKE_NPU_LIB, dir.Write("m.rknn", "classes 1\n"), 1, error)) << error;
  EXPECT_EQ(OpenCoreMask(), 2);
  for (int core : {-1, 3}) {
    EXPECT_FALSE(m.Open(FAKE_NPU_LIB, dir.Write("m.rknn", "classes 1\n"), core, error));
    EXPECT_NE(error.find("NPU core"), std::string::npos) << error;
    EXPECT_FALSE(m.IsOpen());
  }
}

TEST(NpuModel, ReportsFailures) {
  test::TempDir dir;
  NpuModel m;
  std::string error;
  EXPECT_FALSE(m.Open(dir.Path() + "/none.so", dir.Write("m.rknn", "classes 1\n"), 0, error));
  EXPECT_NE(error.find("cannot load"), std::string::npos) << error;
  EXPECT_FALSE(m.Open(FAKE_NPU_LIB, dir.Path() + "/none.rknn", 0, error));
  EXPECT_FALSE(m.Open(FAKE_NPU_LIB, dir.Write("empty.rknn", ""), 0, error));
  EXPECT_FALSE(m.Open(FAKE_NPU_LIB, dir.Write("bad.rknn", "fail_init\n"), 0, error));
  EXPECT_NE(error.find("rknn_init"), std::string::npos) << error;
  EXPECT_FALSE(m.IsOpen());

  ASSERT_TRUE(m.Open(FAKE_NPU_LIB, dir.Write("run.rknn", "fail_run\n"), 0, error)) << error;
  std::vector<uint8_t> image(640 * 480 * 3);
  std::vector<std::vector<int8_t>> outputs;
  EXPECT_FALSE(m.Run(image.data(), image.size(), outputs, error));
  EXPECT_NE(error.find("rknn_run"), std::string::npos) << error;
}

TEST(NpuModel, CloseDestroysTheContext) {
  test::TempDir dir;
  std::string error;
  {
    NpuModel m;
    ASSERT_TRUE(m.Open(FAKE_NPU_LIB, dir.Write("m.rknn", "classes 1\n"), 0, error)) << error;
    // Opening again replaces the context instead of leaking it.
    ASSERT_TRUE(m.Open(FAKE_NPU_LIB, dir.Write("m2.rknn", "classes 2\n"), 0, error)) << error;
    auto open_count = Hook<OpenFn>("fake_open_contexts");
    ASSERT_NE(open_count, nullptr);
    EXPECT_EQ(open_count(), 1U);
  }
  // Closed contexts are destroyed, so only the new one is open.
  NpuModel m;
  ASSERT_TRUE(m.Open(FAKE_NPU_LIB, dir.Write("m3.rknn", "classes 1\n"), 0, error)) << error;
  EXPECT_EQ(Hook<OpenFn>("fake_open_contexts")(), 1U);
}

}  // namespace
}  // namespace tron_vision
