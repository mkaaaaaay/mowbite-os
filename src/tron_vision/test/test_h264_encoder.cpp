#include <dlfcn.h>
#include <gtest/gtest.h>

#include <cstdlib>

#include "tron_vision/h264_encoder.h"
#include "tron_vision/h264_rtp.h"

namespace tron_vision {
namespace {

template <typename Fn>
Fn FakeCall(const char* name) {
  void* lib = dlopen(FAKE_MPP_LIB, RTLD_NOW);
  return lib != nullptr ? reinterpret_cast<Fn>(dlsym(lib, name)) : nullptr;
}

std::string Report() {
  return FakeCall<const char* (*)()>("fake_mpp_report")();
}

int Live() {
  return FakeCall<int (*)()>("fake_mpp_live")();
}

CapturedFrame Frame(uint32_t index, int64_t timestamp_us) {
  CapturedFrame frame;
  frame.index = index;
  frame.bytes = 2560 * 480;
  frame.stride = 2560;
  frame.timestamp_us = timestamp_us;
  frame.dmabuf = 200 + static_cast<int>(index);
  return frame;
}

std::vector<uint8_t> Types(const uint8_t* data, size_t size) {
  std::vector<uint8_t> types;
  for (const NalUnit& nal : SplitAnnexB(data, size)) types.push_back(nal.Type());
  return types;
}

class MppEncoderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    unsetenv("FAKE_MPP_FAIL");
    unsetenv("FAKE_MPP_STREAM");
    Report();
  }
  void TearDown() override {
    unsetenv("FAKE_MPP_FAIL");
    EXPECT_EQ(Live(), 0);
  }
  std::string error;
};

TEST_F(MppEncoderTest, MissingLibrary) {
  MppEncoder encoder("/nonexistent/librockchip_mpp.so.1");
  EXPECT_FALSE(encoder.Probe(error));
  EXPECT_NE(error.find("cannot load /nonexistent/librockchip_mpp.so.1"), std::string::npos) << error;
  EXPECT_FALSE(encoder.Open({}, error));
  std::vector<uint8_t> header;
  EXPECT_FALSE(encoder.Header(header, error));
  EXPECT_EQ(error, "encoder not open");
}

TEST_F(MppEncoderTest, LibraryWithoutTheCalls) {
  MppEncoder encoder(FAKE_NPU_LIB);
  EXPECT_FALSE(encoder.Probe(error));
  EXPECT_EQ(error, "symbol mpp_create missing");
}

TEST_F(MppEncoderTest, SettingsOfTheStream) {
  MppEncoder encoder(FAKE_MPP_LIB);
  ASSERT_TRUE(encoder.Probe(error)) << error;
  EncoderSettings settings;
  settings.fps = 24;
  ASSERT_TRUE(encoder.Open(settings, error)) << error;
  const std::string report = Report();
  for (const char* line :
       {"init 1 7\n", "cfg prep:width=640\n", "cfg prep:height=480\n", "cfg prep:hor_stride=2560\n",
        "cfg prep:ver_stride=480\n", "cfg prep:format=8\n", "cfg rc:mode=1\n", "cfg rc:bps_target=1500000\n",
        "cfg rc:bps_max=1800000\n", "cfg rc:bps_min=1000000\n", "cfg rc:fps_in_num=24\n", "cfg rc:fps_in_denom=1\n",
        "cfg rc:fps_out_num=24\n", "cfg rc:fps_out_denom=1\n", "cfg rc:gop=24\n", "cfg codec:type=7\n",
        "cfg h264:profile=66\n", "cfg h264:level=31\n", "cfg h264:constraint_set=458759\n", "cfg h264:cabac_en=0\n",
        "sei 0\n", "header_mode 1\n"}) {
    EXPECT_NE(report.find(line), std::string::npos) << line << report;
  }
  EXPECT_EQ(report.find("unknown command"), std::string::npos) << report;

  std::vector<uint8_t> header;
  ASSERT_TRUE(encoder.Header(header, error)) << error;
  ParameterSets sets;
  ASSERT_TRUE(FindParameterSets(header.data(), header.size(), sets));
  EXPECT_EQ(ProfileLevelId(sets.sps), "42e01f");
}

TEST_F(MppEncoderTest, BitRateStaysInItsRange) {
  MppEncoder encoder(FAKE_MPP_LIB);
  EncoderSettings settings;
  settings.bitrate_bps = 100000;
  ASSERT_TRUE(encoder.Open(settings, error)) << error;
  EXPECT_NE(Report().find("cfg rc:bps_target=500000\n"), std::string::npos);
  settings.bitrate_bps = 9000000;
  ASSERT_TRUE(encoder.Open(settings, error)) << error;
  EXPECT_NE(Report().find("cfg rc:bps_target=4000000\n"), std::string::npos);
}

TEST_F(MppEncoderTest, EncodesOneFrameAfterTheOther) {
  MppEncoder encoder(FAKE_MPP_LIB);
  EncoderSettings settings;
  settings.fps = 3;
  ASSERT_TRUE(encoder.Open(settings, error)) << error;
  Report();
  const uint8_t* data = nullptr;
  size_t size = 0;
  std::vector<std::vector<uint8_t>> types;
  for (int i = 0; i < 4; i++) {
    ASSERT_TRUE(encoder.Encode(Frame(static_cast<uint32_t>(i % 2), 5000000 + i * 42000), data, size, error)) << error;
    types.push_back(Types(data, size));
  }
  // SPS and PPS come with every IDR picture, once a second.
  EXPECT_EQ(types, (std::vector<std::vector<uint8_t>>{{7, 8, 5}, {1}, {1}, {7, 8, 5}}));
  const std::string report = Report();
  EXPECT_NE(report.find("frame 640x480 stride 2560x480 format 8 pts 5000000 fd 200 index 0 size 1228800 type 2\n"),
            std::string::npos)
      << report;
  EXPECT_NE(report.find("pts 5042000 fd 201 index 1"), std::string::npos) << report;
  // Each buffer is imported once.
  EXPECT_EQ(report.find("import fd 200"), report.rfind("import fd 200"));
  EXPECT_EQ(report.find("import fd 201"), report.rfind("import fd 201"));
  EXPECT_EQ(Live(), 3);

  CapturedFrame moved = Frame(0, 1);
  moved.dmabuf = 300;
  EXPECT_FALSE(encoder.Encode(moved, data, size, error));
  EXPECT_EQ(error, "buffer 0 changed its dma-buf");
  CapturedFrame mapped_only = Frame(2, 1);
  mapped_only.dmabuf = -1;
  EXPECT_FALSE(encoder.Encode(mapped_only, data, size, error));
  encoder.Close();
  EXPECT_FALSE(encoder.Encode(Frame(0, 1), data, size, error));
  EXPECT_EQ(error, "encoder not open");
}

TEST_F(MppEncoderTest, EveryStepMayFail) {
  for (const char* step : {"create", "init", "config"}) {
    setenv("FAKE_MPP_FAIL", step, 1);
    MppEncoder encoder(FAKE_MPP_LIB);
    EXPECT_FALSE(encoder.Open({}, error)) << step;
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(Live(), 0) << step;
  }
  for (const char* step : {"header", "import", "put", "packet"}) {
    unsetenv("FAKE_MPP_FAIL");
    MppEncoder encoder(FAKE_MPP_LIB);
    ASSERT_TRUE(encoder.Open({}, error)) << error;
    setenv("FAKE_MPP_FAIL", step, 1);
    std::vector<uint8_t> header;
    const uint8_t* data = nullptr;
    size_t size = 0;
    const bool header_ok = encoder.Header(header, error);
    const bool encode_ok = encoder.Encode(Frame(0, 1), data, size, error);
    EXPECT_EQ(header_ok, std::string(step) != "header") << step;
    EXPECT_EQ(encode_ok, std::string(step) == "header") << step << " " << error;
  }
}

}  // namespace
}  // namespace tron_vision
