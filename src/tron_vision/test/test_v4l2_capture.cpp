#include <gtest/gtest.h>

#include "fake_video_device.h"
#include "tron_vision/v4l2_camera.h"

namespace tron_vision {
namespace {

using test::FakeVideoDevice;

constexpr uint32_t kStereoBytes = 2560 * 480;
const CaptureFormat kStereo{1280, 480, PixelFormat::kYuyv, 4, true};

// The stereo bridge and its sensors crash the kernel on enumerations, so the node sees these requests and no other.
const std::set<unsigned long> kAllowed = {VIDIOC_QUERYCAP, VIDIOC_S_FMT,  VIDIOC_REQBUFS,  VIDIOC_QUERYBUF, VIDIOC_EXPBUF,
                                          VIDIOC_QBUF,     VIDIOC_DQBUF,  VIDIOC_STREAMON, VIDIOC_STREAMOFF};

class V4l2CaptureTest : public ::testing::TestWithParam<bool> {
 protected:
  void SetUp() override {
    if (!GetParam()) device.caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
  }
  FakeVideoDevice device;
  std::string error;
};

TEST_P(V4l2CaptureTest, StreamsYuyvWithExportedBuffers) {
  {
    V4l2Capture capture(&device);
    ASSERT_TRUE(capture.Open("/dev/video22", kStereo, error)) << error;
    EXPECT_EQ(capture.Stride(), 2560U);
    EXPECT_EQ(capture.FrameBytes(), kStereoBytes);
    EXPECT_EQ(capture.ExportedBuffers(), (std::vector<int>{200, 201, 202, 203}));
    EXPECT_EQ(device.stream_ons, 0);

    ASSERT_TRUE(capture.Start(error)) << error;
    EXPECT_EQ(device.queued.size(), 4U);
    CapturedFrame frame;
    EXPECT_EQ(capture.Next(0, frame, error), 0);

    ASSERT_TRUE(device.Push(77, kStereoBytes));
    ASSERT_EQ(capture.Next(0, frame, error), 1) << error;
    EXPECT_EQ(frame.index, 0U);
    EXPECT_EQ(frame.bytes, kStereoBytes);
    EXPECT_EQ(frame.stride, 2560U);
    EXPECT_EQ(frame.sequence, 1U);
    EXPECT_EQ(frame.timestamp_us, 1042000);
    EXPECT_EQ(frame.dmabuf, 200);
    EXPECT_EQ(frame.data[0], 77);
    EXPECT_EQ(frame.data[kStereoBytes - 1], 77);
    // The frame in hand is not queued; the next call gives it back.
    EXPECT_EQ(device.queued.size(), 3U);
    EXPECT_EQ(capture.Next(0, frame, error), 0);
    EXPECT_EQ(device.queued.size(), 4U);
  }
  EXPECT_EQ(device.stream_ons, 1);
  EXPECT_EQ(device.stream_offs, 1);
  EXPECT_EQ(device.maps, 0);
  EXPECT_TRUE(device.fds.empty());
  EXPECT_TRUE(device.OnlyRequests(kAllowed));
  EXPECT_EQ(device.requests.back(), static_cast<unsigned long>(VIDIOC_REQBUFS));
}

TEST_P(V4l2CaptureTest, SkipsShortAndFaultyFrames) {
  V4l2Capture capture(&device);
  ASSERT_TRUE(capture.Open("/dev/video22", kStereo, error)) << error;
  ASSERT_TRUE(capture.Start(error)) << error;
  CapturedFrame frame;
  device.Push(1, kStereoBytes / 2);
  EXPECT_EQ(capture.Next(0, frame, error), 0);
  device.Push(2, kStereoBytes, V4L2_BUF_FLAG_ERROR);
  EXPECT_EQ(capture.Next(0, frame, error), 0);
  EXPECT_EQ(device.queued.size(), 4U);
  device.Push(3, kStereoBytes);
  ASSERT_EQ(capture.Next(0, frame, error), 1);
  EXPECT_EQ(frame.sequence, 3U);
  EXPECT_EQ(frame.data[0], 3);
}

TEST_P(V4l2CaptureTest, RefusesAnotherFormat) {
  V4l2Capture capture(&device);
  device.forced_fourcc = V4L2_PIX_FMT_SRGGB10;
  EXPECT_FALSE(capture.Open("/dev/video22", kStereo, error));
  EXPECT_NE(error.find("format not accepted"), std::string::npos) << error;
  EXPECT_TRUE(device.fds.empty());

  device.forced_fourcc = 0;
  device.stride = 1280;
  EXPECT_FALSE(capture.Open("/dev/video22", kStereo, error));
  EXPECT_NE(error.find("stride 1280"), std::string::npos) << error;

  device.stride = 2560;
  device.buffer_bytes = kStereoBytes - 1;
  EXPECT_FALSE(capture.Open("/dev/video22", kStereo, error));
  EXPECT_TRUE(device.OnlyRequests(kAllowed));
}

TEST_P(V4l2CaptureTest, StreamOnFailureLeavesNothingStreaming) {
  V4l2Capture capture(&device);
  ASSERT_TRUE(capture.Open("/dev/video22", kStereo, error)) << error;
  device.streamon_errno = EIO;
  EXPECT_FALSE(capture.Start(error));
  EXPECT_NE(error.find("stream on"), std::string::npos) << error;
  CapturedFrame frame;
  EXPECT_EQ(capture.Next(0, frame, error), -1);
  capture.Close();
  EXPECT_EQ(device.stream_offs, 0);
  EXPECT_TRUE(device.fds.empty());
}

TEST_P(V4l2CaptureTest, Nv12WithoutExport) {
  device.stride = 1920;
  device.buffer_bytes = 1920 * 1080 * 3 / 2;
  V4l2Capture capture(&device);
  ASSERT_TRUE(capture.Open("/dev/video53", {1920, 1080, PixelFormat::kNv12, 4, false}, error)) << error;
  EXPECT_TRUE(capture.ExportedBuffers().empty());
  ASSERT_TRUE(capture.Start(error));
  // A driver that leaves bytesused at 0 still delivers a full frame.
  device.Push(9, 0);
  CapturedFrame frame;
  ASSERT_EQ(capture.Next(0, frame, error), 1);
  EXPECT_EQ(frame.bytes, 1920U * 1080 * 3 / 2);
  EXPECT_EQ(frame.dmabuf, -1);
  EXPECT_TRUE(device.OnlyRequests(kAllowed));
}

INSTANTIATE_TEST_SUITE_P(BufferTypes, V4l2CaptureTest, ::testing::Values(true, false),
                         [](const ::testing::TestParamInfo<bool>& info) {
                           return info.param ? "MultiPlanar" : "SinglePlanar";
                         });

TEST(V4l2Capture, RefusesANodeThatCannotCapture) {
  FakeVideoDevice device;
  device.caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;
  V4l2Capture capture(&device);
  std::string error;
  EXPECT_FALSE(capture.Open("/dev/video22", kStereo, error));
  EXPECT_NE(error.find("no streaming capture node"), std::string::npos) << error;
  EXPECT_EQ(device.requests, (std::vector<unsigned long>{VIDIOC_QUERYCAP}));
  EXPECT_TRUE(device.fds.empty());

  device.open_errno = EBUSY;
  EXPECT_FALSE(capture.Open("/dev/video22", kStereo, error));
  EXPECT_NE(error.find("open /dev/video22"), std::string::npos) << error;
}

}  // namespace
}  // namespace tron_vision
