#include <gtest/gtest.h>
#include <linux/videodev2.h>

#include "fake_video_device.h"
#include "test_util.h"
#include "tron_vision/rear_camera.h"

namespace tron_vision {
namespace {

using test::FakeVideoDevice;

// The UVC camera as the driver gives it: MJPEG on a single-planar node, automatic exposure.
struct RearRig {
  RearRig() {
    device.caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    device.stride = 0;
    device.buffer_bytes = 1920 * 1080 * 2;
    device.controls[V4L2_CID_EXPOSURE_AUTO] = {0, 3, V4L2_EXPOSURE_APERTURE_PRIORITY, 0};
    device.controls[V4L2_CID_EXPOSURE_ABSOLUTE] = {1, 5000, 313, V4L2_CTRL_FLAG_INACTIVE};
    device.controls[V4L2_CID_GAIN] = {0, 63, 10, 0};
    config.device = dir.Write("mowbite-rear", "");
  }
  test::TempDir dir;
  FakeVideoDevice device;
  RearCameraConfig config;
};

TEST(UvcRearCamera, OpensMjpegAtFullSize) {
  RearRig rig;
  UvcRearCamera camera(rig.config, &rig.device);
  std::string error;
  ASSERT_TRUE(camera.Present(error));
  ASSERT_TRUE(camera.Open(error)) << error;
  EXPECT_EQ(rig.device.opened, std::vector<std::string>{rig.config.device});
  ASSERT_TRUE(camera.Start(error)) << error;
  ASSERT_TRUE(rig.device.PushBytes({0xff, 0xd8, 0x01, 0xff, 0xd9}));
  CapturedFrame frame;
  ASSERT_EQ(camera.Next(0, frame, error), 1);
  EXPECT_EQ(frame.bytes, 5U);
  camera.Close();
  EXPECT_TRUE(rig.device.fds.empty());
  // Without a single control written: the exposure was never asked for.
  EXPECT_TRUE(rig.device.writes.empty());
}

TEST(UvcRearCamera, MissingDevice) {
  RearRig rig;
  rig.config.device = rig.dir.Path() + "/none";
  UvcRearCamera camera(rig.config, &rig.device);
  std::string error;
  EXPECT_FALSE(camera.Present(error));
  EXPECT_EQ(error, "no " + rig.config.device);
  EXPECT_FALSE(camera.Open(error));
  EXPECT_TRUE(rig.device.opened.empty());
}

TEST(UvcRearCamera, OpenRefusedWhileAnotherProcessHasIt) {
  RearRig rig;
  rig.config.processes_using = [](const std::string&) { return std::vector<int>{4711}; };
  UvcRearCamera camera(rig.config, &rig.device);
  std::string error;
  EXPECT_FALSE(camera.Open(error));
  EXPECT_NE(error.find("open in process 4711"), std::string::npos) << error;
  EXPECT_TRUE(rig.device.opened.empty());
}

// Manual exposure with the range read from the controls; Close gives back the values and the automatic mode last.
TEST(UvcRearCamera, ManualExposureAndBack) {
  RearRig rig;
  UvcRearCamera camera(rig.config, &rig.device);
  std::string error;
  ASSERT_TRUE(camera.Open(error)) << error;
  ExposureRange range;
  ExposureSetting now;
  ASSERT_TRUE(camera.ManualExposure(range, now, error)) << error;
  EXPECT_EQ(range.exposure_min, 1);
  EXPECT_EQ(range.exposure_max, 5000);
  EXPECT_EQ(range.gain_min, 0);
  EXPECT_EQ(range.gain_max, 63);
  EXPECT_EQ(now.exposure, 313);
  EXPECT_EQ(now.gain, 10);
  ASSERT_TRUE(camera.SetExposure({40, 20}, error)) << error;
  camera.Close();
  const std::vector<std::pair<uint32_t, int>> writes = {
      {V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL}, {V4L2_CID_EXPOSURE_ABSOLUTE, 40}, {V4L2_CID_GAIN, 20},
      {V4L2_CID_EXPOSURE_ABSOLUTE, 313},          {V4L2_CID_GAIN, 10},              {V4L2_CID_EXPOSURE_AUTO, 3}};
  EXPECT_EQ(rig.device.writes, writes);
  EXPECT_TRUE(rig.device.fds.empty());
}

TEST(UvcRearCamera, WithoutGainOnlyTheExposureTime) {
  RearRig rig;
  rig.device.controls.erase(V4L2_CID_GAIN);
  UvcRearCamera camera(rig.config, &rig.device);
  std::string error;
  ASSERT_TRUE(camera.Open(error)) << error;
  ExposureRange range;
  ExposureSetting now;
  ASSERT_TRUE(camera.ManualExposure(range, now, error)) << error;
  EXPECT_EQ(range.gain_min, 1);
  EXPECT_EQ(range.gain_max, 1);
  ASSERT_TRUE(camera.SetExposure({40, 1}, error)) << error;
  EXPECT_EQ(rig.device.writes.back(), (std::pair<uint32_t, int>{V4L2_CID_EXPOSURE_ABSOLUTE, 40}));
}

TEST(UvcRearCamera, NoManualExposureLeavesTheCameraAlone) {
  RearRig rig;
  rig.device.controls.erase(V4L2_CID_EXPOSURE_ABSOLUTE);
  UvcRearCamera camera(rig.config, &rig.device);
  std::string error;
  ASSERT_TRUE(camera.Open(error)) << error;
  ExposureRange range;
  ExposureSetting now;
  EXPECT_FALSE(camera.ManualExposure(range, now, error));
  EXPECT_FALSE(camera.SetExposure({40, 20}, error));
  camera.Close();
  EXPECT_TRUE(rig.device.writes.empty());
}

}  // namespace
}  // namespace tron_vision
