#include <gtest/gtest.h>

#include <filesystem>

#include "fake_video_device.h"
#include "test_util.h"
#include "tron_vision/stereo_camera.h"

namespace tron_vision {
namespace {

// Video nodes in a temporary directory, each with its name and the link to its platform device.
struct Sysfs {
  Sysfs() {
    Node("video22", "stream_cif_mipi_id0", "rkcif-mipi-lvds3");
    Node("video23", "stream_cif_mipi_id1", "rkcif-mipi-lvds3");
    Node("video11", "stream_cif_mipi_id0", "rkcif-mipi-lvds2");
    Node("video53", "rkisp_mainpath", "rkisp0-vir0");
    Node("v4l-subdev10", "m03_f_xc9080 8-001b", "8-001b");
  }
  void Node(const std::string& node, const std::string& name, const std::string& device) {
    std::filesystem::create_directories(tmp.Path() + "/" + node);
    std::filesystem::create_directories(tmp.Path() + "/devices/" + device);
    tmp.Write(node + "/name", name + "\n");
    std::filesystem::create_directory_symlink("../devices/" + device, tmp.Path() + "/" + node + "/device");
  }
  test::TempDir tmp;
};

TEST(StereoCamera, FindsTheNodeByDeviceAndName) {
  Sysfs s;
  EXPECT_EQ(FindVideoNode(s.tmp.Path(), "rkcif-mipi-lvds3", "stream_cif_mipi_id0"), "/dev/video22");
  EXPECT_EQ(FindVideoNode(s.tmp.Path(), "rkcif-mipi-lvds2", "stream_cif_mipi_id0"), "/dev/video11");
  EXPECT_EQ(FindVideoNode(s.tmp.Path(), "rkcif-mipi-lvds4", "stream_cif_mipi_id0"), "");
  EXPECT_EQ(FindVideoNode(s.tmp.Path(), "rkcif-mipi-lvds3", "stream_cif_mipi_id9"), "");
  EXPECT_EQ(FindVideoNode(s.tmp.Path() + "/none", "rkcif-mipi-lvds3", "stream_cif_mipi_id0"), "");
  // Subdevs are never taken for video nodes.
  EXPECT_EQ(FindVideoNode(s.tmp.Path(), "8-001b", "m03_f_xc9080 8-001b"), "");
  // Two nodes that fit are no answer.
  s.Node("video24", "stream_cif_mipi_id0", "rkcif-mipi-lvds3");
  EXPECT_EQ(FindVideoNode(s.tmp.Path(), "rkcif-mipi-lvds3", "stream_cif_mipi_id0"), "");
}

TEST(StereoCamera, StreamsBothEyesWithExportedBuffers) {
  Sysfs s;
  test::FakeVideoDevice device;
  StereoCameraConfig config;
  config.sysfs = s.tmp.Path();
  TronStereoCamera camera(config, &device);
  std::string error;
  ASSERT_TRUE(camera.Open(error)) << error;
  EXPECT_EQ(device.opened, (std::vector<std::string>{"/dev/video22"}));
  EXPECT_EQ(device.stream_ons, 0);
  ASSERT_TRUE(camera.Start(error)) << error;
  device.Push(40, 2560 * 480);
  CapturedFrame frame;
  ASSERT_EQ(camera.Next(0, frame, error), 1);
  EXPECT_EQ(frame.bytes, 2560U * 480);
  EXPECT_EQ(frame.dmabuf, 200);
  // Stop ends the stream and keeps the buffers, Close frees them.
  camera.Stop();
  EXPECT_EQ(device.stream_offs, 1);
  EXPECT_EQ(device.fds.size(), 5U);
  camera.Close();
  EXPECT_TRUE(device.fds.empty());
  EXPECT_EQ(device.stream_offs, 1);
  EXPECT_TRUE(device.OnlyRequests({VIDIOC_QUERYCAP, VIDIOC_S_FMT, VIDIOC_REQBUFS, VIDIOC_QUERYBUF, VIDIOC_EXPBUF,
                                   VIDIOC_QBUF, VIDIOC_DQBUF, VIDIOC_STREAMON, VIDIOC_STREAMOFF}));
}

TEST(StereoCamera, RefusesWithoutItsNodeOrWhileAnotherProcessHasIt) {
  Sysfs s;
  test::FakeVideoDevice device;
  StereoCameraConfig config;
  config.sysfs = s.tmp.Path();
  config.capture_device = "rkcif-mipi-lvds9";
  std::string error;
  TronStereoCamera missing(config, &device);
  EXPECT_FALSE(missing.Open(error));
  EXPECT_EQ(error, "no single video node stream_cif_mipi_id0 of rkcif-mipi-lvds9");

  config.capture_device = "rkcif-mipi-lvds3";
  config.processes_using = [](const std::string& node) {
    return node == "/dev/video22" ? std::vector<int>{4711} : std::vector<int>{};
  };
  TronStereoCamera busy(config, &device);
  EXPECT_FALSE(busy.Open(error));
  EXPECT_EQ(error, "/dev/video22 is open in process 4711");
  EXPECT_TRUE(device.opened.empty());
  EXPECT_FALSE(busy.Start(error));
}

}  // namespace
}  // namespace tron_vision
