#include <gtest/gtest.h>

#include <filesystem>

#include "test_util.h"
#include "tron_vision/tron_camera.h"

namespace tron_vision {
namespace {

// sysfs of the Tron as read 09.11. (M2b), nodes in a temporary directory.
struct Sysfs {
  Sysfs() {
    Node("v4l-subdev11", "rockchip-mipi-csi2");
    Node("v4l-subdev13", "m01_b_gc2093 3-0037");
    Node("v4l-subdev14", "m00_b_gc2093 7-0037");
    Node("video53", "rkisp_mainpath");
    Node("video44", "rkisp_mainpath");
  }
  void Node(const std::string& node, const std::string& name) {
    std::filesystem::create_directories(tmp.Path() + "/" + node);
    tmp.Write(node + "/name", name + "\n");
  }
  test::TempDir tmp;
};

TEST(TronCamera, FindsTheSubdevByName) {
  Sysfs s;
  EXPECT_EQ(FindSubdev(s.tmp.Path(), "m01_b_gc2093"), "/dev/v4l-subdev13");
  EXPECT_EQ(FindSubdev(s.tmp.Path(), "mo_simor"), "");
  // Ambiguous prefixes find nothing.
  EXPECT_EQ(FindSubdev(s.tmp.Path(), "m0"), "");
  EXPECT_EQ(FindSubdev(s.tmp.Path() + "/none", "m01_b_gc2093"), "");
  // Video nodes are never taken for subdevs.
  EXPECT_EQ(FindSubdev(s.tmp.Path(), "rkisp_mainpath"), "");
}

TEST(TronCamera, NodeName) {
  Sysfs s;
  EXPECT_EQ(NodeName(s.tmp.Path(), "/dev/video53"), "rkisp_mainpath");
  EXPECT_EQ(NodeName(s.tmp.Path(), "/dev/video99"), "");
}

TEST(TronCamera, StartRefusesWithoutTheSensorOrWithAnotherNode) {
  Sysfs s;
  TronCameraConfig c;
  c.sysfs = s.tmp.Path();
  c.sensor_name = "m02_b_gc2093";
  TronCamera missing(c);
  std::string error;
  EXPECT_FALSE(missing.Start(error));
  EXPECT_NE(error.find("no single subdev"), std::string::npos) << error;

  c.sensor_name = "m01_b_gc2093";
  c.video_device = "/dev/video52";
  TronCamera other(c);
  EXPECT_FALSE(other.Start(error));
  EXPECT_NE(error.find("not rkisp_mainpath"), std::string::npos) << error;
}

TEST(TronCamera, ExposureBeforeStartIsKept) {
  TronCamera camera{TronCameraConfig{}};
  std::string error;
  EXPECT_TRUE(camera.SetExposure({1120, 2048}, error));
  Nv12View frame;
  EXPECT_EQ(camera.Next(0, frame, error), -1);
}

}  // namespace
}  // namespace tron_vision
