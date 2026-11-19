#include <gtest/gtest.h>

#include <filesystem>

#include "test_util.h"
#include "tron_vision/tron_camera.h"

namespace tron_vision {
namespace {

// sysfs of the Tron as read 09.11. (M2b) and 19.11., nodes in a temporary directory, video nodes with the link to
// their platform device.
struct Sysfs {
  Sysfs() {
    Node("v4l-subdev11", "rockchip-mipi-csi2");
    Node("v4l-subdev13", "m01_b_gc2093 3-0037");
    Node("v4l-subdev2", "m00_b_gc2093 5-0037");
    Node("video53", "rkisp_mainpath", "rkisp1-vir0");
    Node("video44", "rkisp_mainpath", "rkisp0-vir0");
    Node("video45", "rkisp_selfpath", "rkisp0-vir0");
  }
  void Node(const std::string& node, const std::string& name, const std::string& device = "") {
    std::filesystem::create_directories(tmp.Path() + "/" + node);
    tmp.Write(node + "/name", name + "\n");
    if (device.empty()) return;
    std::filesystem::create_directories(tmp.Path() + "/devices/" + device);
    std::filesystem::create_directory_symlink("../devices/" + device, tmp.Path() + "/" + node + "/device");
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

// Each side camera finds the main path of its own ISP, whatever its number.
TEST(TronCamera, FindsTheVideoNodeOfItsIsp) {
  Sysfs s;
  EXPECT_EQ(FindVideoNode(s.tmp.Path(), TronCameraConfig{}.capture_device, "rkisp_mainpath"), "/dev/video53");
  EXPECT_EQ(FindVideoNode(s.tmp.Path(), kRightCaptureDevice, "rkisp_mainpath"), "/dev/video44");
  EXPECT_EQ(FindVideoNode(s.tmp.Path(), "rkisp2-vir0", "rkisp_mainpath"), "");
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
  c.capture_device = "rkisp2-vir0";
  TronCamera other(c);
  EXPECT_FALSE(other.Start(error));
  EXPECT_EQ(error, "no single video node rkisp_mainpath of rkisp2-vir0");
}

TEST(TronCamera, StartRefusesADeviceOpenElsewhere) {
  Sysfs s;
  TronCameraConfig c;
  c.sysfs = s.tmp.Path();
  std::vector<std::string> asked;
  c.processes_using = [&asked](const std::string& device) {
    asked.push_back(device);
    return device == "/dev/video53" ? std::vector<int>{4711} : std::vector<int>{};
  };
  TronCamera camera(c);
  std::string error;
  EXPECT_FALSE(camera.Start(error));
  EXPECT_EQ(error, "/dev/video53 is open in process 4711");
  EXPECT_EQ(asked, (std::vector<std::string>{"/dev/v4l-subdev13", "/dev/video53"}));
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
