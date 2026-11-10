// Runs the stereo stream without the Tron, for a trial against a real RTSP client such as MediaMTX: the camera is
// the driver model of the tests at 25 fps, the encoder the stand-in library, which plays the H.264 file named in
// FAKE_MPP_STREAM. Not a test of its own; it prints the state once a second.
//   tron_stereo_trial <fake media library> <port> <seconds>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <thread>

#include "fake_video_device.h"
#include "test_util.h"
#include "tron_vision/stereo_camera.h"
#include "tron_vision/stereo_loop.h"

namespace {

double Now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::fprintf(stderr, "usage: %s <fake media library> <port> <seconds>\n", argv[0]);
    return 2;
  }
  using namespace tron_vision;
  test::TempDir dir;
  std::filesystem::create_directories(dir.Path() + "/sys/video22");
  std::filesystem::create_directories(dir.Path() + "/sys/devices/rkcif-mipi-lvds3");
  std::filesystem::create_directories(dir.Path() + "/markers");
  dir.Write("sys/video22/name", "stream_cif_mipi_id0\n");
  std::filesystem::create_directory_symlink("../devices/rkcif-mipi-lvds3", dir.Path() + "/sys/video22/device");
  dir.Write("stereo", "configured\n");

  // A frame every 40 ms of real time while the node streams.
  test::FakeVideoDevice device;
  device.frame_period_us = 40000;
  double due = 0.0;
  int stream = 0;
  device.on_wait = [&device, &due, &stream](int timeout_ms) {
    if (!device.streaming) return;
    const double now = Now();
    if (stream != device.stream_ons) {
      stream = device.stream_ons;
      due = now + 0.04;
    }
    const double wait = std::min(due - now, timeout_ms / 1000.0);
    if (wait > 0.0) std::this_thread::sleep_for(std::chrono::duration<double>(wait));
    if (Now() >= due) {
      device.Push(0x80, device.buffer_bytes);
      due += 0.04;
    }
  };

  StereoCameraConfig camera_config;
  camera_config.sysfs = dir.Path() + "/sys";
  TronStereoCamera camera(camera_config, &device);
  MppEncoder encoder(argv[1]);
  RtspConfig rtsp;
  rtsp.port = static_cast<uint16_t>(std::atoi(argv[2]));
  RtspServer server(rtsp, Now);
  std::string error;
  if (!server.Listen(error)) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  StereoConfig config;
  config.setup_file = dir.Path() + "/stereo";
  config.marker_dir = dir.Path() + "/markers";
  config.idle_wait_ms = 50;
  CrashGuard guard(config.marker_dir, dir.Path() + "/crash", config.boot_id_file);
  StereoLoop loop(config, camera, encoder, server, guard, Now);

  const double end = Now() + std::atof(argv[3]);
  double next_line = 0.0;
  while (Now() < end) {
    loop.SetMowerStatus({true, Now(), false, false});
    if (!loop.Step()) break;
    if (Now() >= next_line) {
      next_line = Now() + 1.0;
      const StereoStatus& s = loop.Status();
      std::printf("%s %s | %.1f fps %.0f kbit/s, %llu frames, %llu lost, stream on %d off %d\n", StateName(s.state),
                  s.detail.c_str(), s.fps, s.kbps, static_cast<unsigned long long>(s.frames),
                  static_cast<unsigned long long>(s.lost), device.stream_ons, device.stream_offs);
      std::fflush(stdout);
    }
  }
  return loop.Status().exit_code;
}
