#ifndef MOWBITE_CAMERA_TEST_TEST_UTIL_H
#define MOWBITE_CAMERA_TEST_TEST_UTIL_H

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <system_error>

#include "mowbite_camera/camera_list.h"
#include "mowbite_camera/seats.h"

namespace mowbite_camera {

inline CameraEntry Obstacle() {
  CameraEntry entry;
  entry.id = "oa_left";
  entry.name = "Obstacle camera";
  entry.process = "vision";
  entry.kind = "obstacle";
  entry.position = "front";
  entry.width = 640;
  entry.height = 480;
  entry.detections = true;
  entry.protocols = {"mjpeg"};
  entry.available = true;
  return entry;
}

// The obstacle camera with webrtc first, as the Tron has it while its encoder runs.
inline CameraEntry ObstacleWithWebrtc() {
  CameraEntry entry = Obstacle();
  entry.protocols = {"webrtc", "mjpeg"};
  entry.source = "rtsp://127.0.0.1:8556/oa_left";
  return entry;
}

inline CameraEntry Stereo() {
  CameraEntry entry;
  entry.id = "stereo";
  entry.name = "Stereo camera";
  entry.process = "stereo";
  entry.kind = "stereo";
  entry.width = 1280;
  entry.height = 720;
  entry.protocols = {"webrtc"};
  entry.source = "rtsp://127.0.0.1:8555/stereo";
  entry.available = true;
  return entry;
}

// Bytes that count up, so every token is another one.
inline RandomBytes Counting() {
  auto next = std::make_shared<uint8_t>(0);
  return [next](uint8_t* out, size_t size) {
    for (size_t i = 0; i < size; ++i) out[i] = (*next)++;
    return true;
  };
}

// A folder of its own in /tmp, removed with everything in it at the end.
class TempDir {
 public:
  TempDir() {
    char pattern[] = "/tmp/mowbite_camera_test_XXXXXX";
    path_ = std::filesystem::canonical(::mkdtemp(pattern));
  }
  ~TempDir() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;

  const std::filesystem::path& path() const {
    return path_;
  }

 private:
  std::filesystem::path path_;
};

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_TEST_TEST_UTIL_H
