#ifndef MOWBITE_CAMERA_STREAM_CONFIG_H
#define MOWBITE_CAMERA_STREAM_CONFIG_H

#include <cstdint>
#include <map>
#include <string>

namespace mowbite_camera {

// The name of the configuration file in the runtime folder.
constexpr char kStreamConfigFile[] = "mediamtx.yml";

// What the stream server (MediaMTX) is configured with.
struct StreamConfig {
  // Where it asks before it lets a request in: the listener of the node for it on 127.0.0.1.
  uint16_t auth_port = 0;
  // Its API on 127.0.0.1, and what it opens for the viewers: WHEP over HTTP and ICE over UDP.
  uint16_t api_port = 9997;
  uint16_t webrtc_port = 8889;
  uint16_t ice_port = 8189;
  // Camera id -> its RTSP address on this host.
  std::map<std::string, std::string> sources;
};

// The configuration file: only WebRTC towards the viewers, every request asked at auth_port, one path per camera
// that pulls its source while someone watches. An id or address that is not plain text is left out.
std::string StreamConfigText(const StreamConfig& config);

// The folder for the files of this run: mowbite_camera in `parameter`, else in the first folder of
// $RUNTIME_DIRECTORY, else in /tmp; made if missing, with mode 0700. Empty with *error when it is not a folder of
// this user.
std::string RuntimeDir(const std::string& parameter, std::string* error);

// Writes dir/name with mode 0600 by renaming a new file over the old one, so a reader sees one of the two.
bool ReplaceFile(const std::string& dir, const std::string& name, const std::string& text, std::string* error);

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_STREAM_CONFIG_H
