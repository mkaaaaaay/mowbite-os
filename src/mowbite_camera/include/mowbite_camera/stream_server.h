#ifndef MOWBITE_CAMERA_STREAM_SERVER_H
#define MOWBITE_CAMERA_STREAM_SERVER_H

#include <string>
#include <vector>

#include "mowbite_camera/child.h"
#include "mowbite_camera/http_client.h"
#include "mowbite_camera/stream_supervisor.h"

namespace mowbite_camera {

// How long one call of the API may take, in s: it holds up the thread that asks.
constexpr double kStreamApiSeconds = 0.5;
// Limits of the stream server: the Go runtime keeps its heap below this, and when memory runs out the kernel ends
// it before the nodes.
constexpr char kStreamMemoryLimit[] = "GOMEMLIMIT=64MiB";
constexpr int kStreamNice = 10;
constexpr int kStreamOomScoreAdj = 1000;

// MediaMTX as a child of this process, started with its configuration file in the folder of that file.
class MediaServer : public StreamServer {
 public:
  MediaServer(std::string program, std::string directory);

  bool Installed() override;
  bool Start(std::string* error) override;
  bool Running(std::string* how) override;
  void Stop() override;

 private:
  const std::string program_;
  const std::string directory_;
  Child child_;
};

// The ids of the sessions in the answer of /v3/webrtcsessions/list; false when it is not such an answer or has
// more pages than the first.
bool ParseSessions(const std::string& body, std::vector<std::string>* ids);

// The WebRTC sessions the stream server has now; false without an answer.
bool StreamSessions(const LocalServer& api, std::vector<std::string>* ids);
// Ends a session; false when the server did not confirm it.
bool KickStreamSession(const LocalServer& api, const std::string& id);

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_STREAM_SERVER_H
