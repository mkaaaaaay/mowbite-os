#ifndef TRON_VISION_RTSP_SERVER_H
#define TRON_VISION_RTSP_SERVER_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "tron_vision/h264_rtp.h"

namespace tron_vision {

struct RtspConfig {
  // 0 takes any free port.
  uint16_t port = 8555;
  // One stream per path, numbered in this order.
  std::vector<std::string> paths = {"stereo"};
  // A client that sends nothing for this long is dropped; a player sends receiver reports and keep-alives.
  double idle_timeout_s = 60.0;
  // Longest time for one picture, or one answer, to go out to a client that reads too slowly, then it is dropped.
  int send_timeout_ms = 200;
};

// Session description of the one H.264 stream with its track "trackID=0".
std::string StreamSdp(const ParameterSets& sets, const std::string& name);

// RTSP 1.0 server for one H.264 stream per path and one client per path, on the loopback address only. It answers
// OPTIONS, DESCRIBE, SETUP with RTP over the RTSP connection, PLAY, GET_PARAMETER and TEARDOWN; RTCP from the client
// is read and dropped. A connection belongs to the path of its first DESCRIBE, SETUP or PLAY; there are at most as
// many connections as paths, one per path. One thread drives it through Poll() and SendRtp(); `path` is the index in
// RtspConfig::paths.
class RtspServer {
 public:
  RtspServer(RtspConfig config, std::function<double()> now);
  ~RtspServer();
  RtspServer(const RtspServer&) = delete;
  RtspServer& operator=(const RtspServer&) = delete;

  bool Listen(std::string& error);
  void Close();
  // The port in use, 0 while not listening.
  uint16_t Port() const {
    return port_;
  }

  // From now on DESCRIBE offers the stream of the path with these parameter sets.
  void SetAvailable(const ParameterSets& sets, size_t path = 0);
  // From now on DESCRIBE, SETUP and PLAY of the path answer 503, and its client is dropped.
  void SetUnavailable(size_t path = 0);

  // Waits up to timeout_ms for a connection or data, then serves what has come.
  void Poll(int timeout_ms);
  // The client of the path has sent PLAY and is still there.
  bool Playing(size_t path = 0) const;
  // The RTP packets of one picture to the playing client of the path, all within send_timeout_ms; false without one,
  // also when it had to be dropped.
  bool SendRtp(const std::vector<std::vector<uint8_t>>& packets, size_t path = 0);
  void DropClient(size_t path = 0);

 private:
  struct Request {
    std::string method;
    std::string url;
    std::map<std::string, std::string> headers;
  };

  static constexpr size_t kNoPath = static_cast<size_t>(-1);

  struct Mount {
    bool available = false;
    ParameterSets sets;
  };
  struct Connection {
    int fd = -1;
    // Index of its path, kNoPath until the first request that names one.
    size_t path = kNoPath;
    std::string input;
    // Empty until SETUP.
    std::string session;
    bool playing = false;
    uint8_t rtp_channel = 0;
    double last_activity = 0.0;
  };
  void Accept();
  void Read(Connection& c);
  // Takes complete requests and interleaved frames off the input; false once the client is gone.
  bool Consume(Connection& c);
  void Handle(Connection& c, const Request& request);
  void Reply(Connection& c, const Request& request, int code, const std::string& headers = "",
             const std::string& body = "");
  // Index of the path the URL names, kNoPath if none.
  size_t PathOf(const std::string& url) const;
  // The connection of the path, null without one.
  Connection* ClientOf(size_t path);
  const Connection* ClientOf(size_t path) const;
  void Drop(Connection& c);
  bool SendAll(Connection& c, const uint8_t* data, size_t size, std::chrono::steady_clock::time_point give_up);
  std::chrono::steady_clock::time_point SendDeadline() const;

  RtspConfig config_;
  std::function<double()> now_;
  int listen_fd_ = -1;
  uint16_t port_ = 0;
  std::vector<Mount> mounts_;
  // One slot per path; a free slot has fd -1.
  std::vector<Connection> connections_;
  uint32_t session_counter_;
  std::vector<uint8_t> frame_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_RTSP_SERVER_H
