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
  std::string path = "stereo";
  // A client that sends nothing for this long is dropped; a player sends receiver reports and keep-alives.
  double idle_timeout_s = 60.0;
  // Longest time for one picture, or one answer, to go out to a client that reads too slowly, then it is dropped.
  int send_timeout_ms = 200;
};

// Session description of the one H.264 stream with its track "trackID=0".
std::string StreamSdp(const ParameterSets& sets, const std::string& name);

// RTSP 1.0 server for one H.264 stream and one client at a time, on the loopback address only. It answers OPTIONS,
// DESCRIBE, SETUP with RTP over the RTSP connection, PLAY, GET_PARAMETER and TEARDOWN; RTCP from the client is read
// and dropped. One thread drives it through Poll() and SendRtp().
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

  // From now on DESCRIBE offers the stream with these parameter sets.
  void SetAvailable(const ParameterSets& sets);
  // From now on DESCRIBE, SETUP and PLAY answer 503, and a connected client is dropped.
  void SetUnavailable();

  // Waits up to timeout_ms for a connection or data, then serves what has come.
  void Poll(int timeout_ms);
  // A client has sent PLAY and is still there.
  bool Playing() const {
    return playing_;
  }
  // The RTP packets of one picture to the playing client, all within send_timeout_ms; false without one, also when
  // it had to be dropped.
  bool SendRtp(const std::vector<std::vector<uint8_t>>& packets);
  void DropClient();

 private:
  struct Request {
    std::string method;
    std::string url;
    std::map<std::string, std::string> headers;
  };

  void Accept();
  void Read();
  // Takes complete requests and interleaved frames off the input; false once the client is gone.
  bool Consume();
  void Handle(const Request& request);
  void Reply(const Request& request, int code, const std::string& headers = "", const std::string& body = "");
  bool OwnPath(const std::string& url) const;
  bool SendAll(const uint8_t* data, size_t size, std::chrono::steady_clock::time_point give_up);
  std::chrono::steady_clock::time_point SendDeadline() const;

  RtspConfig config_;
  std::function<double()> now_;
  int listen_fd_ = -1;
  int client_fd_ = -1;
  uint16_t port_ = 0;
  bool available_ = false;
  ParameterSets sets_;
  std::string input_;
  // Empty until SETUP.
  std::string session_;
  uint32_t session_counter_;
  bool playing_ = false;
  uint8_t rtp_channel_ = 0;
  double last_activity_ = 0.0;
  std::vector<uint8_t> frame_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_RTSP_SERVER_H
