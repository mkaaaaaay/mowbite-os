#ifndef MOWBITE_CAMERA_CAMERA_SERVICE_H
#define MOWBITE_CAMERA_CAMERA_SERVICE_H

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_camera/camera_list.h"
#include "mowbite_camera/http_server.h"
#include "mowbite_camera/route.h"
#include "mowbite_camera/seats.h"
#include "mowbite_camera/stream_config.h"

namespace mowbite_camera {

struct RpcOutcome {
  bool ok = true;
  Json result;
  int code = 0;
  std::string message;
  // Details of a refusal, null for none.
  Json data;
};

struct ServiceConfig {
  uint16_t mjpeg_port = 8890;
  // Where the stream server takes WHEP.
  uint16_t webrtc_port = StreamConfig().webrtc_port;
  // The host of the urls; empty: the IPv4 address of the default route at the time of camera.stream.
  std::string public_host;
};

struct Clocks {
  // A steady clock (CLOCK_MONOTONIC, as the exit notes have it) and the time of day as unix time, both in s.
  std::function<double()> steady;
  std::function<double()> wall;
};

// What is left to do after calls into the service.
struct Effects {
  // The details of CAMERA_CHANGED events.
  std::vector<Json> events;
  // Connections of the HTTP server and sessions of the stream server to end.
  std::vector<uint64_t> close;
  std::vector<std::string> kick;
};

// camera.list and camera.stream over the cameras of hw/cameras, the places of their viewers, and who may connect:
// viewers of MJPEG at the HTTP server and what the stream server asks before it lets someone in. Every method may
// be called from any thread.
class CameraService {
 public:
  CameraService(ServiceConfig config, Clocks clocks, const RandomBytes& random = SystemRandom,
                std::function<std::string()> route = DefaultRouteAddress);

  static const std::vector<std::string>& Methods();

  // An entry of hw/cameras; what was wrong with it, empty when nothing was.
  std::string OnCamera(const CameraEntry& entry);
  // The exit note of a process: when it ended (its mono) and whether it stays down.
  void OnProcessEnded(const std::string& process, double mono, bool down);
  // Why webrtc does not work now, a reason of the cameras with a few words; an empty reason: it works.
  void SetWebrtc(const std::string& reason, const std::string& detail);
  // nullopt for a method of another node.
  std::optional<RpcOutcome> Call(const std::string& method, const Json& params);
  // Tokens nobody used in time give their place back.
  void Tick();
  Effects TakeEffects();

  bool HasCameras() const;
  std::vector<std::string> Processes() const;
  std::map<std::string, std::string> WebrtcSources() const;
  // The cameras someone watches over mjpeg.
  std::vector<std::string> MjpegCameras() const;

  // The sessions of the stream server that hold a place.
  std::vector<std::string> WebrtcSessions() const;
  // The sessions the stream server has now; a place whose session is missing twice in a row is free.
  void OnSessions(const std::vector<std::string>& alive);
  // The stream server ended and its sessions with it.
  void OnStreamServerGone();

  // GET /camera/<id>.mjpg?t=<token> of a viewer, and the end of its connection.
  HttpAnswer OnViewer(const HttpRequest& request);
  void OnViewerGone(uint64_t connection);
  // POST /auth of the stream server: 200 lets the request in, anything else keeps it out.
  HttpAnswer OnAuth(const HttpRequest& request);

  // What the node sends with its own calls of the stream server; the password is empty without random numbers.
  const std::string& api_user() const {
    return api_user_;
  }
  const std::string& api_password() const {
    return api_password_;
  }

 private:
  RpcOutcome Stream(const Json& params);
  void CollectLocked();

  const ServiceConfig config_;
  const Clocks clocks_;
  const std::function<std::string()> route_;
  const std::string api_user_ = "mowbite";
  const std::string api_password_;

  mutable std::mutex mutex_;
  CameraList list_;
  Seats seats_;
  // Session -> how often in a row the stream server did not have it.
  std::map<std::string, int> missing_;
  Effects effects_;
};

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_CAMERA_SERVICE_H
