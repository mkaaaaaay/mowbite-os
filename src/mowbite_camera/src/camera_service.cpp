#include "mowbite_camera/camera_service.h"

#include <algorithm>
#include <cmath>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/json_rpc.h"

namespace mowbite_camera {

namespace {

// The group of x-reasons whose reasons the refusals of camera.stream carry.
constexpr char kRefused[] = "CAMERA_REFUSED";
constexpr char kPathPrefix[] = "/camera/";
constexpr char kPathSuffix[] = ".mjpg";
// A session the stream server lacks this often in a row is over.
constexpr int kMissingLimit = 2;

RpcOutcome Ok(Json result) {
  return {true, std::move(result), 0, "", Json()};
}

RpcOutcome Invalid(std::string message) {
  return {false, Json(), mowbite_bridge::kInvalidParams, std::move(message), Json()};
}

RpcOutcome Refuse(const std::string& reason, const std::string& message) {
  const mowbite_api::Rating rating = mowbite_api::RateReason(kRefused, reason);
  const Json details = {{"event", kRefused},
                        {"reason", reason},
                        {"message", message},
                        {"severity", rating.severity},
                        {"resume", rating.resume}};
  return {false, Json(), mowbite_bridge::kInvalidRequest, message, details};
}

// A text field of the stream server's request, empty when it is missing or null.
std::string Text(const Json& json, const char* key) {
  const auto found = json.find(key);
  return found != json.end() && found->is_string() ? found->get<std::string>() : "";
}

}  // namespace

CameraService::CameraService(ServiceConfig config, Clocks clocks, const RandomBytes& random,
                             std::function<std::string()> route)
    : config_(std::move(config)), clocks_(std::move(clocks)), route_(std::move(route)),
      api_password_(RandomHex(random, kTokenBytes)), seats_(random) {}

const std::vector<std::string>& CameraService::Methods() {
  static const std::vector<std::string> methods = {"camera.list", "camera.stream"};
  return methods;
}

std::string CameraService::OnCamera(const CameraEntry& entry) {
  const std::lock_guard<std::mutex> lock(mutex_);
  std::string problem;
  list_.Update(entry, clocks_.steady(), &problem);
  CollectLocked();
  return problem;
}

void CameraService::OnProcessEnded(const std::string& process, double mono, bool down) {
  const std::lock_guard<std::mutex> lock(mutex_);
  list_.ProcessEnded(process, mono, down);
  CollectLocked();
}

void CameraService::SetWebrtc(const std::string& reason, const std::string& detail) {
  const std::lock_guard<std::mutex> lock(mutex_);
  list_.SetWebrtc(reason, detail);
  CollectLocked();
}

void CameraService::CollectLocked() {
  for (const CameraChange& change : list_.TakeChanges()) {
    const CameraState& state = change.news.state;
    Json details = {{"camera_id", change.camera_id}, {"available", state.available}};
    if (!state.available) {
      details["reason"] = state.reason;
      if (!state.detail.empty()) details["message"] = state.detail;
    }
    details["protocols"] = change.news.protocols;
    details["width"] = change.news.width;
    details["height"] = change.news.height;
    details["severity"] = "info";
    details["resume"] = "auto";
    effects_.events.push_back(std::move(details));
    std::vector<Seat> ended;
    if (!state.available) {
      ended = seats_.DropCamera(change.camera_id);
    } else {
      for (const std::string& protocol : change.dropped) {
        for (Seat& seat : seats_.DropCamera(change.camera_id, protocol)) ended.push_back(std::move(seat));
      }
    }
    for (const Seat& seat : ended) {
      if (seat.protocol == kMjpeg) {
        effects_.close.push_back(std::stoull(seat.connection));
      } else {
        effects_.kick.push_back(seat.connection);
      }
    }
  }
}

std::optional<RpcOutcome> CameraService::Call(const std::string& method, const Json& params) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (method == "camera.list") return Ok(list_.List());
  if (method == "camera.stream") return Stream(params);
  return std::nullopt;
}

RpcOutcome CameraService::Stream(const Json& params) {
  if (!params.is_object() || !params.contains("id") || !params["id"].is_string()) {
    return Invalid("camera.stream needs the id of a camera");
  }
  const std::string id = params["id"].get<std::string>();
  const CameraEntry* camera = list_.Find(id);
  if (!camera) return Invalid("No camera with this id");
  std::string protocol;
  if (params.contains("protocol")) {
    if (!params["protocol"].is_string()) return Invalid("protocol is mjpeg or webrtc");
    protocol = params["protocol"].get<std::string>();
    if (std::find(camera->protocols.begin(), camera->protocols.end(), protocol) == camera->protocols.end()) {
      return Invalid("Camera " + id + " does not stream with this protocol");
    }
  }
  const CameraState state = list_.State(id);
  if (!state.available) {
    return Refuse(state.reason, "Camera " + id + " cannot stream now: " + state.reason +
                                    (state.detail.empty() ? "" : " (" + state.detail + ")"));
  }
  if (protocol.empty()) protocol = camera->protocols.front();
  if (protocol == kWebrtc && !list_.webrtc_reason().empty()) {
    return Refuse(list_.webrtc_reason(), "Camera " + id + " cannot stream with webrtc now: " + list_.webrtc_detail());
  }
  const std::string host = config_.public_host.empty() ? route_() : config_.public_host;
  if (host.empty()) {
    return Refuse(kNotSetUp, "The mower has no default route to take its address from: set public_host");
  }
  std::string token;
  switch (seats_.Grant(id, protocol, clocks_.steady(), &token)) {
    case Seats::Granted::kOk: break;
    case Seats::Granted::kBusy: return Refuse("CAMERA_BUSY", "Two viewers are watching already");
    case Seats::Granted::kNoRandom:
      return {false, Json(), mowbite_bridge::kInternalError, "No random numbers for a token", Json()};
  }
  const std::string url =
      protocol == kMjpeg
          ? "http://" + host + ":" + std::to_string(config_.mjpeg_port) + kPathPrefix + id + kPathSuffix + "?t=" + token
          : "http://" + host + ":" + std::to_string(config_.webrtc_port) + "/" + id + "/whep?t=" + token;
  // Rounded down, so the client never counts on more time than the token has.
  const auto expires = static_cast<int64_t>(std::floor(clocks_.wall() + kTokenSeconds));
  return Ok({{"url", url}, {"protocol", protocol}, {"expires", expires}});
}

void CameraService::Tick() {
  const std::lock_guard<std::mutex> lock(mutex_);
  seats_.Expire(clocks_.steady());
}

Effects CameraService::TakeEffects() {
  const std::lock_guard<std::mutex> lock(mutex_);
  Effects effects;
  std::swap(effects, effects_);
  return effects;
}

bool CameraService::HasCameras() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return !list_.Empty();
}

std::vector<std::string> CameraService::Processes() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return list_.Processes();
}

std::map<std::string, std::string> CameraService::WebrtcSources() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return list_.WebrtcSources();
}

std::vector<std::string> CameraService::MjpegCameras() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> cameras;
  for (const Seat& seat : seats_.seats()) {
    if (seat.protocol != kMjpeg || seat.connection.empty()) continue;
    if (std::find(cameras.begin(), cameras.end(), seat.camera_id) == cameras.end()) cameras.push_back(seat.camera_id);
  }
  return cameras;
}

std::vector<std::string> CameraService::WebrtcSessions() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> sessions;
  for (const Seat& seat : seats_.seats()) {
    if (seat.protocol == kWebrtc && !seat.connection.empty()) sessions.push_back(seat.connection);
  }
  return sessions;
}

void CameraService::OnSessions(const std::vector<std::string>& alive) {
  const std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, int> missing;
  std::vector<std::string> over;
  for (const Seat& seat : seats_.seats()) {
    if (seat.protocol != kWebrtc || seat.connection.empty()) continue;
    if (std::find(alive.begin(), alive.end(), seat.connection) != alive.end()) continue;
    const int count = missing_[seat.connection] + 1;
    if (count >= kMissingLimit) {
      over.push_back(seat.connection);
    } else {
      missing[seat.connection] = count;
    }
  }
  missing_ = std::move(missing);
  for (const std::string& session : over) seats_.Release(kWebrtc, session);
}

void CameraService::OnStreamServerGone() {
  const std::lock_guard<std::mutex> lock(mutex_);
  seats_.DropProtocol(kWebrtc);
  missing_.clear();
}

HttpAnswer CameraService::OnViewer(const HttpRequest& request) {
  if (request.method != "GET") return {405, "only GET\n", ""};
  const size_t prefix = sizeof(kPathPrefix) - 1;
  const size_t suffix = sizeof(kPathSuffix) - 1;
  const std::string& path = request.path;
  if (path.size() <= prefix + suffix || path.compare(0, prefix, kPathPrefix) != 0 ||
      path.compare(path.size() - suffix, suffix, kPathSuffix) != 0) {
    return {404, "not found\n", ""};
  }
  const std::string id = path.substr(prefix, path.size() - prefix - suffix);
  const std::lock_guard<std::mutex> lock(mutex_);
  // A place is lost when its camera can no longer stream, so a token that connects has a camera that can.
  if (!seats_.Connect(QueryValue(request.query, "t"), id, kMjpeg, std::to_string(request.connection),
                      clocks_.steady())) {
    return {401, "no place for this token\n", ""};
  }
  return {200, "", id};
}

void CameraService::OnViewerGone(uint64_t connection) {
  const std::lock_guard<std::mutex> lock(mutex_);
  seats_.Release(kMjpeg, std::to_string(connection));
}

HttpAnswer CameraService::OnAuth(const HttpRequest& request) {
  if (request.method != "POST" || request.path != "/auth") return {404, "not found\n", ""};
  const Json asked = Json::parse(request.body, nullptr, false);
  if (!asked.is_object()) return {400, "bad request\n", ""};
  const std::string action = Text(asked, "action");
  bool allowed = false;
  if (action == "api") {
    // Both are compared whatever the first one gives.
    const bool user_fits = SameText(Text(asked, "user"), api_user_);
    const bool password_fits = SameText(Text(asked, "password"), api_password_);
    allowed = user_fits && password_fits && !api_password_.empty();
  } else if (action == "read" && Text(asked, "protocol") == kWebrtc) {
    const std::string camera = Text(asked, "path");
    const std::string given = QueryValue(Text(asked, "query"), "t");
    const std::string session = Text(asked, "id");
    const std::lock_guard<std::mutex> lock(mutex_);
    // Without a session the client only asks how to connect; the session that follows takes the place.
    allowed = session.empty() ? seats_.Waiting(given, camera, kWebrtc, clocks_.steady())
                              : seats_.Connect(given, camera, kWebrtc, session, clocks_.steady());
  }
  return {allowed ? 200 : 401, "", ""};
}

}  // namespace mowbite_camera
