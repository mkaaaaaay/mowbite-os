#include "mowbite_camera/camera_service.h"

#include <algorithm>
#include <cmath>

#include "mowbite_bridge/json_rpc.h"

namespace mowbite_camera {

namespace {

// The groups of x-reasons whose reasons the refusals of camera.stream and camera.unlock carry.
constexpr char kRefused[] = "CAMERA_REFUSED";
constexpr char kUnlockRefused[] = "CAMERA_UNLOCK_REFUSED";
constexpr char kLocked[] = "LOCKED_AFTER_CRASH";
// What the unlock service answers for a camera without a lock.
constexpr char kNotLockedAnswer[] = "not locked";
// The reason of the lock goes to the app: its first line, at most this long.
constexpr size_t kMaxReasonBytes = 200;
constexpr char kPathPrefix[] = "/camera/";
constexpr char kPathSuffix[] = ".mjpg";
// The protocols kept of a camera that cannot stream.
const std::vector<std::string> kNone;
// A session the stream server lacks this often in a row is over.
constexpr int kMissingLimit = 2;

using mowbite_bridge::Ok;
using mowbite_bridge::Refusal;

RpcOutcome Invalid(std::string message) {
  return mowbite_bridge::InvalidParams(std::move(message));
}

RpcOutcome RefuseUnlock(const std::string& reason, const std::string& message, const Json& details = Json::object()) {
  return Refusal(kUnlockRefused, reason, message, details);
}

// The first line of text without the blanks around it, cut to max bytes at a whole UTF-8 character.
std::string FirstLine(const std::string& text, size_t max) {
  const std::string line = Trim(text.substr(0, text.find_first_of("\r\n")));
  if (line.size() <= max) return line;
  size_t end = max;
  while (end > 0 && (static_cast<unsigned char>(line[end]) & 0xC0) == 0x80) --end;
  return line.substr(0, end);
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
  static const std::vector<std::string> methods = {"camera.list", "camera.stream", "camera.unlock"};
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
    // Places that no longer fit what the camera offers end.
    for (const Seat& seat : seats_.DropCamera(change.camera_id, state.available ? change.news.protocols : kNone)) {
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
    return Refusal(kRefused, state.reason, "Camera " + id + " cannot stream now: " + state.reason +
                                    (state.detail.empty() ? "" : " (" + state.detail + ")"));
  }
  bool detections = camera->detections;
  if (params.contains("detections")) {
    if (!params["detections"].is_boolean()) return Invalid("detections is true or false");
    if (params["detections"].get<bool>() && !camera->detections) {
      return Invalid("Camera " + id + " has no boxes of the camera AI");
    }
    detections = params["detections"].get<bool>();
  }
  const std::string view = ViewOf(id, camera->detections && !detections);
  if (protocol.empty()) protocol = camera->protocols.front();
  if (protocol == kWebrtc && !list_.webrtc_reason().empty()) {
    return Refusal(kRefused, list_.webrtc_reason(),
                   "Camera " + id + " cannot stream with webrtc now: " + list_.webrtc_detail());
  }
  const std::string host = config_.public_host.empty() ? route_() : config_.public_host;
  if (host.empty()) {
    return Refusal(kRefused, kNotSetUp, "The mower has no default route to take its address from: set public_host");
  }
  std::string token;
  switch (seats_.Grant(id, view, protocol, clocks_.steady(), &token)) {
    case Seats::Granted::kOk: break;
    case Seats::Granted::kBusy: return Refusal(kRefused, "CAMERA_BUSY", "Two viewers are watching already");
    case Seats::Granted::kNoRandom:
      return {false, Json(), mowbite_bridge::kInternalError, "No random numbers for a token", Json()};
  }
  const std::string url =
      protocol == kMjpeg
          ? "http://" + host + ":" + std::to_string(config_.mjpeg_port) + kPathPrefix + view + kPathSuffix +
                "?t=" + token
          : "http://" + host + ":" + std::to_string(config_.webrtc_port) + "/" + view + "/whep?t=" + token;
  // Rounded down, so the client never counts on more time than the token has.
  const auto expires = static_cast<int64_t>(std::floor(clocks_.wall() + kTokenSeconds));
  return Ok({{"url", url}, {"protocol", protocol}, {"expires", expires}, {"detections", detections}});
}

std::optional<RpcOutcome> CameraService::BeginUnlock(const std::string& request_id, const Json& params,
                                                     std::string* id) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!params.is_object() || !params.contains("id") || !params["id"].is_string()) {
    return Invalid("camera.unlock needs the id of a camera");
  }
  *id = params["id"].get<std::string>();
  if (!list_.Find(*id)) return Invalid("No camera with this id");
  const CameraState state = list_.State(*id);
  if (state.reason != kLocked) {
    return RefuseUnlock("NOT_LOCKED", "Camera " + *id + " is not locked after a crash");
  }
  const std::string unrest = Unrest();
  if (!unrest.empty()) {
    return RefuseUnlock("NOT_AT_REST", "Camera " + *id + " is unlocked only at rest: " + unrest);
  }
  unlocks_[next_unlock_++] = {request_id, *id, clocks_.steady() + kUnlockSeconds, false};
  return std::nullopt;
}

std::vector<UnlockCall> CameraService::SendUnlocks(const std::function<bool(const std::string&)>& ready) {
  const std::lock_guard<std::mutex> lock(mutex_);
  std::vector<UnlockCall> calls;
  for (auto& [key, unlock] : unlocks_) {
    if (unlock.sent || !ready(unlock.camera)) continue;
    unlock.sent = true;
    calls.push_back({key, unlock.camera});
  }
  return calls;
}

std::vector<UnlockAnswer> CameraService::ExpiredUnlocks() {
  const std::lock_guard<std::mutex> lock(mutex_);
  const double now = clocks_.steady();
  std::vector<UnlockAnswer> answers;
  for (auto it = unlocks_.begin(); it != unlocks_.end();) {
    if (now < it->second.deadline) {
      ++it;
      continue;
    }
    const UnlockReply::Kind kind = it->second.sent ? UnlockReply::Kind::kTimeout : UnlockReply::Kind::kNoService;
    answers.push_back({it->second.request_id, it->second.camera, EndUnlock(it->second.camera, {kind, false, ""})});
    it = unlocks_.erase(it);
  }
  return answers;
}

std::optional<UnlockAnswer> CameraService::AnswerUnlock(uint64_t key, const UnlockReply& reply) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const auto found = unlocks_.find(key);
  if (found == unlocks_.end()) return std::nullopt;
  UnlockAnswer answer{found->second.request_id, found->second.camera, EndUnlock(found->second.camera, reply)};
  unlocks_.erase(found);
  return answer;
}

RpcOutcome CameraService::EndUnlock(const std::string& id, const UnlockReply& reply) const {
  switch (reply.kind) {
    case UnlockReply::Kind::kNoService:
      return RefuseUnlock("UNSUPPORTED", "Camera " + id + " offers no unlock service");
    case UnlockReply::Kind::kTimeout:
      return RefuseUnlock("TIMEOUT", "The unlock service of camera " + id +
                                         " gave no answer; camera.list tells whether it worked");
    case UnlockReply::Kind::kAnswered: break;
  }
  const std::string message = FirstLine(reply.message, kMaxReasonBytes);
  if (reply.success) return Ok({{"unlocked", true}, {"reason", message}});
  if (message == kNotLockedAnswer) {
    return RefuseUnlock("NOT_LOCKED", "Camera " + id + " has no lock on the mower");
  }
  return RefuseUnlock("FAILED", "Camera " + id + " could not be unlocked: " + message, {{"detail", message}});
}

void CameraService::OnRobotState(const std::string& state, const std::string& sub_state) {
  const std::lock_guard<std::mutex> lock(mutex_);
  state_ = state;
  sub_state_ = sub_state;
  state_at_ = clocks_.steady();
}

void CameraService::OnBlade(bool running) {
  const std::lock_guard<std::mutex> lock(mutex_);
  blade_running_ = running;
  blade_at_ = clocks_.steady();
}

// At rest: IDLE without a sub state (no driving by hand, no calibration), docked or not, and the blade still; a
// kernel crash while mowing would leave the mower without control.
std::string CameraService::Unrest() const {
  const double now = clocks_.steady();
  if (state_at_ < 0 || now - state_at_ > kStateMaxAge) return "the state of the mower is not known";
  if (state_ != "IDLE" || !sub_state_.empty()) {
    return "the mower is in " + state_ + (sub_state_.empty() ? "" : " " + sub_state_);
  }
  if (blade_at_ < 0 || now - blade_at_ > kHardwareMaxAge) return "the blade is not known to stand";
  if (blade_running_) return "the blade runs";
  return "";
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
    if (std::find(cameras.begin(), cameras.end(), seat.view) == cameras.end()) cameras.push_back(seat.view);
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
  const std::string view = path.substr(prefix, path.size() - prefix - suffix);
  const std::lock_guard<std::mutex> lock(mutex_);
  // A place is lost when its camera can no longer stream, so a token that connects has a camera that can.
  if (!seats_.Connect(QueryValue(request.query, "t"), view, kMjpeg, std::to_string(request.connection),
                      clocks_.steady())) {
    return {401, "no place for this token\n", ""};
  }
  return {200, "", view};
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
    const std::string view = Text(asked, "path");
    const std::string given = QueryValue(Text(asked, "query"), "t");
    const std::string session = Text(asked, "id");
    const std::lock_guard<std::mutex> lock(mutex_);
    // Without a session the client only asks how to connect; the session that follows takes the place.
    allowed = session.empty() ? seats_.Waiting(given, view, kWebrtc, clocks_.steady())
                              : seats_.Connect(given, view, kWebrtc, session, clocks_.steady());
  }
  return {allowed ? 200 : 401, "", ""};
}

}  // namespace mowbite_camera
