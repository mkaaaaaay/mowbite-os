#include "mowbite_camera/camera_list.h"

#include <algorithm>
#include <cctype>

#include "mowbite_api/catalog.h"

namespace mowbite_camera {

namespace {

constexpr size_t kMaxIdLength = 32;
constexpr char kSourcePrefix[] = "rtsp://127.0.0.1:";

// A name that may stand in a file name: letters, digits, '_' and '-'.
bool IsProcessName(const std::string& name) {
  if (name.empty() || name.size() > kMaxIdLength) return false;
  return std::all_of(name.begin(), name.end(), [](unsigned char c) { return std::isalnum(c) || c == '_' || c == '-'; });
}

// rtsp://127.0.0.1:<port>/<id>, the only address the stream server is sent to.
bool IsLocalSource(const std::string& source, const std::string& id) {
  const size_t prefix = sizeof(kSourcePrefix) - 1;
  if (source.compare(0, prefix, kSourcePrefix) != 0) return false;
  const size_t slash = source.find('/', prefix);
  if (slash == std::string::npos || slash == prefix || slash - prefix > 5) return false;
  int port = 0;
  for (size_t i = prefix; i < slash; ++i) {
    if (!std::isdigit(static_cast<unsigned char>(source[i]))) return false;
    port = port * 10 + (source[i] - '0');
  }
  return port >= 1 && port <= 65535 && source.compare(slash + 1, std::string::npos, id) == 0;
}

bool IsPosition(const std::string& position) {
  return position == "front" || position == "rear" || position == "left" || position == "right";
}

// A reason CAMERA_CHANGED may carry.
bool IsReason(const std::string& code) {
  const mowbite_api::EventEntry* event = mowbite_api::FindEvent("CAMERA_CHANGED");
  if (!event) return false;
  return std::any_of(event->reasons.begin(), event->reasons.end(),
                     [&](const mowbite_api::ReasonEntry& reason) { return reason.code == code; });
}

constexpr size_t kPlainLength = sizeof(kPlainSuffix) - 1;

bool EndsWithPlain(const std::string& name) {
  return name.size() > kPlainLength && name.compare(name.size() - kPlainLength, kPlainLength, kPlainSuffix) == 0;
}

void Note(std::string* problem, const std::string& text) {
  if (!problem->empty()) *problem += "; ";
  *problem += text;
}

}  // namespace

bool IsCameraId(const std::string& id) {
  if (id.empty() || id.size() > kMaxIdLength || !std::isalpha(static_cast<unsigned char>(id[0]))) return false;
  if (EndsWithPlain(id)) return false;
  return std::all_of(id.begin(), id.end(), [](unsigned char c) { return std::isalnum(c) || c == '_'; });
}

std::string ViewOf(const std::string& id, bool plain) {
  return plain ? id + kPlainSuffix : id;
}

std::string PictureTopic(const std::string& view) {
  if (!EndsWithPlain(view)) return "camera/" + view + "/jpeg";
  return "camera/" + view.substr(0, view.size() - kPlainLength) + "/jpeg" + kPlainSuffix;
}

bool ParseExitNote(const std::string& text, double* mono, bool* down) {
  const Json note = Json::parse(text, nullptr, false);
  if (!note.is_object() || !note.contains("mono") || !note["mono"].is_number()) return false;
  *mono = note["mono"].get<double>();
  *down = note.value("down", Json(false)) == Json(true);
  return true;
}

bool CameraList::Update(const CameraEntry& entry, double t, std::string* problem) {
  problem->clear();
  if (!IsCameraId(entry.id)) {
    *problem = std::string("a camera id is a letter, then at most 31 letters, digits or underscores, not ending in ") +
               kPlainSuffix;
    return false;
  }
  CameraEntry clean = entry;
  if (!IsProcessName(clean.process)) {
    if (!clean.process.empty()) Note(problem, "process is not a name, so its exit note is not read");
    clean.process.clear();
  }
  if (!IsPosition(clean.position)) clean.position.clear();
  clean.protocols.clear();
  for (const std::string& protocol : entry.protocols) {
    if (std::find(clean.protocols.begin(), clean.protocols.end(), protocol) != clean.protocols.end()) continue;
    if (protocol == kMjpeg || (protocol == kWebrtc && IsLocalSource(entry.source, entry.id))) {
      clean.protocols.push_back(protocol);
    } else if (protocol == kWebrtc) {
      Note(problem, std::string("webrtc needs the source ") + kSourcePrefix + "<port>/" + entry.id);
    }
  }
  if (clean.available) {
    clean.reason.clear();
    clean.detail.clear();
  } else if (!IsReason(clean.reason)) {
    clean.detail = "reason " + clean.reason + (clean.detail.empty() ? "" : ": " + clean.detail);
    clean.reason = kNoSignal;
  }

  auto camera = std::find_if(cameras_.begin(), cameras_.end(),
                             [&](const Camera& have) { return have.entry.id == entry.id; });
  if (camera == cameras_.end()) camera = cameras_.insert(cameras_.end(), Camera());
  camera->entry = std::move(clean);
  camera->reported = t;
  return true;
}

void CameraList::ProcessEnded(const std::string& process, double t, bool down) {
  ended_[process] = {t, down};
}

void CameraList::SetWebrtc(const std::string& reason, const std::string& detail) {
  webrtc_reason_ = reason;
  webrtc_detail_ = reason.empty() ? "" : detail;
}

CameraState CameraList::StateOf(const Camera& camera) const {
  const auto ended = ended_.find(camera.entry.process);
  if (ended != ended_.end() && ended->second.t > camera.reported) {
    return {false, kNoSignal,
            "process " + camera.entry.process + (ended->second.down ? " ended and stays down" : " ended")};
  }
  if (!camera.entry.available) return {false, camera.entry.reason, camera.entry.detail};
  if (camera.entry.protocols.empty()) return {false, kNotSetUp, "no protocol the OS serves"};
  const bool only_webrtc = camera.entry.protocols.size() == 1 && camera.entry.protocols[0] == kWebrtc;
  if (only_webrtc && !webrtc_reason_.empty()) return {false, webrtc_reason_, webrtc_detail_};
  return {true, "", ""};
}

std::vector<CameraChange> CameraList::TakeChanges() {
  std::vector<CameraChange> changes;
  for (Camera& camera : cameras_) {
    CameraNews news = {StateOf(camera), camera.entry.protocols, camera.entry.width, camera.entry.height};
    if (camera.told == news) continue;
    camera.told = news;
    changes.push_back({camera.entry.id, std::move(news)});
  }
  return changes;
}

const CameraEntry* CameraList::Find(const std::string& id) const {
  for (const Camera& camera : cameras_) {
    if (camera.entry.id == id) return &camera.entry;
  }
  return nullptr;
}

CameraState CameraList::State(const std::string& id) const {
  for (const Camera& camera : cameras_) {
    if (camera.entry.id == id) return StateOf(camera);
  }
  return {false, kNoSignal, "not listed"};
}

Json CameraList::List() const {
  Json list = Json::array();
  for (const Camera& camera : cameras_) {
    const CameraEntry& entry = camera.entry;
    const CameraState state = StateOf(camera);
    Json item = {{"id", entry.id},
                 {"name", entry.name},
                 {"kind", entry.kind},
                 {"position", entry.position.empty() ? Json() : Json(entry.position)},
                 {"width", entry.width},
                 {"height", entry.height},
                 {"detections", entry.detections},
                 {"protocols", entry.protocols},
                 {"available", state.available}};
    if (!state.available) item["reason"] = state.reason;
    if (!state.available && !state.detail.empty()) item["detail"] = state.detail;
    list.push_back(std::move(item));
  }
  return list;
}

std::vector<std::string> CameraList::Processes() const {
  std::vector<std::string> processes;
  for (const Camera& camera : cameras_) {
    const std::string& process = camera.entry.process;
    if (!process.empty() && std::find(processes.begin(), processes.end(), process) == processes.end()) {
      processes.push_back(process);
    }
  }
  return processes;
}

std::map<std::string, std::string> CameraList::WebrtcSources() const {
  std::map<std::string, std::string> sources;
  for (const Camera& camera : cameras_) {
    const auto& protocols = camera.entry.protocols;
    if (std::find(protocols.begin(), protocols.end(), kWebrtc) == protocols.end()) continue;
    sources[camera.entry.id] = camera.entry.source;
    if (camera.entry.detections) sources[ViewOf(camera.entry.id, true)] = camera.entry.source + kPlainSuffix;
  }
  return sources;
}

}  // namespace mowbite_camera
