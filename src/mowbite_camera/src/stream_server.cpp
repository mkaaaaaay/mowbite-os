#include "mowbite_camera/stream_server.h"

#include <unistd.h>

#include <algorithm>
#include <cctype>

#include "mowbite_camera/camera_list.h"
#include "mowbite_camera/stream_config.h"

namespace mowbite_camera {

namespace {

constexpr char kSessions[] = "/v3/webrtcsessions/";
constexpr size_t kMaxSessionId = 64;

// An id as the stream server gives them: hex digits and '-'. It becomes part of a request line.
bool IsSessionId(const std::string& id) {
  return !id.empty() && id.size() <= kMaxSessionId &&
         std::all_of(id.begin(), id.end(), [](unsigned char c) { return std::isxdigit(c) || c == '-'; });
}

}  // namespace

MediaServer::MediaServer(std::string program, std::string directory)
    : program_(std::move(program)), directory_(std::move(directory)) {}

bool MediaServer::Installed() {
  return !program_.empty() && ::access(program_.c_str(), X_OK) == 0;
}

bool MediaServer::Start(std::string* error) {
  // One that was asked to end and has not yet: now it goes, within its grace.
  if (child_.Running()) child_.Stop();
  ChildSpec spec;
  spec.program = program_;
  spec.args = {directory_ + "/" + kStreamConfigFile};
  spec.environment = {kStreamMemoryLimit};
  spec.directory = directory_;
  spec.nice = kStreamNice;
  spec.oom_score_adj = kStreamOomScoreAdj;
  return child_.Start(spec, error);
}

bool MediaServer::Running(std::string* how) {
  return child_.Running(how);
}

void MediaServer::Stop() {
  child_.Interrupt();
}

void MediaServer::Reap() {
  // Only a stopped one: an end of its own is for Running to find and tell.
  if (child_.Interrupted()) child_.Running();
}

bool ParseSessions(const std::string& body, std::vector<std::string>* ids) {
  const Json answer = Json::parse(body, nullptr, false);
  if (!answer.is_object() || !answer.contains("items") || !answer["items"].is_array()) return false;
  // More pages than this one: a session missing here may be on another.
  const bool one_page = answer["items"].empty() || answer.value("pageCount", Json(1)) == Json(1);
  if (!one_page) return false;
  ids->clear();
  for (const Json& item : answer["items"]) {
    if (item.is_object() && item.contains("id") && item["id"].is_string()) {
      ids->push_back(item["id"].get<std::string>());
    }
  }
  return true;
}

bool StreamSessions(const LocalServer& api, std::vector<std::string>* ids) {
  const HttpReply reply = LocalRequest(api, "GET", std::string(kSessions) + "list", kStreamApiSeconds);
  return reply.status == 200 && ParseSessions(reply.body, ids);
}

bool KickStreamSession(const LocalServer& api, const std::string& id) {
  if (!IsSessionId(id)) return false;
  return LocalRequest(api, "POST", std::string(kSessions) + "kick/" + id, kStreamApiSeconds).status == 200;
}

}  // namespace mowbite_camera
