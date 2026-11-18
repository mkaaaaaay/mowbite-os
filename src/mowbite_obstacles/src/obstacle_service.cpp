#include "mowbite_obstacles/obstacle_service.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <utility>

#include "mowbite_api/catalog.h"
#include "mowbite_bridge/json_rpc.h"
#include "mowbite_hold/file_io.h"
#include "mowbite_map/sha256.h"
#include "mowbite_ntrip/ntrip_protocol.h"

namespace mowbite_obstacles {

namespace {

constexpr char kListFile[] = "list.json";
constexpr size_t kMaxListBytes = 4u << 20;
// The largest image taken from a sensor; the contract allows 100 KB.
constexpr size_t kMaxImageBytes = 200u * 1024u;
constexpr double kSameStamp = 1e-6;

const char* ModeName(Mode mode) {
  return mode == Mode::kShow ? "show" : "off";
}

Outcome Refused(std::string message) {
  return {false, Json(), mowbite_bridge::kInvalidParams, std::move(message)};
}

// A whole number of params, or the fallback without one; false for another type.
bool Whole(const Json& params, const char* name, int64_t fallback, int64_t* value) {
  const auto it = params.find(name);
  if (it == params.end() || it->is_null()) {
    *value = fallback;
    return true;
  }
  if (!it->is_number_integer() && !it->is_number_unsigned()) return false;
  *value = it->get<int64_t>();
  return true;
}

}  // namespace

ObstacleService::ObstacleService(ServiceConfig config, std::function<void(const std::string&)> warn)
    : config_(std::move(config)), warn_(std::move(warn)), list_(config_.limits) {}

const std::vector<std::string>& ObstacleService::Methods() {
  static const std::vector<std::string> methods = {"vision.obstacles.list", "vision.obstacles.image",
                                                   "vision.obstacles.clear"};
  return methods;
}

void ObstacleService::Load(double now) {
  if (config_.dir.empty()) return;
  namespace fs = std::filesystem;
  std::string text, error;
  const auto read = mowbite_hold::ReadSmallFile((fs::path(config_.dir) / kListFile).string(), kMaxListBytes, &text,
                                                &error);
  if (read == mowbite_hold::FileRead::kText && !list_.Load(text)) warn_("The list of obstacles is not valid, a new one");
  if (read == mowbite_hold::FileRead::kFailed || read == mowbite_hold::FileRead::kTooLarge) {
    warn_("The list of obstacles cannot be read: " + (error.empty() ? std::string("too large") : error));
  }
  // Photos the list does not name go, and entries lose photos that are gone.
  std::error_code code;
  for (const auto& file : fs::directory_iterator(config_.dir, code)) {
    const std::string name = file.path().filename().string();
    if (name == kListFile || list_.ByPhoto(name) != 0) continue;
    if (!mowbite_hold::RemoveFileSynced(file.path().string(), &error)) warn_("Cannot remove " + name + ": " + error);
  }
  for (const Json& entry : list_.ToJson(0.0)) {
    const Json& image = entry["image"];
    if (image.is_object() && !fs::exists(fs::path(config_.dir) / image["name"].get<std::string>(), code)) {
      list_.DropPhoto(entry["id"].get<uint32_t>());
      dirty_ = true;
    }
  }
  RemovePhotos(list_.Prune(now));
  pruned_at_ = now;
  Flush();
}

void ObstacleService::SetMode(int mode) {
  mode_ = mode >= 1 ? Mode::kShow : Mode::kOff;
  if (!PhotosWanted()) images_.clear();
}

void ObstacleService::SetPhotos(bool on) {
  photos_ = on;
  if (!PhotosWanted()) images_.clear();
}

void ObstacleService::SetSight(bool ok, const std::string& reason, const std::string& source, double now) {
  const Sight sight{ok, ok ? "" : reason, ok ? "" : source};
  if (sight == sight_) return;
  sight_ = sight;
  sight_since_ = now;
}

void ObstacleService::OnFrame(const Tracker& tracker, const FrameResult& frame, double now) {
  if (mode_ == Mode::kOff) return;
  for (const uint32_t id : frame.confirmed) {
    const Track* track = tracker.TrackOf(id);
    if (!track) continue;
    const ObstacleList::Seen seen = list_.OnConfirmed(*track);
    dirty_ = true;
    if (!seen.added) continue;
    if (PhotosWanted() && !config_.dir.empty()) {
      pending_.push_back({seen.id, now + config_.photo_wait});
    } else {
      Seen(seen.id);
    }
  }
  for (const auto& [id, track] : tracker.tracks()) {
    if (track.confirmed && list_.Follow(track)) dirty_ = true;
  }
  TryPhotos(tracker);
  const size_t before = list_.size();
  RemovePhotos(list_.Prune(0.0));
  if (list_.size() != before) dirty_ = true;
}

void ObstacleService::OnEnded(const std::vector<uint32_t>& tracks) {
  for (const uint32_t id : tracks) list_.ForgetTrack(id);
}

void ObstacleService::OnImage(const Tracker& tracker, Image image, double) {
  if (!PhotosWanted() || config_.dir.empty() || image.jpeg.empty() || image.jpeg.size() > kMaxImageBytes) return;
  images_.push_back(std::move(image));
  while (images_.size() > config_.images) images_.pop_front();
  TryPhotos(tracker);
}

void ObstacleService::TryPhotos(const Tracker& tracker) {
  if (images_.empty()) return;
  for (const auto& [id, track] : tracker.tracks()) {
    const uint32_t entry_id = list_.EntryOf(id);
    const Entry* entry = entry_id ? list_.Get(entry_id) : nullptr;
    if (!entry || entry->photo) continue;
    bool done = false;
    for (auto shot = track.shots.rbegin(); shot != track.shots.rend() && !done; ++shot) {
      for (const Image& image : images_) {
        if (image.source != shot->source || std::abs(image.stamp - shot->stamp) > kSameStamp) continue;
        SavePhoto(entry_id, image, *shot);
        done = true;
        break;
      }
    }
  }
}

void ObstacleService::SavePhoto(uint32_t entry, const Image& image, const Shot& shot) {
  Photo photo;
  photo.name = "o" + std::to_string(entry) + ".jpg";
  photo.sha256 = mowbite_map::Sha256Hex(image.jpeg);
  photo.bytes = static_cast<int64_t>(image.jpeg.size());
  photo.width = image.width;
  photo.height = image.height;
  photo.box_x = shot.box_x;
  photo.box_y = shot.box_y;
  photo.box_w = shot.box_w;
  photo.box_h = shot.box_h;
  std::string error;
  const std::string path = (std::filesystem::path(config_.dir) / photo.name).string();
  if (photo.sha256.empty() || !mowbite_hold::WriteFileSafely(path, image.jpeg, &error)) {
    warn_("Cannot keep the photo of obstacle " + std::to_string(entry) + ": " + error);
    return;
  }
  list_.SetPhoto(entry, photo);
  dirty_ = true;
  RemovePhotos(list_.Prune(0.0));
  const auto waiting = std::find_if(pending_.begin(), pending_.end(), [entry](const Pending& p) {
    return p.entry == entry;
  });
  if (waiting != pending_.end()) {
    pending_.erase(waiting);
    Seen(entry);
  }
}

void ObstacleService::Seen(uint32_t id) {
  const Entry* entry = list_.Get(id);
  if (!entry) return;
  events_.push_back({"OBSTACLE_SEEN",
                     {{"obstacle_id", entry->id},
                      {"kind", KindName(entry->kind)},
                      {"class", entry->class_name},
                      {"confidence", std::round(entry->confidence * 1000.0) / 1000.0},
                      {"x", entry->x},
                      {"y", entry->y},
                      {"r", entry->r},
                      {"camera_id", entry->camera_id},
                      {"action", entry->action},
                      {"image", entry->photo.has_value()}}});
}

void ObstacleService::RemovePhotos(const std::vector<std::string>& names) {
  if (!names.empty()) dirty_ = true;
  if (config_.dir.empty()) return;
  for (const std::string& name : names) {
    std::string error;
    if (!mowbite_hold::RemoveFileSynced((std::filesystem::path(config_.dir) / name).string(), &error)) {
      warn_("Cannot remove the photo " + name + ": " + error);
    }
  }
}

void ObstacleService::Tick(double now) {
  for (auto it = pending_.begin(); it != pending_.end();) {
    if (now < it->until) {
      ++it;
      continue;
    }
    const uint32_t entry = it->entry;
    it = pending_.erase(it);
    Seen(entry);
  }
  if (!(sight_ == told_) && now - sight_since_ >= config_.sight_settle) {
    told_ = sight_;
    if (mode_ != Mode::kOff) {
      Json details = {{"ok", sight_.ok}};
      if (!sight_.ok) {
        details["reason"] = sight_.reason;
        details["source"] = sight_.source;
      }
      events_.push_back({"OBSTACLE_SIGHT", details});
    }
  }
  if (now - pruned_at_ >= config_.prune_gap) {
    pruned_at_ = now;
    RemovePhotos(list_.Prune(now));
  }
  if (dirty_ && now - saved_at_ >= config_.save_gap) {
    saved_at_ = now;
    Flush();
  }
}

void ObstacleService::Flush() {
  if (!dirty_) return;
  dirty_ = false;
  if (config_.dir.empty()) return;
  std::string error;
  if (!mowbite_hold::WriteFileSafely((std::filesystem::path(config_.dir) / kListFile).string(), list_.Text(),
                                     &error)) {
    warn_("Cannot keep the list of obstacles: " + error);
  }
}

std::optional<Outcome> ObstacleService::Call(const std::string& method, const Json& params, double now) {
  if (!params.is_object()) return Refused("The parameters are no object");
  if (method == "vision.obstacles.list") return List(params);
  if (method == "vision.obstacles.image") return Piece(params);
  if (method == "vision.obstacles.clear") return Clear(params, now);
  return std::nullopt;
}

Outcome ObstacleService::List(const Json& params) const {
  double since = 0.0;
  const auto it = params.find("since");
  if (it != params.end() && !it->is_null()) {
    if (!it->is_number()) return Refused("since is a number");
    since = it->get<double>();
  }
  const ListLimits& limits = list_.limits();
  Json sight = {{"ok", sight_.ok}, {"reason", sight_.ok ? Json() : Json(sight_.reason)}};
  Json limit = {{"entries", limits.entries}, {"bytes", limits.bytes}, {"days", limits.days}};
  Json result = {{"mode", ModeName(mode_)},
                 {"photos", photos_},
                 {"sight", sight},
                 {"limit", limit},
                 {"obstacles", list_.ToJson(since)}};
  return {true, result, 0, ""};
}

Outcome ObstacleService::Piece(const Json& params) const {
  const auto name = params.find("name");
  if (name == params.end() || !name->is_string()) return Refused("name is the name of an image of the list");
  const uint32_t entry_id = list_.ByPhoto(name->get<std::string>());
  if (entry_id == 0 || config_.dir.empty()) return Refused("No image " + name->get<std::string>());
  int64_t offset = 0, size = mowbite_api::kObstacleImageBytes;
  if (!Whole(params, "offset", 0, &offset) || offset < 0) return Refused("offset is a whole number from 0");
  if (!Whole(params, "size", mowbite_api::kObstacleImageBytes, &size) || size < 1 ||
      size > mowbite_api::kObstacleImageBytes) {
    return Refused("size is 1 to " + std::to_string(mowbite_api::kObstacleImageBytes));
  }
  const Photo& photo = *list_.Get(entry_id)->photo;
  std::string bytes, error;
  const auto read = mowbite_hold::ReadSmallFile((std::filesystem::path(config_.dir) / photo.name).string(),
                                                kMaxImageBytes, &bytes, &error);
  if (read != mowbite_hold::FileRead::kText) {
    return {false, Json(), mowbite_bridge::kInternalError, "The image cannot be read"};
  }
  if (offset > static_cast<int64_t>(bytes.size())) return Refused("offset is past the end of the image");
  const std::string piece = bytes.substr(static_cast<size_t>(offset), static_cast<size_t>(size));
  return {true,
          {{"size", bytes.size()},
           {"offset", offset},
           {"data", mowbite_ntrip::Base64(piece)},
           {"sha256", photo.sha256},
           {"type", "image/jpeg"}},
          0,
          ""};
}

Outcome ObstacleService::Clear(const Json& params, double) {
  uint32_t id = 0;
  const auto it = params.find("id");
  if (it != params.end() && !it->is_null()) {
    if (!it->is_number_unsigned() && !(it->is_number_integer() && it->get<int64_t>() > 0)) {
      return Refused("id is the id of an obstacle of the list");
    }
    const int64_t value = it->get<int64_t>();
    if (value <= 0 || value > UINT32_MAX || !list_.Get(static_cast<uint32_t>(value))) {
      return Refused("No obstacle " + std::to_string(value));
    }
    id = static_cast<uint32_t>(value);
  }
  const size_t before = list_.size();
  RemovePhotos(list_.Remove(id));
  pending_.erase(std::remove_if(pending_.begin(), pending_.end(),
                                [this](const Pending& p) { return list_.Get(p.entry) == nullptr; }),
                 pending_.end());
  dirty_ = true;
  Flush();
  events_.push_back({"OBSTACLES_CLEARED", {{"obstacle_id", id == 0 ? Json() : Json(id)}}});
  return {true, {{"removed", before - list_.size()}}, 0, ""};
}

std::vector<std::pair<std::string, Json>> ObstacleService::TakeEvents() {
  return std::exchange(events_, {});
}

}  // namespace mowbite_obstacles
