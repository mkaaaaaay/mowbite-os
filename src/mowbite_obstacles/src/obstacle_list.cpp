#include "mowbite_obstacles/obstacle_list.h"

#include <algorithm>
#include <cmath>

namespace mowbite_obstacles {

namespace {

constexpr double kDay = 86400.0;
// Positions in the list to the cm, times to the ms.
double Round(double value, double step) {
  return std::round(value / step) * step;
}

}  // namespace

ObstacleList::ObstacleList(ListLimits limits) : limits_(limits) {}

ObstacleList::Seen ObstacleList::OnConfirmed(const Track& track) {
  if (const uint32_t known = EntryOf(track.id)) return {known, false};
  Entry* same = nullptr;
  for (auto& [id, entry] : entries_) {
    if (entry.kind != track.kind) continue;
    const bool match = Living(track.kind) ? track.last_seen - entry.last_t <= limits_.same_living
                                          : std::hypot(entry.x - track.x, entry.y - track.y) <= limits_.same_place;
    if (match && (!same || entry.last_t > same->last_t)) same = &entry;
  }
  if (same) {
    ++same->count;
    by_track_[track.id] = same->id;
    Follow(track);
    return {same->id, false};
  }
  Entry entry;
  entry.id = next_id_++;
  entry.t = track.first_seen;
  entry.last_t = track.first_seen;
  entry.kind = track.kind;
  entry.class_name = track.class_name;
  entry.confidence = track.score;
  entry.camera_id = track.source;
  entries_[entry.id] = entry;
  by_track_[track.id] = entry.id;
  Follow(track);
  return {entry.id, true};
}

bool ObstacleList::Follow(const Track& track) {
  Entry* entry = Mutable(EntryOf(track.id));
  if (!entry) return false;
  const double last_t = entry->last_t, x = entry->x, y = entry->y, r = entry->r, confidence = entry->confidence;
  entry->last_t = std::max(entry->last_t, track.last_seen);
  entry->x = Round(track.x, 0.01);
  entry->y = Round(track.y, 0.01);
  entry->r = Round(track.radius, 0.01);
  if (track.score > entry->confidence && !track.class_name.empty()) {
    entry->confidence = track.score;
    entry->class_name = track.class_name;
  }
  if (entry->camera_id.empty()) entry->camera_id = track.source;
  return entry->last_t != last_t || entry->x != x || entry->y != y || entry->r != r || entry->confidence != confidence;
}

uint32_t ObstacleList::EntryOf(uint32_t track) const {
  const auto known = by_track_.find(track);
  return known == by_track_.end() || !entries_.count(known->second) ? 0 : known->second;
}

const Entry* ObstacleList::Get(uint32_t id) const {
  const auto it = entries_.find(id);
  return it == entries_.end() ? nullptr : &it->second;
}

Entry* ObstacleList::Mutable(uint32_t id) {
  return const_cast<Entry*>(Get(id));
}

bool ObstacleList::SetPhoto(uint32_t id, const Photo& photo) {
  Entry* entry = Mutable(id);
  if (!entry || entry->photo) return false;
  entry->photo = photo;
  return true;
}

void ObstacleList::DropPhoto(uint32_t id) {
  if (Entry* entry = Mutable(id)) entry->photo.reset();
}

uint32_t ObstacleList::ByPhoto(const std::string& name) const {
  for (const auto& [id, entry] : entries_) {
    if (entry.photo && entry.photo->name == name) return id;
  }
  return 0;
}

std::vector<std::string> ObstacleList::Remove(uint32_t id) {
  std::vector<std::string> photos;
  auto it = id == 0 ? entries_.begin() : entries_.find(id);
  const auto end = id == 0 ? entries_.end() : (it == entries_.end() ? it : std::next(it));
  while (it != end) {
    if (it->second.photo) photos.push_back(it->second.photo->name);
    it = entries_.erase(it);
  }
  for (auto it = by_track_.begin(); it != by_track_.end();) {
    it = entries_.count(it->second) ? std::next(it) : by_track_.erase(it);
  }
  return photos;
}

int64_t ObstacleList::Bytes() const {
  int64_t bytes = 0;
  for (const auto& [id, entry] : entries_) bytes += kEntryBytes + (entry.photo ? entry.photo->bytes : 0);
  return bytes;
}

std::vector<std::string> ObstacleList::Prune(double now) {
  std::vector<std::string> photos;
  const auto drop = [&](uint32_t id) {
    for (std::string& name : Remove(id)) photos.push_back(std::move(name));
  };
  if (now > 0.0) {
    std::vector<uint32_t> old;
    for (const auto& [id, entry] : entries_) {
      if (entry.last_t < now - limits_.days * kDay) old.push_back(id);
    }
    for (uint32_t id : old) drop(id);
  }
  while (!entries_.empty() && (entries_.size() > limits_.entries || Bytes() > limits_.bytes)) {
    const auto oldest = std::min_element(entries_.begin(), entries_.end(), [](const auto& a, const auto& b) {
      return a.second.last_t < b.second.last_t || (a.second.last_t == b.second.last_t && a.first < b.first);
    });
    drop(oldest->first);
  }
  return photos;
}

Json ObstacleList::EntryJson(const Entry& entry) {
  Json photo = nullptr;
  if (entry.photo) {
    const Photo& p = *entry.photo;
    photo = {{"name", p.name},   {"sha256", p.sha256}, {"bytes", p.bytes},
             {"width", p.width}, {"height", p.height},
             {"box", {{"x", Round(p.box_x, 0.1)}, {"y", Round(p.box_y, 0.1)}, {"w", Round(p.box_w, 0.1)},
                      {"h", Round(p.box_h, 0.1)}}}};
  }
  return {{"id", entry.id},
          {"t", Round(entry.t, 0.001)},
          {"last_t", Round(entry.last_t, 0.001)},
          {"count", entry.count},
          {"x", entry.x},
          {"y", entry.y},
          {"r", entry.r},
          {"kind", KindName(entry.kind)},
          {"class", entry.class_name},
          {"confidence", Round(entry.confidence, 0.001)},
          {"camera_id", entry.camera_id},
          {"action", entry.action},
          {"image", photo}};
}

Json ObstacleList::ToJson(double since) const {
  std::vector<const Entry*> picked;
  for (const auto& [id, entry] : entries_) {
    if (since <= 0.0 || entry.last_t > since) picked.push_back(&entry);
  }
  std::sort(picked.begin(), picked.end(), [](const Entry* a, const Entry* b) {
    return a->last_t > b->last_t || (a->last_t == b->last_t && a->id > b->id);
  });
  Json out = Json::array();
  for (const Entry* entry : picked) out.push_back(EntryJson(*entry));
  return out;
}

std::string ObstacleList::Text() const {
  Json entries = Json::array();
  for (const auto& [id, entry] : entries_) entries.push_back(EntryJson(entry));
  return Json{{"next_id", next_id_}, {"entries", entries}}.dump();
}

bool ObstacleList::Load(const std::string& text) {
  const Json json = Json::parse(text, nullptr, false);
  if (!json.is_object() || !json.contains("entries") || !json["entries"].is_array()) return false;
  std::map<uint32_t, Entry> entries;
  uint32_t next_id = json.value("next_id", 1u);
  for (const Json& item : json["entries"]) {
    if (!item.is_object()) continue;
    try {
      Entry entry;
      entry.id = item.at("id").get<uint32_t>();
      entry.t = item.at("t").get<double>();
      entry.last_t = item.at("last_t").get<double>();
      entry.count = std::max(1, item.at("count").get<int>());
      entry.x = item.at("x").get<double>();
      entry.y = item.at("y").get<double>();
      entry.r = item.at("r").get<double>();
      if (entry.id == 0 || !ParseKind(item.at("kind").get<std::string>(), &entry.kind)) continue;
      entry.class_name = item.at("class").get<std::string>();
      entry.confidence = item.at("confidence").get<double>();
      entry.camera_id = item.at("camera_id").get<std::string>();
      entry.action = item.at("action").get<std::string>();
      const Json& image = item.at("image");
      if (image.is_object()) {
        Photo photo;
        photo.name = image.at("name").get<std::string>();
        photo.sha256 = image.at("sha256").get<std::string>();
        photo.bytes = image.at("bytes").get<int64_t>();
        photo.width = image.at("width").get<int>();
        photo.height = image.at("height").get<int>();
        const Json& box = image.at("box");
        photo.box_x = box.at("x").get<double>();
        photo.box_y = box.at("y").get<double>();
        photo.box_w = box.at("w").get<double>();
        photo.box_h = box.at("h").get<double>();
        entry.photo = photo;
      }
      next_id = std::max(next_id, entry.id + 1);
      entries[entry.id] = std::move(entry);
    } catch (const Json::exception&) {
      continue;
    }
  }
  entries_ = std::move(entries);
  by_track_.clear();
  next_id_ = next_id;
  return true;
}

}  // namespace mowbite_obstacles
