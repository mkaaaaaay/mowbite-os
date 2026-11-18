// The list of obstacles for the app, without files and without a clock: entries from confirmed tracks, merged where
// they are the same thing, their photos by name, the limits, and the text it is kept in.
#ifndef MOWBITE_OBSTACLES_OBSTACLE_LIST_H
#define MOWBITE_OBSTACLES_OBSTACLE_LIST_H

#include <cstdint>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_obstacles/tracker.h"

namespace mowbite_obstacles {

using Json = nlohmann::ordered_json;

struct Photo {
  std::string name;
  std::string sha256;
  int64_t bytes = 0;
  int width = 0;
  int height = 0;
  double box_x = 0.0;
  double box_y = 0.0;
  double box_w = 0.0;
  double box_h = 0.0;
};

// One entry as vision.obstacles.list gives it; times are unix s.
struct Entry {
  uint32_t id = 0;
  double t = 0.0;
  double last_t = 0.0;
  int count = 1;
  double x = 0.0;
  double y = 0.0;
  double r = 0.0;
  Kind kind = Kind::kObject;
  std::string class_name;
  double confidence = 0.0;
  std::string camera_id;
  // One of mowbite_api::ObstacleActions().
  std::string action = "seen";
  std::optional<Photo> photo;
};

struct ListLimits {
  size_t entries = 200;
  int64_t bytes = 20 * 1024 * 1024;
  double days = 14.0;
  // An object within this of an entry of objects is that entry (m).
  double same_place = 0.5;
  // A person or animal within this of the last time an entry of its kind was seen is that entry (s).
  double same_living = 30.0;
};

// Bytes an entry takes in the kept text, for the limit of bytes.
constexpr int64_t kEntryBytes = 400;

class ObstacleList {
 public:
  explicit ObstacleList(ListLimits limits = {});

  struct Seen {
    uint32_t id = 0;
    bool added = false;
  };
  // A track that was just confirmed: a new entry, or the entry of the same thing seen before. Times are those of
  // the track.
  Seen OnConfirmed(const Track& track);
  // A confirmed track seen again: its entry follows it. False when nothing changed.
  bool Follow(const Track& track);
  // The entry of a track while it lasts; 0 without one.
  uint32_t EntryOf(uint32_t track) const;
  void ForgetTrack(uint32_t track) { by_track_.erase(track); }

  const Entry* Get(uint32_t id) const;
  bool SetPhoto(uint32_t id, const Photo& photo);
  // What the mower did about it; false when nothing changed.
  bool SetAction(uint32_t id, const std::string& action);
  void DropPhoto(uint32_t id);
  // The entry with this photo name, 0 without one.
  uint32_t ByPhoto(const std::string& name) const;
  // Removes one entry or, with 0, all; returns the photos gone with them.
  std::vector<std::string> Remove(uint32_t id);
  // Keeps the limits: entries not seen for days, then the least recently seen until count and bytes fit. now is
  // unix s; 0 while the clock is not set leaves the days alone. Returns the photos gone.
  std::vector<std::string> Prune(double now);
  int64_t Bytes() const;
  size_t size() const { return entries_.size(); }
  const std::map<uint32_t, Entry>& entries() const { return entries_; }
  const ListLimits& limits() const { return limits_; }

  // The entries seen after since (all with 0), last seen first, as vision.obstacles.list has them.
  Json ToJson(double since) const;
  static Json EntryJson(const Entry& entry);
  // The kept text and back; Load takes what it can read and returns false for a text that is no list.
  std::string Text() const;
  bool Load(const std::string& text);

 private:
  Entry* Mutable(uint32_t id);

  ListLimits limits_;
  std::map<uint32_t, Entry> entries_;
  std::map<uint32_t, uint32_t> by_track_;
  uint32_t next_id_ = 1;
};

}  // namespace mowbite_obstacles

#endif  // MOWBITE_OBSTACLES_OBSTACLE_LIST_H
