// The obstacles for the app: the list and its photos in a folder of data_dir, vision.obstacles.*, and the events
// OBSTACLE_SEEN and OBSTACLES_CLEARED. No ROS; times are s of the clock the node gives (unix s on the mower).
#ifndef MOWBITE_OBSTACLES_OBSTACLE_SERVICE_H
#define MOWBITE_OBSTACLES_OBSTACLE_SERVICE_H

#include <deque>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mowbite_obstacles/obstacle_list.h"
#include "mowbite_obstacles/tracker.h"

namespace mowbite_obstacles {

// obstacle_mode: what the obstacles do. Only off and show are taken until stop and avoid are built.
enum class Mode { kOff = 0, kShow = 1 };
constexpr int kHighestMode = 1;

struct Outcome {
  bool ok = false;
  Json result;
  int code = 0;
  std::string message;
};

struct ServiceConfig {
  // The folder of the list and the photos; empty: kept in memory only.
  std::string dir;
  ListLimits limits;
  // OBSTACLE_SEEN waits this long for the photo of a new entry (s).
  double photo_wait = 2.0;
  // Images kept for the tracks they may show.
  size_t images = 4;
  // OBSTACLE_SIGHT once the sight stays this long in a new state (s).
  double sight_settle = 2.0;
  // The list is written at most this often (s), at once on clear.
  double save_gap = 5.0;
  // The limits are kept this often (s).
  double prune_gap = 3600.0;
};

// An image of hw/obstacle_images.
struct Image {
  std::string source;
  double stamp = 0.0;
  int width = 0;
  int height = 0;
  std::string jpeg;
};

class ObstacleService {
 public:
  ObstacleService(ServiceConfig config, std::function<void(const std::string&)> warn);

  static const std::vector<std::string>& Methods();

  // Reads the list of the folder, removes photos it does not name, and keeps the limits.
  void Load(double now);
  void SetMode(int mode);
  void SetPhotos(bool on);
  Mode mode() const { return mode_; }
  bool photos() const { return photos_; }
  // Whether the sensors are to send images: photos on and the obstacles shown.
  bool PhotosWanted() const { return photos_ && mode_ != Mode::kOff; }
  // The sight of the sensors as the tracker has it now.
  void SetSight(bool ok, const std::string& reason, const std::string& source, double now);

  // After a frame: confirmed tracks become entries, the seen ones move their entries, ended ones are forgotten.
  void OnFrame(const Tracker& tracker, const FrameResult& frame, double now);
  void OnEnded(const std::vector<uint32_t>& tracks);
  void OnImage(const Tracker& tracker, Image image, double now);
  // Events that waited for a photo, writing the list, the limits.
  void Tick(double now);
  // Writes the list now if it changed.
  void Flush();

  // vision.obstacles.*; nullopt for another method.
  std::optional<Outcome> Call(const std::string& method, const Json& params, double now);
  // Details of OBSTACLE_SEEN and OBSTACLES_CLEARED by type, in their order.
  std::vector<std::pair<std::string, Json>> TakeEvents();

  const ObstacleList& list() const { return list_; }

 private:
  struct Pending {
    uint32_t entry = 0;
    double until = 0.0;
  };

  void TryPhotos(const Tracker& tracker);
  void SavePhoto(uint32_t entry, const Image& image, const Shot& shot);
  void Seen(uint32_t entry);
  void RemovePhotos(const std::vector<std::string>& names);
  Outcome List(const Json& params) const;
  Outcome Piece(const Json& params) const;
  Outcome Clear(const Json& params, double now);

  ServiceConfig config_;
  std::function<void(const std::string&)> warn_;
  ObstacleList list_;
  Mode mode_ = Mode::kOff;
  bool photos_ = false;
  struct Sight {
    bool ok = true;
    std::string reason;
    std::string source;
    bool operator==(const Sight& other) const {
      return ok == other.ok && reason == other.reason && source == other.source;
    }
  };
  Sight sight_;
  double sight_since_ = 0.0;
  Sight told_;
  std::deque<Image> images_;
  std::vector<Pending> pending_;
  std::vector<std::pair<std::string, Json>> events_;
  bool dirty_ = false;
  double saved_at_ = -1e18;
  double pruned_at_ = -1e18;
};

}  // namespace mowbite_obstacles

#endif  // MOWBITE_OBSTACLES_OBSTACLE_SERVICE_H
