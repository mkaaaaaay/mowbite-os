// The obstacles for the app: the list and its photos in a folder of data_dir, vision.obstacles.*, and the events
// OBSTACLE_SEEN and OBSTACLES_CLEARED. No ROS; times are s of the clock the node gives (unix s on the mower).
#ifndef MOWBITE_OBSTACLES_OBSTACLE_SERVICE_H
#define MOWBITE_OBSTACLES_OBSTACLE_SERVICE_H

#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_obstacles/obstacle_list.h"

namespace mowbite_obstacles {

// obstacle_mode, its value the index in mowbite_api::ObstacleModes().
enum class Mode { kOff = 0, kShow = 1, kStop = 2, kAvoid = 3 };

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
  // The list is written at most this often (s) after a change, at once on clear; while entries only follow their
  // tracks, at most every touch_gap.
  double save_gap = 5.0;
  double touch_gap = 60.0;
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
  // The setting and the highest mode of the platform (hw/capabilities); the mode is the lower one.
  void SetMode(int mode);
  void SetModeMax(int mode_max);
  void SetPhotos(bool on);
  Mode mode() const { return static_cast<Mode>(mowbite_api::EffectiveObstacleMode(setting_, mode_max_)); }
  // Whether the sensors are to send images: photos on and the obstacles shown.
  bool PhotosWanted() const { return photos_ && mode() != Mode::kOff; }
  // The sight of the sensors as the tracker has it now.
  void SetSight(bool ok, const std::string& reason, const std::string& source, double now);

  // After a frame: confirmed tracks become entries, the seen ones move their entries, ended ones are forgotten.
  void OnFrame(const Tracker& tracker, const FrameResult& frame, double now);
  void OnEnded(const std::vector<uint32_t>& tracks);
  void OnImage(const Tracker& tracker, Image image);
  // What the mower did about a track (obstacles/action), for its entry. avoided with keep_out_r above 0: the circle
  // goes into the entry with OBSTACLE_AVOIDED; with 0 it leaves the entry again, also after the track ended.
  void OnAction(uint32_t track, const std::string& action, double keep_out_r = 0.0, double left_out = 0.0,
                const std::string& area_id = "", const std::string& area_name = "");
  // Events that waited for a photo, writing the list, the limits.
  void Tick(double now);
  // Writes the list now if it changed.
  void Flush();

  // vision.obstacles.*; -32601 for another method.
  Outcome Call(const std::string& method, const Json& params);
  // Details of OBSTACLE_SEEN, OBSTACLE_AVOIDED and OBSTACLES_CLEARED by type, in their order.
  std::vector<std::pair<std::string, Json>> TakeEvents();

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
  Outcome Clear(const Json& params);

  ServiceConfig config_;
  std::function<void(const std::string&)> warn_;
  ObstacleList list_;
  int setting_ = 0;
  int mode_max_ = 1;
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
  // The entries of the circles kept out of, by the track they were made for.
  std::map<uint32_t, uint32_t> circles_;
  bool dirty_ = false;
  bool touched_ = false;
  double saved_at_ = -1e18;
  double pruned_at_ = -1e18;
};

}  // namespace mowbite_obstacles

#endif  // MOWBITE_OBSTACLES_OBSTACLE_SERVICE_H
