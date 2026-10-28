// The map of the mower and the RPC methods around it: map.replace and map.area_properties.
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_map/map_model.h"
#include "mowbite_map/map_store.h"

namespace mowbite_map {

// JSON-RPC error codes of the answers
constexpr int kRpcInvalidParams = -32602;
constexpr int kRpcInternal = -32603;

// The answer of an RPC method: a result, or an error with its code.
struct RpcOutcome {
  bool ok = true;
  json result;
  int code = 0;
  std::string message;
};

// The answer to an edit for the logic: the id of what was added, or why it was refused.
struct EditResult {
  bool ok = false;
  std::string id;
  std::string message;
};

// A line for the log of the node.
struct Notice {
  enum class Level { kInfo, kWarn, kError };
  Level level = Level::kInfo;
  std::string text;
};

// Not thread safe, the node calls it from one callback group.
class MapService {
 public:
  struct Config {
    std::string data_dir;
    // The largest map.json; MQTT drops messages from about 2 MB.
    size_t max_bytes = 1500000;
    // Makes the ids of new areas and docking stations; random 32 characters like OpenMower if empty.
    std::function<std::string()> new_id;
  };

  // Loads map.json, or its previous version, from the data directory.
  explicit MapService(const Config& config);

  static const std::vector<std::string>& Methods();
  // The answer to a method of this service, nothing for any other method.
  std::optional<RpcOutcome> Call(const std::string& method, const json& params);

  // For the logic: adds an area of a recording (outline open or closed) with a new id.
  EditResult AddArea(const std::string& type, const std::string& name, const std::vector<Point>& outline);
  // For the logic: the docking station becomes the only one, with a new id, like with OpenMower.
  EditResult SetDock(const Point& position, double heading);

  const MapData& map() const {
    return map_;
  }
  // The text of map/json: the map as stored.
  const std::string& map_text() const {
    return text_;
  }
  // True once after the map was loaded or changed.
  bool TakeChanged();
  std::vector<Notice> TakeNotices();

 private:
  RpcOutcome Replace(const json& params);
  RpcOutcome AreaProperties() const;
  // Stores the map and makes it the current one.
  RpcOutcome Commit(MapData candidate);
  // Checks a map made by an edit the way a replaced map is checked, then stores it.
  EditResult Apply(const MapData& candidate, const std::string& id, const std::string& what);
  std::string NewId() const;

  MapStore store_;
  std::function<std::string()> new_id_;
  MapData map_;
  std::string text_;
  // Whether map.json holds the current map; false when it came from the previous version.
  bool on_disk_ = false;
  bool changed_ = false;
  std::vector<Notice> notices_;
};

}  // namespace mowbite_map
