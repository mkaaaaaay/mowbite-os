// The map of the mower and the RPC methods around it: map.replace and map.area_properties.
#pragma once

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
  };

  // Loads map.json, or its previous version, from the data directory.
  explicit MapService(const Config& config);

  static const std::vector<std::string>& Methods();
  // The answer to a method of this service, nothing for any other method.
  std::optional<RpcOutcome> Call(const std::string& method, const json& params);

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

  MapStore store_;
  MapData map_;
  std::string text_;
  // Whether map.json holds the current map; false when it came from the previous version.
  bool on_disk_ = false;
  bool changed_ = false;
  std::vector<Notice> notices_;
};

}  // namespace mowbite_map
