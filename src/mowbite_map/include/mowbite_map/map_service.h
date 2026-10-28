// The map of the mower and the RPC methods around it: map.replace and map.area_properties. The map belongs to the
// datum it was recorded in; with another datum in use it is neither changed nor mowed.
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_map/datum.h"
#include "mowbite_map/map_model.h"
#include "mowbite_map/map_store.h"

namespace mowbite_map {

// JSON-RPC error codes of the answers
constexpr int kRpcInvalidParams = -32602;
constexpr int kRpcInternal = -32603;
// A valid request the mower does not do now
constexpr int kRpcRefused = -32000;

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

// An entry for the event history of the app.
struct Event {
  std::string type;
  json details;
};

enum class DatumState {
  // tron_gnss has not told its datum yet
  kUnknown,
  // no areas and no docking station, nothing to tie to a datum
  kEmpty,
  // the map was recorded in the datum in use
  kMatch,
  // the map was recorded in another datum
  kMismatch,
  // the map is there, but not the datum it was recorded in
  kUnbound,
};

const char* DatumStateName(DatumState state);

struct Status {
  DatumState datum = DatumState::kUnknown;
  // Mowing needs a map of the datum in use.
  bool mowing_allowed = false;
  // Why mowing is not allowed, empty if it is.
  std::string reason;
  size_t areas = 0;
  size_t docking_stations = 0;
};

bool operator==(const Status& a, const Status& b);

// Not thread safe, the node calls it from one callback group.
class MapService {
 public:
  struct Config {
    std::string data_dir;
    // The largest map.json; MQTT drops messages from about 2 MB.
    size_t max_bytes = 1500000;
    // Datums closer than this (m) count as the same; a new survey of the same spot is never exact.
    double datum_tolerance_m = 0.01;
    // Makes the ids of new areas and docking stations; random 32 characters like OpenMower if empty.
    std::function<std::string()> new_id;
  };

  // Loads map.json, or its previous version, and the datum it was recorded in from the data directory.
  explicit MapService(const Config& config);

  static const std::vector<std::string>& Methods();
  // The answer to a method of this service, nothing for any other method.
  std::optional<RpcOutcome> Call(const std::string& method, const json& params);

  // For the logic: adds an area of a recording (outline open or closed) with a new id.
  EditResult AddArea(const std::string& type, const std::string& name, const std::vector<Point>& outline);
  // For the logic: the docking station becomes the only one, with a new id, like with OpenMower.
  EditResult SetDock(const Point& position, double heading);

  // Whether a node offers planner.settings: then the app learns from map.area_properties that the map keeps the
  // settings of the planner (they are kept either way).
  void SetPlannerSettingsOffered(bool offered) {
    planner_offered_ = offered;
  }

  // The datum tron_gnss works in. Checks the map against it, and ties a first map to it.
  void SetDatum(const Datum& datum);

  const MapData& map() const {
    return map_;
  }
  // The text of map/json: the map as stored.
  const std::string& map_text() const {
    return text_;
  }
  const Status& status() const {
    return status_;
  }
  // True once after the map was loaded or changed.
  bool TakeChanged();
  // True once after the status changed.
  bool TakeStatusChanged();
  std::vector<Notice> TakeNotices();
  std::vector<Event> TakeEvents();

 private:
  RpcOutcome Replace(const json& params);
  RpcOutcome AreaProperties() const;
  // Stores the map and makes it the current one. replaces: the whole map is sent again (map.replace).
  RpcOutcome Commit(MapData candidate, bool replaces);
  // Checks a map made by an edit the way a replaced map is checked, then stores it.
  EditResult Apply(const MapData& candidate, const std::string& id, const std::string& what);
  // Why this map may not be stored now, empty if it may.
  std::string Refusal(const MapData& candidate, bool replaces) const;
  Status Evaluate() const;
  // Works out the status again; a new mismatch or missing datum goes to the log and the events.
  void Refresh();
  std::string NewId() const;

  MapStore store_;
  double tolerance_m_;
  std::function<std::string()> new_id_;
  MapData map_;
  std::string text_;
  // The datum the map was recorded in, and the one in use.
  std::optional<Datum> recorded_;
  std::optional<Datum> current_;
  Status status_;
  // Whether map.json holds the current map; false when it came from the previous version.
  bool on_disk_ = false;
  bool changed_ = false;
  bool status_changed_ = false;
  bool planner_offered_ = false;
  std::vector<Notice> notices_;
  std::vector<Event> events_;
};

}  // namespace mowbite_map
