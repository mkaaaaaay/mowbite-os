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
// The map is not the one the caller last read (map/replace with expected_sha256)
constexpr int kRpcMapChanged = -32001;
// An update holds the mower (system.hold): a rollback would drop the change without a word
constexpr int kRpcHold = -32004;

// The answer of an RPC method: a result, or an error with its code.
struct RpcOutcome {
  bool ok = true;
  json result;
  int code = 0;
  std::string message;
  // Details of an error for the caller, null for none.
  json data;
};

// The answer to an edit for the logic: the id of what was added, or why it was refused.
struct EditResult {
  bool ok = false;
  std::string id;
  std::string message;
  // SAVE_FAILED code when refused: INVALID, DATUM, TOO_LARGE or STORAGE.
  std::string code;
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

  // map.replace for the planner (service map/replace): the map as JSON text. expected_sha256 is the SHA-256 of the
  // map_text() the caller read; another map text writes nothing (kRpcMapChanged), empty skips the check.
  RpcOutcome ReplaceChecked(const std::string& map, const std::string& expected_sha256);

  // For the logic: adds an area of a recording (outline open or closed) with a new id, and the obstacles recorded
  // inside it as areas of type obstacle in the same write. recorded_body: JSON objects for properties.recorded_body,
  // empty for none; obstacles_recorded_body is empty or has one entry per obstacle.
  EditResult AddArea(const std::string& type, const std::string& name, const std::vector<Point>& outline,
                     const std::vector<std::vector<Point>>& obstacles = {}, const std::string& recorded_body = "",
                     const std::vector<std::string>& obstacles_recorded_body = {});
  // For the logic: properties.recorded_body of every area whose recorded_body has this by becomes body (a JSON object
  // with the same by). areas: how many carry this by. None of them writes nothing.
  EditResult SetRecordedBody(const std::string& by, const std::string& body, int* areas);
  // For the logic: the docking station becomes the only one, with a new id, like with OpenMower.
  EditResult SetDock(const Point& position, double heading);

  // Whether a node offers planner.settings: then the app learns from map.area_properties that the map keeps the
  // settings of the planner (they are kept either way).
  void SetPlannerSettingsOffered(bool offered) {
    planner_offered_ = offered;
  }

  // The datum tron_gnss works in. Checks the map against it, and ties a first map to it.
  void SetDatum(const Datum& datum);
  // The hold of an update (logic/hold), until in unix s, 0 for none: map.replace and map/replace are refused with
  // kRpcHold and data {"reason": "HOLD", "until"} while it lasts.
  void SetHold(int64_t until) {
    hold_until_ = until;
  }

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
  // Checks and stores a map of map.replace or map/replace.
  RpcOutcome ReplaceMap(const json& map);
  RpcOutcome AreaProperties() const;
  // kRpcHold while an update holds the mower.
  std::optional<RpcOutcome> HoldRefusal() const;
  // Stores the map and makes it the current one. replaces: the whole map is sent again (map.replace).
  RpcOutcome Commit(MapData candidate, bool replaces);
  // Checks a map made by an edit the way a replaced map is checked, then stores it.
  EditResult Apply(const MapData& candidate, const std::string& id, const std::string& what);
  // Why this map may not be stored now, empty if it may.
  std::string Refusal(const MapData& candidate, bool replaces) const;
  Status Evaluate() const;
  // Works out the status again; a new mismatch or missing datum goes to the log and the events.
  void Refresh();
  // An id that is not in taken.
  std::string NewId(const MapData& taken) const;

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
  int64_t hold_until_ = 0;
  std::vector<Notice> notices_;
  std::vector<Event> events_;
};

}  // namespace mowbite_map
