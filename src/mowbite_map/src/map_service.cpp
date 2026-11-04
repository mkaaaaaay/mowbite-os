#include "mowbite_map/map_service.h"

#include <cstdio>
#include <random>

#include "mowbite_map/sha256.h"

namespace mowbite_map {

namespace {

std::string RandomId() {
  static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<size_t> pick(0, sizeof(alphabet) - 2);
  std::string id(32, '\0');
  for (auto& c : id) c = alphabet[pick(rng)];
  return id;
}

RpcOutcome Failure(int code, std::string message) {
  RpcOutcome outcome;
  outcome.ok = false;
  outcome.code = code;
  outcome.message = std::move(message);
  return outcome;
}

// The SAVE_FAILED code of an edit for a failure of Commit.
std::string SaveFailedCode(int rpc_code) {
  switch (rpc_code) {
    case kRpcRefused: return "DATUM";
    case kRpcInvalidParams: return "TOO_LARGE";
    default: return "STORAGE";
  }
}

// 7 decimals of a degree are about a centimetre.
std::string Describe(const Datum& datum) {
  char text[64];
  std::snprintf(text, sizeof(text), "%.7f, %.7f", datum.lat, datum.lon);
  return text;
}

}  // namespace

const char* DatumStateName(DatumState state) {
  switch (state) {
    case DatumState::kUnknown: return "unknown";
    case DatumState::kEmpty: return "empty";
    case DatumState::kMatch: return "match";
    case DatumState::kMismatch: return "mismatch";
    case DatumState::kUnbound: return "unbound";
  }
  return "unknown";
}

bool operator==(const Status& a, const Status& b) {
  return a.datum == b.datum && a.mowing_allowed == b.mowing_allowed && a.reason == b.reason && a.areas == b.areas &&
         a.docking_stations == b.docking_stations;
}

MapService::MapService(const Config& config)
    : store_(config.data_dir, config.max_bytes),
      tolerance_m_(config.datum_tolerance_m),
      new_id_(config.new_id ? config.new_id : RandomId) {
  MapStore::Loaded loaded = store_.Load();
  if (loaded.map) {
    map_ = std::move(*loaded.map);
    on_disk_ = loaded.source == "map.json";
    const std::string summary = std::to_string(map_.areas.size()) + " areas and " +
                                std::to_string(map_.docking_stations.size()) + " docking stations from " +
                                loaded.source;
    notices_.push_back({loaded.source == "map.json" ? Notice::Level::kInfo : Notice::Level::kWarn,
                        "Loaded the map: " + summary});
    if (!loaded.problem.empty()) notices_.push_back({Notice::Level::kWarn, "Map file problem: " + loaded.problem});
  } else if (!loaded.problem.empty()) {
    notices_.push_back(
        {Notice::Level::kError, "No usable map file (" + loaded.problem + "), starting with an empty map"});
  } else {
    notices_.push_back({Notice::Level::kInfo, "No map file yet, starting with an empty map"});
  }
  text_ = ToText(map_);
  changed_ = true;

  MapStore::LoadedDatum datum = store_.LoadDatum();
  recorded_ = datum.datum;
  if (!datum.problem.empty()) notices_.push_back({Notice::Level::kWarn, "Datum file problem: " + datum.problem});
  Refresh();
}

const std::vector<std::string>& MapService::Methods() {
  static const std::vector<std::string> methods = {"map.replace", "map.area_properties"};
  return methods;
}

std::optional<RpcOutcome> MapService::Call(const std::string& method, const json& params) {
  if (method == "map.replace") return Replace(params);
  if (method == "map.area_properties") return AreaProperties();
  return std::nullopt;
}

void MapService::SetDatum(const Datum& datum) {
  if (!IsValid(datum)) {
    notices_.push_back({Notice::Level::kError, "Ignoring an invalid datum from the GNSS"});
    return;
  }
  current_ = datum;
  Refresh();
}

bool MapService::TakeChanged() {
  const bool changed = changed_;
  changed_ = false;
  return changed;
}

bool MapService::TakeStatusChanged() {
  const bool changed = status_changed_;
  status_changed_ = false;
  return changed;
}

std::vector<Notice> MapService::TakeNotices() {
  std::vector<Notice> notices;
  notices.swap(notices_);
  return notices;
}

std::vector<Event> MapService::TakeEvents() {
  std::vector<Event> events;
  events.swap(events_);
  return events;
}

RpcOutcome MapService::AreaProperties() const {
  std::vector<std::string> names = AreaPropertyNames();
  if (planner_offered_) names.insert(names.end(), PlannerPropertyNames().begin(), PlannerPropertyNames().end());
  RpcOutcome outcome;
  outcome.result = names;
  return outcome;
}

std::optional<RpcOutcome> MapService::HoldRefusal() const {
  if (hold_until_ <= 0) return std::nullopt;
  RpcOutcome outcome = Failure(kRpcHold, "An update holds the mower. The map changes after it");
  outcome.data = json{{"reason", "HOLD"}, {"until", hold_until_}};
  return outcome;
}

// params is [map], the same shape as map/json, like with OpenMower.
RpcOutcome MapService::Replace(const json& params) {
  if (auto held = HoldRefusal()) return *held;
  if (!params.is_array() || params.size() != 1) return Failure(kRpcInvalidParams, "Missing map parameter");
  return ReplaceMap(params[0]);
}

RpcOutcome MapService::ReplaceChecked(const std::string& map, const std::string& expected_sha256) {
  if (auto held = HoldRefusal()) return *held;
  if (!expected_sha256.empty()) {
    if (!IsSha256Hex(expected_sha256)) return Failure(kRpcInvalidParams, "expected_sha256 is not 64 lower-case hex digits");
    const std::string current = Sha256Hex(text_);
    if (current.empty()) return Failure(kRpcInternal, "Cannot make the SHA-256 of the map");
    if (current != expected_sha256) return Failure(kRpcMapChanged, "map changed meanwhile");
  }
  const json parsed = json::parse(map, nullptr, false);
  if (parsed.is_discarded()) return Failure(kRpcInvalidParams, "Invalid map: not valid JSON");
  return ReplaceMap(parsed);
}

RpcOutcome MapService::ReplaceMap(const json& map) {
  ParseResult parsed = ParseMap(map);
  if (!parsed.map) return Failure(kRpcInvalidParams, "Invalid map: " + parsed.error);
  const size_t areas = parsed.map->areas.size();
  RpcOutcome outcome = Commit(std::move(*parsed.map), true);
  if (!outcome.ok) return outcome;
  notices_.push_back({Notice::Level::kInfo, "Stored the map of map.replace (" + std::to_string(areas) + " areas)"});
  outcome.result = "Successfully stored map (" + std::to_string(areas) + " areas)";
  return outcome;
}

EditResult MapService::AddArea(const std::string& type, const std::string& name, const std::vector<Point>& outline,
                               const std::vector<std::vector<Point>>& obstacles) {
  MapData candidate = map_;
  Area area;
  area.id = NewId(candidate);
  area.name = name;
  area.type = type;
  area.outline = outline;
  const std::string id = area.id;
  candidate.areas.push_back(std::move(area));
  for (const auto& ring : obstacles) {
    Area obstacle;
    obstacle.id = NewId(candidate);
    obstacle.type = "obstacle";
    obstacle.outline = ring;
    candidate.areas.push_back(std::move(obstacle));
  }
  std::string what = "area '" + name + "' (" + type + ")";
  if (!obstacles.empty()) what += " with " + std::to_string(obstacles.size()) + " obstacles";
  return Apply(candidate, id, what);
}

EditResult MapService::SetDock(const Point& position, double heading) {
  DockingStation dock;
  dock.id = NewId(map_);
  dock.name = "Docking Station";
  dock.position = position;
  dock.heading = heading;
  MapData candidate = map_;
  candidate.docking_stations = {dock};
  return Apply(candidate, dock.id, "docking station");
}

EditResult MapService::Apply(const MapData& candidate, const std::string& id, const std::string& what) {
  // The checks of map.replace, which also close the ring of a new area.
  ParseResult parsed = ParseMap(ToJson(candidate));
  if (!parsed.map) return {false, "", "Invalid " + what + ": " + parsed.error, "INVALID"};
  RpcOutcome outcome = Commit(std::move(*parsed.map), false);
  if (!outcome.ok) return {false, "", outcome.message, SaveFailedCode(outcome.code)};
  notices_.push_back({Notice::Level::kInfo, "Stored the " + what + ", id " + id});
  return {true, id, ""};
}

std::string MapService::NewId(const MapData& taken) const {
  // Not in use yet; a made-up source that repeats gives up after a few tries.
  for (int attempt = 0; attempt < 16; ++attempt) {
    std::string id = new_id_();
    bool used = false;
    for (const auto& area : taken.areas) used = used || area.id == id;
    for (const auto& dock : taken.docking_stations) used = used || dock.id == id;
    if (!used) return id;
  }
  return new_id_();
}

std::string MapService::Refusal(const MapData& candidate, bool replaces) const {
  // Emptying the map is always possible, it is the way out of a map that belongs to another datum.
  if (candidate.Empty()) return "";
  switch (status_.datum) {
    case DatumState::kUnknown:
      return "The datum of the GNSS is not known yet (datum_lat and datum_long), so the map cannot be tied to it";
    case DatumState::kMismatch:
      return status_.reason + ". Set datum_lat and datum_long back, or send an empty map to start over";
    case DatumState::kUnbound:
      if (replaces) return "";
      return "The map has no recorded datum: send it again with map.replace to tie it to the datum in use";
    case DatumState::kEmpty:
    case DatumState::kMatch: return "";
  }
  return "";
}

RpcOutcome MapService::Commit(MapData candidate, bool replaces) {
  const std::string refusal = Refusal(candidate, replaces);
  if (!refusal.empty()) return Failure(kRpcRefused, refusal);
  std::string text = ToText(candidate);
  if (text.size() > store_.max_bytes()) {
    return Failure(kRpcInvalidParams, "The map is too large: " + std::to_string(text.size()) + " bytes, the limit is " +
                                          std::to_string(store_.max_bytes()));
  }
  // A map with areas gets the datum in use if it has none of its own (the refusal has seen to it that one is known).
  const bool bind = !candidate.Empty() && current_ && (!recorded_ || DistanceM(*recorded_, *current_) > tolerance_m_);
  // The same map again changes nothing, and does not push the previous version out of map.json.bak.
  if (text == text_ && on_disk_ && !bind) return {};
  const std::string error = store_.Save(text);
  if (!error.empty()) return Failure(kRpcInternal, "Cannot store the map: " + error);
  map_ = std::move(candidate);
  text_ = std::move(text);
  on_disk_ = true;
  changed_ = true;
  // The datum goes after the map: if it fails the map stays without one, which blocks mowing and does no harm.
  if (bind) {
    const std::string datum_error = store_.SaveDatum(*current_);
    if (!datum_error.empty()) {
      Refresh();
      return Failure(kRpcInternal, "Stored the map but not its datum: " + datum_error);
    }
    recorded_ = current_;
  }
  Refresh();
  return {};
}

Status MapService::Evaluate() const {
  Status status;
  status.areas = map_.areas.size();
  status.docking_stations = map_.docking_stations.size();
  if (!current_) {
    status.datum = DatumState::kUnknown;
    status.reason = "The datum of the GNSS is not known yet";
  } else if (map_.Empty()) {
    status.datum = DatumState::kEmpty;
    status.reason = "The map is empty";
  } else if (!recorded_) {
    status.datum = DatumState::kUnbound;
    status.reason = "The map has no recorded datum";
  } else if (DistanceM(*recorded_, *current_) > tolerance_m_) {
    char apart[32];
    std::snprintf(apart, sizeof(apart), "%.2f", DistanceM(*recorded_, *current_));
    status.datum = DatumState::kMismatch;
    status.reason = "The map was recorded for the datum " + Describe(*recorded_) + ", the datum in use is " +
                    Describe(*current_) + " (" + apart + " m apart)";
  } else {
    status.datum = DatumState::kMatch;
    status.mowing_allowed = true;
  }
  return status;
}

void MapService::Refresh() {
  Status next = Evaluate();
  if (next == status_) return;
  const DatumState before = status_.datum;
  status_ = std::move(next);
  status_changed_ = true;
  if (status_.datum == before) return;
  switch (status_.datum) {
    case DatumState::kMismatch:
    case DatumState::kUnbound:
      notices_.push_back({Notice::Level::kError, "No mowing, the map does not fit the datum: " + status_.reason});
      // Names and rating as the events of mowbite_logic (masterplan 3.1a).
      events_.push_back({"MAP_DATUM", json{{"state", DatumStateName(status_.datum)},
                                           {"reason", "MAP_MISMATCH"},
                                           {"message", status_.reason},
                                           {"severity", "error"},
                                           {"resume", "user"}}});
      break;
    case DatumState::kMatch: notices_.push_back({Notice::Level::kInfo, "The map belongs to the datum in use"}); break;
    case DatumState::kUnknown:
    case DatumState::kEmpty: break;
  }
}

}  // namespace mowbite_map
