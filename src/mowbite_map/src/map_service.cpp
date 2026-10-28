#include "mowbite_map/map_service.h"

#include <random>

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

}  // namespace

MapService::MapService(const Config& config)
    : store_(config.data_dir, config.max_bytes), new_id_(config.new_id ? config.new_id : RandomId) {
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

bool MapService::TakeChanged() {
  const bool changed = changed_;
  changed_ = false;
  return changed;
}

std::vector<Notice> MapService::TakeNotices() {
  std::vector<Notice> notices;
  notices.swap(notices_);
  return notices;
}

RpcOutcome MapService::AreaProperties() const {
  RpcOutcome outcome;
  outcome.result = AreaPropertyNames();
  return outcome;
}

// params is [map], the same shape as map/json, like with OpenMower.
RpcOutcome MapService::Replace(const json& params) {
  if (!params.is_array() || params.size() != 1) return Failure(kRpcInvalidParams, "Missing map parameter");
  ParseResult parsed = ParseMap(params[0]);
  if (!parsed.map) return Failure(kRpcInvalidParams, "Invalid map: " + parsed.error);
  const size_t areas = parsed.map->areas.size();
  RpcOutcome outcome = Commit(std::move(*parsed.map));
  if (!outcome.ok) return outcome;
  notices_.push_back({Notice::Level::kInfo, "Stored the map of map.replace (" + std::to_string(areas) + " areas)"});
  outcome.result = "Successfully stored map (" + std::to_string(areas) + " areas)";
  return outcome;
}

EditResult MapService::AddArea(const std::string& type, const std::string& name, const std::vector<Point>& outline) {
  Area area;
  area.id = NewId();
  area.name = name;
  area.type = type;
  area.outline = outline;
  MapData candidate = map_;
  candidate.areas.push_back(std::move(area));
  return Apply(candidate, candidate.areas.back().id, "area '" + name + "' (" + type + ")");
}

EditResult MapService::SetDock(const Point& position, double heading) {
  DockingStation dock;
  dock.id = NewId();
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
  if (!parsed.map) return {false, "", "Invalid " + what + ": " + parsed.error};
  RpcOutcome outcome = Commit(std::move(*parsed.map));
  if (!outcome.ok) return {false, "", outcome.message};
  notices_.push_back({Notice::Level::kInfo, "Stored the " + what + ", id " + id});
  return {true, id, ""};
}

std::string MapService::NewId() const {
  // Not in use yet; a made-up source that repeats gives up after a few tries.
  for (int attempt = 0; attempt < 16; ++attempt) {
    std::string id = new_id_();
    bool used = false;
    for (const auto& area : map_.areas) used = used || area.id == id;
    for (const auto& dock : map_.docking_stations) used = used || dock.id == id;
    if (!used) return id;
  }
  return new_id_();
}

RpcOutcome MapService::Commit(MapData candidate) {
  std::string text = ToText(candidate);
  if (text.size() > store_.max_bytes()) {
    return Failure(kRpcInvalidParams, "The map is too large: " + std::to_string(text.size()) + " bytes, the limit is " +
                                          std::to_string(store_.max_bytes()));
  }
  // The same map again changes nothing, and does not push the previous version out of map.json.bak.
  if (text == text_ && on_disk_) return {};
  const std::string error = store_.Save(text);
  if (!error.empty()) return Failure(kRpcInternal, "Cannot store the map: " + error);
  map_ = std::move(candidate);
  text_ = std::move(text);
  on_disk_ = true;
  changed_ = true;
  return {};
}

}  // namespace mowbite_map
