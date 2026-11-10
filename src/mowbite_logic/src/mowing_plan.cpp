#include "mowbite_logic/mowing_plan.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace mowbite_logic {

namespace {

// 1 cm off the planned line and mm precision are plenty, and keep the answer small
constexpr double kSimplifyTolerance = 0.01;

// An error of the request, answered with its code.
struct PlanError {
  int code;
  std::string message;
};

[[noreturn]] void Invalid(const std::string& message) {
  throw PlanError{kPlanInvalidParams, message};
}

bool IsNumber(const json& j) {
  return j.is_number() && std::isfinite(j.get<double>());
}

// A list of {"x", "y"} points like in map.json.
Ring RequirePoints(const json& points, const std::string& key) {
  if (!points.is_array()) Invalid("parameter is not a list of points: " + key);
  Ring ring;
  for (const auto& p : points) {
    if (!p.is_object() || !p.contains("x") || !p.contains("y") || !p["x"].is_number() || !p["y"].is_number()) {
      Invalid("point without numeric x and y in parameter: " + key);
    }
    const double x = p["x"].get<double>();
    const double y = p["y"].get<double>();
    // the planner's polygons hold float32, anything that does not fit would become infinity
    if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x) > FLT_MAX || std::abs(y) > FLT_MAX) {
      Invalid("point out of range in parameter: " + key);
    }
    ring.push_back({x, y});
  }
  return ring;
}

double RequireNumber(const json& params, const char* key) {
  if (!IsNumber(params.at(key))) Invalid(std::string("non-numeric parameter: ") + key);
  return params.at(key).get<double>();
}

// The planner takes these as uint8
int RequireCount(const json& params, const char* key) {
  const json& v = params.at(key);
  if (!v.is_number_integer() || v.get<double>() < 0 || v.get<double>() > 255) {
    Invalid(std::string("parameter is not a whole number from 0 to 255: ") + key);
  }
  return v.get<int>();
}

// The map stores rings closed, the planner wants the points once.
Ring Opened(Ring ring) {
  if (ring.size() > 1 && ring.front().x == ring.back().x && ring.front().y == ring.back().y) ring.pop_back();
  return ring;
}

}  // namespace

std::vector<Piece> CutIntoPieces(const CoverageAnswer& answer, size_t max_poses) {
  const size_t count = std::min({answer.segment_start.size(), answer.segment_end.size(), answer.segment_type.size()});
  // A section has no way to the next one: the path jumps, a path of its own starts there.
  std::vector<size_t> section(count, 0);
  for (size_t i = 0; i < count; ++i) {
    const auto next = std::upper_bound(answer.section_start.begin(), answer.section_start.end(), answer.segment_start[i]);
    section[i] = next == answer.section_start.begin() ? 0 : static_cast<size_t>(next - answer.section_start.begin()) - 1;
  }
  // A transit goes with what comes after it in its section, so every path is one continuous drive.
  std::vector<bool> outline(count, false);
  bool following = false;
  size_t current = count > 0 ? section[count - 1] : 0;
  for (size_t i = count; i-- > 0;) {
    if (section[i] != current) {
      current = section[i];
      following = false;
    }
    if (answer.segment_type[i] != kSegmentTransit) following = answer.segment_type[i] == kSegmentPerimeter;
    outline[i] = following;
  }
  struct Run {
    bool is_outline;
    size_t first;
    size_t last;
    size_t section;
  };
  std::vector<Run> runs;
  for (size_t i = 0; i < count; ++i) {
    if (!runs.empty() && runs.back().is_outline == outline[i] && runs.back().section == section[i]) {
      runs.back().last = answer.segment_end[i];
    } else {
      runs.push_back({outline[i], answer.segment_start[i], answer.segment_end[i], section[i]});
    }
  }
  std::vector<Piece> pieces;
  const size_t step = std::max<size_t>(2, max_poses > 0 ? max_poses - 1 : 0);
  for (const auto& run : runs) {
    if (run.last < run.first + 2) continue;
    const size_t steps = run.last - run.first;
    // at least 2 steps a piece, the next one starts on the pose the last one ended on
    const size_t parts = std::min((steps + step - 1) / step, steps / 2);
    for (size_t k = 0; k < parts; ++k) {
      pieces.push_back({run.is_outline, run.first + steps * k / parts, run.first + steps * (k + 1) / parts});
    }
  }
  return pieces;
}

std::vector<size_t> SimplifiedIndices(const std::vector<Point2>& points, double tolerance) {
  const size_t n = points.size();
  std::vector<size_t> all(n);
  for (size_t i = 0; i < n; ++i) all[i] = i;
  if (n <= 2) return all;
  std::vector<bool> keep(n, false);
  keep.front() = keep.back() = true;
  std::vector<std::pair<size_t, size_t>> todo{{0, n - 1}};
  while (!todo.empty()) {
    const auto [first, last] = todo.back();
    todo.pop_back();
    const Point2& a = points[first];
    const Point2& b = points[last];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    double farthest = -1;
    size_t at = first;
    for (size_t i = first + 1; i < last; ++i) {
      const double d = length > 1e-12 ? std::abs(dy * (points[i].x - a.x) - dx * (points[i].y - a.y)) / length
                                      : std::hypot(points[i].x - a.x, points[i].y - a.y);
      if (d > farthest) {
        farthest = d;
        at = i;
      }
    }
    if (farthest > tolerance) {
      keep[at] = true;
      todo.push_back({first, at});
      todo.push_back({at, last});
    }
  }
  std::vector<size_t> kept;
  for (size_t i = 0; i < n; ++i) {
    if (keep[i]) kept.push_back(i);
  }
  return kept;
}

void MowingPlanner::SetMap(const std::string& text) {
  map_.clear();
  dock_.reset();
  map_readable_ = false;
  const json j = json::parse(text, nullptr, false);
  if (j.is_discarded() || !j.is_object() || !j.contains("areas") || !j["areas"].is_array()) return;
  try {
    if (j.contains("docking_stations") && j["docking_stations"].is_array()) {
      for (const auto& entry : j["docking_stations"]) {
        if (!entry.is_object() || !entry.contains("position") || !entry.contains("heading")) continue;
        const json properties =
            entry.contains("properties") && entry["properties"].is_object() ? entry["properties"] : json::object();
        if (!properties.value("active", true)) continue;
        const Ring position = RequirePoints(json::array({entry["position"]}), "docking_stations");
        dock_ = DockPose{position[0].x, position[0].y, RequireNumber(entry, "heading")};
        break;
      }
    }
    for (const auto& entry : j["areas"]) {
      if (!entry.is_object() || !entry.contains("id") || !entry["id"].is_string() || !entry.contains("outline")) continue;
      MapArea area;
      area.id = entry["id"].get<std::string>();
      const json properties =
          entry.contains("properties") && entry["properties"].is_object() ? entry["properties"] : json::object();
      area.type = properties.value("type", "");
      area.name = properties.value("name", "");
      area.active = properties.value("active", true);
      area.mowable = properties.value("mowable", true);
      area.outline = Opened(RequirePoints(entry["outline"], "outline"));
      map_.push_back(std::move(area));
    }
  } catch (const PlanError&) {
    map_.clear();
    dock_.reset();
    return;
  } catch (const json::exception&) {
    // a property of another type than the map writes it: not a map to plan from
    map_.clear();
    dock_.reset();
    return;
  }
  map_readable_ = true;
}

std::optional<DockPose> MowingPlanner::Dock() const {
  return map_readable_ ? dock_ : std::nullopt;
}

Reason MowingPlanner::TransitAreas(std::vector<Ring>* drivable, std::vector<Ring>* obstacles) const {
  drivable->clear();
  obstacles->clear();
  if (!map_fits_) return Reason(map_reason_).WithMessage("Cannot drive on the map: " + map_reason_.message);
  if (!map_readable_) return Reason("MAP_UNREADABLE", "Cannot drive on the map: map/json is not readable");
  for (const auto& area : map_) {
    if (!area.active) continue;
    if (area.type == "mow" || area.type == "nav") drivable->push_back(area.outline);
    if (area.type == "obstacle") obstacles->push_back(area.outline);
  }
  if (drivable->empty()) return Reason("NO_MOWING_AREA", "The map has no active mowing or navigation area");
  return {};
}

std::vector<JobArea> MowingPlanner::MowAreas() const {
  std::vector<JobArea> areas;
  if (!map_readable_) return areas;
  for (const auto& area : map_) {
    if (area.type == "mow") areas.push_back({area.id, area.name, area.active && area.mowable});
  }
  return areas;
}

Reason MowingPlanner::MapProblem() const {
  if (!map_fits_) return Reason(map_reason_).WithMessage("Cannot mow on the map: " + map_reason_.message);
  if (!map_readable_) return Reason("MAP_UNREADABLE", "Cannot mow on the map: map/json is not readable");
  return {};
}

void MowingPlanner::SetMapStatus(bool fits, const Reason& reason) {
  map_fits_ = fits;
  map_reason_ = reason.empty() ? MapReason(MapDatum::kMismatch, "") : reason;
}

PlanQuestion MowingPlanner::Prepare(const json& params) const {
  PlanQuestion question;
  try {
    if (!params.is_object()) Invalid("parameters must be an object");
    CoverageRequest& request = question.request;
    // Only what the request sets goes to the planner, which reads the area's own values from the map.
    std::optional<double> angle;
    std::optional<int> outline_count;
    std::optional<int> outline_overlap_count;
    std::optional<double> outline_offset;

    if (params.contains("area_id")) {
      if (!params["area_id"].is_string()) Invalid("non-string parameter: area_id");
      question.area_id = params["area_id"].get<std::string>();
      // The map is only trusted for the datum it was recorded in.
      if (!map_fits_ || !map_readable_) {
        throw PlanError{kPlanRefused, "Cannot plan from the map: " + (map_readable_ ? map_reason_.message : "The map is not readable")};
      }
      const MapArea* found = nullptr;
      for (const auto& area : map_) {
        if (area.id == question.area_id && area.type == "mow" && area.active && area.mowable) found = &area;
      }
      if (!found) Invalid("no mowing area with id: " + question.area_id);
      request.boundary = found->outline;
      // Where the mower may drive between the parts of the area; the area itself is not one of them.
      for (const auto& area : map_) {
        if (&area != found && area.active && (area.type == "mow" || area.type == "nav")) {
          request.drivable_areas.push_back(area.outline);
        }
      }
      for (const auto& area : map_) {
        if (area.active && area.type == "obstacle") request.obstacles.push_back(area.outline);
      }
    }

    if (params.contains("outline")) request.boundary = Opened(RequirePoints(params["outline"], "outline"));
    if (params.contains("obstacles")) {
      if (!params["obstacles"].is_array()) Invalid("parameter is not a list of polygons: obstacles");
      request.obstacles.clear();
      for (const auto& o : params["obstacles"]) request.obstacles.push_back(Opened(RequirePoints(o, "obstacles")));
    }
    if (params.contains("angle")) angle = RequireNumber(params, "angle");
    if (params.contains("outline_count")) outline_count = RequireCount(params, "outline_count");
    if (params.contains("outline_overlap_count")) outline_overlap_count = RequireCount(params, "outline_overlap_count");
    if (params.contains("outline_offset")) outline_offset = RequireNumber(params, "outline_offset");
    if (request.boundary.size() < 3) Invalid("the area needs at least 3 outline points (area_id or outline)");

    if (outline_count) request.perimeter_passes = *outline_count;
    request.lane_overlap_passes = outline_overlap_count;
    request.perimeter_offset = outline_offset;
    if (angle) {
      request.use_angle = true;
      request.angle = *angle;
    }
    question.ok = true;
  } catch (const PlanError& e) {
    question.ok = false;
    question.code = e.code;
    question.message = e.message;
  }
  return question;
}

json MowingPlanner::Result(const PlanQuestion& question, const CoverageAnswer& answer) const {
  json paths = json::array();
  for (const auto& piece : CutIntoPieces(answer)) {
    if (piece.last >= answer.poses.size()) continue;
    const std::vector<Point2> poses(answer.poses.begin() + static_cast<long>(piece.first),
                                    answer.poses.begin() + static_cast<long>(piece.last) + 1);
    json points = json::array();
    json pose_index = json::array();
    for (const size_t i : SimplifiedIndices(poses, kSimplifyTolerance)) {
      points.push_back({Millimetres(poses[i].x), Millimetres(poses[i].y)});
      pose_index.push_back(i);
    }
    paths.push_back({{"is_outline", piece.is_outline}, {"points", std::move(points)}, {"pose_index", std::move(pose_index)}});
  }
  return json{{"area_id", question.area_id},
              {"angle", answer.angle},
              {"paths", std::move(paths)},
              {"fingerprint", answer.fingerprint}};
}

}  // namespace mowbite_logic
