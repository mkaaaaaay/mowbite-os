#ifndef MOWBITE_LOGIC_MOWING_PLAN_H
#define MOWBITE_LOGIC_MOWING_PLAN_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_logic/area_recorder.h"

namespace mowbite_logic {

// JSON-RPC codes of the errors of mowing.plan
constexpr int kPlanInvalidParams = -32602;
constexpr int kPlanRefused = -32000;
constexpr int kPlanInternal = -32603;

// Segment types of the planner's route (PlanCoverage.srv)
constexpr uint8_t kSegmentPerimeter = 0;
constexpr uint8_t kSegmentLane = 1;
constexpr uint8_t kSegmentTurn = 2;
constexpr uint8_t kSegmentTransit = 3;

// A path of the answer has at most this many poses, like the planner's own export: OpenMower counts them in an int16.
constexpr size_t kMaxPathPoses = 30000;

// The parameters of /mower_logic that shape a plan, with the names and the meaning of OpenMower.
struct PlanSettings {
  // m, distance of neighbouring lanes
  double tool_width = 0.187;
  // m, cutting width; the planner takes the lane distance as an overlap of it
  double mower_width = 0.22;
  int outline_count = 3;
  int outline_overlap_count = 0;
  // m; nothing: the perimeter_offset of the planner (half the cutting width). Negative: the same.
  std::optional<double> outline_offset;
  // deg
  double mow_angle_offset = 0;
  bool mow_angle_offset_is_absolute = false;
  // deg, the mow_angle_increment of the mows finished in the job so far (no job yet: 0)
  double angle_increment_sum = 0;
};

// What ~/plan_coverage of the planner is asked, without ROS types.
struct CoverageRequest {
  // Open rings in the map frame
  Ring boundary;
  std::vector<Ring> obstacles;
  std::vector<Ring> drivable_areas;
  double mower_width = 0;
  double overlap = 0;
  int perimeter_passes = 0;
  bool use_angle = false;
  // rad
  double angle = 0;
  // Parameters of the planner node, set for this request only
  int lane_overlap_passes = 0;
  std::optional<double> perimeter_offset;
  std::optional<double> angle_offset_deg;
};

// The route the planner answered ~/plan_coverage with.
struct CoverageAnswer {
  bool success = false;
  // Why it failed, or a summary
  std::string message;
  std::vector<Point2> poses;
  std::vector<uint32_t> section_start;
  std::vector<uint32_t> segment_start;
  std::vector<uint32_t> segment_end;
  std::vector<uint8_t> segment_type;
  // rad, the lane direction
  double angle = 0;
  std::string fingerprint;
};

// A request of mowing.plan checked and turned into a question for the planner.
struct PlanQuestion {
  bool ok = false;
  int code = 0;
  std::string message;
  // Empty for an outline given in the request
  std::string area_id;
  CoverageRequest request;
};

// A piece of the route as the app draws it: a run of poses, indices into the poses of the answer.
struct Piece {
  bool is_outline = false;
  size_t first = 0;
  size_t last = 0;
};

// The route cut into the paths of OpenMower: a new one for every section, where the route changes between the perimeter
// loops and the rest (a transit goes with what follows it) and when one gets longer than max_poses. Runs of fewer than 3
// poses are left out, FTC needs 3.
std::vector<Piece> CutIntoPieces(const CoverageAnswer& answer, size_t max_poses = kMaxPathPoses);

// Indices of the points Douglas-Peucker keeps within tolerance (m), the first and the last always.
std::vector<size_t> SimplifiedIndices(const std::vector<Point2>& points, double tolerance);

// mowing.plan: plans an area the way mowing would, without starting anything. Not thread safe.
class MowingPlanner {
 public:
  explicit MowingPlanner(const PlanSettings& settings) : settings_(settings) {}

  // The text of map/json.
  void SetMap(const std::string& text);
  // map/status: planning from the map needs a map that fits the datum in use.
  void SetMapStatus(bool fits, const std::string& reason);
  PlanSettings& settings() {
    return settings_;
  }

  // params of the RPC: an object with area_id, or outline and obstacles ({x, y} points), and angle, outline_count,
  // outline_overlap_count, outline_offset to override the settings of the area.
  PlanQuestion Prepare(const json& params) const;
  // The result of the RPC: {area_id, angle, paths: [{is_outline, points, pose_index}], fingerprint}; points in m,
  // simplified to 1 cm and rounded to mm, pose_index the index of each point in its path.
  json Result(const PlanQuestion& question, const CoverageAnswer& answer) const;

 private:
  struct MapArea {
    std::string id;
    std::string type;
    bool active = true;
    bool mowable = true;
    Ring outline;
    std::optional<double> angle;
    std::optional<int> outline_count;
    std::optional<int> outline_overlap_count;
    std::optional<double> outline_offset;
  };

  PlanSettings settings_;
  std::vector<MapArea> map_;
  bool map_readable_ = false;
  bool map_fits_ = false;
  std::string map_reason_ = "The map status is not known yet";
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_MOWING_PLAN_H
