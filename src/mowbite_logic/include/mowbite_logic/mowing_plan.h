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

// What ~/plan_coverage of the planner is asked, without ROS types. What the request leaves at its default (-1, false,
// nothing) the planner takes from its settings and from the properties of the area in the map.
struct CoverageRequest {
  // Open rings in the map frame
  Ring boundary;
  std::vector<Ring> obstacles;
  std::vector<Ring> drivable_areas;
  int perimeter_passes = -1;
  bool use_angle = false;
  // rad
  double angle = 0;
  // Parameters of the planner node, set for this request only
  std::optional<int> lane_overlap_passes;
  std::optional<double> perimeter_offset;
  // deg on the lane direction of the planner, from the plan of the schedule that runs
  std::optional<double> angle_offset_deg;
};

// The route the planner answered ~/plan_coverage with.
struct CoverageAnswer {
  bool success = false;
  // Why it failed, or a summary
  std::string message;
  // Empty with success, else the code of PlanCoverage
  std::string code;
  std::vector<Point2> poses;
  // rad, one per pose; may be empty
  std::vector<double> headings;
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

// A mowing area of the map. Its index counts the mow areas in map order, inactive ones included, like current_area of
// OpenMower and the app.
struct JobArea {
  std::string id;
  std::string name;
  // Active and mowable: the job mows it, else it is passed over.
  bool mowable = true;
  // Of its outline and the active obstacles of the map: another value means its plan no longer fits.
  uint64_t shape = 0;
};

// A piece of the route as the app draws it: a run of poses, indices into the poses of the answer.
struct Piece {
  bool is_outline = false;
  size_t first = 0;
  size_t last = 0;
};

// The route cut into the paths of OpenMower: a new one for every section, where the route changes between the perimeter
// loops and the rest (a transit goes with what follows it) and when one gets longer than max_poses. Runs of fewer than 3
// poses are left out, the follower needs 3.
std::vector<Piece> CutIntoPieces(const CoverageAnswer& answer, size_t max_poses = kMaxPathPoses);

// Indices of the points Douglas-Peucker keeps within tolerance (m), the first and the last always.
std::vector<size_t> SimplifiedIndices(const std::vector<Point2>& points, double tolerance);

// mowing.plan: plans an area the way mowing would, without starting anything. Not thread safe.
class MowingPlanner {
 public:

  // The text of map/json.
  void SetMap(const std::string& text);
  // map/status: planning from the map needs a map that fits the datum in use.
  void SetMapStatus(bool fits, const Reason& reason);

  // params of the RPC: an object with area_id, or outline and obstacles ({x, y} points), and angle, outline_count,
  // outline_overlap_count, outline_offset to override the planner's values for this plan.
  PlanQuestion Prepare(const json& params) const;
  // The result of the RPC: {area_id, angle, paths: [{is_outline, points, pose_index}], fingerprint}; points in m,
  // simplified to 1 cm and rounded to mm, pose_index the index of each point in its path.
  json Result(const PlanQuestion& question, const CoverageAnswer& answer) const;
  // Where a drive between areas may go: the active mow and nav areas (also those not to be mowed) and the active
  // obstacles. Empty if the map allows it, else why not (MAP_*, NO_MOWING_AREA). With only_area: that mow area alone
  // is drivable.
  Reason TransitAreas(std::vector<Ring>* drivable, std::vector<Ring>* obstacles,
                      const std::string& only_area = "") const;
  // The mow areas of the map in map order, inactive ones included; nothing while the map is not readable.
  std::vector<JobArea> MowAreas() const;
  // Whether mowing may plan from the map: readable and fitting the datum in use; else why not (MAP_*).
  Reason MapProblem() const;
  // The first active docking station of the map, none while the map is not readable.
  std::optional<DockPose> Dock() const;

 private:
  struct MapArea {
    std::string id;
    std::string name;
    std::string type;
    bool active = true;
    bool mowable = true;
    Ring outline;
  };

  std::vector<MapArea> map_;
  std::optional<DockPose> dock_;
  bool map_readable_ = false;
  bool map_fits_ = false;
  Reason map_reason_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_MOWING_PLAN_H
