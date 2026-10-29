#ifndef MOWBITE_LOGIC_MOWING_JOB_H
#define MOWBITE_LOGIC_MOWING_JOB_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_logic/mowing_plan.h"

namespace mowbite_logic {

// A pose of a path to mow; kind is the segment type of the stretch from this pose to the next one.
struct JobPose {
  double x = 0;
  double y = 0;
  double heading = 0;
  uint8_t kind = kSegmentTransit;
};

// One path of the plan of an area, numbered like the paths of mowing.plan.
struct JobPath {
  bool is_outline = false;
  std::vector<JobPose> poses;
};

// The perimeter, lanes and the turns between them are mowed; the blade is off on a transit.
bool IsMowKind(uint8_t kind);

// The route cut into the paths of mowing.plan, with the heading and segment type of every pose. Without headings in
// the answer each pose faces the next one.
std::vector<JobPath> JobPaths(const CoverageAnswer& answer);

// Where a job stands, saved as the checkpoint in data_dir: a restart goes on from here.
struct JobProgress {
  std::string job_id;
  int area = 0;
  // The id of that area when it was saved; another one there means the map changed.
  std::string area_id;
  int path = 0;
  int index = 0;
  // Of the plan the path and index belong to.
  std::string fingerprint;
};

std::string CheckpointText(const JobProgress& progress);
// Nothing for a text that is not a checkpoint.
std::optional<JobProgress> ParseCheckpoint(const std::string& text);

// The progress of a job through the areas and their paths, without ROS. LogicCore drives it.
class MowingJob {
 public:
  void SetAreas(std::vector<JobArea> areas);
  const std::vector<JobArea>& areas() const {
    return areas_;
  }
  bool HasMowableArea() const;
  void Restore(const JobProgress& progress);
  const JobProgress& progress() const {
    return progress_;
  }
  void SetJobId(const std::string& id) {
    progress_.job_id = id;
  }
  // Something was mowed or planned and not finished.
  bool Unfinished() const;

  // Moves on to the first mowable area from the current one. If the saved area has another id now, the job starts
  // over from the first area (notice says so). False when no area is left.
  bool SeekArea(std::string* notice);
  const JobArea* CurrentArea() const;
  bool HasPlan() const {
    return has_plan_;
  }
  // The plan of the current area. Path and index stay when the fingerprint is the saved one, else the area starts
  // over. A plan without paths finishes the area.
  void SetPlan(std::vector<JobPath> paths, const std::string& fingerprint);
  // Nothing without a plan or after its last path.
  const JobPath* CurrentPath() const;
  const JobPose* CurrentPose() const;
  // The mower got to this pose of the current path.
  void OnIndex(size_t index);
  void PathDone();
  void SkipPath();
  void SkipArea();
  // All areas mowed: back to the first.
  void Complete();
  // Drops the progress and the job id.
  void Reset();

 private:
  void NextArea();

  std::vector<JobArea> areas_;
  JobProgress progress_;
  std::vector<JobPath> paths_;
  bool has_plan_ = false;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_MOWING_JOB_H
