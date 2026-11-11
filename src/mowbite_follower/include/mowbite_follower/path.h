#ifndef MOWBITE_FOLLOWER_PATH_H
#define MOWBITE_FOLLOWER_PATH_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mowbite_follower {

// base_link in the map frame, yaw the way the mower faces.
struct Pose2 {
  double x = 0;
  double y = 0;
  double yaw = 0;
};

// rad in (-pi, pi].
double NormalizeAngle(double angle);

// The segment types of the planner, numbered as in FollowPath.action.
enum class Kind : uint8_t { kPerimeter = 0, kLane = 1, kTurn = 2, kTransit = 3 };

// A point seen from the path: the foot point on one segment and where the path goes there.
struct Projection {
  // The segment from pose `segment` to the next.
  size_t segment = 0;
  // m along the path to the foot point, within the segment.
  double s = 0;
  // m along the path; on the first and last segment of the searched range also before or past its ends.
  double along = 0;
  // m beside the path, left of the direction of travel positive.
  double cross = 0;
  // m to the foot point: like cross on a segment, before or past an open end the way to that end.
  double distance = 0;
  // rad, the smoothed direction of travel at the foot point.
  double heading = 0;
  // 1/m, left turns positive.
  double curvature = 0;
};

// The poses of a path as a line of segments: lengths, directions of travel, corners and a smoothed heading. The
// direction of travel is the way from one pose to the next, so a path driven backwards has the same geometry. A pose
// within 1 mm of the one before adds no segment: it is merged into that one, the last pose into the one before it.
class Path {
 public:
  // Empty if the poses make a path, else why not. A corner is a pose where the direction of travel turns by more
  // than corner_angle (rad), or bends tighter than min_radius (m); there the mower stops and turns on the spot.
  std::string Build(const std::vector<Pose2>& poses, const std::vector<Kind>& kinds, double corner_angle,
                    double min_radius = 0.0);

  // The index of pose i among the poses given to Build, and the pose an index given to Build is merged into.
  size_t source(size_t i) const { return source_[i]; }
  size_t Merged(size_t given) const;

  size_t size() const { return poses_.size(); }
  const Pose2& pose(size_t i) const { return poses_[i]; }
  Kind kind(size_t i) const { return kinds_[i]; }
  // m along the path to pose i.
  double s(size_t i) const { return s_[i]; }
  double length(size_t segment) const { return length_[segment]; }
  double direction(size_t segment) const { return direction_[segment]; }
  double curvature(size_t segment) const { return curvature_[segment]; }
  bool corner(size_t i) const { return corner_[i]; }
  // The first corner after pose i, else the last pose.
  size_t NextStop(size_t i) const;
  // The direction of travel stays within angle (rad) of the one of segment i for the next length m or to the end.
  bool StraightAhead(size_t segment, double length, double angle) const;

  // The foot point of (x, y) on the segments first..last, the nearest one, the later one on a tie.
  Projection Project(double x, double y, size_t first, size_t last) const;

 private:
  std::vector<Pose2> poses_;
  std::vector<Kind> kinds_;
  std::vector<size_t> source_;
  std::vector<double> s_;
  std::vector<double> length_;
  std::vector<double> direction_;
  // The smoothed heading at pose i at the end of the segment before and at the start of the one after; they differ
  // only on corners.
  std::vector<double> heading_in_;
  std::vector<double> heading_out_;
  std::vector<double> curvature_;
  std::vector<bool> corner_;
};

}  // namespace mowbite_follower

#endif  // MOWBITE_FOLLOWER_PATH_H
