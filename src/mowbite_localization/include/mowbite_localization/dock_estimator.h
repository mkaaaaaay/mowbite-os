#ifndef MOWBITE_LOCALIZATION_DOCK_ESTIMATOR_H
#define MOWBITE_LOCALIZATION_DOCK_ESTIMATOR_H

#include <array>
#include <cmath>
#include <deque>
#include <optional>

#include "mowbite_localization/estimator.h"
#include "mowbite_localization/pose_filter.h"
#include "mowbite_localization/state_history.h"

namespace mowbite_localization {

struct DockConfig {
  FilterNoise noise;
  // The marker in base_link of the docked mower: m, and the direction it faces, rad.
  double marker_x = 0.0;
  double marker_y = 0.0;
  double marker_yaw = 0.0;
  // An image older than max_age s when it arrives is dropped.
  double max_age = 0.3;
  // The start, at rest, takes the last start_images images in a row that agree with each other: the heading from their
  // directions if all have one, else from a hint of the logic at most hint_hold s old with hint_sigma rad, else there
  // is no estimate.
  int start_images = 10;
  double hint_hold = 3.0;
  double hint_sigma = 5.0 * M_PI / 180.0;
  // An image is an outlier when the chi-square of its position (two axes) or of its direction (one axis) lies above
  // position_chi2 or yaw_chi2 (99.9 %) and the miss above min_outlier m or min_outlier_yaw rad; outlier_reset of them
  // in a row drop the estimate.
  double position_chi2 = 13.8;
  double yaw_chi2 = 10.8;
  double min_outlier = 0.02;
  double min_outlier_yaw = 2.0 * M_PI / 180.0;
  int outlier_reset = 3;
  // Good while at most good_way m driven and good_time s passed since the image of the last fix. Within
  // blind_distance m before the docked pose, after a fix taken at most good_way m before that, final_way m and
  // final_time s instead: there the marker leaves the image.
  double good_way = 0.05;
  double good_time = 1.0;
  double blind_distance = 0.07;
  double final_way = 0.14;
  double final_time = 12.0;
};

// One image of the marker: when it was taken (s, the clock of the motions), its centre in base_link (m), the direction
// it faces (rad, only with yaw_valid) and the covariance of x, y and yaw, row by row.
struct MarkerImage {
  double time = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool yaw_valid = false;
  std::array<double, 9> covariance{};
};

enum class MarkerResult {
  kNoFrame,     // the heading of the docked mower is not known yet
  kInvalid,     // not a number, or a variance that is not above 0
  kTooOld,      // older than max_age, or taken after it arrived
  kCollecting,  // no estimate: kept for the start while the mower stands, else dropped
  kStarted,     // the images of the start set the estimate
  kFused,       // corrected the estimate
  kOutlier,     // dropped
  kDropped,     // one outlier too many: the estimate is gone
};

// base_link in the station frame from the dock marker, without ROS: origin at base_link of the docked mower, x the
// way in. The motions of the map estimate move it, images of the marker correct it; nothing goes from one estimate
// to the other. Times are seconds of the clock of the motions.
class DockEstimator {
 public:
  explicit DockEstimator(const DockConfig& config);

  void AddMotion(const Motion& motion);
  MarkerResult AddMarker(const MarkerImage& image, double t);
  // The heading of the logic in the station frame (rad) at t, for a start without the direction of the marker.
  void SetHint(double heading, double t);
  // The heading of the docked mower in the station frame: 0 with the contacts in front, pi with them behind. A new one
  // drops the estimate.
  void SetDockedHeading(double heading);

  bool has_estimate() const { return estimate_; }
  // The start took its heading from the hint.
  bool heading_from_hint() const { return from_hint_; }
  Pose2 pose() const;
  const Covariance& covariance() const { return filter_.covariance(); }
  bool Good(double t) const;
  // Standard deviation (m) of the position along its worst axis while good, else kNoAccuracy.
  double PositionAccuracy(double t) const;
  // Standard deviation (rad) of the heading while there is an estimate, else kNoAccuracy.
  double HeadingAccuracy() const;

 private:
  struct Moment {
    double time;
    State state;
    double way;
  };

  // Where the marker is in the station frame.
  State Marker() const;
  bool TryStart(double t);
  bool Outlier(const Eigen::Vector2d& innovation, const AntennaJacobian& jacobian, const Eigen::Matrix2d& noise,
               std::optional<double> yaw_innovation, double yaw_variance) const;
  void Drop();

  DockConfig config_;
  PoseFilter filter_;
  std::optional<double> docked_heading_;
  bool estimate_ = false;
  bool from_hint_ = false;
  // The gyro integrated since the start and the way driven either way, never corrected.
  double gyro_heading_ = 0.0;
  double way_ = 0.0;
  std::optional<double> still_since_;
  std::deque<MarkerImage> start_;
  std::optional<double> hint_offset_;
  double hint_time_ = 0.0;
  int outliers_ = 0;
  // The image of the last fix: when it was taken, the way and x in the station frame then.
  double fix_time_ = 0.0;
  double fix_way_ = 0.0;
  double fix_x_ = 0.0;
  StateHistory<Moment> history_;
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_DOCK_ESTIMATOR_H
