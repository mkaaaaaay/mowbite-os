#ifndef MOWBITE_LOCALIZATION_COURSE_ALIGNMENT_H
#define MOWBITE_LOCALIZATION_COURSE_ALIGNMENT_H

#include <Eigen/Core>
#include <cmath>
#include <deque>
#include <optional>

namespace mowbite_localization {

struct CourseConfig {
  // The antenna and the wheels both at least this fast (m/s), forward or back.
  double min_speed = 0.1;
  // The antenna as fast as the wheels say within speed_tolerance m/s plus speed_ratio of their speed.
  double speed_tolerance = 0.05;
  double speed_ratio = 0.3;
  // run headings in a row that agree within agreement rad make a heading.
  int run = 5;
  double agreement = 10.0 * M_PI / 180.0;
  // Noise of each axis of the GNSS velocity (m/s): the variance of a heading from it.
  double velocity_noise = 0.02;
};

// The heading of base_link that a velocity of the antenna (m/s, map frame) says, for the wheel speed (m/s, negative
// backwards) and the turn rate (rad/s), with the antenna at lever (m, frame of base_link). Nothing when either moves
// too slowly or the antenna not as fast as the wheels say.
std::optional<double> HeadingFromCourse(const Eigen::Vector2d& velocity, double speed, double turn_rate,
                                        const Eigen::Vector2d& lever, const CourseConfig& config);
// Variance (rad²) of a heading from a velocity of that speed (m/s).
double CourseVariance(double speed, const CourseConfig& config);

// Headings from the course minus the estimated heading of the same moment, in a row: once the last `run` of them agree
// they give the correction of the estimate, whatever the estimate was.
class CourseAlignment {
 public:
  explicit CourseAlignment(const CourseConfig& config);

  // Adds a difference (rad); the mean of the run once it is full and agrees.
  std::optional<double> Add(double difference);
  // A fix without a usable course breaks the run.
  void Clear() { differences_.clear(); }

 private:
  CourseConfig config_;
  std::deque<double> differences_;
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_COURSE_ALIGNMENT_H
