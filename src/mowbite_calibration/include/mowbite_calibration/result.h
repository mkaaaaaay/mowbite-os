#ifndef MOWBITE_CALIBRATION_RESULT_H
#define MOWBITE_CALIBRATION_RESULT_H

#include <string>

#include "mowbite_calibration/recording.h"

namespace mowbite_calibration {

// The key of robot.geometry a value of the evaluation goes to (ticks_per_m, wheel_distance_m -> track,
// antenna_offset_x|y -> antenna_x|y, imu_roll, imu_pitch), empty for one that has none.
std::string GeometryKey(const std::string& name);

// How a calibration drive ended, for calibration.result.
struct Ending {
  // A name of its own for the drive, and when it ended (Unix s).
  std::string id;
  double time = 0;
  // DONE or why the program stopped, with its message.
  std::string reason;
  std::string message;
  // The recording, empty without one.
  std::string recording;
  // version of robot.geometry the drive ran with, 0 before the dimensions were known.
  int geometry_version = 0;
};

// calibration.result: the evaluation of the planner (format mowbite-calib/1: ready, version, based_on, warnings,
// values, info) with the key of robot.geometry for every value, and id, time, status, reason, message, recording,
// geometry_version and summary. status: done with an evaluation, not_evaluated without one (no planner, it did not
// answer), failed for a drive that did not run to its end or an evaluation that rejected the data (error, or not
// ready). station_again: an antenna offset that is ok differs from the old one by more than station_limit (m), and the
// docking station recorded with the old one has to be recorded again once it is taken.
json Compose(const Ending& ending, const json& evaluation, double station_limit = 0.02);

// "4 of 6 values ok" for CALIBRATED, or why there are none.
std::string Summary(const json& result);

// Writes text to path whole or not at all (next to it, synced, renamed, the folder synced); false with why.
bool WriteWhole(const std::string& path, const std::string& text, std::string* why);

}  // namespace mowbite_calibration

#endif  // MOWBITE_CALIBRATION_RESULT_H
