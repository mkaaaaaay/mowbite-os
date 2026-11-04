#ifndef MOWBITE_CALIBRATION_RECORDING_H
#define MOWBITE_CALIBRATION_RECORDING_H

#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace mowbite_calibration {

using json = nlohmann::ordered_json;

// What the recordings of calibration drives may take in their folder: the newest max_runs, together at most max_bytes,
// none older than max_days. One recording ends at max_bytes too.
struct RecordingLimits {
  size_t max_runs = 5;
  uint64_t max_bytes = 20ull * 1000 * 1000;
  double max_days = 60.0;
};

// A recording in the format mowbite-calib/1 for the evaluation of the planner: gzip, JSON lines, the head first, the
// end last. A line goes out only with finite numbers and at most kMaxLine bytes; data lines are flat. The file is
// written as <name>.part and gets its name at the end, so a recording that has a name is whole.
class Recording {
 public:
  static constexpr size_t kMaxLine = 4096;

  Recording() = default;
  ~Recording();
  Recording(const Recording&) = delete;
  Recording& operator=(const Recording&) = delete;

  // Starts calib-<UTC time>.jsonl.gz in folder with head as its first line (format and source are set here); false
  // with the reason in why. A running recording is dropped first.
  bool Open(const std::string& folder, json head, double wall_t, const RecordingLimits& limits, std::string* why);
  bool open() const { return file_ != nullptr; }

  // A data line {"t", "k", fields...}; fields hold numbers, text, true/false or null. False if it was not written: no
  // recording, a number that is not finite, a nested value, or too long. Reaching max_bytes ends it with why "size".
  bool Line(double wall_t, const char* kind, const json& fields);
  // The end line and the name: why is stop, enough, time, size or error, reason the code of the program (may be
  // empty). Returns the path of the finished recording, empty without one or if it could not be finished.
  std::string End(double wall_t, const std::string& why, const std::string& reason);
  // The name of the recording opened last, and the path of the last finished one, also after it ended by size.
  const std::string& path() const { return path_; }
  const std::string& finished() const { return finished_; }
  // Lines left out since Open.
  size_t dropped() const { return dropped_; }

 private:
  void Close();

  void* file_ = nullptr;
  std::string path_;
  std::string finished_;
  uint64_t max_bytes_ = 0;
  size_t dropped_ = 0;
};

// Removes what the limits do not keep in folder: parts of recordings that never ended, recordings older than
// max_days, then the oldest ones beyond max_runs or max_bytes. keep (a path) stays in any case. Returns the removed
// names.
std::vector<std::string> Prune(const std::string& folder, double wall_t, const RecordingLimits& limits,
                               const std::string& keep = "");

// Levels of nesting of a JSON value: 0 for a number or text, 1 for a flat object or list.
int Depth(const json& value);

}  // namespace mowbite_calibration

#endif  // MOWBITE_CALIBRATION_RECORDING_H
