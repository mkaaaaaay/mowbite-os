#ifndef TRON_VISION_BUDGET_H
#define TRON_VISION_BUDGET_H

#include <cstddef>
#include <map>
#include <string>

namespace tron_vision {

// Why a camera process ends on its own.
enum ExitCode : int {
  kExitNone = 0,
  kExitMemory = 3,
  kExitSlowNpu = 4,
};

// Resident memory of the process from statm, 0 if unreadable.
size_t ResidentBytes(const std::string& statm = "/proc/self/statm");

// temp file of the thermal zone of that type ("soc-thermal"), empty if none.
std::string FindThermalZone(const std::string& root, const std::string& type);

// Degrees Celsius from a thermal zone temp file (millidegrees); false if unreadable.
bool ReadCelsius(const std::string& temp_file, double& celsius);

// Pause from `pause_at` on, resume below `resume_below`.
class ThermalGate {
 public:
  ThermalGate(double pause_at, double resume_below) : pause_at_(pause_at), resume_below_(resume_below) {}
  bool Paused(double celsius);

 private:
  double pause_at_;
  double resume_below_;
  bool paused_ = false;
};

// One message per key and interval; the count of suppressed ones comes with the next allowed.
class LogGate {
 public:
  explicit LogGate(double interval_s) : interval_s_(interval_s) {}
  // True if a message with this key may go out at `now_s`; `suppressed` is then the number held back since the last.
  bool Allow(const std::string& key, double now_s, int& suppressed);

 private:
  struct Entry {
    double last = 0.0;
    int suppressed = 0;
  };
  double interval_s_;
  std::map<std::string, Entry> entries_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_BUDGET_H
