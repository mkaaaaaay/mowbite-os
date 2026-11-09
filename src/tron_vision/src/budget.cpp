#include "tron_vision/budget.h"

#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "tron_vision/text_file.h"

namespace tron_vision {

size_t ResidentBytes(const std::string& statm) {
  std::ifstream in(statm);
  size_t pages = 0;
  size_t resident = 0;
  if (!(in >> pages >> resident)) return 0;
  const long page = sysconf(_SC_PAGESIZE);
  return resident * static_cast<size_t>(page > 0 ? page : 4096);
}

std::string FindThermalZone(const std::string& root, const std::string& type) {
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(root, ec)) {
    const std::string zone = entry.path().filename().string();
    if (zone.rfind("thermal_zone", 0) == 0 && FirstLine(root + "/" + zone + "/type") == type) {
      return root + "/" + zone + "/temp";
    }
  }
  return "";
}

bool ReadCelsius(const std::string& temp_file, double& celsius) {
  std::ifstream in(temp_file);
  long milli = 0;
  if (!(in >> milli)) return false;
  celsius = static_cast<double>(milli) / 1000.0;
  return true;
}

bool ThermalGate::Paused(double celsius) {
  if (paused_) {
    paused_ = celsius >= resume_below_;
  } else {
    paused_ = celsius >= pause_at_;
  }
  return paused_;
}

bool LogGate::Allow(const std::string& key, double now_s, int& suppressed) {
  auto it = entries_.find(key);
  if (it == entries_.end()) {
    entries_[key].last = now_s;
    suppressed = 0;
    return true;
  }
  Entry& e = it->second;
  if (now_s - e.last < interval_s_) {
    e.suppressed++;
    return false;
  }
  suppressed = e.suppressed;
  e.suppressed = 0;
  e.last = now_s;
  return true;
}

}  // namespace tron_vision
