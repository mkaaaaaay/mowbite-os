#ifndef TRON_MCU_STOP_CLEAR_MARK_H
#define TRON_MCU_STOP_CLEAR_MARK_H

#include <string>

namespace tron {

// A file that stands for STOP cleared on the MCU during an open emergency (G37). The MCU then no longer locks itself,
// and after a restart the driver would not know of the emergency: it is written and synced before the first such clear
// frame and removed once the emergency is released. A driver that starts with it, or cannot tell, latches.
class StopClearMark {
 public:
  enum class State { kAbsent, kPresent, kUnknown };

  explicit StopClearMark(std::string path);

  const std::string& path() const {
    return path_;
  }
  State Check(std::string& error) const;
  // The file and its directory synced; false on any error.
  bool Write(std::string& error) const;
  // Gone already counts as removed.
  bool Remove(std::string& error) const;

 private:
  std::string path_;
};

}  // namespace tron

#endif  // TRON_MCU_STOP_CLEAR_MARK_H
