#ifndef MOWBITE_LOGIC_RECORDING_PART_H
#define MOWBITE_LOGIC_RECORDING_PART_H

#include <limits>
#include <optional>
#include <string>
#include <utility>

#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/core_access.h"

namespace mowbite_logic {

// AREA_RECORDING of the logic: the recorder, the wheels it follows, whether the map takes what it records, and
// teleop. The calls are those of LogicCore with the same names.
class RecordingPart {
 public:
  explicit RecordingPart(const RecorderConfig& config) : recorder_(config) {}

  void OnMapStatus(CoreAccess core, MapDatum datum, const std::string& reason);
  void OnWheelOdom(CoreAccess core, double vx, double now_s);
  // A break of the driven track while OnWheelOdom does not come.
  void TickRecording(CoreAccess core, double now_s);
  void OnMapEditResult(CoreAccess core, bool ok, const std::string& id, const std::string& code,
                       const std::string& message);
  // Moves the events of the recorder into the events of the logic, at the current position.
  void PullRecorder(CoreAccess core);
  json MapOverlay() const;
  bool TakeOverlayChanged();
  void RecordedBodyKept(CoreAccess core, int areas, const std::string& code, const std::string& message);
  std::optional<std::pair<double, double>> OnTeleop(CoreFacts core, double vx, double vz, double now_s);
  // Teleop waits for a zero or a quiet joystick again.
  void DisarmTeleop();
  AreaRecorder& recorder() {
    return recorder_;
  }
  const AreaRecorder& recorder() const {
    return recorder_;
  }
  bool map_writable() const {
    return map_writable_;
  }
  const Reason& map_reason() const {
    return map_reason_;
  }

 private:
  AreaRecorder recorder_;
  // The last OnWheelOdom.
  std::optional<double> last_wheel_s_;
  bool map_writable_ = false;
  Reason map_reason_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");
  bool teleop_armed_ = false;
  double last_teleop_s_ = -std::numeric_limits<double>::infinity();
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_RECORDING_PART_H
