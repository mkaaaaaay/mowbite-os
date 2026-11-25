#ifndef MOWBITE_LOGIC_CALIBRATION_PART_H
#define MOWBITE_LOGIC_CALIBRATION_PART_H

#include <optional>
#include <string>
#include <vector>

#include "mowbite_logic/core_access.h"

namespace mowbite_logic {

// CALIBRATING of the logic: the status of the calibration program and its signs. The calls are those of LogicCore
// with the same names.
class CalibrationPart {
 public:
  void OnCalibrationStatus(const CalibrationStatus& status, double now_s);
  std::vector<Reason> CalibrationProblems(CoreFacts core, double now_s) const;
  std::optional<CalibrationStatus> CalibrationStatusNow(CoreFacts core, double now_s) const;
  Reason StartCalibration(CoreAccess core, double now_s);
  void RefuseCalibration(CoreAccess core, const Reason& why);
  void OnCalibrationSign(CoreFacts core, double now_s);
  void OnCalibrationFeedback(CoreAccess core, const std::string& phase, int index, double now_s);
  void OnCalibrationEnded(CoreAccess core, const CalibrationOutcome& outcome);
  bool AbortCalibration(CoreAccess core, const Reason& why);
  bool TickCalibration(CoreAccess core, double now_s);
  // Ends a calibration drive here: CALIBRATION_FAILED unless done, CALIBRATION end, IDLE; cancel: the program too.
  void EndCalibration(CoreAccess core, const std::optional<Reason>& failure, bool cancel);

 private:
  // The program's status and when it came, its last sign and the phase it reported last.
  std::optional<CalibrationStatus> calibration_status_;
  double calibration_status_s_ = 0;
  double calibration_sign_s_ = 0;
  std::string calibration_phase_;
  int calibration_index_ = 0;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_CALIBRATION_PART_H
