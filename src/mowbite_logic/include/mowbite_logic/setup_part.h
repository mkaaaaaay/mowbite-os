#ifndef MOWBITE_LOGIC_SETUP_PART_H
#define MOWBITE_LOGIC_SETUP_PART_H

#include <optional>
#include <string>

#include "mowbite_logic/core_access.h"

namespace mowbite_logic {

// SETUP of the logic: the step of the setup program, its last setup/motion and the blade the step asks for. The calls
// are those of LogicCore with the same names.
class SetupPart {
 public:
  Reason OnSetupMotion(CoreAccess core, bool active, const std::string& step, double now_s);
  bool TickSetup(CoreAccess core, double now_s);
  void OnSetupBlade(CoreFacts core, const std::string& step, int blade, double input_hold_s);
  bool SetupBladeOnce(CoreFacts core) const;
  // Leaves SETUP; why goes out as a notice unless the program ended it.
  void EndSetup(CoreAccess core, const std::optional<std::string>& why);
  // hw/blade refused the blade of the step: off until the step asks for another blade.
  void BladeRefused(CoreAccess core, const std::string& message);
  // An emergency (also no telemetry) or the station stopped the blade of the step: off until the step asks for
  // another.
  void BladeStopped() {
    if (setup_blade_ != 0) setup_blade_refused_ = true;
  }
  int setup_blade() const {
    return setup_blade_;
  }
  bool setup_blade_refused() const {
    return setup_blade_refused_;
  }
  double setup_blade_hold_s() const {
    return setup_blade_hold_s_;
  }

 private:
  std::string setup_step_;
  double setup_sign_s_ = 0;
  // SetupMotion BLADE_* the step asks for and whether hw/blade refused it.
  int setup_blade_ = 0;
  bool setup_blade_refused_ = false;
  // How long the blade order stays after an emergency of only inputs of the board (LogicCore::SetupBladeHeld).
  double setup_blade_hold_s_ = 0;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_SETUP_PART_H
