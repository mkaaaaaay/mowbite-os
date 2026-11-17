#ifndef MOWBITE_LOGIC_CORE_ACCESS_H
#define MOWBITE_LOGIC_CORE_ACCESS_H

#include <optional>
#include <string>

#include "mowbite_logic/logic_types.h"

namespace mowbite_logic {

class LogicCore;

// What a part of the logic reads of the core it belongs to. The core hands it in with each call and the parts keep no
// pointer to the core, so a copy of the core stays whole.
class CoreFacts {
 public:
  explicit CoreFacts(const LogicCore& core) : read_(&core) {}

  const LogicConfig& config() const;
  State state() const;
  bool emergency() const;
  const HardwareSnapshot& hw() const;
  const std::optional<PoseSnapshot>& pose() const;
  const Reason& drivable_problem() const;
  bool DockContact() const;
  Reason Busy() const;
  Reason HoldProblem() const;
  Reason Unsafe() const;
  Reason AccuracyReason(float accuracy, double limit) const;

 private:
  const LogicCore* read_;
};

// What a part of the logic reads and changes of the core, handed in like CoreFacts.
class CoreAccess : public CoreFacts {
 public:
  explicit CoreAccess(LogicCore& core) : CoreFacts(core), core_(&core) {}

  EffectQueue& effects();
  // The client of the command being run, for its events.
  std::string& client();
  HomeWatch& home();
  std::string AddEvent(const std::string& type, json details);
  void Notify(std::string line);
  void SetState(State state);
  void CommandCancels();

 private:
  LogicCore* core_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_CORE_ACCESS_H
