// The calls of LogicCore that belong to one of its parts.
#include <optional>
#include <string>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

Reason LogicCore::OnSetupMotion(bool active, const std::string& step, double now_s) {
  return setup_.OnSetupMotion(CoreAccess(*this), active, step, now_s);
}

void LogicCore::OnSetupBlade(const std::string& step, int blade) {
  setup_.OnSetupBlade(CoreFacts(*this), step, blade);
}

bool LogicCore::SetupBladeOnce() const {
  return setup_.SetupBladeOnce(CoreFacts(*this));
}

bool LogicCore::TickSetup(double now_s) {
  return setup_.TickSetup(CoreAccess(*this), now_s);
}

bool LogicCore::InSetup() const {
  return state_ == State::kSetup;
}

bool LogicCore::SetupActive() const {
  return state_ == State::kSetup && !emergency_;
}

}  // namespace mowbite_logic
