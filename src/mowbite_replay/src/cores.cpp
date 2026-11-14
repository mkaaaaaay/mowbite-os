#include "mowbite_replay/cores.h"

#include <mowbite_localization/dock_estimator_journal.h>
#include <mowbite_localization/estimator_journal.h>
#include <mowbite_logic/logic_journal.h>
#include <mowbite_safety/safety_journal.h>
#include <mowbite_schedule/schedule_journal.h>
#include <tron_mcu/mcu_journal.h>

namespace mowbite_replay {

mowbite_journal::ReplayRegistry Cores() {
  mowbite_journal::ReplayRegistry registry;
  mowbite_journal::Register<mowbite_localization::Estimator>(registry);
  mowbite_journal::Register<mowbite_localization::DockEstimator>(registry);
  mowbite_journal::Register<mowbite_logic::LogicCore>(registry);
  mowbite_journal::Register<mowbite_safety::SafetyFilter>(registry);
  mowbite_journal::Register<mowbite_schedule::Scheduler>(registry);
  mowbite_journal::Register<tron::McuControl>(registry);
  return registry;
}

}  // namespace mowbite_replay
