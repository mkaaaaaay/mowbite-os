// The step hardware: an installation leaves the drivers of the platform off (MOWBITE_HARDWARE=false). The step is ok
// while they run; its start asks mowbite-host to switch them on, which restarts the OS.
#include <cmath>

#include "mowbite_setup/program.h"
#include "program_util.h"

namespace mowbite_setup {

using namespace util;

bool Program::IsHardware(const Step& step) {
  return step.check == "hardware";
}

void Program::SetDrivers(bool on) {
  drivers_on_ = on;
  MarkHardware();
}

void Program::MarkHardware() {
  for (const Step& step : list_.steps) {
    if (!IsHardware(step)) continue;
    Record& record = records_.at(step.id);
    if (record.state == "running") continue;
    if (drivers_on_) {
      record.state = "ok";
    } else if (record.state == "ok") {
      record.state = "open";
    }
  }
}

bool Program::HardwareRestart() const {
  return run_ && IsHardware(StepOf(*run_)) && !run_->restarted;
}

std::vector<Refusal> Program::HardwareProblems() const {
  std::vector<Refusal> problems;
  if (drivers_on_) problems.push_back(Refuse("NOT_OFFERED", "The drivers run already"));
  if (list_.restart != "host") problems.push_back(Refuse("NOT_OFFERED", "Only mowbite-host switches the drivers"));
  if (!world_.state.empty() && world_.state != "IDLE") {
    problems.push_back(Refuse("BUSY", "The mower is in " + world_.state, {{"state", world_.state}}));
  }
  return problems;
}

void Program::BeginHardware(const Step& step, double wall) {
  RecordOf(step).applied = {{"t", std::floor(wall)}};
  WantRestart();
}

void Program::TickHardware(double now, double wall) {
  Run& run = *run_;
  const double restart_time = StepOf(run).Number("restart_time");
  if (!run.restarted) {
    if (restart_sent_ >= 0 && now - restart_sent_ > restart_time) {
      restart_sent_ = -1;
      Fail("RESTART_FAILED", "The OS did not restart", {{"limit", restart_time}}, now, wall);
    }
    return;
  }
  if (!drivers_on_) {
    Fail("NOT_APPLIED", "The drivers are still off after the restart", json::object(), now, wall);
    return;
  }
  Pass(StepOf(run), now, wall);
}

}  // namespace mowbite_setup
