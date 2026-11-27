#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include "mowbite_logic/setup_part.h"

namespace mowbite_logic {

namespace {

// s without setup/motion after which SETUP ends; the program sends it at 4 Hz.
constexpr double kSetupSignTimeout = 1.0;

}  // namespace

Reason SetupPart::OnSetupMotion(CoreAccess core, bool active, const std::string& step, double now_s) {
  if (!active) {
    if (core.state() == State::kSetup) EndSetup(core, std::nullopt);
    return {};
  }
  if (core.state() == State::kSetup) {
    setup_sign_s_ = now_s;
    if (step == setup_step_) return {};
    // Another step without an end of the first: it begins anew.
    EndSetup(core, std::nullopt);
  }
  Reason why = core.Busy();
  if (why.empty()) why = core.HoldProblem();
  if (why.empty() && core.emergency()) why = Reason("EMERGENCY", "Emergency: reset it first");
  if (why.empty() && (core.DockContact() || (core.hw().telemetry_ok && core.hw().dock_contact_raw))) {
    why = Reason("ON_DOCK", "The mower is on the docking station");
  }
  if (!why.empty()) return why;
  core.CommandCancels();
  setup_step_ = step;
  setup_sign_s_ = now_s;
  setup_blade_ = 0;
  setup_blade_refused_ = false;
  setup_blade_hold_s_ = 0;
  core.SetState(State::kSetup);
  return {};
}

void SetupPart::OnSetupBlade(CoreFacts core, const std::string& step, int blade, double input_hold_s) {
  if (core.state() != State::kSetup || step != setup_step_) return;
  setup_blade_hold_s_ = std::isfinite(input_hold_s) && input_hold_s > 0 ? input_hold_s : 0;
  const int asked = blade == 1 || blade == 2 ? blade : 0;
  if (asked == setup_blade_) return;
  setup_blade_ = asked;
  setup_blade_refused_ = false;
}

bool SetupPart::SetupBladeOnce(CoreFacts core) const {
  return core.state() == State::kSetup && setup_blade_ == 2;
}

bool SetupPart::TickSetup(CoreAccess core, double now_s) {
  if (core.state() != State::kSetup) return false;
  const double quiet = now_s - setup_sign_s_;
  if (quiet > kSetupSignTimeout) {
    char text[96];
    std::snprintf(text, sizeof(text), "No sign of the setup program for %.1f s", quiet);
    EndSetup(core, std::string(text));
    return true;
  }
  Reason why = core.HoldProblem();
  if (why.empty() && (core.DockContact() || (core.hw().telemetry_ok && core.hw().dock_contact_raw))) {
    why = Reason("ON_DOCK", "The mower is on the docking station");
  }
  if (why.empty()) return false;
  EndSetup(core, why.message);
  return true;
}

void SetupPart::EndSetup(CoreAccess core, const std::optional<std::string>& why) {
  if (why) core.Notify("Setup step " + setup_step_ + " stopped: " + *why);
  setup_step_.clear();
  // The blade goes off with the state: BladeWanted holds outside SETUP only while mowing.
  setup_blade_ = 0;
  setup_blade_refused_ = false;
  setup_blade_hold_s_ = 0;
  // One zero follows from leaving the state; the wheels stand.
  core.SetState(State::kIdle);
}

void SetupPart::BladeRefused(CoreAccess core, const std::string& message) {
  setup_blade_refused_ = true;
  core.Notify("Setup step " + setup_step_ + ": the blade was refused: " + message);
}

}  // namespace mowbite_logic
