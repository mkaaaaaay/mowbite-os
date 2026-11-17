#include <cstdio>
#include <optional>
#include <string>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

namespace {

// s without setup/motion after which SETUP ends; the program sends it at 4 Hz.
constexpr double kSetupSignTimeout = 1.0;

}  // namespace

Reason LogicCore::OnSetupMotion(bool active, const std::string& step, double now_s) {
  if (!active) {
    if (state_ == State::kSetup) EndSetup(std::nullopt);
    return {};
  }
  if (state_ == State::kSetup) {
    setup_sign_s_ = now_s;
    if (step == setup_step_) return {};
    // Another step without an end of the first: it begins anew.
    EndSetup(std::nullopt);
  }
  Reason why = Busy();
  if (why.empty()) why = HoldProblem();
  if (why.empty() && emergency_) why = Reason("EMERGENCY", "Emergency: reset it first");
  if (why.empty() && (DockContact() || (hw_.telemetry_ok && hw_.dock_contact_raw))) {
    why = Reason("ON_DOCK", "The mower is on the docking station");
  }
  if (!why.empty()) return why;
  CommandCancels();
  setup_step_ = step;
  setup_sign_s_ = now_s;
  setup_blade_ = 0;
  setup_blade_refused_ = false;
  SetState(State::kSetup);
  return {};
}

void LogicCore::OnSetupBlade(const std::string& step, int blade) {
  if (state_ != State::kSetup || step != setup_step_) return;
  const int asked = blade == 1 || blade == 2 ? blade : 0;
  if (asked == setup_blade_) return;
  setup_blade_ = asked;
  setup_blade_refused_ = false;
}

bool LogicCore::SetupBladeOnce() const {
  return state_ == State::kSetup && setup_blade_ == 2;
}

bool LogicCore::TickSetup(double now_s) {
  if (state_ != State::kSetup) return false;
  const double quiet = now_s - setup_sign_s_;
  if (quiet > kSetupSignTimeout) {
    char text[96];
    std::snprintf(text, sizeof(text), "No sign of the setup program for %.1f s", quiet);
    EndSetup(std::string(text));
    return true;
  }
  Reason why = HoldProblem();
  if (why.empty() && (DockContact() || (hw_.telemetry_ok && hw_.dock_contact_raw))) {
    why = Reason("ON_DOCK", "The mower is on the docking station");
  }
  if (why.empty()) return false;
  EndSetup(why.message);
  return true;
}

bool LogicCore::InSetup() const {
  return state_ == State::kSetup;
}

bool LogicCore::SetupActive() const {
  return state_ == State::kSetup && !emergency_;
}

void LogicCore::EndSetup(const std::optional<std::string>& why) {
  if (why) Notify("Setup step " + setup_step_ + " stopped: " + *why);
  setup_step_.clear();
  // The blade goes off with the state: BladeWanted holds outside SETUP only while mowing.
  setup_blade_ = 0;
  setup_blade_refused_ = false;
  // One zero follows from leaving the state; the wheels stand.
  SetState(State::kIdle);
}

}  // namespace mowbite_logic
