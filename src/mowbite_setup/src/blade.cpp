// The step blade: the blade taken off, the wheels without a command. Each part starts the blade through the logic
// (setup/motion), which turns it only in SETUP; on_off, lease and silent stop it by themselves, stop, lift and bumper
// wait for the input the user triggers. After a stand nothing may turn; an emergency a part brought the program
// releases itself as in stop_moving.
#include <algorithm>
#include <cmath>

#include "mowbite_api/setup.h"
#include "mowbite_setup/program.h"
#include "program_util.h"

namespace mowbite_setup {

using namespace util;

namespace {

constexpr int kBladeOff = 0;
constexpr int kBladeOn = 1;
constexpr int kBladeOnce = 2;

}  // namespace

bool Program::IsBlade(const Step& step) {
  return step.check == "blade";
}

bool Program::BladePress() const {
  if (!run_ || !IsBlade(StepOf(*run_))) return false;
  return !StepOf(*run_).parts[run_->part].role.empty();
}

double Program::BladeLease(const Step& step) const {
  const double most = capabilities_.blade_lease_max_s;
  return most > 0 ? std::min(step.Number("lease"), most) : step.Number("lease");
}

// How long the blade may take to stand in a part, from its trigger (s).
double Program::BladeLimit(const Step& step, const std::string& part) const {
  if (part == "on_off") return step.Number("off_max");
  if (part == "lease") return BladeLease(step) + step.Number("lease_extra");
  if (part == "silent") return step.Number("silent_max");
  return step.Number("stop_max");
}

bool Program::BladeTurns(const Step& step) const {
  return hardware_.blade_ok && std::abs(hardware_.blade_rpm) >= step.Number("still_rpm");
}

std::vector<Refusal> Program::BladeProblems(const Step& step) const {
  std::vector<Refusal> problems;
  if (!capabilities_known_) {
    problems.push_back(Refuse("DRIVER_MISSING", "The driver of the platform sent no hw/capabilities"));
    return problems;
  }
  if (capabilities_.silence_max_s <= 0) problems.push_back(Refuse("NOT_OFFERED", "The driver offers no hw/silence"));
  if (capabilities_.blade_allowed) return problems;
  if (!layer_set_) {
    problems.push_back(Refuse("STORAGE_NOT_WRITABLE", "No data_dir for allow_blade", {{"file", "setup.yaml"}}));
    return problems;
  }
  // The restart with allow_blade is asked for or under way.
  if (restart_wanted_ || restart_sent_ >= 0) {
    problems.push_back(Refuse("BUSY", "The OS restarts with the blade allowed", {{"state", "RESTART"}}));
    return problems;
  }
  const std::string& node = step.Word("node");
  const std::string key = node + "." + mowbite_api::kAllowBlade;
  // The program set it and the driver still says no: the file kept by hand or the driver itself overrules it.
  if (shadowed_.count(key) || layer_.Get(node, mowbite_api::kAllowBlade) == true) {
    json data = json::object();
    if (shadowed_.count(key)) data["key"] = key;
    problems.push_back(Refuse("BLADE_NOT_ALLOWED", "The driver does not allow the blade", std::move(data)));
  }
  return problems;
}

bool Program::AllowBlade(const Step& step) {
  if (capabilities_.blade_allowed) return false;
  layer_.Set(step.Word("node"), mowbite_api::kAllowBlade, true);
  WantRestart();
  events_.push_back({"SETUP", {{"what", "blade_allowed"}, {"step", step.id}}});
  Changed();
  return true;
}

void Program::BeginBladePart(double now, double wall) {
  Run& run = *run_;
  run.part_states[run.part] = "waiting";
  run.part_began = now;
  run.part_wall = wall;
  run.trigger = run.stood = run.silence_end = -1;
  run.expected = 0;
  run.armed = false;
  run.asked = false;
  run.peak = 0;
  run.blade = kBladeOn;
  EnterDrive(Drive::kRunUp, now, wall);
}

void Program::NextBladePart(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  run.part_states[run.part] = "ok";
  run.blade = kBladeOff;
  if (run.part + 1 >= step.parts.size()) {
    Pass(step, now, wall);
    return;
  }
  ++run.part;
  BeginBladePart(now, wall);
}

void Program::TickBlade(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  const Part& part = step.parts[run.part];
  const double elapsed = now - run.drive_began;
  if (run.drive != Drive::kEnter && !hardware_.blade_ok) {
    DriveFail("NO_DATA", "hw/status reports no blade motor", {{"source", "hw/status"}}, now, wall);
    return;
  }
  const double rpm = std::abs(hardware_.blade_rpm);
  const bool press = BladePress();
  const auto started_again = [&](const char* when) {
    DriveFail("STARTED_AGAIN", std::string("The blade turned ") + when, {{"rpm", std::round(rpm)}}, now, wall);
  };
  switch (run.drive) {
    case Drive::kEnter:
      if (world_.state == "SETUP") {
        BeginBladePart(now, wall);
      } else if (elapsed > kEnterWait) {
        DriveFail("LOGIC_REFUSED", "The logic did not take SETUP, it is in " + world_.state,
                  {{"mower_state", world_.state}}, now, wall);
      }
      return;
    case Drive::kRunUp:
      run.peak = std::max(run.peak, rpm);
      if (rpm >= step.Number("rpm")) {
        if (part.id == "on_off") run.ground["rpm"] = run.peak;
        if (part.id == "on_off" || part.id == "lease") {
          // The off, or one request on a lease that is not renewed: the driver has to switch it off.
          run.blade = part.id == "on_off" ? kBladeOff : kBladeOnce;
          run.trigger = now;
          EnterDrive(Drive::kStop, now, wall);
        } else if (part.id == "silent") {
          silence_due_ = capabilities_.silence_max_s;
          run.asked = true;
          run.asked_at = now;
          EnterDrive(Drive::kTrigger, now, wall);
        } else {
          EnterDrive(Drive::kTrigger, now, wall);
        }
      } else if (elapsed > step.Number("spin_up")) {
        DriveFail("NO_START",
                  "The blade did not reach " + std::to_string(step.Number("rpm")) + " rpm within " +
                      std::to_string(step.Number("spin_up")) + " s, it turned at " + std::to_string(run.peak),
                  {{"rpm", std::round(run.peak)}}, now, wall);
      }
      return;
    case Drive::kTrigger:
      if (press) {
        if (TakeTrigger(part, now)) {
          EnterDrive(Drive::kStop, now, wall);
        } else if (now - run.drive_began > step.Number("time")) {
          run.blade = kBladeOff;
          AskPressed("NOT_SEEN", json::object(), now, wall);
        }
      } else if (run.asked && now - run.asked_at > kServiceWait) {
        DriveFail("DRIVER_REFUSED", "The driver did not answer", {{"code", "NO_ANSWER"}}, now, wall);
      }
      return;
    case Drive::kStop: {
      const double limit = BladeLimit(step, part.id);
      if (!BladeTurns(step) && blade_still_since_ >= 0) {
        const double stand = std::max(0.0, blade_still_since_ - run.trigger);
        run.ground[part.id] = stand;
        run.stood = blade_still_since_;
        run.blade = kBladeOff;
        if (stand > limit) {
          DriveFail("LIMIT", "The blade stood " + std::to_string(stand) + " s after the " + part.id,
                    {{"value_id", part.id}, {"value", Rounded(stand, 100)}, {"limit", Rounded(limit, 100)},
                     {"unit", "s"}},
                    now, wall);
          return;
        }
        EnterDrive(Drive::kQuiet, now, wall);
      } else if (now - run.trigger > limit) {
        DriveFail("LIMIT", "The blade did not stand within " + std::to_string(limit) + " s",
                  {{"value_id", part.id}, {"value", Rounded(now - run.trigger, 100)}, {"limit", Rounded(limit, 100)},
                   {"unit", "s"}},
                  now, wall);
      }
      return;
    }
    case Drive::kQuiet:
      if (BladeTurns(step)) {
        started_again("after it stood");
      } else if (now >= std::max(run.stood, run.silence_end) + step.Number("hold")) {
        if (run.expected == 0) {
          NextBladePart(now, wall);
        } else {
          EnterDrive(Drive::kLatch, now, wall);
        }
      }
      return;
    case Drive::kLatch:
      if ((hardware_.emergency_reason & run.expected) != 0) {
        // The press took its own latch at the trigger; silent takes the one the driver latched after it.
        if (!press) {
          OwnLatch own;
          own.bit = run.expected;
          own.held = true;
          own.edge = now;
          own_ = own;
        }
        if (own_) own_->confirmed = true;
        WatchOwnLatch();
        EnterDrive(Drive::kRelease, now, wall);
      } else if (elapsed > kLatchWait) {
        DriveFail("NO_EMERGENCY", "No emergency " + mowbite_api::EmergencyNames(static_cast<int>(run.expected)),
                  {{"limit", Rounded(now - run.trigger, 100)}}, now, wall);
      }
      return;
    case Drive::kRelease:
      if (BladeTurns(step)) {
        started_again("before the release");
      } else if (hardware_.emergency_reason == 0) {
        EnterDrive(Drive::kArm, now, wall);
      } else if (!own_ || elapsed > kReleaseWithin) {
        DriveFail("NOT_RELEASED", "The emergency stays: " +
                                      mowbite_api::EmergencyNames(static_cast<int>(hardware_.emergency_reason)),
                  json::object(), now, wall);
      }
      return;
    case Drive::kArm:
      if (BladeTurns(step)) {
        started_again("after the release");
      } else if (elapsed >= step.Number("hold")) {
        NextBladePart(now, wall);
      }
      return;
    default:
      return;
  }
}

json Program::BladeValues(const Run& run, const Step& step, bool final) const {
  json values = json::array();
  const auto find = [&run](const std::string& id) -> std::optional<double> {
    const auto found = run.ground.find(id);
    if (found == run.ground.end()) return std::nullopt;
    return found->second;
  };
  const auto rpm = find("rpm");
  values.push_back(Measured("blade", "rpm", rpm ? json(std::round(*rpm)) : json(), step.Number("rpm"), nullptr,
                            rpm ? json(*rpm >= step.Number("rpm")) : json()));
  for (size_t i = 0; i < step.parts.size(); ++i) {
    const Part& part = step.parts[i];
    const double limit = BladeLimit(step, part.id);
    const auto stand = find(part.id);
    json ok = stand ? json(*stand <= limit) : final && i == run.part ? json(false) : json();
    values.push_back(Value(part.id, stand ? json(Rounded(*stand, 100)) : json(), "s", nullptr, Rounded(limit, 100),
                           std::move(ok), part.texts));
  }
  return values;
}

}  // namespace mowbite_setup
