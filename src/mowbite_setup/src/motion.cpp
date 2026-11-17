// The steps of the setup program that move the wheels (checks of kind drive) or the blade (kind blade). They drive on
// setup/cmd_vel, which the logic passes only in its sub state SETUP while setup/motion of the step lasts and no
// emergency stands, and which mowbite_safety caps to the bounds of setup/motion and its own; the blade turns as
// setup/motion asks, also only in SETUP.
#include <algorithm>
#include <cmath>

#include "mowbite_api/catalog.h"
#include "mowbite_api/setup.h"
#include "mowbite_setup/program.h"
#include "program_util.h"

namespace mowbite_setup {

using namespace util;

namespace {

// A part of wheels speeds up for this long before it is measured (s).
constexpr double kRunUp = 0.5;
// The wheels turn within kRestartWait of the command after the release (s).
constexpr double kRestartWait = 2.0;
// The wheels stand again within this time of the zero at the end of a pass (s).
constexpr double kPauseWait = 3.0;
// A wheel below kStandSpeed (m/s) stands; both have to for kStandHold s in a row, so that a slow wheel between two
// ticks of its encoder does not count as standing.
constexpr double kStandSpeed = 0.01;
constexpr double kStandHold = 0.3;
// A wheel faster than this (m/s), now and filtered, has started.
constexpr double kStartedSpeed = 0.02;
// Time constant of the filtered speeds (s); a gap in hw/wheel_odom this long starts the filter anew (s).
constexpr double kWheelFilter = 0.2;
constexpr double kWheelGap = 0.2;
// hw/status away this long after hw/end: the driver ended, the next one is the new driver (s). It is back once it hears
// its board and shows no emergency but those of the step: what a driver reports while it starts (a missing input, a
// service not ready yet) ends within return_max, else the step fails with NO_RETURN.
constexpr double kDriverGap = 0.5;
// The wheels turn once both reach this share of the speed asked for. A part of wheels measures 0.5 to 1.5 times it,
// and a wheel that is to stand turns at most at kStillShare of it.
constexpr double kTurnShare = 0.5;
constexpr double kSpeedLow = 0.5;
constexpr double kSpeedHigh = 1.5;
constexpr double kStillShare = 0.25;

// Signs of the wheels in a part of wheels (left, right); one_wheel turns the right one alone.
struct WheelPart {
  const char* id;
  double left;
  double right;
};
constexpr WheelPart kWheelParts[] = {
    {"forward", 1, 1}, {"backward", -1, -1}, {"left", -1, 1}, {"right", 1, -1}, {"one_wheel", 0, 1}};

const WheelPart& WheelPartOf(const std::string& id) {
  for (const auto& part : kWheelParts) {
    if (id == part.id) return part;
  }
  return kWheelParts[0];
}

// The emergency a press of the role brings; the whole bumper pressed in its middle, or the mower lifted at the front,
// may bring both sides at once.
uint32_t RoleBit(const std::string& role) {
  const uint32_t bit = EmergencyBit(mowbite_api::FindSetupName(mowbite_api::SetupRoles(), role)->emergency);
  if (role == "bumper") return bit | EmergencyBit("COLLISION_MULTIPLE");
  if (role == "lift") return bit | EmergencyBit("LIFT_MULTIPLE");
  return bit;
}

uint32_t BitsOf(const std::vector<std::string>& names) {
  uint32_t bits = 0;
  for (const auto& name : names) bits |= EmergencyBit(name);
  return bits;
}

}  // namespace

bool Program::IsDrive(const Step& step) {
  return step.kind == "drive" || step.kind == "blade";
}

double Program::Speed(const Step& step) const {
  return step.Number("speed");
}

// A turn on the spot at the speed of the step on each wheel, at most the turn rate of a mower jacked up.
double Program::TurnRate(const Step& step) const {
  return track_ > 0 ? std::min(Speed(step) / (track_ / 2), mowbite_api::kSetupRaisedTurn) : 0;
}

json Program::MotionOf(const Step& step) const {
  if (!IsDrive(step)) return nullptr;
  if (IsBlade(step)) {
    return {{"wheels", false},  {"blade", true},       {"raised", false},
            {"max_speed", 0.0}, {"max_distance", 0.0}, {"max_time", mowbite_api::kSetupMoveTime}};
  }
  const bool ground = IsGround(step);
  const double speed = ground ? std::max(Speed(step), step.Number("bumper_speed")) : Speed(step);
  return {{"wheels", true},      {"blade", false},          {"raised", !ground},
          {"max_speed", speed}, {"max_distance", nullptr}, {"max_time", mowbite_api::kSetupMoveTime}};
}

void Program::OnWheels(double left, double right, double now) {
  const double dt = wheels_at_ < 0 ? -1 : now - wheels_at_;
  if (run_ && IsGround(StepOf(*run_)) && dt > 0) {
    // The way and the turn by odometry; a gap counts as one sample long at most.
    const double step = std::min(dt, kWheelGap);
    run_->way += std::abs(left + right) / 2 * step;
    run_->travel += (left + right) / 2 * step;
    if (track_ > 0) run_->yaw_odom += (right - left) / track_ * step;
  }
  if (dt < 0 || dt > kWheelGap) {
    left_filtered_ = left;
    right_filtered_ = right;
  } else {
    const double share = dt / (kWheelFilter + dt);
    left_filtered_ += share * (left - left_filtered_);
    right_filtered_ += share * (right - right_filtered_);
  }
  left_ = left;
  right_ = right;
  wheels_at_ = now;
  if (std::abs(left) >= kStandSpeed || std::abs(right) >= kStandSpeed) {
    still_since_ = -1;
  } else if (still_since_ < 0) {
    still_since_ = now;
  }
  if (run_ && StepOf(*run_).check == "wheels" && run_->drive == Drive::kMeasure) {
    run_->sum_left += left;
    run_->sum_right += right;
    ++run_->samples;
  }
}

// A step driver_end waits for the new driver: hw/status after a gap, from a driver that hears its board and shows no
// emergency but those of the step.
void Program::NoteDriverBack(const Hardware& hardware, double now) {
  if (!run_ || StepOf(*run_).check != "driver_end" || run_->drive != Drive::kReturn || run_->returned >= 0) return;
  if (hardware_at_ >= 0 && now - hardware_at_ > kDriverGap) run_->gap = true;
  const bool settled = (hardware.emergency_reason & ~(run_->expected | LatchBit())) == 0;
  if (run_->gap && hardware.telemetry_ok && settled) run_->returned = now;
}

void Program::SetTrack(double track) {
  track_ = std::isfinite(track) && track > 0 ? track : 0;
}

void Program::SetDriverRestarts(int left) {
  restarts_left_ = left;
}

bool Program::Standing(double now) const {
  return Fresh(wheels_at_, now, kFresh) && still_since_ >= 0 && now - still_since_ >= kStandHold;
}

bool Program::Started() const {
  const auto above = [](double a, double b) { return std::max(std::abs(a), std::abs(b)) > kStartedSpeed; };
  return above(left_, right_) && above(left_filtered_, right_filtered_);
}

bool Program::Turning(double speed) const {
  if (run_ && IsGround(StepOf(*run_))) {
    // Along the command of the part: forwards, backwards or on the spot.
    const Command command = GroundCommand();
    const double along = (left_filtered_ + right_filtered_) / 2;
    const double spin = (right_filtered_ - left_filtered_) / 2;
    if (command.linear != 0) return along * command.linear > 0 && std::abs(along) >= kTurnShare * speed;
    return spin * command.angular > 0 && std::abs(spin) >= kTurnShare * speed;
  }
  return left_filtered_ >= kTurnShare * speed && right_filtered_ >= kTurnShare * speed;
}

std::vector<Refusal> Program::DriveProblems(const Step& step, double now) const {
  std::vector<Refusal> problems;
  if (run_) {
    problems.push_back(Refuse("BUSY", StepOf(*run_).id + " is running", {{"step", StepOf(*run_).id}}));
  } else if (!world_.state.empty() && world_.state != "IDLE") {
    problems.push_back(Refuse("BUSY", "The mower is in " + world_.state, {{"state", world_.state}}));
  }
  if (learning_) problems.push_back(Refuse("BUSY", "The driver learns its inputs"));
  if (hardware_.hold) problems.push_back(Refuse("BUSY", "An update holds the mower", {{"state", "HOLD"}}));
  if (!Fresh(hardware_at_, now, kFresh) || !Fresh(wheels_at_, now, kFresh)) {
    problems.push_back(Refuse("DRIVER_MISSING", "The driver of the platform reports no status or no wheel speeds"));
  } else {
    if (hardware_.emergency_reason != 0) {
      problems.push_back(Refuse("EMERGENCY", "An emergency stands: " + mowbite_api::EmergencyNames(static_cast<int>(
                                                                         hardware_.emergency_reason))));
    }
    if (hardware_.dock_contact) problems.push_back(Refuse("ON_DOCK", "The mower is on the charging contacts"));
  }
  // Rule (3) of the program: no wheel turns before stop, lift and bumper passed standing; skipped does not count.
  json missing = json::array();
  for (const Step& other : list_.steps) {
    if (other.check != "input" || records_.at(other.id).state == "ok") continue;
    const std::string group = GroupOf(other.Word("role"));
    if (group == "stop" || group == "lift" || group == "bumper") missing.push_back(other.id);
  }
  if (!missing.empty()) {
    problems.push_back(
        Refuse("SAFETY_INPUTS", "The inputs are not all checked: " + missing.dump(), {{"steps", missing}}));
  }
  if (IsBlade(step)) {
    for (auto& problem : BladeProblems(step)) problems.push_back(std::move(problem));
    return problems;
  }
  if (track_ <= 0) problems.push_back(Refuse("NOT_OFFERED", "The track of the mower is not known yet"));
  if (IsGround(step)) {
    for (auto& problem : GroundProblems(now)) problems.push_back(std::move(problem));
  }
  if (step.check == "silence" && capabilities_.silence_max_s <= 0) {
    problems.push_back(Refuse("NOT_OFFERED", "The driver offers no hw/silence"));
  }
  if (step.check == "driver_end") {
    if (!capabilities_.driver_end) {
      problems.push_back(Refuse("NOT_OFFERED", "The driver offers no hw/end"));
    } else if (restarts_left_ < 2) {
      problems.push_back(Refuse("RESTARTS_LOW", "The start has " + std::to_string(restarts_left_) +
                                                    " restarts of the driver left, 2 are needed",
                                {{"left", restarts_left_}}));
    }
  }
  return problems;
}

void Program::BeginDrive(double now, double wall) {
  Run& run = *run_;
  run.keep = now;
  run.command.reset();
  run.move_began = -1;
  EnterDrive(Drive::kEnter, now, wall);
}

void Program::EnterDrive(Drive drive, double now, double wall) {
  Run& run = *run_;
  run.drive = drive;
  run.drive_began = now;
  run.drive_wall = wall;
  const Step& step = StepOf(run);
  // The wheels of a step blade get no command at all.
  if (IsBlade(step)) {
    run.command.reset();
    run.move_began = -1;
    return;
  }
  // What the wheels are asked for in each stage: forwards while they speed up and until the trigger, nothing after
  // the trigger of no_command, zero while they have to stand.
  std::optional<Command> command;
  switch (drive) {
    case Drive::kRunUp:
    case Drive::kRestart:
      command = Command{Speed(step), 0};
      if (IsGround(step)) command = GroundCommand();
      if (step.check == "wheels") {
        const WheelPart& part = WheelPartOf(step.parts[run.part].id);
        // one_wheel: the right wheel at speed, the left one standing.
        const double speed = Speed(step);
        command = part.left == 0 ? Command{speed / 2, track_ > 0 ? speed / track_ : 0}
                                 : Command{(part.left + part.right) / 2 * speed,
                                           (part.right - part.left) / 2 * TurnRate(step)};
      }
      break;
    case Drive::kTrigger:
    case Drive::kStop:
    case Drive::kReturn:
    case Drive::kQuiet:
    case Drive::kMeasure:
      command = run.command;
      if (step.check == "no_command" && drive != Drive::kTrigger) command.reset();
      // straight falls silent at its trigger.
      if (IsGround(step) && step.parts[run.part].id == "straight" && drive != Drive::kMeasure) command.reset();
      break;
    case Drive::kLatch:
    case Drive::kRelease:
    case Drive::kArm:
    case Drive::kPause:
    case Drive::kAsk:
      command = Command{0, 0};
      break;
    case Drive::kEnter:
      break;
  }
  const bool moves = command && (command->linear != 0 || command->angular != 0);
  if (!moves) {
    run.move_began = -1;
  } else if (run.move_began < 0) {
    run.move_began = now;
  }
  run.command = command;
}

// The pass of now is done: the next part or round, or the end of the step.
void Program::NextPass(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  bool more = false;
  if (!step.parts.empty()) {
    run.part_states[run.part] = "ok";
    more = run.part + 1 < step.parts.size();
    if (more) ++run.part;
  } else if (step.check == "silence") {
    ++run.round;
    more = run.round < static_cast<int>(std::lround(step.Number("rounds")));
  }
  if (!more) {
    if (step.check == "silence" && run.stands.size() > 1) {
      const auto [low, high] = std::minmax_element(run.stands.begin(), run.stands.end());
      const double spread = *high - *low;
      if (spread > step.Number("spread_max")) {
        DriveFail("LIMIT", "The stand times of the rounds differ by " + std::to_string(spread) + " s",
                  {{"value_id", "spread"}, {"value", Rounded(spread, 100)}, {"limit", step.Number("spread_max")},
                   {"unit", "s"}},
                  now, wall);
        return;
      }
    }
    Pass(step, now, wall);
    return;
  }
  run.trigger = run.stood = run.silence_end = run.returned = -1;
  run.gap = false;
  run.expected = 0;
  run.armed = false;
  run.asked = false;
  run.sum_left = run.sum_right = 0;
  run.samples = 0;
  if (!step.parts.empty()) {
    run.part_states[run.part] = "waiting";
    run.part_began = now;
    run.part_wall = wall;
  }
  EnterDrive(Drive::kRunUp, now, wall);
}

void Program::DriveFail(const std::string& reason, const std::string& message, json fields, double now,
                        double wall) {
  const Step& step = StepOf(*run_);
  if (step.check == "silence" && !fields.contains("part")) fields["part"] = "round_" + std::to_string(run_->round + 1);
  Fail(reason, message, std::move(fields), now, wall);
}

uint32_t Program::AllowedNow() const {
  const Run& run = *run_;
  const Step& step = StepOf(run);
  switch (run.drive) {
    case Drive::kRunUp:
      // On the ground the user may act as soon as the mower moves off.
      if (IsGround(step) && GroundPress()) return RoleBit(step.parts[run.part].role) | LatchBit();
      return 0;
    case Drive::kTrigger:
      // A press may latch before hw/status shows the input.
      if (step.check == "stop_moving" || (IsGround(step) && GroundPress()) || BladePress()) {
        return RoleBit(step.parts[run.part].role) | LatchBit();
      }
      return 0;
    case Drive::kStop:
    case Drive::kReturn:
    case Drive::kQuiet:
    case Drive::kLatch:
    case Drive::kRelease:
      return run.expected == 0 ? 0 : run.expected | LatchBit();
    default:
      return 0;
  }
}

void Program::TickDrive(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  if (now - run.keep > mowbite_api::kSetupKeepTimeout) {
    DriveFail("RELEASED", "No setup.keep for " + std::to_string(now - run.keep) + " s", json::object(), now, wall);
    return;
  }
  const bool away = run.drive == Drive::kReturn;
  if (!away && !Fresh(hardware_at_, now, kNoData)) {
    DriveFail("NO_DATA", "No hw/status any more", {{"source", "hw/status"}}, now, wall);
    return;
  }
  if (!away && run.drive != Drive::kEnter && !IsBlade(step) && !Fresh(wheels_at_, now, kNoData)) {
    DriveFail("NO_DATA", "No hw/wheel_odom any more", {{"source", "hw/wheel_odom"}}, now, wall);
    return;
  }
  if (!away) {
    const uint32_t other = hardware_.emergency_reason & ~AllowedNow();
    if (other != 0) {
      const std::string names = mowbite_api::EmergencyNames(static_cast<int>(other));
      DriveFail("EMERGENCY", "Emergency " + names, {{"emergency", names}}, now, wall);
      return;
    }
  }
  if (IsBlade(step)) {
    if (run.drive == Drive::kAsk) {
      TickPressed(now, wall);
    } else {
      TickBlade(now, wall);
    }
    return;
  }
  // ground: its own stages; the release after a press goes on below.
  if (IsGround(step) && run.drive != Drive::kLatch && run.drive != Drive::kRelease && run.drive != Drive::kArm &&
      run.drive != Drive::kAsk) {
    TickGround(now, wall);
    return;
  }
  // Where the user did not act in time: before the limit of a single move, which is no measurement.
  const bool stop_moving = step.check == "stop_moving";
  if (stop_moving && run.drive == Drive::kTrigger && run.move_began >= 0 &&
      now - run.move_began > step.Number("time")) {
    AskPressed("NOT_SEEN", json::object(), now, wall);
    return;
  }
  if (step.check == "wheels" && run.drive == Drive::kMeasure && WheelPartOf(step.parts[run.part].id).left == 0 &&
      run.answer.empty() && now - run.move_began > step.Number("answer_time")) {
    DriveFail("NOT_DONE", "No answer which wheel turns", json::object(), now, wall);
    return;
  }
  // A single move: while the wheels are to turn. After a trigger the commands go on only to show that the wheels stay.
  const bool turns = run.drive == Drive::kRunUp || run.drive == Drive::kTrigger || run.drive == Drive::kMeasure ||
                     run.drive == Drive::kRestart;
  if (turns && run.move_began >= 0 && now - run.move_began > mowbite_api::kSetupMoveTime) {
    DriveFail("LIMIT", "A move lasted longer than " + std::to_string(mowbite_api::kSetupMoveTime) + " s",
              {{"limit", mowbite_api::kSetupMoveTime}, {"unit", "s"}}, now, wall);
    return;
  }
  if (step.check == "wheels") {
    TickWheels(now, wall);
    return;
  }
  const double elapsed = now - run.drive_began;
  const double speed = Speed(step);
  switch (run.drive) {
    case Drive::kEnter:
      if (world_.state == "SETUP") {
        if (!step.parts.empty()) run.part_states[run.part] = "waiting";
        EnterDrive(Drive::kRunUp, now, wall);
      } else if (elapsed > kEnterWait) {
        DriveFail("LOGIC_REFUSED", "The logic did not take SETUP, it is in " + world_.state,
                  {{"mower_state", world_.state}}, now, wall);
      }
      return;
    case Drive::kRunUp:
      if (Turning(speed)) {
        EnterDrive(Drive::kTrigger, now, wall);
        if (step.check == "no_command") {
          run.trigger = now;
          EnterDrive(Drive::kStop, now, wall);
        } else if (step.check == "silence") {
          silence_due_ = capabilities_.silence_max_s;
          run.asked = true;
          run.asked_at = now;
        } else if (step.check == "driver_end") {
          end_due_ = true;
          run.asked = true;
          run.asked_at = now;
        }
      } else if (elapsed > kStartWait) {
        DriveFail("NO_START", "The wheels did not turn within " + std::to_string(kStartWait) + " s", json::object(),
                  now, wall);
      }
      return;
    case Drive::kTrigger:
      if (stop_moving) {
        if (TakeTrigger(step.parts[run.part], now)) EnterDrive(Drive::kStop, now, wall);
      } else if (run.asked && now - run.asked_at > kServiceWait) {
        DriveFail("DRIVER_REFUSED", "The driver did not answer", {{"code", "NO_ANSWER"}}, now, wall);
      }
      return;
    case Drive::kStop:
      if (Standing(now)) {
        const double stand = std::max(0.0, still_since_ - run.trigger);
        run.stood = still_since_;
        if (stop_moving) run.measured[run.part] = stand;
        run.stands.push_back(stand);
        EnterDrive(Drive::kQuiet, now, wall);
      } else if (now - run.trigger > step.Number("stop_max")) {
        const std::string id =
            stop_moving ? step.parts[run.part].id : step.check == "silence" ? "round_" + std::to_string(run.round + 1)
                                                                            : "stand";
        DriveFail("LIMIT", "The wheels did not stand within " + std::to_string(step.Number("stop_max")) + " s",
                  {{"value_id", id}, {"value", Rounded(now - run.trigger, 100)}, {"limit", step.Number("stop_max")},
                   {"unit", "s"}},
                  now, wall);
      }
      return;
    case Drive::kReturn:
      if (run.returned >= 0) {
        run.returned_after = run.returned - run.trigger;
        run.stood = run.returned;
        EnterDrive(Drive::kQuiet, now, wall);
      } else if (now - run.trigger > step.Number("return_max")) {
        const std::string shown =
            run.gap ? ", it shows " + mowbite_api::EmergencyNames(static_cast<int>(hardware_.emergency_reason)) : "";
        DriveFail("NO_RETURN",
                  "The driver was not back within " + std::to_string(step.Number("return_max")) + " s" + shown,
                  {{"limit", step.Number("return_max")}}, now, wall);
      }
      return;
    case Drive::kQuiet: {
      if (Started()) {
        DriveFail("STARTED_AGAIN", "A wheel turned after the stop",
                  {{"speed", Rounded(std::max(std::abs(left_), std::abs(right_)), 1000)}}, now, wall);
        return;
      }
      const double until = std::max(run.stood, run.silence_end) + step.Number("hold");
      if (now < until) return;
      if (step.check == "driver_end" && !Standing(now)) {
        DriveFail("STARTED_AGAIN", "The wheels do not stand after the driver came back",
                  {{"speed", Rounded(std::max(std::abs(left_), std::abs(right_)), 1000)}}, now, wall);
        return;
      }
      EnterDrive(step.check == "no_command" ? Drive::kArm : Drive::kLatch, now, wall);
      return;
    }
    case Drive::kLatch:
      if ((hardware_.emergency_reason & run.expected) != 0) {
        // The press of stop_moving and of ground took its own latch at the trigger.
        if (!stop_moving && !IsGround(step)) {
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
      if (Started()) {
        DriveFail("STARTED_AGAIN", "A wheel turned before the release",
                  {{"speed", Rounded(std::max(std::abs(left_), std::abs(right_)), 1000)}}, now, wall);
      } else if (hardware_.emergency_reason == 0) {
        EnterDrive(Drive::kArm, now, wall);
      } else if (!own_ || elapsed > kReleaseWithin) {
        DriveFail("NOT_RELEASED", "The emergency stays: " +
                                      mowbite_api::EmergencyNames(static_cast<int>(hardware_.emergency_reason)),
                  json::object(), now, wall);
      }
      return;
    case Drive::kArm:
      if (Started()) {
        DriveFail("STARTED_AGAIN", "A wheel turned with zero asked for",
                  {{"speed", Rounded(std::max(std::abs(left_), std::abs(right_)), 1000)}}, now, wall);
      } else if (elapsed >= step.Number("hold") && IsGround(step)) {
        // No new command after a stop on the ground.
        NextGroundPart(now, wall);
      } else if (elapsed >= step.Number("hold")) {
        EnterDrive(Drive::kRestart, now, wall);
      }
      return;
    case Drive::kRestart:
      if (Turning(speed)) {
        EnterDrive(Drive::kPause, now, wall);
      } else if (elapsed > kRestartWait) {
        DriveFail("NO_RESTART", "A new command did not turn the wheels within " + std::to_string(kRestartWait) + " s",
                  json::object(), now, wall);
      }
      return;
    case Drive::kPause:
      if (Standing(now) && still_since_ >= run.drive_began) {
        NextPass(now, wall);
      } else if (elapsed > kPauseWait) {
        DriveFail("STARTED_AGAIN", "The wheels did not stand after the zero",
                  {{"speed", Rounded(std::max(std::abs(left_), std::abs(right_)), 1000)}}, now, wall);
      }
      return;
    case Drive::kMeasure:
      return;
    case Drive::kAsk:
      TickPressed(now, wall);
      return;
  }
}

bool Program::TakeTrigger(const Part& part, double now) {
  Run& run = *run_;
  const uint32_t bit = RoleBit(part.role);
  const bool latched = (hardware_.emergency_reason & bit) != 0;
  if (!run.armed) {
    run.armed = !Signal(part.role) && !latched;
    return false;
  }
  if (!Signal(part.role) && !latched) return false;
  run.trigger = now;
  run.expected = bit;
  const auto lines = LinesOf(part.role);
  OwnLatch own;
  own.role = part.role;
  own.bit = bit;
  own.held = std::any_of(lines.begin(), lines.end(), [](const Line* line) { return line->held; });
  own.edge = now;
  own_ = own;
  return true;
}

void Program::TickWheels(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  const double elapsed = now - run.drive_began;
  switch (run.drive) {
    case Drive::kEnter:
      if (world_.state == "SETUP") {
        run.part_states[run.part] = "waiting";
        EnterDrive(Drive::kRunUp, now, wall);
      } else if (elapsed > kEnterWait) {
        DriveFail("LOGIC_REFUSED", "The logic did not take SETUP, it is in " + world_.state,
                  {{"mower_state", world_.state}}, now, wall);
      }
      return;
    case Drive::kRunUp:
      if (elapsed >= kRunUp) {
        run.sum_left = run.sum_right = 0;
        run.samples = 0;
        EnterDrive(Drive::kMeasure, now, wall);
      }
      return;
    case Drive::kMeasure: {
      const Part& part = step.parts[run.part];
      const WheelPart& signs = WheelPartOf(part.id);
      const bool one = signs.left == 0;
      if (one ? run.answer.empty() : elapsed < step.Number("time")) return;
      if (run.samples == 0) {
        DriveFail("NO_DATA", "No hw/wheel_odom while the part drove", {{"source", "hw/wheel_odom"}}, now, wall);
        return;
      }
      const double left = run.sum_left / run.samples;
      const double right = run.sum_right / run.samples;
      const double target = one || signs.left == signs.right ? Speed(step) : TurnRate(step) * track_ / 2;
      const double still = kStillShare * target;
      const double along = one ? right : (signs.left * left + signs.right * right) / 2;
      run.measured[run.part] = along;
      const std::string seen = "left " + std::to_string(left) + " m/s, right " + std::to_string(right) + " m/s";
      if (std::abs(left) < still && std::abs(right) < still) {
        DriveFail("NO_START", "The wheels did not turn: " + seen, json::object(), now, wall);
        return;
      }
      if (one) {
        if (run.answer != "right") {
          DriveFail("WRONG_WHEEL", "The answer names the wheel " + run.answer, {{"answer", run.answer}}, now, wall);
          return;
        }
        if (std::abs(left) > still || right < kSpeedLow * target) {
          DriveFail("WRONG_WHEEL", "Another wheel than the right one turned: " + seen, {{"answer", run.answer}}, now,
                    wall);
          return;
        }
      } else if ((signs.left * left < 0 && std::abs(left) > still) ||
                 (signs.right * right < 0 && std::abs(right) > still)) {
        DriveFail("WRONG_DIRECTION", "A wheel turned the other way: " + seen, json::object(), now, wall);
        return;
      }
      if (along < kSpeedLow * target || along > kSpeedHigh * target) {
        DriveFail("LIMIT", "The wheels turned at " + std::to_string(along) + " m/s, not about " +
                               std::to_string(target),
                  {{"value_id", part.id},
                   {"value", Rounded(along, 1000)},
                   {"limit", Rounded((along < kSpeedLow * target ? kSpeedLow : kSpeedHigh) * target, 1000)},
                   {"unit", "m/s"}},
                  now, wall);
        return;
      }
      run.part_states[run.part] = "ok";
      EnterDrive(Drive::kPause, now, wall);
      return;
    }
    case Drive::kPause:
      if (Standing(now) && still_since_ >= run.drive_began) {
        NextPass(now, wall);
      } else if (elapsed > kPauseWait) {
        DriveFail("STARTED_AGAIN", "The wheels did not stand after the zero",
                  {{"speed", Rounded(std::max(std::abs(left_), std::abs(right_)), 1000)}}, now, wall);
      }
      return;
    default:
      return;
  }
}

void Program::OnSilenceAnswer(bool success, const std::string& code, double now, double wall) {
  if (!run_ || run_->drive != Drive::kTrigger || !run_->asked) return;
  const Step& step = StepOf(*run_);
  if (step.check != "silence" && !IsBlade(step)) return;
  Run& run = *run_;
  run.asked = false;
  if (!success) {
    DriveFail("DRIVER_REFUSED", "hw/silence was refused: " + code, {{"code", code}}, now, wall);
    return;
  }
  run.trigger = now;
  run.silence_end = now + capabilities_.silence_max_s;
  run.expected = BitsOf(step.lists.at("emergency"));
  // The driver drops the off while it is silent: the board alone stops the blade.
  run.blade = 0;
  EnterDrive(Drive::kStop, now, wall);
}

void Program::OnEndAnswer(bool success, const std::string& code, double now, double wall) {
  if (!run_ || StepOf(*run_).check != "driver_end" || run_->drive != Drive::kTrigger || !run_->asked) return;
  Run& run = *run_;
  run.asked = false;
  if (!success) {
    DriveFail("DRIVER_REFUSED", "hw/end was refused: " + code, {{"code", code}}, now, wall);
    return;
  }
  run.trigger = now;
  run.expected = BitsOf(StepOf(run).lists.at("emergency"));
  EnterDrive(Drive::kReturn, now, wall);
}

std::optional<Refusal> Program::AnswerWheel(const Step& step, const json& params) {
  const std::string& name = mowbite_api::SetupWheelQuestion().id;
  if (!IsRun(step) || step.parts[run_->part].id != "one_wheel" || run_->drive != Drive::kMeasure ||
      !run_->answer.empty()) {
    return Refuse("WRONG_METHOD", step.id + " asks nothing right now");
  }
  const json none = json::object();
  const json& values = params.contains("values") && params["values"].is_object() ? params["values"] : none;
  const std::string answer =
      values.size() == 1 && values.contains(name) && values[name].is_string() ? values[name].get<std::string>() : "";
  if (!mowbite_api::FindSetupName(mowbite_api::SetupWheelValues(), answer)) {
    return Refuse("INVALID_VALUE", step.id + " takes " + name + " alone: left, right, both or none", {{"name", name}});
  }
  run_->answer = answer;
  Changed();
  return std::nullopt;
}

std::optional<Refusal> Program::Keep(const json& params, double now, json* result) {
  if (!params.is_object() || !params.contains("step") || !params["step"].is_string()) {
    return BadParam("step", "step is missing or no text");
  }
  const bool running = run_ && StepOf(*run_).id == params["step"].get<std::string>();
  if (running) run_->keep = now;
  *result = {{"running", running}};
  return std::nullopt;
}

std::optional<Motion> Program::MotionNow() const {
  if (!run_ || !IsDrive(StepOf(*run_))) return std::nullopt;
  const Step& step = StepOf(*run_);
  if (IsBlade(step)) {
    return Motion{step.id,
                  false,
                  0,
                  0,
                  run_->blade,
                  static_cast<int>(std::lround(step.Number("percent"))),
                  static_cast<int>(std::lround(step.Number("height"))),
                  BladeLease(step)};
  }
  if (IsGround(step)) {
    return Motion{step.id, false, std::max(Speed(step), step.Number("bumper_speed")), step.Number("turn_rate")};
  }
  return Motion{step.id, true, Speed(step), TurnRate(step)};
}

std::optional<Command> Program::CommandNow() const {
  if (!run_ || !IsDrive(StepOf(*run_))) return std::nullopt;
  return run_->command;
}

std::optional<std::string> Program::TakeMotionEnd() {
  std::optional<std::string> step;
  step.swap(motion_end_);
  return step;
}

std::optional<double> Program::TakeSilence() {
  std::optional<double> seconds;
  seconds.swap(silence_due_);
  return seconds;
}

bool Program::SilenceWithBlade() const {
  return run_ && IsBlade(StepOf(*run_));
}

bool Program::TakeEnd() {
  const bool due = end_due_;
  end_due_ = false;
  return due;
}

json Program::DriveValues(const Run& run, const Step& step, bool final) const {
  if (IsGround(step)) return GroundValues(run, step);
  if (IsBlade(step)) return BladeValues(run, step, final);
  json values = json::array();
  const auto judged = [](const std::string& state) {
    return state == "ok" ? json(true) : state == "failed" ? json(false) : json();
  };
  if (step.check == "wheels") {
    for (size_t i = 0; i < step.parts.size(); ++i) {
      const WheelPart& signs = WheelPartOf(step.parts[i].id);
      const double target =
          signs.left == 0 || signs.left == signs.right ? Speed(step) : TurnRate(step) * track_ / 2;
      const auto& measured = run.measured[i];
      values.push_back(Value(step.parts[i].id, measured ? json(Rounded(*measured, 1000)) : json(), "m/s",
                             Rounded(kSpeedLow * target, 1000), Rounded(kSpeedHigh * target, 1000),
                             judged(run.part_states[i]), step.parts[i].texts));
    }
    return values;
  }
  const double stop_max = step.check == "driver_end" ? 0 : step.Number("stop_max");
  const auto within = [final](std::optional<double> value, double max) {
    return value ? json(*value <= max) : final ? json(false) : json();
  };
  if (step.check == "stop_moving") {
    for (size_t i = 0; i < step.parts.size(); ++i) {
      const auto& measured = run.measured[i];
      values.push_back(Value(step.parts[i].id, measured ? json(Rounded(*measured, 100)) : json(), "s", nullptr,
                             stop_max, measured ? json(*measured <= stop_max) : json(), step.parts[i].texts));
    }
  } else if (step.check == "no_command") {
    const std::optional<double> stand = run.stands.empty() ? std::nullopt : std::optional<double>(run.stands[0]);
    values.push_back(Measured("no_command", "stand", stand ? json(Rounded(*stand, 100)) : json(), nullptr, stop_max,
                              within(stand, stop_max)));
  } else if (step.check == "silence") {
    const int rounds = static_cast<int>(std::lround(step.Number("rounds")));
    for (int r = 0; r < rounds; ++r) {
      const std::optional<double> stand =
          r < static_cast<int>(run.stands.size()) ? std::optional<double>(run.stands[r]) : std::nullopt;
      json value = Measured("silence", "stand", stand ? json(Rounded(*stand, 100)) : json(), nullptr, stop_max,
                            stand ? json(*stand <= stop_max) : json());
      value["id"] = "round_" + std::to_string(r + 1);
      for (auto& language : value["texts"].items()) {
        language.value()["name"] = language.value().value("name", "") + " " + std::to_string(r + 1);
      }
      values.push_back(std::move(value));
    }
    std::optional<double> spread;
    if (run.stands.size() > 1) {
      const auto [low, high] = std::minmax_element(run.stands.begin(), run.stands.end());
      spread = *high - *low;
    }
    values.push_back(Measured("silence", "spread", spread ? json(Rounded(*spread, 100)) : json(), nullptr,
                              step.Number("spread_max"),
                              spread && run.stands.size() == static_cast<size_t>(rounds)
                                  ? json(*spread <= step.Number("spread_max"))
                                  : json()));
  } else if (step.check == "driver_end") {
    values.push_back(Measured("driver_end", "return",
                              run.returned_after ? json(Rounded(*run.returned_after, 100)) : json(), nullptr,
                              step.Number("return_max"), within(run.returned_after, step.Number("return_max"))));
  }
  return values;
}

}  // namespace mowbite_setup
