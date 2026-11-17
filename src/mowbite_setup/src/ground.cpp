// The step ground: the mower on the ground drives its parts in one run. straight and turn go by themselves, stop and
// bumper wait for the input the user triggers; after a stop the release goes on as in stop_moving, without a new
// command. mowbite_safety holds setup/cmd_vel to its bounds on the ground, without pose and fence.
#include <algorithm>
#include <cmath>

#include "mowbite_api/setup.h"
#include "mowbite_setup/program.h"
#include "program_util.h"

namespace mowbite_setup {

using namespace util;

namespace {

// The mower stands within this time of a stop, else NO_STANDSTILL (s); the catalog names it.
constexpr double kStandWait = 3.0;
// straight and each side of turn drive at most this long, below the bound of a single move (s).
constexpr double kAutoMove = 8.0;
constexpr double kRadPerDeg = M_PI / 180;
constexpr int kFixed = 4;

}  // namespace

bool Program::IsGround(const Step& step) {
  return step.check == "ground";
}

bool Program::GroundPress() const {
  if (!run_ || !IsGround(StepOf(*run_))) return false;
  const std::string& id = StepOf(*run_).parts[run_->part].id;
  return id == "stop" || id == "bumper";
}

Command Program::GroundCommand() const {
  const Run& run = *run_;
  const Step& step = StepOf(run);
  const std::string& id = step.parts[run.part].id;
  if (id == "straight") return Command{Speed(step), 0};
  if (id == "turn") {
    const double rate = step.Number("turn_rate");
    return Command{0, run.side == 0 ? rate : -rate};
  }
  if (id == "stop") return Command{-Speed(step), 0};
  const double sign = step.Word("bumper_direction") == "backward" ? -1 : 1;
  return Command{sign * step.Number("bumper_speed"), 0};
}

std::vector<Refusal> Program::GroundProblems(double now) const {
  std::vector<Refusal> problems;
  if (solution_ != kFixed || fixes_.empty() || !Fresh(fixes_.back(), now, kFresh)) {
    problems.push_back(Refuse("NO_RTK", "No RTK fix: the step compares its way with the GNSS"));
  }
  if (LockOpen()) return problems;
  // Second rule of the program: with the lock closed the mower drives on the ground only after every step jacked up
  // passed; skipped does not count.
  json missing = json::array();
  for (const Step& other : list_.steps) {
    const bool raised = IsDrive(other) && !IsGround(other) && !IsBlade(other);
    if (raised && records_.at(other.id).state != "ok") missing.push_back(other.id);
  }
  if (!missing.empty()) {
    problems.push_back(Refuse("SAFETY_AIR", "The lock is closed and the steps jacked up are not all passed: " +
                                                missing.dump(),
                              {{"steps", missing}}));
  }
  return problems;
}

void Program::BeginGroundPart(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  run.part_states[run.part] = "waiting";
  run.part_began = now;
  run.part_wall = wall;
  run.trigger = run.stood = -1;
  run.expected = 0;
  run.armed = false;
  run.way = run.travel = 0;
  run.yaw_odom = run.yaw_imu = 0;
  run.side = 0;
  run.last_imu = -1;
  if (step.parts[run.part].id == "straight") {
    if (solution_ != kFixed || fixes_.empty() || !Fresh(fixes_.back(), now, kFresh)) {
      DriveFail("NO_RTK", "No RTK fix where straight begins", json::object(), now, wall);
      return;
    }
    run.fix_x = fix_x_;
    run.fix_y = fix_y_;
  }
  EnterDrive(Drive::kRunUp, now, wall);
}

void Program::NextGroundPart(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  run.part_states[run.part] = "ok";
  if (run.part + 1 >= step.parts.size()) {
    Pass(step, now, wall);
    return;
  }
  ++run.part;
  BeginGroundPart(now, wall);
}

bool Program::GroundStood(double max_time, double max_way, double now, double wall) {
  Run& run = *run_;
  const std::string& id = StepOf(run).parts[run.part].id;
  if (!Standing(now)) {
    if (now - run.trigger > kStandWait) {
      DriveFail("NO_STANDSTILL",
                "The mower did not stand within " + std::to_string(kStandWait) + " s, it went " +
                    std::to_string(run.way) + " m",
                {{"value", Rounded(run.way, 100)}}, now, wall);
    }
    return false;
  }
  const double stand = std::max(0.0, still_since_ - run.trigger);
  run.ground[id + "_stand"] = stand;
  run.ground[id + "_way"] = run.way;
  if (stand > max_time) {
    DriveFail("LIMIT", "The mower stood " + std::to_string(stand) + " s after the stop",
              {{"value_id", id + "_stand"}, {"value", Rounded(stand, 100)}, {"limit", max_time}, {"unit", "s"}}, now,
              wall);
    return false;
  }
  if (run.way > max_way) {
    DriveFail("LIMIT", "The mower went " + std::to_string(run.way) + " m after the stop",
              {{"value_id", id + "_way"}, {"value", Rounded(run.way, 100)}, {"limit", max_way}, {"unit", "m"}}, now,
              wall);
    return false;
  }
  run.stood = still_since_;
  return true;
}

void Program::GroundStraightDone(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  if (solution_ != kFixed || fixes_.empty() || !Fresh(fixes_.back(), now, kFresh)) {
    DriveFail("NO_RTK", "The RTK fix was lost while straight drove", json::object(), now, wall);
    return;
  }
  run.straight_way = run.travel;
  const double gnss = std::hypot(fix_x_ - run.fix_x, fix_y_ - run.fix_y);
  const double ratio = run.travel > 0 ? gnss / run.travel : 0;
  run.ground["straight_gnss"] = ratio;
  const double r = step.Number("ratio");
  if (std::abs(ratio - 1) > r) {
    DriveFail("LIMIT",
              "The way by GNSS was " + std::to_string(gnss) + " m, by odometry " + std::to_string(run.travel) + " m",
              {{"value_id", "straight_gnss"},
               {"value", Rounded(ratio, 100)},
               {"limit", ratio < 1 ? 1 - r : 1 + r},
               {"unit", ""}},
              now, wall);
    return;
  }
  NextGroundPart(now, wall);
}

void Program::GroundTurnDone(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  const double sign = run.side == 0 ? 1 : -1;
  const std::string id = run.side == 0 ? "turn_left" : "turn_right";
  const std::string seen = "odometry " + std::to_string(run.yaw_odom / kRadPerDeg) + " deg, IMU " +
                           std::to_string(run.yaw_imu / kRadPerDeg) + " deg";
  if (run.yaw_odom * sign <= 0 || run.yaw_imu * sign <= 0) {
    DriveFail("WRONG_DIRECTION", "The mower turned the other way: " + seen, json::object(), now, wall);
    return;
  }
  const double ratio = run.yaw_imu / run.yaw_odom;
  run.ground[id] = ratio;
  const double r = step.Number("ratio");
  if (std::abs(ratio - 1) > r) {
    DriveFail("LIMIT", "The turn differs: " + seen,
              {{"value_id", id}, {"value", Rounded(ratio, 100)}, {"limit", ratio < 1 ? 1 - r : 1 + r}, {"unit", ""}},
              now, wall);
    return;
  }
  if (run.side == 1) {
    NextGroundPart(now, wall);
    return;
  }
  run.side = 1;
  run.yaw_odom = run.yaw_imu = 0;
  EnterDrive(Drive::kRunUp, now, wall);
}

void Program::TickGround(double now, double wall) {
  Run& run = *run_;
  const Step& step = StepOf(run);
  const Part& part = step.parts[run.part];
  const double elapsed = now - run.drive_began;
  const bool press = GroundPress();
  // stop and bumper: the user did not act within time, or stop came back to where straight began. Before the bound
  // of a single move, which is no measurement.
  if (press && (run.drive == Drive::kRunUp || run.drive == Drive::kTrigger) && run.move_began >= 0) {
    const bool late = now - run.move_began > step.Number("time");
    const bool back = part.id == "stop" && -run.travel >= run.straight_way;
    if (late || back) {
      AskPressed("NOT_SEEN", json::object(), now, wall);
      return;
    }
  }
  const bool moves = run.drive == Drive::kRunUp || run.drive == Drive::kMeasure || run.drive == Drive::kTrigger;
  if (moves && run.move_began >= 0 && now - run.move_began > mowbite_api::kSetupMoveTime) {
    DriveFail("LIMIT", "A move lasted longer than " + std::to_string(mowbite_api::kSetupMoveTime) + " s",
              {{"limit", mowbite_api::kSetupMoveTime}, {"unit", "s"}}, now, wall);
    return;
  }
  const Command command = GroundCommand();
  const double wheel = part.id == "turn"     ? step.Number("turn_rate") * track_ / 2
                       : part.id == "bumper" ? step.Number("bumper_speed")
                                             : Speed(step);
  switch (run.drive) {
    case Drive::kEnter:
      if (world_.state == "SETUP") {
        BeginGroundPart(now, wall);
      } else if (elapsed > kEnterWait) {
        DriveFail("LOGIC_REFUSED", "The logic did not take SETUP, it is in " + world_.state,
                  {{"mower_state", world_.state}}, now, wall);
      }
      return;
    case Drive::kRunUp: {
      if (press && TakeTrigger(part, now)) {
        run.way = 0;
        EnterDrive(Drive::kStop, now, wall);
        return;
      }
      const double along = (left_filtered_ + right_filtered_) / 2;
      const double spin = (right_filtered_ - left_filtered_) / 2;
      const double moved = command.linear != 0 ? along * command.linear : spin * command.angular;
      if (moved < 0 && std::max(std::abs(along), std::abs(spin)) >= 0.5 * wheel) {
        DriveFail("WRONG_DIRECTION",
                  "The wheels turned the other way: left " + std::to_string(left_) + " m/s, right " +
                      std::to_string(right_) + " m/s",
                  json::object(), now, wall);
      } else if (Turning(wheel)) {
        EnterDrive(press ? Drive::kTrigger : Drive::kMeasure, now, wall);
      } else if (elapsed > kStartWait) {
        DriveFail("NO_START", "The mower did not move within " + std::to_string(kStartWait) + " s", json::object(),
                  now, wall);
      }
      return;
    }
    case Drive::kMeasure:
      if (part.id == "straight") {
        if (run.travel < step.Number("distance") && now - run.move_began <= kAutoMove) return;
        // The program falls silent: the mower has to stand by itself.
        run.trigger = now;
        run.way = 0;
        EnterDrive(Drive::kStop, now, wall);
      } else if (std::abs(run.yaw_odom) >= step.Number("angle") * kRadPerDeg || now - run.move_began > kAutoMove) {
        run.trigger = now;
        EnterDrive(Drive::kPause, now, wall);
      }
      return;
    case Drive::kTrigger:
      if (TakeTrigger(part, now)) {
        run.way = 0;
        EnterDrive(Drive::kStop, now, wall);
      }
      return;
    case Drive::kStop: {
      const bool straight = part.id == "straight";
      const bool stood = straight ? GroundStood(step.Number("quiet_max"), step.Number("quiet_way"), now, wall)
                                  : GroundStood(step.Number("stop_max"), step.Number("stop_way"), now, wall);
      if (stood) EnterDrive(Drive::kQuiet, now, wall);
      return;
    }
    case Drive::kQuiet:
      if (Started()) {
        DriveFail("STARTED_AGAIN", "A wheel turned after the stop",
                  {{"speed", Rounded(std::max(std::abs(left_), std::abs(right_)), 1000)}}, now, wall);
      } else if (now >= run.stood + step.Number("hold")) {
        if (part.id == "straight") {
          GroundStraightDone(now, wall);
        } else {
          EnterDrive(Drive::kLatch, now, wall);
        }
      }
      return;
    case Drive::kPause:
      if (Standing(now) && still_since_ >= run.drive_began) {
        GroundTurnDone(now, wall);
      } else if (elapsed > kStandWait) {
        DriveFail("NO_STANDSTILL", "The mower did not stand within " + std::to_string(kStandWait) + " s of the zero",
                  json::object(), now, wall);
      }
      return;
    default:
      return;
  }
}

json Program::GroundValues(const Run& run, const Step& step) const {
  const double r = step.Number("ratio");
  json values = json::array();
  // The sides of turn, so that its two values do not read alike.
  const json sides = {{"turn_left", {{"de", " links"}, {"en", " left"}}},
                      {"turn_right", {{"de", " rechts"}, {"en", " right"}}}};
  const auto add = [&](const Part& part, const char* measure, const std::string& id, json min, json max) {
    const auto found = run.ground.find(id);
    json value = nullptr;
    json ok = nullptr;
    if (found != run.ground.end()) {
      value = Rounded(found->second, 100);
      const double v = found->second;
      ok = (min.is_null() || v >= min.get<double>()) && (max.is_null() || v <= max.get<double>());
    }
    json entry = Measured("ground", measure, value, std::move(min), std::move(max), ok);
    entry["id"] = id;
    for (auto& language : entry["texts"].items()) {
      const json& named = part.texts.contains(language.key()) ? part.texts[language.key()] : json::object();
      const std::string side = sides.contains(id) ? sides[id].value(language.key(), "") : "";
      language.value()["name"] = named.value("name", part.id) + side + ": " + language.value().value("name", "");
    }
    values.push_back(std::move(entry));
  };
  for (const Part& part : step.parts) {
    if (part.id == "straight") {
      add(part, "stand", "straight_stand", nullptr, step.Number("quiet_max"));
      add(part, "way", "straight_way", nullptr, step.Number("quiet_way"));
      add(part, "gnss", "straight_gnss", 1 - r, 1 + r);
    } else if (part.id == "turn") {
      add(part, "turn", "turn_left", 1 - r, 1 + r);
      add(part, "turn", "turn_right", 1 - r, 1 + r);
    } else {
      add(part, "stand", part.id + "_stand", nullptr, step.Number("stop_max"));
      add(part, "way", part.id + "_way", nullptr, step.Number("stop_way"));
    }
  }
  return values;
}

}  // namespace mowbite_setup
