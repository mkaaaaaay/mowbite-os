#include "mowbite_follower/path_follower.h"

#include <algorithm>
#include <cstdio>
#include <limits>

namespace mowbite_follower {

namespace {

// m before a corner or the end at which the mower counts as there.
constexpr double kStopDistance = 0.01;
// m ahead of the foot point the next foot point may lie: the path never jumps to a later part of itself.
constexpr double kWindow = 0.5;
// m a mower has to move to count as moving, along the path or anywhere.
constexpr double kMoved = 0.05;
// A straight path: every pose within this many m of the line and facing along it within this many rad.
constexpr double kStraightOffset = 0.01;
constexpr double kStraightAngle = 2.0 * M_PI / 180.0;
// s the command of a turn on the spot and the turn of the heading are smoothed over to compare them, and the rad/s a
// step of the heading counts as at most (a heading set anew is no turn).
constexpr double kTurnFilterS = 0.15;
constexpr double kMaxTurnRate = 2.0;

std::string Format(const char* format, double a, double b = 0) {
  char text[200];
  std::snprintf(text, sizeof(text), format, a, b);
  return text;
}

bool Turning(Phase phase) {
  return phase == Phase::kStartTurn || phase == Phase::kCornerTurn || phase == Phase::kRealign ||
         phase == Phase::kGoalTurn;
}

}  // namespace

EffectiveSpeeds Effective(const FollowerConfig& config, double driver_max_linear, double driver_max_angular) {
  auto under = [](double value, double limit) { return limit > 0 && std::isfinite(limit) ? std::min(value, limit) : value; };
  const double linear = under(config.speed.max_linear, driver_max_linear);
  EffectiveSpeeds out;
  out.speed_fast = std::min(config.speed.speed_fast, linear);
  out.speed_slow = std::min(config.speed.speed_slow, linear);
  out.max_angular = under(config.max_angular, driver_max_angular);
  return out;
}

std::string ParamsJson(const std::string& node, const EffectiveSpeeds& speeds) {
  char text[200];
  std::snprintf(text, sizeof(text), "{\"node\":\"%s\",\"params\":{\"speed_fast\":%.6g,\"speed_slow\":%.6g,"
                "\"max_angular\":%.6g}}", node.c_str(), speeds.speed_fast, speeds.speed_slow, speeds.max_angular);
  return text;
}

std::string PathFollower::Start(const std::vector<Pose2>& poses, const std::vector<Kind>& kinds, size_t start_index) {
  phase_ = Phase::kIdle;
  std::string problem = path_.Build(poses, kinds, config_.corner_angle, config_.straight ? 0.0 : config_.min_arc_radius);
  if (!problem.empty()) return problem;
  if (start_index + 1 >= poses.size()) return "start_index must lie before the last pose";
  if (config_.straight) {
    const Pose2& a = path_.pose(0);
    const Pose2& b = path_.pose(path_.size() - 1);
    const double line = std::atan2(b.y - a.y, b.x - a.x);
    const double facing = config_.reverse ? NormalizeAngle(line + M_PI) : line;
    for (size_t i = 0; i < path_.size(); i++) {
      const Pose2& p = path_.pose(i);
      const double side = std::cos(line) * (p.y - a.y) - std::sin(line) * (p.x - a.x);
      const bool along = i + 1 == path_.size() || std::abs(NormalizeAngle(path_.direction(i) - line)) <= kStraightAngle;
      if (std::abs(side) > kStraightOffset || !along || std::abs(NormalizeAngle(p.yaw - facing)) > kStraightAngle) {
        return "a straight path must lie on one line, its poses facing along it";
      }
    }
  }
  plan_ = SpeedPlan(path_, config_.speed);
  failure_kind_ = Failure::kNone;
  failure_.clear();
  begun_ = false;
  joined_ = false;
  segment_ = std::min(path_.Merged(start_index), path_.size() - 2);
  stop_ = path_.NextStop(segment_);
  along_ = path_.s(segment_);
  time_ = 0;
  speed_ = 0;
  phase_ = Phase::kFollowing;
  phase_start_ = 0;
  return "";
}

PathFollower::Command PathFollower::Step(const Pose2& pose, double dt) {
  if (phase_ == Phase::kIdle || phase_ == Phase::kDone || phase_ == Phase::kFailed) return {};
  time_ += std::clamp(dt, 0.0, 0.2);
  step_dt_ = std::clamp(dt, 0.0, 0.2);
  const Pose2 travel = Travel(pose);
  if (!begun_) {
    begun_ = true;
    const Pose2& first = path_.pose(segment_);
    const double distance = std::hypot(pose.x - first.x, pose.y - first.y);
    if (distance > config_.max_start_distance) {
      return Fail(Failure::kStart, Format("the mower is too far from the start of the path: %.2f m, at most %.2f m",
                                          distance, config_.max_start_distance));
    }
    if (config_.straight) {
      const double off = NormalizeAngle(travel.yaw - path_.direction(segment_));
      if (std::abs(off) > config_.corner_angle) {
        return Fail(Failure::kStart, Format("the mower faces %.0f deg off the start of the path, more than %.0f deg, "
                                            "and a straight path does not turn",
                                            std::abs(off) * 180.0 / M_PI, config_.corner_angle * 180.0 / M_PI));
      }
      Enter(Phase::kFollowing);
    } else {
      const double off = NormalizeAngle(travel.yaw - DesiredCourse(config_.steering, Foot(pose)));
      Enter(std::abs(off) > config_.corner_angle ? Phase::kStartTurn : Phase::kFollowing);
    }
  }
  if (phase_ == Phase::kGoalTurn) {
    if (auto turn = TurnTowards(pose.yaw, path_.pose(path_.size() - 1).yaw)) return *turn;
    Enter(Phase::kDone);
    return {};
  }
  if (Turning(phase_)) {
    if (auto turn = TurnTowards(travel.yaw, DesiredCourse(config_.steering, Foot(pose)))) return *turn;
    Enter(Phase::kFollowing);
  }
  return Follow(pose, travel, dt);
}

PathFollower::Command PathFollower::Follow(const Pose2& pose, const Pose2& travel, double dt) {
  const Projection foot = Foot(pose);
  segment_ = foot.segment;
  along_ = std::max(along_, foot.along);
  if (!joined_ && std::abs(foot.cross) <= config_.steering.approach_distance) joined_ = true;
  if (CheckMoving(pose, foot)) return {};

  const size_t last = path_.size() - 1;
  const double remaining = path_.s(stop_) - foot.along;
  if (stop_ == last && config_.end_tolerance > 0) {
    if (remaining <= config_.end_tolerance) {
      Enter(Phase::kDone);
      return {};
    }
  } else if (remaining <= kStopDistance) {
    if (stop_ != last) {
      // On the corner: from now on the path goes on after it.
      segment_ = stop_;
      along_ = path_.s(stop_);
      stop_ = path_.NextStop(stop_);
      Enter(Phase::kCornerTurn);
      if (auto turn = TurnTowards(travel.yaw, DesiredCourse(config_.steering, Foot(pose)))) return *turn;
      Enter(Phase::kFollowing);
      return Follow(pose, travel, dt);
    }
    const Pose2& goal = path_.pose(last);
    const double distance = std::hypot(pose.x - goal.x, pose.y - goal.y);
    if (distance > config_.goal_tolerance) {
      return Fail(Failure::kTimeout, Format("the mower got to the end of the path %.2f m from its last pose, more than "
                                            "%.2f m",
                                            distance, config_.goal_tolerance));
    }
    if (config_.straight) {
      Enter(Phase::kDone);
      return {};
    }
    Enter(Phase::kGoalTurn);
    if (auto turn = TurnTowards(pose.yaw, goal.yaw)) return *turn;
    Enter(Phase::kDone);
    return {};
  }

  const double to_next = path_.s(foot.segment + 1) - foot.along;
  double speed = std::min(plan_.At(foot.segment, to_next), speed_ + config_.speed.acceleration * dt);
  speed = std::max(speed, plan_.Floor(foot.segment));
  double angular = 0;
  if (!config_.straight) {
    const double error = NormalizeAngle(travel.yaw - DesiredCourse(config_.steering, foot));
    if (std::abs(error) > config_.max_drive_angle) {
      Enter(Phase::kRealign);
      if (auto turn = TurnTowards(travel.yaw, DesiredCourse(config_.steering, foot))) return *turn;
      Enter(Phase::kFollowing);
    }
    angular = TurnRate(config_.steering, foot, travel.yaw, speed);
    // Too sharp for max_angular: slower on the same curve.
    if (std::abs(angular) > config_.max_angular) {
      speed *= config_.max_angular / std::abs(angular);
      angular = std::copysign(config_.max_angular, angular);
    }
  }
  speed_ = speed;
  return FromTravel(speed, angular);
}

void PathFollower::Cancel() {
  phase_ = Phase::kIdle;
  speed_ = 0;
}

size_t PathFollower::index() const {
  if (path_.size() == 0) return 0;
  return path_.source(phase_ == Phase::kDone ? path_.size() - 1 : segment_);
}

bool PathFollower::OnPath() const {
  return joined_ && (phase_ == Phase::kFollowing || phase_ == Phase::kCornerTurn || phase_ == Phase::kRealign ||
                     phase_ == Phase::kGoalTurn);
}

double PathFollower::CrossTrack(const Pose2& pose) const {
  if (path_.size() == 0) return 0.0;
  return Foot(pose).cross;
}

double PathFollower::OffPath(const Pose2& pose) const {
  if (path_.size() == 0) return 0.0;
  return path_.Project(pose.x, pose.y, 0, path_.size()).distance;
}

Projection PathFollower::Foot(const Pose2& pose) const {
  size_t last = segment_;
  const double reach = std::max(along_, path_.s(segment_)) + kWindow;
  while (last + 1 < stop_ && path_.s(last + 1) <= reach) last++;
  return path_.Project(pose.x, pose.y, segment_, last);
}

Pose2 PathFollower::Travel(const Pose2& pose) const {
  if (!config_.reverse) return pose;
  return {pose.x, pose.y, NormalizeAngle(pose.yaw + M_PI)};
}

PathFollower::Command PathFollower::Fail(Failure kind, const std::string& message) {
  phase_ = Phase::kFailed;
  failure_kind_ = kind;
  failure_ = message;
  speed_ = 0;
  return {};
}

std::optional<PathFollower::Command> PathFollower::TurnTowards(double heading, double target) {
  const double error = NormalizeAngle(target - heading);
  if (std::abs(error) <= config_.turn_tolerance) return std::nullopt;
  if (time_ - phase_start_ > config_.phase_timeout) {
    return Fail(Failure::kTimeout, Format("turning on the spot took longer than %.0f s", config_.phase_timeout));
  }
  const double cap = config_.spot_max_angular > 0 ? config_.spot_max_angular : config_.max_angular;
  const double slowing = config_.turn_deceleration > 0 ? std::sqrt(2.0 * config_.turn_deceleration * std::abs(error))
                                                       : config_.steering.heading_gain * std::abs(error);
  const double sign = error > 0 ? 1.0 : -1.0;
  const double planned = std::clamp(slowing, std::min(config_.min_turn_rate, cap), cap);
  const double rate = std::min(planned + TurnBoost(heading, sign), cap);
  turn_command_ = sign * rate;
  return Command{0.0, turn_command_};
}

double PathFollower::TurnBoost(double heading, double sign) {
  if (turn_heading_ && step_dt_ > 0) {
    // The last command and the turn it made, through the same filter: a mower that follows shows no difference.
    const double share = std::min(1.0, step_dt_ / kTurnFilterS);
    const double turned = std::clamp(NormalizeAngle(heading - *turn_heading_) / step_dt_, -kMaxTurnRate, kMaxTurnRate);
    turn_expected_ += (turn_command_ - turn_expected_) * share;
    turn_measured_ += (turned - turn_measured_) * share;
  }
  turn_heading_ = heading;
  if (!(config_.turn_feedback > 0) || !(config_.turn_feedback_max > 0)) return 0.0;
  const double missing = std::max(0.0, sign * (turn_expected_ - turn_measured_));
  return std::min(config_.turn_feedback * missing, config_.turn_feedback_max);
}

void PathFollower::Enter(Phase phase) {
  if (phase == Phase::kFollowing && phase_ != Phase::kRealign) {
    // A planned turn on the spot is no lack of progress; a realignment is.
    progress_along_ = -std::numeric_limits<double>::infinity();
  }
  if (phase == Phase::kFollowing || Turning(phase)) still_at_.reset();
  if (Turning(phase)) speed_ = 0;
  turn_heading_.reset();
  turn_command_ = turn_expected_ = turn_measured_ = 0;
  phase_ = phase;
  phase_start_ = time_;
}

PathFollower::Command PathFollower::FromTravel(double speed, double angular) const {
  return {config_.reverse ? -speed : speed, angular};
}

bool PathFollower::CheckMoving(const Pose2& pose, const Projection& foot) {
  if (config_.stall_time > 0) {
    if (!still_at_ || std::hypot(pose.x - still_at_->x, pose.y - still_at_->y) >= kMoved) {
      still_at_ = pose;
      still_since_ = time_;
    } else if (time_ - still_since_ >= config_.stall_time) {
      Fail(Failure::kTimeout, Format("the mower does not move: it stood %.1f s while driven", time_ - still_since_));
      return true;
    }
    return false;
  }
  if (foot.along >= progress_along_ + kMoved) {
    progress_along_ = foot.along;
    progress_since_ = time_;
  } else if (time_ - progress_since_ >= config_.progress_timeout) {
    Fail(Failure::kTimeout, Format("no progress along the path for %.0f s", time_ - progress_since_));
    return true;
  }
  return false;
}

}  // namespace mowbite_follower
