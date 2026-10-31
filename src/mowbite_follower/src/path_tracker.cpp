// Based on ftc_local_planner (FTCPlanner) from open_mower_ros, BSD License.
#include "mowbite_follower/path_tracker.h"

#include <algorithm>
#include <cstdio>
#include <limits>

namespace mowbite_follower {

double NormalizeAngle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0) a += 2.0 * M_PI;
  return a - M_PI;
}

const char* PhaseName(Phase phase) {
  switch (phase) {
    case Phase::kIdle: return "idle";
    case Phase::kPreRotate: return "turning to the path";
    case Phase::kFollowing: return "following";
    case Phase::kCornerApproach: return "approaching a corner";
    case Phase::kCornerRotate: return "turning on a corner";
    case Phase::kGoalApproach: return "approaching the goal";
    case Phase::kPostRotate: return "turning at the goal";
    case Phase::kDone: return "done";
    case Phase::kFailed:
    default: return "failed";
  }
}

namespace {

double Clamp(double v, double limit) {
  return std::clamp(v, -limit, limit);
}

// A straight path: every pose within this (m) of the line of the first one and facing along it within kStraightAngle.
constexpr double kStraightOffset = 0.01;
constexpr double kStraightAngle = 1.0 * M_PI / 180.0;

std::string Format(const char* format, double a, double b, double c) {
  char text[160];
  std::snprintf(text, sizeof(text), format, a, b, c);
  return text;
}

// Signed distance of p to the segment a-b, left of a->b positive, and how far along it the closest point lies (0..1).
double SegmentDistance(const Pose2& a, const Pose2& b, const Pose2& p, double* along) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  double t = len2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0;
  t = std::clamp(t, 0.0, 1.0);
  if (along) *along = t;
  const double cx = a.x + t * dx - p.x;
  const double cy = a.y + t * dy - p.y;
  const double d = std::hypot(cx, cy);
  const double side = dx * (p.y - a.y) - dy * (p.x - a.x);
  return side < 0 ? -d : d;
}

}  // namespace

std::string PathTracker::Start(const std::vector<Pose2>& poses, const std::vector<Kind>& kinds, size_t start_index) {
  poses_.clear();
  kinds_.clear();
  phase_ = Phase::kIdle;
  failure_.clear();
  if (poses.size() < 3) return "a path needs at least 3 poses";
  if (!kinds.empty() && kinds.size() != poses.size()) return "kinds must be empty or one per pose";
  if (start_index + 1 >= poses.size()) return "start_index must lie before the last pose";
  for (size_t i = 0; i < poses.size(); i++) {
    const auto& p = poses[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw)) {
      return "pose " + std::to_string(i) + " is not finite";
    }
    if (i > 0 && std::hypot(p.x - poses[i - 1].x, p.y - poses[i - 1].y) < 1e-6) {
      return "pose " + std::to_string(i) + " lies on the one before";
    }
  }
  if (config_.straight) {
    const double c = std::cos(poses[0].yaw);
    const double s = std::sin(poses[0].yaw);
    double along_before = 0;
    for (size_t i = 1; i < poses.size(); i++) {
      const double dx = poses[i].x - poses[0].x;
      const double dy = poses[i].y - poses[0].y;
      const double along = c * dx + s * dy;
      if (std::abs(c * dy - s * dx) > kStraightOffset || along <= along_before ||
          std::abs(NormalizeAngle(poses[i].yaw - poses[0].yaw)) > kStraightAngle) {
        return "a straight path must lie on one line, its poses facing along it";
      }
      along_before = along;
    }
  }
  poses_ = poses;
  kinds_ = kinds.empty() ? std::vector<Kind>(poses.size(), Kind::kTransit) : kinds;
  index_ = start_index;
  progress_ = 0;
  speed_ = 0;
  control_ = poses_[index_];
  i_lon_ = i_lat_ = i_angle_ = 0;
  first_step_ = true;
  corner_done_ = SIZE_MAX;
  stall_anchored_ = false;
  stall_s_ = 0;
  SetPhase(Phase::kPreRotate);
  return "";
}

void PathTracker::Cancel() {
  if (phase_ != Phase::kDone && phase_ != Phase::kFailed) SetPhase(Phase::kIdle);
}

void PathTracker::SetPhase(Phase phase) {
  phase_ = phase;
  phase_time_ = 0;
  // A new phase has a new control law: no derivative across the change.
  first_step_ = true;
}

void PathTracker::Fail(const std::string& why) {
  failure_ = why;
  SetPhase(Phase::kFailed);
}

bool PathTracker::SpinAt(size_t i) const {
  return i > 0 && i < poses_.size() && std::abs(NormalizeAngle(poses_[i].yaw - poses_[i - 1].yaw)) > config_.spin_angle;
}

double PathTracker::ArrivalYaw(size_t i) const {
  return SpinAt(i) ? poses_[i - 1].yaw : poses_[i].yaw;
}

double PathTracker::StraightAhead() const {
  double distance = 0;
  Pose2 last = control_;
  for (size_t i = index_ + 1; i < poses_.size(); i++) {
    if (distance > config_.fast_distance) break;
    if (std::abs(NormalizeAngle(poses_[i].yaw - control_.yaw)) > config_.fast_angle) break;
    distance += std::hypot(poses_[i].x - last.x, poses_[i].y - last.y);
    last = poses_[i];
  }
  return distance;
}

void PathTracker::AdvanceControlPoint(double dt) {
  double target = StraightAhead() >= config_.fast_distance ? config_.speed_fast : config_.speed_slow;
  if (kinds_[index_] == Kind::kTurn) target = std::min(target, config_.speed_turn);
  if (target > speed_) {
    speed_ = std::min(target, speed_ + dt * config_.acceleration);
  } else {
    speed_ = std::max(target, speed_ - dt * config_.acceleration);
  }
  double distance = dt * speed_;
  double angle = dt * config_.turn_rate;
  while (distance > 0 && angle > 0 && index_ + 1 < poses_.size()) {
    const Pose2& a = poses_[index_];
    const Pose2& b = poses_[index_ + 1];
    const double d = std::hypot(b.x - a.x, b.y - a.y);
    // Towards a turn on the spot the heading stays the one before it.
    const double turn = SpinAt(index_ + 1) ? 0.0 : std::abs(NormalizeAngle(b.yaw - a.yaw));
    const double rest_d = d * (1.0 - progress_);
    const double rest_turn = turn * (1.0 - progress_);
    if (rest_d < distance && rest_turn < angle) {
      index_++;
      progress_ = 0;
      distance -= rest_d;
      angle -= rest_turn;
      // A turn on the spot: the control point waits on it until the mower got there and turned.
      if (index_ + 1 < poses_.size() && corner_done_ != index_ && SpinAt(index_)) {
        control_ = poses_[index_];
        control_.yaw = ArrivalYaw(index_);
        SetPhase(Phase::kCornerApproach);
        return;
      }
      continue;
    }
    const double by_distance = (d * progress_ + distance) / d;
    const double by_angle = turn > 0 ? (turn * progress_ + angle) / turn : std::numeric_limits<double>::infinity();
    progress_ = std::min({by_distance, by_angle, 1.0});
    break;
  }
  if (index_ + 1 >= poses_.size()) {
    control_ = poses_.back();
    return;
  }
  const Pose2& a = poses_[index_];
  const Pose2& b = poses_[index_ + 1];
  control_.x = a.x + progress_ * (b.x - a.x);
  control_.y = a.y + progress_ * (b.y - a.y);
  control_.yaw = SpinAt(index_ + 1) ? a.yaw : NormalizeAngle(a.yaw + progress_ * NormalizeAngle(b.yaw - a.yaw));
}

void PathTracker::CheckStall(const Pose2& pose, double linear, double dt) {
  if (config_.stall_time <= 0) return;
  const bool driven = std::abs(linear) >= config_.stall_speed;
  const bool moved = stall_anchored_ && std::hypot(pose.x - stall_anchor_.x, pose.y - stall_anchor_.y) >=
                                            config_.stall_distance;
  if (!driven || !stall_anchored_ || moved) {
    stall_anchored_ = driven;
    stall_anchor_ = pose;
    stall_s_ = 0;
    return;
  }
  stall_s_ += dt;
  if (stall_s_ < config_.stall_time) return;
  Fail(Format("the mower does not move: %.2f m in %.1f s while driven at %.2f m/s, stuck?",
              std::hypot(pose.x - stall_anchor_.x, pose.y - stall_anchor_.y), stall_s_, std::abs(linear)));
}

double PathTracker::CrossTrack(const Pose2& pose) const {
  if (poses_.size() < 2) return 0;
  // The segments a few poses around the control point: the mower lags behind it a little.
  const size_t from = index_ > 5 ? index_ - 5 : 0;
  const size_t to = std::min(poses_.size() - 1, index_ + 5);
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = from; i < to; i++) {
    const double d = SegmentDistance(poses_[i], poses_[i + 1], pose, nullptr);
    if (std::abs(d) < std::abs(best)) best = d;
  }
  return std::isfinite(best) ? best : 0;
}

PathTracker::Command PathTracker::Step(const Pose2& pose, double dt) {
  Command cmd;
  if (phase_ == Phase::kIdle || phase_ == Phase::kDone || phase_ == Phase::kFailed) return cmd;
  if (!(dt > 0)) dt = 1e-3;
  phase_time_ += dt;

  switch (phase_) {
    case Phase::kPreRotate:
      if (std::hypot(poses_[index_].x - pose.x, poses_[index_].y - pose.y) > config_.max_follow_distance) {
        Fail("the mower is too far from the start of the path");
        return cmd;
      }
      if (phase_time_ > config_.phase_timeout) {
        Fail("turning to the path took too long");
        return cmd;
      }
      control_ = poses_[index_];
      break;
    case Phase::kFollowing:
      AdvanceControlPoint(dt);
      if (phase_ == Phase::kFollowing && index_ + 1 >= poses_.size()) SetPhase(Phase::kGoalApproach);
      break;
    case Phase::kCornerApproach:
      if (phase_time_ > config_.phase_timeout) {
        Fail("reaching a corner of the path took too long");
        return cmd;
      }
      control_ = poses_[index_];
      control_.yaw = ArrivalYaw(index_);
      break;
    case Phase::kCornerRotate:
      if (phase_time_ > config_.phase_timeout) {
        Fail("turning on a corner of the path took too long");
        return cmd;
      }
      control_ = poses_[index_];
      break;
    case Phase::kGoalApproach:
      if (phase_time_ > config_.phase_timeout) {
        Fail("the goal was not reached in time");
        return cmd;
      }
      control_ = poses_.back();
      control_.yaw = ArrivalYaw(poses_.size() - 1);
      break;
    case Phase::kPostRotate:
      if (phase_time_ > config_.phase_timeout) {
        Fail("turning at the goal took too long");
        return cmd;
      }
      control_ = poses_.back();
      break;
    default: break;
  }

  // The control point seen from base_link.
  const double dx = control_.x - pose.x;
  const double dy = control_.y - pose.y;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  lon_error_ = c * dx + s * dy;
  lat_error_ = -s * dx + c * dy;
  angle_error_ = NormalizeAngle(control_.yaw - pose.yaw);

  const double distance = std::hypot(dx, dy);
  if (phase_ == Phase::kFollowing && distance > config_.max_follow_distance) {
    Fail("the mower stays behind the path (" + std::to_string(distance).substr(0, 4) + " m), stuck?");
    return Command();
  }
  if (config_.straight) {
    if (phase_ == Phase::kPreRotate && !(std::abs(angle_error_) < config_.max_goal_angle_error)) {
      char text[160];
      std::snprintf(text, sizeof(text),
                    "the mower faces %.0f deg off the start of the path, which it drives straight (limit %.0f deg)",
                    std::abs(angle_error_) * 180.0 / M_PI, config_.max_goal_angle_error * 180.0 / M_PI);
      Fail(text);
      return Command();
    }
    // Along the line, wherever the mower is across it: it does not steer back onto it.
    const Pose2& a = poses_.front();
    const Pose2& b = poses_.back();
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double along = ((pose.x - a.x) * (b.x - a.x) + (pose.y - a.y) * (b.y - a.y)) / length;
    if ((phase_ == Phase::kFollowing || phase_ == Phase::kGoalApproach) &&
        along >= length - config_.max_goal_distance_error) {
      SetPhase(Phase::kDone);
      return Command();
    }
  }
  if (phase_ == Phase::kPreRotate && std::abs(angle_error_) < config_.max_goal_angle_error) {
    SetPhase(Phase::kFollowing);
  } else if (phase_ == Phase::kGoalApproach && distance < config_.max_goal_distance_error) {
    SetPhase(Phase::kPostRotate);
  } else if (phase_ == Phase::kCornerApproach && distance < config_.max_goal_distance_error) {
    SetPhase(Phase::kCornerRotate);
    return Command();
  } else if (phase_ == Phase::kCornerRotate && std::abs(angle_error_) < config_.max_goal_angle_error) {
    corner_done_ = index_;
    // The mower stood still: the control point starts from standstill too.
    speed_ = 0;
    SetPhase(Phase::kFollowing);
    return Command();
  }
  if (phase_ == Phase::kPostRotate && std::abs(angle_error_) < config_.max_goal_angle_error) {
    SetPhase(Phase::kDone);
    return Command();
  }

  if (first_step_) {
    last_lon_error_ = lon_error_;
    last_lat_error_ = lat_error_;
    last_angle_error_ = angle_error_;
    first_step_ = false;
  }
  i_lon_ = Clamp(i_lon_ + lon_error_ * dt, config_.ki_lon_max);
  i_lat_ = Clamp(i_lat_ + lat_error_ * dt, config_.ki_lat_max);
  i_angle_ = Clamp(i_angle_ + angle_error_ * dt, config_.ki_ang_max);
  const double d_lon = (lon_error_ - last_lon_error_) / dt;
  const double d_lat = (lat_error_ - last_lat_error_) / dt;
  const double d_angle = (angle_error_ - last_angle_error_) / dt;
  last_lon_error_ = lon_error_;
  last_lat_error_ = lat_error_;
  last_angle_error_ = angle_error_;

  const double angular = angle_error_ * config_.kp_ang + i_angle_ * config_.ki_ang + d_angle * config_.kd_ang;
  if (phase_ == Phase::kFollowing || phase_ == Phase::kGoalApproach || phase_ == Phase::kCornerApproach) {
    const double linear = lon_error_ * config_.kp_lon + i_lon_ * config_.ki_lon + d_lon * config_.kd_lon;
    // Forwards only: a control point behind the mower means waiting for it, one facing far off turning first.
    cmd.linear = std::abs(angle_error_) > config_.max_drive_angle_error ? 0.0 : std::clamp(linear, 0.0, config_.max_linear);
    double gain = 1.0;
    if (config_.lateral_priority_distance > 0) {
      gain = std::max(config_.lateral_priority_min_gain,
                      (config_.lateral_priority_distance - std::abs(lat_error_)) / config_.lateral_priority_distance);
    }
    cmd.angular = Clamp(gain * angular + lat_error_ * config_.kp_lat + i_lat_ * config_.ki_lat + d_lat * config_.kd_lat,
                        config_.max_angular);
  } else {
    // Turning on the spot before the start, on a corner and at the goal.
    cmd.angular = Clamp(angular, config_.max_angular);
  }
  if (config_.straight) cmd.angular = 0;
  CheckStall(pose, cmd.linear, dt);
  if (phase_ == Phase::kFailed) return Command();
  return cmd;
}

}  // namespace mowbite_follower
