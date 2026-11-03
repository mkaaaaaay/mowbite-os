#include "mowbite_calibration/program.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mowbite_calibration {

namespace {

constexpr double kTimeEpsilon = 1e-6;
// Gyro samples further apart than this are not integrated (s).
constexpr double kMaxImuGap = 0.1;
// Look-backs (s): the IMU rate, the rate of RTK fixed positions, the mean of gravity and the speed of the antenna.
constexpr double kImuRateWindow = 1.0;
constexpr double kGnssRateWindow = 2.0;
constexpr double kGravityWindow = 0.5;
constexpr double kSpeedWindow = 1.0;
// A stand is judged after the mower came to rest (s); its gyro gives the offset from then on.
constexpr double kSettle = 1.0;
// Antenna positions this close to half a turn apart (rad) make a pair whose middle is the pivot.
constexpr double kPairTolerance = 10.0 * M_PI / 180.0;
// Fewest samples for a mean of gravity and for an offset of the gyro.
constexpr size_t kMinGravitySamples = 5;
constexpr size_t kMinBiasSamples = 10;

double Wrap(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double Deg(double rad) {
  return rad * 180.0 / M_PI;
}

std::string Text(const char* format, double a = 0, double b = 0, double c = 0) {
  char text[240];
  std::snprintf(text, sizeof(text), format, a, b, c);
  return text;
}

Problem Make(const char* reason, std::string message, std::vector<std::pair<std::string, double>> numbers = {}) {
  return Problem{reason, std::move(message), std::move(numbers)};
}

bool Turning(Phase phase) {
  return phase == Phase::kAlign || phase == Phase::kSpinLeft || phase == Phase::kSpinRight;
}

}  // namespace

const char* PhaseName(Phase phase) {
  switch (phase) {
    case Phase::kStart: return "start";
    case Phase::kAlign: return "align";
    case Phase::kStand: return "stand";
    case Phase::kLine: return "line";
    case Phase::kSpinLeft: return "spin_left";
    case Phase::kSpinRight: return "spin_right";
    case Phase::kEnd: return "end";
  }
  return "unknown";
}

bool Program::Hold::Failing(bool now_failing, double now, double hold_time) {
  if (!now_failing) {
    since.reset();
    return false;
  }
  if (!since) since = now;
  return now - *since + kTimeEpsilon >= hold_time;
}

Program::Program(const ProgramConfig& config) : config_(config) {
}

void Program::OnImu(const ImuSample& sample, double now) {
  if (running_ && imu_s_) {
    const double dt = now - *imu_s_;
    const Phase current = phase();
    if (dt > 0 && dt <= kMaxImuGap && Turning(current)) turned_ += (0.5 * (imu_.gz + sample.gz) - gyro_bias_) * dt;
    if (current == Phase::kStand && now - step_s_ >= kSettle) {
      gyro_sum_ += sample.gz;
      gyro_count_++;
    }
  }
  imu_s_ = now;
  imu_ = sample;
  imu_times_.push_back(now);
  while (!imu_times_.empty() && imu_times_.front() < now - kImuRateWindow) imu_times_.pop_front();
  accel_.emplace_back(now, sample);
  while (!accel_.empty() && accel_.front().first < now - kGravityWindow) accel_.pop_front();
}

void Program::OnWheels(double /*linear*/, double angular, double now) {
  wheels_s_ = now;
  wheel_angular_ = angular;
}

void Program::OnGnss(const GnssSample& sample, double now) {
  const bool good = sample.fixed && sample.accuracy <= config_.rtk_accuracy;
  fixes_.push_back({now, sample.x, sample.y, good});
  while (!fixes_.empty() && fixes_.front().t < now - kGnssRateWindow) fixes_.pop_front();
  if (!good) return;
  good_fix_s_ = now;
  if (!running_) return;
  const Phase current = phase();
  if (current == Phase::kLine && !antenna_origin_) antenna_origin_ = std::make_pair(sample.x, sample.y);
  if (current != Phase::kSpinLeft && current != Phase::kSpinRight) return;
  // Half a turn apart the antenna stands on the other side of the pivot: the middle of the two is the pivot.
  const std::pair<double, double>* other = nullptr;
  double best = kPairTolerance;
  for (const auto& [angle, position] : turn_fixes_) {
    const double off = std::abs(std::abs(turned_ - angle) - M_PI);
    if (off <= best) {
      best = off;
      other = &position;
    }
  }
  if (other) {
    const double mx = 0.5 * (sample.x + other->first);
    const double my = 0.5 * (sample.y + other->second);
    if (!pivot_) {
      pivot_ = std::make_pair(mx, my);
    } else {
      pivot_drift_ = std::hypot(mx - pivot_->first, my - pivot_->second);
    }
  }
  turn_fixes_.emplace_back(turned_, std::make_pair(sample.x, sample.y));
}

void Program::OnPose(const PoseSample& sample, double now) {
  pose_s_ = now;
  pose_ = sample;
}

void Program::OnEmergency(bool emergency, double /*now*/) {
  emergency_ = emergency;
}

std::optional<Problem> Program::Gravity(double since, double now) const {
  double ax = 0, ay = 0, az = 0;
  size_t n = 0;
  for (const auto& [t, sample] : accel_) {
    if (t < since - kTimeEpsilon || t > now + kTimeEpsilon) continue;
    ax += sample.ax;
    ay += sample.ay;
    az += sample.az;
    n++;
  }
  if (n < kMinGravitySamples) return std::nullopt;
  ax /= n;
  ay /= n;
  az /= n;
  const double norm = std::sqrt(ax * ax + ay * ay + az * az);
  const double tilt = norm > 0 ? std::acos(std::clamp(az / norm, -1.0, 1.0)) : M_PI;
  if (!(norm > 0) || std::abs(norm - config_.gravity) > config_.gravity_tolerance * config_.gravity ||
      tilt > config_.gravity_max_tilt) {
    return Make("IMU_GRAVITY",
                Text("Gravity %.2f m/s2 at %.0f deg from +z: is the IMU mounted as configured?", norm, Deg(tilt)),
                {{"gravity", norm}, {"tilt", Deg(tilt)}});
  }
  if (tilt > config_.max_slope) {
    return Make("SLOPE", Text("Tilted by %.1f deg, at most %.0f deg for a calibration", Deg(tilt), Deg(config_.max_slope)),
                {{"slope", Deg(tilt)}, {"limit", Deg(config_.max_slope)}});
  }
  return std::nullopt;
}

std::vector<Problem> Program::Readiness(double now) const {
  std::vector<Problem> problems;
  size_t imu_count = 0;
  for (double t : imu_times_) {
    if (t >= now - kImuRateWindow - kTimeEpsilon && t <= now + kTimeEpsilon) imu_count++;
  }
  const double imu_rate = imu_count / kImuRateWindow;
  if (!imu_s_ || now - *imu_s_ > config_.sensor_timeout || imu_rate + kTimeEpsilon < config_.min_imu_rate) {
    problems.push_back(Make("SENSOR_MISSING",
                            Text("IMU: %.0f samples per s, at least %.0f needed", imu_rate, config_.min_imu_rate),
                            {{"rate", imu_rate}, {"limit", config_.min_imu_rate}}));
  }
  if (!wheels_s_ || now - *wheels_s_ > config_.sensor_timeout) {
    problems.push_back(Make("SENSOR_MISSING", "No wheel odometry from the driver"));
  }
  if (!good_fix_s_ || now - *good_fix_s_ > config_.rtk_timeout) {
    problems.push_back(Make("NO_RTK",
                            Text("No RTK fixed position of %.2f m or better in the last %.1f s", config_.rtk_accuracy,
                                 config_.rtk_timeout),
                            {{"limit", config_.rtk_accuracy}}));
  } else {
    size_t good = 0;
    for (const auto& fix : fixes_) {
      if (fix.good && fix.t >= now - kGnssRateWindow - kTimeEpsilon && fix.t <= now + kTimeEpsilon) good++;
    }
    const double rate = good / kGnssRateWindow;
    if (rate + kTimeEpsilon < config_.min_gnss_rate) {
      problems.push_back(Make("GNSS_RATE",
                              Text("%.1f RTK fixed positions per s, at least %.0f needed", rate, config_.min_gnss_rate),
                              {{"rate", rate}, {"limit", config_.min_gnss_rate}}));
    }
  }
  if (auto level = Gravity(now - kGravityWindow, now)) problems.push_back(*level);
  return problems;
}

bool Program::Start(const Lane& lane, double now) {
  if (running_) return false;
  lane_ = lane;
  steps_.clear();
  edges_.clear();
  outcome_.reset();
  commands_.clear();
  sign_hold_ = slow_hold_ = fast_hold_ = lane_hold_ = Hold();
  turned_ = 0;
  turn_fixes_.clear();
  pivot_.reset();
  pivot_drift_ = 0;
  antenna_origin_.reset();
  const double spin_up = config_.spin_rate / config_.spin_accel;
  const double off = pose_s_ ? std::abs(Wrap(lane.heading - pose_.heading)) : M_PI / 2;
  steps_.push_back({Phase::kStart, 0, 0.0});
  steps_.push_back({Phase::kAlign, 0, off / config_.spin_rate + spin_up});
  steps_.push_back({Phase::kStand, 0, config_.first_stand});
  const double line = lane.length / config_.line_speed + config_.line_speed / config_.line_accel;
  const double spin = config_.spin_angle / config_.spin_rate + spin_up;
  for (int round = 1; round <= config_.rounds; round++) {
    steps_.push_back({Phase::kLine, round, line});
    steps_.push_back({Phase::kStand, round, config_.stand});
    steps_.push_back({round % 2 == 1 ? Phase::kSpinLeft : Phase::kSpinRight, round, spin});
    steps_.push_back({Phase::kStand, round, config_.stand});
  }
  running_ = true;
  Begin(0, now);
  return true;
}

void Program::Begin(size_t index, double now) {
  index_ = index;
  step_s_ = now;
  last_tick_s_ = now;
  cmd_ = 0;
  slow_hold_ = fast_hold_ = lane_hold_ = Hold();
  Step& step = steps_[index_];
  switch (step.phase) {
    case Phase::kAlign: {
      const double off = Wrap(lane_.heading - pose_.heading);
      if (std::abs(off) < config_.align_skip) {
        // Facing the lane already: no turn, no phase.
        step.expected = 0;
        Begin(index + 1, now);
        return;
      }
      align_sign_ = off > 0 ? 1.0 : -1.0;
      break;
    }
    case Phase::kStand:
      gyro_sum_ = 0;
      gyro_count_ = 0;
      break;
    case Phase::kLine:
      line_x_ = pose_.x;
      line_y_ = pose_.y;
      // The mower stood until now: the latest fix is where the antenna starts.
      antenna_origin_.reset();
      if (!fixes_.empty() && fixes_.back().good) antenna_origin_ = std::make_pair(fixes_.back().x, fixes_.back().y);
      break;
    case Phase::kSpinLeft:
    case Phase::kSpinRight:
      turned_ = 0;
      turn_fixes_.clear();
      pivot_.reset();
      pivot_drift_ = 0;
      break;
    case Phase::kStart:
    case Phase::kEnd:
      break;
  }
  Edge(true, now);
}

std::optional<Twist2> Program::Next(double now, bool moved) {
  Edge(false, now);
  const Step& step = steps_[index_];
  // The offset of the gyro for the turns: the mean of the stand that ends.
  if (step.phase == Phase::kStand && gyro_count_ >= kMinBiasSamples) gyro_bias_ = gyro_sum_ / gyro_count_;
  if (index_ + 1 >= steps_.size()) {
    running_ = false;
    outcome_ = Make("DONE", "The calibration drive ran to its end");
  } else {
    Begin(index_ + 1, now);
  }
  if (!moved) return std::nullopt;
  cmd_ = 0;
  return Twist2{};
}

std::optional<Twist2> Program::Abort(Problem problem, double now) {
  if (!running_) return std::nullopt;
  Edge(false, now);
  running_ = false;
  outcome_ = std::move(problem);
  cmd_ = 0;
  return Twist2{};
}

std::optional<Twist2> Program::Stop(const std::string& message, double now) {
  return Abort(Make("STOPPED", message), now);
}

std::optional<Twist2> Program::Record(const Twist2& twist, double now) {
  commands_.emplace_back(now, twist);
  while (!commands_.empty() && commands_.front().first < now - config_.overspeed_window) commands_.pop_front();
  return twist;
}

void Program::Edge(bool start, double now) {
  const Step& step = steps_[index_];
  if (step.phase == Phase::kStart || step.phase == Phase::kEnd) return;
  edges_.push_back({step.phase, step.round, start, now});
}

std::optional<Twist2> Program::Tick(double now) {
  if (!running_) return std::nullopt;
  const Step& step = steps_[index_];
  if (step.phase == Phase::kStart) {
    // Everything has to be there, then the mower has to stand level.
    std::optional<Problem> missing;
    if (!imu_s_ || now - *imu_s_ > config_.sensor_timeout) {
      missing = Make("SENSOR_MISSING", "No IMU sample at the start");
    } else if (!wheels_s_ || now - *wheels_s_ > config_.sensor_timeout) {
      missing = Make("SENSOR_MISSING", "No wheel odometry at the start");
    } else if (!pose_s_ || now - *pose_s_ > config_.pose_timeout || !pose_.heading_valid) {
      missing = Make("POSE_LOST", "No pose with a heading from the localization at the start");
    } else if (!good_fix_s_ || now - *good_fix_s_ > config_.rtk_timeout) {
      missing = Make("NO_RTK", Text("No RTK fixed position of %.2f m or better at the start", config_.rtk_accuracy),
                     {{"limit", config_.rtk_accuracy}});
    }
    if (missing) {
      if (now - step_s_ + kTimeEpsilon >= config_.start_timeout) return Abort(*missing, now);
      return std::nullopt;
    }
    if (auto level = Gravity(now - kGravityWindow, now)) return Abort(*level, now);
    return Next(now, false);
  }
  if (auto problem = Common(now)) return Abort(*problem, now);
  std::optional<Twist2> out;
  switch (step.phase) {
    case Phase::kAlign: out = TickAlign(now); break;
    case Phase::kLine: out = TickLine(now); break;
    case Phase::kSpinLeft:
    case Phase::kSpinRight: out = TickSpin(now); break;
    case Phase::kStand:
      if (auto problem = StandChecks(now)) return Abort(*problem, now);
      if (now - step_s_ + kTimeEpsilon >= step.expected) out = Next(now, false);
      break;
    case Phase::kStart:
    case Phase::kEnd: break;
  }
  last_tick_s_ = now;
  return out;
}

std::optional<Problem> Program::Common(double now) {
  if (emergency_) return Make("EMERGENCY", "Emergency: the calibration drive stops");
  if (!imu_s_ || now - *imu_s_ > config_.sensor_timeout) {
    const double quiet = imu_s_ ? now - *imu_s_ : 0;
    return Make("SENSOR_MISSING", Text("No IMU sample for %.1f s", quiet),
                {{"seconds", quiet}, {"limit", config_.sensor_timeout}});
  }
  if (!wheels_s_ || now - *wheels_s_ > config_.sensor_timeout) {
    const double quiet = wheels_s_ ? now - *wheels_s_ : 0;
    return Make("SENSOR_MISSING", Text("No wheel odometry for %.1f s", quiet),
                {{"seconds", quiet}, {"limit", config_.sensor_timeout}});
  }
  if (!pose_s_ || now - *pose_s_ > config_.pose_timeout) {
    const double quiet = pose_s_ ? now - *pose_s_ : 0;
    return Make("POSE_LOST", Text("No pose from the localization for %.1f s", quiet),
                {{"seconds", quiet}, {"limit", config_.pose_timeout}});
  }
  if (!pose_.heading_valid) return Make("POSE_LOST", "The localization has no heading any more");
  if (!good_fix_s_ || now - *good_fix_s_ > config_.rtk_timeout) {
    const double quiet = good_fix_s_ ? now - *good_fix_s_ : 0;
    return Make("NO_RTK",
                Text("No RTK fixed position of %.2f m or better for %.1f s", config_.rtk_accuracy, quiet),
                {{"seconds", quiet}, {"limit", config_.rtk_timeout}});
  }
  const Step& step = steps_[index_];
  const double limit = config_.timeout_factor * step.expected + config_.timeout_extra;
  if (now - step_s_ > limit) {
    char text[160];
    std::snprintf(text, sizeof(text), "The phase %s took %.0f s, at most %.0f s", PhaseName(step.phase),
                  now - step_s_, limit);
    return Make("TIMEOUT", text, {{"seconds", now - step_s_}, {"limit", limit}});
  }
  const double gyro = imu_.gz - gyro_bias_;
  const bool against = std::abs(wheel_angular_) >= config_.gyro_sign_min && std::abs(gyro) >= config_.gyro_sign_min &&
                       (wheel_angular_ > 0) != (gyro > 0);
  if (sign_hold_.Failing(against, now, config_.hold_time)) {
    return Make("GYRO_SIGN", Text("The gyro turns at %.2f rad/s against the wheels at %.2f rad/s: is the IMU mounted "
                                  "as configured?", gyro, wheel_angular_),
                {{"gyro", gyro}, {"wheels", wheel_angular_}});
  }
  return std::nullopt;
}

std::optional<Problem> Program::StandChecks(double now) const {
  if (now - step_s_ < kSettle) return std::nullopt;
  return Gravity(now - kGravityWindow, now);
}

double Program::CommandPeak(bool angular, double now) const {
  double peak = 0;
  for (const auto& [t, twist] : commands_) {
    if (t < now - config_.overspeed_window - kTimeEpsilon) continue;
    peak = std::max(peak, std::abs(angular ? twist.angular : twist.linear));
  }
  return peak;
}

std::optional<double> Program::AntennaSpeed(double now) const {
  const Fix* last = nullptr;
  for (auto it = fixes_.rbegin(); it != fixes_.rend(); ++it) {
    if (it->good) {
      last = &*it;
      break;
    }
  }
  if (!last || now - last->t > config_.rtk_timeout) return std::nullopt;
  const Fix* first = nullptr;
  for (const auto& fix : fixes_) {
    if (fix.good && fix.t >= last->t - kSpeedWindow - kTimeEpsilon) {
      first = &fix;
      break;
    }
  }
  if (!first || last->t - first->t < 0.5 * kSpeedWindow) return std::nullopt;
  return std::hypot(last->x - first->x, last->y - first->y) / (last->t - first->t);
}

std::optional<Problem> Program::TurnChecks(double rate_cmd, double now) {
  const double gyro = imu_.gz - gyro_bias_;
  const double direction = rate_cmd >= 0 ? 1.0 : -1.0;
  const bool full = std::abs(rate_cmd) >= config_.spin_rate - kTimeEpsilon;
  const bool slow = now - step_s_ + kTimeEpsilon >= config_.turn_check_after && full &&
                    gyro * direction < config_.turn_min_ratio * std::abs(rate_cmd);
  if (slow_hold_.Failing(slow, now, config_.hold_time)) {
    const double limit = config_.turn_min_ratio * std::abs(rate_cmd);
    return Make("NOT_TURNING",
                Text("The mower turns at %.2f rad/s, at least %.2f rad/s expected: dead zone or slipping wheels",
                     gyro * direction, limit),
                {{"rate", gyro * direction}, {"limit", limit}});
  }
  const double allowed = config_.overspeed_ratio * CommandPeak(true, now) + config_.overspeed_floor;
  if (fast_hold_.Failing(std::abs(gyro) > allowed, now, config_.hold_time)) {
    return Make("OVERSPEED", Text("The mower turns at %.2f rad/s, at most %.2f rad/s allowed", std::abs(gyro), allowed),
                {{"rate", std::abs(gyro)}, {"limit", allowed}});
  }
  if (pivot_ && lane_hold_.Failing(pivot_drift_ > config_.lane_limit, now, config_.hold_time)) {
    return Make("LEFT_LANE",
                Text("The pivot of the turn moved %.2f m, at most %.2f m", pivot_drift_, config_.lane_limit),
                {{"distance", pivot_drift_}, {"limit", config_.lane_limit}});
  }
  return std::nullopt;
}

std::optional<Problem> Program::LineChecks(double now) {
  const auto speed = AntennaSpeed(now);
  const bool full = cmd_ >= config_.line_speed - kTimeEpsilon;
  const bool slow = speed && now - step_s_ + kTimeEpsilon >= config_.move_check_after && full &&
                    *speed < config_.move_min_ratio * cmd_;
  if (slow_hold_.Failing(slow, now, config_.hold_time)) {
    const double limit = config_.move_min_ratio * cmd_;
    return Make("NOT_MOVING",
                Text("The antenna moves at %.2f m/s, at least %.2f m/s expected: dead zone or slipping wheels", *speed,
                     limit),
                {{"speed", *speed}, {"limit", limit}});
  }
  const double allowed =
      std::min(config_.overspeed_ratio * CommandPeak(false, now) + config_.overspeed_floor, config_.max_speed);
  if (fast_hold_.Failing(speed && *speed > allowed, now, config_.hold_time)) {
    return Make("OVERSPEED", Text("The antenna moves at %.2f m/s, at most %.2f m/s allowed", *speed, allowed),
                {{"speed", *speed}, {"limit", allowed}});
  }
  if (antenna_origin_ && !fixes_.empty() && fixes_.back().good) {
    const double heading = LineHeading();
    const double dx = fixes_.back().x - antenna_origin_->first;
    const double dy = fixes_.back().y - antenna_origin_->second;
    const double off = std::abs(-dx * std::sin(heading) + dy * std::cos(heading));
    if (lane_hold_.Failing(off > config_.lane_limit, now, config_.hold_time)) {
      return Make("LEFT_LANE", Text("The antenna is %.2f m off its line, at most %.2f m", off, config_.lane_limit),
                  {{"distance", off}, {"limit", config_.lane_limit}});
    }
  }
  return std::nullopt;
}

double Program::LineHeading() const {
  const int round = steps_[index_].round;
  return round % 2 == 1 ? lane_.heading : Wrap(lane_.heading + M_PI);
}

std::optional<Twist2> Program::TickAlign(double now) {
  const double dt = std::clamp(now - last_tick_s_, 0.0, 0.1);
  const double off = Wrap(lane_.heading - pose_.heading);
  // Within the tolerance, or past the lane already: the lines steer the rest.
  if (std::abs(off) <= config_.align_tolerance || off * align_sign_ < 0) return Next(now, true);
  cmd_ = std::min(config_.spin_rate, cmd_ + config_.spin_accel * dt);
  const double rate = align_sign_ * cmd_;
  if (auto problem = TurnChecks(rate, now)) return Abort(*problem, now);
  return Record(Twist2{0.0, rate}, now);
}

std::optional<Twist2> Program::TickSpin(double now) {
  const double dt = std::clamp(now - last_tick_s_, 0.0, 0.1);
  const double direction = steps_[index_].phase == Phase::kSpinLeft ? 1.0 : -1.0;
  if (direction * turned_ >= config_.spin_angle) return Next(now, true);
  cmd_ = std::min(config_.spin_rate, cmd_ + config_.spin_accel * dt);
  const double rate = direction * cmd_;
  if (auto problem = TurnChecks(rate, now)) return Abort(*problem, now);
  return Record(Twist2{0.0, rate}, now);
}

std::optional<Twist2> Program::TickLine(double now) {
  const double dt = std::clamp(now - last_tick_s_, 0.0, 0.1);
  const double heading = LineHeading();
  const double ux = std::cos(heading);
  const double uy = std::sin(heading);
  const double dx = pose_.x - line_x_;
  const double dy = pose_.y - line_y_;
  const double remaining = lane_.length - (dx * ux + dy * uy);
  if (remaining <= 0) return Next(now, true);
  // Up by the ramp, down along it towards the end, never below the least speed that moves the wheels.
  const double profile = std::max(config_.line_min_speed, std::sqrt(2.0 * config_.line_accel * remaining));
  cmd_ = std::min({config_.line_speed, profile, cmd_ + config_.line_accel * dt});
  // Left of the line and facing left of it both turn right.
  const double cross = -dx * uy + dy * ux;
  const double facing = Wrap(pose_.heading - heading);
  const double steer = std::clamp(-(config_.steer_cross_gain * cross + config_.steer_heading_gain * facing),
                                  -config_.steer_max, config_.steer_max);
  if (auto problem = LineChecks(now)) return Abort(*problem, now);
  return Record(Twist2{cmd_, steer}, now);
}

bool Program::Running() const {
  return running_;
}

Phase Program::phase() const {
  if (running_) return steps_[index_].phase;
  return outcome_ ? Phase::kEnd : Phase::kStart;
}

int Program::round() const {
  return steps_.empty() ? 0 : steps_[index_].round;
}

double Program::Progress(double now) const {
  if (outcome_ && outcome_->reason == "DONE") return 1.0;
  if (steps_.empty()) return 0.0;
  double total = 0;
  double done = 0;
  for (size_t i = 0; i < steps_.size(); i++) {
    total += steps_[i].expected;
    if (i < index_) done += steps_[i].expected;
  }
  if (running_) done += std::min(std::max(now - step_s_, 0.0), steps_[index_].expected);
  return total > 0 ? std::clamp(done / total, 0.0, 1.0) : 0.0;
}

const std::optional<Problem>& Program::outcome() const {
  return outcome_;
}

std::vector<PhaseEdge> Program::TakeEdges() {
  std::vector<PhaseEdge> out;
  out.swap(edges_);
  return out;
}

}  // namespace mowbite_calibration
