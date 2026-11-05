#include "mowbite_logic/docking.h"

#include <algorithm>
#include <cstdio>

namespace mowbite_logic {

namespace {

// The follower needs poses at most this far apart (m) on a straight path, and at least 3 of them.
constexpr double kPoseSpacing = 0.1;
// m before its end where the line to the gate and the way out are done.
constexpr double kLineEndTolerance = 0.02;
// m before its end, 0.05 m past the station, where the contact leg is done without contact.
constexpr double kContactEndTolerance = 0.005;
// A try begins by driving out when base_link stands at most kOnLineAcross m beside the line, more than kOutMin m past
// the approach point and at most kOutMin m past the end of the line.
constexpr double kOutMin = 0.1;
constexpr double kOnLineAcross = 0.3;
// Within kOutMin of the approach point and facing within this of the way the mower faces on the station, a try begins
// with the line: the follower starts it without turning on the spot.
constexpr double kAtApproachHeading = 20.0 * M_PI / 180.0;

double Normalize(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

const char* DockingLegName(DockingLeg::Kind kind) {
  switch (kind) {
    case DockingLeg::Kind::kOut: return "out";
    case DockingLeg::Kind::kLine: return "line";
    case DockingLeg::Kind::kContact: return "contact";
    case DockingLeg::Kind::kApproach:
    default: return "approach";
  }
}

DockingSequence::DockingSequence(const DockConfig& config, const UndockConfig& out, const DockPose& station,
                                 double facing)
    : config_(config), out_(out), station_(station), facing_(facing) {}

Point2 DockingSequence::ApproachPoint() const {
  return {station_.x - config_.approach_distance * std::cos(station_.heading),
          station_.y - config_.approach_distance * std::sin(station_.heading)};
}

double DockingSequence::Along(double x, double y) const {
  const Point2 a = ApproachPoint();
  return (x - a.x) * std::cos(station_.heading) + (y - a.y) * std::sin(station_.heading);
}

double DockingSequence::Across(double x, double y) const {
  const Point2 a = ApproachPoint();
  return (y - a.y) * std::cos(station_.heading) - (x - a.x) * std::sin(station_.heading);
}

bool DockingSequence::OnLine(double x, double y) const {
  const double along = Along(x, y);
  return along > kOutMin && along <= config_.approach_distance + config_.overshoot + kOutMin &&
         std::abs(Across(x, y)) <= kOnLineAcross;
}

DockingLeg DockingSequence::FirstLeg(double x, double y, double heading) const {
  if (OnLine(x, y)) return Out(x, y, heading);
  if (std::abs(Along(x, y)) <= kOutMin && std::abs(Across(x, y)) <= kOutMin &&
      std::abs(Normalize(heading - facing_)) < kAtApproachHeading) {
    return Line();
  }
  return Approach();
}

DockingLeg DockingSequence::Approach() const {
  DockingLeg leg;
  leg.kind = DockingLeg::Kind::kApproach;
  const Point2 p = ApproachPoint();
  leg.goal = {p.x, p.y, facing_};
  return leg;
}

DockingLeg DockingSequence::Out(double x, double y, double heading) const {
  // Straight ahead the way it faces, as far as the approach point lies along the line: the line onto the station then
  // steers it back onto it.
  UndockConfig out = out_;
  out.distance = std::max(Along(x, y), kPoseSpacing);
  const UndockPath path = PlanUndock(x, y, heading, out);
  DockingLeg leg;
  leg.kind = DockingLeg::Kind::kOut;
  leg.poses = path.poses;
  leg.corridor = path.corridor;
  leg.straight = true;
  leg.speed = path.speed;
  leg.stall_time = path.stall_time;
  leg.end_tolerance = kLineEndTolerance;
  return leg;
}

DockingLeg DockingSequence::Backwards(DockingLeg::Kind kind, double from, double to) const {
  DockingLeg leg;
  leg.kind = kind;
  leg.reverse = true;
  const Point2 a = ApproachPoint();
  const double c = std::cos(station_.heading);
  const double s = std::sin(station_.heading);
  const int steps = std::max(2, static_cast<int>(std::ceil((to - from) / kPoseSpacing - 1e-9)));
  for (int i = 0; i <= steps; i++) {
    const double along = from + (to - from) * i / steps;
    leg.poses.push_back({a.x + along * c, a.y + along * s, facing_});
  }
  // The whole line, from corridor m before the approach point to corridor m past its end.
  const double begin = -out_.corridor;
  const double end = config_.approach_distance + config_.overshoot + out_.corridor;
  const double side_x = -s * out_.corridor;
  const double side_y = c * out_.corridor;
  leg.corridor = {{a.x + begin * c + side_x, a.y + begin * s + side_y},
                  {a.x + begin * c - side_x, a.y + begin * s - side_y},
                  {a.x + end * c - side_x, a.y + end * s - side_y},
                  {a.x + end * c + side_x, a.y + end * s + side_y}};
  return leg;
}

DockingLeg DockingSequence::Line() const {
  const double gate = std::max(config_.approach_distance - config_.gate_distance, kPoseSpacing);
  DockingLeg leg = Backwards(DockingLeg::Kind::kLine, 0.0, gate);
  leg.speed = config_.speed;
  // Nothing should stop the mower before the gate: standing there counts as stuck as soon as on the way out.
  leg.stall_time = out_.stall_time;
  leg.end_tolerance = kLineEndTolerance;
  return leg;
}

DockingLeg DockingSequence::Contact() const {
  const double gate = std::max(config_.approach_distance - config_.gate_distance, kPoseSpacing);
  DockingLeg leg = Backwards(DockingLeg::Kind::kContact, gate, config_.approach_distance + config_.overshoot);
  leg.speed = config_.contact_speed;
  // At the station the contacts need some pushing.
  leg.stall_time = config_.contact_timeout;
  leg.end_tolerance = kContactEndTolerance;
  return leg;
}

std::string DockingSequence::CheckGate(double x, double y, double heading, float accuracy,
                                       double accuracy_limit) const {
  char text[200];
  const double across = Across(x, y);
  if (std::abs(across) > config_.gate_lateral) {
    std::snprintf(text, sizeof(text),
                  "at the gate the mower stood %.3f m beside the line of the docking station, at most %.3f m allowed",
                  std::abs(across), config_.gate_lateral);
    return text;
  }
  const double off = std::abs(Normalize(heading - facing_));
  if (off > config_.gate_heading) {
    std::snprintf(text, sizeof(text),
                  "at the gate the mower faced %.1f deg off the line of the docking station, at most %.1f deg allowed",
                  off * 180.0 / M_PI, config_.gate_heading * 180.0 / M_PI);
    return text;
  }
  if (!(accuracy <= accuracy_limit)) {
    std::snprintf(text, sizeof(text), "at the gate the position accuracy %.3f m was worse than %.3f m", accuracy,
                  accuracy_limit);
    return text;
  }
  return "";
}

bool DockingSequence::Failed() {
  failures_++;
  return failures_ <= config_.retry_count;
}

ChargeCheck::ChargeCheck(const ChargeConfig& config, double start) : config_(config), start_(start) {}

ChargeCheck::Result ChargeCheck::OnSample(const ChargeSample& sample) {
  if (result_ != Result::kWaiting) return result_;
  if (!sample.contact) return result_ = Result::kContactLost;
  const bool charging = sample.battery_ok && sample.charge_current >= config_.current;
  const bool full = sample.battery_ok && sample.percentage >= config_.expected_below;
  if (sample.battery_ok) {
    max_current_ = std::max(max_current_, sample.charge_current);
    percentage_ = sample.percentage;
  }
  if (!charging && !full) {
    good_since_.reset();
  } else if (!good_since_) {
    good_since_ = sample.time;
  }
  if (good_since_ && sample.time - *good_since_ >= config_.time) {
    return result_ = charging ? Result::kCharging : Result::kFull;
  }
  return OnTick(sample.time);
}

ChargeCheck::Result ChargeCheck::OnTick(double now) {
  // A stretch that began before the deadline may still end it well.
  if (result_ == Result::kWaiting && !good_since_ && now - start_ >= config_.timeout) result_ = Result::kNoCharge;
  return result_;
}

ChargeWatch::Change ChargeWatch::OnSample(const ChargeSample& sample, bool emergency) {
  if (!sample.contact) {
    good_since_.reset();
    current_gone_since_.reset();
    charging_ = false;
    if (!on_station_) return Change::kNone;
    if (!contact_gone_since_) {
      contact_gone_since_ = sample.time;
      emergency_ = false;
    }
    emergency_ = emergency_ || emergency;
    if (sample.time - *contact_gone_since_ < config_.lost_time) return Change::kNone;
    // Lifted or stopped by hand: the emergency tells already.
    const bool tell = !emergency_;
    Leave();
    return tell ? Change::kContactLost : Change::kNone;
  }
  if (!on_station_) {
    on_station_ = true;
    stay_since_ = sample.time;
  }
  contact_gone_since_.reset();
  const bool current = sample.battery_ok && sample.charge_current >= config_.current;
  const bool full = sample.battery_ok && sample.percentage >= config_.expected_below;
  if (!current && !full) {
    good_since_.reset();
  } else if (!good_since_) {
    good_since_ = sample.time;
  }
  if (current) {
    charging_ = true;
    current_gone_since_.reset();
  }
  if (!confirmed_) {
    if (good_since_ && sample.time - *good_since_ >= config_.time) {
      confirmed_ = true;
      return Change::kConfirmed;
    }
    if (no_current_told_ || good_since_ || emergency || sample.time - stay_since_ < config_.timeout) {
      return Change::kNone;
    }
    no_current_told_ = true;
    return Change::kNoCurrent;
  }
  // Confirmed: a current that stops near full ends there, below it is lost after lost_time; unknown tells nothing.
  if (current || !charging_ || !sample.battery_ok) {
    current_gone_since_.reset();
    return Change::kNone;
  }
  if (full) {
    charging_ = false;
    current_gone_since_.reset();
    return Change::kNone;
  }
  if (!current_gone_since_) {
    current_gone_since_ = sample.time;
    emergency_ = false;
  }
  emergency_ = emergency_ || emergency;
  if (sample.time - *current_gone_since_ < config_.lost_time) return Change::kNone;
  charging_ = false;
  current_gone_since_.reset();
  confirmed_ = false;
  no_current_told_ = true;
  return emergency_ ? Change::kNone : Change::kCurrentLost;
}

void ChargeWatch::OnBusy(bool contact_ended) {
  on_station_ = false;
  contact_gone_since_.reset();
  current_gone_since_.reset();
  good_since_.reset();
  charging_ = false;
  emergency_ = false;
  if (contact_ended) Leave();
}

void ChargeWatch::Confirm() {
  confirmed_ = true;
  no_current_told_ = false;
}

void ChargeWatch::Unconfirm(bool told) {
  confirmed_ = false;
  no_current_told_ = told;
}

void ChargeWatch::Leave() {
  on_station_ = false;
  confirmed_ = false;
  no_current_told_ = false;
  contact_gone_since_.reset();
  current_gone_since_.reset();
  good_since_.reset();
  charging_ = false;
  emergency_ = false;
}

}  // namespace mowbite_logic
