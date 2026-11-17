#ifndef MOWBITE_LOGIC_DOCKING_H
#define MOWBITE_LOGIC_DOCKING_H

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/undocking.h"

namespace mowbite_logic {

// Docking onto the station of the map, contacts first: over the drivable areas to the point approach_distance m in
// front of it, facing the way the mower faces on it, then along its line to the gate (backwards with the contacts at
// the back, forwards with them in front), where the mower stops and is checked, and on at contact_speed until the
// contacts. RTK steers the whole way; a camera may take over the last part later (O8). Assumptions until measured on
// the Tron in phase 7.
struct DockConfig {
  // m in front of the station along its way in (docking_approach_distance of params.set).
  double approach_distance = 0.8;
  // m/s along the line, and from the gate on; also the most for any way off the station backwards.
  double speed = 0.1;
  double contact_speed = 0.06;
  // gate_distance m before the station base_link has to stand at most gate_lateral m beside the line, its heading
  // within gate_heading (rad) of the one on the station, with the position accuracy of the station; else the try fails.
  double gate_distance = 0.25;
  double gate_lateral = 0.02;
  double gate_heading = 8.0 * M_PI / 180.0;
  // m the line reaches past the station: that far the mower looks for the contacts.
  double overshoot = 0.05;
  // s the mower may stand at the station without a debounced contact, pushing or waiting; on the line before the gate
  // standing counts as stuck after the stall time of undocking.
  double contact_timeout = 10.0;
  // Failed tries that are tried again (docking_retry_count of params.set).
  int retry_count = 4;
  // A planner that gives no answer for the way to the point in front of the station: the mower drives straight
  // there where CheckDirectRoute finds that way free, base_link at most direct_margin m off it; off: the try fails.
  bool direct = true;
  double direct_margin = 0.15;
};

// The frame of a leg: the map (localization/pose) or the station frame (localization/dock_pose, SOURCE_DOCK).
enum class LegFrame : uint8_t { kMap = 0, kDock = 1 };

// What the follower drives for docking.
struct DockingLeg {
  // kApproach: over the drivable areas to the point in front of the station, planned like a drive. kOut: straight along
  // the mower's own heading against the way in, out to that point. kLine: along the line to the gate. kContact: from
  // the gate onto the contacts, or blind from the end of kClose. kDirect: straight to the point in front of the
  // station, without the planner. kClose: in the station frame from the gate to where the marker leaves the image.
  enum class Kind { kApproach, kOut, kLine, kContact, kDirect, kClose };
  Kind kind = Kind::kApproach;
  // kApproach: the point in front of the station, facing the way the mower faces on it.
  DockPose goal;
  // The others: poses at most 0.1 m apart that face the way the mower faces, and where base_link may be besides the
  // drivable areas (none for kDirect); the rest as in FollowPath.
  std::vector<DockPose> poses;
  Ring corridor;
  bool straight = false;
  bool reverse = false;
  LegFrame frame = LegFrame::kMap;
  double speed = 0;
  double stall_time = 0;
  double end_tolerance = 0;
  // m base_link may get off its path (kDirect, every leg in the station frame), beyond that the follower ends the leg;
  // 0: the limit of the follower.
  double max_cross_track = 0;
};

const char* DockingLegName(DockingLeg::Kind kind);
// The legs that drive onto the station: the line, the close part and the contact.
bool OntoStation(DockingLeg::Kind kind);

// Docking in the station frame: every leg at most cross_track m off its path, and the contact stretch split
// blind_distance m before the station into kClose, where the marker guides, and the blind kContact.
struct StationLegs {
  double cross_track = 0;
  double blind_distance = 0;
};

// The straight way from base_link at (x, y) along its heading, forwards or backwards; backwards at most at the docking
// speed (G41).
UndockPath PlanLeave(double x, double y, double heading, bool reverse, UndockConfig out, const DockConfig& dock);

// The geometry and the tries of one docking, without ROS. LogicCore decides which leg comes next.
class DockingSequence {
 public:
  DockingSequence() = default;
  // station as in the map (heading: the way in); contacts_back: the mower faces away from it there and backs in; out:
  // how the mower leaves the station, like undocking; station_frame: the legs are in the station frame.
  DockingSequence(const DockConfig& config, const UndockConfig& out, const DockPose& station, bool contacts_back,
                  const std::optional<StationLegs>& station_frame = std::nullopt);

  // Where the line onto the station begins.
  Point2 ApproachPoint() const;
  // m from the approach point along the way in (towards the station), and across it (left positive).
  double Along(double x, double y) const;
  double Across(double x, double y) const;
  // Whether base_link stands on the line between the approach point and the station facing along it either way, where
  // a try begins by driving straight out to the approach point.
  bool OnLine(double x, double y, double heading) const;

  // The first leg of a try from base_link at (x, y) facing heading: out to the approach point from the line, the line
  // from the approach point, else the approach over the drivable areas.
  DockingLeg FirstLeg(double x, double y, double heading) const;
  DockingLeg Approach() const;
  // To stop_along m from the approach point along the way in (0: to the approach point).
  DockingLeg Out(double x, double y, double heading, double stop_along = 0.0) const;
  // Whether a straight way out to stop_along is left from (x, y).
  bool OutLeft(double x, double y, double stop_along) const;
  DockingLeg Line() const;
  // From the gate: kClose where the contact stretch is split, else kContact.
  DockingLeg AfterGate() const;
  DockingLeg Close() const;
  DockingLeg Contact() const;
  // The straight way from base_link at (x, y) to the approach point: poses along it, the last one facing the way the
  // mower faces on the station, base_link at most direct_margin off them. None from within 0.1 m of the approach
  // point: the line begins there.
  std::optional<DockingLeg> Direct(double x, double y) const;

  // Why the mower may not go on from where it stands at the gate, empty if it may: across (m) beside the line, the
  // heading off, or the position accuracy worse than accuracy_limit.
  std::string CheckGate(double across, double heading, float accuracy, double accuracy_limit) const;

  // Counts a failed try; true when another one follows.
  bool Failed();
  // Whether a failure now would be the last one: no try follows it.
  bool LastTry() const {
    return failures_ >= config_.retry_count;
  }
  int failures() const {
    return failures_;
  }
  const DockConfig& config() const {
    return config_;
  }
  // The heading of the mower on the station.
  double facing() const {
    return facing_;
  }
  bool station_frame() const {
    return station_frame_.has_value();
  }

 private:
  // Poses facing the mower's heading on the station from along m to to m (from the approach point), driven contacts
  // first, and the corridor of the line.
  DockingLeg Onto(DockingLeg::Kind kind, double from, double to) const;
  // m along the line where the gate is, and where the blind part begins.
  double GateAlong() const;
  double BlindAlong() const;
  bool Split() const;
  // Sets the frame of a leg and its limit beside the path.
  DockingLeg Framed(DockingLeg leg) const;

  DockConfig config_;
  UndockConfig out_;
  DockPose station_;
  bool contacts_back_ = true;
  std::optional<StationLegs> station_frame_;
  double facing_ = 0;
  int failures_ = 0;
};

// The charge on the contacts confirms docking (G19): the contact and a charge current of at least current A without a
// gap for time s, or at expected_below percent or more the contact alone for that long, as a charger stays off near
// full. Without either within timeout s of the debounced contact the try fails. In IDLE on the station charging
// counts as lost after lost_time s without the contact, or without the current that was there while the battery is
// below expected_below. Per platform, assumptions until measured (Tron phase 7, YardForce "Dock YF").
struct ChargeConfig {
  double current = 0.5;
  double time = 5.0;
  double timeout = 20.0;
  double expected_below = 95.0;
  double lost_time = 10.0;
};

// What one report of hw/status says about charging.
struct ChargeSample {
  // Monotonic s.
  double time = 0;
  // Debounced, with telemetry.
  bool contact = false;
  bool battery_ok = false;
  // A into the battery, 0 while it discharges.
  double charge_current = 0;
  int percentage = 0;
};

// The check after the debounced contact while docking, without ROS.
class ChargeCheck {
 public:
  enum class Result { kWaiting, kCharging, kFull, kContactLost, kNoCharge };

  ChargeCheck(const ChargeConfig& config, double start);

  // Every report while it runs; once it is not kWaiting the result stays.
  Result OnSample(const ChargeSample& sample);
  // The deadline also between reports.
  Result OnTick(double now);
  Result result() const {
    return result_;
  }
  // The largest charge current seen (A) and the last battery percentage, for the messages.
  double max_current() const {
    return max_current_;
  }
  int percentage() const {
    return percentage_;
  }

 private:
  ChargeConfig config_;
  double start_ = 0;
  // Since when the contact and the charge current, or a full battery, held without a gap.
  std::optional<double> good_since_;
  double max_current_ = 0;
  int percentage_ = 0;
  Result result_ = Result::kWaiting;
};

// IDLE on the station, without ROS: whether charging is confirmed there, by the rule of ChargeCheck, and when it
// stops. Unconfirmed after the start, after a loss, once the contact ended outside of IDLE, and where docking ended
// on the contacts without a confirmed charge; each confirmation from there is a DOCKED.
class ChargeWatch {
 public:
  // kConfirmed: DOCKED. kContactLost, kCurrentLost (it charged, and stopped below expected_below) and kNoCurrent
  // (unconfirmed for timeout s without a current, once per stay): CHARGING_LOST.
  enum class Change { kNone, kConfirmed, kContactLost, kCurrentLost, kNoCurrent };

  explicit ChargeWatch(const ChargeConfig& config = ChargeConfig()) : config_(config) {}

  // Every report in IDLE with telemetry; emergency: whether one is on.
  Change OnSample(const ChargeSample& sample, bool emergency);
  // Every report outside of IDLE or without telemetry: the stay is not watched; contact_ended unconfirms.
  void OnBusy(bool contact_ended);
  // Docking confirmed the charge.
  void Confirm();
  // Docking ended on the contacts without a confirmed charge; told: its lack was told already (NO_CHARGE_CURRENT).
  void Unconfirm(bool told);
  bool confirmed() const {
    return confirmed_;
  }

 private:
  // The stay ends: off the station.
  void Leave();

  ChargeConfig config_;
  bool confirmed_ = false;
  bool no_current_told_ = false;
  // The stay on the contacts in IDLE: since when, since when the contact or the current is gone and whether an
  // emergency came meanwhile, since when the rule of the check holds, whether a current flowed.
  bool on_station_ = false;
  double stay_since_ = 0;
  std::optional<double> contact_gone_since_;
  std::optional<double> current_gone_since_;
  bool emergency_ = false;
  std::optional<double> good_since_;
  bool charging_ = false;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_DOCKING_H
