#ifndef MOWBITE_LOGIC_LOGIC_JOURNAL_H
#define MOWBITE_LOGIC_LOGIC_JOURNAL_H

#include <mowbite_journal/journaled.h>

#include "mowbite_logic/logic_core.h"

// The journal of the LogicCore (stream "logic"). A new field of a struct below needs its place in its line, the size
// check makes the build say so.

namespace mowbite_logic {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Reason, code, message, fields)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Event, type, details, id)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point2, x, y)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DockPose, x, y, heading)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(RecorderConfig, min_point_distance, max_position_accuracy, min_area,
                                                max_overhang, max_points, min_dock_distance, dock_backwards)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(MapEdit, kind, type, outline, obstacles, outline_driven_reason,
                                                obstacles_driven_reason, x, y, heading)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(JobArea, id, name, mowable)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(UndockConfig, distance, speed, stall_time, course_limit, course_distance, corridor)
// reverse only where set, so that journals of mowers leaving forwards stay as they were.
inline void to_json(nlohmann::json& j, const UndockPath& path) {
  j = nlohmann::json{
      {"poses", path.poses}, {"corridor", path.corridor}, {"speed", path.speed}, {"stall_time", path.stall_time}};
  if (path.reverse) j["reverse"] = true;
}
inline void from_json(const nlohmann::json& j, UndockPath& path) {
  j.at("poses").get_to(path.poses);
  j.at("corridor").get_to(path.corridor);
  j.at("speed").get_to(path.speed);
  j.at("stall_time").get_to(path.stall_time);
  path.reverse = j.value("reverse", false);
}
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DockConfig, approach_distance, speed, contact_speed, gate_distance, gate_lateral,
                                   gate_heading, overshoot, contact_timeout, retry_count, direct, direct_margin)
// frame only where it is the station frame, so that journals of legs in the map stay as they were.
inline void to_json(nlohmann::json& j, const DockingLeg& leg) {
  j = nlohmann::json{{"kind", leg.kind},
                     {"goal", leg.goal},
                     {"poses", leg.poses},
                     {"corridor", leg.corridor},
                     {"straight", leg.straight},
                     {"reverse", leg.reverse},
                     {"speed", leg.speed},
                     {"stall_time", leg.stall_time},
                     {"end_tolerance", leg.end_tolerance},
                     {"max_cross_track", leg.max_cross_track}};
  if (leg.frame != LegFrame::kMap) j["frame"] = leg.frame;
}
inline void from_json(const nlohmann::json& j, DockingLeg& leg) {
  j.at("kind").get_to(leg.kind);
  j.at("goal").get_to(leg.goal);
  j.at("poses").get_to(leg.poses);
  j.at("corridor").get_to(leg.corridor);
  j.at("straight").get_to(leg.straight);
  j.at("reverse").get_to(leg.reverse);
  j.at("speed").get_to(leg.speed);
  j.at("stall_time").get_to(leg.stall_time);
  j.at("end_tolerance").get_to(leg.end_tolerance);
  j.at("max_cross_track").get_to(leg.max_cross_track);
  leg.frame = j.value("frame", LegFrame::kMap);
}
// With defaults: guide and the marker place came later, journals from before stay readable.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(MarkerConfig, range, blind_distance, wait, cross_track, scale_limit,
                                                guide, marker_x, marker_y)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DockMarkerSample, time, state, valid, yaw_valid, reason)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DockPoseSample, time, x, y, heading, heading_valid, position_accuracy,
                                                heading_accuracy, speed)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ChargeConfig, current, time, timeout, expected_below, lost_time)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(JobPose, x, y, heading, kind)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(JobPath, is_outline, poses)
// With defaults: tasks came later, journals from before stay readable.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JobProgress, job_id, area, area_id, path, index, fingerprint, tasks,
                                                task, angle_offset_deg)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(HardwareSnapshot::Motor, ok, rpm, current, temperature)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(HardwareSnapshot, telemetry_ok, emergency, emergency_reason, rain, dock_contact,
                                   dock_contact_raw, hold, battery_ok, battery_voltage, battery_current,
                                   battery_percentage, battery_temperature, blade, left, right, time_s)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PoseSnapshot, x, y, heading, position_accuracy, orientation_accuracy,
                                   orientation_valid)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CalibrationConfig, max_position_accuracy, min_battery, status_timeout, timeout)
// With defaults: driving around objects came later, journals from before stay readable.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ObstacleConfig, free_wait, near_wait, near_ahead, stale, avoid_wait,
                                                avoid_still, avoid_sigma, avoid_clearance, avoid_pass_clearance,
                                                avoid_keep, avoid_speed)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ObstacleTrackSample, id, kind, confirmed, in_view, ahead, zone, x, y,
                                                radius, sigma)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObstacleSample, sight, tracks)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ObstacleNote, track_id, action, keep_out_r, left_out, area_id,
                                                area_name)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AvoidCircle, track_id, follows, x, y, r, pass_r, since)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(JobPiece, path, first, last)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CutAnswer, ok, code, message, pieces)
// With defaults: marker and obstacle came later, journals from before stay readable.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(LogicConfig, battery_empty_voltage, battery_full_voltage,
                                                battery_low_window, dock_charge_voltage, max_position_accuracy,
                                                recorder, mow_with_blade, mower_spinup_rpm, mower_spinup_timeout,
                                                blade_running_rpm, gps_wait_time, gps_timeout,
                                                battery_critical_voltage, battery_critical_time,
                                                battery_hot_temperature, motor_hot_temperature,
                                                motor_cold_temperature, rain_mode, rain_check_seconds,
                                                approach_distance, checkpoint_period, dock_contact_x,
                                                dock_max_position_accuracy, dock_accuracy_per_m, dock_reference_limit,
                                                undock, dock, marker, charge, calibration, obstacle, id_seed)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CalibrationStatus, problems, heading, lane_length, strip_length, strip_width,
                                   running)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CalibrationGoal, heading, lane_length)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CalibrationOutcome, reason, message, status, summary)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(UndockOutcome, undocked, for_drive, why)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CommandOutcome, invalid, invalid_field, accepted, release, result, why, data)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ReasonChange, from, to)
// With defaults: paths and circles came later, journals from before stay readable.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JobStep, kind, area_id, goal, paths, circles, path, start_index)
// With defaults: stalled came later, journals from before stay readable.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(RestartReport, cause, detail, crashed_at, state_before, job_id_before,
                                                hold, stalled)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DriveCancel, why)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ReferenceCheck, heading)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Notice, line)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Checkpoint, text)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RunChanged, run_id, phase, end_code, detail)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ScheduleRun, run_id, area_ids, pass_over, restart, has_angle, angle_offset_deg,
                                   dark_allowed, until_sunset, end_in_s)

// An effect in the export of a journal: the name of its type and its fields; for json and ordered_json.
template <typename Json>
void to_json(Json& j, const Effect& effect) {
  std::visit(
      [&j](const auto& value) {
        if constexpr (std::is_empty_v<std::decay_t<decltype(value)>>) {
          j = Json::object();
        } else {
          j = value;
        }
      },
      effect);
  j["effect"] = EffectName(effect);
}

static_assert(sizeof(Reason) == 80);
static_assert(sizeof(Event) == 80);
static_assert(sizeof(RecorderConfig) == 56);
static_assert(sizeof(MapEdit) == 168);
static_assert(sizeof(JobArea) == 72);
static_assert(sizeof(UndockConfig) == 48);
static_assert(sizeof(UndockPath) == 72);
static_assert(sizeof(DockConfig) == 80);
static_assert(sizeof(DockingLeg) == 120);
static_assert(sizeof(ChargeConfig) == 40);
static_assert(sizeof(JobPose) == 32);
static_assert(sizeof(JobPath) == 32);
static_assert(sizeof(JobProgress) == 160);
static_assert(sizeof(HardwareSnapshot) == 72);
static_assert(sizeof(PoseSnapshot) == 40);
static_assert(sizeof(CalibrationConfig) == 32);
static_assert(sizeof(LogicConfig) == 600);
static_assert(sizeof(MarkerConfig) == 64);
static_assert(sizeof(DockMarkerSample) == 48);
static_assert(sizeof(DockPoseSample) == 56);
static_assert(sizeof(CalibrationStatus) == 64);
static_assert(sizeof(CalibrationGoal) == 16);
static_assert(sizeof(CalibrationOutcome) == 128);
static_assert(sizeof(UndockOutcome) == 88);
static_assert(sizeof(CommandOutcome) == 184);
static_assert(sizeof(JobStep) == 160);
static_assert(sizeof(RestartReport) == 160);
static_assert(sizeof(DriveCancel) == 80);
static_assert(sizeof(ReferenceCheck) == 8);
static_assert(sizeof(Notice) == 32);
static_assert(sizeof(Checkpoint) == 32);
static_assert(sizeof(RunChanged) == 104);
static_assert(sizeof(ObstacleConfig) == 88);
static_assert(sizeof(ObstacleTrackSample) == 96);
static_assert(sizeof(ObstacleSample) == 32);
static_assert(sizeof(ObstacleNote) == 120);
static_assert(sizeof(ScheduleRun) == 112);
static_assert(sizeof(AvoidCircle) == 48);
static_assert(sizeof(JobPiece) == 24);
static_assert(sizeof(CutAnswer) == 96);

// What the logic decides: state, sub_state, the reason bits of an emergency, blade wanted, follower driving.
struct LogicView {
  std::string state;
  std::string sub_state;
  int emergency_reason = 0;
  bool blade_wanted = false;
  bool follower_active = false;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LogicView, state, sub_state, emergency_reason, blade_wanted, follower_active)

}  // namespace mowbite_logic

template <>
struct mowbite_journal::CoreTraits<mowbite_logic::LogicCore> {
  using L = mowbite_logic::LogicCore;
  static constexpr const char* kName = "logic";
  using Config = mowbite_logic::LogicConfig;
  MOWBITE_JOURNAL_METHODS(
      &L::OnHardwareStatus, &L::OnHardwareTimeout, &L::OnPose, &L::OnPoseTimeout, &L::OnGnssFix, &L::OnPositionJump,
      &L::OnAction, &L::Command, &L::OnMapStatus, &L::OnMapEditResult, &L::OnTeleop, &L::OnCapabilities, &L::StartDrive,
      &L::RefuseDrive, &L::OnDriveEnded, &L::OnCalibrationStatus, &L::StartCalibration, &L::RefuseCalibration,
      &L::OnCalibrationSign, &L::OnCalibrationFeedback, &L::OnCalibrationEnded, &L::AbortCalibration,
      &L::TickCalibration, &L::OnMowMap, &L::OnMapDock, &L::OnDrivableAreas, &L::SetUndockDistance,
      &L::CheckDockBeforeDrive, &L::OnDockCheck, &L::OnChargeOff, &L::OnUndockEnded, &L::AbortUndocking,
      &L::StartDocking, &L::TakeDockingLeg, &L::OnDockingLegEnded, &L::SetDockingSettings, &L::SetHomeSettings,
      &L::RestoreJob, &L::RefuseStart, &L::RefuseReset, &L::OnReleaseAnswered, &L::GeometryChanged, &L::SetHold,
      &L::SetGpsSettings, &L::NextJobStep, &L::OnJobPlanned, &L::OnJobPlanFailed, &L::OnJobProgress, &L::OnLegEnded,
      &L::OnTick, &L::OnBladeRefused, &L::TakeOverlayChanged, &L::SetRestartHold, &L::ReportRestart, &L::OnStopRelease,
      &L::TakeEffects, &L::SetUnwritableFile, &L::AbandonStart, &L::SetTurningRadius,
      &L::RecordedBodyKept, &L::OnWheelOdom, &L::SetSunsetSettings, &L::OnClock, &L::OnDatum,
      &L::StartRun, &L::DropTasks, &L::OnDockMarker, &L::OnDockPose,
      &L::SetShapeOutside, &L::OnSetupMotion, &L::TickSetup, &L::OnSetupBlade, &L::OnObstacleStatus,
      &L::SetObstacleMode, &L::OnCutAnswer);
  static mowbite_logic::LogicView View(const L& core) {
    const auto state = core.RobotState();
    return {state.state, state.sub_state, state.emergency_reason, core.BladeWanted(), core.FollowerActive()};
  }

  // Fields of details and reasons in unix s.
  static inline const std::vector<std::string> kUnixKeys = {"until", "crashed_at"};
  template <typename O>
  static void AnonymizeReason(const O& o, mowbite_logic::Reason& why) {
    why.message = o.ShiftDates(why.message);
    o.ShiftWallIn(&why.fields, kUnixKeys);
  }

  // Every call with a place or the wall clock, for a copy of a journal without its place (mowbite_replay anonymize).
  template <typename A>
  static void Anonymize(A& a) {
    using mowbite_logic::Reason;
    a.template Args<&L::OnDatum>([](const auto& o, double& latitude, double& longitude) {
      latitude = o.latitude;
      longitude = o.longitude;
    });
    a.template Args<&L::OnClock>(
        [](const auto& o, bool&, std::string&, double& unix_minus_mono_s) { unix_minus_mono_s += o.shift_s(); });
    a.template Args<&L::SetHold>([](const auto& o, int64_t& until) {
      if (until > 0) until += o.shift_s();
    });
    a.template Args<&L::ReportRestart>([](const auto& o, mowbite_logic::RestartReport& report) {
      if (report.crashed_at > 0) report.crashed_at += o.shift_s();
    });
    a.template Args<&L::Command>([](const auto& o, std::string&, mowbite_logic::json& params,
                                    Reason& node_problem) {
      o.ShiftWallIn(&params, kUnixKeys);
      AnonymizeReason(o, node_problem);
    });
    a.template Result<&L::Command>([](const auto& o, mowbite_logic::CommandOutcome& outcome, std::string&,
                                      mowbite_logic::json&, Reason&) {
      AnonymizeReason(o, outcome.why);
      o.ShiftWallIn(&outcome.data, kUnixKeys);
    });
    a.template Args<&L::StartRun>([](const auto& o, mowbite_logic::ScheduleRun& run, Reason& node_problem, double&) {
      run.run_id = o.ShiftDates(run.run_id);
      AnonymizeReason(o, node_problem);
    });
    a.template Result<&L::StartRun>(
        [](const auto& o, Reason& why, mowbite_logic::ScheduleRun&, Reason&, double&) { AnonymizeReason(o, why); });
    a.template Args<&L::DropTasks>(
        [](const auto& o, std::string& run_id, std::vector<std::string>&) { run_id = o.ShiftDates(run_id); });
    a.template Result<&L::DropTasks>(
        [](const auto& o, Reason& why, std::string&, std::vector<std::string>&) { AnonymizeReason(o, why); });
    a.template Result<&L::StartDrive>([](const auto& o, Reason& why, double&, double&) { AnonymizeReason(o, why); });
    a.template Result<&L::StartCalibration>([](const auto& o, Reason& why, double&) { AnonymizeReason(o, why); });
    a.template Result<&L::StartDocking>([](const auto& o, Reason& why) { AnonymizeReason(o, why); });
    a.template Result<&L::OnDockCheck>(
        [](const auto& o, Reason& why, bool&, double&, double&, float&, std::string&) { AnonymizeReason(o, why); });
    a.template Result<&L::TakeEffects>([](const auto& o, std::vector<mowbite_logic::Effect>& effects) {
      for (auto& effect : effects) {
        if (auto* event = std::get_if<mowbite_logic::Event>(&effect)) {
          o.ShiftWallIn(&event->details, kUnixKeys);
        } else if (auto* run = std::get_if<mowbite_logic::RunChanged>(&effect)) {
          run->run_id = o.ShiftDates(run->run_id);
        } else if (auto* notice = std::get_if<mowbite_logic::Notice>(&effect)) {
          notice->line = o.ShiftDates(notice->line);
        }
      }
    });
  }
};

#endif  // MOWBITE_LOGIC_LOGIC_JOURNAL_H
