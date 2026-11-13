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
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(UndockPath, poses, corridor, speed, stall_time)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DockConfig, approach_distance, speed, contact_speed, gate_distance, gate_lateral,
                                   gate_heading, overshoot, contact_timeout, retry_count, direct, direct_margin)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DockingLeg, kind, goal, poses, corridor, straight, reverse, speed, stall_time,
                                   end_tolerance, max_cross_track)
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
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LogicConfig, battery_empty_voltage, battery_full_voltage, battery_low_window,
                                   dock_charge_voltage, max_position_accuracy, recorder, mow_with_blade,
                                   mower_spinup_rpm, mower_spinup_timeout, blade_running_rpm, gps_wait_time,
                                   gps_timeout, battery_critical_voltage, battery_critical_time,
                                   battery_hot_temperature, motor_hot_temperature, motor_cold_temperature, rain_mode,
                                   rain_check_seconds, approach_distance, checkpoint_period, dock_contact_x,
                                   dock_max_position_accuracy, dock_reference_limit, undock, dock, charge, calibration,
                                   id_seed)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CalibrationStatus, problems, heading, lane_length, strip_length, strip_width,
                                   running)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CalibrationGoal, heading, lane_length)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CalibrationOutcome, reason, message, status, summary)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(UndockOutcome, undocked, for_drive, why)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CommandOutcome, invalid, invalid_field, accepted, release, result, why, data)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ReasonChange, from, to)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(JobStep, kind, area_id, goal, path, start_index)
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
static_assert(sizeof(UndockPath) == 64);
static_assert(sizeof(DockConfig) == 80);
static_assert(sizeof(DockingLeg) == 120);
static_assert(sizeof(ChargeConfig) == 40);
static_assert(sizeof(JobPose) == 32);
static_assert(sizeof(JobPath) == 32);
static_assert(sizeof(JobProgress) == 160);
static_assert(sizeof(HardwareSnapshot) == 72);
static_assert(sizeof(PoseSnapshot) == 40);
static_assert(sizeof(CalibrationConfig) == 32);
static_assert(sizeof(LogicConfig) == 440);
static_assert(sizeof(CalibrationStatus) == 64);
static_assert(sizeof(CalibrationGoal) == 16);
static_assert(sizeof(CalibrationOutcome) == 128);
static_assert(sizeof(UndockOutcome) == 88);
static_assert(sizeof(CommandOutcome) == 184);
static_assert(sizeof(JobStep) == 112);
static_assert(sizeof(RestartReport) == 160);
static_assert(sizeof(DriveCancel) == 80);
static_assert(sizeof(ReferenceCheck) == 8);
static_assert(sizeof(Notice) == 32);
static_assert(sizeof(Checkpoint) == 32);
static_assert(sizeof(RunChanged) == 104);
static_assert(sizeof(ScheduleRun) == 112);

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
      &L::StartRun, &L::DropTasks);
  static mowbite_logic::LogicView View(const L& core) {
    const auto state = core.RobotState();
    return {state.state, state.sub_state, state.emergency_reason, core.BladeWanted(), core.FollowerActive()};
  }
};

#endif  // MOWBITE_LOGIC_LOGIC_JOURNAL_H
