#ifndef MOWBITE_LOGIC_LOGIC_CORE_H
#define MOWBITE_LOGIC_LOGIC_CORE_H

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "mowbite_api/catalog.h"
#include "mowbite_logic/area_recorder.h"
#include "mowbite_logic/calibration_part.h"
#include "mowbite_logic/core_access.h"
#include "mowbite_logic/docking.h"
#include "mowbite_logic/home_watch.h"
#include "mowbite_logic/logic_types.h"
#include "mowbite_logic/marker_dock.h"
#include "mowbite_logic/mowing_job.h"
#include "mowbite_logic/reasons.h"
#include "mowbite_logic/release_part.h"
#include "mowbite_logic/setup_part.h"
#include "mowbite_logic/undocking.h"

namespace mowbite_logic {

// Mower states and actions: IDLE, AREA_RECORDING (remote control and recording of areas), driving to a pose,
// UNDOCKING, MOWING and PAUSED, DOCKING, emergency and sensors.
class LogicCore {
 public:
  explicit LogicCore(const LogicConfig& config);

  void OnHardwareStatus(const HardwareSnapshot& hw);
  // hw/status stopped coming.
  void OnHardwareTimeout();
  void OnPose(const PoseSnapshot& pose);
  // localization/pose stopped coming; events keep the last position.
  void OnPoseTimeout();
  // An RTK fixed position of the GNSS antenna (gps/absolute_pose). The course probe of undocking measures the way the
  // mower moves with these: the fused pose starts from the very heading of the station the probe checks.
  void OnGnssFix(double x, double y);
  // The localization moved its estimate onto GPS again by distance_m, the event is at the new position.
  void OnPositionJump(double to_x, double to_y, double distance_m, bool after_outliers);
  // hw/wheel_odom, which the node passes only in AREA_RECORDING: vx (m/s, negative backwards) since the one before
  // gives the way the recorder tells reversing by. More than RecorderConfig::kWheelGap after it, a value that is no
  // number, or none for that long (OnTick), is a break of the driven track.
  void OnWheelOdom(double vx, double now_s);
  bool AreaRecording() const;
  // The command the action stands for, as its RPC without client; kUnknown for an action that is not the logic's.
  ActionResult OnAction(const std::string& action_id);
  // A command by its RPC name with its params (client among them). node_problem: what only the node knows against a
  // start (planner, follower, the last drive still stopping), taken after the logic's own reasons. The events of this
  // step carry client.
  CommandOutcome Command(const std::string& name, const json& params, const Reason& node_problem = Reason());
  // The answer to an accepted command: state and sub_state as robot_state has them now, then fields.
  json CommandAnswer(const json& fields = json::object()) const;
  // The data of a refusal: event (the group of the reason), the reason with its rating and fields, state, sub_state
  // and event_id when the refusal went out as that event too.
  json RefusalData(const std::string& group, const Reason& why, const std::string& event_id = "") const;
  // Recording starts and saves only while the map fits the datum in use, or is still empty and takes it.
  void OnMapStatus(MapDatum datum, const std::string& reason);
  // The answer of mowbite_map to a MapEdit a recording asked for.
  void OnMapEditResult(bool ok, const std::string& id, const std::string& code, const std::string& message);
  // Remote control reaches the wheels only while recording an area, like in OpenMower. After an emergency or a state
  // change it waits for a zero command or a pause of kTeleopQuietS, so a held joystick does not move the mower.
  // now_s is a monotonic clock.
  std::optional<std::pair<double, double>> OnTeleop(double vx, double vz, double now_s);

  // hw/capabilities: whether the platform has a safety measurement (without one only remote control, no driving) and
  // whether its driver can stop charging (charge_stop; without it undocking leaves out that step).
  void OnCapabilities(bool safety_measured, bool charge_stop = true);
  // Why a drive cannot start now (BUSY, EMERGENCY, NO_SAFETY_MEASUREMENT, position), empty if it can.
  Reason DriveProblem() const;
  // Starts a drive to (x, y): empty if it may start, else why not (event DRIVE_REFUSED). Shown as IDLE with the sub
  // state DRIVING; the node plans the way and hands it to the follower.
  Reason StartDrive(double x, double y);
  // A drive refused by the node (map, planner or follower missing): event DRIVE_REFUSED.
  void RefuseDrive(const Reason& why);
  // The drive is over: arrived, and the reason (DONE, ALREADY_THERE or why not).
  void OnDriveEnded(bool arrived, const Reason& reason);
  bool Driving() const;
  // The commands of the follower reach the wheels only while driving, or mowing on a leg and not paused, and without
  // emergency; teleop only while recording. So safety/drive has one source at a time.
  bool FollowerActive() const;

  // The calibration drive (mowbite_calibration): IDLE with the sub state CALIBRATING. It starts only on
  // calibration.start, never by itself; the program drives on calibration/cmd_vel, which reaches the wheels only while
  // calibrating and without emergency (SOURCE_CALIBRATION), and never with the blade. It ends with one zero at an
  // emergency, without a pose or a safety measurement, on a low battery, on calibration.abort (the mower stays where
  // it is) and after CalibrationConfig::timeout without a sign of the program; jumps of the position do not end it.
  void OnCalibrationStatus(const CalibrationStatus& status, double now_s);
  // Every reason against a start now, the logic's first, then the program's (calibration.check); empty: it may start.
  std::vector<Reason> CalibrationProblems(double now_s) const;
  // The program's status while it counts, none without one.
  std::optional<CalibrationStatus> CalibrationStatusNow(double now_s) const;
  // calibration.start: empty when CALIBRATING began, else the first reason against it (also CALIBRATION_REFUSED).
  Reason StartCalibration(double now_s);
  // A start refused by the node (no program, the last drive still stopping): CALIBRATION_REFUSED. After a start the
  // node hands the CalibrationGoal to calibration/run.
  void RefuseCalibration(const Reason& why);
  // A sign of the program: the goal taken, or feedback, which also tells the phase (event CALIBRATION when it changes).
  void OnCalibrationSign(double now_s);
  void OnCalibrationFeedback(const std::string& phase, int index, double now_s);
  // The program ended the drive: CALIBRATED or CALIBRATION_FAILED, then IDLE.
  void OnCalibrationEnded(const CalibrationOutcome& outcome);
  // calibration.abort: true when a drive was running; it stops here without the program and the mower stays there.
  bool AbortCalibration(const Reason& why);
  // The watchdog of the program and the battery while it drives; true when the drive ended now.
  bool TickCalibration(double now_s);
  bool Calibrating() const;
  bool CalibrationActive() const;

  // A step of the setup program that moves the wheels (setup/motion of mowbite_setup): IDLE with the sub state SETUP.
  // It begins only from IDLE without a sub state, off the charging contacts, without emergency and without a hold; the
  // program drives on setup/cmd_vel, which reaches safety/drive only in SETUP and without emergency (SOURCE_SETUP). An
  // emergency does not end it (the step stops the wheels on purpose); it ends with one zero when setup/motion ends or
  // stays away kSetupSignTimeout s, on the charging contacts and with a hold. Empty when SETUP holds now, else why not.
  Reason OnSetupMotion(bool active, const std::string& step, double now_s);
  // The watchdog of setup/motion; true when SETUP ended now.
  bool TickSetup(double now_s);
  bool InSetup() const;
  bool SetupActive() const;
  // The blade the step of setup/motion asks for (SetupMotion BLADE_*): it turns in SETUP without a safety measurement,
  // otherwise under the conditions of BladeWanted; a refusal holds it off until the step asks for another blade.
  void OnSetupBlade(const std::string& step, int blade);
  // The blade in SETUP on one lease that is not renewed (BLADE_ONCE).
  bool SetupBladeOnce() const;

  // The mow areas of the map and why mowing may not plan from it (empty if it may).
  void OnMowMap(std::vector<JobArea> areas, const Reason& problem);
  // The docking station of the map, none without one.
  void OnMapDock(const std::optional<DockPose>& dock);
  // Where base_link may end leaving the station and where docking begins: the active mow and navigation areas, outside
  // of the obstacles; problem: why the map gives none (MAP_*), empty if it does.
  void OnDrivableAreas(std::vector<Ring> drivable, std::vector<Ring> obstacles, const Reason& problem = Reason());
  // undock_distance of params.set.
  void SetUndockDistance(double distance);
  // The driver reports a debounced contact with the docking station.
  bool DockContact() const;
  // On the docking station mowing (start_mowing) and drives start only after the reference check (G24): the node sets
  // the heading of the ReferenceCheck with localization/set_heading and hands the answer to OnDockCheck. A drive to
  // (x, y) asks for its check here, after DriveProblem found nothing.
  void CheckDockBeforeDrive(double x, double y);
  bool DockCheckPending() const;
  // The answer of localization/set_heading: base_link with the heading of the station and its accuracy; ok false
  // without a recent fix or without an answer (message says which). Empty when the check passed and undocking
  // starts, else why mowing or the drive does not start, which also goes out as MOWING_REFUSED or DRIVE_REFUSED.
  Reason OnDockCheck(bool ok, double x, double y, float accuracy, const std::string& message);
  // UNDOCKING: first charging off where the driver can stop it (ChargeOff, whose answer goes to OnChargeOff), then
  // the UndockPath to the follower, whose end goes to OnUndockEnded. After UNDOCKED mowing or the drive starts like
  // away from the station. Charging is allowed again (ChargeOn) when undocking ends otherwise, as the mower may still
  // stand on the contacts. The UndockOutcome tells the node how it ended. Docking uses ChargeOff, OnChargeOff and
  // ChargeOn the same way before it drives off the contacts.
  bool Undocking() const;
  bool UndockingForDrive() const;
  void OnChargeOff(bool ok, const std::string& message);
  void OnUndockEnded(LegEnd end, const Reason& reason);
  // Stops undocking without UNDOCKING_FAILED, like abort_undocking (also logic/stop_drive for a drive).
  void AbortUndocking(const Reason& why);
  // DOCKING: back onto the station of the map after JOB_COMPLETE, abort_mowing or a reason of HomeWatch, or on
  // docking.start, where the platform has charging contacts (CanDock); else IDLE as before. Each but JOB_COMPLETE begins with
  // the event DOCKING and its reason. The node hands the leg of TakeDockingLeg to the follower,
  // once and when nothing else is on its way, and its end to OnDockingLegEnded. On the way onto the station the first
  // raw contact stops the mower (one zero command, G18); the debounced one, also at the end of a leg that failed,
  // starts the check of the charge (ChargeCheck), and only a confirmed charge ends in DOCKED. A failed try drives out
  // to the approach point, off the contacts with charging off first where the driver can stop it, then DOCKING_RETRY
  // or after docking_retry_count DOCKING_FAILED; the last try without a charge current (NO_CHARGE_CURRENT) leaves the
  // mower on the contacts, and a stable charge current there later still ends in DOCKED. A bad position waits like
  // mowing, at most gps_timeout, then DOCKING_FAILED NO_POSITION, NO_HEADING or POSITION_ACCURACY. An emergency and
  // abort_docking end it without DOCKING_FAILED. In IDLE on the contacts CHARGING_LOST tells when the contact or the
  // charge current goes away (ChargeConfig); the mower stays.
  // Where the planner gives no answer for the way to the point in front of the station (PlannerSilent as the reason
  // of the approach), the try takes the straight way (DockConfig::direct): free for the body in every heading, with
  // direct_margin and the accuracy of a good position to spare (CheckDirectRoute), it is the event DOCKING_DIRECT and
  // the leg kDirect, then the line as after the approach. Not free, or failed on the way, the try fails as
  // approach_failed with cause and direct_blocked; with the switch off only with cause.
  bool CanDock() const;
  bool Docking() const;
  // Why docking.start cannot start now, empty if it can: BUSY, NO_CHARGE_CONTACTS, ALREADY_THERE on the contacts,
  // EMERGENCY or NO_SAFETY_MEASUREMENT, the position, MAP_*, NO_DOCK, DOCK_UNREACHABLE.
  Reason DockingProblem() const;
  // docking.start: empty when DOCKING began, else why not (also as DRIVE_REFUSED).
  Reason StartDocking();
  std::optional<DockingLeg> TakeDockingLeg();
  void OnDockingLegEnded(LegEnd end, const Reason& reason);
  // docking.test (G43): docking by the dock marker alone, from where the mower stands in front of the station, without
  // map or position; offered only where the platform has a marker (MarkerSource). Refused like docking.start, and with
  // MARKER_UNAVAILABLE without a marker. DOCKING with the reason TEST, then every leg in the station frame
  // (FRAME_DOCK, SOURCE_DOCK): a sight first (MarkerWanted asks hw/dock_marker, localization/dock_pose good within
  // marker_wait, the direction from the marker itself), the start checked (on the line or at the approach point,
  // facing it, the straight way out ending on the line), out to the approach point, the line, the gate with the
  // pose of the marker and the scale of its way against the wheels, kClose up to marker_blind_distance, the blind
  // contact, the charge. The marker lost on the way: the mower stands; a good pose back within marker_wait s lets it
  // go on after gps_wait_time s of it; a try failed by the marker carries cause (MARKER_UNAVAILABLE, MARKER_NOT_SEEN,
  // MARKER_LOST) and detail. Without a good pose of the marker nothing drives: docking ends (DOCKING_FAILED).
  // DOCKED, DOCKING_RETRY and DOCKING_FAILED of a docking carry guide: marker or position.
  // With dock_guide prefer or require (Guided) docking.start and going home use the marker as well: the approach over
  // the areas ends at most marker_range in front of the station, from kMarkerRequestDistance m before that point
  // docking asks for the marker (with DockHeadingHint for the start of the estimate), and there each try chooses its
  // way: a sight as above, where the marker may come without a direction of its own. While the map has a good position
  // the station by the marker has to fit the recorded one, at the gate also its heading (MARKER_MISMATCH): else the
  // marker guides nothing more and docking ends where the mower stands, never by the position. Where the source cannot
  // measure, prefer docks this try by the position (DOCKING_WITHOUT_MARKER), require fails it. A try by the marker that
  // failed drives out in the station frame, else in the map with a good position, else docking ends.
  bool MarkerSource() const;
  bool Guided() const;
  bool MarkerWanted() const;
  // The heading of base_link in the station frame by the map and the recorded station, for the start of the estimate
  // without the direction of the marker: while docking asks for the marker with a good position, not in docking.test.
  std::optional<double> DockHeadingHint() const;
  // The follower drives a leg in the station frame now: its commands go out as SOURCE_DOCK.
  bool DockFollowerActive() const;
  // hw/dock_marker and localization/dock_pose, which the node passes only while MarkerWanted.
  void OnDockMarker(const DockMarkerSample& sample);
  void OnDockPose(const DockPoseSample& pose);
  // docking_approach_distance and docking_retry_count of params.set, for the next docking.
  void SetDockingSettings(double approach_distance, int retry_count);
  // m from base_link to the farthest point of the body (Geometry::TurningRadius), once the dimensions are loaded and
  // after every change; until then no straight way counts as free.
  void SetTurningRadius(double radius);
  // rain_mode, rain_check_seconds and the temperatures of the blade motor of params.set, for the next run.
  void SetHomeSettings(int rain_mode, double rain_check_seconds, double motor_hot_temperature,
                       double motor_cold_temperature);
  // dock_at_sunset of params.set: a run goes home at the first sunset after its start or after the user resumed it.
  // At once in a run: off drops its deadline, on reckons the first sunset from now.
  void SetSunsetSettings(bool dock_at_sunset);
  // time/clock: whether the wall clock is safe, why not, and wall minus monotonic s when it came.
  void OnClock(bool safe, const std::string& reason, double unix_minus_mono_s);
  // gps/datum, the datum in use (deg): where the sun is reckoned.
  void OnDatum(double latitude, double longitude);
  // The checkpoint read at the start.
  void RestoreJob(const JobProgress& progress);
  // A plan of the schedule starts (S1, S3, S7): empty when its announcement began, else why not. INVALID (no event)
  // for a list the logic does not take; any other refusal is MOWING_REFUSED with source schedule. Checked in this
  // order: busy, hold, on the contacts (NOT_DOCKED), the clock (CLOCK_UNSAFE), the start problems of the areas left,
  // without dark_allowed the sun (NO_POSITION without datum, DARK), then node_problem. kAnnounceS later (OnTick) the
  // checks run again, then the reference check and undocking as for mowing.start. An emergency, a hold, leaving the
  // contacts and any command to the logic cancel the announcement (SCHEDULE_START_CANCELED); the command then runs.
  // logic/run (RunChanged) follows the run until the mower is idle again; nothing of it is kept over a restart (S5).
  Reason StartRun(const ScheduleRun& run, const Reason& node_problem, double now_s);
  // The schedule pauses areas of its run run_id while it mows, empty for all: empty when taken, else why not. The
  // current area among them is skipped (AREA_SKIPPED SCHEDULE); with none left the run ends with SCHEDULE pause and
  // the job stays open, going home as for a reason of HomeWatch, else idle where it is.
  Reason DropTasks(const std::string& run_id, const std::vector<std::string>& area_ids);
  // The angle of the lanes the open job asks of the planner (the one of the plan that began it), none without.
  std::optional<double> PlanAngle() const;
  // Why mowing of these areas (none for all) cannot start now, empty if it can.
  Reason StartProblem(const std::vector<std::string>& tasks = {}) const;
  // A start refused by the node (planner or follower missing): event MOWING_REFUSED.
  void RefuseStart(const Reason& why);
  // reset_emergency did not release: EMERGENCY_RESET_REFUSED with the reason of the driver, of S2 or of the node;
  // returns the id of the event.
  std::string RefuseReset(const Reason& why);
  const ReleaseOutcome& LastReleaseOutcome() const {
    return release_.release_outcome();
  }
  // The restart hold (G35): while it stands the logic counts as in emergency with UNEXPECTED_RESTART, so nothing that
  // drives or mows starts, whatever the driver reports. The node sets it at the start and clears it on emergency.reset
  // once the file is gone.
  void SetRestartHold(bool held);
  bool RestartHeld() const {
    return restart_hold_;
  }
  // The event UNEXPECTED_RESTART, rated by cause and hold.
  void ReportRestart(const RestartReport& report);
  // The job of mowing, finished or not; empty without one.
  const std::string& JobId() const;
  // The safety node released after emergency.reset or STOP (now_s monotonic): the end of the emergency that follows
  // within kReleaseSourceS carries the source, rpc with the client of the command or button. Without that end the
  // tick after kReleaseSourceS refuses it with STILL_ACTIVE and the bits that stay.
  void OnReleaseAnswered(double now_s);
  // STOP was let go after the gesture of the driver (hw/button LONG): true in an emergency, which the node then
  // releases like emergency.reset with source button; false outside of one.
  bool OnStopRelease();
  // robot.geometry.set changed the dimensions: event GEOMETRY_CHANGED with the keys that are other now and the keys of
  // the body below the platform's value; with the latter it is a warning with the reason BELOW_DEFAULT, else info. It
  // stops nothing, so it has no resume. With parts of SetShapeOutside it is a warning as well and names them in
  // outside_shape, with the reason SHAPE_OUTSIDE unless BELOW_DEFAULT counts.
  void GeometryChanged(const std::vector<std::string>& changed, const std::vector<std::string>& below_default,
                       int version);
  // The parts of robot_shape the body leaves out since the last robot.geometry.set, as the JSON list
  // [{"part", "by"}] (by in m); "[]" for none.
  void SetShapeOutside(const std::string& parts);
  // map/set_recorded_body failed after robot.geometry.set changed the body: event RECORDED_BODY_KEPT, the areas
  // recorded here keep the old body. areas: how many carry the id of this installation, below 0 when not known.
  void RecordedBodyKept(int areas, const std::string& code, const std::string& message);
  bool Mowing() const;
  // IDLE without a drive: no recording, no job (also paused). Settings change only then.
  bool Idle() const;
  // The hold of an update (system.hold), until in unix s, 0 for none. While it or the hold the driver reports in
  // hw/status lasts, nothing starts the wheels or the blade: start_mowing, continue, start_area_recording, drives and
  // docking.start are refused with the reason HOLD.
  void SetHold(int64_t until);
  // until of the logic's own hold, 0 without one.
  int64_t HoldUntil() const {
    return hold_until_;
  }
  bool Held() const;
  // HOLD while held, with until when the logic holds; else STORAGE_NOT_WRITABLE while a file of the restart hold
  // cannot be written; else empty. mowing.continue checks only the hold: PAUSED holds on the disk already.
  Reason HoldProblem() const;
  // The file in data_dir of the restart hold (os-session.json, restart-hold.json) that the node cannot write, empty
  // once it can again. While one cannot, every start from IDLE is refused with STORAGE_NOT_WRITABLE; the event STORAGE
  // tells it once each time it begins.
  void SetUnwritableFile(const std::string& file);
  // The record of a start from IDLE did not reach the disk (G35), and the node carried out nothing of the start: the
  // start ends at once with the refusal of its kind and STORAGE_NOT_WRITABLE, and the mower is IDLE again.
  void AbandonStart(const std::string& file);
  // A hold is granted only in IDLE with nothing on its way, not even the reference check on the docking station.
  bool HoldAllowed() const;
  // GPS settings of params.set for the position, the recording, and mowing or docking on after a bad position.
  void SetGpsSettings(double max_position_accuracy, double gps_wait_time, double gps_timeout);
  // Called whenever the node has nothing on its way for the job; the step it returns is on its way until its result
  // comes back. now_s is a monotonic clock.
  JobStep NextJobStep(double now_s);
  // The plan of the area asked for by kPlan, or why there is none: PLANNING_FAILED with the answer of the planner,
  // PLANNER_TIMEOUT or PLANNER_MISSING without one.
  void OnJobPlanned(std::vector<JobPath> paths, const std::string& fingerprint);
  void OnJobPlanFailed(const Reason& why);
  // The follower got to this pose of the path of kFollow.
  void OnJobProgress(size_t index);
  // The approach or path ended; index is the pose of the path it got to (kFollow only), reason why it ended. An
  // approach or a plan the planner gave no answer for (PlannerSilent) is asked for again, up to kPlannerRetries times
  // in a row and never after a plan that timed out. Then the job ends, kept for the next start, and the mower goes
  // home with that reason at the event DOCKING; without a way home, or in a pause that waits for the user, mowing
  // pauses as for a failed approach or plan.
  // After a skip an approach that ended with kNoWay does not pause while the mower stands on the path it left and
  // poses of it are left: the next step is kSkippedPath, a piece of that path with the blade off, and after it the
  // approach is asked for again. For kSkippedPath index is the pose of that path the mower got to.
  void OnLegEnded(LegEnd end, size_t index, const Reason& reason);
  // Timers of the job: GPS back, going home (HomeWatch), checkpoint period. While mowing and not waiting for the user
  // the first reason that holds sends the mower home with the event DOCKING; without a way home only the battery
  // ends mowing (BATTERY_LOW, IDLE). A pause that waits for the user (manual, emergency, fault) never goes home: there
  // only the battery ends it the same way.
  void OnTick(double now_s);
  // Whether the blade should turn now: mowing, not paused, a mow segment (or spinning up for one), mow_with_blade, a
  // safety measurement, telemetry and not on the dock; or in SETUP as the setup program asks, without emergency, with
  // telemetry and not on the dock. The node keeps the lease of hw/blade alive only while it is.
  bool BladeWanted() const;
  // hw/blade refused while the blade was wanted: mowing pauses, with reason bits of the answer as for that emergency;
  // in SETUP the blade stays off for the step, which sees it.
  void OnBladeRefused(const std::string& message, int emergency_reason = 0);

  RobotStateData RobotState() const;
  std::vector<ActionGroup> Actions() const;
  std::vector<SensorDefinition> Sensors() const;
  std::vector<SensorReading> Readings() const;

  // Everything the node has to do since the last call, oldest first; a checkpoint with the progress now. The next step
  // of the job (NextJobStep), of docking (TakeDockingLeg) and the overlay stay queries.
  std::vector<Effect> TakeEffects();
  bool Emergency() const;
  // The lines of map_overlay/json, none outside of AREA_RECORDING.
  json MapOverlay() const;
  bool TakeOverlayChanged();
  const AreaRecorder& recorder() const {
    return recorder_;
  }

 private:
  // The only way of the parts into the core.
  friend class CoreFacts;
  friend class CoreAccess;

  void SetState(State state);
  // The command after its params were checked: empty when accepted, else why not; extra fields of the answer go to
  // out->result.
  Reason RunCommand(const mowbite_api::Command& command, const json& params, const Reason& node_problem,
                    CommandOutcome* out);
  Reason RecordingCommand(const mowbite_api::Command& command, const json& params, CommandOutcome* out);
  // WRONG_STATE with state and sub_state of robot_state.
  Reason WrongState() const;
  // Moves the events of the recorder into the events of the logic, at the current position.
  void PullRecorder();
  // A break of the driven track while OnWheelOdom does not come.
  void TickRecording(double now_s);
  // Returns the id of the event.
  std::string AddEvent(const std::string& type, json details);
  // 32 letters and digits from id_seed: the ids of jobs, sessions and events.
  std::string NewId();
  // report: called for a report of the driver or its absence, whose first one only sets the state.
  void UpdateEmergency(bool report = true);
  int EmergencyReason() const;
  static const char* StateName(State state);
  std::string CurrentStateName() const;
  // Each empty if nothing stands in the way: BUSY; EMERGENCY or NO_SAFETY_MEASUREMENT; NO_POSITION,
  // POSITION_ACCURACY or NO_HEADING (on the docking station the stricter accuracy of the reference check and no
  // heading); NO_DOCK or DOCK_UNREACHABLE; what else keeps mowing or a drive from starting, busy or not.
  Reason Busy() const;
  Reason Unsafe() const;
  Reason AccuracyReason(float accuracy, double limit) const;
  Reason PositionProblem() const;
  Reason DockProblem() const;
  // Of these areas, none for all.
  Reason MowProblem(const std::vector<std::string>& tasks) const;
  Reason MoveProblem() const;
  // DOCK_UNREACHABLE when the way out from base_link at (x, y) on the station ends outside of the drivable areas.
  Reason UndockEndProblem(double x, double y) const;
  // DOCK_UNREACHABLE when the line onto the station begins outside of the drivable areas.
  Reason ApproachProblem() const;
  double DockAccuracyLimit() const;
  // The limit for a pose from the dock marker at (x, y) in the station frame.
  double MarkerAccuracyLimit(double x, double y) const;
  // The heading of the mower on the docking station.
  double DockedHeading() const;
  // The contacts lie behind base_link: the mower backs onto the station and leaves it forwards.
  bool ContactsBack() const;
  // "straight ahead" or "straight back": the way off the station, for the messages.
  const char* LeaveWords() const;
  void BeginDockCheck(DockCheck purpose);
  void BeginDrive(double x, double y);
  // Undocking from base_link at (x, y) for mowing or the drive: charging off first.
  void StartUndocking(DockCheck purpose, double x, double y);
  // Stops the follower on its way out; undocking ends once it stopped, with UNDOCKING_FAILED if failure is set.
  void StopUndocking(const std::optional<Reason>& failure, const Reason& why);
  void EndUndocking(const std::optional<Reason>& failure, const Reason& why);
  void Undocked();
  // Docking with the current settings onto the station of the map, the line within marker_range where Guided; in
  // docking.test onto the station of the marker.
  DockingSequence NewDocking() const;
  // After JOB_COMPLETE, abort_mowing and a reason to go home: DOCKING where the platform docks and the map has a
  // station, else IDLE. why: the reason of the event DOCKING, none after JOB_COMPLETE.
  void GoHome(const Reason& why = Reason());
  // Whether mowing can go home from here: a station the platform docks onto, a way to it, safe, not on it already.
  bool HomeReachable() const;
  HomeConfig HomeRules() const;
  // Samples the rules of going home while mowing, sets the deadline of the sunset and acts on the reasons that hold.
  void WatchHome(double now_s);
  // What the reasons to stop mowing do (question 47 iii); they are those of HomeWatch in its order, then dark (empty
  // when the sunset is not due). With a way home and no pause that waits for the user: home with the first. Else a
  // reason of the battery ends the run. Else a run without a pause pauses with the first and waits for the user (G11).
  void ActOnHome(const Reason& dark);
  // mowing.continue: rain or heat of HomeWatch that still holds while there is no way home.
  Reason ContinueProblem() const;
  // The user resumed: a new deadline of the sunset, and the reasons without dark act at once.
  void ResumedByUser();
  // The deadline of the sunset of this run in monotonic s, when it is empty and the switch is on.
  void TrySunset(double now_s);
  Reason DarkWhy(double now_s) const;
  // The reason of the event DOCKING for a rule of HomeWatch.
  Reason HomeWhy(HomeReason reason) const;
  // The battery ends the run where it cannot go home: BATTERY_LOW with the limit it fell below, IDLE.
  void BatteryEndsMowing(HomeReason reason);
  void BeginDocking();
  // The first leg of the next try from where the mower stands, or the wait for a good position.
  void NextDockingTry();
  // Busy, hold, no charge contacts, on the contacts already, unsafe: what refuses docking.start and docking.test.
  Reason DockStartProblem() const;
  // runs 1 to 10: with more than one a series (runs above 1): after each DOCKED out to the start of the line in the
  // station frame and docking again, every run ends with DOCKING_TEST_RUN, the first failure ends the series.
  Reason StartDockingTest(const Reason& node_problem, int runs);
  bool Series() const {
    return docking_test_ && test_runs_ > 1;
  }
  // The run of the series ends: DOCKING_TEST_RUN, ok without a reason; nothing without an open run.
  void TestRunEnded(const Reason& why);
  // After a DOCKED of the series: out of the station to the start of the line for the next run, charging off first;
  // without a good pose of the marker the series ends.
  void DriveOutForNextRun();
  // Docking by the marker (logic_marker.cpp): the next try in the station frame or the sight; a failed try, which
  // drives out only with a good pose of the marker; the waits; why the mower may not go on at the gate (also by the
  // map); a try failed by the marker; the end.
  void NextMarkerTry();
  // At the approach point of a try that may use the marker: its sight. Where the source cannot measure under prefer:
  // the try by the position, with DOCKING_WITHOUT_MARKER.
  void ChooseGuide();
  void DockByPosition();
  // The legs of the try: in the station frame by the marker, else in the map.
  const DockingSequence& Legs() const;
  // Docking asks for the marker once the map puts the mower within kMarkerRequestDistance of the approach point.
  void WatchMarkerNear();
  // Why the station by the marker does not fit the one of the map (gate: also the heading), empty if it does or there
  // is no good position to compare; in docking.test only told.
  std::string MarkerMapCheck(bool gate);
  void WaitForMarker(bool sight);
  void MarkerTryFailed(const Reason& why);
  void TickMarkerDocking(double now_s);
  Reason GateProblem();
  void MarkerFailed(const std::string& cause, const std::string& text);
  void EndMarkerDocking();
  bool DockPoseGood() const;
  bool SightDone() const;
  // marker or position, for the events of docking.
  const char* Guide() const;
  void OnDockingContact();
  // A try failed: counted at once, or once the mower drove out to the approach point when it stands on the line.
  void DockingTryFailed(const Reason& why);
  // The planner gave no answer for the approach of a try: the straight way, or the try failed.
  void DockingWithoutPlanner(const Reason& silent);
  // The leg for the node; a way out from the contacts waits until charging is off, where the driver can stop it.
  void SetDockingLeg(DockingLeg leg);
  // Only while docking_out_after_charge_off_ holds the way out; OnChargeOff checks it.
  void DockingChargeOff(bool ok, const std::string& message);
  // Once the docking switched charging off: allowed again after the way out and when docking ends.
  void AllowChargeAgain();
  // The check of the charge from the debounced contact on, its samples and how it ended.
  void BeginChargeCheck();
  ChargeSample ChargeSampleNow() const;
  void OnChargeResult(ChargeCheck::Result result);
  // IDLE on the contacts: DOCKED once the charge is confirmed there, CHARGING_LOST when it stops (ChargeWatch).
  void WatchCharge();
  void ChargingLost(const Reason& why);
  // DOCKING_RETRY and the next try, or DOCKING_FAILED and IDLE (also when last). A copy: why may be the failure kept.
  void CountDockingFailure(Reason why, bool last = false);
  void WaitForDockingGps();
  void TickDocking(double now_s);
  // Stops the leg on its way, if any.
  void CancelDockingLeg(const Reason& why);
  void Docked();
  // IDLE; a leg on its way stops with why.
  void EndDocking(const Reason& why);
  // What a mowing.start asks for: the areas in order (none for all) and whether to begin anew; a start of the schedule
  // also the areas it leaves out.
  struct StartRequest {
    std::vector<std::string> tasks;
    bool restart = false;
    std::vector<std::string> pass_over;
    // Taken by a job that begins with this start, kept by one it goes on with.
    std::optional<double> angle_offset_deg;
  };
  // The start problems of the areas of the request that are not passed over; NO_MOWING_AREA when none is left.
  Reason RequestMowProblem(const StartRequest& request) const;
  // Why a start of the schedule cannot go now (StartRun without node_problem), empty if it can.
  Reason ScheduleProblem(const StartRequest& request, const ScheduleRun& run, double now_s) const;
  static StartRequest RunRequest(const ScheduleRun& run);
  // Why the list of a start of the schedule is not taken, empty if it is.
  std::string InvalidRun(const ScheduleRun& run) const;
  // The announcement ran out: checked again, then the reference check, or the refusal ends the run.
  void EndAnnouncement(double now_s);
  // An emergency, a hold or leaving the contacts cancels the announcement; a command cancels it with source rpc.
  void WatchAnnouncement();
  void CommandCancels();
  void CancelAnnouncement(const std::string& source);
  void SetRunPhase(RunPhase phase);
  // How the run ends, the first word counts; FinishRun tells it (EMERGENCY or STOPPED without one) and drops the run.
  void RunEnds(const std::string& code, const std::string& detail = "");
  void FinishRun();
  // Whether a run goes home at sunset: one of the schedule unless it may mow in the dark without until_sunset, one by
  // hand with dock_at_sunset.
  bool SunsetWanted() const;
  // AREA_SKIPPED SCHEDULE for the areas the job passed over.
  void AreasPassedOver(const std::vector<JobArea>& areas);
  // The first deadline of the run that is due: the sunset, then the end of the plan; empty when none is.
  Reason DeadlineDue(double now_s);
  // tasks and restart of mowing.start; the problem with them for -32602 and its field, empty if none. The request
  // is set only without a problem.
  std::string ReadStartRequest(const json& params, StartRequest* request, std::string* field) const;
  // mowing.start as ReadStartRequest read it: the refusal, or the reference check on the station, or mowing.
  Reason RequestStart(StartRequest request, const Reason& node_problem, CommandOutcome* out);
  // Continues the open job, or begins the one mowing.start asked for.
  void StartMowing();
  void Pause(int why, const Reason& reason);
  // Stops a drive or a leg on its way; a drive ends with the first reason given.
  void CancelDrive(const Reason& reason);
  void Unpause(int why);
  void ApplySkips();
  // The follower drives a leg of the job.
  bool LegDrives() const;
  // The path the last skip left, while the job stands at the start of the one after it, the mower within
  // approach_distance of the pose it got to on the path left, and poses of it are left; else nothing.
  const JobPath* SkippedPathHere() const;
  // Its next piece as the step of the job: the poses within kSkippedPathPiece m of the one the mower stands at, at
  // least the next one.
  JobStep SkippedPiece(const JobPath& path);
  // Ends the job: finished (JOB_COMPLETE) or not (it goes on from the checkpoint next time); home: docking after it,
  // with why as the reason of the event DOCKING.
  void EndJob(bool finished, bool home, const Reason& why = Reason());
  // The planner gave the job no answer: it asks again where retry is set and tries are left, else the mower goes
  // home; pause: the pause where it cannot.
  void JobWithoutPlanner(const Reason& silent, const Reason& pause, bool retry);
  bool PoseGood() const;
  void CheckpointNow();
  // STORAGE_NOT_WRITABLE with the file while one cannot be written, else empty.
  Reason StorageProblem() const;
  // The effects of one type in effects_: whether one waits; a flag added once or every one removed; the waiting one
  // replaced; every one removed.
  template <typename T>
  bool Pending() const;
  template <typename T>
  void SetDue(bool due);
  template <typename T>
  void Replace(T value);
  template <typename T>
  void Drop();
  void Notify(std::string line);

  LogicConfig config_;
  std::mt19937_64 ids_;
  HardwareSnapshot hw_;
  State state_ = State::kIdle;
  bool emergency_ = false;
  // Without telemetry the mower counts as stopped until the driver reports.
  int reason_ = kEmergencyTimeoutInputs;
  // OpenMower logs the first reason as a change from none.
  int logged_reason_ = 0;
  std::optional<PoseSnapshot> pose_;
  double last_x_ = 0;
  double last_y_ = 0;
  bool hw_seen_ = false;
  bool restart_hold_ = false;
  // The file of SetUnwritableFile.
  std::string unwritable_file_;
  bool teleop_armed_ = false;
  bool safety_measured_ = false;
  // Until the driver says otherwise, undocking waits for charging to be off.
  bool charge_stop_ = true;
  double last_teleop_s_ = -std::numeric_limits<double>::infinity();
  // What the node has not taken yet, oldest first; flags (StopRequest, ChargeOff, ...) at most once.
  EffectQueue effects_;
  AreaRecorder recorder_;
  // The last OnWheelOdom.
  std::optional<double> last_wheel_s_;
  bool map_writable_ = false;
  Reason map_reason_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");

  MowingJob job_;
  // The start accepted last, until mowing begins.
  StartRequest start_request_;
  Reason mow_map_problem_ = MapReason(MapDatum::kUnknown, "The map status is not known yet");
  std::optional<DockPose> dock_;
  // The reference check on its way and what for.
  DockCheck dock_check_ = DockCheck::kNone;
  // The goal of a drive from the station, for after undocking.
  double drive_x_ = 0;
  double drive_y_ = 0;
  std::vector<Ring> drivable_;
  std::vector<Ring> obstacles_;
  Reason drivable_problem_;
  // Undocking: what for, where it started and the way it faces, and once it is stopping, whether that ends in
  // UNDOCKING_FAILED.
  enum class UndockStep { kChargeOff, kDrive };
  DockCheck undock_for_ = DockCheck::kNone;
  UndockStep undock_step_ = UndockStep::kChargeOff;
  double undock_x_ = 0;
  double undock_y_ = 0;
  double undock_heading_ = 0;
  std::optional<Point2> undock_fix_start_;
  bool undock_stopping_ = false;
  std::optional<Reason> undock_failure_;
  Reason undock_stop_why_;
  // Docking: its tries, the leg due for the node and the kind of the one on its way (and of the last one), stopped at a
  // first contact, a failed try waiting for the mower to drive out, and the wait for a good position.
  DockingSequence docking_;
  std::optional<DockingLeg> docking_leg_due_;
  std::optional<DockingLeg::Kind> docking_leg_;
  DockingLeg::Kind docking_last_kind_ = DockingLeg::Kind::kApproach;
  bool docking_hold_ = false;
  std::optional<Reason> docking_failure_;
  bool docking_gps_wait_ = false;
  std::optional<double> docking_gps_since_;
  std::optional<double> docking_gps_good_since_;
  // The check of the charge on the contacts and whether it ran in this try; the way out kept until charging is off;
  // whether this docking switched charging off (stopped once the driver confirmed it) and has to allow it again.
  std::optional<ChargeCheck> docking_charge_;
  bool docking_checked_ = false;
  std::optional<DockingLeg> docking_out_after_charge_off_;
  bool docking_charge_touched_ = false;
  bool docking_charge_stopped_ = false;
  // Why the planner gave no way for the try that drives straight, and the turning circle of the body for its check.
  std::string docking_direct_cause_;
  double turning_radius_ = 0;
  json shape_outside_ = json::array();
  // Docking by the marker: this docking, the test, whether a leg went out, whether the scale of the marker failed (no
  // drive by it then), whether the next wait is a sight, what the marker told, and the latest monotonic s of it.
  bool marker_docking_ = false;
  bool docking_test_ = false;
  // The series of docking.test: its runs, the one on its way (from the drive out on), whether its event is due, and
  // whether the drive out of the station leads to the next run.
  int test_runs_ = 0;
  int test_run_ = 0;
  bool test_run_open_ = false;
  bool test_next_run_ = false;
  // m along the line from its start where run 1 began: the drive out for the next run ends there, where the marker
  // gave a direction.
  double test_start_along_ = 0.0;
  bool marker_started_ = false;
  bool marker_untrusted_ = false;
  bool marker_sight_ = false;
  // The legs in the station frame; the try chose the marker or the position; docking asks for the marker; the frame of
  // the leg on its way.
  DockingSequence marker_legs_;
  bool try_by_marker_ = false;
  bool try_by_position_ = false;
  bool marker_near_ = false;
  LegFrame docking_leg_frame_ = LegFrame::kMap;
  MarkerWatch marker_;
  double marker_clock_ = 0;
  // Whether charging is confirmed on the station, and its loss.
  ChargeWatch charge_watch_;
  std::string session_id_;
  int pause_ = 0;
  Reason pause_reason_;
  Leg leg_ = Leg::kNone;
  bool approached_ = false;
  // Questions of the job in a row the planner gave no answer for.
  int planner_silent_ = 0;
  bool skip_path_ = false;
  bool skip_area_ = false;
  // Where the job stood when a skip left a path that was not the last of its plan; index follows the mower along it.
  std::optional<JobProgress> skipped_;
  // The planner found no way from there to the next path: a piece of the path left is the next step.
  bool drive_on_ = false;
  std::string announced_area_;
  std::optional<double> spinup_since_;
  std::optional<double> gps_good_since_;
  // The rules of going home while mowing or calibrating.
  HomeWatch home_;
  // The run of the schedule from StartRun until the mower is idle again: its request, phase, the end of the
  // announcement and of the plan (monotonic s), whether that end came, and how it ends.
  struct Run {
    ScheduleRun request;
    RunPhase phase = RunPhase::kNone;
    double announce_until = 0;
    std::optional<double> end_due;
    bool end_passed = false;
    std::string end_code;
    std::string detail;
  };
  std::optional<Run> run_;
  // Why the next skip of an area is made: MANUAL by the app, SCHEDULE for a pause of the schedule.
  Reason skip_why_;
  // The sunset: the switch, the clock and the datum it needs, and the deadline of this run (monotonic s), due once
  // more after a start, a continue, a clock or a datum. sunset_told_: the first try of the run could not reckon it.
  bool sunset_on_ = false;
  bool clock_safe_ = false;
  std::string clock_reason_ = "no_report";
  double clock_offset_ = 0;
  std::optional<std::pair<double, double>> datum_;
  std::optional<double> sunset_due_;
  bool sunset_try_ = false;
  bool sunset_tried_ = false;
  bool sunset_told_ = false;
  double last_checkpoint_s_ = 0;
  bool blades_running_ = false;
  // The reason of StartProblem() at the last actions, empty while a start would go.
  std::string last_start_reason_ = "-";
  SetupPart setup_;
  CalibrationPart calibration_;
  int64_t hold_until_ = 0;
  // The client of the command being run, for its events; the one of a skip that waits for the leg to stop.
  std::string client_;
  std::string skip_client_;
  ReleasePart release_;
};

template <typename T>
bool LogicCore::Pending() const {
  return effects_.Pending<T>();
}

template <typename T>
void LogicCore::SetDue(bool due) {
  effects_.SetDue<T>(due);
}

template <typename T>
void LogicCore::Replace(T value) {
  effects_.Replace(std::move(value));
}

template <typename T>
void LogicCore::Drop() {
  effects_.Drop<T>();
}

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_LOGIC_CORE_H
