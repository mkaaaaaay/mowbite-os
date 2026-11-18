#ifndef MOWBITE_LOGIC_MOWING_PART_H
#define MOWBITE_LOGIC_MOWING_PART_H

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "mowbite_logic/core_access.h"

namespace mowbite_logic {

// MOWING: the job, its session, pauses, the legs of the job and skips (logic_mowing.cpp). The calls are those of
// LogicCore with the same names.
class MowingPart {
 public:
  // tasks and restart of mowing.start; the problem with them for -32602 and its field, empty if none. The request
  // is set only without a problem.
  std::string ReadStartRequest(CoreFacts core, const json& params, StartRequest* request,
                               std::string* field) const;
  // mowing.start as ReadStartRequest read it: the refusal, or the reference check on the station, or mowing.
  Reason RequestStart(CoreAccess core, StartRequest request, const Reason& node_problem, CommandOutcome* out);
  // The job StartMowing continues or begins, before the state changes.
  void PrepareStart(CoreAccess core);
  void ApplySkips(CoreAccess core);
  void Pause(CoreAccess core, int why, const Reason& reason);
  void Unpause(CoreAccess core, int why);
  JobStep NextJobStep(CoreAccess core, double now_s);
  void OnJobPlanned(CoreAccess core, std::vector<JobPath> paths, const std::string& fingerprint);
  void OnJobPlanFailed(CoreAccess core, const Reason& why);
  void OnJobProgress(CoreFacts core, size_t index);
  void OnLegEnded(CoreAccess core, LegEnd end, size_t index, const Reason& reason);
  // The planner's cut of the plan by the circles (JobStep kCut); the pieces carry the numbers of the plan's paths.
  void OnCutAnswer(CoreAccess core, const CutAnswer& answer);
  // The follower drives a leg of the job.
  bool LegDrives() const;
  // SetState into MOWING: the job id once, a new session.
  void Enter(CoreAccess core);
  // SetState out of MOWING: the session, the leg, pauses and skips end.
  void Leave(CoreAccess core);
  // In a pause for GPS: mowing goes on once the position stayed good for gps_wait_time.
  void TickGpsWait(CoreAccess core, double now_s);
  // On a path the checkpoint is saved every checkpoint_period.
  void TickCheckpoint(CoreAccess core, double now_s);

  MowingJob& job() {
    return job_;
  }
  const MowingJob& job() const {
    return job_;
  }
  StartRequest& start_request() {
    return start_request_;
  }
  const StartRequest& start_request() const {
    return start_request_;
  }
  const std::string& session_id() const {
    return session_id_;
  }
  int pause() const {
    return pause_;
  }
  // The reason of the pause that holds the mower now for robot_state: an emergency, a fault, the user, an obstacle,
  // then GPS; empty without a pause.
  const Reason& PauseReason() const;
  Leg leg() const {
    return leg_;
  }
  const std::optional<double>& spinup_since() const {
    return spinup_since_;
  }
  bool& skip_path() {
    return skip_path_;
  }
  bool& skip_area() {
    return skip_area_;
  }
  Reason& skip_why() {
    return skip_why_;
  }
  std::string& skip_client() {
    return skip_client_;
  }

 private:
  // The planner gave the job no answer: it asks again where retry is set and tries are left, else the mower goes
  // home; pause: the pause where it cannot.
  void JobWithoutPlanner(CoreAccess core, const Reason& silent, const Reason& pause, bool retry);
  // The path the last skip left, while the job stands at the start of the one after it, the mower within
  // approach_distance of the pose it got to on the path left, and poses of it are left; else nothing.
  const JobPath* SkippedPathHere(CoreFacts core) const;
  // Its next piece as the step of the job: the poses within kSkippedPathPiece m of the one the mower stands at, at
  // least the next one.
  JobStep SkippedPiece(CoreAccess core, const JobPath& path);
  // The blade got up to speed for a leg that mows; false while it spins up (a pause after mower_spinup_timeout).
  bool BladeReady(CoreAccess core, double now_s);
  // Asks the planner to cut the rest of the plan by the circles kept now and the ones added (from a hold).
  JobStep CutStep(CoreAccess core, std::vector<AvoidCircle> added);
  // The job onto the first piece of the cut not behind it; poses left out on the way make the next leg a detour. No
  // piece left: the rest of the area is left out.
  void FollowCut();
  // The pieces of an answer lie in the plan where the cut began, in order: else the answer counts as failed.
  bool PiecesFit(const std::vector<JobPiece>& pieces) const;
  // m of the plan from where the job stands that the pieces leave out.
  double LeftOut(const std::vector<JobPiece>& pieces) const;

  MowingJob job_;
  // The start accepted last, until mowing begins.
  StartRequest start_request_;
  std::string session_id_;
  int pause_ = 0;
  // The reason each kind of pause came with, by its bit.
  std::array<Reason, 5> pause_reasons_;
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
  // obstacle_mode avoid: the pieces of the plan left to mow while the mower keeps out of the circles, in order (none:
  // the plan as it is); the circles the cut on its way adds and the area and plan it is for; the circles changed or a
  // plan came while there are some (the plan is cut anew before the next leg); poses were left out (the next leg drives
  // to the piece within the area, the blade on); answers of SEARCH_LIMIT in a row.
  std::optional<std::vector<JobPiece>> cut_;
  std::vector<AvoidCircle> cutting_added_;
  JobProgress cutting_at_;
  bool recut_ = false;
  bool detour_ = false;
  int cut_retries_ = 0;
  // Why the next skip of an area is made: MANUAL by the app, SCHEDULE for a pause of the schedule.
  Reason skip_why_;
  double last_checkpoint_s_ = 0;
  // The client of a skip that waits for the leg to stop, for its events.
  std::string skip_client_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_MOWING_PART_H
