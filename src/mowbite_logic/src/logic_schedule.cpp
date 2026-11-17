// The run of the schedule from its announcement until the mower is idle again.
#include <cmath>
#include <string>
#include <utility>

#include "mowbite_logic/schedule_part.h"

namespace mowbite_logic {

StartRequest SchedulePart::RunRequest(const ScheduleRun& run) {
  StartRequest request{run.area_ids, run.restart, run.pass_over, std::nullopt};
  if (run.has_angle) request.angle_offset_deg = run.angle_offset_deg;
  return request;
}

Reason SchedulePart::StartRun(CoreAccess core, const ScheduleRun& run, const Reason& node_problem, double now_s) {
  if (const std::string invalid = core.InvalidRun(run); !invalid.empty()) {
    core.Notify("Start of the schedule " + run.run_id + " refused: " + invalid);
    return Reason("INVALID", invalid);
  }
  const StartRequest request = RunRequest(run);
  Reason why;
  if (run_) why = Reason("BUSY", "The mower is busy: a start of the schedule is announced").With("state", "announcing");
  if (why.empty()) why = core.ScheduleProblem(request, run, now_s);
  if (why.empty()) why = node_problem;
  if (!why.empty()) {
    json details = {{"source", "schedule"}};
    PutReason(&details, "MOWING_REFUSED", why);
    core.AddEvent("MOWING_REFUSED", std::move(details));
    return why;
  }
  run_ = Run{run, RunPhase::kNone, now_s + kAnnounceS, std::nullopt, false, "", ""};
  if (run.end_in_s > 0) run_->end_due = now_s + static_cast<double>(run.end_in_s);
  SetRunPhase(core, RunPhase::kAnnouncing);
  core.AddEvent("SCHEDULE_START", {{"seconds", static_cast<int>(kAnnounceS)},
                              {"until", static_cast<int64_t>(std::llround(now_s + core.clock_offset() + kAnnounceS))},
                              {"severity", "info"},
                              {"resume", "auto"}});
  core.Notify("Schedule: " + run.run_id + " starts in " + std::to_string(static_cast<int>(kAnnounceS)) + " s");
  core.effects().SetDue<ActionsChanged>(true);
  return {};
}

void SchedulePart::WatchAnnouncement(CoreAccess core) {
  if (!run_ || run_->phase != RunPhase::kAnnouncing) return;
  if (core.emergency()) {
    CancelAnnouncement(core, "emergency");
  } else if (core.Held()) {
    CancelAnnouncement(core, "hold");
  } else if (!core.DockContact()) {
    CancelAnnouncement(core, "undocked");
  }
}

void SchedulePart::CommandCancels(CoreAccess core) {
  if (run_ && run_->phase == RunPhase::kAnnouncing) CancelAnnouncement(core, "rpc");
}

void SchedulePart::CancelAnnouncement(CoreAccess core, const std::string& source) {
  if (!run_) return;
  core.AddEvent("SCHEDULE_START_CANCELED", {{"source", source}, {"severity", "info"}, {"resume", "auto"}});
  core.Notify("Schedule: the start of " + run_->request.run_id + " was canceled (" + source + ")");
  RunEnds("CANCELED", source);
  FinishRun(core);
}

void SchedulePart::SetRunPhase(CoreAccess core, RunPhase phase) {
  if (!run_ || run_->phase == phase) return;
  run_->phase = phase;
  core.effects().push_back(RunChanged{run_->request.run_id, phase, "", ""});
}

void SchedulePart::RunEnds(const std::string& code, const std::string& detail) {
  if (!run_ || !run_->end_code.empty()) return;
  run_->end_code = code;
  run_->detail = detail;
}

void SchedulePart::FinishRun(CoreAccess core) {
  if (!run_) return;
  if (run_->end_code.empty()) run_->end_code = core.emergency() ? "EMERGENCY" : "STOPPED";
  core.effects().push_back(RunChanged{run_->request.run_id, RunPhase::kEnded, run_->end_code, run_->detail});
  run_.reset();
  core.effects().SetDue<ActionsChanged>(true);
}

}  // namespace mowbite_logic
