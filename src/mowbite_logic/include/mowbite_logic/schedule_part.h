#ifndef MOWBITE_LOGIC_SCHEDULE_PART_H
#define MOWBITE_LOGIC_SCHEDULE_PART_H

#include <optional>
#include <string>

#include "mowbite_logic/core_access.h"

namespace mowbite_logic {

// The run of the schedule from StartRun until the mower is idle again. The calls are those of LogicCore with the same
// names.
class SchedulePart {
 public:
  // The run: its request, phase, the end of the announcement and of the plan (monotonic s), whether that end came,
  // and how it ends.
  struct Run {
    ScheduleRun request;
    RunPhase phase = RunPhase::kNone;
    double announce_until = 0;
    std::optional<double> end_due;
    bool end_passed = false;
    std::string end_code;
    std::string detail;
  };

  static StartRequest RunRequest(const ScheduleRun& run);
  Reason StartRun(CoreAccess core, const ScheduleRun& run, const Reason& node_problem, double now_s);
  // An emergency, a hold or leaving the contacts cancels the announcement; a command cancels it with source rpc.
  void WatchAnnouncement(CoreAccess core);
  void CommandCancels(CoreAccess core);
  void CancelAnnouncement(CoreAccess core, const std::string& source);
  void SetRunPhase(CoreAccess core, RunPhase phase);
  // How the run ends, the first word counts; FinishRun tells it (EMERGENCY or STOPPED without one) and drops the run.
  void RunEnds(const std::string& code, const std::string& detail = "");
  void FinishRun(CoreAccess core);
  std::optional<Run>& run() {
    return run_;
  }
  const std::optional<Run>& run() const {
    return run_;
  }

 private:
  std::optional<Run> run_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_SCHEDULE_PART_H
