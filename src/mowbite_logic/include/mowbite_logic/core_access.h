#ifndef MOWBITE_LOGIC_CORE_ACCESS_H
#define MOWBITE_LOGIC_CORE_ACCESS_H

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/logic_types.h"

namespace mowbite_logic {

class LogicCore;

// What a part of the logic reads of the core it belongs to. The core hands it in with each call and the parts keep no
// pointer to the core, so a copy of the core stays whole.
class CoreFacts {
 public:
  explicit CoreFacts(const LogicCore& core) : read_(&core) {}

  const LogicConfig& config() const;
  State state() const;
  bool emergency() const;
  const HardwareSnapshot& hw() const;
  const std::optional<PoseSnapshot>& pose() const;
  const Reason& drivable_problem() const;
  bool DockContact() const;
  Reason Busy() const;
  Reason HoldProblem() const;
  Reason Unsafe() const;
  Reason AccuracyReason(float accuracy, double limit) const;
  bool Held() const;
  double clock_offset() const;
  std::string InvalidRun(const ScheduleRun& run) const;
  Reason ScheduleProblem(const StartRequest& request, const ScheduleRun& run, double now_s) const;
  bool SunsetWanted() const;
  bool clock_safe() const;
  const std::string& clock_reason() const;
  const std::optional<std::pair<double, double>>& datum() const;
  const std::optional<DockPose>& dock() const;
  bool charge_stop() const;
  const StartRequest& start_request() const;
  double DockedHeading() const;
  bool ContactsBack() const;
  bool LeavesForwards() const;
  const char* LeaveWords() const;
  Reason RequestMowProblem(const StartRequest& request) const;
  Reason DriveProblem() const;
  Reason MoveProblem() const;
  double DockAccuracyLimit() const;
  Reason UndockEndProblem(double x, double y) const;
  bool CanDock() const;
  bool PoseGood() const;
  Reason PositionProblem() const;
  const std::vector<Ring>& drivable() const;
  const std::vector<Ring>& obstacles() const;
  const Reason& mow_map_problem() const;
  Reason StartProblem(const std::vector<std::string>& tasks) const;
  const std::string& JobId() const;
  bool HomeReachable() const;
  const ObstaclePart& obstacle_part() const;

 private:
  const LogicCore* read_;
};

// What a part of the logic reads and changes of the core, handed in like CoreFacts.
class CoreAccess : public CoreFacts {
 public:
  explicit CoreAccess(LogicCore& core) : CoreFacts(core), core_(&core) {}

  EffectQueue& effects();
  // The client of the command being run, for its events.
  std::string& client();
  HomeWatch& home();
  std::string AddEvent(const std::string& type, json details);
  void Notify(std::string line);
  void SetState(State state);
  void CommandCancels();
  void RefuseStart(const Reason& why);
  void RefuseDrive(const Reason& why);
  void RunEnds(const std::string& code, const std::string& detail = "");
  void FinishRun();
  void SetRunPhase(RunPhase phase);
  void BeginDrive(double x, double y);
  void StartMowing();
  void CancelDrive(const Reason& reason);
  std::string NewId();
  void CheckpointNow();
  void BeginDockCheck(DockCheck purpose);
  void AreasPassedOver(const std::vector<JobArea>& areas);
  void AreaSkipped(const JobArea& area, const Reason& why);
  void EndJob(bool finished, bool home, const Reason& why = Reason());
  ObstaclePart& obstacle_part();

 private:
  LogicCore* core_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_CORE_ACCESS_H
