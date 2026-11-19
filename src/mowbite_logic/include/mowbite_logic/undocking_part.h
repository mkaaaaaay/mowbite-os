#ifndef MOWBITE_LOGIC_UNDOCKING_PART_H
#define MOWBITE_LOGIC_UNDOCKING_PART_H

#include <optional>
#include <string>

#include "mowbite_logic/core_access.h"

namespace mowbite_logic {

// The reference check on the docking station and UNDOCKING after it, for mowing or a drive. The calls are those of
// LogicCore with the same names.
class UndockingPart {
 public:
  void OnGnssFix(CoreAccess core, double x, double y);
  void BeginDockCheck(CoreAccess core, DockCheck purpose);
  void CheckDockBeforeDrive(CoreAccess core, double x, double y);
  bool DockCheckPending() const;
  Reason OnDockCheck(CoreAccess core, bool ok, double x, double y, float accuracy, const std::string& message);
  Reason RefuseDockCheck(CoreAccess core, const Reason& why);
  // Undocking from base_link at (x, y) for mowing or the drive: charging off first.
  void StartUndocking(CoreAccess core, DockCheck purpose, double x, double y);
  bool UndockingForDrive(CoreFacts core) const;
  // The answer to ChargeOff while undocking waits for it.
  void OnChargeOff(CoreAccess core, bool ok, const std::string& message);
  // Stops the follower on its way out; undocking ends once it stopped, with UNDOCKING_FAILED if failure is set.
  void StopUndocking(CoreAccess core, const std::optional<Reason>& failure, const Reason& why);
  void AbortUndocking(CoreAccess core, const Reason& why);
  void OnUndockEnded(CoreAccess core, LegEnd end, const Reason& reason);
  void EndUndocking(CoreAccess core, const std::optional<Reason>& failure, const Reason& why);
  void Undocked(CoreAccess core);
  DockCheck dock_check() const {
    return dock_check_;
  }
  DockCheck undock_for() const {
    return undock_for_;
  }
  // The follower drives the way out.
  bool Leaving() const {
    return undock_step_ == UndockStep::kDrive && !undock_stopping_;
  }

 private:
  // Ends the check on its way and gives what it was for, kNone without one.
  DockCheck TakeDockCheck(CoreAccess core);
  // MOWING_REFUSED and the end of a run, or DRIVE_REFUSED.
  static void RefuseFor(CoreAccess core, DockCheck purpose, const Reason& why);
  // The reference check on its way and what for.
  DockCheck dock_check_ = DockCheck::kNone;
  // The goal of a drive from the station, for after undocking.
  double drive_x_ = 0;
  double drive_y_ = 0;
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
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_UNDOCKING_PART_H
