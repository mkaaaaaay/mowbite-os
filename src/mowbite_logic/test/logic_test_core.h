#ifndef MOWBITE_LOGIC_TEST_LOGIC_TEST_CORE_H
#define MOWBITE_LOGIC_TEST_LOGIC_TEST_CORE_H

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mowbite_logic/logic_core.h"

namespace mowbite_logic {

// The core for the tests, which check the effects by kind: each Take takes what the core has and hands out its kind,
// the others wait for their own call.
class TestCore : public LogicCore {
 public:
  using LogicCore::LogicCore;

  std::vector<Effect> TakeEffects() {
    Collect();
    std::vector<Effect> out;
    out.swap(waiting_);
    return out;
  }
  std::vector<Event> TakeEvents() {
    return Take<Event>();
  }
  std::vector<ReasonChange> TakeReasonChanges() {
    return Take<ReasonChange>();
  }
  std::optional<Reason> TakeDriveCancel() {
    auto cancels = Take<DriveCancel>();
    if (cancels.empty()) return std::nullopt;
    return cancels.front().why;
  }
  bool TakeStopRequest() {
    return !Take<StopRequest>().empty();
  }
  bool TakeActionsChanged() {
    return !Take<ActionsChanged>().empty();
  }
  bool TakeCalibrationCancel() {
    return !Take<CalibrationCancel>().empty();
  }
  bool TakeChargeOff() {
    return !Take<ChargeOff>().empty();
  }
  bool TakeChargeOn() {
    return !Take<ChargeOn>().empty();
  }
  std::optional<double> TakeDockCheck() {
    return Last<ReferenceCheck>([](const ReferenceCheck& check) { return check.heading; });
  }
  std::optional<UndockPath> TakeUndockPath() {
    return Last<UndockPath>([](const UndockPath& path) { return path; });
  }
  std::optional<UndockOutcome> TakeUndockOutcome() {
    return Last<UndockOutcome>([](const UndockOutcome& outcome) { return outcome; });
  }
  std::optional<CalibrationGoal> TakeCalibrationGoal() {
    return Last<CalibrationGoal>([](const CalibrationGoal& goal) { return goal; });
  }
  std::optional<MapEdit> TakeMapEdit() {
    return Last<MapEdit>([](const MapEdit& edit) { return edit; });
  }
  std::optional<std::string> TakeCheckpoint() {
    return Last<Checkpoint>([](const Checkpoint& checkpoint) { return checkpoint.text; });
  }
  std::vector<RunChanged> TakeRuns() {
    return Take<RunChanged>();
  }
  std::vector<std::string> TakeNotices() {
    std::vector<std::string> lines;
    for (auto& notice : Take<Notice>()) lines.push_back(std::move(notice.line));
    return lines;
  }

 private:
  void Collect() {
    for (auto& effect : LogicCore::TakeEffects()) waiting_.push_back(std::move(effect));
  }
  template <typename T>
  std::vector<T> Take() {
    Collect();
    std::vector<T> out;
    std::vector<Effect> kept;
    for (auto& effect : waiting_) {
      if (auto* value = std::get_if<T>(&effect)) {
        out.push_back(std::move(*value));
      } else {
        kept.push_back(std::move(effect));
      }
    }
    waiting_.swap(kept);
    return out;
  }
  template <typename T, typename F>
  auto Last(F value) -> std::optional<decltype(value(std::declval<const T&>()))> {
    auto all = Take<T>();
    if (all.empty()) return std::nullopt;
    return value(all.back());
  }

  std::vector<Effect> waiting_;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_TEST_LOGIC_TEST_CORE_H
