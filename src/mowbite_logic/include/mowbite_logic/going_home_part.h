#ifndef MOWBITE_LOGIC_GOING_HOME_PART_H
#define MOWBITE_LOGIC_GOING_HOME_PART_H

#include <optional>

#include "mowbite_logic/core_access.h"
#include "mowbite_logic/home_watch.h"

namespace mowbite_logic {

// The rules of going home while mowing or calibrating (HomeWatch) and the deadline of the sunset. The calls are those
// of LogicCore with the same names.
class GoingHomePart {
 public:
  explicit GoingHomePart(const HomeConfig& rules) : home_(rules) {}

  Reason HomeWhy(CoreFacts core, HomeReason reason) const;
  void SetSunsetSettings(CoreFacts core, bool dock_at_sunset);
  void TrySunset(CoreAccess core, double now_s);
  // Mowing began or ended: the rules and the sunset start from nothing.
  void StartOver() {
    home_.Reset();
    sunset_due_.reset();
    sunset_try_ = sunset_tried_ = sunset_told_ = false;
  }
  // The user resumed: a deadline that came is dropped and the next one reckoned.
  void Resumed() {
    sunset_due_.reset();
    sunset_try_ = true;
  }
  // A start, a clock or a datum: the deadline is reckoned again.
  void TryAgain() {
    sunset_try_ = true;
  }
  HomeWatch& watch() {
    return home_;
  }
  const HomeWatch& watch() const {
    return home_;
  }
  bool sunset_on() const {
    return sunset_on_;
  }
  const std::optional<double>& sunset_due() const {
    return sunset_due_;
  }
  bool sunset_try() const {
    return sunset_try_;
  }

 private:
  HomeWatch home_;
  // The sunset: the switch and the deadline of this run (monotonic s), due once more after a start, a continue, a
  // clock or a datum. sunset_told_: the first try of the run could not reckon it.
  bool sunset_on_ = false;
  std::optional<double> sunset_due_;
  bool sunset_try_ = false;
  bool sunset_tried_ = false;
  bool sunset_told_ = false;
};

}  // namespace mowbite_logic

#endif  // MOWBITE_LOGIC_GOING_HOME_PART_H
