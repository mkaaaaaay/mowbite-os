#ifndef MOWBITE_LOCALIZATION_STATE_HISTORY_H
#define MOWBITE_LOCALIZATION_STATE_HISTORY_H

#include <algorithm>
#include <deque>
#include <iterator>

#include "mowbite_localization/pose_filter.h"

namespace mowbite_localization {

// The estimate at its last moments, oldest first, to fuse a measurement at the time it was taken. Moment has the
// members time (s) and state.
template <typename Moment>
class StateHistory {
 public:
  // Appends a moment and drops those before oldest.
  void Add(const Moment& moment, double oldest) {
    moments_.push_back(moment);
    while (!moments_.empty() && moments_.front().time < oldest) moments_.pop_front();
  }

  // The newest moment not after t, else the oldest one kept; nullptr while empty.
  const Moment* At(double t) const {
    if (moments_.empty()) return nullptr;
    auto it = std::upper_bound(moments_.begin(), moments_.end(), t + 1e-9,
                               [](double time, const Moment& moment) { return time < moment.time; });
    return it == moments_.begin() ? &moments_.front() : &*std::prev(it);
  }

  // Moves every kept state by what a correction moved the estimate from before to after.
  void Shift(const State& before, const State& after) {
    State delta = after - before;
    delta(2) = WrapAngle(delta(2));
    for (Moment& moment : moments_) {
      moment.state += delta;
      moment.state(2) = WrapAngle(moment.state(2));
    }
  }

  void Clear() { moments_.clear(); }
  bool empty() const { return moments_.empty(); }

 private:
  std::deque<Moment> moments_;
};

}  // namespace mowbite_localization

#endif  // MOWBITE_LOCALIZATION_STATE_HISTORY_H
