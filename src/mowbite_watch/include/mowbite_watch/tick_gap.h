#ifndef MOWBITE_WATCH_TICK_GAP_H
#define MOWBITE_WATCH_TICK_GAP_H

#include <optional>

namespace mowbite_watch {

// Whether a periodic tick may judge the age of its inputs. After the thread of the node stood still for longer than
// gap (a write, a busy CPU), what came in meanwhile still waits in its queue and every input looks old: the first
// tick after such a gap does not judge, the next one does. Never two ticks in a row, so a node that is always late
// still judges every other tick.
class TickGap {
 public:
  explicit TickGap(double gap_s) : gap_s_(gap_s) {}

  // now: s of a monotonic clock, once per tick.
  bool Judge(double now) {
    const bool late = last_ && now - *last_ > gap_s_;
    last_ = now;
    skipped_ = late && !skipped_;
    return !skipped_;
  }

 private:
  double gap_s_;
  std::optional<double> last_;
  bool skipped_ = false;
};

}  // namespace mowbite_watch

#endif  // MOWBITE_WATCH_TICK_GAP_H
