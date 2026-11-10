#ifndef MOWBITE_WATCH_WATCH_CORE_H
#define MOWBITE_WATCH_WATCH_CORE_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace mowbite_watch {

// Places of one process; a place is never used again after its node left, so the watch thread reads names without a
// lock.
constexpr int kMaxPlaces = 32;
constexpr size_t kNameSize = 48;

enum class Finding {
  kNone,
  // A node gave no sign for longer than its limit.
  kSilent,
  // A node of the expected list did not enter within the start limit.
  kMissing,
  // The process did not end within the limit after the shutdown of ROS.
  kShutdown,
};

// What ends the process; node and mark point into the core (or are the text the node marked) and stay valid.
struct Verdict {
  Finding finding = Finding::kNone;
  const char* node = "";
  int64_t seconds_ns = 0;
  int64_t limit_ns = 0;
  // The last mark of a silent node, nullptr without one.
  const char* mark = nullptr;
};

// When which node gave its last sign, and what follows from that at a time; times in ns of CLOCK_MONOTONIC as input.
// Enter, Leave and Shutdown come from the nodes (Enter one at a time), Sign and SetMark from their threads, Judge from
// the watch thread: Sign, SetMark and Judge take no lock and do not allocate.
class WatchCore {
 public:
  // expected: the nodes that have to enter within start_limit_ns of started_ns; at most kMaxPlaces.
  WatchCore(const std::vector<std::string>& expected, int64_t started_ns, int64_t start_limit_ns,
            int64_t shutdown_limit_ns);

  // The place of a node that signs from now on, -1 when all places are taken. A name of kNameSize or longer throws.
  int Enter(std::string_view name, int64_t limit_ns, int64_t now_ns);
  void Sign(int place, int64_t now_ns);
  // where must outlive the place.
  void SetMark(int place, const char* where);
  // The node no longer counts, also not as missing.
  void Leave(int place);
  // From here on signs no longer count; only the end of the process within the shutdown limit.
  void Shutdown(int64_t now_ns);

  // The oldest silent node beyond its limit (exactly at the limit is in time), else the first expected node missing
  // after the start limit; after a shutdown only whether the process outlived the shutdown limit.
  Verdict Judge(int64_t now_ns) const;

 private:
  enum State : int { kFree, kActive, kLeft };
  struct Place {
    std::atomic<int> state{kFree};
    std::atomic<int64_t> last_ns{0};
    // Written before state turns kActive, read after.
    int64_t limit_ns = 0;
    char name[kNameSize] = {};
    std::atomic<const char*> mark{nullptr};
  };

  std::array<Place, kMaxPlaces> places_;
  std::atomic<int> used_{0};
  // Fixed before any node enters.
  std::vector<std::string> expected_;
  std::array<std::atomic<bool>, kMaxPlaces> seen_{};
  const int64_t started_ns_;
  const int64_t start_limit_ns_;
  const int64_t shutdown_limit_ns_;
  std::atomic<bool> shut_down_{false};
  std::atomic<int64_t> shutdown_ns_{0};
};

}  // namespace mowbite_watch

#endif  // MOWBITE_WATCH_WATCH_CORE_H
