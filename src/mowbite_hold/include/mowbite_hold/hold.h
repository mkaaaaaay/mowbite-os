#ifndef MOWBITE_HOLD_HOLD_H
#define MOWBITE_HOLD_HOLD_H

#include <sys/types.h>

#include <cstdint>
#include <optional>
#include <string>

namespace mowbite_hold {

// The hold of an update (system.hold, updater contract 3.5): while it lasts nothing starts the wheels or the blade.
// The logic grants it and writes <data_dir>/hold.json; the drivers read that file themselves, so the hold also stands
// without the logic and right after a restart.

// Longest hold, counted on the monotonic clock from the time it was set or read: an updater that never comes back
// holds the mower at most this long, whatever the wall clock says after a power loss.
constexpr int64_t kMaxHoldSeconds = 1800;

// hold.json: {"until": <unix s>, "set_at": <unix s>}.
struct HoldRecord {
  int64_t until = 0;
  int64_t set_at = 0;
};

// Both fields as integers and nothing else of another type; empty for anything else.
std::optional<HoldRecord> ParseHold(const std::string& text);
std::string HoldText(const HoldRecord& record);

// Seconds left of a hold at wall time now_wall: until - now_wall, at least 0 and at most kMaxHoldSeconds.
int64_t HoldSeconds(const HoldRecord& record, int64_t now_wall);

struct HoldFile {
  enum class State { kMissing, kRecord, kUnreadable };
  State state = State::kMissing;
  HoldRecord record;
  // kUnreadable: why.
  std::string error;
};

HoldFile ReadHoldFile(const std::string& path);
// Whole or not at all: written next to it as <path>.tmp, synced, renamed over it, the folder synced.
bool WriteHoldFile(const std::string& path, const HoldRecord& record, std::string* error);
// Removes it and syncs the folder; a file that is not there counts as removed.
bool RemoveHoldFile(const std::string& path, std::string* error);

// A hold on the monotonic clock: it ends HoldSeconds after it started, whatever the wall clock does meanwhile.
class HoldTimer {
 public:
  void Start(const HoldRecord& record, double now_mono, int64_t now_wall);
  void Clear();
  bool Active(double now_mono) const;
  // The record it started from, empty when cleared.
  const std::optional<HoldRecord>& record() const {
    return record_;
  }
  // until of the record while active, else 0.
  int64_t Until(double now_mono) const;

 private:
  std::optional<HoldRecord> record_;
  double end_mono_ = 0;
};

struct HoldView {
  bool held = false;
  // The file is there but cannot be read or is not a hold: held until it changes or goes (fail-closed).
  bool unreadable = false;
  // unix s, 0 when not held or unreadable.
  int64_t until = 0;
  std::string error;
};

// The drivers' view of the file: one stat per poll, read again only when the file changed.
class HoldWatch {
 public:
  // An empty path never holds.
  explicit HoldWatch(std::string path);
  HoldView Poll(double now_mono, int64_t now_wall);
  const std::string& path() const {
    return path_;
  }

 private:
  struct Identity {
    dev_t dev = 0;
    ino_t ino = 0;
    off_t size = 0;
    int64_t mtime_s = 0;
    int64_t mtime_ns = 0;
    bool operator==(const Identity& other) const {
      return dev == other.dev && ino == other.ino && size == other.size && mtime_s == other.mtime_s &&
             mtime_ns == other.mtime_ns;
    }
  };

  std::string path_;
  std::optional<Identity> seen_;
  HoldTimer timer_;
  bool unreadable_ = false;
  std::string error_;
};

}  // namespace mowbite_hold

#endif  // MOWBITE_HOLD_HOLD_H
