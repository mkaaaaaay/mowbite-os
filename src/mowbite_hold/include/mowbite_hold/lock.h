#ifndef MOWBITE_HOLD_LOCK_H
#define MOWBITE_HOLD_LOCK_H

#include <sys/types.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mowbite_hold {

// The lock of autonomous driving and mowing. The safety measurement date of hw/capabilities opens it; the setup
// program closes it again with <data_dir>/safety_lock.json when a safety check failed, at once and against any date,
// also one kept by hand. Every reader of the date takes it through SafetyMeasured.

constexpr char kSafetyLockFile[] = "safety_lock.json";
// The mower the setup program chose ({"platform", "t"}), for a start without MOWBITE_PLATFORM (stack.py).
constexpr char kPlatformFile[] = "platform.json";

// Why the lock is closed: a safety step that failed, or a reset (the program opened it itself and was reset, or the
// file could not be read), which only a passed summary takes away.
struct LockCause {
  std::string step;
  std::string part;
  // A reason of SETUP_STEP, or with reset: UNREADABLE or RESET.
  std::string reason;
  std::optional<double> value;
  std::optional<double> limit;
  // unix s.
  int64_t t = 0;
  std::string firmware;
  bool reset = false;
};

// safety_lock.json: {"failures": n, "causes": [{"step", "part", "reason", "value", "limit", "t", "firmware"} |
// {"reset": true, "reason", "t"}]}. failures goes up with every failed safety step and never down.
struct LockRecord {
  uint32_t failures = 0;
  std::vector<LockCause> causes;
};

// Nothing for a text that is not of this form.
std::optional<LockRecord> ParseSafetyLock(const std::string& text);
std::string SafetyLockText(const LockRecord& record);

struct SafetyLock {
  bool closed = false;
  // The file is there but cannot be read or is not of the form: closed (fail-closed).
  bool unreadable = false;
  uint32_t failures = 0;
  // English, why it is closed; empty while open.
  std::string text;
};

// The file missing: open. Any other error, a file larger than the form allows, or one without causes: closed.
SafetyLock ReadSafetyLock(const std::string& path);

// The one rule: open with a safety measurement date and no closed lock.
bool SafetyMeasured(const std::string& date, const SafetyLock& lock);

// The readers' view of the file: one stat per poll, read again only when the file changed.
class SafetyLockWatch {
 public:
  // An empty path is never closed.
  explicit SafetyLockWatch(std::string path);
  const SafetyLock& Poll();
  // What the last poll found.
  const SafetyLock& last() const {
    return lock_;
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
  SafetyLock lock_;
};

}  // namespace mowbite_hold

#endif  // MOWBITE_HOLD_LOCK_H
