#ifndef MOWBITE_HOLD_RESTART_H
#define MOWBITE_HOLD_RESTART_H

#include <cstdint>
#include <optional>
#include <string>

namespace mowbite_hold {

// The restart hold (G35): the kernel crashed, or MowBite OS ended without its own stop while the mower was not IDLE.
// Until a release nothing drives or mows, also after power off and on. The logic decides at its start and writes
// <data_dir>/restart-hold.json; S2 latches UNEXPECTED_RESTART over hw/emergency while the file is there.

// Emergency bit of hw/status and robot_state while the restart hold stands.
constexpr uint32_t kReasonUnexpectedRestart = 1u << 21;
// The bits a node above the driver may latch with over hw/emergency: HIGH_LEVEL, TILT, FENCE, STUCK and
// UNEXPECTED_RESTART.
constexpr uint32_t kHostLatchReasons = (1u << 7) | (1u << 13) | (1u << 14) | (1u << 15) | kReasonUnexpectedRestart;

// In data_dir.
constexpr char kSessionFile[] = "os-session.json";
constexpr char kRestartHoldFile[] = "restart-hold.json";

constexpr char kIdle[] = "IDLE";

// os-session.json: the state of the logic since its last change, written at every change; clean after its own stop.
struct SessionRecord {
  // As SessionState gives it.
  std::string state;
  // The job of mowing, empty without one.
  std::string job_id;
  // /proc/sys/kernel/random/boot_id of the boot it was written in.
  std::string boot_id;
  // unix s of the change.
  int64_t since = 0;
  bool clean = false;
};

std::optional<SessionRecord> ParseSession(const std::string& text);
std::string SessionText(const SessionRecord& record);

// Whether a start after this record, not ended clean, holds (KERNEL_CRASH holds whatever the record says).
bool HoldsOnRestart(const SessionRecord& record);
// Whether the logic waits for next to be on the disk before it acts: next would hold after a crash and the record
// surely on the disk does not (G35).
bool MustWaitForSession(bool written_holds, const SessionRecord& next);

// current_state of robot_state, with IDLE replaced by its sub_state while it drives (DRIVING, CALIBRATING).
std::string SessionState(const std::string& state, const std::string& sub_state);

enum class Cause { kNone, kKernelCrash, kOsCrash, kPowerLoss };

// KERNEL_CRASH, OS_CRASH, POWER_LOSS; empty for kNone.
const char* CauseName(Cause cause);

// What a start finds about the end before it, and whether the mower is held for it.
struct Verdict {
  Cause cause = Cause::kNone;
  // English, for the event and the log.
  std::string detail;
  // unix s of the kernel crash, 0 when not known.
  int64_t crashed_at = 0;
  // state of the last session, empty when not known.
  std::string state_before;
  std::string job_id_before;
  bool hold = false;
};

// The session file as a start finds it: absent, a record, or there but not readable (error says why).
struct SessionFile {
  bool present = false;
  std::optional<SessionRecord> record;
  std::string error;
};

SessionFile ReadSessionFile(const std::string& path);

// The rule of the start. The session of this boot ended clean: nothing; not clean: OS_CRASH. The first start of a
// boot: a kernel record ("<unix s> <reason>" of the platform, none where it has none) is KERNEL_CRASH, else a session
// that did not end clean is POWER_LOSS. KERNEL_CRASH always holds, the others only after a state other than IDLE; a
// session file that cannot be read counts as POWER_LOSS in an unknown state and holds.
Verdict JudgeStart(const SessionFile& session, const std::string& boot_id, const std::optional<std::string>& kernel);

// restart-hold.json: the verdict that set it and when.
std::string RestartHoldText(const Verdict& verdict, int64_t since);

// Whether the file is there; one that cannot be looked at counts as there (fail-closed).
bool RestartHoldPresent(const std::string& path);

// /proc/sys/kernel/random/boot_id without the line end, empty when it cannot be read.
std::string ReadBootId(const std::string& path = "/proc/sys/kernel/random/boot_id");

// The kernel record of the platform, none when path is empty or the file is not there; one that cannot be read is
// its error text.
std::optional<std::string> ReadKernelRecord(const std::string& path);

// Written whole or not at all, like the hold; a file that is not there counts as removed.
bool WriteRecordFile(const std::string& path, const std::string& text, std::string* error);
bool RemoveRecordFile(const std::string& path, std::string* error);

}  // namespace mowbite_hold

#endif  // MOWBITE_HOLD_RESTART_H
