#ifndef MOWBITE_JOURNAL_RUN_INDEX_H
#define MOWBITE_JOURNAL_RUN_INDEX_H

#include <cstdint>
#include <string>
#include <vector>

namespace mowbite_journal {

struct SegmentInfo {
  std::string path;
  unsigned index = 0;
  // From the head of the segment; 0 and empty when it cannot be read.
  int64_t start_ns = 0;
  std::string start_utc;
  uint64_t bytes = 0;
  // A later segment of the run exists, or this one holds its end record.
  bool closed = false;
};

struct RunInfo {
  std::string dir;
  std::string name;
  // From the first segment whose head can be read.
  std::string process;
  std::string boot_id;
  std::string revision;
  std::string image;
  std::string platform;
  int64_t pid = 0;
  // A process holds the lock of the run right now.
  bool active = false;
  // The last segment ends with the end of the process; only known after reading its end.
  bool ended = false;
  // The reason of the end record in the last segment, empty without one.
  std::string end_reason;
  std::vector<SegmentInfo> segments;
  // Why the run cannot be replayed from its start, empty if it can.
  std::string missing;
  // Segments that are broken or cut off, empty if none.
  std::string problem;
};

// One segment from the head in its first block, and its size; closed is left to the caller.
bool InspectSegment(const std::string& path, SegmentInfo* info, std::string* error);

// A run from the heads of its segments; read_end also unpacks the last segment of a run that is not active, for
// ended and end_reason (an active run has not ended and its last block may still be written). The head of the
// segment an active run is writing is no problem while it is missing.
RunInfo InspectRun(const std::string& dir, bool read_end);
// The runs in a journal_dir that have segments, oldest first; other folders there are skipped.
std::vector<RunInfo> InspectRuns(const std::string& journal_dir, bool read_end_of_inactive);

struct RunState;

// The runs of a journal_dir kept up to date like InspectRuns: the head and size of a segment that no longer changes
// are read once, the end of a run that is no longer active once, and what is gone from the disk is forgotten.
class RunIndex {
 public:
  explicit RunIndex(std::string journal_dir);
  ~RunIndex();
  RunIndex(RunIndex&&) noexcept;
  RunIndex& operator=(RunIndex&&) noexcept;

  const std::vector<RunInfo>& Refresh(bool read_end_of_inactive);

  // Heads and ends read so far, for tests of the cache.
  uint64_t heads_read() const;
  uint64_t ends_read() const;

 private:
  std::string journal_dir_;
  std::vector<RunInfo> runs_;
  // One for each run, in the same order.
  std::vector<RunState> states_;
  uint64_t heads_read_ = 0;
  uint64_t ends_read_ = 0;
};

// A folder name the journal gives a run: <utc>_<process>_<pid>.
bool IsRunName(const std::string& name);
// True while a process holds the lock of the run; the check takes the lock only for a moment if it is free.
bool RunActive(const std::string& dir);

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_RUN_INDEX_H
