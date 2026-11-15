#ifndef MOWBITE_JOURNAL_TIMELINE_H
#define MOWBITE_JOURNAL_TIMELINE_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mowbite_journal {

// What happened from from_ns to to_ns (monotonic, both included) in the runs (each a run or a journal_dir), one JSON
// line per event in the order of time over all of them, without knowing any core: {"t_ns", "process", "stream"} and
// "view" (decoded as the definition says), "method" (its name, no arguments), "gap" ({"why", "dropped": {stream: n}}),
// "end" (the reason) or, for a raw stream, "records" and "bytes" of one second. Views, gaps, ends and raw counts are
// always there; method lines only while all lines stay within max_bytes, the latest first, and a last line
// {"truncated": n} counts those left out. Empty with error for a dir without a journal or an unreadable segment.
std::string Timeline(const std::vector<std::string>& run_dirs, int64_t from_ns, int64_t to_ns, size_t max_bytes,
                     std::string* error);

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_TIMELINE_H
