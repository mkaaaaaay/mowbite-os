#ifndef MOWBITE_JOURNAL_READER_H
#define MOWBITE_JOURNAL_READER_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mowbite_journal {

struct Record {
  uint16_t stream = 0;
  uint16_t method = 0;
  int64_t t_ns = 0;
  const uint8_t* data = nullptr;
  size_t size = 0;
};

// One segment file unpacked: its records in order. Reading stops at the first block that is cut off or broken, as
// after a crash; problem says so.
struct Segment {
  std::vector<uint8_t> raw;
  std::vector<Record> records;
  std::string problem;
};

bool ReadSegment(const std::string& path, Segment* segment, std::string* error);

// A run: the segment files of one process in their order, and whether some before the first are missing.
struct RunFiles {
  std::string dir;
  std::vector<std::string> segments;
  std::vector<unsigned> indices;
  // Why the run cannot be replayed from its start, empty if it can.
  std::string missing;
};

RunFiles ListRun(const std::string& dir);
// The runs in a journal_dir, or the run itself if dir is one.
std::vector<std::string> ListRuns(const std::string& dir);

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_READER_H
