#ifndef MOWBITE_JOURNAL_JOURNAL_H
#define MOWBITE_JOURNAL_JOURNAL_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mowbite_journal/codec.h"

namespace mowbite_journal {

struct Options {
  // journal_dir; empty: no journal.
  std::string dir;
  // Name of the platform for the head of every segment (tron, nx60i, sim).
  std::string platform;
  // All runs in dir together, every process counts them: whole old runs go first, then the oldest segments of the
  // running ones (the rest of such a run cannot be replayed from the start any more).
  double max_mb = 2048;
  double max_days = 14;
  // Below this much free space on the file system of dir nothing is written until there is room again.
  double reserve_mb = 1024;
  // Each of the two buffers; a record that does not fit is dropped and counted.
  size_t buffer_kb = 1024;
  // A segment ends after this time or size (packed).
  double segment_s = 300;
  double segment_mb = 16;
  // Packed and written this often, earlier when a buffer is half full.
  double flush_s = 1.0;
};

struct StreamStats {
  uint16_t stream = 0;
  std::string name;
  uint64_t records = 0;
  uint64_t bytes = 0;
  uint64_t dropped = 0;
};

struct Stats {
  bool writing = false;
  std::string run_dir;
  // Why nothing is written right now, empty while it is.
  std::string problem;
  uint64_t segments = 0;
  // Unpacked and packed bytes written since the start.
  uint64_t raw_bytes = 0;
  uint64_t packed_bytes = 0;
  uint64_t dropped = 0;
  double longest_write_s = 0;
  // CPU time of the writer thread (packing, writing, keeping the limits).
  double writer_cpu_s = 0;
  double seconds = 0;
  std::vector<StreamStats> streams;
};

// One journal per process: the producers copy records into a buffer under a small lock and never wait for the disk;
// a thread of its own swaps the buffers, packs them with zstd and writes once a flush_s, at the idle I/O class.
class Journal {
 public:
  // The journal of this process: the first call with a dir opens it, later calls get the same one. Null without a dir
  // or when the run cannot be created (error says why).
  static Journal* Open(const Options& options, std::string* error);
  // A journal of its own with a new run in options.dir, for tests and tools.
  static std::unique_ptr<Journal> Create(const Options& options, std::string* error);

  // A stream with {"name", "kind", ...}: gets its number in "stream", goes into the journal now and at the start of
  // every later segment.
  uint16_t Define(json definition);
  // Copies one record; from any thread, also from I/O threads: no allocation, no I/O, no log line.
  void Append(uint16_t stream, uint16_t method, const uint8_t* data, size_t size);

  Stats GetStats() const;
  const Options& options() const;

  ~Journal();
  Journal(const Journal&) = delete;
  Journal& operator=(const Journal&) = delete;

  class Impl;

 private:
  explicit Journal(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

// Monotonic ns, the time of every record.
int64_t MonotonicNs();

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_JOURNAL_H
