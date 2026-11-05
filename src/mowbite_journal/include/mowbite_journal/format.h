#ifndef MOWBITE_JOURNAL_FORMAT_H
#define MOWBITE_JOURNAL_FORMAT_H

#include <cstddef>
#include <cstdint>

namespace mowbite_journal {

// A segment file: kMagic, the format version (u32), then blocks of [u32 raw size][u32 packed size][zstd frame]. A
// block unpacks to records of [u16 stream][u16 method][i64 monotonic ns][u32 size][payload], all little endian.
constexpr char kMagic[4] = {'M', 'B', 'J', 'S'};
constexpr uint32_t kFormatVersion = 1;
constexpr size_t kFileHeaderSize = 8;
constexpr size_t kBlockHeaderSize = 8;
constexpr size_t kRecordHeaderSize = 16;
constexpr const char* kSegmentSuffix = ".mbj";

// Stream 0 is the journal itself; every core and raw stream gets its own number in the process when it is defined.
constexpr uint16_t kJournalStream = 0;
enum JournalMethod : uint16_t {
  // CBOR: process, versions, segment.
  kHead = 0,
  // CBOR: a stream with its name, kind, config and method table.
  kDefine = 1,
  // CBOR: records that were not written, by stream.
  kGap = 2,
  // CBOR: the segment ends, with the next one or the end of the process.
  kEnd = 3,
  // CBOR: what the writer measured since the process started.
  kStats = 4,
};

// A core call: [u32 size of the arguments][arguments][result][out parameters]. The view of a core after a call that
// changed it comes as its own record.
constexpr uint16_t kViewMethod = 0xFFFF;
// A raw stream (bytes on a port) has one method, the bytes themselves.
constexpr uint16_t kRawBytes = 0;

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_FORMAT_H
