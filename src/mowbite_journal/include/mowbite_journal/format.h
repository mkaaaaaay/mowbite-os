#ifndef MOWBITE_JOURNAL_FORMAT_H
#define MOWBITE_JOURNAL_FORMAT_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

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

// The reasons of kEnd: the next segment follows, or the process ended.
constexpr const char* kEndRotate = "rotate";
constexpr const char* kEndExit = "exit";

// A core call: [u32 size of the arguments][arguments][result][out parameters]. The view of a core after a call that
// changed it comes as its own record.
constexpr uint16_t kViewMethod = 0xFFFF;
// A raw stream (bytes on a port) has one method, the bytes themselves.
constexpr uint16_t kRawBytes = 0;

struct RecordHeader {
  uint16_t stream = 0;
  uint16_t method = 0;
  int64_t t_ns = 0;
  uint32_t size = 0;
};

inline void PutRecordHeader(uint8_t* at, const RecordHeader& h) {
  std::memcpy(at, &h.stream, 2);
  std::memcpy(at + 2, &h.method, 2);
  std::memcpy(at + 4, &h.t_ns, 8);
  std::memcpy(at + 12, &h.size, 4);
}

inline RecordHeader GetRecordHeader(const uint8_t* at) {
  RecordHeader h;
  std::memcpy(&h.stream, at, 2);
  std::memcpy(&h.method, at + 2, 2);
  std::memcpy(&h.t_ns, at + 4, 8);
  std::memcpy(&h.size, at + 12, 4);
  return h;
}

inline std::string Hex(const uint8_t* data, size_t size) {
  static const char kDigits[] = "0123456789abcdef";
  std::string hex(2 * size, '0');
  for (size_t i = 0; i < size; ++i) {
    hex[2 * i] = kDigits[data[i] >> 4];
    hex[2 * i + 1] = kDigits[data[i] & 15];
  }
  return hex;
}

}  // namespace mowbite_journal

#endif  // MOWBITE_JOURNAL_FORMAT_H
