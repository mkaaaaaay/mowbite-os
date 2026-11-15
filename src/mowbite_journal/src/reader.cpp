#include "mowbite_journal/reader.h"

#include <dirent.h>
#include <sys/stat.h>
#include <zstd.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <memory>

#include "mowbite_journal/format.h"

namespace mowbite_journal {

bool ReadSegment(const std::string& path, Segment* segment, std::string* error) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    if (error) *error = "cannot open " + path;
    return false;
  }
  file.seekg(0, std::ios::end);
  std::vector<uint8_t> bytes(static_cast<size_t>(file.tellg()));
  file.seekg(0);
  file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (bytes.size() < kFileHeaderSize || std::memcmp(bytes.data(), kMagic, 4) != 0) {
    if (error) *error = path + " is no journal segment";
    return false;
  }
  uint32_t version;
  std::memcpy(&version, bytes.data() + 4, 4);
  if (version != kFormatVersion) {
    if (error) *error = path + " has format " + std::to_string(version) + ", this reader " +
                        std::to_string(kFormatVersion);
    return false;
  }
  segment->raw.clear();
  segment->records.clear();
  segment->problem.clear();
  size_t at = kFileHeaderSize;
  while (at < bytes.size()) {
    if (bytes.size() - at < kBlockHeaderSize) {
      segment->problem = "cut off in a block header";
      break;
    }
    uint32_t raw_size, packed_size;
    std::memcpy(&raw_size, bytes.data() + at, 4);
    std::memcpy(&packed_size, bytes.data() + at + 4, 4);
    if (bytes.size() - at - kBlockHeaderSize < packed_size) {
      segment->problem = "cut off in a block";
      break;
    }
    // The writer puts the size into every frame: a block header that disagrees is broken, not a reason to allocate.
    const unsigned long long content = ZSTD_getFrameContentSize(bytes.data() + at + kBlockHeaderSize, packed_size);
    if (content != raw_size) {
      segment->problem = "broken block at byte " + std::to_string(at);
      break;
    }
    const size_t start = segment->raw.size();
    segment->raw.resize(start + raw_size);
    const size_t n = ZSTD_decompress(segment->raw.data() + start, raw_size, bytes.data() + at + kBlockHeaderSize,
                                     packed_size);
    if (ZSTD_isError(n) || n != raw_size) {
      segment->raw.resize(start);
      segment->problem = "broken block at byte " + std::to_string(at);
      break;
    }
    at += kBlockHeaderSize + packed_size;
  }
  size_t pos = 0;
  const auto& raw = segment->raw;
  while (pos + kRecordHeaderSize <= raw.size()) {
    const RecordHeader h = GetRecordHeader(raw.data() + pos);
    if (raw.size() - pos - kRecordHeaderSize < h.size) {
      segment->problem = "record beyond its block";
      break;
    }
    segment->records.push_back({h.stream, h.method, h.t_ns, raw.data() + pos + kRecordHeaderSize, h.size});
    pos += kRecordHeaderSize + h.size;
  }
  return true;
}

namespace {

// A head carries versions and build ids, a few kB; more is a broken size.
constexpr size_t kMaxHeadBytes = 1 << 20;
constexpr size_t kHeadChunk = 16 * 1024;

}  // namespace

bool ReadSegmentHead(const std::string& path, json* head, std::string* error) {
  auto fail = [&](const std::string& why) {
    if (error) *error = path + ": " + why;
    return false;
  };
  std::ifstream file(path, std::ios::binary);
  if (!file) return fail("cannot open");
  uint8_t start[kFileHeaderSize + kBlockHeaderSize];
  file.read(reinterpret_cast<char*>(start), sizeof(start));
  const size_t got = static_cast<size_t>(file.gcount());
  if (got < kFileHeaderSize || std::memcmp(start, kMagic, 4) != 0) return fail("no journal segment");
  uint32_t version;
  std::memcpy(&version, start + 4, 4);
  if (version != kFormatVersion) return fail("format " + std::to_string(version));
  if (got < sizeof(start)) return fail("cut off before its first block");
  uint32_t packed_left;
  std::memcpy(&packed_left, start + kFileHeaderSize + 4, 4);

  std::unique_ptr<ZSTD_DCtx, size_t (*)(ZSTD_DCtx*)> zstd(ZSTD_createDCtx(), ZSTD_freeDCtx);
  if (!zstd) return fail("no zstd context");
  std::vector<uint8_t> raw(kRecordHeaderSize);
  std::vector<char> chunk(kHeadChunk);
  ZSTD_outBuffer out{raw.data(), raw.size(), 0};
  ZSTD_inBuffer in{chunk.data(), 0, 0};
  bool sized = false;
  for (;;) {
    if (out.pos == out.size) {
      if (sized) break;
      const RecordHeader h = GetRecordHeader(raw.data());
      if (h.stream != kJournalStream || h.method != kHead) return fail("first record is no head");
      if (h.size > kMaxHeadBytes) return fail("head of " + std::to_string(h.size) + " bytes");
      raw.resize(kRecordHeaderSize + h.size);
      out = {raw.data(), raw.size(), out.pos};
      sized = true;
      continue;
    }
    if (in.pos == in.size) {
      if (packed_left == 0) break;
      file.read(chunk.data(), static_cast<std::streamsize>(std::min<size_t>(chunk.size(), packed_left)));
      const size_t n = static_cast<size_t>(file.gcount());
      if (n == 0) break;
      packed_left -= static_cast<uint32_t>(n);
      in = {chunk.data(), n, 0};
    }
    const size_t left = ZSTD_decompressStream(zstd.get(), &out, &in);
    if (ZSTD_isError(left)) return fail("broken first block");
    if (left == 0 && out.pos < out.size) break;
  }
  if (!sized || out.pos < out.size) return fail("cut off before the end of its head");
  json value = json::from_cbor(raw.begin() + kRecordHeaderSize, raw.end(), true, false);
  if (!value.is_object()) return fail("broken head");
  *head = std::move(value);
  return true;
}

namespace {

std::vector<std::string> Entries(const std::string& dir) {
  std::vector<std::string> names;
  DIR* d = opendir(dir.c_str());
  if (!d) return names;
  while (dirent* entry = readdir(d)) {
    std::string name = entry->d_name;
    if (name != "." && name != "..") names.push_back(name);
  }
  closedir(d);
  std::sort(names.begin(), names.end());
  return names;
}

bool IsSegmentName(const std::string& name) {
  const size_t suffix = std::strlen(kSegmentSuffix);
  return name.size() > suffix && name.compare(name.size() - suffix, suffix, kSegmentSuffix) == 0 &&
         std::all_of(name.begin(), name.end() - static_cast<long>(suffix), [](char c) { return c >= '0' && c <= '9'; });
}

}  // namespace

RunFiles ListRun(const std::string& dir) {
  RunFiles run;
  run.dir = dir;
  bool cut = false;
  for (const auto& name : Entries(dir)) {
    if (name == "cut") cut = true;
    if (!IsSegmentName(name)) continue;
    run.segments.push_back(dir + "/" + name);
    run.indices.push_back(static_cast<unsigned>(std::stoul(name)));
  }
  if (cut) {
    run.missing = "its first segments were removed for the limits";
  } else if (!run.indices.empty() && run.indices.front() != 0) {
    run.missing = "segment 0 is missing";
  } else {
    for (size_t i = 1; i < run.indices.size(); ++i) {
      if (run.indices[i] != run.indices[i - 1] + 1) {
        run.missing = "segment " + std::to_string(run.indices[i - 1] + 1) + " is missing";
        break;
      }
    }
  }
  return run;
}

std::vector<std::string> ListRuns(const std::string& dir) {
  std::vector<std::string> runs;
  bool segments_here = false;
  for (const auto& name : Entries(dir)) {
    if (IsSegmentName(name)) segments_here = true;
    struct stat st {};
    const std::string path = dir + "/" + name;
    if (stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode) && !ListRun(path).segments.empty()) runs.push_back(path);
  }
  if (segments_here) return {dir};
  return runs;
}

}  // namespace mowbite_journal
