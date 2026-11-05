#include "mowbite_journal/reader.h"

#include <dirent.h>
#include <sys/stat.h>
#include <zstd.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iterator>

#include "mowbite_journal/format.h"

namespace mowbite_journal {

bool ReadSegment(const std::string& path, Segment* segment, std::string* error) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    if (error) *error = "cannot open " + path;
    return false;
  }
  const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
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
  segment->path = path;
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
    Record record;
    uint32_t size;
    std::memcpy(&record.stream, raw.data() + pos, 2);
    std::memcpy(&record.method, raw.data() + pos + 2, 2);
    std::memcpy(&record.t_ns, raw.data() + pos + 4, 8);
    std::memcpy(&size, raw.data() + pos + 12, 4);
    if (raw.size() - pos - kRecordHeaderSize < size) {
      segment->problem = "record beyond its block";
      break;
    }
    record.data = raw.data() + pos + kRecordHeaderSize;
    record.size = size;
    segment->records.push_back(record);
    pos += kRecordHeaderSize + size;
  }
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
