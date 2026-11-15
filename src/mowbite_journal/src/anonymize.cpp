#include "mowbite_journal/anonymize.h"

#include <sys/stat.h>
#include <zstd.h>

#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <map>
#include <memory>

#include "mowbite_journal/format.h"
#include "mowbite_journal/reader.h"
#include "mowbite_journal/replay.h"

namespace mowbite_journal {

namespace {

// A block of a written segment unpacks to at most this much, cut at a record.
constexpr size_t kBlockRaw = 1 << 20;

bool Digits(const std::string& text, size_t at, size_t count) {
  if (at + count > text.size()) return false;
  for (size_t i = at; i < at + count; ++i) {
    if (!std::isdigit(static_cast<unsigned char>(text[i]))) return false;
  }
  return true;
}

// A real date of the calendar: timegm keeps it as it is.
bool RealDate(int year, int month, int day) {
  if (month < 1 || month > 12 || day < 1 || day > 31) return false;
  std::tm tm{};
  tm.tm_year = year - 1900;
  tm.tm_mon = month - 1;
  tm.tm_mday = day;
  tm.tm_hour = 12;
  std::time_t t = timegm(&tm);
  std::tm back{};
  gmtime_r(&t, &back);
  return back.tm_year == year - 1900 && back.tm_mon == month - 1 && back.tm_mday == day;
}

// A name YYYYMMDD... with its date moved.
std::string ShiftRunName(const std::string& name, const AnonymizeOptions& options) {
  if (!Digits(name, 0, 8)) return name;
  int year = std::stoi(name.substr(0, 4)), month = std::stoi(name.substr(4, 2)), day = std::stoi(name.substr(6, 2));
  if (!RealDate(year, month, day)) return name;
  options.ShiftDate(&year, &month, &day);
  char date[16];
  std::snprintf(date, sizeof(date), "%04d%02d%02d", year % 10000, month, day);
  return date + name.substr(8);
}

json AnonymizeHead(json head, const AnonymizeOptions& options) {
  for (const char* key : {"start_utc", "segment_start_utc"}) {
    if (head.contains(key) && head[key].is_string()) head[key] = options.ShiftDates(head[key].get<std::string>());
  }
  if (head.contains("run") && head["run"].is_string()) {
    head["run"] = ShiftRunName(head["run"].get<std::string>(), options);
  }
  if (head.contains("boot_id")) head["boot_id"] = options.boot_id;
  return head;
}

void PutRecord(std::vector<uint8_t>& out, const Record& record, const uint8_t* data, size_t size) {
  const size_t at = out.size();
  out.resize(at + kRecordHeaderSize + size);
  PutRecordHeader(out.data() + at, {record.stream, record.method, record.t_ns, static_cast<uint32_t>(size)});
  if (size > 0) std::memcpy(out.data() + at + kRecordHeaderSize, data, size);
}

void PutCbor(std::vector<uint8_t>& out, const Record& record, const json& value) {
  const std::vector<uint8_t> cbor = json::to_cbor(value);
  PutRecord(out, record, cbor.data(), cbor.size());
}

// A segment file of the records in raw, in blocks of about kBlockRaw.
bool WriteSegment(const std::string& path, const std::vector<uint8_t>& raw, std::string* error) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    *error = "cannot create " + path;
    return false;
  }
  file.write(kMagic, 4);
  file.write(reinterpret_cast<const char*>(&kFormatVersion), 4);
  std::vector<uint8_t> packed;
  size_t start = 0;
  while (start < raw.size()) {
    size_t end = start;
    while (end + kRecordHeaderSize <= raw.size()) {
      const size_t next = end + kRecordHeaderSize + GetRecordHeader(raw.data() + end).size;
      if (next > raw.size() || (end > start && next - start > kBlockRaw)) break;
      end = next;
    }
    packed.resize(ZSTD_compressBound(end - start));
    const size_t n = ZSTD_compress(packed.data(), packed.size(), raw.data() + start, end - start, 3);
    if (ZSTD_isError(n)) {
      *error = std::string("zstd: ") + ZSTD_getErrorName(n);
      return false;
    }
    const uint32_t raw_size = static_cast<uint32_t>(end - start);
    const uint32_t packed_size = static_cast<uint32_t>(n);
    file.write(reinterpret_cast<const char*>(&raw_size), 4);
    file.write(reinterpret_cast<const char*>(&packed_size), 4);
    file.write(reinterpret_cast<const char*>(packed.data()), static_cast<std::streamsize>(n));
    start = end;
  }
  if (!file.flush()) {
    *error = "cannot write " + path;
    return false;
  }
  return true;
}

struct Stream {
  std::string kind;
  std::unique_ptr<CoreReplay> replay;
};

}  // namespace

void AnonymizeOptions::ShiftDate(int* year, int* month, int* day) const {
  std::tm tm{};
  tm.tm_year = *year - 1900;
  tm.tm_mon = *month - 1;
  tm.tm_mday = *day;
  tm.tm_hour = 12;
  const std::time_t t = timegm(&tm) + static_cast<std::time_t>(shift_s());
  std::tm moved{};
  gmtime_r(&t, &moved);
  *year = moved.tm_year + 1900;
  *month = moved.tm_mon + 1;
  *day = moved.tm_mday;
}

std::string AnonymizeOptions::ShiftName(const std::string& name) const {
  return ShiftRunName(name, *this);
}

std::string AnonymizeOptions::ShiftDates(const std::string& text) const {
  std::string out = text;
  for (size_t at = 0; at + 10 <= out.size(); ++at) {
    if (!Digits(out, at, 4) || out[at + 4] != '-' || !Digits(out, at + 5, 2) || out[at + 7] != '-' ||
        !Digits(out, at + 8, 2) || (at > 0 && std::isdigit(static_cast<unsigned char>(out[at - 1]))) ||
        Digits(out, at + 10, 1)) {
      continue;
    }
    int year = std::stoi(out.substr(at, 4)), month = std::stoi(out.substr(at + 5, 2));
    int day = std::stoi(out.substr(at + 8, 2));
    if (!RealDate(year, month, day)) continue;
    ShiftDate(&year, &month, &day);
    char date[16];
    std::snprintf(date, sizeof(date), "%04d-%02d-%02d", year % 10000, month, day);
    out.replace(at, 10, date);
    at += 9;
  }
  return out;
}

bool AnonymizeRun(const std::string& run_dir, const std::string& out_dir, const ReplayRegistry& registry,
                  const AnonymizeOptions& options, std::string* error) {
  const RunFiles files = ListRun(run_dir);
  if (files.segments.empty()) {
    *error = run_dir + " has no segments";
    return false;
  }
  if (mkdir(out_dir.c_str(), 0755) != 0 && errno != EEXIST) {
    *error = "cannot create " + out_dir + ": " + std::strerror(errno);
    return false;
  }
  if (!ListRun(out_dir).segments.empty()) {
    *error = out_dir + " has segments already";
    return false;
  }
  std::map<uint16_t, Stream> streams;
  Out call;
  for (size_t s = 0; s < files.segments.size(); ++s) {
    Segment segment;
    if (!ReadSegment(files.segments[s], &segment, error)) return false;
    std::vector<uint8_t> raw;
    raw.reserve(segment.raw.size());
    for (size_t i = 0; i < segment.records.size(); ++i) {
      const Record& record = segment.records[i];
      const std::string where = files.segments[s] + " record " + std::to_string(i);
      try {
        if (record.stream == kJournalStream) {
          if (record.method == kHead) {
            PutCbor(raw, record, AnonymizeHead(json::from_cbor(record.data, record.data + record.size), options));
          } else if (record.method == kDefine) {
            json definition = json::from_cbor(record.data, record.data + record.size);
            const uint16_t id = definition.at("stream").get<uint16_t>();
            Stream& stream = streams[id];
            stream.kind = definition.value("kind", "");
            if (stream.kind == "core") {
              const std::string name = definition.value("name", "");
              if (!stream.replay) {
                const auto factory = registry.find(name);
                if (factory == registry.end()) {
                  *error = where + ": this tool does not know core " + name;
                  return false;
                }
                stream.replay = factory->second(definition);
              }
              definition["config"] = stream.replay->AnonymizeConfig(definition.at("config"), options);
            } else if (stream.kind != "raw") {
              *error = where + ": stream " + std::to_string(id) + " of unknown kind " + stream.kind;
              return false;
            }
            PutCbor(raw, record, definition);
          } else {
            PutRecord(raw, record, record.data, record.size);
          }
          continue;
        }
        const auto found = streams.find(record.stream);
        if (found == streams.end()) {
          *error = where + ": stream " + std::to_string(record.stream) + " is not defined";
          return false;
        }
        const Stream& stream = found->second;
        if (stream.kind == "raw") {
          if (options.keep_raw) {
            PutRecord(raw, record, record.data, record.size);
          } else {
            const std::vector<uint8_t> zeros(record.size, 0);
            PutRecord(raw, record, zeros.data(), zeros.size());
          }
        } else if (record.method != kViewMethod &&
                   stream.replay->Anonymize(record.method, record.data, record.size, options, &call)) {
          PutRecord(raw, record, call.data(), call.size());
        } else {
          PutRecord(raw, record, record.data, record.size);
        }
      } catch (const std::exception& e) {
        *error = where + ": " + e.what();
        return false;
      }
    }
    const std::string name = files.segments[s].substr(files.segments[s].find_last_of('/') + 1);
    if (!WriteSegment(out_dir + "/" + name, raw, error)) return false;
  }
  std::ifstream cut(run_dir + "/cut");
  if (cut) {
    std::ofstream copy(out_dir + "/cut", std::ios::trunc);
    copy << cut.rdbuf();
  }
  return true;
}

}  // namespace mowbite_journal
