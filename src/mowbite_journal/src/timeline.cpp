#include "mowbite_journal/timeline.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <tuple>

#include "mowbite_journal/codec.h"
#include "mowbite_journal/format.h"
#include "mowbite_journal/reader.h"

namespace mowbite_journal {

namespace {

constexpr int64_t kSecondNs = 1000000000;

struct StreamInfo {
  std::string name;
  std::string kind;
  std::string view;
  std::vector<std::string> methods;
};

struct Line {
  int64_t t_ns;
  size_t run;
  size_t seq;
  std::string text;
};

// A method call, its text made only when it gets a place.
struct Call {
  int64_t t_ns;
  size_t run;
  size_t seq;
  const std::string* process;
  const std::string* stream;
  const std::string* method;
};

struct RawSecond {
  int64_t first_ns = 0;
  size_t seq = 0;
  uint64_t records = 0;
  uint64_t bytes = 0;
};

std::string Dump(const json& value) {
  return value.dump(-1, ' ', false, json::error_handler_t::replace);
}

std::string Text(int64_t t_ns, const std::string& process, const std::string& stream, const char* key,
                 const std::string& value) {
  return "{\"t_ns\":" + std::to_string(t_ns) + ",\"process\":" + Dump(process) + ",\"stream\":" + Dump(stream) +
         ",\"" + key + "\":" + value + "}";
}

template <typename T>
bool Number(const uint8_t* data, size_t size, json* value) {
  if (size != sizeof(T)) return false;
  T number;
  std::memcpy(&number, data, sizeof(T));
  *value = number;
  return true;
}

// The view as its definition says, CBOR when it reads as one without a word, hex else.
json DecodeView(const std::string& encoding, const uint8_t* data, size_t size) {
  json value;
  if (encoding == "bool" && size == 1) return data[0] != 0;
  if ((encoding == "u8" && Number<uint8_t>(data, size, &value)) ||
      (encoding == "u16" && Number<uint16_t>(data, size, &value)) ||
      (encoding == "u32" && Number<uint32_t>(data, size, &value)) ||
      (encoding == "u64" && Number<uint64_t>(data, size, &value)) ||
      (encoding == "i8" && Number<int8_t>(data, size, &value)) ||
      (encoding == "i16" && Number<int16_t>(data, size, &value)) ||
      (encoding == "i32" && Number<int32_t>(data, size, &value)) ||
      (encoding == "i64" && Number<int64_t>(data, size, &value)) ||
      (encoding == "f32" && Number<float>(data, size, &value)) ||
      (encoding == "f64" && Number<double>(data, size, &value))) {
    return value;
  }
  if ((encoding == "cbor" || encoding.empty()) && size >= 4) {
    uint32_t cbor_size;
    std::memcpy(&cbor_size, data, 4);
    if (cbor_size == size - 4) {
      try {
        return json::from_cbor(data + 4, data + size);
      } catch (const std::exception&) {
      }
    }
  }
  return {{"hex", Hex(data, size)}};
}

}  // namespace

std::string Timeline(const std::vector<std::string>& run_dirs, int64_t from_ns, int64_t to_ns, size_t max_bytes,
                     std::string* error) {
  std::vector<std::string> runs;
  for (const auto& dir : run_dirs) {
    const auto found = ListRuns(dir);
    if (found.empty()) {
      *error = dir + ": no journal";
      return "";
    }
    runs.insert(runs.end(), found.begin(), found.end());
  }
  // Per run: its process and streams; map nodes stay where they are, the calls point into them.
  std::vector<std::string> processes(runs.size());
  std::vector<std::map<uint16_t, StreamInfo>> streams(runs.size());
  std::vector<Line> lines;
  std::vector<Call> calls;
  for (size_t r = 0; r < runs.size(); ++r) {
    processes[r] = runs[r].substr(runs[r].find_last_of('/') + 1);
    std::map<std::pair<uint16_t, int64_t>, RawSecond> raw;
    size_t seq = 0;
    for (const auto& path : ListRun(runs[r]).segments) {
      Segment segment;
      if (!ReadSegment(path, &segment, error)) return "";
      for (const Record& record : segment.records) {
        ++seq;
        const bool inside = record.t_ns >= from_ns && record.t_ns <= to_ns;
        if (record.stream == kJournalStream) {
          if (record.method != kHead && record.method != kDefine && (!inside || record.method == kStats)) continue;
          json value;
          try {
            value = json::from_cbor(record.data, record.data + record.size);
          } catch (const std::exception& e) {
            *error = path + ": " + e.what();
            return "";
          }
          // A record of other types than the journal writes is left out, not the whole timeline.
          if (!value.is_object()) continue;
          try {
            if (record.method == kHead) {
              if (value.contains("process") && value["process"].is_string()) {
                processes[r] = value["process"].get<std::string>();
              }
            } else if (record.method == kDefine) {
              const uint16_t id = value.value("stream", uint16_t{0});
              if (streams[r].count(id)) continue;
              StreamInfo& stream = streams[r][id];
              stream.name = value.value("name", "");
              stream.kind = value.value("kind", "");
              stream.view = value.value("view", "");
              for (const auto& method : value.value("methods", json::array())) {
                stream.methods.push_back(method.value("name", "?"));
              }
            } else if (record.method == kGap) {
              json dropped = json::object();
              for (const auto& entry : value.value("dropped", json::array())) {
                const auto id = entry.at(0).get<uint16_t>();
                const auto found = streams[r].find(id);
                const std::string name = id == kJournalStream          ? std::string("journal")
                                         : found != streams[r].end() ? found->second.name
                                                                     : "#" + std::to_string(id);
                dropped[name] = entry.at(1);
              }
              json gap = {{"why", value.value("why", "")}, {"dropped", dropped}};
              lines.push_back({record.t_ns, r, seq, Text(record.t_ns, processes[r], "journal", "gap", Dump(gap))});
            } else if (record.method == kEnd) {
              lines.push_back({record.t_ns, r, seq,
                               Text(record.t_ns, processes[r], "journal", "end", Dump(value.value("reason", "")))});
            }
          } catch (const json::exception&) {
          }
          continue;
        }
        if (!inside) continue;
        const auto found = streams[r].find(record.stream);
        if (found == streams[r].end()) continue;
        const StreamInfo& stream = found->second;
        if (stream.kind == "raw") {
          RawSecond& second = raw[{record.stream, record.t_ns / kSecondNs}];
          if (second.records == 0) {
            second.first_ns = record.t_ns;
            second.seq = seq;
          }
          ++second.records;
          second.bytes += record.size;
        } else if (record.method == kViewMethod) {
          lines.push_back({record.t_ns, r, seq,
                           Text(record.t_ns, processes[r], stream.name, "view",
                                Dump(DecodeView(stream.view, record.data, record.size)))});
        } else if (record.method < stream.methods.size()) {
          calls.push_back({record.t_ns, r, seq, &processes[r], &stream.name, &stream.methods[record.method]});
        }
      }
    }
    for (const auto& [key, second] : raw) {
      const std::string text = "{\"t_ns\":" + std::to_string(second.first_ns) + ",\"process\":" + Dump(processes[r]) +
                               ",\"stream\":" + Dump(streams[r][key.first].name) +
                               ",\"records\":" + std::to_string(second.records) +
                               ",\"bytes\":" + std::to_string(second.bytes) + "}";
      lines.push_back({second.first_ns, r, second.seq, text});
    }
  }
  size_t used = 0;
  for (const auto& line : lines) used += line.text.size() + 1;
  std::sort(calls.begin(), calls.end(), [](const Call& a, const Call& b) {
    return std::tie(a.t_ns, a.run, a.seq) > std::tie(b.t_ns, b.run, b.seq);
  });
  size_t kept = 0;
  for (; kept < calls.size(); ++kept) {
    const Call& call = calls[kept];
    std::string text = Text(call.t_ns, *call.process, *call.stream, "method", Dump(*call.method));
    if (used + text.size() + 1 > max_bytes) break;
    used += text.size() + 1;
    lines.push_back({call.t_ns, call.run, call.seq, std::move(text)});
  }
  std::sort(lines.begin(), lines.end(), [](const Line& a, const Line& b) {
    return std::tie(a.t_ns, a.run, a.seq) < std::tie(b.t_ns, b.run, b.seq);
  });
  std::string out;
  out.reserve(used + 32);
  for (const auto& line : lines) out += line.text + '\n';
  if (kept < calls.size()) out += "{\"truncated\":" + std::to_string(calls.size() - kept) + "}\n";
  return out;
}

}  // namespace mowbite_journal
