#include "mowbite_journal/replay.h"

#include <cstdio>
#include <set>

namespace mowbite_journal {

namespace detail {

std::vector<std::string> MethodNames(const char* text) {
  std::vector<std::string> names;
  std::string all = text;
  size_t start = 0;
  while (start <= all.size()) {
    size_t end = all.find(',', start);
    if (end == std::string::npos) end = all.size();
    std::string item = all.substr(start, end - start);
    const size_t colon = item.rfind("::");
    if (colon != std::string::npos) item = item.substr(colon + 2);
    const size_t first = item.find_first_not_of(" \t\n&");
    const size_t last = item.find_last_not_of(" \t\n");
    names.push_back(first == std::string::npos ? "" : item.substr(first, last - first + 1));
    start = end + 1;
  }
  return names;
}

std::string TableChecksum(const std::vector<std::string>& names, const std::vector<std::string>& signatures) {
  uint64_t hash = 14695981039346656037ull;
  auto add = [&hash](const std::string& text) {
    for (unsigned char c : text) {
      hash ^= c;
      hash *= 1099511628211ull;
    }
  };
  for (size_t i = 0; i < names.size(); ++i) {
    add(names[i]);
    add("(");
    add(i < signatures.size() ? signatures[i] : "");
    add(");");
  }
  char text[17];
  std::snprintf(text, sizeof(text), "%016llx", static_cast<unsigned long long>(hash));
  return text;
}

}  // namespace detail

namespace {

const char* JournalMethodName(uint16_t method) {
  switch (method) {
    case kHead: return "head";
    case kDefine: return "define";
    case kGap: return "gap";
    case kEnd: return "end";
    case kStats: return "stats";
  }
  return "unknown";
}

struct Stream {
  std::string name;
  std::string kind;
  json methods = json::array();
  std::unique_ptr<CoreReplay> replay;
  std::string stopped;
  uint64_t calls = 0;
  bool differs = false;

  std::string MethodName(uint16_t method) const {
    if (method == kViewMethod) return "view";
    if (kind == "raw") return "bytes";
    if (method < methods.size()) return methods[method].value("name", "?");
    return "#" + std::to_string(method);
  }
};

// Walks the records of a run in order and keeps the streams it defines.
class RunWalk {
 public:
  explicit RunWalk(const ReplayRegistry& registry) : registry_(registry) {}

  std::map<uint16_t, Stream>& streams() {
    return streams_;
  }
  int64_t start_ns() const {
    return start_ns_;
  }

  // The journal's own records; false with error for one that cannot be read.
  bool Journal(const Record& record, std::string* error) {
    const json value = json::from_cbor(record.data, record.data + record.size);
    if (record.method == kHead) {
      if (value.value("format", 0u) != kFormatVersion) {
        *error = "head of format " + value.value("format", json()).dump();
        return false;
      }
      if (start_ns_ == 0) start_ns_ = value.value("start_ns", int64_t{0});
    } else if (record.method == kDefine) {
      Define(value);
    } else if (record.method == kGap) {
      const std::string why = value.value("why", "");
      std::set<uint16_t> hit;
      for (const auto& entry : value.value("dropped", json::array())) hit.insert(entry.at(0).get<uint16_t>());
      for (auto& [id, stream] : streams_) {
        if (hit.count(id) && stream.stopped.empty()) {
          stream.stopped = "gap in the journal (" + why + ")";
        }
      }
    } else if (record.method == kEnd) {
      ended_ = value.value("reason", "") == "exit";
    }
    return true;
  }

  bool ended() const {
    return ended_;
  }

 private:
  void Define(const json& definition) {
    const uint16_t id = definition.at("stream").get<uint16_t>();
    if (streams_.count(id)) return;
    Stream& stream = streams_[id];
    stream.name = definition.value("name", "");
    stream.kind = definition.value("kind", "");
    stream.methods = definition.value("methods", json::array());
    if (stream.kind != "core") return;
    const auto factory = registry_.find(stream.name);
    if (factory == registry_.end()) {
      stream.stopped = "this tool has no replay of core " + stream.name;
      return;
    }
    try {
      stream.replay = factory->second(definition);
    } catch (const std::exception& e) {
      stream.stopped = std::string("cannot rebuild the core: ") + e.what();
    }
  }

  const ReplayRegistry& registry_;
  std::map<uint16_t, Stream> streams_;
  int64_t start_ns_ = 0;
  bool ended_ = false;
};

std::string Seconds(int64_t t_ns, int64_t start_ns) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.6f", static_cast<double>(t_ns - start_ns) * 1e-9);
  return text;
}

void CheckRun(const std::string& dir, const ReplayRegistry& registry, CheckReport& report) {
  auto fail = [&report](CheckOutcome outcome, const std::string& line) {
    report.lines.push_back(line);
    report.outcome = Worse(report.outcome, outcome);
  };
  const RunFiles files = ListRun(dir);
  if (!files.missing.empty()) return fail(CheckOutcome::kNotReplayable, dir + ": not replayable, " + files.missing);
  RunWalk walk(registry);
  for (size_t s = 0; s < files.segments.size(); ++s) {
    Segment segment;
    std::string error;
    if (!ReadSegment(files.segments[s], &segment, &error)) return fail(CheckOutcome::kError, dir + ": " + error);
    for (size_t i = 0; i < segment.records.size(); ++i) {
      const Record& record = segment.records[i];
      const std::string where = files.segments[s] + " record " + std::to_string(i);
      if (record.stream == kJournalStream) {
        try {
          if (!walk.Journal(record, &error)) return fail(CheckOutcome::kError, dir + ": " + error);
        } catch (const std::exception& e) {
          return fail(CheckOutcome::kError, where + ": " + e.what());
        }
        continue;
      }
      auto found = walk.streams().find(record.stream);
      if (found == walk.streams().end()) {
        return fail(CheckOutcome::kError,
                    where + ": stream " + std::to_string(record.stream) + " is not defined");
      }
      Stream& stream = found->second;
      if (!stream.replay || !stream.stopped.empty()) continue;
      std::optional<Mismatch> mismatch;
      try {
        mismatch = record.method == kViewMethod ? stream.replay->CheckView(record.data, record.size)
                                                : stream.replay->Apply(record.method, record.data, record.size);
      } catch (const std::exception& e) {
        mismatch = Mismatch{std::string("cannot read the record: ") + e.what(), {}, {}};
      }
      if (record.method != kViewMethod) ++stream.calls;
      if (mismatch) {
        stream.stopped = "first difference in segment " + std::to_string(files.indices[s]) + ", record " +
                         std::to_string(i) + ", t " + Seconds(record.t_ns, walk.start_ns()) + " s, " +
                         stream.MethodName(record.method) + " (" + mismatch->what + "): expected " +
                         mismatch->expected.dump() + ", got " + mismatch->got.dump();
        stream.differs = true;
      }
    }
    if (!segment.problem.empty()) {
      const bool last = s + 1 == files.segments.size();
      if (!last) {
        for (auto& [id, stream] : walk.streams()) {
          if (stream.stopped.empty()) stream.stopped = "segment " + files.segments[s] + " is " + segment.problem;
        }
      } else {
        report.lines.push_back(dir + ": the last segment is " + segment.problem + " (crash or still running)");
      }
    }
  }
  if (!walk.ended()) report.lines.push_back(dir + ": no end of the process in the journal (crash or still running)");
  for (const auto& [id, stream] : walk.streams()) {
    if (stream.kind != "core") continue;
    if (stream.differs) {
      report.lines.push_back(dir + " " + stream.name + ": differs after " + std::to_string(stream.calls) +
                             " calls, " + stream.stopped);
      report.outcome = Worse(report.outcome, CheckOutcome::kDiffers);
    } else if (!stream.stopped.empty()) {
      report.lines.push_back(dir + " " + stream.name + ": not replayable, " + stream.stopped + " (after " +
                             std::to_string(stream.calls) + " calls)");
      report.outcome = Worse(report.outcome, CheckOutcome::kNotReplayable);
    } else {
      report.lines.push_back(dir + " " + stream.name + ": bit-identical, " + std::to_string(stream.calls) + " calls");
    }
  }
}

// Calls f(segment index, record index, record, walk) for every record of every run; false with error on one that
// cannot be read.
template <typename F>
bool Walk(const std::string& dir, const ReplayRegistry& registry, F&& f, std::string* error) {
  for (const auto& run : ListRuns(dir)) {
    const RunFiles files = ListRun(run);
    RunWalk walk(registry);
    for (size_t s = 0; s < files.segments.size(); ++s) {
      Segment segment;
      if (!ReadSegment(files.segments[s], &segment, error)) return false;
      for (size_t i = 0; i < segment.records.size(); ++i) {
        const Record& record = segment.records[i];
        if (record.stream == kJournalStream && !walk.Journal(record, error)) return false;
        f(run, files.indices[s], i, record, walk);
      }
    }
  }
  return true;
}

// Stream and method of a record by name, known before anything is decoded.
std::pair<std::string, std::string> Names(const Record& record, RunWalk& walk) {
  if (record.stream == kJournalStream) return {"journal", JournalMethodName(record.method)};
  auto found = walk.streams().find(record.stream);
  if (found == walk.streams().end()) {
    return {"#" + std::to_string(record.stream), "#" + std::to_string(record.method)};
  }
  return {found->second.name, found->second.MethodName(record.method)};
}

// The record as JSON; one that cannot be decoded gives its error and hex.
json DecodeRecord(const Record& record, RunWalk& walk) {
  try {
    if (record.stream == kJournalStream) {
      return {{"value", json::from_cbor(record.data, record.data + record.size)}};
    }
    auto found = walk.streams().find(record.stream);
    if (found == walk.streams().end() || !found->second.replay) return {{"hex", Hex(record.data, record.size)}};
    const CoreReplay& replay = *found->second.replay;
    if (record.method == kViewMethod) return {{"view", replay.DecodeView(record.data, record.size)}};
    return replay.Decode(record.method, record.data, record.size);
  } catch (const std::exception& e) {
    return {{"error", e.what()}, {"hex", Hex(record.data, record.size)}};
  }
}

bool Passes(const ExportFilter& filter, const std::string& stream, const std::string& method) {
  return (filter.stream.empty() || filter.stream == stream) && (filter.method.empty() || filter.method == method);
}

void Flatten(const json& value, const std::string& prefix, std::vector<std::pair<std::string, json>>& out) {
  if (value.is_object() || value.is_array()) {
    size_t index = 0;
    for (auto it = value.begin(); it != value.end(); ++it, ++index) {
      const std::string key = value.is_object() ? it.key() : std::to_string(index);
      Flatten(*it, prefix.empty() ? key : prefix + "." + key, out);
    }
  } else {
    out.emplace_back(prefix, value);
  }
}

std::string CsvField(const json& value) {
  if (!value.is_string()) return value.is_null() ? "" : value.dump();
  std::string text = value.get<std::string>();
  if (text.find_first_of(",\"\n") == std::string::npos) return text;
  std::string quoted = "\"";
  for (char c : text) {
    if (c == '"') quoted += '"';
    quoted += c;
  }
  return quoted + "\"";
}

}  // namespace

CheckOutcome Worse(CheckOutcome a, CheckOutcome b) {
  auto rank = [](CheckOutcome o) {
    switch (o) {
      case CheckOutcome::kSame: return 0;
      case CheckOutcome::kNotReplayable: return 1;
      case CheckOutcome::kError: return 2;
      case CheckOutcome::kDiffers: return 3;
    }
    return 0;
  };
  return rank(b) > rank(a) ? b : a;
}

CheckReport CheckRuns(const std::string& dir, const ReplayRegistry& registry) {
  CheckReport report;
  const auto runs = ListRuns(dir);
  if (runs.empty()) {
    report.lines.push_back(dir + ": no journal");
    report.outcome = CheckOutcome::kError;
    return report;
  }
  for (const auto& run : runs) CheckRun(run, registry, report);
  return report;
}

bool ExportJsonl(const std::string& dir, const ReplayRegistry& registry, const ExportFilter& filter, std::ostream& out,
                 std::string* error) {
  return Walk(
      dir, registry,
      [&](const std::string& run, unsigned segment, size_t index, const Record& record, RunWalk& walk) {
        const auto [stream_name, method_name] = Names(record, walk);
        if (!Passes(filter, stream_name, method_name)) return;
        json line = DecodeRecord(record, walk);
        line["run"] = run.substr(run.find_last_of('/') + 1);
        line["segment"] = segment;
        line["record"] = index;
        line["t"] = static_cast<double>(record.t_ns - walk.start_ns()) * 1e-9;
        line["stream"] = stream_name;
        line["method"] = method_name;
        out << line.dump(-1, ' ', false, json::error_handler_t::replace) << '\n';
      },
      error);
}

bool ExportCsv(const std::string& dir, const ReplayRegistry& registry, const ExportFilter& filter, std::ostream& out,
               std::string* error) {
  if (filter.stream.empty() || filter.method.empty()) {
    *error = "--csv needs --stream and --method";
    return false;
  }
  std::vector<std::string> columns;
  return Walk(
      dir, registry,
      [&](const std::string&, unsigned, size_t, const Record& record, RunWalk& walk) {
        const auto [stream_name, method_name] = Names(record, walk);
        if (!Passes(filter, stream_name, method_name)) return;
        const json decoded = DecodeRecord(record, walk);
        std::vector<std::pair<std::string, json>> leaves;
        Flatten(decoded, "", leaves);
        if (columns.empty()) {
          out << "t";
          for (const auto& leaf : leaves) {
            columns.push_back(leaf.first);
            out << ',' << leaf.first;
          }
          out << '\n';
        }
        std::map<std::string, json> row(leaves.begin(), leaves.end());
        out << Seconds(record.t_ns, walk.start_ns());
        for (const auto& column : columns) {
          auto it = row.find(column);
          out << ',' << (it == row.end() ? "" : CsvField(it->second));
        }
        out << '\n';
      },
      error);
}

}  // namespace mowbite_journal
