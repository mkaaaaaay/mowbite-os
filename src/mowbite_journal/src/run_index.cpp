#include "mowbite_journal/run_index.h"

#include <dirent.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>

#include "mowbite_journal/format.h"
#include "mowbite_journal/reader.h"

namespace mowbite_journal {

struct SegmentState {
  // The head is read, or cannot be read in a segment that no longer changes.
  bool head_done = false;
  // bytes is the final size.
  bool bytes_done = false;
  std::string problem;
};

struct RunState {
  bool have_head = false;
  std::vector<SegmentState> segments;
  // The last segment whose end was read, and what it held.
  std::string end_of;
  bool end_closed = false;
  std::string end_reason;
  std::string end_problem;
};

namespace {

std::string Text(const json& head, const char* key) {
  const auto it = head.find(key);
  return it != head.end() && it->is_string() ? it->get<std::string>() : std::string();
}

int64_t Integer(const json& head, const char* key) {
  const auto it = head.find(key);
  return it != head.end() && it->is_number_integer() ? it->get<int64_t>() : 0;
}

void AddProblem(std::string& problem, const std::string& what) {
  if (what.empty()) return;
  if (!problem.empty()) problem += "; ";
  problem += what;
}

std::string BaseName(const std::string& path) {
  return path.substr(path.find_last_of('/') + 1);
}

uint64_t FileBytes(const std::string& path) {
  struct stat st {};
  return stat(path.c_str(), &st) == 0 ? static_cast<uint64_t>(st.st_size) : 0;
}

bool ReadHead(const std::string& path, SegmentInfo* info, json* head, std::string* error) {
  if (!ReadSegmentHead(path, head, error)) return false;
  info->start_ns = Integer(*head, "segment_start_ns");
  info->start_utc = Text(*head, "segment_start_utc");
  return true;
}

// The end record of the last segment of a run that is no longer active.
void ReadEnd(const std::string& path, RunState& state) {
  state.end_of = path;
  state.end_closed = false;
  state.end_reason.clear();
  state.end_problem.clear();
  Segment segment;
  std::string error;
  if (!ReadSegment(path, &segment, &error)) {
    state.end_problem = error;
    return;
  }
  if (!segment.problem.empty()) state.end_problem = path + ": " + segment.problem;
  for (const Record& record : segment.records) {
    if (record.stream != kJournalStream || record.method != kEnd) continue;
    const json end = json::from_cbor(record.data, record.data + record.size, true, false);
    state.end_reason = end.is_object() ? Text(end, "reason") : std::string();
    state.end_closed = true;
  }
}

// Brings run and its state up to date with the files in its folder; what the state holds is not read again.
void Update(RunInfo& run, RunState& state, bool read_end, uint64_t& heads_read, uint64_t& ends_read) {
  // Before the listing: a run that is not active gets no further segments.
  run.active = RunActive(run.dir);
  const RunFiles files = ListRun(run.dir);
  run.missing = files.missing;
  std::vector<SegmentInfo> segments;
  std::vector<SegmentState> states;
  segments.reserve(files.segments.size());
  states.reserve(files.segments.size());
  size_t old = 0;
  for (size_t i = 0; i < files.segments.size(); ++i) {
    const std::string& path = files.segments[i];
    while (old < run.segments.size() && run.segments[old].path < path) ++old;
    if (old < run.segments.size() && run.segments[old].path == path) {
      segments.push_back(std::move(run.segments[old]));
      states.push_back(std::move(state.segments[old]));
      ++old;
    } else {
      segments.emplace_back();
      segments.back().path = path;
      segments.back().index = files.indices[i];
      states.emplace_back();
    }
    SegmentInfo& segment = segments.back();
    SegmentState& done = states.back();
    segment.closed = i + 1 < files.segments.size();
    // Only the last segment of an active run may still grow or wait for its head.
    const bool final = segment.closed || !run.active;
    if (!done.head_done) {
      json head;
      std::string error;
      ++heads_read;
      if (ReadHead(path, &segment, &head, &error)) {
        done.head_done = true;
        done.problem.clear();
        if (!state.have_head) {
          state.have_head = true;
          run.process = Text(head, "process");
          run.pid = Integer(head, "pid");
          run.boot_id = Text(head, "boot_id");
          run.revision = Text(head, "os_revision");
          run.image = Text(head, "image");
          run.platform = Text(head, "platform");
        }
      } else if (final) {
        done.head_done = true;
        done.problem = error;
      }
    }
    if (!done.bytes_done) {
      segment.bytes = FileBytes(path);
      done.bytes_done = final;
    }
  }
  run.segments = std::move(segments);
  state.segments = std::move(states);

  run.ended = false;
  run.end_reason.clear();
  run.problem.clear();
  for (const auto& done : state.segments) AddProblem(run.problem, done.problem);
  if (run.segments.empty()) {
    if (!run.active) AddProblem(run.problem, "no segments");
    return;
  }
  SegmentInfo& last = run.segments.back();
  if (run.active) return;
  if (read_end && state.end_of != last.path) {
    ++ends_read;
    ReadEnd(last.path, state);
  }
  if (state.end_of != last.path) return;
  last.closed = last.closed || state.end_closed;
  run.end_reason = state.end_reason;
  run.ended = state.end_closed && state.end_reason == kEndExit;
  AddProblem(run.problem, state.end_problem);
}

// The folders in journal_dir named like runs, oldest first.
std::vector<std::string> RunDirs(const std::string& journal_dir) {
  std::vector<std::string> dirs;
  DIR* dir = opendir(journal_dir.c_str());
  if (!dir) return dirs;
  while (dirent* entry = readdir(dir)) {
    const std::string name = entry->d_name;
    if (!IsRunName(name)) continue;
    const std::string path = journal_dir + "/" + name;
    struct stat st {};
    if (stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) dirs.push_back(path);
  }
  closedir(dir);
  std::sort(dirs.begin(), dirs.end());
  return dirs;
}

}  // namespace

bool IsRunName(const std::string& name) {
  return name.size() > 17 && std::isdigit(static_cast<unsigned char>(name[0])) && name[8] == 'T' && name[15] == 'Z';
}

bool RunActive(const std::string& dir) {
  const int fd = open((dir + "/lock").c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) return false;
  const bool active = flock(fd, LOCK_EX | LOCK_NB) != 0 && errno == EWOULDBLOCK;
  close(fd);
  return active;
}

bool InspectSegment(const std::string& path, SegmentInfo* info, std::string* error) {
  *info = SegmentInfo();
  info->path = path;
  info->index = static_cast<unsigned>(std::strtoul(BaseName(path).c_str(), nullptr, 10));
  info->bytes = FileBytes(path);
  json head;
  return ReadHead(path, info, &head, error);
}

RunInfo InspectRun(const std::string& dir, bool read_end) {
  RunInfo run;
  run.dir = dir;
  run.name = BaseName(dir);
  RunState state;
  uint64_t heads_read = 0;
  uint64_t ends_read = 0;
  Update(run, state, read_end, heads_read, ends_read);
  return run;
}

std::vector<RunInfo> InspectRuns(const std::string& journal_dir, bool read_end_of_inactive) {
  RunIndex index(journal_dir);
  return index.Refresh(read_end_of_inactive);
}

RunIndex::RunIndex(std::string journal_dir) : journal_dir_(std::move(journal_dir)) {}
RunIndex::~RunIndex() = default;
RunIndex::RunIndex(RunIndex&&) noexcept = default;
RunIndex& RunIndex::operator=(RunIndex&&) noexcept = default;

const std::vector<RunInfo>& RunIndex::Refresh(bool read_end_of_inactive) {
  std::vector<RunInfo> runs;
  std::vector<RunState> states;
  size_t old = 0;
  for (const auto& dir : RunDirs(journal_dir_)) {
    while (old < runs_.size() && runs_[old].dir < dir) ++old;
    RunInfo run;
    RunState state;
    if (old < runs_.size() && runs_[old].dir == dir) {
      run = std::move(runs_[old]);
      state = std::move(states_[old]);
      ++old;
    } else {
      run.dir = dir;
      run.name = BaseName(dir);
    }
    Update(run, state, read_end_of_inactive, heads_read_, ends_read_);
    if (run.segments.empty()) continue;
    runs.push_back(std::move(run));
    states.push_back(std::move(state));
  }
  runs_ = std::move(runs);
  states_ = std::move(states);
  return runs_;
}

uint64_t RunIndex::heads_read() const {
  return heads_read_;
}

uint64_t RunIndex::ends_read() const {
  return ends_read_;
}

}  // namespace mowbite_journal
