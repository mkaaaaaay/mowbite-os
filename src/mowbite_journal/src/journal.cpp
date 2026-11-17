#include "mowbite_journal/journal.h"

#include <dirent.h>
#include <elf.h>
#include <fcntl.h>
#include <link.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <zstd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <mutex>
#include <thread>

#include "mowbite_journal/format.h"
#include "mowbite_journal/run_index.h"

namespace mowbite_journal {

int64_t MonotonicNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

namespace {

constexpr size_t kMaxStreams = 64;
constexpr double kLimitsCheckS = 60.0;

double Seconds() {
  return static_cast<double>(MonotonicNs()) * 1e-9;
}

std::string UtcText(std::time_t t, const char* format) {
  std::tm utc{};
  gmtime_r(&t, &utc);
  char text[32];
  std::strftime(text, sizeof(text), format, &utc);
  return text;
}

std::string ReadFirstLine(const char* path) {
  std::ifstream file(path);
  std::string line;
  std::getline(file, line);
  return line;
}

std::string ProcessName() {
  std::string name = ReadFirstLine("/proc/self/comm");
  for (char& c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-') c = '_';
  }
  return name.empty() ? "process" : name;
}

const char* Architecture() {
#if defined(__aarch64__)
  return "arm64";
#elif defined(__x86_64__)
  return "amd64";
#else
  return "other";
#endif
}

struct BuildIdSearch {
  const std::vector<std::string>& libraries;
  json ids = json::object();
};

// Whether a loaded object is the program, a library of mowbite or one of the platform, by a part of its name.
bool Own(const std::string& name, const std::vector<std::string>& libraries) {
  if (name == "program" || name.find("mowbite") != std::string::npos) return true;
  return std::any_of(libraries.begin(), libraries.end(),
                     [&](const std::string& part) { return !part.empty() && name.find(part) != std::string::npos; });
}

// GNU build IDs of the program and of the libraries of MowBite OS it loaded: those of mowbite and of the platform.
json BuildIds(const std::vector<std::string>& libraries) {
  BuildIdSearch search{libraries};
  dl_iterate_phdr(
      [](dl_phdr_info* info, size_t, void* data) {
        auto* found = static_cast<BuildIdSearch*>(data);
        std::string name = info->dlpi_name && *info->dlpi_name ? info->dlpi_name : "program";
        if (!Own(name, found->libraries)) return 0;
        for (int i = 0; i < info->dlpi_phnum; ++i) {
          const auto& ph = info->dlpi_phdr[i];
          if (ph.p_type != PT_NOTE) continue;
          const auto* at = reinterpret_cast<const uint8_t*>(info->dlpi_addr + ph.p_vaddr);
          const auto* end = at + ph.p_memsz;
          while (at + sizeof(ElfW(Nhdr)) <= end) {
            const auto* note = reinterpret_cast<const ElfW(Nhdr)*>(at);
            const uint8_t* desc = at + sizeof(ElfW(Nhdr)) + ((note->n_namesz + 3) & ~3u);
            if (note->n_type == NT_GNU_BUILD_ID && desc + note->n_descsz <= end) {
              found->ids[name] = Hex(desc, note->n_descsz);
            }
            at = desc + ((note->n_descsz + 3) & ~3u);
          }
        }
        return 0;
      },
      &search);
  return search.ids;
}

void SetIdleIoClass() {
  constexpr int kWhoProcess = 1;
  constexpr int kClassIdle = 3;
  constexpr int kClassShift = 13;
  // 0 is the calling thread.
  syscall(SYS_ioprio_set, kWhoProcess, 0, kClassIdle << kClassShift);
}

double ThreadCpuSeconds() {
  timespec ts{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

void PutRecord(std::vector<uint8_t>& out, uint16_t stream, uint16_t method, int64_t t_ns, const uint8_t* data,
               size_t size) {
  const size_t at = out.size();
  out.resize(at + kRecordHeaderSize + size);
  PutRecordHeader(out.data() + at, {stream, method, t_ns, static_cast<uint32_t>(size)});
  if (size > 0) std::memcpy(out.data() + at + kRecordHeaderSize, data, size);
}

std::string EnvOr(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value && *value ? value : fallback;
}

std::string IsoUtc(std::time_t t) {
  return UtcText(t, "%Y-%m-%dT%H:%M:%SZ");
}

void PutCbor(std::vector<uint8_t>& out, uint16_t method, const json& value) {
  const std::vector<uint8_t> cbor = json::to_cbor(value);
  PutRecord(out, kJournalStream, method, MonotonicNs(), cbor.data(), cbor.size());
}

struct FileInfo {
  std::string path;
  uint64_t bytes = 0;
  std::time_t mtime = 0;
};

std::vector<FileInfo> Segments(const std::string& run) {
  std::vector<FileInfo> files;
  DIR* dir = opendir(run.c_str());
  if (!dir) return files;
  while (dirent* entry = readdir(dir)) {
    std::string name = entry->d_name;
    if (name.size() <= std::strlen(kSegmentSuffix) ||
        name.compare(name.size() - std::strlen(kSegmentSuffix), std::string::npos, kSegmentSuffix) != 0) {
      continue;
    }
    struct stat st {};
    std::string path = run + "/" + name;
    if (stat(path.c_str(), &st) != 0) continue;
    files.push_back({path, static_cast<uint64_t>(st.st_size), st.st_mtime});
  }
  closedir(dir);
  std::sort(files.begin(), files.end(), [](const FileInfo& a, const FileInfo& b) { return a.path < b.path; });
  return files;
}

void RemoveRun(const std::string& run) {
  DIR* dir = opendir(run.c_str());
  if (dir) {
    while (dirent* entry = readdir(dir)) {
      std::string name = entry->d_name;
      if (name != "." && name != "..") unlink((run + "/" + name).c_str());
    }
    closedir(dir);
  }
  rmdir(run.c_str());
}

// LOCK_EX within about 50 ms: a check of RunActive holds LOCK_SH only for a moment.
bool LockSoon(int fd) {
  for (int i = 0; i < 5; ++i) {
    if (flock(fd, LOCK_EX | LOCK_NB) == 0) return true;
    if (errno != EWOULDBLOCK) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

}  // namespace

class Journal::Impl {
 public:
  explicit Impl(const Options& options) : options_(options) {}

  bool Start(std::string* error) {
    if (options_.dir.empty()) {
      if (error) *error = "no journal_dir";
      return false;
    }
    mkdir(options_.dir.c_str(), 0755);
    start_ns_ = MonotonicNs();
    const std::time_t start_utc = std::time(nullptr);
    const std::string process = ProcessName();
    run_name_ = UtcText(start_utc, "%Y%m%dT%H%M%SZ") + "_" + process + "_" + std::to_string(getpid());
    run_dir_ = options_.dir + "/" + run_name_;
    if (mkdir(run_dir_.c_str(), 0755) != 0) {
      if (error) *error = "cannot create " + run_dir_ + ": " + std::strerror(errno);
      return false;
    }
    lock_fd_ = open((run_dir_ + "/lock").c_str(), O_WRONLY | O_CREAT | O_CLOEXEC, 0644);
    if (lock_fd_ < 0 || !LockSoon(lock_fd_)) {
      if (error) *error = "cannot lock " + run_dir_ + ": " + std::strerror(errno);
      return false;
    }
    capacity_ = options_.buffer_kb * 1024;
    active_.reset(new uint8_t[capacity_]);
    standby_.reset(new uint8_t[capacity_]);
    head_ = {{"format", kFormatVersion},
             {"run", run_name_},
             {"process", process},
             {"pid", getpid()},
             {"os_revision", EnvOr("MOWBITE_OS_REVISION", "unknown")},
             {"image", EnvOr("MOWBITE_OS_IMAGE", "unknown")},
             {"arch", Architecture()},
             {"compiler", __VERSION__},
             {"platform", options_.platform},
             {"boot_id", ReadFirstLine("/proc/sys/kernel/random/boot_id")},
             {"start_ns", start_ns_},
             {"start_utc", IsoUtc(start_utc)},
             {"build_ids", BuildIds(options_.libraries)}};
    zstd_ = ZSTD_createCCtx();
    ZSTD_CCtx_setParameter(zstd_, ZSTD_c_compressionLevel, 3);
    stats_.run_dir = run_dir_;
    writer_ = std::thread([this] { Run(); });
    return true;
  }

  ~Impl() {
    if (writer_.joinable()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
      }
      cv_.notify_all();
      writer_.join();
    }
    if (zstd_) ZSTD_freeCCtx(zstd_);
    if (lock_fd_ >= 0) close(lock_fd_);
  }

  uint16_t Define(json definition) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (next_stream_ >= kMaxStreams) return kMaxStreams;
    const uint16_t stream = next_stream_++;
    definition["stream"] = stream;
    names_[stream] = definition.value("name", "");
    definitions_.push_back(json::to_cbor(definition));
    const auto& cbor = definitions_.back();
    AppendLocked(kJournalStream, kDefine, MonotonicNs(), cbor.data(), cbor.size());
    return stream;
  }

  void Append(uint16_t stream, uint16_t method, const uint8_t* data, size_t size) {
    const int64_t t_ns = MonotonicNs();
    std::lock_guard<std::mutex> lock(mutex_);
    AppendLocked(stream, method, t_ns, data, size);
  }

  Stats GetStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
  }

  const Options& options() const {
    return options_;
  }

 private:
  void AppendLocked(uint16_t stream, uint16_t method, int64_t t_ns, const uint8_t* data, size_t size) {
    if (stream >= kMaxStreams) return;
    const size_t need = kRecordHeaderSize + size;
    // After the first record that did not fit, the rest until the swap goes too: the gap record stands where they were.
    if (full_ || active_size_ + need > capacity_) {
      full_ = true;
      ++dropped_[stream];
      return;
    }
    uint8_t* p = active_.get() + active_size_;
    PutRecordHeader(p, {stream, method, t_ns, static_cast<uint32_t>(size)});
    if (size > 0) std::memcpy(p + kRecordHeaderSize, data, size);
    active_size_ += need;
    ++records_[stream];
    bytes_[stream] += need;
  }

  void Run() {
    SetIdleIoClass();
    double last_flush = Seconds();
    double last_space = -options_.check_s;
    double last_limits = -kLimitsCheckS;
    for (;;) {
      bool stop;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(50), [this] { return stop_; });
        stop = stop_;
      }
      const double now = Seconds();
      if (now - last_space >= options_.check_s) {
        last_space = now;
        CheckSpace();
      }
      bool half_full;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        half_full = active_size_ > capacity_ / 2;
      }
      if (stop || half_full || now - last_flush >= options_.flush_s) {
        last_flush = now;
        Flush();
      }
      if (stop) break;
      if (fd_ >= 0 && (now - segment_start_s_ >= options_.segment_s ||
                       static_cast<double>(segment_bytes_) >= options_.segment_mb * 1e6)) {
        EndSegment(kEndRotate);
        last_limits = -kLimitsCheckS;
      }
      if (now - last_limits >= kLimitsCheckS) {
        last_limits = now;
        KeepLimits();
      }
    }
    EndSegment(kEndExit);
  }

  void CheckSpace() {
    struct statvfs fs {};
    if (statvfs(options_.dir.c_str(), &fs) != 0) return;
    const double free_mb = static_cast<double>(fs.f_bavail) * static_cast<double>(fs.f_frsize) / 1e6;
    const bool was_low = low_space_;
    low_space_ = free_mb < options_.reserve_mb;
    if (low_space_) {
      SetProblem("less than " + std::to_string(static_cast<int>(options_.reserve_mb)) + " MB free on " +
                 options_.dir);
    } else if (was_low) {
      SetProblem("");
    }
  }

  void SetProblem(const std::string& problem) {
    problem_ = problem;
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.problem = problem;
  }

  // A segment that could not be created or packed: the next one is tried check_s from now.
  void Fail(const std::string& problem) {
    SetProblem(problem);
    retry_s_ = Seconds() + options_.check_s;
  }

  // Swaps the buffers and writes the full one as a block, with the head and the definitions first in a new segment and
  // a gap record after the records when some were dropped.
  void Flush() {
    // The problem stays in the stats until the new segment is open.
    if (!problem_.empty() && !low_space_ && Seconds() >= retry_s_) problem_.clear();
    size_t size;
    std::array<uint64_t, kMaxStreams> dropped{};
    std::vector<std::vector<uint8_t>> definitions;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::swap(active_, standby_);
      size = active_size_;
      active_size_ = 0;
      full_ = false;
      std::swap(dropped, dropped_);
      if (fd_ < 0 && problem_.empty()) definitions = definitions_;
      UpdateStreamStats(dropped);
    }
    const uint8_t* data = standby_.get();
    if (!problem_.empty()) {
      Lose(data, size, dropped, problem_);
      return;
    }
    prefix_.clear();
    suffix_.clear();
    if (fd_ < 0) {
      if (!OpenSegment()) {
        Lose(data, size, dropped, problem_);
        return;
      }
      for (const auto& definition : definitions) {
        PutRecord(prefix_, kJournalStream, kDefine, MonotonicNs(), definition.data(), definition.size());
      }
    }
    if (HasAny(pending_gap_)) {
      PutCbor(prefix_, kGap, GapJson(pending_gap_, pending_why_));
      pending_gap_.fill(0);
    }
    if (HasAny(dropped)) PutCbor(suffix_, kGap, GapJson(dropped, "buffer full"));
    if (prefix_.empty() && size == 0 && suffix_.empty()) return;
    WriteBlock({{prefix_.data(), prefix_.size()}, {data, size}, {suffix_.data(), suffix_.size()}});
  }

  static bool HasAny(const std::array<uint64_t, kMaxStreams>& counts) {
    return std::any_of(counts.begin(), counts.end(), [](uint64_t n) { return n > 0; });
  }

  static void CountRecords(const uint8_t* data, size_t size, std::array<uint64_t, kMaxStreams>& counts) {
    size_t at = 0;
    while (at + kRecordHeaderSize <= size) {
      const RecordHeader h = GetRecordHeader(data + at);
      if (h.stream < kMaxStreams) ++counts[h.stream];
      at += kRecordHeaderSize + h.size;
    }
  }

  // Records that are not written: counted by stream for the gap record of the next block.
  void Lose(const uint8_t* data, size_t size, const std::array<uint64_t, kMaxStreams>& dropped,
            const std::string& why) {
    std::array<uint64_t, kMaxStreams> counts = dropped;
    CountRecords(data, size, counts);
    for (size_t i = 0; i < kMaxStreams; ++i) pending_gap_[i] += counts[i];
    pending_why_ = why;
  }

  json GapJson(const std::array<uint64_t, kMaxStreams>& counts, const std::string& why) const {
    json dropped = json::array();
    for (size_t i = 0; i < kMaxStreams; ++i) {
      if (counts[i] > 0) dropped.push_back({i, counts[i]});
    }
    return {{"why", why}, {"dropped", dropped}};
  }

  // Called with mutex_ held.
  void UpdateStreamStats(const std::array<uint64_t, kMaxStreams>& dropped) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.streams.clear();
    for (uint16_t i = 0; i < next_stream_; ++i) {
      total_dropped_[i] += dropped[i];
      stats_.streams.push_back({i, i == 0 ? "journal" : names_[i], records_[i], bytes_[i], total_dropped_[i]});
    }
    stats_.dropped = 0;
    for (uint64_t n : total_dropped_) stats_.dropped += n;
  }

  struct Part {
    const uint8_t* data;
    size_t size;
  };

  void WriteBlock(const std::vector<Part>& parts) {
    size_t raw = 0;
    for (const auto& part : parts) raw += part.size;
    block_.resize(kBlockHeaderSize + ZSTD_compressBound(raw));
    ZSTD_outBuffer out{block_.data() + kBlockHeaderSize, block_.size() - kBlockHeaderSize, 0};
    ZSTD_CCtx_reset(zstd_, ZSTD_reset_session_only);
    ZSTD_CCtx_setPledgedSrcSize(zstd_, raw);
    for (size_t i = 0; i < parts.size(); ++i) {
      ZSTD_inBuffer in{parts[i].data, parts[i].size, 0};
      const ZSTD_EndDirective mode = i + 1 == parts.size() ? ZSTD_e_end : ZSTD_e_continue;
      size_t left;
      do {
        left = ZSTD_compressStream2(zstd_, &out, &in, mode);
        if (ZSTD_isError(left)) {
          Fail(std::string("zstd: ") + ZSTD_getErrorName(left));
          CloseLosing(parts, problem_);
          return;
        }
      } while (mode == ZSTD_e_end ? left != 0 : in.pos < in.size);
    }
    const uint32_t raw32 = static_cast<uint32_t>(raw);
    const uint32_t packed32 = static_cast<uint32_t>(out.pos);
    std::memcpy(block_.data(), &raw32, 4);
    std::memcpy(block_.data() + 4, &packed32, 4);
    const size_t total = kBlockHeaderSize + out.pos;
    const double start = Seconds();
    const bool ok = WriteAll(block_.data(), total);
    const double took = Seconds() - start;
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.longest_write_s = std::max(stats_.longest_write_s, took);
    stats_.writer_cpu_s = ThreadCpuSeconds();
    stats_.seconds = Seconds() - static_cast<double>(start_ns_) * 1e-9;
    if (!ok) {
      // The segment may end in a broken block now: it is closed, the next flush starts a new one with a gap.
      stats_.problem = std::string("write: ") + std::strerror(errno);
      CloseLosing(parts, stats_.problem);
      return;
    }
    stats_.raw_bytes += raw;
    stats_.packed_bytes += total;
    segment_bytes_ += total;
  }

  // Closes the segment after a block that was not written; its records go into the gap of the next segment.
  void CloseLosing(const std::vector<Part>& parts, const std::string& why) {
    close(fd_);
    fd_ = -1;
    for (const auto& part : parts) Lose(part.data, part.size, {}, why);
  }

  bool WriteAll(const uint8_t* data, size_t size) {
    while (size > 0) {
      const ssize_t n = write(fd_, data, size);
      if (n < 0) {
        if (errno == EINTR) continue;
        return false;
      }
      data += n;
      size -= static_cast<size_t>(n);
    }
    return true;
  }

  bool OpenSegment() {
    char name[16];
    std::snprintf(name, sizeof(name), "%06u", segment_index_);
    const std::string path = run_dir_ + "/" + name + kSegmentSuffix;
    fd_ = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
    if (fd_ < 0) {
      Fail("cannot create " + path + ": " + std::strerror(errno));
      return false;
    }
    uint8_t header[kFileHeaderSize];
    std::memcpy(header, kMagic, 4);
    std::memcpy(header + 4, &kFormatVersion, 4);
    if (!WriteAll(header, sizeof(header))) {
      Fail("cannot write " + path + ": " + std::strerror(errno));
      close(fd_);
      fd_ = -1;
      // Removed, so that the next try can create it again.
      unlink(path.c_str());
      return false;
    }
    segment_start_s_ = Seconds();
    segment_bytes_ = kFileHeaderSize;
    json head = head_;
    head["segment"] = segment_index_;
    head["segment_start_ns"] = MonotonicNs();
    head["segment_start_utc"] = IsoUtc(std::time(nullptr));
    PutCbor(prefix_, kHead, head);
    ++segment_index_;
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.segments = segment_index_;
    stats_.problem.clear();
    return true;
  }

  // Closes the segment with the statistics and an end record, synced to the disk.
  void EndSegment(const std::string& reason) {
    if (fd_ < 0) return;
    std::vector<uint8_t> tail;
    PutCbor(tail, kStats, StatsJson());
    PutCbor(tail, kEnd, {{"reason", reason}});
    WriteBlock({{tail.data(), tail.size()}});
    if (fd_ < 0) return;
    const double start = Seconds();
    fsync(fd_);
    close(fd_);
    fd_ = -1;
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.longest_write_s = std::max(stats_.longest_write_s, Seconds() - start);
  }

  json StatsJson() const {
    const Stats stats = GetStats();
    json streams = json::array();
    for (const auto& s : stats.streams) {
      streams.push_back({{"stream", s.stream}, {"name", s.name}, {"records", s.records}, {"bytes", s.bytes},
                         {"dropped", s.dropped}});
    }
    return {{"seconds", stats.seconds},         {"raw_bytes", stats.raw_bytes},
            {"packed_bytes", stats.packed_bytes}, {"longest_write_s", stats.longest_write_s},
            {"writer_cpu_s", stats.writer_cpu_s}, {"dropped", stats.dropped},
            {"streams", streams}};
  }

  // Whole old runs first, then the oldest closed segments of this run; the age limit the same way.
  void KeepLimits() {
    std::vector<std::string> runs;
    DIR* dir = opendir(options_.dir.c_str());
    if (!dir) return;
    while (dirent* entry = readdir(dir)) {
      std::string name = entry->d_name;
      if (IsRunName(name)) runs.push_back(name);
    }
    closedir(dir);
    std::sort(runs.begin(), runs.end());
    const std::time_t now = std::time(nullptr);
    const double max_age_s = options_.max_days * 86400.0;
    const double max_bytes = options_.max_mb * 1e6;
    struct Run {
      std::string path;
      bool own;
      bool active;
      std::vector<FileInfo> segments;
      uint64_t bytes = 0;
      std::time_t newest = 0;
    };
    std::vector<Run> found;
    double total = 0;
    for (const auto& name : runs) {
      Run run{options_.dir + "/" + name, name == run_name_, false, Segments(options_.dir + "/" + name)};
      run.active = run.own || RunActive(run.path);
      for (const auto& file : run.segments) {
        run.bytes += file.bytes;
        run.newest = std::max(run.newest, file.mtime);
      }
      if (!run.active && !run.segments.empty() && static_cast<double>(now - run.newest) > max_age_s) {
        RemoveRun(run.path);
        continue;
      }
      total += static_cast<double>(run.bytes);
      found.push_back(std::move(run));
    }
    for (auto& run : found) {
      if (total <= max_bytes) break;
      if (run.active) continue;
      RemoveRun(run.path);
      total -= static_cast<double>(run.bytes);
    }
    for (auto& run : found) {
      if (!run.own) continue;
      size_t removed = 0;
      // The open segment is the last one; it stays.
      for (size_t i = 0; i + 1 < run.segments.size(); ++i) {
        const auto& file = run.segments[i];
        if (total <= max_bytes && static_cast<double>(now - file.mtime) <= max_age_s) break;
        if (unlink(file.path.c_str()) == 0) {
          total -= static_cast<double>(file.bytes);
          ++removed;
        }
      }
      if (removed > 0) {
        std::ofstream cut(run.path + "/cut", std::ios::trunc);
        cut << "segments before " << run.segments[removed].path.substr(run.path.size() + 1)
            << " removed for the limits: this run cannot be replayed from its start\n";
      }
    }
  }

  const Options options_;
  std::string run_name_;
  std::string run_dir_;
  json head_;
  int64_t start_ns_ = 0;
  int lock_fd_ = -1;

  // Guards the buffers, the counters and the definitions; held only for copies.
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool stop_ = false;
  std::unique_ptr<uint8_t[]> active_;
  std::unique_ptr<uint8_t[]> standby_;
  size_t capacity_ = 0;
  size_t active_size_ = 0;
  bool full_ = false;
  uint16_t next_stream_ = 1;
  std::array<std::string, kMaxStreams> names_;
  std::array<uint64_t, kMaxStreams> records_{};
  std::array<uint64_t, kMaxStreams> bytes_{};
  std::array<uint64_t, kMaxStreams> dropped_{};
  std::vector<std::vector<uint8_t>> definitions_;

  // Only the writer thread.
  std::thread writer_;
  ZSTD_CCtx* zstd_ = nullptr;
  int fd_ = -1;
  unsigned segment_index_ = 0;
  double segment_start_s_ = 0;
  uint64_t segment_bytes_ = 0;
  std::vector<uint8_t> block_;
  std::vector<uint8_t> prefix_;
  std::vector<uint8_t> suffix_;
  std::string problem_;
  bool low_space_ = false;
  double retry_s_ = 0;
  std::array<uint64_t, kMaxStreams> pending_gap_{};
  std::string pending_why_;
  std::array<uint64_t, kMaxStreams> total_dropped_{};

  mutable std::mutex stats_mutex_;
  Stats stats_;
};

Journal::Journal(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Journal::~Journal() = default;

std::unique_ptr<Journal> Journal::Create(const Options& options, std::string* error) {
  auto impl = std::make_unique<Impl>(options);
  if (!impl->Start(error)) return nullptr;
  return std::unique_ptr<Journal>(new Journal(std::move(impl)));
}

Journal* Journal::Open(const Options& options, std::string* error) {
  static std::mutex mutex;
  static std::unique_ptr<Journal> journal;
  std::lock_guard<std::mutex> lock(mutex);
  if (!journal && !options.dir.empty()) journal = Create(options, error);
  return journal.get();
}

uint16_t Journal::Define(json definition) {
  return impl_->Define(std::move(definition));
}

void Journal::Append(uint16_t stream, uint16_t method, const uint8_t* data, size_t size) {
  impl_->Append(stream, method, data, size);
}

Stats Journal::GetStats() const {
  return impl_->GetStats();
}

const Options& Journal::options() const {
  return impl_->options();
}

}  // namespace mowbite_journal
