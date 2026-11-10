#include "mowbite_watch/watch.h"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace mowbite_watch {
namespace {

int64_t Now() {
  timespec t{};
  clock_gettime(CLOCK_MONOTONIC, &t);
  return int64_t{t.tv_sec} * 1000000000 + t.tv_nsec;
}

int64_t Ns(double seconds) {
  return static_cast<int64_t>(seconds * 1e9);
}

// Text into a fixed buffer, cut at its end; nothing allocates, for the path that ends the process.
class Text {
 public:
  Text(char* buf, size_t size) : buf_(buf), size_(size) {}

  Text& Add(const char* s) {
    while (*s) Put(*s++);
    return *this;
  }
  // A node name or a mark as given; anything outside [A-Za-z0-9_/.-] as '?', so the note stays JSON.
  Text& Name(const char* s) {
    for (; *s; ++s) {
      const char c = *s;
      const bool plain = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
                         c == '/' || c == '.' || c == '-';
      Put(plain ? c : '?');
    }
    return *this;
  }
  // Printable ASCII as given, anything else as '?'.
  Text& Plain(const char* s) {
    for (; *s; ++s) Put(*s >= ' ' && *s <= '~' ? *s : '?');
    return *this;
  }
  // ns as seconds with one decimal.
  Text& Seconds(int64_t ns) {
    const int64_t tenths = (ns < 0 ? 0 : ns + 50000000) / 100000000;
    char digits[24];
    int n = 0;
    int64_t whole = tenths / 10;
    do {
      digits[n++] = static_cast<char>('0' + whole % 10);
      whole /= 10;
    } while (whole > 0);
    while (n > 0) Put(digits[--n]);
    Put('.');
    Put(static_cast<char>('0' + tenths % 10));
    return *this;
  }
  size_t size() const { return len_; }

 private:
  void Put(char c) {
    if (len_ + 1 < size_) buf_[len_++] = c;
    if (size_ > 0) buf_[len_] = '\0';
  }

  char* buf_;
  size_t size_;
  size_t len_ = 0;
};

void WriteAll(int fd, const char* data, size_t size) {
  while (size > 0) {
    const ssize_t n = write(fd, data, size);
    if (n < 0 && errno == EINTR) continue;
    if (n <= 0) return;
    data += n;
    size -= static_cast<size_t>(n);
  }
}

// The note next to its path and renamed over it; a disk that hangs here meets the kernel timer.
void WriteNote(const std::string& path, const Verdict& verdict) {
  char tmp[4096];
  if (path.size() + 5 > sizeof(tmp)) return;
  std::memcpy(tmp, path.c_str(), path.size());
  std::memcpy(tmp + path.size(), ".tmp", 5);
  const int fd = open(tmp, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) return;
  char text[256];
  WriteAll(fd, text, NoteText(verdict, text, sizeof(text)));
  close(fd);
  rename(tmp, path.c_str());
}

}  // namespace

Options OptionsFromEnvironment(std::string* warning) {
  Options options;
  warning->clear();
  if (const char* nodes = std::getenv(kNodesEnv)) {
    std::string list = nodes;
    size_t start = 0;
    while (start <= list.size()) {
      const size_t end = std::min(list.find(',', start), list.size());
      if (end > start) options.expected.push_back(list.substr(start, end - start));
      start = end + 1;
    }
  }
  if (const char* note = std::getenv(kNoteEnv)) options.note = note;
  if (const char* factor = std::getenv(kFactorEnv)) {
    char* end = nullptr;
    const double value = std::strtod(factor, &end);
    if (end != factor && *end == '\0' && value >= 1.0 && value <= 100.0) {
      options.factor = value;
      if (value > 1.0) *warning = std::string("all limits times ") + factor + " (" + kFactorEnv + ", sanitizer runs)";
    } else {
      *warning = std::string(kFactorEnv) + "=" + factor + " is not a factor from 1 to 100: limits as they are";
    }
  }
  return options;
}

size_t NoteText(const Verdict& verdict, char* buf, size_t size) {
  Text text(buf, size);
  text.Add("{\"node\":\"").Name(verdict.node).Add("\",\"seconds\":").Seconds(verdict.seconds_ns);
  text.Add(",\"limit\":").Seconds(verdict.limit_ns).Add("}\n");
  return text.size();
}

size_t EndLine(const Verdict& verdict, char* buf, size_t size) {
  Text text(buf, size);
  text.Add("mowbite_watch: ");
  switch (verdict.finding) {
    case Finding::kSilent:
      text.Name(verdict.node).Add(" silent for ").Seconds(verdict.seconds_ns).Add(" s (limit ");
      text.Seconds(verdict.limit_ns).Add(" s)");
      if (verdict.mark) text.Add(", last '").Plain(verdict.mark).Add("'");
      break;
    case Finding::kMissing:
      text.Name(verdict.node).Add(" did not start within ").Seconds(verdict.limit_ns).Add(" s");
      break;
    case Finding::kShutdown:
      text.Add("still running ").Seconds(verdict.seconds_ns).Add(" s after the shutdown (limit ");
      text.Seconds(verdict.limit_ns).Add(" s)");
      break;
    case Finding::kNone:
      text.Add("no finding");
      break;
  }
  text.Add(": ending the process\n");
  return text.size();
}

Runtime::Runtime(Options options)
    : options_(std::move(options)),
      core_(options_.expected, Now(), Ns(options_.start_limit * options_.factor),
            Ns(options_.shutdown_limit * options_.factor)) {
  sigevent event{};
  event.sigev_notify = SIGEV_SIGNAL;
  event.sigev_signo = SIGKILL;
  if (timer_create(CLOCK_MONOTONIC, &event, &timer_) != 0) {
    char line[160];
    Text text(line, sizeof(line));
    text.Add("mowbite_watch: no kernel timer (").Add(std::strerror(errno)).Add("): ending the process\n");
    WriteAll(STDERR_FILENO, line, text.size());
    _exit(kExitCode);
  }
  Arm(options_.deadman * options_.factor);
  std::thread([this] { Run(); }).detach();
}

Runtime& Runtime::Process() {
  static Runtime* const runtime = [] {
    std::string warning;
    Options options = OptionsFromEnvironment(&warning);
    if (!warning.empty()) std::fprintf(stderr, "mowbite_watch: %s\n", warning.c_str());
    return new Runtime(std::move(options));
  }();
  return *runtime;
}

int Runtime::Enter(std::string_view name, double limit_s) {
  std::lock_guard<std::mutex> lock(enter_mutex_);
  const int place = core_.Enter(name, Ns(limit_s * options_.factor), Now());
  if (place < 0) throw std::length_error("all places of the watch are taken, " + std::string(name) + " has none");
  return place;
}

void Runtime::Sign(int place) {
  core_.Sign(place, Now());
}

void Runtime::Mark(int place, const char* where) {
  core_.SetMark(place, where);
}

void Runtime::Leave(int place) {
  core_.Leave(place);
}

void Runtime::Shutdown() {
  core_.Shutdown(Now());
}

void Runtime::Arm(double seconds) const {
  itimerspec spec{};
  spec.it_value.tv_sec = static_cast<time_t>(seconds);
  spec.it_value.tv_nsec = static_cast<long>((seconds - static_cast<double>(spec.it_value.tv_sec)) * 1e9);
  timer_settime(timer_, 0, &spec, nullptr);
}

void Runtime::Run() {
  const int64_t period = Ns(options_.period);
  const timespec pause{static_cast<time_t>(period / 1000000000), static_cast<long>(period % 1000000000)};
  for (;;) {
    Arm(options_.deadman * options_.factor);
    if (options_.before_judge) options_.before_judge();
    const Verdict verdict = core_.Judge(Now());
    if (verdict.finding != Finding::kNone) End(verdict);
    clock_nanosleep(CLOCK_MONOTONIC, 0, &pause, nullptr);
  }
}

void Runtime::End(const Verdict& verdict) const {
  // Line and note may hang on a full pipe or a stalled disk; the kernel timer ends the process all the same.
  Arm(options_.end_deadman * options_.factor);
  char line[512];
  WriteAll(STDERR_FILENO, line, EndLine(verdict, line, sizeof(line)));
  if (!options_.note.empty() && verdict.finding != Finding::kShutdown) WriteNote(options_.note, verdict);
  _exit(kExitCode);
}

Watch::Watch(std::string_view name, double limit_s, Runtime& runtime)
    : runtime_(runtime), place_(runtime.Enter(name, limit_s)) {}

Watch::~Watch() {
  runtime_.Leave(place_);
}

void Watch::Sign() {
  runtime_.Sign(place_);
}

void Watch::Mark(const char* where) {
  runtime_.Mark(place_, where);
}

}  // namespace mowbite_watch
