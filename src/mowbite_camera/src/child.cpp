#include "mowbite_camera/child.h"

#include <fcntl.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <thread>

#include "mowbite_camera/error_text.h"

extern char** environ;

namespace mowbite_camera {

namespace {

constexpr int kCannotRun = 127;
// Descriptors closed one by one where the kernel has no close_range.
constexpr int kMaxDescriptors = 65536;
// How often Stop looks whether the child has ended.
constexpr auto kStopStep = std::chrono::milliseconds(20);

std::vector<char*> Pointers(std::vector<std::string>& texts) {
  std::vector<char*> pointers;
  pointers.reserve(texts.size() + 1);
  for (std::string& text : texts) pointers.push_back(text.data());
  pointers.push_back(nullptr);
  return pointers;
}

}  // namespace

Child::~Child() {
  Stop();
}

bool Child::Start(const ChildSpec& spec, std::string* error) {
  if (pid_ > 0) {
    *error = "it runs already";
    return false;
  }
  // Everything the child needs is made before the fork: after it only async-signal-safe calls are allowed, for
  // another thread of this process may hold any lock.
  std::vector<std::string> args = {spec.program};
  args.insert(args.end(), spec.args.begin(), spec.args.end());
  std::vector<std::string> environment;
  for (char** entry = environ; entry && *entry; ++entry) {
    const std::string have = *entry;
    const std::string name = have.substr(0, have.find('=') + 1);
    const bool replaced = std::any_of(spec.environment.begin(), spec.environment.end(),
                                      [&](const std::string& set) { return set.compare(0, name.size(), name) == 0; });
    if (!replaced) environment.push_back(have);
  }
  environment.insert(environment.end(), spec.environment.begin(), spec.environment.end());
  const std::vector<char*> argv = Pointers(args);
  const std::vector<char*> envp = Pointers(environment);
  errno = 0;
  const int own_nice = ::getpriority(PRIO_PROCESS, 0);
  const int nice = errno == 0 ? std::max(own_nice, spec.nice) : spec.nice;
  const std::string oom = std::to_string(spec.oom_score_adj);
  const pid_t parent = ::getpid();

  const pid_t pid = ::fork();
  if (pid < 0) {
    *error = std::string("fork: ") + ErrorText(errno);
    return false;
  }
  if (pid == 0) {
    ::prctl(PR_SET_PDEATHSIG, SIGKILL);
    // The parent may have ended before the line above took effect.
    if (::getppid() != parent) ::_exit(kCannotRun);
    sigset_t none;
    ::sigemptyset(&none);
    ::pthread_sigmask(SIG_SETMASK, &none, nullptr);
    ::signal(SIGINT, SIG_DFL);
    ::signal(SIGTERM, SIG_DFL);
    const int null = ::open("/dev/null", O_RDONLY);
    if (null >= 0) ::dup2(null, STDIN_FILENO);
    if (::close_range(3, ~0U, 0) != 0) {
      for (int fd = 3; fd < kMaxDescriptors; ++fd) ::close(fd);
    }
    if (!spec.directory.empty() && ::chdir(spec.directory.c_str()) != 0) ::_exit(kCannotRun);
    if (spec.nice != 0) ::setpriority(PRIO_PROCESS, 0, nice);
    if (spec.oom_score_adj != 0) {
      const int fd = ::open("/proc/self/oom_score_adj", O_WRONLY);
      if (fd >= 0 && ::write(fd, oom.data(), oom.size()) < 0) {
        // It keeps the value of this process.
      }
      if (fd >= 0) ::close(fd);
    }
    ::execve(argv[0], argv.data(), envp.data());
    ::_exit(kCannotRun);
  }
  pid_ = pid;
  return true;
}

bool Child::Running(std::string* how) {
  if (pid_ <= 0) return false;
  int status = 0;
  const pid_t ended = ::waitpid(pid_, &status, WNOHANG);
  if (ended == 0 || (ended < 0 && errno == EINTR)) return true;
  if (how) {
    *how = ended < 0             ? "nothing known"
           : WIFEXITED(status)   ? "exit code " + std::to_string(WEXITSTATUS(status))
           : WIFSIGNALED(status) ? "signal " + std::to_string(WTERMSIG(status))
                                 : "status " + std::to_string(status);
  }
  pid_ = -1;
  return false;
}

void Child::Stop(double grace_s) {
  if (pid_ <= 0) return;
  ::kill(pid_, SIGINT);
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(grace_s);
  while (Running() && std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(kStopStep);
  }
  if (pid_ <= 0) return;
  ::kill(pid_, SIGKILL);
  while (::waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {
  }
  pid_ = -1;
}

}  // namespace mowbite_camera
