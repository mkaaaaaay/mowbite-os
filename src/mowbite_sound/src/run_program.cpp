#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

#include "mowbite_sound/convert.h"

extern char** environ;

namespace mowbite_sound {

namespace {

constexpr size_t kKeepOutput = 512;

double Now() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// The last line of what the program wrote.
std::string LastLine(std::string text) {
  while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) text.pop_back();
  const size_t newline = text.rfind('\n');
  return newline == std::string::npos ? text : text.substr(newline + 1);
}

// Reads what is there into tail, keeping its end.
void Drain(int fd, std::string* tail) {
  char buffer[256];
  ssize_t n;
  while ((n = ::read(fd, buffer, sizeof buffer)) > 0) {
    tail->append(buffer, static_cast<size_t>(n));
    if (tail->size() > kKeepOutput) tail->erase(0, tail->size() - kKeepOutput);
  }
}

}  // namespace

bool RunProgram(const std::vector<std::string>& argv, double timeout, std::string* error) {
  int pipe_fds[2];
  if (::pipe2(pipe_fds, O_CLOEXEC | O_NONBLOCK) != 0) {
    *error = std::string("pipe: ") + std::strerror(errno);
    return false;
  }
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  posix_spawn_file_actions_adddup2(&actions, pipe_fds[1], STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions, pipe_fds[1], STDERR_FILENO);
  // The threads of the node may block signals; the program starts with none blocked and default handlers.
  posix_spawnattr_t attr;
  posix_spawnattr_init(&attr);
  sigset_t none;
  sigemptyset(&none);
  sigset_t defaults;
  sigemptyset(&defaults);
  for (const int number : {SIGPIPE, SIGTERM, SIGINT, SIGXCPU, SIGXFSZ}) sigaddset(&defaults, number);
  posix_spawnattr_setsigmask(&attr, &none);
  posix_spawnattr_setsigdefault(&attr, &defaults);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF);

  std::vector<char*> args;
  for (const auto& arg : argv) args.push_back(const_cast<char*>(arg.c_str()));
  args.push_back(nullptr);
  pid_t pid = 0;
  const int spawned = posix_spawn(&pid, args[0], &actions, &attr, args.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  posix_spawnattr_destroy(&attr);
  ::close(pipe_fds[1]);
  if (spawned != 0) {
    ::close(pipe_fds[0]);
    *error = "cannot start " + argv[0] + ": " + std::strerror(spawned);
    return false;
  }

  const double deadline = Now() + timeout;
  std::string output;
  int status = 0;
  bool killed = false;
  while (true) {
    pollfd poll_fd{pipe_fds[0], POLLIN, 0};
    ::poll(&poll_fd, 1, 100);
    Drain(pipe_fds[0], &output);
    const pid_t done = ::waitpid(pid, &status, WNOHANG);
    if (done == pid) break;
    if (done < 0 && errno != EINTR) {
      *error = std::string("waitpid: ") + std::strerror(errno);
      ::close(pipe_fds[0]);
      return false;
    }
    if (Now() > deadline) {
      ::kill(pid, SIGKILL);
      while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
      }
      killed = true;
      break;
    }
  }
  Drain(pipe_fds[0], &output);
  ::close(pipe_fds[0]);

  if (!killed && WIFEXITED(status) && WEXITSTATUS(status) == 0) return true;
  if (killed) {
    *error = argv[0] + " took longer than " + std::to_string(static_cast<int>(timeout)) + " s";
  } else if (WIFSIGNALED(status)) {
    *error = argv[0] + " ended by signal " + std::to_string(WTERMSIG(status));
  } else {
    *error = argv[0] + " exited with " + std::to_string(WEXITSTATUS(status));
  }
  const std::string line = LastLine(output);
  if (!line.empty()) *error += ": " + line;
  return false;
}

}  // namespace mowbite_sound
