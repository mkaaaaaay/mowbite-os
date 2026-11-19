#ifndef MOWBITE_CAMERA_CHILD_H
#define MOWBITE_CAMERA_CHILD_H

#include <sys/types.h>

#include <string>
#include <vector>

namespace mowbite_camera {

struct ChildSpec {
  std::string program;
  // Its arguments after the program name.
  std::vector<std::string> args;
  // NAME=value, each instead of what this process has under that name.
  std::vector<std::string> environment;
  // Its working directory; empty: the one of this process.
  std::string directory;
  // Its nice value unless this process is nicer already, and its oom_score_adj; 0: as this process.
  int nice = 0;
  int oom_score_adj = 0;
};

// One child process with stdin from /dev/null and no other descriptor of this process. The kernel ends it with
// SIGKILL when the thread that started it ends, so it never outlives this process: start it from a thread that
// lives as long as the child shall.
class Child {
 public:
  Child() = default;
  ~Child();
  Child(const Child&) = delete;
  Child& operator=(const Child&) = delete;

  // False with *error when there is one already or the process cannot be made. A program that cannot be run ends
  // with exit code 127.
  bool Start(const ChildSpec& spec, std::string* error);
  // Whether it runs. The call that finds it ended tells in *how what it ended with ("exit code 1", "signal 9").
  bool Running(std::string* how = nullptr);
  // SIGINT, and SIGKILL when it has not ended grace_s later; returns when it is gone.
  void Stop(double grace_s = 2.0);
  // SIGINT without waiting: Running() sends SIGKILL once it has not ended grace_s later.
  void Interrupt(double grace_s = 2.0);

 private:
  pid_t pid_ = -1;
  // Since Interrupt: when SIGKILL follows (steady s), negative without one.
  double kill_at_ = -1.0;
};

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_CHILD_H
