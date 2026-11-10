#ifndef MOWBITE_WATCH_WATCH_H
#define MOWBITE_WATCH_WATCH_H

#include <ctime>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "mowbite_watch/watch_core.h"

// The watch of a process (G39): every node signs from the thread that runs its callbacks; one that stays silent
// beyond its limit, or a node of the expected list that never comes, ends the process. Then the launch starts the
// container again and the restart hold judges as after a crash. A kernel timer with SIGKILL ends the process should
// the watch thread itself stall.
namespace mowbite_watch {

// Limits in s. A node's limit is its own (Attach), kLimit by default; none of them is a parameter.
constexpr double kLimit = 10.0;
constexpr double kStartLimit = 60.0;
constexpr double kShutdownLimit = 20.0;
// How often nodes sign and the watch judges.
constexpr double kPeriod = 0.5;
// The kernel timer over the watch thread, set again at every round, and once the verdict is taken.
constexpr double kDeadman = 5.0;
constexpr double kEndDeadman = 2.0;
// Exit code of an end by the watch.
constexpr int kExitCode = 86;

// The environment of the container (stack.py): the expected nodes ("a,b,c"), the path of the note written before the
// end, and a factor >= 1 on all limits for runs with a sanitizer.
constexpr char kNodesEnv[] = "MOWBITE_WATCH_NODES";
constexpr char kNoteEnv[] = "MOWBITE_WATCH_NOTE";
constexpr char kFactorEnv[] = "MOWBITE_WATCH_FACTOR";

struct Options {
  std::vector<std::string> expected;
  // Empty: no note.
  std::string note;
  double factor = 1.0;
  double start_limit = kStartLimit;
  double shutdown_limit = kShutdownLimit;
  double period = kPeriod;
  double deadman = kDeadman;
  double end_deadman = kEndDeadman;
  // Tests only: runs in the watch thread before each verdict.
  void (*before_judge)() = nullptr;
};

// The options from the environment; what was wrong with it (the factor is then 1) in *warning, else empty.
Options OptionsFromEnvironment(std::string* warning);

// {"node":..,"seconds":..,"limit":..} of a verdict, as the note holds it; written into buf without allocating, cut at
// size. Returns the length.
size_t NoteText(const Verdict& verdict, char* buf, size_t size);
// The line on stderr for a verdict, with its line end.
size_t EndLine(const Verdict& verdict, char* buf, size_t size);

// The watch thread with its kernel timer; it is never destroyed, the thread runs until the process ends.
class Runtime {
 public:
  // Ends the process when the kernel timer cannot be made: a watch without it would not be one.
  explicit Runtime(Options options);
  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  // The one of this process, with the options from the environment, made at the first call.
  static Runtime& Process();

  // The place of a node with its limit (s, times the factor); throws when all places are taken.
  int Enter(std::string_view name, double limit_s);
  void Sign(int place);
  void Mark(int place, const char* where);
  void Leave(int place);
  // At the shutdown of ROS: from here on only the shutdown limit.
  void Shutdown();
  double factor() const { return options_.factor; }

 private:
  ~Runtime() = default;
  void Run();
  void Arm(double seconds) const;
  [[noreturn]] void End(const Verdict& verdict) const;

  const Options options_;
  WatchCore core_;
  // Enter one node at a time.
  std::mutex enter_mutex_;
  timer_t timer_{};
};

// A node's place in the watch of its process: enters on construction, leaves on destruction.
class Watch {
 public:
  Watch(std::string_view name, double limit_s, Runtime& runtime = Runtime::Process());
  ~Watch();
  Watch(const Watch&) = delete;
  Watch& operator=(const Watch&) = delete;

  // From the thread that runs the node's callbacks.
  void Sign();
  // What the node does now, named in the line should it stall; where must outlive the watch.
  void Mark(const char* where);

 private:
  Runtime& runtime_;
  const int place_;
};

}  // namespace mowbite_watch

#endif  // MOWBITE_WATCH_WATCH_H
