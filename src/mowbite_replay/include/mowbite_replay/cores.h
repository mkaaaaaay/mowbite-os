#ifndef MOWBITE_REPLAY_CORES_H
#define MOWBITE_REPLAY_CORES_H

#include <mowbite_journal/replay.h>

#include <string>

namespace mowbite_replay {

// Every core of MowBite OS this tool can replay.
mowbite_journal::ReplayRegistry Cores();

// A copy of one run in out_dir with the places and wall clock of all cores replaced (mowbite_journal::AnonymizeRun).
bool AnonymizeRun(const std::string& run_dir, const std::string& out_dir,
                  const mowbite_journal::AnonymizeOptions& options, std::string* error);

}  // namespace mowbite_replay

#endif  // MOWBITE_REPLAY_CORES_H
