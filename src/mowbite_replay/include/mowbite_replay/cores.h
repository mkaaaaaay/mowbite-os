#ifndef MOWBITE_REPLAY_CORES_H
#define MOWBITE_REPLAY_CORES_H

#include <mowbite_journal/replay.h>

namespace mowbite_replay {

// Every core of MowBite OS this tool can replay.
mowbite_journal::ReplayRegistry Cores();

}  // namespace mowbite_replay

#endif  // MOWBITE_REPLAY_CORES_H
