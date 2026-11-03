#ifndef MOWBITE_GNSS_SOLUTION_H
#define MOWBITE_GNSS_SOLUTION_H

#include <cstdint>

namespace mowbite_gnss {

// Kind of a solution, numbered as in mowbite_msgs/GnssFix.
enum class Solution : uint8_t { kNone = 0, kSingle = 1, kDgps = 2, kRtkFloat = 3, kRtkFixed = 4, kDeadReckoning = 5 };

}  // namespace mowbite_gnss

#endif  // MOWBITE_GNSS_SOLUTION_H
