#ifndef MOWBITE_GNSS_FLAGS_H
#define MOWBITE_GNSS_FLAGS_H

#include <cstdint>

namespace mowbite_gnss {

// Flags of a GPS pose as in mowbite_msgs/AbsolutePose.
constexpr uint16_t kFlagRtk = 1;
constexpr uint16_t kFlagRtkFixed = 2;
constexpr uint16_t kFlagRtkFloat = 4;
constexpr uint16_t kFlagDeadReckoning = 8;

}  // namespace mowbite_gnss

#endif  // MOWBITE_GNSS_FLAGS_H
