#ifndef MOWBITE_MSGS_BLADE_HPP
#define MOWBITE_MSGS_BLADE_HPP

#include "mowbite_msgs/msg/hardware_status.hpp"

namespace mowbite_msgs {

// For every gate that needs the blade still: it may turn while it is commanded on or its motor reports any rpm, also
// while it runs down after off. A motor without a report counts by blade_on alone. BLADES and the statistics count
// mowing from blade_running_rpm instead.
inline bool BladeMayTurn(const msg::HardwareStatus& status) {
  return status.blade_on || (status.blade_motor.ok && status.blade_motor.rpm != 0);
}

}  // namespace mowbite_msgs

#endif  // MOWBITE_MSGS_BLADE_HPP
