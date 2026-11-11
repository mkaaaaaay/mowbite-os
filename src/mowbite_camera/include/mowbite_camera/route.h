#ifndef MOWBITE_CAMERA_ROUTE_H
#define MOWBITE_CAMERA_ROUTE_H

#include <string>

namespace mowbite_camera {

// The IPv4 address this host sends from on its default route, empty without one. Nothing is sent to find it.
std::string DefaultRouteAddress();

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_ROUTE_H
