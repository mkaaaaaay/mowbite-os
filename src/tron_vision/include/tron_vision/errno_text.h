#ifndef TRON_VISION_ERRNO_TEXT_H
#define TRON_VISION_ERRNO_TEXT_H

#include <cerrno>
#include <cstring>
#include <string>

namespace tron_vision {

// `what` with the text of the current errno ("open /dev/video22: Device or resource busy").
inline std::string ErrnoText(const std::string& what) {
  char buf[128];
  return what + ": " + strerror_r(errno, buf, sizeof(buf));
}

}  // namespace tron_vision

#endif  // TRON_VISION_ERRNO_TEXT_H
