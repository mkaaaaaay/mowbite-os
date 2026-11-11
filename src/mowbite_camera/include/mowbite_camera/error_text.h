#ifndef MOWBITE_CAMERA_ERROR_TEXT_H
#define MOWBITE_CAMERA_ERROR_TEXT_H

#include <cstring>
#include <string>

namespace mowbite_camera {

// The words of an errno value; may be called from any thread.
inline std::string ErrorText(int error) {
  char words[128];
  return ::strerror_r(error, words, sizeof words);
}

}  // namespace mowbite_camera

#endif  // MOWBITE_CAMERA_ERROR_TEXT_H
