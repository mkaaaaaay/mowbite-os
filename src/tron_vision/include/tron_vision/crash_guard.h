#ifndef TRON_VISION_CRASH_GUARD_H
#define TRON_VISION_CRASH_GUARD_H

#include <string>

namespace tron_vision {

// Keeps a camera from streaming again after a kernel crash during its stream. Before STREAMON a marker with the boot
// id goes to <dir>/stream-<camera>, after STREAMOFF it goes away. A marker of an earlier boot together with the crash
// flag of the system turns into <dir>/locked-<camera>, which holds until someone deletes it.
class CrashGuard {
 public:
  CrashGuard(std::string dir, std::string crash_flag, std::string boot_id_file);

  // False while the camera is locked; `detail` then says why.
  bool MayStream(const std::string& camera, std::string& detail);
  // Writes the marker durably; false if that fails, then the camera must not stream.
  bool Arm(const std::string& camera, std::string& error);
  void Disarm(const std::string& camera);

 private:
  std::string dir_;
  std::string crash_flag_;
  std::string boot_id_file_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_CRASH_GUARD_H
