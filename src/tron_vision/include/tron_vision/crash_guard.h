#ifndef TRON_VISION_CRASH_GUARD_H
#define TRON_VISION_CRASH_GUARD_H

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>

namespace tron_vision {

// Keeps a camera from streaming again after a kernel crash during its stream. Before STREAMON a marker with the boot
// id goes to <dir>/stream-<camera>, after STREAMOFF it goes away. A marker of an earlier boot together with the crash
// flag of the system turns into <dir>/locked-<camera>, which holds until someone deletes it; only a crash record that
// names a site known to lie outside the cameras (crash_guard.cpp) leaves the camera free.
class CrashGuard {
 public:
  CrashGuard(std::string dir, std::string crash_flag, std::string boot_id_file);

  // False while the camera is locked; `detail` then says why.
  bool MayStream(const std::string& camera, std::string& detail);
  // Writes the marker durably; false if that fails, then the camera must not stream.
  bool Arm(const std::string& camera, std::string& error);
  void Disarm(const std::string& camera);
  // Deletes <dir>/locked-<camera> durably. True with the reason it held; false with "not locked" or the error.
  bool Unlock(const std::string& camera, std::string& text);

 private:
  std::string dir_;
  std::string crash_flag_;
  std::string boot_id_file_;
};

// Hands an unlock request of a service thread to the loop thread of the camera and its answer back.
class UnlockRequests {
 public:
  // Service thread: false with "no answer" if the loop does not serve the request within `wait`.
  bool Ask(std::chrono::milliseconds wait, std::string& text);
  // Loop thread: answers a waiting request with `unlock`.
  void Serve(const std::function<bool(std::string&)>& unlock);

 private:
  std::mutex mutex_;
  std::condition_variable answered_;
  bool asked_ = false;
  bool done_ = false;
  bool ok_ = false;
  std::string text_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_CRASH_GUARD_H
