#include "tron_vision/crash_guard.h"

#include <sys/stat.h>

#include <cerrno>
#include <utility>

#include "tron_vision/errno_text.h"
#include "tron_vision/text_file.h"

namespace tron_vision {

namespace {

bool Exists(const std::string& path) {
  struct stat st {};
  return stat(path.c_str(), &st) == 0;
}

}  // namespace

CrashGuard::CrashGuard(std::string dir, std::string crash_flag, std::string boot_id_file)
    : dir_(std::move(dir)), crash_flag_(std::move(crash_flag)), boot_id_file_(std::move(boot_id_file)) {}

bool CrashGuard::MayStream(const std::string& camera, std::string& detail) {
  const std::string locked = dir_ + "/locked-" + camera;
  const std::string marker = dir_ + "/stream-" + camera;
  if (Exists(locked)) {
    detail = "locked after a kernel crash during the stream: " + FirstLine(locked);
    return false;
  }
  if (!Exists(marker)) return true;
  const std::string marker_boot = FirstLine(marker);
  // Same boot: only this process ended during the stream, the kernel lived on.
  if (marker_boot == FirstLine(boot_id_file_) || !Exists(crash_flag_)) {
    Disarm(camera);
    return true;
  }
  const std::string reason = FirstLine(crash_flag_);
  std::string error;
  if (!WriteDurably(locked, reason + "\n", error)) {
    detail = "locked, " + error;
    return false;
  }
  Disarm(camera);
  detail = "locked after a kernel crash during the stream: " + reason;
  return false;
}

bool CrashGuard::Arm(const std::string& camera, std::string& error) {
  if (mkdir(dir_.c_str(), 0755) != 0 && errno != EEXIST) {
    error = ErrnoText("create " + dir_);
    return false;
  }
  const std::string boot = FirstLine(boot_id_file_);
  if (boot.empty()) {
    error = "no boot id in " + boot_id_file_;
    return false;
  }
  return WriteDurably(dir_ + "/stream-" + camera, boot + "\n", error);
}

void CrashGuard::Disarm(const std::string& camera) {
  RemoveDurably(dir_ + "/stream-" + camera);
}

}  // namespace tron_vision
