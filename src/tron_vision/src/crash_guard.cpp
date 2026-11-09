#include "tron_vision/crash_guard.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <utility>

#include "tron_vision/text_file.h"

namespace tron_vision {

namespace {

std::string SysError(const std::string& what) {
  char buf[128];
  return what + ": " + strerror_r(errno, buf, sizeof(buf));
}

bool Exists(const std::string& path) {
  struct stat st {};
  return stat(path.c_str(), &st) == 0;
}

bool SyncDir(const std::string& dir) {
  const int fd = open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) return false;
  const bool ok = fsync(fd) == 0;
  close(fd);
  return ok;
}

// Writes `text` to `path` through a temporary file, synced before and after the rename.
bool WriteDurably(const std::string& dir, const std::string& path, const std::string& text, std::string& error) {
  const std::string tmp = path + ".new";
  const int fd = open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    error = SysError("create " + tmp);
    return false;
  }
  const bool written = write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size()) && fsync(fd) == 0;
  close(fd);
  if (!written || std::rename(tmp.c_str(), path.c_str()) != 0 || !SyncDir(dir)) {
    error = SysError("write " + path);
    std::remove(tmp.c_str());
    return false;
  }
  return true;
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
  if (!WriteDurably(dir_, locked, reason + "\n", error)) {
    detail = "locked, " + error;
    return false;
  }
  Disarm(camera);
  detail = "locked after a kernel crash during the stream: " + reason;
  return false;
}

bool CrashGuard::Arm(const std::string& camera, std::string& error) {
  if (mkdir(dir_.c_str(), 0755) != 0 && errno != EEXIST) {
    error = SysError("create " + dir_);
    return false;
  }
  const std::string boot = FirstLine(boot_id_file_);
  if (boot.empty()) {
    error = "no boot id in " + boot_id_file_;
    return false;
  }
  return WriteDurably(dir_, dir_ + "/stream-" + camera, boot + "\n", error);
}

void CrashGuard::Disarm(const std::string& camera) {
  if (std::remove((dir_ + "/stream-" + camera).c_str()) == 0) SyncDir(dir_);
}

}  // namespace tron_vision
