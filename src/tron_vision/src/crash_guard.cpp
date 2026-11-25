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

// Kernel functions whose crash has a known cause outside the cameras; the crash record names the site of the last
// oops as " at <function>" (rootfs script crash). Anything else, or no site, locks.
constexpr const char* kSitesOutsideTheCameras[] = {
    "bluesleep_read_proc_btwrite",  // a read of /proc/bluetooth/sleep/btwrite (25.11.)
};

bool OutsideTheCameras(const std::string& reason) {
  const auto at = reason.rfind(" at ");
  if (at == std::string::npos) return false;
  const std::string site = reason.substr(at + 4);
  for (const char* known : kSitesOutsideTheCameras) {
    if (site == known) return true;
  }
  return false;
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
  if (OutsideTheCameras(reason)) {
    Disarm(camera);
    return true;
  }
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

bool CrashGuard::Unlock(const std::string& camera, std::string& text) {
  const std::string locked = dir_ + "/locked-" + camera;
  if (!Exists(locked)) {
    text = "not locked";
    return false;
  }
  const std::string reason = FirstLine(locked);
  if (!RemoveDurably(locked)) {
    text = ErrnoText("remove " + locked);
    return false;
  }
  text = reason;
  return true;
}

bool UnlockRequests::Ask(std::chrono::milliseconds wait, std::string& text) {
  std::unique_lock<std::mutex> lock(mutex_);
  // One request at a time; a second one waits for the first.
  if (!answered_.wait_for(lock, wait, [this] { return !asked_; })) {
    text = "no answer";
    return false;
  }
  asked_ = true;
  done_ = false;
  if (!answered_.wait_for(lock, wait, [this] { return done_; })) {
    asked_ = false;
    answered_.notify_all();
    text = "no answer";
    return false;
  }
  asked_ = false;
  answered_.notify_all();
  text = text_;
  return ok_;
}

void UnlockRequests::Serve(const std::function<bool(std::string&)>& unlock) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!asked_ || done_) return;
  ok_ = unlock(text_);
  done_ = true;
  answered_.notify_all();
}

}  // namespace tron_vision
