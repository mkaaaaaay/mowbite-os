#include "tron_mcu/stop_clear_mark.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace tron {

namespace {

std::string Errno(const char* what) {
  return std::string(what) + ": " + std::strerror(errno);  // NOLINT(concurrency-mt-unsafe): only the I/O thread
}

bool SyncDirectory(const std::string& path, std::string& error) {
  const size_t slash = path.rfind('/');
  const std::string dir = slash == std::string::npos ? "." : slash == 0 ? "/" : path.substr(0, slash);
  const int fd = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) {
    error = Errno("open directory");
    return false;
  }
  const bool ok = ::fsync(fd) == 0;
  if (!ok) error = Errno("fsync directory");
  ::close(fd);
  return ok;
}

}  // namespace

StopClearMark::StopClearMark(std::string path) : path_(std::move(path)) {
}

StopClearMark::State StopClearMark::Check(std::string& error) const {
  struct stat info {};
  if (::lstat(path_.c_str(), &info) == 0) return State::kPresent;
  if (errno == ENOENT) return State::kAbsent;
  error = Errno("lstat");
  return State::kUnknown;
}

bool StopClearMark::Write(std::string& error) const {
  const int fd = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    error = Errno("open");
    return false;
  }
  static const char kText[] = "STOP cleared on the MCU during an open emergency\n";
  const bool written = ::write(fd, kText, sizeof(kText) - 1) == static_cast<ssize_t>(sizeof(kText) - 1);
  if (!written) error = Errno("write");
  const bool synced = written && ::fsync(fd) == 0;
  if (written && !synced) error = Errno("fsync");
  const bool closed = ::close(fd) == 0;
  if (synced && !closed) error = Errno("close");
  return synced && closed && SyncDirectory(path_, error);
}

bool StopClearMark::Remove(std::string& error) const {
  if (::unlink(path_.c_str()) != 0) {
    if (errno == ENOENT) return true;
    error = Errno("unlink");
    return false;
  }
  return SyncDirectory(path_, error);
}

}  // namespace tron
