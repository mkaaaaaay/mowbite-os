#include "tron_vision/text_file.h"

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>

#include "tron_vision/errno_text.h"

namespace tron_vision {

namespace {

bool SyncDirOf(const std::string& path) {
  const size_t slash = path.rfind('/');
  const std::string dir = slash == std::string::npos ? "." : path.substr(0, slash + 1);
  const int fd = open(dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) return false;
  const bool ok = fsync(fd) == 0;
  close(fd);
  return ok;
}

}  // namespace

std::string FirstLine(const std::string& path) {
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);
  return line;
}

bool WriteDurably(const std::string& path, const std::string& text, std::string& error) {
  const std::string tmp = path + ".new";
  const int fd = open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    error = ErrnoText("create " + tmp);
    return false;
  }
  const bool written = write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size()) && fsync(fd) == 0;
  close(fd);
  if (!written || std::rename(tmp.c_str(), path.c_str()) != 0 || !SyncDirOf(path)) {
    error = ErrnoText("write " + path);
    std::remove(tmp.c_str());
    return false;
  }
  return true;
}

bool RemoveDurably(const std::string& path) {
  if (std::remove(path.c_str()) != 0) return false;
  SyncDirOf(path);
  return true;
}

}  // namespace tron_vision
